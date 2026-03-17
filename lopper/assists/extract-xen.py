#/*
# * Copyright (c) 2022 AMD Inc. All rights reserved.
# *
# * Author:
# *       Bruce Ashfield <bruce.ashfield@amd.com>
# *
# * SPDX-License-Identifier: BSD-3-Clause
# */

import sys
import types
import os
import getopt
import re
import copy
import logging
from collections import OrderedDict
from pathlib import Path
from pathlib import PurePath
from lopper import Lopper
from lopper import LopperFmt
from lopper.tree import LopperAction
from lopper.tree import LopperTree
from lopper.tree import LopperNode
from lopper.tree import LopperProp
import lopper
import lopper.log

lopper.log._init(__name__)

try:
    import yaml
except Exception:
    yaml = None

IMAGEBUILDER_DEFAULTS = OrderedDict([
    ("memory_start", "0x0"),
    ("memory_end", "0x80000000"),
    ("device_tree", "mpsoc.dtb"),
    ("xen", "xen"),
    ("xen_cmd", "console=dtuart dtuart=serial0 dom0_mem=1G dom0_max_vcpus=1 bootscrub=0 vwfi=native sched=null"),
    ("dom0_kernel", "Image-dom0"),
    ("dom0_cmd", "console=hvc0 earlycon=xen earlyprintk=xen clk_ignore_unused"),
    ("dom0_ramdisk", "dom0-ramdisk.cpio"),
    ("uboot_source", "boot.source"),
    ("uboot_script", "boot.scr"),
    ("domu_kernel", "Image-domU"),
    ("domu_ramdisk", "domU-ramdisk.cpio"),
    ("domu_mem", 512),
    ("domu_vcpus", 1),
    ("output", "config"),
])

IMAGEBUILDER_GLOBAL_FIELD_MAP = {
    "memory_start": ["memory-start", "memory_start", "memorystart"],
    "memory_end": ["memory-end", "memory_end", "memoryend"],
    "device_tree": ["device-tree", "device_tree", "devicetree"],
    "xen": ["xen"],
    "xen_cmd": ["xen-cmd", "xen_cmd"],
    "dom0_kernel": ["dom0-kernel", "dom0_kernel"],
    "dom0_cmd": ["dom0-cmd", "dom0_cmd"],
    "dom0_ramdisk": ["dom0-ramdisk", "dom0_ramdisk"],
    "uboot_source": ["uboot-source", "uboot_source"],
    "uboot_script": ["uboot-script", "uboot_script"],
}

IMAGEBUILDER_DOMAIN_FIELD_MAP = {
    "role": ["role", "domain-role", "domain_role"],
    "kernel": ["kernel", "kernel-image", "kernel_image"],
    "ramdisk": ["ramdisk", "initrd"],
    "passthrough_dtb": ["passthrough-dtb", "passthrough_dtb"],
    "memory_mb": ["memory-mb", "memory_mb"],
    "vcpus": ["vcpus", "vcpus-count", "vcpu-count"],
    "cmdline": ["cmdline", "bootargs", "boot-args"],
}

def is_compat( node, compat_string_to_test ):
    if re.search( "module,extract-xen", compat_string_to_test):
        return extract_xen
    if re.search( "xen,domain-v1", compat_string_to_test):
        return extract_xen
    return ""

def usage():
    print( """
   Usage: extract-xen -t <target node> [OPTION]

      -t       target node (full path)
      -p       permissive matching on target node (regex)
      -v       enable verbose debug/processing
      -o       output file for extracted device tree

   Usage: extract-xen imagebuilder -t <xen-domain> [OPTION]

      -t       target Xen domain node name under /domains

      --output <file>         output ImageBuilder config filename
      --output-dir <dir>      output directory for the generated config
      --dom0 <name>           explicit Dom0 domain name
      --memory-start <addr>   override MEMORY_START
      --memory-end <addr>     override MEMORY_END
      --device-tree <path>    override DEVICE_TREE
      --xen <path>            override XEN
      --xen-cmd <args>        override XEN_CMD
      --dom0-kernel <path>    override DOM0_KERNEL
      --dom0-cmd <args>       override DOM0_CMD
      --dom0-ramdisk <path>   override DOM0_RAMDISK
      --uboot-source <path>   override UBOOT_SOURCE
      --uboot-script <path>   override UBOOT_SCRIPT

    """)


def _coerce_string(value):
    if value is None:
        return None
    if isinstance(value, str):
        return value
    return str(value)


def _coerce_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return False


def _parse_int(value):
    if value is None or value == "":
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return int(value)
    value = str(value).strip()
    try:
        return int(value, 0)
    except Exception:
        return None


def _parse_size_to_bytes(value):
    if value is None or value == "":
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return int(value)

    sval = str(value).strip()
    size_match = re.fullmatch(r"(?i)\s*(0x[0-9a-f]+|\d+)\s*([kmgt]?)b?\s*", sval)
    if not size_match:
        return None

    base = int(size_match.group(1), 0)
    unit = size_match.group(2).upper()
    shift_map = {
        "": 0,
        "K": 10,
        "M": 20,
        "G": 30,
        "T": 40,
    }
    return base << shift_map[unit]


def _bytes_to_mb(value):
    if value is None:
        return None
    return max(1, int((value + (1024 * 1024) - 1) / (1024 * 1024)))


def _format_hex(value):
    parsed = _parse_int(value)
    if parsed is None:
        return value
    return f"0x{parsed:x}"


def _warning(message, warnings=None):
    if warnings is not None:
        warnings.append(message)
    lopper.log._warning(message)


def _load_domains_yaml(yaml_path):
    if yaml is None:
        raise RuntimeError("python yaml support is required for extract-xen imagebuilder mode")

    yaml_file = Path(yaml_path)
    if not yaml_file.exists():
        raise FileNotFoundError(f"domains yaml {yaml_path} not found")

    with open(yaml_file, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if not isinstance(data, dict):
        raise ValueError("domains yaml root must be a mapping")

    return data


def _iter_domains(doc):
    domains = doc.get("domains", {})
    if not isinstance(domains, dict):
        return []
    return list(domains.items())


def _node_propval(node, name):
    try:
        value = node.propval(name)
    except Exception:
        return None

    if value == ['']:
        return None
    return value


def _node_compatible_strings(node):
    compat = _node_propval(node, "compatible")
    if compat is None:
        return []
    if isinstance(compat, list):
        return [_coerce_string(v) for v in compat if v is not None]
    return [_coerce_string(compat)]


def _node_has_compat(node, pattern):
    for compat in _node_compatible_strings(node):
        if compat and re.search(pattern, compat):
            return True
    return False


def _node_first_string(node, names):
    for name in names:
        value = _node_propval(node, name)
        if value is None:
            continue
        if isinstance(value, list) and value:
            return _coerce_string(value[0])
        return _coerce_string(value)
    return None


def _lookup_mapping_value(mapping, keys):
    if not isinstance(mapping, dict):
        return None
    for key in keys:
        if key in mapping:
            return mapping[key]
    return None


def _lookup_imagebuilder_field(mapping, field_map, field_name):
    return _lookup_mapping_value(mapping, field_map[field_name])


def _find_xen_domain(doc, explicit_name=None):
    domains = _iter_domains(doc)
    if explicit_name:
        for name, domain in domains:
            if name == explicit_name:
                return name, domain
        raise ValueError(f"xen domain {explicit_name} not found in domains yaml")

    for name, domain in domains:
        if not isinstance(domain, dict):
            continue
        if str(domain.get("os,type", "")).lower() == "xen":
            return name, domain
    for name, domain in domains:
        if not isinstance(domain, dict):
            continue
        if name.lower() == "xen":
            return name, domain
    for name, domain in domains:
        if not isinstance(domain, dict):
            continue
        if isinstance(domain.get("domains"), dict):
            return name, domain

    raise ValueError("unable to identify a Xen domain container in domains yaml")


def _child_domains(xen_domain_name, xen_domain, doc):
    if isinstance(xen_domain, dict) and isinstance(xen_domain.get("domains"), dict):
        return list(xen_domain["domains"].items())

    children = []
    for name, domain in _iter_domains(doc):
        if name == xen_domain_name:
            continue
        if isinstance(domain, dict):
            children.append((name, domain))
    return children


def _count_domain_vcpus(domain):
    cpus = domain.get("cpus", [])
    if not isinstance(cpus, list):
        return None

    total = 0
    for cpu_entry in cpus:
        if not isinstance(cpu_entry, dict) or not cpu_entry:
            continue
        cpumask = cpu_entry.get("cpumask")
        parsed_mask = _parse_int(cpumask)
        if parsed_mask is None:
            total += 1
        else:
            total += bin(parsed_mask).count("1")
    return total or None


def _domain_memory_bounds(domain):
    memory = domain.get("memory", [])
    if not isinstance(memory, list):
        return None, None

    starts = []
    ends = []
    for entry in memory:
        if not isinstance(entry, dict) or not entry:
            continue
        start = _parse_int(entry.get("start"))
        size = _parse_size_to_bytes(entry.get("size"))
        if start is None or size is None:
            continue
        starts.append(start)
        ends.append(start + size)

    if not starts:
        return None, None

    return min(starts), max(ends)


def _domain_memory_mb(domain):
    start, end = _domain_memory_bounds(domain)
    if start is None or end is None:
        return None
    return _bytes_to_mb(end - start)


def _select_dom0(child_domains, explicit_name=None):
    if explicit_name:
        for name, domain in child_domains:
            if name == explicit_name:
                return name, domain
        raise ValueError(f"dom0 domain {explicit_name} not found in domains yaml")

    for name, domain in child_domains:
        ib = domain.get("imagebuilder", {})
        role = _lookup_imagebuilder_field(ib, IMAGEBUILDER_DOMAIN_FIELD_MAP, "role")
        if isinstance(role, str) and role.lower() == "dom0":
            return name, domain
        if _coerce_bool(domain.get("xen,dom0")):
            return name, domain
        if str(domain.get("role", "")).lower() == "dom0":
            return name, domain
    for name, domain in child_domains:
        if "dom0" in name.lower():
            return name, domain
    for name, domain in child_domains:
        if str(domain.get("os,type", "")).lower() == "linux":
            return name, domain
    if child_domains:
        return child_domains[0]
    return None, None


def _default_domu_path(domain_name, filename):
    return f"{domain_name}/{filename}"


def _memory_bounds_from_cells(cells):
    if not isinstance(cells, list) or not cells:
        return None, None
    if len(cells) % 4 != 0:
        return None, None

    starts = []
    ends = []
    for idx in range(0, len(cells), 4):
        start = ((int(cells[idx]) & 0xFFFFFFFF) << 32) | (int(cells[idx + 1]) & 0xFFFFFFFF)
        size = ((int(cells[idx + 2]) & 0xFFFFFFFF) << 32) | (int(cells[idx + 3]) & 0xFFFFFFFF)
        starts.append(start)
        ends.append(start + size)

    if not starts:
        return None, None
    return min(starts), max(ends)


def _domain_tree_memory_bounds(domain_node):
    memory = _node_propval(domain_node, "memory")
    if memory is not None:
        start, end = _memory_bounds_from_cells(memory)
        if start is not None:
            return start, end

    sram = _node_propval(domain_node, "sram")
    if sram is not None:
        start, end = _memory_bounds_from_cells(sram)
        if start is not None:
            return start, end

    return None, None


def _domain_tree_memory_mb(domain_node):
    start, end = _domain_tree_memory_bounds(domain_node)
    if start is None or end is None:
        return None
    return _bytes_to_mb(end - start)


def _domain_tree_vcpus(domain_node):
    cpus = _node_propval(domain_node, "cpus")
    if not isinstance(cpus, list) or not cpus:
        return None

    if len(cpus) % 3 == 0:
        masks = [cpus[idx + 1] for idx in range(0, len(cpus), 3)]
        return sum(bin(int(mask)).count("1") for mask in masks)
    if len(cpus) % 2 == 0:
        masks = [cpus[idx + 1] for idx in range(0, len(cpus), 2)]
        return sum(bin(int(mask)).count("1") for mask in masks)

    return None


def _tree_child_domains(domains_node):
    return list(domains_node.subnodes(children_only=True))


def _tree_child_domains_for_xen_domain(xen_domain_node):
    domains_container = None
    try:
        domains_container = xen_domain_node["domains"]
    except Exception:
        domains_container = None

    if domains_container:
        return [n for n in domains_container.subnodes(children_only=True) if _node_has_compat(n, r"domain-v1")]

    return [n for n in xen_domain_node.subnodes(children_only=True) if _node_has_compat(n, r"domain-v1")]


def _find_tree_xen_domain(tree, domain_name):
    try:
        domain_node = tree[f"/domains/{domain_name}"]
    except Exception:
        raise ValueError(f"imagebuilder: domain {domain_name} not found under /domains")

    if not _node_has_compat(domain_node, r"xen,domain-v1"):
        raise ValueError(f"imagebuilder: domain {domain_name} is not a Xen domain (compatible must include xen,domain-v1)")

    return domain_node


def _select_dom0_tree(child_nodes, explicit_name=None):
    if explicit_name:
        for node in child_nodes:
            if node.name == explicit_name:
                return node
        raise ValueError(f"dom0 domain {explicit_name} not found in /domains")

    for node in child_nodes:
        role = _node_first_string(node, ["imagebuilder,role", "imagebuilder-role", "role"])
        if role and role.lower() == "dom0":
            return node
    for node in child_nodes:
        if _coerce_bool(_node_first_string(node, ["xen,dom0", "xen-dom0"])):
            return node
    for node in child_nodes:
        os_type = _node_first_string(node, ["os,type", "os-type"])
        if os_type and os_type.lower() == "linux":
            return node
    for node in child_nodes:
        lowered = node.name.lower()
        if "dom0" in lowered or "linux" in lowered:
            return node
    return child_nodes[0] if child_nodes else None


def _collect_tree_global_config(xen_domain_node, dom0_node, cli_overrides, warnings):
    config = OrderedDict()
    for field_name, default_value in IMAGEBUILDER_DEFAULTS.items():
        if field_name in ("domu_kernel", "domu_ramdisk", "domu_mem", "domu_vcpus", "output"):
            continue

        value = cli_overrides.get(field_name)
        if value is not None:
            config[field_name] = value
            continue

        tree_prop_names = {
            "memory_start": ["imagebuilder,memory-start", "imagebuilder-memory-start", "memory-start"],
            "memory_end": ["imagebuilder,memory-end", "imagebuilder-memory-end", "memory-end"],
            "device_tree": ["imagebuilder,device-tree", "imagebuilder-device-tree", "device-tree"],
            "xen": ["imagebuilder,xen", "imagebuilder-xen", "xen"],
            "xen_cmd": ["imagebuilder,xen-cmd", "imagebuilder-xen-cmd", "xen-cmd"],
            "dom0_kernel": ["imagebuilder,dom0-kernel", "imagebuilder-dom0-kernel", "dom0-kernel"],
            "dom0_cmd": ["imagebuilder,dom0-cmd", "imagebuilder-dom0-cmd", "dom0-cmd"],
            "dom0_ramdisk": ["imagebuilder,dom0-ramdisk", "imagebuilder-dom0-ramdisk", "dom0-ramdisk"],
            "uboot_source": ["imagebuilder,uboot-source", "imagebuilder-uboot-source", "uboot-source"],
            "uboot_script": ["imagebuilder,uboot-script", "imagebuilder-uboot-script", "uboot-script"],
        }[field_name]

        value = _node_first_string(xen_domain_node, tree_prop_names)
        if value is None and dom0_node is not None:
            dom0_prop_names = {
                "dom0_kernel": ["imagebuilder,kernel", "imagebuilder-kernel", "kernel"],
                "dom0_cmd": ["imagebuilder,cmdline", "imagebuilder-cmdline", "cmdline", "bootargs"],
                "dom0_ramdisk": ["imagebuilder,ramdisk", "imagebuilder-ramdisk", "ramdisk"],
            }
            if field_name in dom0_prop_names:
                value = _node_first_string(dom0_node, dom0_prop_names[field_name])

        if value is None and field_name in ("memory_start", "memory_end") and dom0_node is not None:
            start, end = _domain_tree_memory_bounds(xen_domain_node)
            if field_name == "memory_start" and start is not None:
                value = _format_hex(start)
            elif field_name == "memory_end" and end is not None:
                value = _format_hex(end)

        if value is None:
            value = default_value
            _warning(f"imagebuilder: {field_name} not provided, using default '{value}'", warnings)

        if field_name in ("memory_start", "memory_end"):
            value = _format_hex(value)

        config[field_name] = value

    return config


def _collect_tree_domain_config(domain_node, role, index, warnings):
    domain_name = domain_node.name
    config = {
        "name": domain_name,
        "role": role,
        "index": index,
    }

    if _node_first_string(domain_node, ["imagebuilder,rootfs", "imagebuilder-rootfs", "rootfs"]) and \
       not _node_first_string(domain_node, ["imagebuilder,ramdisk", "imagebuilder-ramdisk", "ramdisk"]):
        raise ValueError(f"imagebuilder: domain {domain_name} uses unsupported rootfs-only input")

    kernel = _node_first_string(domain_node, ["imagebuilder,kernel", "imagebuilder-kernel", "kernel"])
    if kernel is None:
        kernel = IMAGEBUILDER_DEFAULTS["dom0_kernel"] if role == "dom0" else _default_domu_path(domain_name, IMAGEBUILDER_DEFAULTS["domu_kernel"])
        _warning(f"imagebuilder: {domain_name} kernel not provided, using default '{kernel}'", warnings)
    config["kernel"] = kernel

    ramdisk = _node_first_string(domain_node, ["imagebuilder,ramdisk", "imagebuilder-ramdisk", "ramdisk"])
    if ramdisk is None:
        ramdisk = IMAGEBUILDER_DEFAULTS["dom0_ramdisk"] if role == "dom0" else _default_domu_path(domain_name, IMAGEBUILDER_DEFAULTS["domu_ramdisk"])
        _warning(f"imagebuilder: {domain_name} ramdisk not provided, using default '{ramdisk}'", warnings)
    config["ramdisk"] = ramdisk

    config["cmdline"] = _node_first_string(domain_node, ["imagebuilder,cmdline", "imagebuilder-cmdline", "cmdline", "bootargs"])

    passthrough = _node_first_string(domain_node, ["imagebuilder,passthrough-dtb", "imagebuilder-passthrough-dtb", "passthrough-dtb"])
    config["passthrough_dtb"] = passthrough

    memory_mb = _parse_int(_node_first_string(domain_node, ["imagebuilder,memory-mb", "imagebuilder-memory-mb", "memory-mb"]))
    if memory_mb is None:
        memory_mb = _domain_tree_memory_mb(domain_node)
    if memory_mb is None and role != "dom0":
        memory_mb = IMAGEBUILDER_DEFAULTS["domu_mem"]
        _warning(f"imagebuilder: {domain_name} memory not provided, using default '{memory_mb}' MB", warnings)
    config["memory_mb"] = memory_mb

    vcpus = _parse_int(_node_first_string(domain_node, ["imagebuilder,vcpus", "imagebuilder-vcpus", "vcpus"]))
    if vcpus is None:
        vcpus = _domain_tree_vcpus(domain_node)
    if vcpus is None and role != "dom0":
        vcpus = IMAGEBUILDER_DEFAULTS["domu_vcpus"]
        _warning(f"imagebuilder: {domain_name} vcpus not provided, using default '{vcpus}'", warnings)
    config["vcpus"] = vcpus

    return config


def generate_imagebuilder_config_from_tree(tree, xen_domain_name, cli_overrides=None):
    cli_overrides = cli_overrides or {}
    warnings = []

    xen_domain_node = _find_tree_xen_domain(tree, xen_domain_name)
    child_nodes = _tree_child_domains_for_xen_domain(xen_domain_node)
    if not child_nodes:
        raise ValueError(f"imagebuilder: Xen domain {xen_domain_name} has no child domains")

    dom0_node = _select_dom0_tree(child_nodes, cli_overrides.get("dom0"))
    if dom0_node is None:
        raise ValueError(f"imagebuilder: no Dom0 domain found under Xen domain {xen_domain_name}")

    global_config = _collect_tree_global_config(xen_domain_node, dom0_node, cli_overrides, warnings)
    dom0_config = _collect_tree_domain_config(dom0_node, "dom0", -1, warnings)

    if cli_overrides.get("dom0_kernel") is not None:
        dom0_config["kernel"] = cli_overrides["dom0_kernel"]
    elif global_config.get("dom0_kernel"):
        dom0_config["kernel"] = global_config["dom0_kernel"]

    if cli_overrides.get("dom0_ramdisk") is not None:
        dom0_config["ramdisk"] = cli_overrides["dom0_ramdisk"]
    elif global_config.get("dom0_ramdisk"):
        dom0_config["ramdisk"] = global_config["dom0_ramdisk"]

    if cli_overrides.get("dom0_cmd") is not None:
        global_config["dom0_cmd"] = cli_overrides["dom0_cmd"]
    elif dom0_config.get("cmdline"):
        global_config["dom0_cmd"] = dom0_config["cmdline"]

    domu_configs = []
    next_index = 0
    for node in child_nodes:
        if node.abs_path == dom0_node.abs_path:
            continue
        domu_configs.append(_collect_tree_domain_config(node, "domU", next_index, warnings))
        next_index += 1

    content = _render_imagebuilder_config(global_config, dom0_config, domu_configs)
    return content, warnings


def _collect_global_config(doc, xen_domain, dom0_domain, cli_overrides, warnings):
    global_imagebuilder = doc.get("imagebuilder", {}) if isinstance(doc.get("imagebuilder"), dict) else {}
    xen_imagebuilder = xen_domain.get("imagebuilder", {}) if isinstance(xen_domain.get("imagebuilder"), dict) else {}
    dom0_imagebuilder = dom0_domain.get("imagebuilder", {}) if isinstance(dom0_domain.get("imagebuilder"), dict) else {}

    config = OrderedDict()
    for field_name, default_value in IMAGEBUILDER_DEFAULTS.items():
        if field_name in ("domu_kernel", "domu_ramdisk", "domu_mem", "domu_vcpus", "output"):
            continue

        value = cli_overrides.get(field_name)
        if value is not None:
            config[field_name] = value
            continue

        value = _lookup_imagebuilder_field(xen_imagebuilder, IMAGEBUILDER_GLOBAL_FIELD_MAP, field_name)
        if value is None:
            value = _lookup_imagebuilder_field(global_imagebuilder, IMAGEBUILDER_GLOBAL_FIELD_MAP, field_name)
        if value is None:
            if field_name == "memory_start" or field_name == "memory_end":
                start, end = _domain_memory_bounds(xen_domain)
                if field_name == "memory_start" and start is not None:
                    value = _format_hex(start)
                elif field_name == "memory_end" and end is not None:
                    value = _format_hex(end)
            elif field_name == "dom0_kernel":
                value = _lookup_imagebuilder_field(dom0_imagebuilder, IMAGEBUILDER_DOMAIN_FIELD_MAP, "kernel")
            elif field_name == "dom0_cmd":
                value = _lookup_imagebuilder_field(dom0_imagebuilder, IMAGEBUILDER_DOMAIN_FIELD_MAP, "cmdline")
                if value is None:
                    value = _lookup_imagebuilder_field(global_imagebuilder, IMAGEBUILDER_DOMAIN_FIELD_MAP, "cmdline")
            elif field_name == "dom0_ramdisk":
                value = _lookup_imagebuilder_field(dom0_imagebuilder, IMAGEBUILDER_DOMAIN_FIELD_MAP, "ramdisk")

        if value is None:
            value = default_value
            _warning(f"imagebuilder: {field_name} not provided, using default '{value}'", warnings)

        if field_name in ("memory_start", "memory_end"):
            value = _format_hex(value)

        config[field_name] = value

    return config


def _collect_domain_config(domain_name, domain, role, index, warnings):
    imagebuilder = domain.get("imagebuilder", {}) if isinstance(domain.get("imagebuilder"), dict) else {}

    if imagebuilder.get("rootfs") and not imagebuilder.get("ramdisk"):
        raise ValueError(f"imagebuilder: domain {domain_name} uses unsupported rootfs-only input")

    config = {}
    config["name"] = domain_name
    config["role"] = role

    kernel = _lookup_imagebuilder_field(imagebuilder, IMAGEBUILDER_DOMAIN_FIELD_MAP, "kernel")
    if kernel is None:
        kernel = IMAGEBUILDER_DEFAULTS["dom0_kernel"] if role == "dom0" else _default_domu_path(domain_name, IMAGEBUILDER_DEFAULTS["domu_kernel"])
        _warning(f"imagebuilder: {domain_name} kernel not provided, using default '{kernel}'", warnings)
    config["kernel"] = kernel

    ramdisk = _lookup_imagebuilder_field(imagebuilder, IMAGEBUILDER_DOMAIN_FIELD_MAP, "ramdisk")
    if ramdisk is None:
        ramdisk = IMAGEBUILDER_DEFAULTS["dom0_ramdisk"] if role == "dom0" else _default_domu_path(domain_name, IMAGEBUILDER_DEFAULTS["domu_ramdisk"])
        _warning(f"imagebuilder: {domain_name} ramdisk not provided, using default '{ramdisk}'", warnings)
    config["ramdisk"] = ramdisk

    cmdline = _lookup_imagebuilder_field(imagebuilder, IMAGEBUILDER_DOMAIN_FIELD_MAP, "cmdline")
    config["cmdline"] = cmdline

    passthrough = _lookup_imagebuilder_field(imagebuilder, IMAGEBUILDER_DOMAIN_FIELD_MAP, "passthrough_dtb")
    if passthrough is not None and not isinstance(passthrough, str):
        raise ValueError(f"imagebuilder: domain {domain_name} passthrough-dtb must be a string path")
    config["passthrough_dtb"] = passthrough

    memory_mb = _lookup_imagebuilder_field(imagebuilder, IMAGEBUILDER_DOMAIN_FIELD_MAP, "memory_mb")
    memory_mb = _parse_int(memory_mb)
    if memory_mb is None:
        memory_mb = _domain_memory_mb(domain)
    if memory_mb is None and role != "dom0":
        memory_mb = IMAGEBUILDER_DEFAULTS["domu_mem"]
        _warning(f"imagebuilder: {domain_name} memory not provided, using default '{memory_mb}' MB", warnings)
    config["memory_mb"] = memory_mb

    vcpus = _lookup_imagebuilder_field(imagebuilder, IMAGEBUILDER_DOMAIN_FIELD_MAP, "vcpus")
    vcpus = _parse_int(vcpus)
    if vcpus is None:
        vcpus = _count_domain_vcpus(domain)
    if vcpus is None and role != "dom0":
        vcpus = IMAGEBUILDER_DEFAULTS["domu_vcpus"]
        _warning(f"imagebuilder: {domain_name} vcpus not provided, using default '{vcpus}'", warnings)
    config["vcpus"] = vcpus

    config["index"] = index
    return config


def _render_imagebuilder_config(global_config, dom0_config, domu_configs):
    lines = [
        f'MEMORY_START="{global_config["memory_start"]}"',
        f'MEMORY_END="{global_config["memory_end"]}"',
        "",
        f'DEVICE_TREE="{global_config["device_tree"]}"',
        f'XEN="{global_config["xen"]}"',
        f'XEN_CMD="{global_config["xen_cmd"]}"',
        f'DOM0_KERNEL="{dom0_config["kernel"]}"',
        f'DOM0_CMD="{global_config["dom0_cmd"]}"',
        f'DOM0_RAMDISK="{dom0_config["ramdisk"]}"',
        "",
        f'NUM_DOMUS={len(domu_configs)}',
    ]

    for domu in domu_configs:
        idx = domu["index"]
        lines.append(f'DOMU_KERNEL[{idx}]="{domu["kernel"]}"')
        lines.append(f'DOMU_RAMDISK[{idx}]="{domu["ramdisk"]}"')
        if domu["passthrough_dtb"]:
            lines.append(f'DOMU_PASSTHROUGH_DTB[{idx}]="{domu["passthrough_dtb"]}"')
        if domu["memory_mb"] is not None:
            lines.append(f'DOMU_MEM[{idx}]={domu["memory_mb"]}')
        if domu["vcpus"] is not None:
            lines.append(f'DOMU_VCPUS[{idx}]={domu["vcpus"]}')

    lines.extend([
        "",
        f'UBOOT_SOURCE="{global_config["uboot_source"]}"',
        f'UBOOT_SCRIPT="{global_config["uboot_script"]}"',
        "",
    ])

    return "\n".join(lines)


def generate_imagebuilder_config_from_yaml(yaml_path, cli_overrides=None):
    cli_overrides = cli_overrides or {}
    warnings = []
    doc = _load_domains_yaml(yaml_path)

    xen_domain_name, xen_domain = _find_xen_domain(doc, cli_overrides.get("xen_domain"))
    child_domains = _child_domains(xen_domain_name, xen_domain, doc)
    dom0_name, dom0_domain = _select_dom0(child_domains, cli_overrides.get("dom0"))
    if dom0_domain is None:
        raise ValueError("imagebuilder: no Dom0 domain found under Xen domain")

    dom0_config = _collect_domain_config(dom0_name, dom0_domain, "dom0", -1, warnings)
    global_config = _collect_global_config(doc, xen_domain, dom0_domain, cli_overrides, warnings)

    if dom0_config["kernel"] == IMAGEBUILDER_DEFAULTS["dom0_kernel"] and global_config.get("dom0_kernel"):
        dom0_config["kernel"] = global_config["dom0_kernel"]
    if dom0_config["ramdisk"] == IMAGEBUILDER_DEFAULTS["dom0_ramdisk"] and global_config.get("dom0_ramdisk"):
        dom0_config["ramdisk"] = global_config["dom0_ramdisk"]
    if cli_overrides.get("dom0_kernel") is not None:
        dom0_config["kernel"] = cli_overrides["dom0_kernel"]
    if cli_overrides.get("dom0_ramdisk") is not None:
        dom0_config["ramdisk"] = cli_overrides["dom0_ramdisk"]
    if cli_overrides.get("dom0_cmd") is not None:
        global_config["dom0_cmd"] = cli_overrides["dom0_cmd"]
    elif dom0_config.get("cmdline"):
        global_config["dom0_cmd"] = dom0_config["cmdline"]

    domu_configs = []
    next_index = 0
    for name, domain in child_domains:
        if name == dom0_name:
            continue
        role = "domU"
        domu_config = _collect_domain_config(name, domain, role, next_index, warnings)
        domu_configs.append(domu_config)
        next_index += 1

    content = _render_imagebuilder_config(global_config, dom0_config, domu_configs)
    return content, warnings


def _resolve_output_path(output_dir, output_name):
    output_dir = output_dir or "."
    output_name = output_name or IMAGEBUILDER_DEFAULTS["output"]

    output_path = Path(output_name)
    if output_path.is_absolute() or output_path.parent != Path("."):
        return output_path

    return Path(output_dir) / output_name


def _extract_xen_imagebuilder(sdt, args, verbose=0):
    getopt_parser = getattr(getopt, "gnu_getopt", getopt.getopt)
    opts, args2 = getopt_parser(
        args,
        "vt:o:",
        [
            "verbose",
            "output=",
            "output-dir=",
            "dom0=",
            "memory-start=",
            "memory-end=",
            "device-tree=",
            "xen=",
            "xen-cmd=",
            "dom0-kernel=",
            "dom0-cmd=",
            "dom0-ramdisk=",
            "uboot-source=",
            "uboot-script=",
        ],
    )

    cli_overrides = {
        "dom0": None,
        "memory_start": None,
        "memory_end": None,
        "device_tree": None,
        "xen": None,
        "xen_cmd": None,
        "dom0_kernel": None,
        "dom0_cmd": None,
        "dom0_ramdisk": None,
        "uboot_source": None,
        "uboot_script": None,
    }
    output_name = None
    output_dir = getattr(sdt, "outdir", ".") if sdt else "."
    xen_domain_name = None

    for o, a in opts:
        if o == "-t":
            xen_domain_name = a
        elif o in ("-o", "--output"):
            output_name = a
        elif o == "--output-dir":
            output_dir = a
        elif o == "--dom0":
            cli_overrides["dom0"] = a
        elif o == "--memory-start":
            cli_overrides["memory_start"] = a
        elif o == "--memory-end":
            cli_overrides["memory_end"] = a
        elif o == "--device-tree":
            cli_overrides["device_tree"] = a
        elif o == "--xen":
            cli_overrides["xen"] = a
        elif o == "--xen-cmd":
            cli_overrides["xen_cmd"] = a
        elif o == "--dom0-kernel":
            cli_overrides["dom0_kernel"] = a
        elif o == "--dom0-cmd":
            cli_overrides["dom0_cmd"] = a
        elif o == "--dom0-ramdisk":
            cli_overrides["dom0_ramdisk"] = a
        elif o == "--uboot-source":
            cli_overrides["uboot_source"] = a
        elif o == "--uboot-script":
            cli_overrides["uboot_script"] = a
        elif o in ("-v", "--verbose"):
            verbose += 1

    if args2:
        lopper.log._warning(f"imagebuilder: ignoring unexpected positional arguments: {args2}")

    if not xen_domain_name:
        usage()
        raise SystemExit(1)

    if not sdt or not getattr(sdt, "tree", None):
        raise ValueError("imagebuilder: current SDT tree is not available")
    content, _ = generate_imagebuilder_config_from_tree(sdt.tree, xen_domain_name, cli_overrides)

    output_path = _resolve_output_path(output_dir, output_name)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(content, encoding="utf-8")
    lopper.log._info(f"imagebuilder: wrote config to {output_path}")
    return True


def extract_xen( tgt_node, sdt, options ):
    try:
        verbose = options['verbose']
    except:
        verbose = 0

    try:
        args = options['args']
    except:
        args = []

    mode = None
    if args and not args[0].startswith("-"):
        mode = args[0]
        args = args[1:]

    if mode == "imagebuilder":
        return _extract_xen_imagebuilder(sdt, args, verbose)

    try:
        xen_tree = sdt.subtrees["extracted"]
    except:
        lopper.log._error( "no extracted tree detected, returning" )
        return False

    opts,args2 = getopt.getopt( args, "vpt:o:", [ "verbose", "permissive" ] )

    permissive=False
    target_node_name=None
    output=None
    for o,a in opts:
        # print( "o: %s a: %s" % (o,a))
        if o in ('-o'):
            output=a
        elif o in ('-t'):
            target_node_name=a
        elif o in ('-v', "--verbose"):
            verbose = verbose + 1
        elif o in ('-p', "--permissive"):
            permissive = True

    ## current TODO:
    ##  - create a zynqmp-firmware parent node to the clock-controller
    ##      - this is actually the parent node in the input device tree, but
    ##        not the only parent. How do we want to indicate that we only
    ##        should extract and pull in one level of parent ?? Maybe pull them
    ##        all in, and have this delete the one it doesn't need, since that is
    ##        xen specific ?
    ##  - add the xen,<> arguments to the serial@ff01000 node
    ##      xen,reg: generated from the reg property, with some hardcoded 2nd/3rd groupings
    ##      xen,path: path of the node we got the extracted target from in the sdt .. what is
    ##                the best way to pass this through ? maybe do a generic extract,path and
    ##                just update it to xen,path here [done (generic extract,path route)]
    ##  - change interrupt-parent = <0xfde8> in serial@ff0100 node, or if it isn't present
    ##    (i.e. ignored by extract) it must be added.
    ##  - remove the interrupt-controller node
    ##  - renamed "/extracted" to "/passthrough" [done]

    try:
        extracted_node = xen_tree["/extracted"]
    except:
        lopper.log._error( "no extracted node detected" )
        return False

    # rename the containing node from /extracted to /passthrough
    extracted_node.name = "passthrough"

    # copy sdt root compatibles into extracted node
    root_compat = sdt.tree["/"]["compatible"].value
    root_compat.append(extracted_node["compatible"].value)
    extracted_node["compatible"].value = root_compat

    # walk the nodes in the tree, and look for the property "extracted,path"
    # and update it to "xen,path" (when conditions are met)
    for n in xen_tree:
        try:
            p = n["extracted,path"]
            # if there's an iommu in the node, we convert to xen,path, otherwise
            # do nothing
            iommu = n["iommus"]
            # we'll have thrown an exception if the property wasn't there, so this
            # only runs in the sucess case
            p.name = "xen,path"
        except:
            # TODO: we may want to check for nodes that have "reg" and use that
            #       as a secondary trigger to convert to xen,path .. but that still
            #       may be too broad
            pass

        try:
            ip = n["interrupt-parent"]
            n["interrupt-parent"].value = "0xfde8"
            lopper.log._info( f"{n.name} interrupt parent found, updating" )

            # this is a known non-existent phandle, we need to inhibit
            # phandle resolution and just have the number used
            ip.phandle_resolution = False
            ip.resolve( strict = False )
        except:
            pass

    if target_node_name:
        nodes_to_delete = []
        for n in xen_tree:
            if n.name == target_node_name:
                # the target node may not have had a iommus property, but we do
                # always want it to have a xen,path property, so we force it here
                p = n["extracted,path"]
                p.name = "xen,path"

                # is there an iommu property ? if so, that tells us what to do about the
                # without iommu
                need_force_assign = False
                try:
                    iommus_prop = n["iommus"]

                    # remove the property and all the other nodes it may have brought in
                    refs = iommus_prop.resolve_phandles()
                    n - iommus_prop

                    for r in refs:
                        nodes_to_delete.append( r )
                except:
                    need_force_assign = True


                if need_force_assign:
                    np = LopperProp( "xen,force-assign-without-iommu" )
                    np.value = 1
                    n + np

                # reach into the SDT and add "xen,passthrough" to the device
                sdt_device_path = n["extracted,path"].value
                if sdt_device_path:
                    lopper.log._info( "updating sdt with passthrough property" )
                    x_pass = LopperProp( "xen,passthrough" )
                    x_pass.value = ""
                    sdt.tree[sdt_device_path] + x_pass

                # check for the reg property
                try:
                    reg = n["reg"]
                    lopper.log._info( f"reg found: {reg} copying and extending to xen,reg" )
                    # make a xen,reg from it
                    xen_reg = LopperProp( "xen,reg" )

                    # split reg.value into chunks (memory regions) of 4 items (2 for address, 2 for size)
                    reg_chunks  = [reg.value[x:x+4] for x in range(0, len(reg.value), 4)]

                    # xen,reg is an array of <phys_addr size guest_addr> and we always
                    # set guest_addr to phys_addr. Iterate over splitted memory regions
                    # and fill in xen,reg according to the format mentioned above
                    for i in range(len(reg_chunks)):
                        addr = [reg_chunks[i][0], reg_chunks[i][1]]
                        size = [reg_chunks[i][2], reg_chunks[i][3]]
                        xen_reg.value.extend(addr)
                        xen_reg.value.extend(size)
                        xen_reg.value.extend(addr)

                    # magic. these need to be generated in the future
                    # xen_reg.value.extend( [0x0, 0xff110000, 0x0, 0x1000, 0x0, 0xff110000] )
                    # xen_reg.value.extend( [0x0, 0xff120000, 0x0, 0x1000, 0x0, 0xff120000] )
                    # xen_reg.value.extend( [0x0, 0xff130000, 0x0, 0x1000, 0x0, 0xff130000] )
                    # xen_reg.value.extend( [0x0, 0xff140000, 0x0, 0x1000, 0x0, 0xff140000] )

                    n = n + xen_reg
                except Exception as e:
                    lopper.log._debug( f"{e}", level=lopper.log.TRACE2 )

        if nodes_to_delete:
            for n in nodes_to_delete:
                lopper.log._info( f"deleting node (referencing node was removed): {n.abs_path}" )
                xen_tree - n

    # resolve() isn't strictly required, but better to be safe
    xen_tree.strict = False
    xen_tree.resolve()

    if output:
        sdt.write( xen_tree, output, True, True )
    elif lopper.log._is_enabled(logging.INFO):
        xen_tree.output = None
        xen_tree.print( sys.stdout )

    return True
