#/*
# * Copyright (c) 2019,2020 Xilinx Inc. All rights reserved.
# *
# * Author:
# *       Bruce Ashfield <bruce.ashfield@xilinx.com>
# *
# * SPDX-License-Identifier: BSD-3-Clause
# */

import copy
import struct
import sys
import types
import unittest
import os
import getopt
import re
import subprocess
import shutil
from pathlib import Path
from pathlib import PurePath
from io import StringIO
import contextlib
import importlib
from lopper import Lopper
from lopper import LopperFmt
import lopper
from lopper_tree import *
from re import *

sys.path.append(os.path.dirname(__file__))
from openamp_xlnx_common import *

RPU_PATH = "/rpu@ff9a0000"

def get_r5_needed_symbols(carveout_list):
    rsc_mem_pa = -1
    shared_mem_size = -1
    for i in carveout_list:
        if "vdev0buffer" in i[0]:
            shared_mem_size = int(i[1][3],16)
        elif "elfload" in i[0] or "rproc" in i[0]:
            rsc_mem_pa =  int( i[1][1],16)+0x20000

    return [rsc_mem_pa, shared_mem_size]

# table relating ipi's to IPI_BASE_ADDR -> IPI_IRQ_VECT_ID and IPI_CHN_BITMASK
#versal_ipi_lookup_table = {
#  "0xff340000" : [63, 0x0000020,  ],
#  "0xff360000" : [0 , 0x0000008, ]
#}
# TODO : fix so its accurate for all avail ipi's
# update so that it is ipi -> the ipi irq vect id + chn bitmask that map TO  it
versal_ipi_lookup_table = {
  "0xff340000" : [0,   0x0000020, False ],
  "0xff360000" : [63 , 0x0000008, False ]
}

zynqmp_ipi_lookup_table = { "0xff310000" : [65, 0x1000000 ] , "0xff340000" : [0 , 0x100 ] }

def trim_ipis(sdt):
    unneeded_props = ["compatible", "xlnx,ipi-bitmask","interrupts", "xlnx,ipi-id", "xlnx,ipi-target-count", "xlnx,s-axi-highaddr", "xlnx,cpu-name", "xlnx,buffer-base", "xlnx,buffer-index", "xlnx,s-axi-baseaddr", "xlnx,int-id", "xlnx,bit-position"]

    amba_sub_nodes = sdt.tree['/amba'].subnodes()
    for node in amba_sub_nodes:
      node_compat = node.propval("compatible")
      if node_compat != [""]:
       if 'xlnx,zynqmp-ipi-mailbox' in node_compat:
         for i in unneeded_props:
           node[i].value = ""
         node.sync(sdt.FDT)

def parse_ipis_for_rpu(sdt, options):
    try:
        verbose = options['verbose']
    except:
        verbose = 0

    ipi_list = []
    unneeded_props = ["compatible", "xlnx,ipi-bitmask","interrupts", "xlnx,ipi-id", "xlnx,ipi-target-count", "xlnx,s-axi-highaddr", "xlnx,cpu-name", "xlnx,buffer-base", "xlnx,buffer-index", "xlnx,s-axi-baseaddr", "xlnx,int-id", "xlnx,bit-position"]
    amba_sub_nodes = sdt.tree['/amba'].subnodes()
    for node in amba_sub_nodes:
      node_compat = node.propval("compatible")
      if node_compat != [""]:
       if 'xlnx,zynqmp-ipi-mailbox' in node_compat:
         ipi_list.append(hex(int(node.propval("reg")[1])))
         for i in unneeded_props:
           node[i].value = ""
         node.sync(sdt.FDT)

    if verbose:
        print( "[INFO]: Dedicated IPIs for OpenAMP: %s" % ipi_list)

    return ipi_list

def is_compat( node, compat_string_to_test ):
    if re.search( "openamp,xlnx-rpu", compat_string_to_test):
        print("calling xlnx_openamp_rpu")
        return xlnx_openamp_rpu
    return ""

def setup_ipi_inputs(inputs, platform, ipi_list, options):
    try:
        verbose = options['verbose']
    except:
        verbose = 0
    ipi_details_list = None
    if platform == SOC_TYPE.VERSAL:
        ipi_details_list = versal_ipi_lookup_table
    elif platform == SOC_TYPE.ZYNQMP:
        ipi_details_list = zynqmp_ipi_lookup_table
    else:
        if verbose != 0:
            print ("[WARNING]: invalid device tree. no valid platform found")
            return

    master_role_used = False
    remote_role_used = False
    for preset_ipi in ipi_details_list:
      for index,value in enumerate(ipi_list):
        if str(value) in list(ipi_details_list.keys()):
          # from there append master,remote to inputs
          # use first available ipi for master
          if master_role_used == False:
            master_role_used = True
            current_role = "MASTER"
            other_role = "REMOTE"
          elif remote_role_used == False:
            remote_role_used = True
            current_role = "REMOTE"
            other_role = "MASTER"
          else:
            print("master and remote roles for ipi have been assigned for this channel")
            return inputs

          val = ipi_details_list[str(value)]
          inputs[current_role+"_IPI_BASE_ADDR"] = str(value)+"U"
          inputs[current_role+"_IPI_NAME"] = '\"'+value.replace("0x","")+".ps_ipi\""
          inputs[other_role+"_IRQ_VECT_ID"] = str(val[0])
          inputs[other_role+"_CHN_BITMASK"] = str(hex(val[1]))+"U"
          # update dict
          val[2] = True
          updated_entry = { str(value) : val }
          ipi_details_list.update(updated_entry)

    return inputs

def handle_rpmsg_userspace_case(tgt_node, sdt, options, memory_node, rpu_node, rsc_mem_pa, shared_mem_size, platform):
    if platform == SOC_TYPE.VERSAL:
        gic_node = sdt.tree["/amba_apu/interrupt-controller@f9000000"]
    if platform == SOC_TYPE.ZYNQMP:
        gic_node = sdt.tree["/amba-apu@0/interrupt-controller@f9010000"]
    openamp_shm_node = sdt.tree["/amba/shm@0"]
    openamp_shm_node.name = "shm@"+hex(rsc_mem_pa).replace("0x","")
    openamp_shm_node["reg"].value = [0x0 , rsc_mem_pa, 0x0, shared_mem_size]
    openamp_shm_node.sync ( sdt.FDT )
    for node in sdt.tree:
        if "ps_ipi" in node.abs_path:
            prop = LopperProp("interrupt-parent")
            prop.value = gic_node.phandle
            node + prop
            node.sync ( sdt.FDT )

def update_remoteproc_rpus(sdt, memory_node):
  remoteproc_node = sdt.tree[RPU_PATH]
  core_to_mboxes = {}

  mailbox_cntr_node = sdt.tree["/zynqmp_ipi1"]
  current_core_to_mbox = 0
  for index,node in enumerate(mailbox_cntr_node.subnodes()):
    # only 2 cores per rpu cluster
    if current_core_to_mbox > 1:
      break

    if "mailbox" in node.abs_path:
      node.resolve(sdt.FDT)
      node.sync(sdt.FDT)
      node.resolve_all_refs()
      sdt.tree.sync()
      sdt.tree.resolve()

      core_to_mboxes[str(current_core_to_mbox)] = node.phandle
      current_core_to_mbox += 1

  for i in list(core_to_mboxes.keys()):
    # for each rpu core in remoteproc kernel driver
    try:
      core_node = sdt.tree[RPU_PATH+"/r5_"+i]
    except:
      print(RPU_PATH+"/r5_"+i+" does not exist in sdt")
      return

    # 1. find corresponding mbox controller and update mboxes
    new_val = core_node["mboxes"].value
    new_val[0] = core_to_mboxes[i]
    new_val[2]  = core_to_mboxes[i]
    core_node["mboxes"].value = new_val

  sdt.tree.sync()
  sdt.tree.resolve()

def update_mbox_cntr_intr_parent(sdt):
  # find phandle of a72 gic for mailbox controller
  a72_gic_node = sdt.tree["/amba_apu/interrupt-controller@f9000000"]
  # set mailbox controller interrupt-parent to this phandle
  mailbox_cntr_node = sdt.tree["/zynqmp_ipi1"]
  mailbox_cntr_node["interrupt-parent"].value = a72_gic_node.phandle
  sdt.tree.sync()
  sdt.tree.resolve()

def update_pnode_ids_for_tcm(sdt, platform):
    pnode_id_table = {}
    tcm_path_prefex = ""
    if platform == SOC_TYPE.VERSAL:
        tcm_path_prefex = "/amba/psv_tcm_global@"
        pnode_id_table = {
          "ffe00000" : 0x1831800b,
          "ffe20000" : 0x1831800c,
          "ffe90000" : 0x1831800d,
          "ffeb0000" : 0x1831800e,
          "r5_0"     : 0x18110005,
          "r5_1"     : 0x18110006,
        }
    else:
        print("presently not populating pnode ids for platforms other than versal")
        return 1

    # update tcm pnode ids
    for i in list(pnode_id_table.keys()):
      if "ffe" in i:
        current_tcm = sdt.tree[tcm_path_prefex+i]
        prop = LopperProp("pnode-id")
        prop.value = pnode_id_table[i]
        current_tcm + prop
        current_tcm.sync ( sdt.FDT )
      
    # TODO: when only openamp domains are used, add r5 pnodeid info


def handle_rpmsg_kernelspace_case(tgt_node, sdt, options, memory_node, rpu_node, platform):
    try:
        verbose = options['verbose']
    except:
        verbose = 0

    if rpu_node == None:
        print( "not valid input systemDT for openamp rpmsg kernelspace case")
        return False

    update_mbox_cntr_intr_parent(sdt)
    update_remoteproc_rpus(sdt, memory_node)
    update_pnode_ids_for_tcm(sdt, platform)
    return memory_node

# 1 for master, 0 for slave
# for each openamp channel, return mapping of role to resource group
def determine_role(sdt, domain_node):
  print("in determine_role")
  include_prop = domain_node["include"]
  rsc_groups = []
  current_rsc_group = None
  if len(list(include_prop.value)) % 2 == 1:
    print("odd num elem in include list")
    return -1
  for index,value in enumerate(include_prop.value):
    if index % 2 == 0:
      current_rsc_group = sdt.tree.pnode(value)
    else:
      if value == 1: # only for openamp master
        if current_rsc_group == None:
          print("invalid phandle to rsc group")
          return -1
        rsc_groups.append(current_rsc_group)
      else:
        print("only do processing in host openamp channel domain ", value)
        return -1
  return rsc_groups

# in this case remote is rpu
# find node that is other end of openamp channel
def find_remote(sdt, domain_node, rsc_group_node):
  domains = sdt.tree["/domains"]
  # find other domain including the same resource group
  remote_domain = None
  for node in domains.subnodes():
    # look for other domains with include
    if node.propval("include") != [''] and node != domain_node:
      # if node includes same rsc group, then this is remote
      for i in node.propval("include"):
        included_node = sdt.tree.pnode(i)
        if included_node != None and included_node == rsc_group_node:
           return node

  return -1

# tests for a bit that is set, going fro 31 -> 0 from MSB to LSB
def check_bit_set(n, k):
    if n & (1 << (k)):
        return True

    return False

# return rpu cluster configuration
# rpu cpus property fields: Cluster | cpus-mask | execution-mode
#
#execution mode ARM-R CPUs:
#bit 30: lockstep (lockstep enabled == 1)
#bit 31: secure mode / normal mode (secure mode == 1)
# e.g. &cpus_r5 0x2 0x80000000>
# this maps to arg1 as rpu_cluster node
# arg2: cpus-mask: 0x2 is r5-1, 0x1 is r5-0, 0x3 is both nodes
#        if 0x3/both nodes and in split then need to openamp channels provided,
#        otherwise return error
#        if lockstep valid cpus-mask is 0x3 needed to denote both being used
#  
def construct_carveouts(sdt, rsc_group_node, core):
  print("construct_carveouts")
  # carveouts each have addr,range
  mem_regions = [[0 for x in range(2)] for y in range(4)] 
  mem_region_names = {
    0 : "elfload",
    1 : "vdev0vring0",
    2 : "vdev0vring1",
    3 : "vdev0buffer",
  }
  for index,value in enumerate(rsc_group_node["memory"].value):
    if index % 4 == 1:
      mem_regions[index//4][0] = value
    elif index % 4 == 3:
       mem_regions[index//4][1] = value
  print("added carveouts from rsc_group_node ",rsc_group_node)
  carveout_phandle = 0x5ed0
  carveout_phandle_list = []
  for i in range(4):
    name = "rpu"+str(core)+mem_region_names[i]
    addr = mem_regions[i][0]
    length = mem_regions[i][1]
    print(name, addr, length)
    new_node = LopperNode(-1, "/reserved-memory/"+name)
    new_node + LopperProp(name="no-map", value=[])
    new_node + LopperProp(name="reg",value=[0,addr,0,length])
    new_node + LopperProp(name="phandle",value=carveout_phandle)
    new_node.phandle = new_node

    sdt.tree.add(new_node)

    carveout_phandle_list.append(carveout_phandle)
    carveout_phandle += 1

  print("added carveouts to tree")
  return carveout_phandle_list

def construct_mem_region(sdt, domain_node, rsc_group_node, core):
  print("construct_mem_region")
  # add reserved mem if not present
  res_mem_node = None
  carveout_phandle_list = None
  try:
    res_mem_node = sdt.tree["/reserved-memory"]
    # look for carveouts
    for node in res_mem_node:
      if "vdev0" in node.abs_path:
        print("found carveouts")
        return
    carveout_phandle_list = construct_carveouts(sdt, rsc_group_node, core)


  except:
    res_mem_node = LopperNode(-1, "/reserved-memory")
    res_mem_node + LopperProp(name="#address-cells",value=2)
    res_mem_node + LopperProp(name="#size-cells",value=2)
    res_mem_node + LopperProp(name="ranges",value=[])

    sdt.tree.add(res_mem_node)
    print("added reserved mem node ", res_mem_node)
    carveout_phandle_list = construct_carveouts(sdt, rsc_group_node, core)
  return carveout_phandle_list


# set pnode id for current rpu node
def set_rpu_pnode(sdt, r5_node, rpu_config, core, platform, remote_domain):
  print("set_rpu_pnode", platform, core, rpu_config, r5_node, remote_domain)
  print("remote_domain cpu prop vals", remote_domain.propval("cpus"))
  if r5_node.propval("pnode-id") != ['']:
    print(str(r5_node), " already has pnode-id prop with value: ",r5_node["pnode-id"].value)
    return

  rpu_pnodes = {}
  if platform == SOC_TYPE.VERSAL:
    rpu_pnodes = {0 : 0x18110005, 1: 0x18110006}
  else:
    print("only versal supported for openamp domains")
    return -1
  rpu_pnode = None
  # rpu config : true is split
  if rpu_config == "lockstep":
    rpu_pnode = rpu_pnodes[0]
  else:
     rpu_pnode = rpu_pnodes[core]

  r5_node + LopperProp(name="pnode-id", value = rpu_pnodes[0])
  r5_node.sync(sdt.FDT)

  return
 


def setup_mbox_info(sdt, domain_node, r5_node, mbox_ctr):
  print("setup_mbox_info")
  if mbox_ctr.propval("reg-names") == [''] or mbox_ctr.propval("xlnx,ipi-id") == ['']:
    print("invalid mbox ctr")
    return -1
  
  r5_node + LopperProp(name="mboxes",value=[mbox_ctr.phandle,0,mbox_ctr.phandle,1])
  r5_node + LopperProp(name="mbox-names", value = ["tx", "rx"]);
  sdt.tree.sync()
  r5_node.sync(sdt.FDT)
  return

  
def set_rpu_openamp_channel(sdt, domain_node, remoteproc_node):
  print("set_rpu_openamp_channel")

# based on rpu_cluster_config + cores determine which tcm nodes to use
# add tcm nodes to device tree
def setup_tcm_nodes(sdt, r5_node, platform, rsc_group_node):
  print("setup_tcm_nodes")
  tcm_nodes = {}
  if platform == SOC_TYPE.VERSAL:
    tcm_pnodes = {
      "ffe00000" : 0x1831800b,
      "ffe20000" : 0x1831800c,
      "ffe90000" : 0x1831800d,
      "ffeb0000" : 0x1831800e,
    }
    tcm_to_hex = {
      "ffe00000" : 0xffe00000,
      "ffe20000" : 0xffe20000,
      "ffe90000" : 0xffe90000,
      "ffeb0000" : 0xffeb0000,
    }

  else:
    print("only versal supported for openamp domains")
    return -1
  # determine which tcm nodes to use based on access list in rsc group
  bank = 0
  for phandle_val in rsc_group_node["access"].value:
    tcm = sdt.tree.pnode(phandle_val)
    if tcm != None:
      key = tcm.abs_path.split("@")[1]
      node_name = r5_node.abs_path+"/tcm_remoteproc"+str(bank)+"@"+key
      tcm_node = LopperNode(-1, node_name)
      tcm_node + LopperProp(name="pnode-id",value=tcm_pnodes[key])
      tcm_node + LopperProp(name="reg",value=[0,tcm_to_hex[key],0,0x10000])
      sdt.tree.add(tcm_node)
      bank +=1

def setup_r5_core_node(rpu_config, sdt, domain_node, rsc_group_node, core, remoteproc_node, platform, remote_domain, mbox_ctr):
  print("setup_r5_core_node")
  carveout_phandle_list = None
  r5_node = None
  # add r5 node if not present
  try:
    r5_node = sdt.tree["/rpu@ff9a0000/r5_"+str(core)]
    print("node already exists: ", r5_node)
  except:
    r5_node = LopperNode(-1, "/rpu@ff9a0000/r5_"+str(core))
    r5_node + LopperProp(name="#address-cells",value=2)
    r5_node + LopperProp(name="#size-cells",value=2)
    r5_node + LopperProp(name="ranges",value=[])
    sdt.tree.add(r5_node)
    print("added r5 node ", r5_node)
    print("add props for ",str(r5_node))
  # props
  set_rpu_pnode(sdt, r5_node, rpu_config, core, platform, remote_domain)
  setup_mbox_info(sdt, domain_node, r5_node, mbox_ctr)
  carveout_phandle_list = construct_mem_region(sdt, domain_node, rsc_group_node, core)
  if carveout_phandle_list != None:
    r5_node + LopperProp(name="memory-region",value=carveout_phandle_list)

  #tcm nodes
  for i in r5_node.subnodes():
    if "tcm" in i.abs_path:
      "tcm nodes exist"
      return

  # tcm nodes do not exist. set them up
  setup_tcm_nodes(sdt, r5_node, platform, rsc_group_node)
           
# add props to remoteproc node
def set_remoteproc_node(remoteproc_node, sdt, rpu_config):
  print("populate_remoteproc_node")
  props = []
  props.append(LopperProp(name="reg", value =   [0x0, 0xff9a0000, 0x0, 0x10000]))
  props.append(LopperProp(name="#address-cells",value=2))
  props.append(LopperProp(name="ranges",value=[]))
  props.append(LopperProp(name="#size-cells",value=2))
  props.append(LopperProp(name="core_conf",value=rpu_config))
  props.append(LopperProp(name="compatible",value="xlnx,zynqmp-r5-remoteproc-1.0"))
  for i in props:
    remoteproc_node + i
  # 

core = []
# this should only add nodes  to tree
def construct_remoteproc_node(remote_domain, rsc_group_node, sdt, domain_node,  platform, mbox_ctr):
  print("construct_remoteproc_node")
    
  print("remote_domain: ",str(remote_domain), " parent: ", remote_domain.parent)
  rpu_cluster_node = remote_domain.parent
  rpu_config = None # split or lockstep
  cpus_prop_val = rpu_cluster_node.propval("cpus")
  if cpus_prop_val != ['']:
    if len(cpus_prop_val) != 3:
      print("rpu cluster cpu prop invalid len")
      return -1
    rpu_config = "lockstep" if  check_bit_set(cpus_prop_val[2], 30)==True else "split"
    print("rpu_config: ", str(hex(cpus_prop_val[2])))
    if rpu_config == "lockstep":
      core = 0
    else:
      if cpus_prop_val[1] == 3:
        # if here this means that cluster is in split mode. look at which core from remote domain
        core_prop_val = remote_domain.propval("cpus")
        if core_prop_val == ['']:
          print("no cpus val for core ", remote_domain)
        else:
          if core_prop_val[1] == 2:
            core  = 1
          elif core_prop_val[1] == 1:
            core = 0
          else:
            print("invalid cpu prop for core ", remote_domain, core_prop_val[1])
            return -1
      else:
        print("invalid cpu prop for rpu: ",remote_domain, cpus_prop_val[1])
        return -1
  print("rpu_config ", rpu_config, "core ", str(core))

  # only add remoteproc node if mbox is present in access list of domain node
  # check domain's access list for mbox
  has_corresponding_mbox = False
  if domain_node.propval("access") != ['']:
    for i in domain_node.propval("access"):
      possible_mbox = sdt.tree.pnode(i)
      if possible_mbox != None:
        if possible_mbox.propval("reg-names") != ['']:
          has_corresponding_mbox = True

  # setup remoteproc node if not already present
  remoteproc_node = None
  try:
    remoteproc_node = sdt.tree["/rpu@ff9a0000"]
  except:
    print("remoteproc node not present. now add it to tree")
    remoteproc_node = LopperNode(-1, "/rpu@ff9a0000")
    set_remoteproc_node(remoteproc_node, sdt, rpu_config)
    sdt.tree.add(remoteproc_node, dont_sync = True)
    remoteproc_node.sync(sdt.FDT)
    remoteproc_node.resolve_all_refs()
    sdt.tree.sync()

  print("core ", core, " rpu config ", rpu_config)
  setup_r5_core_node(rpu_config, sdt, domain_node, rsc_group_node, core, remoteproc_node, platform, remote_domain, mbox_ctr)

def find_mbox_cntr(remote_domain, sdt, domain_node, rsc_group):
  print("find_mbox_cntr: ",rsc_group, rsc_group.phandle)
  # if there are multiple openamp channels
  # then there can be multiple mbox controllers
  # with this in mind, there can be pairs of rsc groups and mbox cntr's
  # per channel
  # if there are i  channels, then determine 'i' here by
  # associating a index for the resource group, then find i'th
  # mbox cntr from domain node's access list
  include_list = domain_node.propval("include")
  if include_list == ['']:
    print("no include prop for domain node")
    return -1
  rsc_group_index = 0
  for val in include_list:
    # found corresponding mbox
    if sdt.tree.pnode(val) != None:
      if "resource_group" in sdt.tree.pnode(val).abs_path:
        print("find_mbox_cntr: getting index for rsc group: ", sdt.tree.pnode(val).abs_path, rsc_group_index, sdt.tree.pnode(val).phandle)
        if sdt.tree.pnode(val).phandle == rsc_group.phandle:
          break
        rsc_group_index += 1
  access_list = domain_node.propval("access")
  if access_list == ['']:
    print("no access prop for domain node")
    return -1
  mbox_index = 0
  for val in access_list:
    print(domain_node, " access_list: ", str(access_list), " current val ", str(val), "rsc_group_index ", rsc_group_index, " mbox_index ", mbox_index)
    mbox = sdt.tree.pnode(val)
    if mbox != None and mbox.propval("reg-names") != [''] and  mbox.propval("xlnx,ipi-id") != ['']:
      print(mbox.abs_path, rsc_group_index, mbox_index)
      if mbox_index == rsc_group_index:
        return mbox
      mbox_index += 1
  print("did not find corresponding mbox")
  return -1

      
      

def parse_openamp_domain(sdt, options, tgt_node):
  print("here to parse openamp domain")
  domain_node = sdt.tree[tgt_node]
  root_node = sdt.tree["/"]
  platform = SOC_TYPE.UNINITIALIZED
  if 'versal' in str(root_node['compatible']):
      platform = SOC_TYPE.VERSAL
  elif 'zynqmp' in str(root_node['compatible']):
      platform = SOC_TYPE.ZYNQMP
  else:
      print("invalid input system DT")
      return False

  rsc_groups = determine_role(sdt, domain_node)
  if rsc_groups == -1:
    return rsc_groups

  # if master, find corresponding  slave
  # if none report error
  for current_rsc_group in rsc_groups:
    # each openamp channel's remote/slave should be different domain
    # the domain can be identified by its unique combination of domain that includes the same resource group as the
    # openamp remote domain in question
    remote_domain = find_remote(sdt, domain_node, current_rsc_group)
    if remote_domain == -1:
      print("failed to find_remote")
      return -1
    mbox_ctr = find_mbox_cntr(remote_domain, sdt, domain_node, current_rsc_group)
    if mbox_ctr == -1:
      print("find_mbox_cntr failed")
      return -1
    # should only add nodes to tree
    construct_remoteproc_node(remote_domain, current_rsc_group, sdt, domain_node, platform, mbox_ctr)
  # ensure interrupt parent for openamp-related ipi message buffers is set
  update_mbox_cntr_intr_parent(sdt)
  # ensure that extra ipi mboxes do not have props that interfere with linux boot
  trim_ipis(sdt) 

# as of nov 17 2020:
# this is what it needs to account for:
#
# identify ipis, shared pages (have defaults but allow them to be overwritten
# by system architect
#
#
# kernel space case
#   linux
#   - update memory-region
#   - mboxes
#   - zynqmp_ipi1::interrupt-parent
#   rpu
#   - header
# user space case
#   linux
#   - header
#   rpu
#   - header
def xlnx_openamp_rpu( tgt_node, sdt, options ):
    print("here xlnx_openamp_rpu")
    try:
        verbose = options['verbose']
    except:
        verbose = 0

    if verbose:
        print( "[INFO]: cb: xlnx_openamp_rpu( %s, %s, %s )" % (tgt_node, sdt, verbose))

    root_node = sdt.tree["/"]
    platform = SOC_TYPE.UNINITIALIZED
    if 'versal' in str(root_node['compatible']):
        platform = SOC_TYPE.VERSAL
    elif 'zynqmp' in str(root_node['compatible']):
        platform = SOC_TYPE.ZYNQMP
    else:
        print("invalid input system DT")
        return False

    # here parse openamp domain if applicable
    ret = parse_openamp_domain(sdt, options, tgt_node)
    if ret == -2:
      return True
    # the below is being depcrecated
    return True

    # find the added rpu node
    try:
        rpu_node = sdt.tree[RPU_PATH]
        is_kernel_case = True
    except:
        print( "[ERROR] 2: cannot find the target rpu node" )
        rpu_node = None
        is_kernel_case = False
    try:
        memory_node = sdt.tree[ "/reserved-memory" ]
    except:
        return False
    ipis = parse_ipis_for_rpu(sdt, options)

    if is_kernel_case:
        remoteproc_node = sdt.tree[RPU_PATH]
    else:
       remoteproc_node = None
    for current_r5 in remoteproc_node.subnodes():
      if current_r5.propval("memory-region") != ['']:
        mem_carveouts = parse_memory_carevouts(sdt, options, current_r5)
        [rsc_mem_pa,shared_mem_size] = get_r5_needed_symbols(mem_carveouts)
        if rsc_mem_pa == -1 or shared_mem_size == -1:
          print("[ERROR]: failed to find rsc_mem_pa or shared_mem_size")
        inputs = {
          "CHANNEL_0_RSC_MEM_SIZE" : "0x2000UL",
          "CHANNEL_0_TX" : "FW_RSC_U32_ADDR_ANY",
          "CHANNEL_0_RX" : "FW_RSC_U32_ADDR_ANY",
        }
        # userspace case is accounted for later on so do not worry about vring tx/rx
        inputs = setup_ipi_inputs(inputs, platform, ipis, options)
        generate_openamp_file( mem_carveouts, options, platform, is_kernel_case, inputs )

    if rpu_node != None:
        handle_rpmsg_kernelspace_case(tgt_node, sdt, options, memory_node, rpu_node, platform)
    #else:
    #    handle_rpmsg_userspace_case(tgt_node, sdt, options, memory_node, rpu_node, rsc_mem_pa, shared_mem_size, platform)

    return True

