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

def trim_ipis(sdt):
    interrupts_mapping = {'/amba/mailbox@ff360000':33,'/amba/mailbox@ff370000':34}
    unneeded_props = [ "xlnx,ipi-bitmask", "xlnx,ipi-id", "xlnx,ipi-target-count", "xlnx,s-axi-highaddr", "xlnx,cpu-name", "xlnx,buffer-base", "xlnx,buffer-index", "xlnx,s-axi-baseaddr", "xlnx,int-id", "xlnx,bit-position", "#size-cells","#address-cells","firewall-0", "status"]

    amba_sub_nodes = sdt.tree['/amba'].subnodes()
    for node in amba_sub_nodes:
      node_compat = node.propval("compatible")
      if node_compat != [""]:
       if 'xlnx,zynqmp-ipi-mailbox' in node_compat:
         if node.abs_path in list(interrupts_mapping.keys()):
           # update interrupts
           interrupts_prop = node["interrupts"]
           print("update ",node,str(interrupts_prop))
           interrupts_prop.value = [0, interrupts_mapping[node.abs_path],4]
           print("node interrupts value now", str(interrupts_prop))

         for up in unneeded_props:
           node.delete(up)

         for child in node.subnodes():
           if child.propval("xlnx,ipi-id") != ['']:
             child.delete(child["xlnx,ipi-id"])
           if child.propval("xlnx,ipi-bitmask") != ['']:
             child.delete(child["xlnx,ipi-bitmask"])
           if child.propval("status") != ['']:
             child.delete(child["status"])


         newprop = LopperProp("xlnx,ipi")
         node + newprop
         node["compatible"].value = "ipi_uio"
         old_reg_base = node["reg"].value[1]
         node["reg"].value = [0x0, old_reg_base, 0x0, 0x1000]
         node.sync(sdt.FDT)

def is_compat( node, compat_string_to_test ):
    if re.search( "openamp,xlnx-rpu", compat_string_to_test):
        return xlnx_openamp_rpu
    return ""

def update_mbox_cntr_intr_parent(sdt):
  # find phandle of a72 gic for mailbox controller
  a72_gic_node = sdt.tree["/amba_apu/interrupt-controller@f9000000"]
  # set mailbox controller interrupt-parent to this phandle
  mailbox_cntr_node = sdt.tree["/zynqmp_ipi1"]
  mailbox_cntr_node["interrupt-parent"].value = a72_gic_node.phandle
  sdt.tree.sync()
  sdt.tree.resolve()

# 1 for master, 0 for slave
# for each openamp channel, return mapping of role to resource group
def determine_role(sdt, domain_node):
  include_prop = domain_node["include"]
  rsc_groups = []
  current_rsc_group = None
  if len(list(include_prop.value)) % 2 == 1:
    return -1
  for index,value in enumerate(include_prop.value):
    if index % 2 == 0:
      current_rsc_group = sdt.tree.pnode(value)
    else:
      if value == 1: # only for openamp master
        if current_rsc_group == None:
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
  # static var that persists beyond lifetime of first function call
  # this is needed as there may be more than 1 openamp channel
  # so multiple carveouts' phandles are required
  if not hasattr(construct_carveouts,"carveout_phandle"):
    # it doesn't exist yet, so initialize it
    construct_carveouts.carveout_phandle = 0x5ed0

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
  carveout_phandle_list = []

  for i in range(4):
    name = "rpu"+str(core)+mem_region_names[i]
    addr = mem_regions[i][0]
    length = mem_regions[i][1]
    new_node = LopperNode(-1, "/reserved-memory/"+name)
    new_node + LopperProp(name="no-map", value=[])
    new_node + LopperProp(name="reg",value=[0,addr,0,length])
    new_node + LopperProp(name="phandle",value=construct_carveouts.carveout_phandle)
    new_node.phandle = new_node

    sdt.tree.add(new_node)
    print("added node: ",new_node)

    carveout_phandle_list.append(construct_carveouts.carveout_phandle)
    construct_carveouts.carveout_phandle += 1

  return carveout_phandle_list

def construct_mem_region(sdt, domain_node, rsc_group_node, core):
  # add reserved mem if not present
  print("construct_mem_region: core: ",core)
  res_mem_node = None
  carveout_phandle_list = None
  try:
    res_mem_node = sdt.tree["/reserved-memory"]
    print("found pre-existing reserved mem node")
  except:
    res_mem_node = LopperNode(-1, "/reserved-memory")
    res_mem_node + LopperProp(name="#address-cells",value=2)
    res_mem_node + LopperProp(name="#size-cells",value=2)
    res_mem_node + LopperProp(name="ranges",value=[])

    sdt.tree.add(res_mem_node)
    print("added reserved mem node ", res_mem_node)

  return construct_carveouts(sdt, rsc_group_node, core)


# set pnode id for current rpu node
def set_rpu_pnode(sdt, r5_node, rpu_config, core, platform, remote_domain):
  if r5_node.propval("pnode-id") != ['']:
    print("pnode id already exists for node ", r5_node)
    return -1

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

  r5_node + LopperProp(name="pnode-id", value = rpu_pnodes[core])
  r5_node.sync(sdt.FDT)
  print("set ",r5_node,"pnode-id")

  return

def setup_mbox_info(sdt, domain_node, r5_node, mbox_ctr):
  if mbox_ctr.propval("reg-names") == [''] or mbox_ctr.propval("xlnx,ipi-id") == ['']:
    print("invalid mbox ctr")
    return -1
  
  r5_node + LopperProp(name="mboxes",value=[mbox_ctr.phandle,0,mbox_ctr.phandle,1])
  r5_node + LopperProp(name="mbox-names", value = ["tx", "rx"]);
  sdt.tree.sync()
  r5_node.sync(sdt.FDT)
  print("set ",r5_node," mbox info")
  return
  
# based on rpu_cluster_config + cores determine which tcm nodes to use
# add tcm nodes to device tree
def setup_tcm_nodes(sdt, r5_node, platform, rsc_group_node):
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
      print('added ',tcm_node.abs_path)

  return 0

def setup_r5_core_node(rpu_config, sdt, domain_node, rsc_group_node, core, remoteproc_node, platform, remote_domain, mbox_ctr):
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
  ret = set_rpu_pnode(sdt, r5_node, rpu_config, core, platform, remote_domain)
  if ret == -1:
    print("set_rpu_pnode failed")
    return ret
  ret = setup_mbox_info(sdt, domain_node, r5_node, mbox_ctr)
  if ret == -1:
    print("setup_mbox_info failed")
    return ret

  carveout_phandle_list = construct_mem_region(sdt, domain_node, rsc_group_node, core)
  if carveout_phandle_list == -1:
    print("construct_mem_region failed")
    return ret

  if carveout_phandle_list != None:
    print("adding prop memory-region to ",r5_node)
    r5_node + LopperProp(name="memory-region",value=carveout_phandle_list)

  #tcm nodes
  for i in r5_node.subnodes():
    if "tcm" in i.abs_path:
      "tcm nodes exist"
      return -1

  # tcm nodes do not exist. set them up
  setup_tcm_nodes(sdt, r5_node, platform, rsc_group_node)
           
# add props to remoteproc node
def set_remoteproc_node(remoteproc_node, sdt, rpu_config):
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
  rpu_cluster_node = remote_domain.parent
  rpu_config = None # split or lockstep
  cpus_prop_val = rpu_cluster_node.propval("cpus")
  if cpus_prop_val != ['']:
    if len(cpus_prop_val) != 3:
      print("rpu cluster cpu prop invalid len")
      return -1
    rpu_config = "lockstep" if  check_bit_set(cpus_prop_val[2], 30)==True else "split"
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

  return setup_r5_core_node(rpu_config, sdt, domain_node, rsc_group_node, core, remoteproc_node, platform, remote_domain, mbox_ctr)

def find_mbox_cntr(remote_domain, sdt, domain_node, rsc_group):
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
    mbox = sdt.tree.pnode(val)
    if mbox != None and mbox.propval("reg-names") != [''] and  mbox.propval("xlnx,ipi-id") != [''] and mbox.propval('xlnx,ipi') == ['']:
      if mbox_index == rsc_group_index:
        return mbox
      mbox_index += 1
  print("did not find corresponding mbox")
  return -1

# only do this if mailbox controller provided
def rpmsg_kernelspace_flow(sdt, remote_domain, domain_node, current_rsc_group,
                           platform, mbox_ctr):
  # should only add nodes to tree
  ret = construct_remoteproc_node(remote_domain, current_rsc_group, sdt, domain_node, platform, mbox_ctr)
  if ret == -1:
    print("construct_remoteproc_node failed")
    return ret
  # ensure interrupt parent for openamp-related ipi message buffers is set
  update_mbox_cntr_intr_parent(sdt)

def set_channel_dict_mem_info(current_rsc_group, openamp_config_map):
  vring_mem_size = 0

  for index,value in enumerate(current_rsc_group["memory"].value):
    if index == 1:
      openamp_config_map['RSC_MEM_PA'] =  str(hex(0x20000+value))+'UL'
      openamp_config_map['SHM_DEV_NAME'] =  str(hex(0x20000+value)).replace('0x','')+'.shm'
    if index == 3:
      openamp_config_map['SHARED_BUF_SIZE'] = str(hex(value))
    elif index == 5:
      openamp_config_map['VRING_MEM_PA'] = str(hex(value))+'UL'
      vring_mem_pa = str(hex(value))
    elif index == 7 or index == 11:
      vring_mem_size += value
    elif index == 13:
      openamp_config_map['SHARED_BUF_PA'] = str(hex(value))+'UL'

  openamp_config_map['VRING_MEM_SIZE'] = str(hex(vring_mem_size))+'UL'
  openamp_config_map['RSC_MEM_SIZE'] = str(hex(0x2000))+'UL'

# generate mem carveouts for userspace flow
def set_rpmsg_userspace_mem_info(current_rsc_group, sdt, openamp_config_map):
  base = 0xffffffff
  reserved_mem_base = 0
  for index,value in enumerate(current_rsc_group["memory"].value):
    if index % 4 == 1 and value < base:
      base = value
    if index == 1:
      reserved_mem_base = base
    elif index == 15:
      length = value << 4
  print("reserved_mem_base ",str(hex(reserved_mem_base)), " length ",str(hex(length)))
  new_node = LopperNode(-1, "/reserved-memory/shm0-userspace@"+str(hex(reserved_mem_base)).replace('0x',''))
  new_node + LopperProp(name="no-map", value=[])
  new_node + LopperProp(name="reg",value=[0,reserved_mem_base,0,length])
  sdt.tree.add(new_node)

  rsc_mem_pa = 0x20000+base
  addr_str = str(hex(rsc_mem_pa)).replace('0x','')
  # add node to tree
  rpmsg_shared_mem_node = LopperNode(-1,"/amba/shm@"+addr_str)
  rpmsg_shared_mem_node + LopperProp("compatible",value="openamp-shm")
  rpmsg_shared_mem_node + LopperProp("reg",value=[0x0,rsc_mem_pa,0x0,0x0100000])
  sdt.tree.add(rpmsg_shared_mem_node, dont_sync = True)
  rpmsg_shared_mem_node.sync(sdt.FDT)
  rpmsg_shared_mem_node.resolve_all_refs()
  sdt.tree.sync()

  set_channel_dict_mem_info(current_rsc_group, openamp_config_map)

  return 0

def find_host_ipi_userspace_flow(remote_domain, sdt, domain_node, rsc_group):
  # if there are multiple openamp channels
  # then there can be multiple host ipis
  # with this in mind, there can be pairs of rsc groups and ipis
  # per channel
  # if there are i  channels, then determine 'i' here by
  # associating a index for the resource group, then find i'th
  # ipi from domain node's access list
  include_list = domain_node.propval("include")
  if include_list == ['']:
    print("no include prop for domain node")
    return -1
  rsc_group_index = 0
  for val in include_list:
    # found corresponding ipi
    if sdt.tree.pnode(val) != None:
      if "resource_group" in sdt.tree.pnode(val).abs_path:
        if sdt.tree.pnode(val).phandle == rsc_group.phandle:
          break
        rsc_group_index += 1
  access_list = domain_node.propval("access")
  if access_list == ['']:
    print("no access prop for domain node")
    return -1
  ipi_phandle = access_list[rsc_group_index]
  ipi = sdt.tree.pnode(ipi_phandle)
  if ipi != None and ipi.propval("xlnx,ipi") != ['']:
    return ipi
  else:
    print("did not find corresponding ipi")
    return -1


# given remote domain, read access list
# look for xlnx,ipi compat node. map this to the ipi channel bitmask
# that the host can communicate with
def set_rpmsg_userspace_ipi_info(domain_node, remote_domain, sdt, openamp_config_map, rsc_group):
  ipi_to_bitmask = {
    0xff340000 : 0x08,
    0xff350000 : 0x10,
    0xff360000 : 0x20,
    0xff370000 : 0x40,
  }
  access_prop = remote_domain.propval('access')
  if access_prop == ['']:
    print('no access prop for ',remote_domain)
    return -1
  elif len(access_prop) != 1:
    print('invalid access prop ',access_prop)
    return -1

  remote_domain_ipi = sdt.tree.pnode(access_prop[0])
  if remote_domain_ipi == None:
    print("invalid phandle for remote domain", access_prop)
    return -1

 
  host_ipi = find_host_ipi_userspace_flow(remote_domain, sdt, domain_node, rsc_group)
  if host_ipi == -1:
    print("find_host_ipi_userspace_flow failed")
    return -1

  openamp_config_map["IPI_CHN_BITMASK"] = str(hex(ipi_to_bitmask[remote_domain_ipi["reg"].value[1]]))+'UL'
  openamp_config_map["IPI_DEV_NAME"] = str(hex(host_ipi["reg"].value[1])).replace('0x','')  + '.mailbox'

  return 0
 


#TODO: handle generating header file for linux, remote
# only do this if mailbox controller provided
def rpmsg_userspace_flow(sdt, remote_domain, domain_node, current_rsc_group,
                           platform, options, index):
  openamp_config_map = {}

  ret = set_rpmsg_userspace_mem_info(current_rsc_group,sdt, openamp_config_map)
  if ret == -1:
    print("set_rpmsg_userspace_mem_info failed")
    return ret
  
     # ipi channel bitmask to communicate with remote
  ipi_chn_bitmask = set_rpmsg_userspace_ipi_info(domain_node, remote_domain, sdt, openamp_config_map, current_rsc_group)
  if ipi_chn_bitmask == -1:
    print("set_rpmsg_userspace_ipi_info failed")
    return ipi_chn_bitmask
  options['args'] = ['openamp_xlnx_ipc_rpmsg_userspace_host_to_'+str(index)+'.h']
  write_openamp_header(platform, openamp_config_map,IPC_ROLE.HOST, options)
  return 0


def parse_openamp_domain(sdt, options, tgt_node):
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
  for index,current_rsc_group in enumerate(rsc_groups):
    # each openamp channel's remote/slave should be different domain
    # the domain can be identified by its unique combination of domain that includes the same resource group as the
    # openamp remote domain in question
    remote_domain = find_remote(sdt, domain_node, current_rsc_group)
    if remote_domain == -1:
      print("failed to find_remote")
      return remote_domain
    mbox_ctr = find_mbox_cntr(remote_domain, sdt, domain_node, current_rsc_group)
    if mbox_ctr == -1:
      ret = rpmsg_userspace_flow(sdt, remote_domain, domain_node, current_rsc_group,
                                 platform, options, index)
      if ret == -1:
        print("rpmsg_userspace_flow failed")
        return ret
    else:
      ret = rpmsg_kernelspace_flow(sdt, remote_domain, domain_node,
                                   current_rsc_group, platform, mbox_ctr)
      if ret == -1:
        print("openamp xilinx: setup for romsg in kernelspace flow failed")
        return ret

  # ensure that extra ipi mboxes do not have props that interfere with linux boot
  trim_ipis(sdt) 
  return True

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
    return parse_openamp_domain(sdt, options, tgt_node)
