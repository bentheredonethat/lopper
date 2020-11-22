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
  print(rsc_groups)
  return rsc_groups

# in this case remote is rpu
# find node that is other end of openamp channel
def find_remote_cluster(sdt, domain_node, rsc_group_node):
  domains = sdt.tree["/domains"]
  # find other domain including the same resource group
  remote_domain = None
  for node in domains.subnodes():
    if node.propval("include") != [''] and node != domain_node:
      remote_domain = node
      break
  
  if remote_domain != None:
    return remote_domain
  else:
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
def determine_rpu_config(sdt, domain_node, rpu_cluster):
  print("in determine_rpu_config")
  include_prop = rpu_cluster.propval("cpus")
  if include_prop  != ['']:
    if len(include_prop) != 3:
      print("include prop doesnt have correct length of args")
      return -1
    print("rpu config: ",  check_bit_set(include_prop[2], 30))
    return check_bit_set(include_prop[2], 30)

  else:
    print("no include prop for remote openamp domain")
    return -1

def determine_core_for_domain(sdt, rpu_cluster):
  include_prop = rpu_cluster.propval("cpus")
  return include_prop[1]

def parse_rsc_group_for_mems(sdt, domain_node):
  print("parse_rsc_group_for_mems")

def construct_carveouts(sdt, rsc_group_node):
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
    name = "rpuX"+ mem_region_names[i]
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

def construct_mem_region(sdt, domain_node, rsc_group_node):
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
    carveout_phandle_list = construct_carveouts(sdt, rsc_group_node)


  except:
    res_mem_node = LopperNode(-1, "/reserved-memory")
    res_mem_node + LopperProp(name="#address-cells",value=2)
    res_mem_node + LopperProp(name="#size-cells",value=2)
    res_mem_node + LopperProp(name="ranges",value=[])

    sdt.tree.add(res_mem_node)
    print("added reserved mem node ", res_mem_node)
    carveout_phandle_list = construct_carveouts(sdt, rsc_group_node)
  return carveout_phandle_list



def set_rpu_pnode(sdt, r5_node, rpu_config, cores_to_add, platform):
  print("set_rpu_pnode", platform, cores_to_add, rpu_config, r5_node)
  rpu_pnodes = {}
  if platform == SOC_TYPE.VERSAL:
    rpu_pnodes = {0 : 0x18110005, 1: 0x18110006}
  else:
    print("only versal supported for openamp domains")
    return -1
  # rpu config : true is split

  # if lockstep set to r5 0
  if rpu_config == False:
       r5_node + LopperProp(name="pnode-id", value = rpu_pnodes[0])
       r5_node.sync(sdt.FDT)

  # if split and cores to add is length(1) then directly call map
  # TODO
  # if split and cores to is length(2)
  # TODO
  #    add for each r5 node in list

  return
 


def setup_mbox_info(sdt, domain_node, r5_node):
  print("setup_mbox_info")
  access_prop = domain_node["access"]
  print(domain_node)
  for i in access_prop.value:
    mbox = sdt.tree.pnode(i)
    if mbox != None and mbox.propval("reg-names") != [''] and mbox.propval("xlnx,ipi-id") != ['']:
      print(mbox)
      r5_node + LopperProp(name="mboxes",value=[i,0,i,1])
      r5_node + LopperProp(name="mbox-names", value = ["tx", "rx"]);
      sdt.tree.sync()
      r5_node.sync(sdt.FDT)
      return

  
def set_rpu_openamp_channel(sdt, domain_node, remoteproc_node):
  print("set_rpu_openamp_channel")

# based on rpu_cluster_config + cores determine which tcm nodes to use
# add tcm nodes to device tree
def setup_tcm_nodes(sdt, r5_node, platform, rpu_config, rsc_group_node):
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
  for phandle_val in rsc_group_node["access"].value:
    tcm = sdt.tree.pnode(phandle_val)
    bank = 0
    if tcm != None:
      key = tcm.abs_path.split("@")[1]
      node_name = r5_node.abs_path+"/tcm_remoteproc"+str(bank)+"@"+key
      tcm_node = LopperNode(-1, node_name)
      tcm_node + LopperProp(name="pnode-id",value=tcm_pnodes[key])
      tcm_node + LopperProp(name="reg",value=[0,tcm_to_hex[key],0,0x10000])
      sdt.tree.add(tcm_node)

def setup_r5_core_node(rpu_config, sdt, domain_node, rsc_group_node, cores_to_add, remoteproc_node, platform):

  print("setup_r5_core_node")
  carveout_phandle_list = None
  # add r5 nodes if not present
  for i in cores_to_add:
    print("cores_to_add: ",i)
    try:
      r5_node = sdt.tree["/rpu@ff9a0000/r5_"+str(i)]
    except:
      r5_node = LopperNode(-1, "/rpu@ff9a0000/r5_"+str(i))
      r5_node + LopperProp(name="#address-cells",value=2)
      r5_node + LopperProp(name="#size-cells",value=2)
      r5_node + LopperProp(name="ranges",value=[])
      sdt.tree.add(r5_node)
      print("added r5 node ", r5_node)
      # props
      set_rpu_pnode(sdt, r5_node, rpu_config, cores_to_add, platform)
      setup_mbox_info(sdt, domain_node, r5_node)
      carveout_phandle_list = construct_mem_region(sdt, domain_node, rsc_group_node)

      #tcm nodes
      for i in r5_node.subnodes():
        if "tcm" in i.abs_path:
          "tcm nodes exist"
          return

      # tcm nodes do not exist. set them up
      setup_tcm_nodes(sdt, r5_node, platform, rpu_config, rsc_group_node)
      # add mem regions
      if carveout_phandle_list != None:
        r5_node + LopperProp(name="memory-region",value=carveout_phandle_list)
        
# add props to remoteproc node
def set_remoteproc_node(remoteproc_node, sdt, rpu_config):
  print("populate_remoteproc_node")
  props = []
  props.append(LopperProp(name="reg", value =   [0x0, 0xff9a0000, 0x0, 0x10000]))
  props.append(LopperProp(name="#address-cells",value=2))
  props.append(LopperProp(name="ranges",value=[]))
  props.append(LopperProp(name="#size-cells",value=2))
  props.append(LopperProp(name="core_conf",value="split" if rpu_config == False else "lockstep"))
  props.append(LopperProp(name="compatible",value="xlnx,zynqmp-r5-remoteproc-1.0"))
  for i in props:
    remoteproc_node + i
  # 

# this should only add nodes  to tree
def construct_remoteproc_node(rpu_config, rsc_group_node, sdt, domain_node, rpu_cluster, platform):
  print("construct_remoteproc_node")

  # only add remoteproc node if mbox is present in access list of domain node
  # check domain's access list for mbox
  has_corresponding_mbox = False
  if domain_node.propval("access") != ['']:
    for i in domain_node.propval("access"):
      possible_mbox = sdt.tree.pnode(i)
      if possible_mbox != None:
        if possible_mbox.propval("reg-names") != ['']:
          has_corresponding_mbox = True
  if has_corresponding_mbox == False:
    print("domain node does not have access list, so do not construct remoteproc node")

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


  cores_used = determine_core_for_domain(sdt, rpu_cluster)
  print("cores_used ", cores_used, " rpu config ", rpu_config)
  num_cores = 1 if (rpu_config == False or cores_used < 3) else 2
  cores_to_add = []
  print("num cores ", num_cores)
  if num_cores < 2:
    # figure out which core. 0x2 is rpu1, 0x1 is rpu0
    cores_to_add.append( 0 if cores_used == 1 else 1)
  else:
    cores_to_add.append(0)
    cores_to_add.append(1)
    print("TODO: add support for multiple channels")
  # lockstep is false, split is true
  # if lockstep from rpu config , only generate 1 r5 core node in remoteproc
  # if split, and 0x1 or 0x2, then generate 1 node
  # if r5 nodes not present
  setup_r5_core_node(rpu_config, sdt, domain_node, rsc_group_node, cores_to_add, remoteproc_node, platform)


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
    rpu_cluster = find_remote_cluster(sdt, domain_node, current_rsc_group)
    if rpu_cluster == -1:
      print("failed to find_remote_cluster")
      return -1
    rpu_config = determine_rpu_config(sdt, domain_node, rpu_cluster)

    # should only add nodes to tree
    construct_remoteproc_node(rpu_config, current_rsc_group, sdt, domain_node, rpu_cluster, platform)
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

