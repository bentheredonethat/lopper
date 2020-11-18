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

def parse_ipis_for_rpu(sdt, options):
    try:
        verbose = options['verbose']
    except:
        verbose = 0

    ipi_list = []
    amba_sub_nodes = sdt.tree['/amba'].subnodes()
    for node in amba_sub_nodes:
      node_compat = node.propval("compatible")
      if node_compat != [""]:
       if 'xlnx,zynqmp-ipi-mailbox' in node_compat:
         ipi_list.append(hex(int(node.propval("reg")[1])))

    if verbose:
        print( "[INFO]: Dedicated IPIs for OpenAMP: %s" % ipi_list)

    return ipi_list

def is_compat( node, compat_string_to_test ):
    if re.search( "openamp,xlnx-rpu", compat_string_to_test):
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
  print("here 2")
  a72_gic_node = sdt.tree["/amba_apu/interrupt-controller@f9000000"]
  # set mailbox controller interrupt-parent to this phandle
  mailbox_cntr_node = sdt.tree["/zynqmp_ipi1"]
  mailbox_cntr_node["interrupt-parent"].value = a72_gic_node.phandle
  sdt.tree.sync()
  sdt.tree.resolve()


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
    return memory_node

# tgt_node: is the openamp domain node number
# sdt: is the system device tree
# TODO: this routine needs to be factored and made smaller


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

