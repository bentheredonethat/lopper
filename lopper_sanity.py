#!/usr/bin/python3

#/*
# * Copyright (c) 2019,2020 Xilinx Inc. All rights reserved.
# *
# * Author:
# *       Bruce Ashfield <bruce.ashfield@xilinx.com>
# *
# * SPDX-License-Identifier: BSD-3-Clause
# */

import struct
import sys
import types
import os
import re
import shutil
import filecmp
from pathlib import Path
from pathlib import PurePath
import tempfile
from enum import Enum
import textwrap
from collections import UserDict
from collections import OrderedDict
import copy

import libfdt
from libfdt import Fdt, FdtException, QUIET_NOTFOUND, QUIET_ALL

from lopper_tree import *
from lopper import *
from lopper_fdt import *

def setup_lops( outdir ):
    with open( outdir + "/lops.dts", "w") as w:
            w.write("""\
/*
 * Copyright (c) 2019,2020 Xilinx Inc. All rights reserved.
 *
 * Author:
 *       Bruce Ashfield <bruce.ashfield@xilinx.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


/dts-v1/;

/ {
        compatible = "system-device-tree-v1";
        lops {
                // compatible = "system-device-tree-v1,lop";
                lop_0 {
                        // sanity on modules is not currently implemented
                        compatible = "system-device-tree-v1,lop,assist-v1";
                        node = "/domains/openamp_r5";
                        id = "openamp,domain-v1";
                        noexec;
                };
                lop_1 {
                        // node name modify
                        compatible = "system-device-tree-v1,lop,modify";
                        modify = "/cpus::cpus_a72";
                };
                lop_2 {
                        compatible = "system-device-tree-v1,lop,modify";
                        // format is: "path":"property":"replacement"
                        //    - modify to "nothing", is a remove operation
                        //    - modify with no property is node operation (rename or remove)
                        modify = "/cpus/:access:";
                };
                lop_3 {
                        // modify to "nothing", is a remove operation
                        compatible = "system-device-tree-v1,lop,modify";
                        modify = "/cpus_a72/:no-access:";
                };
                lop_4 {
                        // node delete
                        compatible = "system-device-tree-v1,lop,modify";
                        modify = "/anode_to_delete::";
                };
                lop_5 {
                        // node name modify
                        compatible = "system-device-tree-v1,lop,modify";
                        modify = "/amba/::";
                        noexec;
                };
                lop_6 {
                        compatible = "system-device-tree-v1,lop,modify";
                        modify = "/amba_apu/nested-node::";
                };
                lop_7 {
                        // node add
                        compatible = "system-device-tree-v1,lop,add";
                        node_src = "zynqmp-rpu";
                        node_dest = "/zynqmp-rpu";
                        zynqmp-rpu {
                            compatible = "xlnx,zynqmp-r5-remoteproc-1.0";
                            #address-cells = <2>;
                            #size-cells = <2>;
                            ranges;
                            core_conf = "__core_conf__";
                            r5_0: __cpu__ {
                                  // #address-cells = <2>;
                                  // #size-cells = <2>;
                                  // <0xF> indicates that it must be replaced
                                  #address-cells = <0xF>;
                                  #size-cells = <0xF>;
                                  ranges;
                                  // memory-region = <&rproc_0_reserved>, <&rproc_0_dma>;
                                  memory-region = <&__memory_access__>;
                                  // mboxes = <&ipi_mailbox_rpu0 0>, <&ipi_mailbox_rpu0 1>;
                                  mboxes = <&__mailbox_ipi__>;
                                  // mbox-names = "tx", "rx";
                                  mbox-names = "__mbox_names__";
                                  tcm_0_a: tcm_0@0 {
                                           reg = <0x0 0xFFE00000 0x0 0x10000>;
                                  };
                                  tcm_0_b: tcm_0@1 {
                                         reg = <0x0 0xFFE20000 0x0 0x10000>;
                                  };
                            };
                        };
                  };
                  lop_9 {
                          compatible = "system-device-tree-v1,lop,modify";
                          modify = "/zynqmp-rpu:mbox-names:lopper-mboxes";
                  };
                  lop_10 {
                          // optionally execute a routine in a loaded module. If the routine
                          // isn't found, this is NOT a failure. Since we don't want to tightly
                          // couple these transforms and loaded modules
                          compatible = "system-device-tree-v1,lop,assist-v1";
                          id = "openamp,xlnx-rpu";
                          node = "/domains/openamp_r5";
                  };
                  lop_11 {
                        // property value modify
                        compatible = "system-device-tree-v1,lop,modify";
                        // disabled for now: will be put in a test transforms .dts file
                        modify = "/:model:this is a test";
                 };
                 lop_11_1 {
                         compatible = "system-device-tree-v1,lop,modify";
                         modify = "/amba/.*ethernet.*phy.*:regexprop:lopper-id-regex";
                 };
                 lop_12 {
                        // test: property add
                        // example: add a special ID into various nodes
                        compatible = "system-device-tree-v1,lop,modify";
                        // disabled for now: will be put in a test transforms .dts file
                        modify = "/:pnode-id:0x7";
                 };
                 lop_13 {
                        compatible = "system-device-tree-v1,lop,output";
                        outfile = "openamp-test.dts";
                        nodes = "reserved-memory", "zynqmp-rpu", "zynqmp_ipi1";
                 };
                 lop_13_1 {
                        compatible = "system-device-tree-v1,lop,output";
                        outfile = "openamp-test.dtb";
                        nodes = "reserved-memory", "zynqmp-rpu", "zynqmp_ipi1";
                 };
                 lop_14 {
                        compatible = "system-device-tree-v1,lop,output";
                        outfile = "linux.dts";
                        nodes = "*";
                 };
                 lop_14_1 {
                        compatible = "system-device-tree-v1,lop,output";
                        outfile = "linux-amba.dts";
                        nodes = ".*amba.*";
                 };
        };
};
            """)

    return "/tmp/lops.dts"

def setup_assist_lops( outdir ):
    with open( outdir + "/lops-assists.dts", "w") as w:
            w.write("""\
/*
 * Copyright (c) 2019,2020 Xilinx Inc. All rights reserved.
 *
 * Author:
 *       Bruce Ashfield <bruce.ashfield@xilinx.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


/dts-v1/;

/ {
        compatible = "system-device-tree-v1";
        lops {
                lop_0 {
                        compatible = "system-device-tree-v1,lop,assist-v1";
                        node = "/domains/openamp_r5";
                        id = "access-domain,domain-v1";
                };
        };
};
            """)

    return "/tmp/lops-assists.dts"


def setup_device_tree( outdir ):
    with open( outdir + "/tester.dts", "w") as w:
            w.write("""\
/*
 * Copyright (c) 2019,2020 Xilinx Inc. All rights reserved.
 *
 * Author:
 *       Bruce Ashfield <bruce.ashfield@xilinx.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/dts-v1/;

/ {
        compatible = "xlnx,versal-vc-p-a2197-00-revA", "xlnx,versal-vc-p-a2197-00", "xlnx,versal-vc-p-a2197", "xlnx,versal";
        #address-cells = <0x2>;
        #size-cells = <0x2>;
        model = "Xilinx Versal A2197 Processor board revA";


        /* test comment */
        cpus: cpus {
                #address-cells = <0x1>;
                #size-cells = <0x0>;
                #cpus-mask-cells = <0x1>;
                compatible = "cpus,cluster";

                cpu@0 {
                        compatible = "arm,cortex-a72", "arm,armv8";
                        device_type = "cpu";
                        enable-method = "psci";
                        operating-points-v2 = <0x1>;
                        reg = <0x0>;
                        cpu-idle-states = <0x2>;
                        clocks = <0x3 0x4d>;
                };

                cpu@1 {
                        compatible = "arm,cortex-a72", "arm,armv8";
                        device_type = "cpu";
                        enable-method = "psci";
                        operating-points-v2 = <0x1>;
                        reg = <0x1>;
                        cpu-idle-states = <0x2>;
                };

                idle-states {
                        entry-method = "psci";

                        cpu-sleep-0 {
                                compatible = "arm,idle-state";
                                arm,psci-suspend-param = <0x40000000>;
                                local-timer-stop;
                                entry-latency-us = <0x12c>;
                                exit-latency-us = <0x258>;
                                min-residency-us = <0x2710>;
                                phandle = <0x2>;
                        };
                };
        };

        amba: amba {
                compatible = "simple-bus";
                #address-cells = <0x2>;
                #size-cells = <0x2>;
                phandle = <0xbeef>;
                ranges;

                /* Proxy Interrupt Controller */
                imux: interrupt-multiplex {
                        compatible = "interrupt-multiplex";
                        #address-cells = <0x0>;
                        #interrupt-cells = <3>;
                        /* copy all attributes from child to parent */
                        interrupt-map-pass-thru = <0xffffffff 0xffffffff 0xffffffff>;
                        /* mask all child bits to always match the first 0x0 entries */
                        interrupt-map-mask = <0x0 0x0 0x0>;
                        /* 1:1 mapping of all interrupts to gic_a72 and gic_r5 */
                        /* child address cells, child interrupt cells, parent, parent interrupt cells */
                        interrupt-map = <0x0 0x0 0x0 &gic_a72 0x0 0x0 0x0>,
                                        <0x0 0x0 0x0 &gic_r5 0x0 0x0 0x0>;
                };

        };

        amba_apu: amba_apu {
                compatible = "simple-bus";
                #address-cells = <0x2>;
                #size-cells = <0x2>;
                ranges;

                gic_a72: interrupt-controller@f9000000 {
                        compatible = "arm,gic-v3";
                        #interrupt-cells = <0x3>;
                        #address-cells = <0x2>;
                        #size-cells = <0x2>;
                        ranges;
                        reg = <0x0 0xf9000000 0x0 0x80000 0x0 0xf9080000 0x0 0x80000>;
                        interrupt-controller;
                        interrupt-parent = <&gic_a72>;
                        interrupts = <0x1 0x9 0x4>;
                        num_cpus = <0x2>;
                        num_interrupts = <0x60>;
                        phandle = <0x5>;

                        gic-its@f9020000 {
                                compatible = "arm,gic-v3-its";
                                msi-controller;
                                msi-cells = <0x1>;
                                reg = <0x0 0xf9020000 0x0 0x20000>;
                                phandle = <0x1b>;
                        };
                };

                iommu: smmu@fd800000 {
                    compatible = "arm,mmu-500";
                    status = "okay";
                    reg = <0x0 0xfd800000 0x0 0x40000>;
                    stream-match-mask = <0x7c00>;
                    #iommu-cells = <0x1>;
                    #global-interrupts = <0x1>;
                    interrupt-parent = <&gic_a72>;
                    interrupts = <0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4>;
                };

                timer {
                     compatible = "arm,armv8-timer";
                     interrupt-parent = <&gic_a72>;
                     interrupts = <0x1 0xd 0x4 0x1 0xe 0x4 0x1 0xb 0x4 0x1 0xa 0x4>;
                };
        };


        domains {
                #address-cells = <0x2>;
                #size-cells = <0x2>;

                openamp_r5 {
                        compatible = "openamp,domain-v1";
                        #address-cells = <0x2>;
                        #size-cells = <0x2>;
                        /*
                         * 1:1 map, it should match the memory regions
                         * specified under access below.
                         *
                         * It is in the form:
                         * memory = <address size address size ...>
                         */
                        memory = <0x0 0x0 0x0 0x8000000>;
                        /*
                         * cpus specifies on which CPUs this domain runs
                         * on
                         *
                         * link to cluster | cpus-mask | execution-mode
                         *
                         * execution mode for ARM-R CPUs:
                         * bit 30: lockstep (lockstep enabled == 1)
                         * bit 31: secure mode / normal mode (secure mode == 1)
                         */
                        /* cpus = <&cpus_r5 0x2 0x80000000>, <&cpus 0x3 0x80000000>; */
                        cpus = <&cpus_r5 0x2 0x80000000>;
                        /*
                         * Access specifies which resources this domain
                         * has access to.
                         *
                         * Link to resource | flags
                         *
                         * The "flags" field is mapping specific
                         *
                         * For memory, reserved-memory, and sram:
                         *   bit 0: 0/1: RO/RW
                         *
                         * Other cases: unused
                         *
                         * In this example we are assigning:
                         * - memory range 0x0-0x8000000 RW
                         * - tcm RW
                         * - ethernet card at 0xff0c0000
                         */
                        access = <&memory_r5 0x1>, <&tcm 0x1>, <&ethernet0 0x0>;
                        /*access = <&tcm 0x1>;*/
                };
        };

        memory: memory@00000000 {
                device_type = "memory";
                reg = <0x0 0x0 0x0 0x80000000>;
        };
};
""")

    return "/tmp/tester.dts"

def setup_system_device_tree( outdir ):
    with open( outdir + "/sdt-tester.dts", "w") as w:
            w.write("""\
/*
 * Copyright (c) 2019,2020 Xilinx Inc. All rights reserved.
 *
 * Author:
 *       Bruce Ashfield <bruce.ashfield@xilinx.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/dts-v1/;

/ {
        compatible = "xlnx,versal-vc-p-a2197-00-revA", "xlnx,versal-vc-p-a2197-00", "xlnx,versal-vc-p-a2197", "xlnx,versal";
        #address-cells = <0x2>;
        #size-cells = <0x2>;
        model = "Xilinx Versal A2197 Processor board revA";


        /* test comment */
        cpus: cpus {
                #address-cells = <0x1>;
                #size-cells = <0x0>;
                #cpus-mask-cells = <0x1>;
                compatible = "cpus,cluster";
                no-access = <0x1>;

                cpu@0 {
                        compatible = "arm,cortex-a72", "arm,armv8";
                        device_type = "cpu";
                        enable-method = "psci";
                        operating-points-v2 = <0x1>;
                        reg = <0x0>;
                        cpu-idle-states = <0x2>;
                        clocks = <0x3 0x4d>;
                };

                cpu@1 {
                        compatible = "arm,cortex-a72", "arm,armv8";
                        device_type = "cpu";
                        enable-method = "psci";
                        operating-points-v2 = <0x1>;
                        reg = <0x1>;
                        cpu-idle-states = <0x2>;
                };

                idle-states {
                        entry-method = "psci";

                        cpu-sleep-0 {
                                compatible = "arm,idle-state";
                                arm,psci-suspend-param = <0x40000000>;
                                local-timer-stop;
                                entry-latency-us = <0x12c>;
                                exit-latency-us = <0x258>;
                                min-residency-us = <0x2710>;
                                phandle = <0x2>;
                        };
                };
        };

        amba: amba {
                compatible = "simple-bus";
                #address-cells = <0x2>;
                #size-cells = <0x2>;
                phandle = <0xbeef>;
                ranges;

                /* Proxy Interrupt Controller */
                imux: interrupt-multiplex {
                        compatible = "interrupt-multiplex";
                        #address-cells = <0x0>;
                        #interrupt-cells = <3>;
                        /* copy all attributes from child to parent */
                        interrupt-map-pass-thru = <0xffffffff 0xffffffff 0xffffffff>;
                        /* mask all child bits to always match the first 0x0 entries */
                        interrupt-map-mask = <0x0 0x0 0x0>;
                        /* 1:1 mapping of all interrupts to gic_a72 and gic_r5 */
                        /* child address cells, child interrupt cells, parent, parent interrupt cells */
                        interrupt-map = <0x0 0x0 0x0 &gic_a72 0x0 0x0 0x0>,
                                        <0x0 0x0 0x0 &gic_r5 0x0 0x0 0x0>;
                };

                ethernet0: ethernet@ff0c0000 {
                        compatible = "cdns,versal-gem";
                        status = "okay";
                        reg = <0x0 0xff0c0000 0x0 0x1000>;
                        interrupts = <0x0 0x38 0x4 0x0 0x38 0x4>;
                        clock-names = "pclk", "hclk", "tx_clk", "rx_clk", "tsu_clk";
                        #stream-id-cells = <0x1>;
                        #address-cells = <0x1>;
                        #size-cells = <0x0>;
                        iommus = <&iommu 0x234>;
                        phy-handle = <0x9>;
                        phy-mode = "rgmii-id";
                        clocks = <0x3 0x52 0x3 0x58 0x3 0x31 0x3 0x30 0x3 0x2b>;
                        power-domains = <0x7 0x18224019>;
                        phandle = <0xb>;

                        phy@1 {
                                reg = <0x1>;
                                ti,rx-internal-delay = <0xb>;
                                ti,tx-internal-delay = <0xa>;
                                ti,fifo-depth = <0x1>;
                                ti,dp83867-rxctrl-strap-quirk;
                                phandle = <0x9>;
                        };

                        phy@2 {
                                reg = <0x2>;
                                ti,rx-internal-delay = <0xb>;
                                ti,tx-internal-delay = <0xa>;
                                ti,fifo-depth = <0x1>;
                                ti,dp83867-rxctrl-strap-quirk;
                                phandle = <0xa>;
                        };
                };

                ethernet@ff0d0000 {
                        compatible = "cdns,versal-gem";
                        status = "okay";
                        reg = <0x0 0xff0d0000 0x0 0x1000>;
                        interrupts = <0x0 0x3a 0x4 0x0 0x3a 0x4>;
                        clock-names = "pclk", "hclk", "tx_clk", "rx_clk", "tsu_clk";
                        #stream-id-cells = <0x1>;
                        #address-cells = <0x1>;
                        #size-cells = <0x0>;
                        iommus = <&iommu 0x235>;
                        phy-handle = <0xa>;
                        phy-mode = "rgmii-id";
                        clocks = <0x3 0x52 0x3 0x59 0x3 0x33 0x3 0x32 0x3 0x2b>;
                        power-domains = <0x7 0x1822401a>;
                        phandle = <0xc>;
                };
        };

        anode_to_delete {
            compatible = "i should be deleted";
        };

        amba_apu: amba_apu {
                compatible = "simple-bus";
                #address-cells = <0x2>;
                #size-cells = <0x2>;
                ranges;

                gic_a72: interrupt-controller@f9000000 {
                        compatible = "arm,gic-v3";
                        #interrupt-cells = <0x3>;
                        #address-cells = <0x2>;
                        #size-cells = <0x2>;
                        ranges;
                        reg = <0x0 0xf9000000 0x0 0x80000 0x0 0xf9080000 0x0 0x80000>;
                        interrupt-controller;
                        interrupt-parent = <&gic_a72>;
                        interrupts = <0x1 0x9 0x4>;
                        num_cpus = <0x2>;
                        num_interrupts = <0x60>;
                        phandle = <0x5>;

                        gic-its@f9020000 {
                                compatible = "arm,gic-v3-its";
                                msi-controller;
                                msi-cells = <0x1>;
                                reg = <0x0 0xf9020000 0x0 0x20000>;
                                phandle = <0x1b>;
                        };
                };

                iommu: smmu@fd800000 {
                    compatible = "arm,mmu-500";
                    status = "okay";
                    reg = <0x0 0xfd800000 0x0 0x40000>;
                    stream-match-mask = <0x7c00>;
                    #iommu-cells = <0x1>;
                    #global-interrupts = <0x1>;
                    interrupt-parent = <&gic_a72>;
                    interrupts = <0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4 0x0 0x6b 0x4>;
                };

                timer {
                     compatible = "arm,armv8-timer";
                     interrupt-parent = <&gic_a72>;
                     interrupts = <0x1 0xd 0x4 0x1 0xe 0x4 0x1 0xb 0x4 0x1 0xa 0x4>;
                };

                nested-node {
                    compatible = "delete-me";
                    nested-node-child1 {
                         compatible = "delete-me2";
                    };
                };
        };

        domains {
                #address-cells = <0x2>;
                #size-cells = <0x2>;

                openamp_r5 {
                        compatible = "openamp,domain-v1";
                        #address-cells = <0x2>;
                        #size-cells = <0x2>;
                        /*
                         * 1:1 map, it should match the memory regions
                         * specified under access below.
                         *
                         * It is in the form:
                         * memory = <address size address size ...>
                         */
                        memory = <0x0 0x0 0x0 0x8000000>;
                        /*
                         * cpus specifies on which CPUs this domain runs
                         * on
                         *
                         * link to cluster | cpus-mask | execution-mode
                         *
                         * execution mode for ARM-R CPUs:
                         * bit 30: lockstep (lockstep enabled == 1)
                         * bit 31: secure mode / normal mode (secure mode == 1)
                         */
                        /* cpus = <&cpus_r5 0x2 0x80000000>, <&cpus 0x3 0x80000000>; */
                        cpus = <&cpus_r5 0x2 0x80000000>;
                        /*
                         * Access specifies which resources this domain
                         * has access to.
                         *
                         * Link to resource | flags
                         *
                         * The "flags" field is mapping specific
                         *
                         * For memory, reserved-memory, and sram:
                         *   bit 0: 0/1: RO/RW
                         *
                         * Other cases: unused
                         *
                         * In this example we are assigning:
                         * - memory range 0x0-0x8000000 RW
                         * - tcm RW
                         * - ethernet card at 0xff0c0000
                         */
                        access = <&memory_r5 0x1>, <&tcm 0x1>, <&ethernet0 0x0>;
                        /*access = <&tcm 0x1>;*/
                };
        };

        memory: memory@00000000 {
                device_type = "memory";
                reg = <0x0 0x0 0x0 0x80000000>;
        };
};
""")

    return "/tmp/sdt-tester.dts"



def setup_fdt( device_tree, outdir ):
    Lopper.dt_compile( device_tree, "", "", True, outdir )
    return libfdt.Fdt(open( device_tree + ".dtb", mode='rb').read())


def test_failed( error_msg ):
    print( "[TEST FAILED]: " + error_msg )
    sys.exit(1)

def test_passed( msg ):
    print( "[TEST PASSED]: " + msg )

def test_pattern_count( fpp, pattern ):
    count = 0
    with open(fpp) as fp:
        for line in fp:
            if re.search( pattern, line ):
                count += 1

    return count

def tree_sanity_test( fdt, verbose=0 ):
    # test1: simple tree walking routine
    print( "[TEST]: start: tree walk" )
    #  - test:
    #      - that we don't assert
    #      - that we have the right number of nodes
    walker = LopperTree( fdt )

    fpp = tempfile.NamedTemporaryFile( delete=True )
    fpw = open( fpp.name, 'w+')
    for n in walker:
        print( "\nnode: %s:%s [%s] parent: %s children: %s depth: %s" % (n.name, n.number,
                                                                         hex(n.phandle), n.parent,
                                                                         n.children, n.depth), file=fpw)
        for prop in n:
            print( "    property: %s %s" % (prop.name, prop.value), file=fpw)
            print( "    raw: %s" % (n[prop.name]), file=fpw )

    # we should have: <x> "node:" prints
    with open(fpp.name) as fp:
        node_count = 0
        for line in fp:
            if re.search( "node:", line ):
                node_count += 1

    if node_count != 15:
        test_failed( "node count is incorrect" )
    else:
        test_passed( "end: node walk passed\n" )

    fpw.close()

    # test2: tree print
    print( "[TEST]: start: tree print" )
    fpp = tempfile.NamedTemporaryFile( delete=True )

    printer = LopperTreePrinter( fdt, True, fpp.name )
    printer.exec()

    with open(fpp.name) as fp:
        node_count = 0
        for line in fp:
            if re.search( "{", line ):
                node_count += 1

    if node_count != 16:
        test_failed( "node count is incorrect (%s expected %s)" % (node_count, 15) )
    else:
        test_passed( "end: tree print passed\n")
    fpw.close()

    # test3: node manipulations/access
    print( "[TEST]: start: node manipulations" )
    try:
        n = printer['/amba']
        if verbose:
            print( "    node access via '/amba' found: %s, %s" % (n, [n]) )
        test_passed( "node access by path" )
    except:
        test_failed( "node access by path failed" )

    try:
        n2 = printer[n.number]
        if verbose:
            print( "    node access by number '%s' found: %s" % (n.number,[n2]) )
        test_passed( "node access by number" )
    except:
        test_failed( "node access by path failed" )

    # write it back, as a test only
    try:
        printer['/amba'] = n
        test_passed( "node reassignment by name" )
    except:
        test_failed( "node reassignment by name" )

    try:
        pp = n['compatible']
        if verbose:
            print( "    property access (compatible): name: %s value: %s string: %s" % (pp.name, pp.value, pp) )
        test_passed( "property access by name" )
    except:
        test_failed( "property access by name" )

    # phandle tests
    try:
        n = printer.pnode( n.phandle )
        if verbose:
            print( "    node access via phandle %s: %s" % (hex(n.phandle), n) )
        test_passed( "node access via phandle" )
    except:
        test_failed( "node access via phandle" )

    print( "[TEST]: end: node manipulations\n" )

    # iteration tests
    print( "[TEST]: start: custom node lists" )

    fpp = tempfile.NamedTemporaryFile( delete=True )

    printer.reset( fpp.name)

    printer.__new_iteration__ = True
    printer.__current_node__ = "/amba_apu"
    printer.exec()

    c = test_pattern_count( fpp.name, "{" )
    if c == 5:
        test_passed( "custom node print" )
    else:
        test_failed( "custom node print (%s vs %s)" % (c,5) )

    # shouldn't break anything: random re-resolves
    printer.resolve()
    fpp.close()

    fpp = tempfile.NamedTemporaryFile( delete=False )
    fpw = open( fpp.name, 'w+')

    print( "\n[SUB TEST]: full node walk after custom node list" )
    for p in printer:
        print( "        node: %s" % p, file=fpw )

    fpw.close()
    c = test_pattern_count( fpp.name, ".*node:" )
    if c == 16:
        test_passed( "full walk, after restricted walk" )
    else:
        test_failed( "full walk, after restricted walk" )

    print( "[SUB TEST]: end full node walk after custom node list\n" )

    os.unlink( fpw.name )


    print( "[SUB TEST]: subtree walk" )
    # this should only walk the amba sub-tree
    printer.__current_node__ = "/amba"
    count = 0
    for p in printer:
        if verbose:
            print( "    /amba restricted test: node: %s" % p )
        count += 1

    if count == 2:
        test_passed( "subtree walk" )
    else:
        test_failed( "subtree walk (%s vs %s)" % (count,2))
    print( "[SUB TEST]: end subtree walk\n" )

    print( "[SUB TEST]: start node -> end of tree walk" )
    printer.reset()

    # the difference here, is that this starts at amba and walks ALL the way to
    # the end of the tree.
    printer.__start_node__ = "/amba"
    count = 0
    for p in printer:
        if verbose:
            print( "       starting node test: node: %s" % p )
        count += 1

    if count == 10:
        test_passed( "start -> end walk" )
    else:
        test_failed( "start -> end walk (%s vs %s)" % (count,10))

    print( "[SUB TEST]: start node -> end of tree walk\n" )

    # debug level bump up.
    printer.__dbg__ = 3

    # subnode routine tests
    print( "[TEST]: start: subnode calls (both should be the same)" )
    kiddies = printer.subnodes( printer.__nodes__['/amba'] )
    subnode_count = 0
    if verbose:
        print( "amba subnodes: %s" % kiddies )
    for k in kiddies:
        if verbose:
            print( "    node: %s" % k.abs_path )
        subnode_count += 1

    subnode_count2 = 0
    kiddies2 = printer['/amba'].subnodes()
    if verbose:
        print( "abma subnodes type 2: %s" % kiddies2 )
    for k in kiddies2:
        if verbose:
            print( "    node2: %s" % k.abs_path )
        subnode_count2 += 1

    if subnode_count == subnode_count2:
        test_passed( "subnode count" )
    else:
        test_failed( "subnode count (%s vs %s)" % (subnode_count,subnode_count2))

    subnodecount = 0
    kiddies = printer.subnodes( printer['/'] )
    if verbose:
        print( "/ subnodes: %s" % kiddies )
    for k in kiddies:
        if verbose:
            print( "    node: %s" % k.abs_path )
        subnodecount += 1

    if subnodecount == 16:
        test_passed( "full tree subnode" )
    else:
        test_failed( "full tree subnode (%s vs %s)" % (subnodecount,16))

    subnodecount = 0
    kiddies = printer.subnodes( printer['/'], ".*amba.*" )
    if verbose:
        print( "/ subnodes matching regex '.*amba.*': %s" % kiddies )
    for k in kiddies:
        if verbose:
            print( "    node: %s" % k.abs_path )
        subnodecount += 1

    if subnodecount == 7:
        test_passed( "regex subnode" )
    else:
        test_failed( "regex subnode (%s vs %s)" % (subnodecount,7))

    print( "[TEST]: end: subnode calls\n" )

    print( "[TEST]: start: resolve test" )
    refcount = 0
    root_found = False
    amba_found = False
    all_refs = printer['/amba/interrupt-multiplex'].resolve_all_refs( printer.fdt )
    for a in all_refs:
        if a.abs_path == "/":
            root_found = True
        if a.abs_path == "/amba":
            amba_found = True

        if verbose:
            print( "/amba/interrupt-multiplex ref: %s" % a.abs_path )

        refcount += 1

    if refcount == 5:
        test_passed( "resolve" )
    else:
        test_failed( "resolve (%s vs %s)" % (refcount,5))

    if root_found and amba_found:
        test_passed( "parent nodes found" )
    else:
        test_failed( "parent nodes found (%s,%s)" % (root_found,amba_found) )

    print( "[TEST]: end: resolve test" )

    print( "[TEST]: start: node access tests and __str__ routine" )
    printer.__dbg__ = 0
    if verbose:
        print( "amba node: %s" % printer.__nodes__['/amba'] )
        print( "amba node number: %s " % int(printer.__nodes__['/amba']))
    if "/amba" == str(printer.__nodes__['/amba']):
        test_passed( "__str__" )
    else:
        test_failed( "__str__" )

    printer.__dbg__ = 3

    if verbose:
        print( "amba node raw: %s" % printer.__nodes__['/amba'] )
    if re.search( "<lopper_tree.LopperNode.*", str(printer.__nodes__['/amba']) ):
        test_passed( "__str__ raw" )
    else:
        test_failed( "__str__ raw" )

    printer.__dbg__ = 0

    if verbose:
        print( "type: %s" % type(printer.__nodes__['/amba']) )
    if isinstance( printer.__nodes__['/amba'], LopperNode ):
        test_passed( "instance type" )
    else:
        test_failed( "instance type" )

    print( "[TEST]: end: node access tests and __str__ routine\n" )

    print( "[TEST]: start: node comparison tests" )
    if verbose:
        print( "Comparing '/amba' and '/amba_apu'" )
    if printer.__nodes__['/amba'] == printer.__nodes__['/amba_apu']:
        test_failed( "equality test" )
    else:
        test_passed( "equality test" )
    print( "[TEST]: end: node comparison tests" )

    print( "[TEST]: start: node regex find test" )

    if verbose:
        print( "Searching for /amba/.*" )

    matches = printer.nodes( "/amba/.*" )
    count = 0
    multiplex = False
    for m in matches:
        count += 1
        if m.name == "interrupt-multiplex":
            multiplex = True
        if verbose:
            print( " match: %s [%s]" % (m.abs_path, m) )

    if count == 1 and multiplex:
        test_passed( "regex node match" )
    else:
        test_failed( "regex node match" )

    if verbose:
        print( "searching for /amba.*" )
    count = 0
    matches = printer.nodes( "/amba.*" )
    for m in matches:
        count += 1
        if m.name == "interrupt-multiplex":
            multiplex = True
        if verbose:
            print( "    match: %s [%s]" % (m.abs_path, m) )

    if count == 7 and multiplex:
        test_passed( "regex node match 2" )
    else:
        test_failed( "regex node match 2" )

    if verbose:
        print( "exact node match: /amba" )
    matches = printer.nodes( "/amba" )
    for m in matches:
        if verbose:
            print( "    match: %s" % m.abs_path )
        pass

    amba = matches[0]
    if len(matches) == 1 and amba.abs_path == "/amba":
        test_passed( "exact node match" )
    else:
        test_failed( "exact node match" )
    print( "[TEST]: end: node regex find test\n" )

    print( "[TEST]: start: property regex find test" )
    p = amba.props( 'compat.*' )
    p = p[0]
    if verbose:
        print( "prop type is: %s" % type(p) )
        print( "amba p0: %s" % p.value[0] )
        print( "amba p1: %s" % p )
    if isinstance( p, LopperProp ):
        test_passed( "prop match type" )
    else:
        test_failed( "prop match type" )
    if p.value[0] == "simple-bus":
        test_passed( "prop value" )
    else:
        test_failed( "prop value (%s vs %s)" % ( p.value[0], "simple-bus" ) )
    if str(p) == "compatible = \"simple-bus\";":
        test_passed( "prop str" )
    else:
        test_failed( "prop str" )
    print( "[TEST]: end: property regex find test\n" )

    print( "[TEST]: start: property assign test" )
    p.value = "testing 1.2.3"
    if verbose:
        print( "amba p2: %s" % p.value[0] )
        print( "amba p3: %s" % str(p) )

    if p.value[0] == "testing 1.2.3":
        test_passed( "prop value re-assign" )
    else:
        test_failed( "prop value re-assign (%s vs %s)" % (p.value[0],"testing 1.2.3"))

    if str(p) == "compatible = \"testing 1.2.3\";":
        test_passed( "prop re-assign, resolve" )
    else:
        test_failed( "prop re-assign, resolve (%s vs %s)" % (str(p), "compatible = \"testing 1.2.3\";" ))
    print( "[TEST]: end: property assign test\n" )

    print( "[TEST]: start: tree manipulation tests" )
    new_node = LopperNode( -1, "/amba/bruce" )
    if verbose:
        print( "    new node name: %s" % new_node.name )
        print( "    new node refcount: %s" % new_node.ref )
    new_node.ref = 2
    new_node.ref = 1
    if verbose:
        print( "    new node refcount is: %s" % new_node.ref )

    if new_node.ref == 3:
        test_passed( "node ref" )
    else:
        test_failed( "node ref (%s vs %s)" % (new_node.ref,3))

    if verbose:
        print( "\n" )
        print( "creating new property for new node .... " )

    new_property = LopperProp( "foobar", -1, new_node, [ "testingfoo" ] )

    if verbose:
        print( "Property add to node: ")

    # new_node.add( new_property )
    new_node + new_property
    # printer.add( new_node )
    printer + new_node

    # confirm the node details are the same:
    if verbose:
        print( "new_node path: %s" % new_node.abs_path )
        print( "ref count: %s" % printer[new_node].ref )
    if new_node.abs_path == "/amba/bruce" and new_node.ref == 3:
        test_passed( "node + prop + tree" )
    else:
        test_failed( "node + prop + tree" )

    print( "[TEST]: end: tree manipulation tests\n" )

    print( "[TEST]: start: tree ref count test" )
    refd = printer.refd()
    if verbose:
        print( "referenced nodes: %s" % refd[0].abs_path )

    if len(refd) == 1 and refd[0].abs_path == "/amba/bruce":
        test_passed( "node ref persistence" )
    else:
        test_failed( "node ref persistence" )

    printer.ref( 0 )
    refd = printer.refd()
    if verbose:
        print( "After clear, referenced nodes: %s" % refd )
    if len(refd) == 0:
        test_passed( "node ref reset" )
    else:
        test_failed( "node ref reset" )
    print( "[TEST]: end: tree ref count test\n" )

    print( "[TEST]: start: tree re-resolve test" )
    if verbose:
        print( "======================= re resolving =======================" )
    printer.resolve()
    if verbose:
        print( "======================= resolve done =======================" )

    if verbose:
        for n in printer:
            print( "node: %s" % n.abs_path )

    fpp = tempfile.NamedTemporaryFile( delete=False )

    printer.__dbg__ = 0
    printer.__start_node__ = 0
    printer.reset( fpp.name )
    printer.exec()

    # if we get here, we passed
    test_passed( "tree re-resolve\n" )

    print( "[TEST]: end: tree re-resolve test\n" )

    print( "[TEST]: start: second tree test" )
    print2 = LopperTreePrinter( printer.fdt )
    if verbose:
        for n in print2:
            print( "2node: %s" % n )

        print( "\n" )

    fpp2 = tempfile.NamedTemporaryFile( delete=False )
    print2.reset( fpp2.name )
    print2.exec()

    if filecmp.cmp( fpp.name, fpp2.name ):
        test_passed( "two tree print" )
    else:
        test_failed( "two tree print" )

    print( "[TEST]: end: second tree test\n" )

    print( "[TEST]: start: node persistence test" )
    latched_state = new_node.__nstate__
    if verbose:
        print( "new node's state is now: %s" % new_node.__nstate__ )

    if new_node.__nstate__ == "resolved":
        test_passed( "node persistence test" )
    else:
        test_failed( "node persistence test" )

    print( "[TEST]: end: node persistence test\n" )

    print( "[TEST]: start: second property test" )
    new_property2 = LopperProp( "number2", -1, new_node, [ "i am 2" ] )

    # type1: can we just add to the node and sync it ?
    new_node + new_property2
    if verbose:
        print( "syncing new property" )
    printer.sync()
    if verbose:
        print( "end syncing new property" )
    # the above works when nodes are fully reused

    # type2: or can we fetch it out of the tree, assign and sync
    # printer[new_node] + new_property2
    # printer.sync()
    # the above works

    # is a double node add an error ?
    printer + new_node
    printer.sync()
    # end double add

    if verbose:
        print( "writing to: /tmp/tester-output.dts" )
    Lopper.write_fdt( printer.fdt, "/tmp/tester-output.dts", None, True, verbose, True )

    # remove the 2nd property, re-write
    if verbose:
        print( "writing to: /tmp/tester-output2.dts (with one less property" )
    new_node - new_property2
    printer.sync()

    Lopper.write_fdt( printer.fdt, "/tmp/tester-output2.dts", None, True, verbose, True )

    if filecmp.cmp( "/tmp/tester-output.dts", "/tmp/tester-output2.dts", False ):
        test_failed( "node remove write should have differed" )
    else:
        test_passed( "node remove write" )

    test_passed( "second property test" )
    print( "[TEST]: end: second property test\n" )

    print( "[TEST]: start: second tree test, node deep copy" )

    tree2 = LopperTreePrinter( fdt, True )
    # new_node2 = LopperNode()
    # invokes a deep copy on the node
    new_node2 = new_node()

    if verbose:
        print( "node2: %s" % new_node2.abs_path )
    #new_node2 = new_node2(new_node)
    if verbose:
        print( "node2: %s" % new_node2.abs_path )

    # the property objects, should be different, since these are copies
    if verbose:
        print( "node1 props: %s" % new_node.__props__ )
        print( "node2 props: %s" % new_node2.__props__ )

    if new_node != new_node2:
        test_failed( "copied nodes should be equal" )

    if new_node.__props__ == new_node2.__props__:
        test_failed( "copied properties should not be equal" )

    # not required, but could re-add to make sure they don't harm anything
    # new_node2.resolve( tree2.fdt )
    # new_node2.sync( tree2.fdt )

    if verbose:
        tree2.__dbg__ = 3

    tree2 + new_node2
    Lopper.write_fdt( tree2.fdt, "/tmp/tester-output-tree2.dts", None, True, verbose, True )

    print( "[TEST]: end: second tree test, node deep copy\n" )

    print( "[TEST]: start: tree test, node remove" )

    printer = printer - new_node
    #printer.delete( new_node )

    Lopper.write_fdt( printer.fdt, "/tmp/tester-output-node-removed.dts", None, True, verbose, True )
    print( "[TEST]: end: tree test, node remove" )

    if filecmp.cmp( "/tmp/tester-output-node-removed.dts", "/tmp/tester-output-tree2.dts", False ):
        test_failed( "node remove write should have differed" )
    else:
        test_passed( "node remove differed" )


def lops_sanity_test( device_tree, lop_file, verbose ):
    device_tree.setup( dt, [lop_file], "", "", True )
    device_tree.perform_lops()

    print( "[TEST]: writing to %s" % (device_tree.output_file))
    Lopper.write_fdt( device_tree.FDT, device_tree.output_file, device_tree, True, device_tree.verbose, device_tree.pretty )

    print( "\n[TEST]: check lopper operations on: %s" % (device_tree.output_file))
    c = test_pattern_count( device_tree.output_file, "anode_to_delete" )
    if c != 0:
        test_failed( "node deletion failed" )
    else:
        test_passed( "node deletion" )

    c = test_pattern_count( device_tree.output_file, "cpus_a72" )
    if c != 1:
        test_failed( "node rename failed" )
    else:
        test_passed( "node rename" )

    c = test_pattern_count( device_tree.output_file, "no-access" )
    if c != 0:
        test_failed( "property remove failed" )
    else:
        test_passed( "property remove" )

    c = test_pattern_count( device_tree.output_file, "nested-node" )
    if c != 0:
        test_failed( "nested node deletion failed" )
    else:
        test_passed( "nested node deletion" )

    c = test_pattern_count( device_tree.output_file, "zynqmp-rpu" )
    if c == 1:
        c = test_pattern_count( device_tree.output_file, "__cpu__" )
        if c == 1:
            test_passed( "node add" )
        else:
            test_failed( "node add" )
    else:
        test_failed( "node add" )

    c = test_pattern_count( device_tree.output_file, "lopper-mboxes" )
    if c == 1:
        test_passed( "new node property modify" )
    else:
        test_failed( "new node property modify" )

    c = test_pattern_count( device_tree.output_file, "model = \"this is a test\"" )
    if c == 1:
        test_passed( "root property modify" )
    else:
        test_failed( "root property modify" )

    c = test_pattern_count( "/tmp/openamp-test.dts", "zynqmp-rpu" )
    if c == 1:
        test_passed( "node selective output" )
    else:
        test_failed( "node selective output" )

    c = test_pattern_count( "/tmp/linux-amba.dts", ".*amba.*{" )
    if c == 2:
        test_passed( "node regex output" )
    else:
        test_failed( "node regex output" )

    c = test_pattern_count( device_tree.output_file, "pnode-id =" )
    if c == 1:
        test_passed( "property add" )
    else:
        test_failed( "property add" )

    c = test_pattern_count( device_tree.output_file, "lopper-id-regex" )
    if c == 2:
        test_passed( "property via regex add" )
    else:
        test_failed( "property via regex add" )

    device_tree.cleanup()

def assists_sanity_test( device_tree, lop_file, verbose ):
    device_tree.setup( dt, [lop_file], "", [ "domain-access.py" ], True )

    print( "[TEST]: running assist against tree" )
    device_tree.perform_lops()

    print( "[TEST]: writing resulting FDT to %s" % device_tree.output_file )
    Lopper.write_fdt( device_tree.FDT, device_tree.output_file, device_tree, True, device_tree.verbose, device_tree.pretty )

    device_tree.cleanup()

def usage():
    prog = os.path.basename(sys.argv[0])
    print('Usage: %s [OPTION] ...' % prog)
    print('  -v, --verbose       enable verbose/debug processing (specify more than once for more verbosity)')
    print('  -t, --tree          run lopper tree tests' )
    print('  -l, --lops          run lop tests' )
    print('  -a, --assists       run assist tests' )
    print('    , --werror        treat warnings as errors' )
    print('  -h, --help          display this help and exit')
    print('')

def main():
    global verbose
    global force
    global werror
    global outdir
    global lops
    global tree
    global assists

    verbose = 0
    force = False
    werror = False
    outdir="/tmp/"
    tree = False
    lops = False
    assists = False
    try:
        opts, args = getopt.getopt(sys.argv[1:], "avtlh", [ "assists", "tree", "lops", "werror","verbose", "help"])
    except getopt.GetoptError as err:
        print('%s' % str(err))
        usage()
        sys.exit(2)

    if opts == [] and args == []:
        usage()
        sys.exit(1)

    for o, a in opts:
        if o in ('-v', "--verbose"):
            verbose = verbose + 1
        elif o in ('-f', "--force"):
            force = True
        elif o in ('-h', '--help'):
            usage()
            sys.exit(0)
        elif o in ('-O', '--outdir'):
            outdir = a
        elif o in ('--werror'):
            werror=True
        elif o in ( '-l','--lops'):
            lops=True
        elif o in ( '-t','--tree'):
            tree=True
        elif o in ( '-a','--assists'):
            assists=True
        elif o in ('--version'):
            print( "%s" % LOPPER_VERSION )
            sys.exit(0)
        else:
            assert False, "unhandled option"


if __name__ == "__main__":

    main()

    if tree:
        dt = setup_device_tree( outdir )
        fdt = setup_fdt( dt, outdir )
        tree_sanity_test( fdt, verbose )

    if lops:
        dt = setup_system_device_tree( outdir )
        lop_file = setup_lops( outdir )

        device_tree = LopperSDT( dt )

        device_tree.dryrun = False
        device_tree.verbose = 0
        device_tree.werror = werror
        device_tree.output_file = outdir + "/sdt-output.dts"
        device_tree.cleanup_flag = True
        device_tree.save_temps = False
        device_tree.pretty = True
        device_tree.outdir = outdir

        lops_sanity_test( device_tree, lop_file, verbose )

    if assists:
        dt = setup_system_device_tree( outdir )
        lop_file = setup_assist_lops( outdir )

        device_tree = LopperSDT( dt )

        device_tree.dryrun = False
        device_tree.verbose = 0
        device_tree.werror = werror
        device_tree.output_file = outdir + "/assist-output.dts"
        device_tree.cleanup_flag = True
        device_tree.save_temps = False
        device_tree.pretty = True
        device_tree.outdir = outdir

        assists_sanity_test( device_tree, lop_file, verbose )
