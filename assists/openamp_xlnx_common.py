import os
import sys
sys.path.append(os.path.dirname(__file__))

class SOC_TYPE:
    UNINITIALIZED = -1
    VERSAL = 0
    ZYNQMP = 1
    ZYNQ = 2

general_template="""
#ifndef OPENAMP_LOPPER_INFO_H_
#define OPENAMP_LOPPER_INFO_H_


#define CHANNEL_0_MEM_VDEV0VRING0_ADDR	{CHANNEL_0_VDEV0VRING0_ADDR}
#define CHANNEL_0_MEM_VDEV0VRING0_RANGE	{CHANNEL_0_VDEV0VRING0_RANGE}
#define CHANNEL_0_MEM_VDEV0VRING1_ADDR	{CHANNEL_0_VDEV0VRING1_ADDR}
#define CHANNEL_0_MEM_VDEV0VRING1_RANGE	{CHANNEL_0_VDEV0VRING1_RANGE}
#define CHANNEL_0_MEM_VDEV0BUFFER_ADDR	{CHANNEL_0_VDEV0BUFFER_ADDR}
#define CHANNEL_0_MEM_VDEV0BUFFER_RANGE	{CHANNEL_0_VDEV0BUFFER_RANGE}
#define CHANNEL_0_MEM_ELFLOAD_ADDR	{CHANNEL_0_ELFLOAD_ADDR}
#define CHANNEL_0_MEM_ELFLOAD_RANGE	{CHANNEL_0_ELFLOAD_RANGE}
#define CHANNEL_0_MEM_RANGE	        {CHANNEL_0_MEM_RANGE}

#define CHANNEL_0_MEM_SHARED_MEM_PA	{CHANNEL_0_SHARED_MEM_PA}
#define CHANNEL_0_MEM_RING_TX           {CHANNEL_0_TX}
#define CHANNEL_0_MEM_RING_RX	        {CHANNEL_0_RX}
#define CHANNEL_0_MEM_SHARED_MEM_SIZE	{CHANNEL_0_SHARED_MEM_SIZE}
#define CHANNEL_0_MEM_SHARED_BUF_OFFSET	{CHANNEL_0_SHARED_BUF_OFFSET}
#define CHANNEL_0_MEM_VRING_MEM_SIZE	{CHANNEL_0_VRING_MEM_SIZE}
#define CHANNEL_0_MEM_RSC_MEM_SIZE	{CHANNEL_0_RSC_MEM_SIZE}
#define CHANNEL_0_MEM_NUM_VRINGS	2
#define CHANNEL_0_MEM_VRING_ALIGN	0x1000
#define CHANNEL_0_MEM_VRING_SIZE	256
#define CHANNEL_0_MEM_NUM_TABLE_ENTRIES	1
#define REMOTE_BUS_NAME                 "generic"
#define MASTER_BUS_NAME                 "platform"

{soc_template}

#endif /* OPENAMP_LOPPER_INFO_H_ */
"""

zynq_template="""
#define DEVICE_MEMORY 0xC06     /* Device memory */
#define STRONG_ORDERED 0xC02    /* Strongly ordered */
#define RESERVED 0x0            /* reserved memory */
#define NORM_NONCACHE 0x11DE2   /* Normal Non-cacheable */
#define REMOTE_SCUGIC_DEV_NAME	"scugic_dev"
#define SCUGIC_PERIPH_BASE	0xF8F00000
#define SCUGIC_DIST_BASE	(SCUGIC_PERIPH_BASE + 0x00001000)
#define ZYNQ_CPU_ID_MASK	0x1UL
/* SGIs */
#define SGI_TO_NOTIFY           15 /* SGI to notify the remote */
#define SGI_NOTIFICATION        14 /* SGI from the remote */
"""

r5_template="""
#define CHANNEL_0_MASTER_IPI_BASE_ADDR	{MASTER_IPI_BASE_ADDR}
#define CHANNEL_0_MASTER_IPI_NAME	{MASTER_IPI_NAME}
#define CHANNEL_0_MASTER_IRQ_VECT_ID    {MASTER_IRQ_VECT_ID}
#define CHANNEL_0_MASTER_CHN_BITMASK    {MASTER_CHN_BITMASK}
#define CHANNEL_0_REMOTE_IPI_BASE_ADDR  {REMOTE_IPI_BASE_ADDR}
#define CHANNEL_0_REMOTE_IPI_NAME       {REMOTE_IPI_NAME}
#define CHANNEL_0_REMOTE_IRQ_VECT_ID    {REMOTE_IRQ_VECT_ID}
#define CHANNEL_0_REMOTE_CHN_BITMASK    {REMOTE_CHN_BITMASK}
"""


# platform determine SoC
# is_kernel_case for SoC's that support user/kernelspace
# inputs is dictionary with inputs for template
def write_openamp_header(platform, is_kernel_case, inputs, options):
    if (len(options["args"])) > 0:
        f_name = options["args"][0]
    else:
        f_name = "openamp_lopper_info.h"
    try:
        verbose = options['verbose']
    except:
        verbose = 0

    if not inputs:
        print( "[WARNING]: unable to generate openamp_lopper_info.h, no valid inputs" )
        return

    f =  open(f_name,"w")
    if platform == SOC_TYPE.ZYNQ:
        inputs["soc_template"] = zynq_template
    else:
        inputs["soc_template"] = r5_template.format(**inputs)
        
    f.write(general_template.format(**inputs))
    f.close()

def generate_openamp_file(carveout_list, options, platform, is_kernel_case, inputs):
    symbol_name = "CHANNEL_0_MEM_"
    current_channel_count = 0 # if == 4 then got complete channel range
    vring_mems = []
    channel_range = 0

    if platform == SOC_TYPE.ZYNQ:
        addr_column = 0
        range_column = 1
    else:
        addr_column = 1
        range_column = 3

    for i in carveout_list:
        if "vdev0buffer" in i[0]:
            current_channel_count += 1
            inputs["CHANNEL_0_VDEV0BUFFER_ADDR"] = i[1][addr_column]
            inputs["CHANNEL_0_VDEV0BUFFER_RANGE"] = i[1][range_column]
            inputs["CHANNEL_0_SHARED_MEM_SIZE"] = i[1][range_column]
            vdev0buffer_range = i[1][range_column]
            channel_range += int(i[1][range_column],16)
        elif "vdev0vring0" in i[0]:
            current_channel_count += 1
            inputs["CHANNEL_0_VDEV0VRING0_ADDR"] = i[1][addr_column]
            inputs["CHANNEL_0_VDEV0VRING0_RANGE"] = i[1][range_column]
            inputs["CHANNEL_0_SHARED_MEM_PA"] =  i[1][range_column]
            if is_kernel_case == False:
                inputs["CHANNEL_0_TX"] = i[1][addr_column]

            channel_range += int(i[1][range_column],16)
            vring_mems.append(i[1][range_column])
        elif "vdev0vring1" in i[0]:
            vring_mems.append(i[1][range_column])
            inputs["CHANNEL_0_VDEV0VRING1_ADDR"] = i[1][addr_column]
            inputs["CHANNEL_0_VDEV0VRING1_RANGE"] = i[1][range_column]
            if is_kernel_case == False:
                inputs["CHANNEL_0_RX"] = i[1][addr_column]

            current_channel_count += 1
            channel_range += int(i[1][range_column],16)
        elif "elfload" in i[0]:
            current_channel_count += 1
            channel_range += int(i[1][range_column],16)
            inputs["CHANNEL_0_ELFLOAD_ADDR"] = i[1][addr_column]
            inputs["CHANNEL_0_ELFLOAD_RANGE"] = i[1][range_column]

        if current_channel_count == 4:
            inputs["CHANNEL_0_MEM_RANGE"] = hex(channel_range)
            current_channel_count += 1 # TODO account for >1 channels
            vring_mems_size_total = 0
            for i in vring_mems:
                vring_mems_size_total += int(i,16)
            inputs["CHANNEL_0_SHARED_BUF_OFFSET"] = hex(vring_mems_size_total)
            inputs["CHANNEL_0_VRING_MEM_SIZE"] = hex(vring_mems_size_total)

            vring_mem_size = 0

    write_openamp_header(platform, is_kernel_case, inputs, options)

def parse_memory_carevouts(sdt, options, remoteproc_node):
    try:
        verbose = options['verbose']
    except:
        verbose = 0

    carveout_list = [] # string representation of mem carveout nodes
    phandle_list = []

    reserved_mem_node = sdt.tree["/reserved-memory"]

    for node in reserved_mem_node.subnodes():
        if node.propval("compatible") != [""]:
            phandle_list.append(node.phandle)
            new_val = []
            for i in node["reg"].value:
                new_val.append(str(hex(i)))
            carveout_list.append( (str(node), new_val ) )

    # output to DT
    remoteproc_node["memory-region"].value = phandle_list
    remoteproc_node.sync ( sdt.FDT )

    return carveout_list

