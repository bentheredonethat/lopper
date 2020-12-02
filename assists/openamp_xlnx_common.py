class SOC_TYPE:
    UNINITIALIZED = -1
    VERSAL = 0
    ZYNQMP = 1
    ZYNQ = 2

class IPC_ROLE:
    UNINITIALIZED = -1
    HOST = 0
    REMOTE = 1

host_template="""
#ifndef OPENAMP_LOPPER_INFO_H_
#define OPENAMP_LOPPER_INFO_H_

#define IPI_CHN_BITMASK     {IPI_CHN_BITMASK}
#define RSC_MEM_PA          {RSC_MEM_PA}
#define RSC_MEM_SIZE        {RSC_MEM_SIZE}
#define VRING_MEM_PA        {VRING_MEM_PA}
#define VRING_MEM_SIZE      {VRING_MEM_SIZE}
#define SHARED_BUF_PA       {SHARED_BUF_PA}
#define SHARED_BUF_SIZE     {SHARED_BUF_SIZE}
#define IPI_DEV_NAME        {IPI_DEV_NAME}
#define SHM_DEV_NAME        {SHM_DEV_NAME}

#endif /* OPENAMP_LOPPER_INFO_H_ */
"""

# for now only do host for rpmsg-userspace
# host is 1, remote is 0
# TODO, add support for remote for rpmsg-user and rpmsg-kernelspace
def write_openamp_header(platform, inputs, role, options):
    if (len(options["args"])) > 0:
        f_name = options["args"][0]
    else:
      print("write_openamp_header: no file name provided.")
      return -1
    try:
        verbose = options['verbose']
    except:
        verbose = 0

    if not inputs:
        print( "[WARNING]: unable to generate openamp_lopper_info.h, no valid inputs" )
        return
    f =  open(f_name,"w")
    template = None
    if platform == SOC_TYPE.VERSAL:
      if IPC_ROLE.HOST == role:
        template = host_template
        
    f.write(template.format(**inputs))
    f.close()
