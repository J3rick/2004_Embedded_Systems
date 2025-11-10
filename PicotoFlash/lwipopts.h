#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Generally you would define your own explicit list of lwIP options
// (see https://www.nongnu.org/lwip/2_1_x/group__lwip__opts.html)
//
// This example uses a common include to avoid repetition
#include "lwipopts_examples_common.h"

#if !NO_SYS
#define TCPIP_THREAD_STACKSIZE 4096
#define DEFAULT_THREAD_STACKSIZE 1024
#define DEFAULT_RAW_RECVMBOX_SIZE 8
#define TCPIP_MBOX_SIZE 8
#define LWIP_TIMEVAL_PRIVATE 0

// not necessary, can be done either way
#define LWIP_TCPIP_CORE_LOCKING_INPUT 1

// Add these to your lwipopts.h or create one if it doesn't exist
//#define TCP_MSS                 1460
//#define TCP_SND_BUF             (8 * TCP_MSS)  // 8 segments = ~11KB
//#define TCP_WND                 (8 * TCP_MSS)  // Match send buffer
//#define PBUF_POOL_SIZE          24             // Increase packet buffer pool
//#define MEMP_NUM_TCP_SEG        32             // More TCP segments

// ping_thread sets socket receive timeout, so enable this feature
#define LWIP_SO_RCVTIMEO 1
#endif


#endif
