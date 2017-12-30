#ifndef _APPLICATION_H
#define _APPLICATION_H

#ifndef CORE_MODULE
#define CORE_MODULE 1
#endif

#ifndef VERSION
#define VERSION "vdev"
#endif

#if CORE_MODULE
#define FIRMWARE "bcf-gateway-usb-dongle"
#define GPIO_LED 19
#else
#define FIRMWARE "bcf-gateway-core-module"
#define GPIO_LED BC_GPIO_LED
#endif

#include <bcl.h>
#define TALK_OVER_CDC 1

#endif
