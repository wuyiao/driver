#ifndef __GLOBAL_H
#define __GLOBAL_H

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/gpio.h>
#include <linux/of.h>

#define ARM64



#define TS700 1//0
#define TS5000 0//1

#define GSV2015_MAX_NUM 8

#if TS700
//#define GSV2015_NUM 3
#else if TS5000
//#define GSV2015_NUM 5
#endif

#endif
