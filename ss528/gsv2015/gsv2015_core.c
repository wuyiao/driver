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
#include <linux/uaccess.h>

#include "gsv2015_core.h"

// typedef unsigned char  uchar;
// typedef char           schar;
// typedef unsigned char  uint8;
// typedef signed char    int8;
// typedef unsigned short uint16;
// typedef short          int16;
// typedef unsigned int   uint32;
// typedef int            int32;

#define GET_GSV2015_CHIP1_VIDEO_INFO    0x1210
#define GET_GSV2015_CHIP2_VIDEO_INFO    0x1211
#define GET_GSV2015_CHIP3_VIDEO_INFO    0x1212
#define GET_GSV2015_CHIP4_VIDEO_INFO    0x1213
#define GET_GSV2015_CHIP5_VIDEO_INFO    0x1214

int gsv2015_num = 3;
module_param(gsv2015_num,int,S_IRUSR);

gsv2015_core_dev g_gsv2015_dev;

struct i2c_client *gsv2015_client[MAX_GSV2015_CLIENT];

struct task_struct * gsv2015_thread;

//extern AvPort gsv2k1Ports[12];
extern AvPort gsv2k11Ports[PORT_NUM];

extern VIDINFO vidinfo[GSV2015_MAX_NUM];

//extern uint8 RegTxAFrom;
//extern uint8 RegTxBFrom;
//extern uint8 RegTxCFrom;
//extern uint8 RegTxDFrom;

/* define return values of funcitons, including api */
// typedef enum
// {
//     AvNotSupport = -4,
//     AvNotAvailable = -3,
//     AvInvalidParameter = -2,
//     AvError = -1,
//     AvOk = 0
// } AvRet;

AvRet ManI2cRead(uint32 devAddress, uint32 regAddress, uint8 *i2cdata, uint16 count, uint8 index, uint8 Flag16bit)
{
	struct i2c_adapter *adap = g_gsv2015_dev.client->adapter;
	struct i2c_msg msg[2];
	unsigned char buf[4];
	//unsigned char r_buf[1];
	int len;
	int ret;

	if (Flag16bit == 1) {//表示 寄存器的地址是16bit
		buf[0] = (regAddress >> 8) & 0xff; //注意,根据demo,先发高八位,也就是页地址,一开始写返了导致读不成功
		buf[1] = regAddress & 0xff;
		//buf[0] = regAddress & 0xff;
		//buf[1] = (regAddress >> 8) & 0xff;
		len = 2;
	} else {
		//不支持 Flag16bit == 0 的情况，目前 gsv2008 都是 16 位的寄存器地址，发 8 位的地址会导致 i2c 报错
		//目前在某些逻辑下会导致 Flag16bit == 0   例如 调用 AvApiConnectPort(&gsv2k8Ports[6], &gsv2k8Ports[2], AvConnectAudio);
		//为了避免这种情况，直接返回
		return AvNotSupport;
		//buf[0] = regAddress & 0xff;
		//len = 1;
	}
/*
	ret = i2c_master_recv(g_gsv2008_dev.client, buf, len);
	printk("%s client->addr : %x ret : %d !!!!!!!!!!!!!!!!\n", __func__, g_gsv2008_dev.client->addr, ret);
	if (ret >= 0) {
		i2cdata[0] = buf[0];
	}
*/
	//drivers/i2c/busses/i2c-rk3x.c 描述了 3308 传输 i2c 的逻辑,
	//当i2c设备的寄存器地址为16位即2个字节时,需传输多个msg,msg定义如下
        msg[0].addr = g_gsv2015_dev.client->addr;
        msg[0].flags = g_gsv2015_dev.client->flags & I2C_M_TEN;
        //msg.flags |= I2C_M_RD;
        msg[0].len = len;
        msg[0].buf = buf;

	msg[1].addr = g_gsv2015_dev.client->addr; //这里的和rk的不同, gk7628 msg[1] 也要设置addr
	msg[1].flags = g_gsv2015_dev.client->flags & I2C_M_TEN; //同上, 和 rk 也不同, 如果没有这一句, i2c读取会失败
	msg[1].flags |= I2C_M_RD; //
	//msg[1].len = 1; //读取的数据长度
	//msg[1].buf = r_buf; //读到的数据存放在 msg[1] 的 buf 中
	msg[1].len = count;
	msg[1].buf = i2cdata;

        ret = i2c_transfer(adap, msg, 2);
		if (ret < 0) {
			printk("%s i2c_transfer error ret : %d regAddress : 0x%x count : %d Flag16bit : %d !!\n", __func__, ret, regAddress, count, Flag16bit);
		}
// 	printk("%s ret : %d !!!!!!!!!\n", __func__, ret);
// 	if (ret >= 0) {
// 		i2cdata[0] = msg[1].buf[0];
// 		printk("%s read data : 0x%x !!!!!!!!!!!\n", __func__, msg[1].buf[0]);
// 	}
	//return (ret == 1) ? count : ret;
	
	return AvOk;
	//i2c_master_send(bcm2079x_dev.client, );
}

AvRet ManI2cWrite(uint32 devAddress, uint32 regAddress, uint8 *i2cdata, uint16 count, uint8 index, uint8 Flag16bit)
{
	int ret;
	int len;
	int i;
	unsigned char buf[512];

// 	if (count > (8 - 2)) {
// 		printk("%s count : %d invalid,should under 6 !!!!!!!!\n", __func__, count);
// 		return AvInvalidParameter;
// 	}

	if (Flag16bit == 1) {//表示 寄存器的地址是16bit
		buf[0] = (regAddress >> 8) & 0xff;
		buf[1] = regAddress & 0xff;
		//buf[2] = i2cdata[0];
		//len = 3;
		for (i = 0; i < count; i++) {
			buf[2 + i] = i2cdata[i];
		}
		len = count + 2;
	} else {
		//这里也是同个道理
		//不支持 Flag16bit == 0 的情况，目前 gsv2008 都是 16 位的寄存器地址，发 8 位的地址会导致 i2c 报错
		//目前在某些逻辑下会导致 Flag16bit == 0   例如 调用 AvApiConnectPort(&gsv2k8Ports[6], &gsv2k8Ports[2], AvConnectAudio);
		//为了避免这种情况，直接返回
		return AvNotSupport;
#if 0
		buf[0] = regAddress & 0xff;
		//buf[1] = i2cdata[0];
		//len = 2;
		for (i = 0; i < count; i++) {
			buf[1 + i] = i2cdata[i];
		}
		len = count + 1;
#endif
	}

	ret = i2c_master_send(g_gsv2015_dev.client, buf, len); //目前返回值是 -6 这个错误表明i2c设备或地址错误
	if (ret < 0) {
		printk("%s i2c_master_send error ret : %d regAddress : 0x%x !!\n", __func__, ret, regAddress);
	}
	//ret = i2c_smbus_write_byte_data(client, buf[0], buf[1]);
	//printk("%s ret = %d!\n", __func__, ret);
    
	return AvOk;
}

static int gsv2015_handle_thread(void * arg)
{
	printk("%s enter !!!!!!!!!!!\n", __func__);
	GsvMain(); //放到线程中执行
	
	return 0;
}

static int gsv2015_open(struct inode * inode, struct file * filp)
{

	return 0;
}

static int gsv2015_close(struct inode * inode, struct file * filp)
{
	return 0;
}

static ssize_t gsv2015_read(struct file * file, char __user * buf, size_t count, loff_t * offset)
{
	return 0;
}

static ssize_t gsv2015_write(struct file * file, const char __user * buf, size_t count, loff_t * offset)
{
	return 0;
}

static long gsv2015_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned int k_arg;
	unsigned char val;
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	AvPort *RxPort = &gsv2k11Ports[0];

	switch(cmd) {
		case GET_RXA_INPUT_5V_STATUS:
			put_user(!!(RxPort->content.rx->Input5V), p);
			break;
		case GET_GSV2015_CHIP1_VIDEO_INFO:
			if(copy_to_user((void*)arg, (void*)&vidinfo[0], sizeof(VIDINFO)));
			break;
		case GET_GSV2015_CHIP2_VIDEO_INFO:
			if(copy_to_user((void*)arg, (void*)&vidinfo[1], sizeof(VIDINFO)));
			break;
		case GET_GSV2015_CHIP3_VIDEO_INFO:
			if(copy_to_user((void*)arg, (void*)&vidinfo[2], sizeof(VIDINFO)));
			break;
		case GET_GSV2015_CHIP4_VIDEO_INFO:
			if(copy_to_user((void*)arg, (void*)&vidinfo[3], sizeof(VIDINFO)));
			break;
		case GET_GSV2015_CHIP5_VIDEO_INFO:
			if(copy_to_user((void*)arg, (void*)&vidinfo[4], sizeof(VIDINFO)));
			break;
		default: {
			printk("Kernel: No such gsv2015 command %#x!\n", cmd);
			return -1;
		}
	}

	return 0;
}

static long gsv2015_ioctl32(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned int k_arg;
	unsigned char val;
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	AvPort *RxPort = &gsv2k11Ports[0];

	switch(cmd) {
		case GET_RXA_INPUT_5V_STATUS:
			put_user(!!(RxPort->content.rx->Input5V), p);
			break;
		case GET_GSV2015_CHIP1_VIDEO_INFO:
			if(copy_to_user((void*)arg, (void*)&vidinfo[0], sizeof(VIDINFO)));
			break;
		case GET_GSV2015_CHIP2_VIDEO_INFO:
			if(copy_to_user((void*)arg, (void*)&vidinfo[1], sizeof(VIDINFO)));
			break;
		case GET_GSV2015_CHIP3_VIDEO_INFO:
			if(copy_to_user((void*)arg, (void*)&vidinfo[2], sizeof(VIDINFO)));
			break;
		case GET_GSV2015_CHIP4_VIDEO_INFO:
			if(copy_to_user((void*)arg, (void*)&vidinfo[3], sizeof(VIDINFO)));
			break;
		case GET_GSV2015_CHIP5_VIDEO_INFO:
			if(copy_to_user((void*)arg, (void*)&vidinfo[4], sizeof(VIDINFO)));
			break;
		default: {
			printk("Kernel: No such gsv2015 command %#x!\n", cmd);
			return -1;
		}
	}

	return 0;
}


static struct file_operations gsv2015_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl		= gsv2015_ioctl,
	.open			= gsv2015_open,
	.release		= gsv2015_close,
	.read			= gsv2015_read,
	.write			= gsv2015_write,
#ifdef CONFIG_COMPAT
	.compat_ioctl 		= gsv2015_ioctl32,
#endif
};

static struct miscdevice gsv2015_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name    = "gsv2015",
	.fops    = &gsv2015_fops,
};

#if 0  //注释掉 用另外一种方式，不依赖设备树
static const struct i2c_device_id gsv2008_i2c_id[] = {
        { "gsv2008", 0 },
        { }
};

static int gsv2008_i2c_probe(struct i2c_client *client,
                    const struct i2c_device_id *id)
{
	unsigned char r_data = 0;
	unsigned char w_data;
	int ret;

	printk("%s gsv2008_i2c_probe probe!!!!!!!!!!!!!\n", __func__);

	ret = misc_register(&gsv2008_misc);
	if(0 != ret)
	{
		printk("Kernel: register gsv2008 device failed!\n");
		return -1;
	}

	g_gsv2008_dev.client = client;

#if 0
	w_data = 0xff;
	ManI2cWrite(0x58, 0xfe, &w_data, 1, 1, 1);

	ManI2cRead(0x58, 0xfe, &r_data, 1, 1, 1);

	printk("%s read data : 0x%x !!!!!!!!!!!!!!\n", __func__, r_data);

        w_data = 0x00;
        ManI2cWrite(0x58, 0xfe, &w_data, 1, 1, 1);

        ManI2cRead(0x58, 0xfe, &r_data, 1, 1, 1);
	printk("%s read data : 0x%x !!!!!!!!!!!!!!\n", __func__, r_data);
#endif
	gsv2008_thread = kthread_run(gsv2008_handle_thread, NULL ,"gsv2008_thread");
	//GsvMain();
	//ManI2cWrite(0x58, 0xfe, &w_data, 1, 1, 1);

	return 0;
}

static int gsv2008_i2c_remove(struct i2c_client *i2c)
{
	misc_deregister(&gsv2008_misc);

	if (gsv2008_thread) {
		kthread_stop(gsv2008_thread);	
	}
    return 0;
}


static const struct of_device_id gsv2008_of_match[] = {
        { .compatible = "gsc,gsv2008", },
        {},
};
MODULE_DEVICE_TABLE(of, gsv2008_of_match);

/* machine i2c codec control layer */
static struct i2c_driver gsv2008_i2c_driver = {
        .driver = {
                .name = "gsv2008",
                .of_match_table = of_match_ptr(gsv2008_of_match),
        },
        .probe  = gsv2008_i2c_probe,
        .remove = gsv2008_i2c_remove,
        .id_table = gsv2008_i2c_id,
};

module_i2c_driver(gsv2008_i2c_driver);
#endif

#if 1


//#if TS5000
static struct i2c_board_info rk_info[] =
{
	//I2C_BOARD_INFO("gsv2015", 0xB0),
	[0] = {I2C_BOARD_INFO("gsv2015_1", 0x58)},
	[1] = {I2C_BOARD_INFO("gsv2015_2", 0x58)},
};
//#else
# if 0
static struct i2c_board_info rk_info[1] =
{
	//I2C_BOARD_INFO("gsv2015", 0xB0),
	{I2C_BOARD_INFO("gsv2015_1", 0x58)},
};
#endif
//#endif
static int init_i2c_client(void)
{
	struct i2c_adapter* i2c_adap;

	// gsv2008 use i2c1
	// 这里不是随意指定的，要指定已经注册的i2c，比如i2c1和i2c3
	// 如果要用到 i2c2 后面尝试在设备树增加i2c device，注册到新的 i2c 控制器
	// 在设备树&i2c2设置status 为 okay 即可使用 i2c2
	i2c_adap = i2c_get_adapter(1);
	//gsv2015 use i2c3
	//i2c_adap = i2c_get_adapter(3); //i2c3
    
	if (NULL == i2c_adap)
	{
		printk("find i2c adapter fail. \n");
		return -1;
	}
	g_gsv2015_dev.client = i2c_new_device(i2c_adap, &rk_info[0]);
	if (NULL == g_gsv2015_dev.client)
	{
		printk("instantiate an i2c device fail. \n");
		return -1;
	}

	gsv2015_client[0] = g_gsv2015_dev.client;

	i2c_put_adapter(i2c_adap);

//#if TS5000
	if (gsv2015_num > 4)
	{
		i2c_adap = i2c_get_adapter(0);

		if (NULL == i2c_adap)
		{
			printk("find i2c adapter(0) fail. \n");
			return -1;
		}
		g_gsv2015_dev.client = i2c_new_device(i2c_adap, &rk_info[1]);
		if (NULL == g_gsv2015_dev.client)
		{
			printk("instantiate an i2c device fail. \n");
			return -1;
		}

		gsv2015_client[1] = g_gsv2015_dev.client;

		i2c_put_adapter(i2c_adap);
	}
//#endif

	return 0;
}

static void uninit_i2c_client(void)
{
	//i2c_unregister_device(g_gsv2015_dev.client);
	i2c_unregister_device(gsv2015_client[0]);
	if (gsv2015_num > 4)
		i2c_unregister_device(gsv2015_client[1]);
}

#define GPIO23_5_MUXCTRL_BASE 0x010FF0010
#define GPIO23_5_MUXCTRL_BASE_SIZE 4

void __iomem * gpio23_5_muxctrl;

#define GPIO23_5_MUXCTRL (*(volatile unsigned int *)gpio23_5_muxctrl)

#define GPIO23_3_MUXCTRL_BASE 0x010FF0008
#define GPIO23_3_MUXCTRL_BASE_SIZE 4

void __iomem * gpio23_3_muxctrl;

#define GPIO23_3_MUXCTRL (*(volatile unsigned int *)gpio23_3_muxctrl)

#define GPIO23_BASE 0x110a7000
#define GPIO23_BASE_SIZE      0x1000
void __iomem* gpio23;

#define GPIO23_DIR (*(volatile unsigned int *)(gpio23+0x400))
#define GPIO23_5     (*(volatile unsigned int *)(gpio23+(1<<7)))
#define GPIO23_3     (*(volatile unsigned int *)(gpio23+(1<<5)))

static void gsv2015_reset(void)
{
	gpio23_5_muxctrl = ioremap_nocache(GPIO23_5_MUXCTRL_BASE, GPIO23_5_MUXCTRL_BASE_SIZE);
	if (!gpio23_5_muxctrl) {
		printk("Kernel: ioremap gpio25_1_muxctrl failed!\n");
		return;
	}
	GPIO23_5_MUXCTRL = 0x00; //mux to gpio

	gpio23_3_muxctrl = ioremap_nocache(GPIO23_3_MUXCTRL_BASE, GPIO23_3_MUXCTRL_BASE_SIZE);
	if (!gpio23_3_muxctrl) {
		printk("Kernel: ioremap gpio25_3_muxctrl failed!\n");
		return;
	}
	GPIO23_3_MUXCTRL = 0x00; //mux to gpio

	gpio23 = ioremap_nocache(GPIO23_BASE, GPIO23_BASE_SIZE);
	if (!gpio23) {
		printk("Kernel: ioremap gpio23 failed!\n");
		return;
	}

	GPIO23_DIR |= (1 << 5); //gpio23_5 out
	GPIO23_DIR |= (1 << 3);
	GPIO23_5 = 0x00; //low
	GPIO23_3 = 0x00;
	msleep(200);
	GPIO23_5 = 0xff; //high
	GPIO23_3 = 0xff;
	msleep(200);
}

static int __init gsv2015_init(void)
{
	int ret;

	ret = init_i2c_client();
	if (ret < 0) {
		printk("init i2c client failed !\n");
		return ret;
	}

	ret = misc_register(&gsv2015_misc);
	if(ret != 0)
	{
		printk("Kernel: register gsv2015 device failed!\n");
		return ret;
	}

	gsv2015_thread = kthread_run(gsv2015_handle_thread, NULL ,"gsv2015_thread");
	if(IS_ERR(gsv2015_thread))
	{
		printk("gsv2015_thread kthread_run fail\n");
	}

	return 0;
}

static void __exit gsv2015_exit(void)
{
	if (gsv2015_thread) {
		kthread_stop(gsv2015_thread);
	}
	misc_deregister(&gsv2015_misc);
	uninit_i2c_client();
	return ;
}

module_init(gsv2015_init);
module_exit(gsv2015_exit);
#endif

MODULE_LICENSE("GPL");
MODULE_VERSION("version 1.0");
