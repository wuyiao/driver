INCLUDE      += -I$(shell pwd)/
EXTRA_CFLAGS += -I$(shell pwd)/ -I./userapp/stm32f030cc_gsv2k11_ffc_gsv/apps/

platform = linux
#platform = android

export ARCH=arm64
ifeq ($(platform), android)
  $(warning platform = android !!!)
  KERNELSRC=/home/touch/projects/rk3568/rk3568_Android11/kernel
  export CROSS_COMPILE=/home/touch/projects/rk3568/rk3568_Android11/prebuilts/gcc/linux-x86/aarch64/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
endif

ifeq ($(platform), linux)
  $(warning platform = linux !!!)
  KERNELSRC=/home/disk2/sdk/gk7682/SS528V100_SDK_V2.0.0.1/open_source/linux/linux-4.19.y
  export CROSS_COMPILE=aarch64-mix210-linux-
endif

export CC=$(CROSS_COMPILE)gcc
export STRIP=$(CROSS_COMPILE)strip

#VPATH = ./kapi:./uapi:./uapi/gsv2k6_r1_reg:./userapp/stm32f030f8_gsv2k6_2to4_disti/apps:./userbsp/keil/stm32f030c8_gsv2k6_2to4_disti

#SRC  := $(wildcard *.c ./kapi/*.c ./uapi/*.c ./userapp/stm32f030_gsv2k8_4to2_gsv/apps/*.c ./userbsp/keil/stm32f030_gsv2k8_4to2_gsv/*.c)
SRC  := $(wildcard *.c ./kapi/*.c ./uapi/*.c ./userapp/stm32f030cc_gsv2k11_ffc_gsv/apps/*.c ./userbsp/keil/stm32f030cc_gsv2k11_ffc_gsv/*.c)
#OBJ  := $(SRC:%.c=%.o)

OBJ  := gsv2015_core.o ./kapi/uapi_function_mapper.o ./kapi/kernel_status_update.o ./kapi/kernel_fsm.o ./kapi/kapi_debug_info.o ./kapi/kapi.o ./uapi/gsv2k11.o ./uapi/hal.o ./uapi/uapi.o ./userapp/stm32f030cc_gsv2k11_ffc_gsv/apps/av_main.o ./userapp/stm32f030cc_gsv2k11_ffc_gsv/apps/av_edid_manage.o ./userapp/stm32f030cc_gsv2k11_ffc_gsv/apps/av_irda_cmd.o ./userapp/stm32f030cc_gsv2k11_ffc_gsv/apps/av_uart_cmd.o ./userapp/stm32f030cc_gsv2k11_ffc_gsv/apps/av_key_cmd.o ./userapp/stm32f030cc_gsv2k11_ffc_gsv/apps/av_event_handler.o ./userapp/stm32f030cc_gsv2k11_ffc_gsv/apps/av_common.o ./userbsp/keil/stm32f030cc_gsv2k11_ffc_gsv/bsp.o

NAME=gsv2015
#$(NAME)-y = $(NAME)_core.o
$(NAME)-y := $(OBJ)
obj-m := $(NAME).o

# target source
#SRC  := $(wildcard *.c)
#OBJ  := $(SRC:%.c=%.o)

ccflags-y := -I$(PWD) -I$(PWD)/userapp/stm32f030cc_gsv2k11_ffc_gsv/apps -I$(PWD)/uapi -I$(PWD)/kapi -I$(PWD)/uapi/gsv2k11_r1_reg -I$(PWD)/userbsp/keil/stm32f030cc_gsv2k11_ffc_gsv

all: clean $(NAME).ko

$(NAME).ko:
	make -C $(KERNELSRC) M=$(shell pwd) modules
	$(STRIP) -g --strip-unneeded $(NAME).ko

#touch_key_board_device.ko:
#       make -C $(KERNELSRC) M=$(shell pwd) modules
#       $(STRIP) -g --strip-unneeded touch_key_board_device.ko

#install: $(NAME).ko
#        cp -rf $(NAME).ko $(PATH_MTD)

clean:
	@rm  *.mod.* -rf
	@rm  *.ko -rf
	@rm  *.o* -rf
	@rm  *.o.* -rf
	@rm  .*.cmd -rf
	@rm  .tmp_versions -rf
	@rm  *.symvers -rf
	@rm  *.order *.markers -rf

