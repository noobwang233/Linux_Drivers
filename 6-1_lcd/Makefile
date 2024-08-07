# 1. 使用不同的开发板内核时, 一定要修改 KERN_DIR
# 2. KERN_DIR 中的内核要事先配置、编译, 为了能编译内核, 要先设置下列环境变量:
# 2.1 ARCH, 比如: export ARCH=arm
# 2.2 CROSS_COMPILE=arm-linux-gnueabihf-, 比如: export CROSS_COMPILE=aarch64-linux-gnu06 # 2.3 PATH, 比如: export PATH=$PATH:/home/book/100ask_roc-rk3399-pc/ToolChain-6.3.1/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin
# 注意: 不同的开发板不同的编译器上述 3 个环境变量不一定相同,
# 请参考各开发板的高级用户使用手册

MODULE_NAME := lcd
DTB_NAME := imx6ull-atk-mini-emmc
DIRVER_NAME := $(MODULE_NAME)_drv
APP_NAME := $(MODULE_NAME)_app
KERNELDIR := $(shell pwd)/../../kernel/linux-imx_4.1.15
CURRENT_PATH := $(shell pwd)
OUTPUT_DIR := ~/OSD_share/$(MODULE_NAME)

obj-m += $(DIRVER_NAME).o
# $(DIRVER_NAME)-objs = led_drv.o # 可以支持多个c文件
build: kernel_modules copy

copy: $(OUTPUT_DIR)/$(DIRVER_NAME).ko $(OUTPUT_DIR)/$(DTB_NAME).dtb

kernel_modules: $(OUTPUT_DIR) $(DIRVER_NAME).ko $(DTB_NAME).dtb

app: $(OUTPUT_DIR) $(OUTPUT_DIR)/$(APP_NAME)

all: kernel_modules app

$(OUTPUT_DIR):
	@mkdir -p $(OUTPUT_DIR)

$(OUTPUT_DIR)/$(DIRVER_NAME).ko: $(DIRVER_NAME).ko
	cp $(DIRVER_NAME).ko $(OUTPUT_DIR)/ -f

$(DIRVER_NAME).ko: $(DIRVER_NAME).c
	@export ARCH=arm
	@export CROSS_COMPILE=arm-linux-gnueabihf-
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

$(OUTPUT_DIR)/$(DTB_NAME).dtb: $(DTB_NAME).dtb
	cp $(KERNELDIR)/arch/arm/boot/dts/$(DTB_NAME).dtb $(OUTPUT_DIR)/ -f

$(DTB_NAME).dtb: $(KERNELDIR)/arch/arm/boot/dts/$(DTB_NAME).dts
	@export ARCH=arm
	@export CROSS_COMPILE=arm-linux-gnueabihf-
	$(MAKE) -C $(KERNELDIR) $(DTB_NAME).dtb

$(OUTPUT_DIR)/$(APP_NAME): $(APP_NAME).c
	arm-linux-gnueabihf-gcc -o $(APP_NAME) $(APP_NAME).c $(LFLAGS)
	cp $(APP_NAME)  $(OUTPUT_DIR)/

.PHONY: clean clean_all
clean:
	rm -rf *.o *app *.ko .*.o.d .*.cmd *.mod.* *.temp *.mod modules.order Module.symvers .tmp_versions
clean_all:
	rm -rf $(OUTPUT_DIR) *.o *app *.ko .*.o.d .*.cmd *.mod.* *.temp *.mod modules.order Module.symvers .tmp_versions