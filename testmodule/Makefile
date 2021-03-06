#多模块 多目录 相互引用Makefile
#export KRNELDIR CROSS_COMPILE CROSS_ARCH can fix the bug
#[fixed] 已知BUG test 比 testlib 先编译，因此第一次编译的时候无法生成正确的test.ko
obj-m += cmd.o
obj-y += print/
obj-y += subprint/

PWD :=$(shell pwd)
PKG_NAME=testmodule

# This is the version number of the package
VERSION_MAJOR:=1
VERSION_MINOR:=0
VERSION_PATCH:=0

# Name of the include file that will contain version information
VERSION_DIR:=$(PWD)/include
VERSION_FILE:=$(VERSION_DIR)/version.h

# END USER CONFIGURABLE SECTION
# (DO NOT EDIT BELOW THIS LINE)
#-------------------------------------------------------------------

#\1 extract([a-z]) a letter
#\2 extract([a-zA-Z0-9]*\) a letter and left letters
#\u convert uppercase
PKG_FIRST_UC:=$(shell echo $(PKG_NAME) | sed 's/\([a-z]\)\([a-zA-Z0-9]*\)/\u\1\2/g')
PKG_NAME_UC:=$(shell echo $(PKG_NAME) | tr a-z A-Z)

# Make sure our version of doxygen is used
DOXYGEN:=$(PWD)/doxygen/doxygen
DOXYFILE:=$(PWD)/doxygen/doxyfile.in
DOCS_DIR:=$(PWD)/docs/$(PKG_NAME)

define VERSION_TEXT
/***********************************
 * DO NOT EDIT THIS FILE DIRECTLY! *
 *  (This file is auto-generated)  *
 *    Instead, edit Makefile       *
 ***********************************/

#ifndef $(PKG_NAME_UC)_VERSION_H
#define $(PKG_NAME_UC)_VERSION_H

  #define $(PKG_NAME_UC)_VERSION_MAJOR   $(VERSION_MAJOR)
  #define $(PKG_NAME_UC)_VERSION_MINOR   $(VERSION_MINOR)
  #define $(PKG_NAME_UC)_VERSION_PATCH   $(VERSION_PATCH)

  #define _PVS(maj,min,pat) #maj "." #min "." #pat
  #define $(PKG_NAME_UC)_VERSION_STRING(maj,min,pat) _PVS(maj,min,pat)

  static const char *s$(PKG_FIRST_UC)Version =
    $(PKG_NAME_UC)_VERSION_STRING($(PKG_NAME_UC)_VERSION_MAJOR,
                                 $(PKG_NAME_UC)_VERSION_MINOR,
                                 $(PKG_NAME_UC)_VERSION_PATCH);

#endif

endef

KRNELDIR :=/home/MinXu/Perforce/ti_ezsdk_phoenix_kernel_dev/ti-ezsdk_dm814x-evm_5_02_02_60/board-support/linux-2.6.37-psp04.01.00.05.patch1
CROSS_COMPILE =/home/MinXu/Perforce/ti_ezsdk_phoenix_kernel_dev/linux/host/ub1004/x86/cs-2010.09/bin/arm-none-linux-gnueabi-
CC :=$(CROSS_COMPILE)gcc
LD :=$(CROSS_COMPILE)ld
STRP := $(CROSS_COMPILE)strip
CROSS_ARCH := ARCH=arm

export KRNELDIR CROSS_COMPILE CROSS_ARCH

#查找本目录ko文件 list
#MODULES := $(patsubst %.c,%.ko,$(wildcard *.c))
#SOURCE := $(filter-out %.mod.ko, $(SOURCE))
#递归查找全部ko文件 list
SOURCE := $(shell find ./ -name '*.ko')

#KBUILD_EXTRA_SYMBOLS += $(PWD)/testlib
#export KBUILD_EXTRA_SYMBOLS

PREFIX :=$(shell date "+%H:%M:%S*%d-%m-%y")

all:version docs module
version:
	@mkdir -p $(VERSION_DIR)
	@echo "$$VERSION_TEXT" > $(VERSION_FILE)

docs:
	@if [ ! -f $(DOXYGEN) ]; then \
	  echo "Doxygen not found! Have you installed the package?"; \
	  exit 1; \
	 fi
	@mkdir -p $(DOCS_DIR)
	@cat $(DOXYFILE) > Doxyfile
	@echo "PROJECT_NAME = $(PKG_NAME)" >> Doxyfile
	@echo "PROJECT_NUMBER = $(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_PATCH)" >> Doxyfile
	@# This should come from package.txt eventually
	@echo "PROJECT_BRIEF = " >> Doxyfile
	@echo "INPUT = $(PKG_ROOT)" >> Doxyfile
	@echo "OUTPUT_DIRECTORY = $(DOCS_DIR)" >> Doxyfile
	@$(DOXYGEN) Doxyfile;

module:
#	$(MAKE) $(CROSS_ARCH)  -C $(KRNELDIR) SUBDIRS=$(PWD) $(EXTRA_CFLAGS) modules
	$(MAKE) $(CROSS_ARCH)  -C $(KRNELDIR) M=$(PWD) modules

install:all
	$(MAKE) $(CROSS_ARCH)  -C $(KRNELDIR) M=$(PWD) modules
	$(STRP) -S $(SOURCE)
	$(MAKE) INSTALL_MOD_PATH=$(PWD) $(CROSS_ARCH)  -C $(KRNELDIR) M=$(PWD) modules_install

clean:cleanversion cleanpackage cleaninstall
	@rm -rf $(DOCS_DIR)
	@rm -rf docs
	@rm -rf manifests
	@rm -f Doxyfile
#       $(MAKE) $(CROSS_ARCH)  -C $(KRNELDIR) SUBDIRS=$(PWD) $(EXTRA_CFLAGS) clean
	$(MAKE) $(CROSS_ARCH)  -C $(KRNELDIR) M=$(PWD) clean
cleanversion:
	@rm -rf $(VERSION_FILE)
cleanpackage:
	@rm -rf $(PKG_NAME)*.tgz
cleaninstall:
	@rm -rf lib

package:install
	@mkdir -p manifests
	@du -cha --time > manifests/$(PKG_NAME)_$(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_PATCH)
	@tree >> manifests/$(PKG_NAME)_$(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_PATCH)
	@tar czf $(PKG_NAME)_$(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_PATCH).tgz include lib docs manifests

.PHONY : all clean install package
export VERSION_TEXT
