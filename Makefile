TARGET := hid-ft260
KRELEASE ?= $(shell uname -r)
KBUILD ?= /lib/modules/$(KRELEASE)/build
obj-m += hid-ft260.o

modules:
	@echo "Making modules $(TARGET) ..."
	$(MAKE) -C $(KBUILD) M=$(PWD) modules

install: modules
	/usr/bin/install -m 644 -D $(TARGET).ko /lib/modules/$(KRELEASE)/kernel/drivers/hid/$(TARGET).ko
	/usr/bin/install -m 644 -D $(TARGET).conf /usr/lib/modules-load.d/$(TARGET).conf

clean:
	$(MAKE) -C $(KBUILD) M=$(PWD) clean
