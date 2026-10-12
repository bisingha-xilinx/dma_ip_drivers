# Out-of-tree PF0 glue for the RAVE2 native HDMA endpoint.
# Build on the Ryzen, against the running kernel:
#   make -C /lib/modules/$(uname -r)/build M=$PWD modules

obj-m += rave2-hdma.o

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
