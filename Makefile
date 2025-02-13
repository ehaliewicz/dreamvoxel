all: rm-elf main.elf output.bin 1st_read.bin

include $(KOS_BASE)/Makefile.rules

CFLAGS = -std=c99 -O3  -ffast-math -ffp-contract=fast -mfsca -mfsrra -fomit-frame-pointer -flto  -mpretend-cmove -falign-loops=4 -falign-labels=2 -falign-functions=32 -falign-jumps=2
CFLAGS = -std=c99 -O3  -ffast-math -ffp-contract=fast -mfsca -mfsrra -fomit-frame-pointer -mpretend-cmove -falign-loops=4 -falign-labels=2 -falign-functions=32 -falign-jumps=2
# -mrelax

#KOS_CFLAGS=-O3 -ffast-math -fomit-frame-pointer -ml -m4-single-only -ffunction-sections -fdata-sections  -I/opt/toolchains/dc/kos/include -I/opt/toolchains/dc/kos/kernel/arch/dreamcast/include -I/opt/toolchains/dc/kos/addons/include -I/opt/toolchains/dc/kos/../kos-ports/include -I/opt/toolchains/dc/kos/include -I/opt/toolchains/dc/kos/kernel/arch/dreamcast/include -I/opt/toolchains/dc/kos/addons/include -I/opt/toolchains/dc/kos/../kos-ports/include -D_arch_dreamcast -D_arch_sub_pristine -Wall -fno-builtin -ml -m4-single-only -ffunction-sections -fdata-sections
KOS_CFLAGS+=$(CFLAGS)
#-O3 -ffast-math -mfsca -mfsrra -fomit-frame-pointer  -flto

OBJS = common.o load_map.o stats.o main.o
#main.o
	
clean:
	-rm -f main.elf $(OBJS)
	-rm -f romdisk_boot.*
	-rm -f output.bin
	-rm -f ./iso/1st_read.bin
	
clean-all:
	-rm -f main.elf $(OBJS) main.iso output.bin Program.cdi 1st_read.bin
	-rm -f romdisk_boot.*

dist:
	-rm -f $(OBJS)
	$(KOS_STRIP) main.elf
	
rm-elf:
	-rm -f main.elf
	-rm -f romdisk_boot.*

main.elf: $(OBJS) romdisk_boot.o
	$(KOS_CC) $(KOS_CFLAGS) $(KOS_LDFLAGS) -o $@ $(KOS_START) $^ -lm $(KOS_LIBS)  -lkmg -lkosutils
	# -Wl,--relax

romdisk_boot.img:
	$(KOS_GENROMFS) -f $@ -d romdisk_boot -v

romdisk_boot.o: romdisk_boot.img
	$(KOS_BASE)/utils/bin2o/bin2o $< romdisk_boot $@

output.bin: main.elf
	sh-elf-objcopy -R .stack -O binary main.elf output.bin

1st_read.bin: output.bin
	$(KOS_BASE)/utils/scramble/scramble output.bin ./iso/1st_read.bin