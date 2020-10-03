PROGRAM = main

EXTRA_COMPONENTS = \
    extras/onewire \
    extras/ds18b20 \
	extras/dht \
	extras/http-parser \
	extras/dhcpserver \
	extras/rboot-ota \
	$(abspath ../../lib/adv_button) \
	$(abspath ../../lib/led_codes)\
	$(abspath ../../components/esp-8266/wifi_config) \
	$(abspath ../../components/common/wolfssl) \
	$(abspath ../../components/esp-8266/cJSON) \
	$(abspath ../../components/esp-led-status) \
	$(abspath ../../components/common/homekit)\
	$(abspath ../../components/esp-ir)\
	$(abspath ../../components/UDPlogger)


FLASH_SIZE = 8
FLASH_MODE = dout
FLASH_SPEED = 40

HOMEKIT_SPI_FLASH_BASE_ADDR = 0x8c000
HOMEKIT_MAX_CLIENTS = 16
HOMEKIT_SMALL = 0

##AYto einai gia to hostname


EXTRA_CFLAGS += -I../.. -DHOMEKIT_SHORT_APPLE_UUIDS
EXTRA_CFLAGS += -DHOMEKIT_OVERCLOCK_PAIR_VERIFY
EXTRA_CFLAGS += -DHOMEKIT_OVERCLOCK_PAIR_SETUP
EXTRA_CFLAGS += -DUDPLOG_PRINTF_TO_UDP
EXTRA_CFLAGS += -DUDPLOG_PRINTF_ALSO_SERIAL
EXTRA_CFLAGS += -DLWIP_NETIF_HOSTNAME=1
EXTRA_CFLAGS += -DconfigUSE_TRACE_FACILITY

## DEBUG
#EXTRA_CFLAGS += -DHOMEKIT_DEBUG=1


include $(SDK_PATH)/common.mk

signature:
	$(shell openssl sha384 -binary -out firmware/main.bin.sig firmware/main.bin)
	$(shell printf "%08x" `cat firmware/main.bin | wc -c`| xxd -r -p >>firmware/main.bin.sig)

monitor:
	$(FILTEROUTPUT) --port $(ESPPORT) --baud 115200 --elf $(PROGRAM_OUT)
