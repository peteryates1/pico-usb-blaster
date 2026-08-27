/*
 * jtag_wiggle — drive one USB-Blaster output to a known state so it can be
 * measured on the target side of the level shifters.
 *
 * jtag_pintest answers "is TDO coming back?". This answers the other half:
 * "is the Pico actually driving TCK/TMS/TDI out through the shifters?" — which
 * a DMM can confirm without a scope.
 *
 *   static levels : the pin sits at 0 V or VTREF; probe with any meter
 *   toggle mode   : ~1 Hz square wave, so a meter reads about VTREF/2 and a
 *                   cheap logic probe or LED blinks visibly
 *
 * Every command carries the OE bit, so the Blaster output stage is enabled.
 * On the pico-usb-debug-jtag carrier the shifter DIR pin (GP3) is driven high
 * by the firmware at init and stays high, so the outbound shifters should be
 * passing A->B the whole time this runs.
 *
 *   gcc -O2 -o jtag_wiggle jtag_wiggle.c -lusb-1.0
 *   ./jtag_wiggle 1:49 tck toggle
 *   ./jtag_wiggle 1:49 tms high
 *   ./jtag_wiggle 1:49 all low
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <libusb-1.0/libusb.h>

#define EP_OUT 0x02
#define TIMEOUT_MS 500

/* Bit-bang byte: bit0 TCK, bit1 TMS, bit4 TDI, bit5 OE, bit6 READ */
#define B_OE   0x20
#define B_TCK  0x01
#define B_TMS  0x02
#define B_TDI  0x10

static libusb_device_handle *dev;
static volatile sig_atomic_t stop;

static void on_int(int s) { (void)s; stop = 1; }

static int put(unsigned char lvl)
{
	unsigned char cmd = (unsigned char)(B_OE | lvl);
	int transferred = 0;
	return libusb_bulk_transfer(dev, EP_OUT, &cmd, 1, &transferred, TIMEOUT_MS);
}

int main(int argc, char **argv)
{
	int want_bus = -1, want_addr = -1;
	if (argc < 4) {
		fprintf(stderr,
			"Usage: %s <bus:dev|serial|auto> <tck|tms|tdi|all> <high|low|toggle>\n"
			"  e.g. %s auto tck toggle\n"
			"  'auto' picks the 09fb:6001 whose iManufacturer is \"Pico\",\n"
			"  which is how this clone is told apart from a genuine cable.\n"
			"  Prefer it: the device address changes on every reflash.\n",
			argv[0], argv[0]);
		return 2;
	}
	const char *want_serial = NULL;
	if (strcmp(argv[1], "auto") != 0 &&
	    sscanf(argv[1], "%d:%d", &want_bus, &want_addr) != 2) {
		/* not bus:dev — treat it as an iSerial substring */
		want_serial = argv[1];
		want_bus = want_addr = -1;
	}

	unsigned char mask;
	if      (!strcmp(argv[2], "tck")) mask = B_TCK;
	else if (!strcmp(argv[2], "tms")) mask = B_TMS;
	else if (!strcmp(argv[2], "tdi")) mask = B_TDI;
	else if (!strcmp(argv[2], "all")) mask = B_TCK | B_TMS | B_TDI;
	else { fprintf(stderr, "unknown signal '%s'\n", argv[2]); return 2; }

	int mode; /* 0 low, 1 high, 2 toggle */
	if      (!strcmp(argv[3], "low"))    mode = 0;
	else if (!strcmp(argv[3], "high"))   mode = 1;
	else if (!strcmp(argv[3], "toggle")) mode = 2;
	else { fprintf(stderr, "unknown mode '%s'\n", argv[3]); return 2; }

	libusb_init(NULL);
	libusb_device **list;
	ssize_t n = libusb_get_device_list(NULL, &list);
	for (ssize_t i = 0; i < n; i++) {
		struct libusb_device_descriptor d;
		if (libusb_get_device_descriptor(list[i], &d) != 0)
			continue;
		if (d.idVendor != 0x09fb || d.idProduct != 0x6001)
			continue;
		int bus = libusb_get_bus_number(list[i]);
		int addr = libusb_get_device_address(list[i]);
		if (want_bus >= 0 && (bus != want_bus || addr != want_addr))
			continue;
		if (libusb_open(list[i], &dev) != 0) {
			dev = NULL;
			continue;
		}
		if (want_serial) {
			/* iSerial comes from the RP2040 unique flash ID, so it is
			 * the only selector that stays unambiguous once there is
			 * more than one Pico probe on the bus. */
			unsigned char ser[64] = {0};
			libusb_get_string_descriptor_ascii(dev, d.iSerialNumber,
							   ser, sizeof(ser));
			if (!strstr((char *)ser, want_serial)) {
				libusb_close(dev);
				dev = NULL;
				continue;
			}
		} else if (want_bus < 0) {
			/* Weakest selector: every Pico running this firmware
			 * reports iManufacturer "Pico" (a genuine cable reports
			 * "Altera"), so this is ambiguous with two Pico probes.
			 * Prefer a serial. */
			unsigned char mfg[64] = {0};
			libusb_get_string_descriptor_ascii(dev, d.iManufacturer,
							   mfg, sizeof(mfg));
			if (strcmp((char *)mfg, "Pico") != 0) {
				libusb_close(dev);
				dev = NULL;
				continue;
			}
		}
		printf("probe at bus %d dev %d\n", bus, addr);
		break;
	}
	libusb_free_device_list(list, 1);
	if (!dev) { fprintf(stderr, "no matching probe (bus:dev wrong, or no \"Pico\" cable present)\n"); return 1; }

	libusb_detach_kernel_driver(dev, 0);
	if (libusb_claim_interface(dev, 0) != 0) {
		fprintf(stderr, "claim interface failed\n");
		return 1;
	}

	signal(SIGINT, on_int);

	if (mode != 2) {
		put(mode ? mask : 0);
		printf("%s driven %s — measure on the TARGET side of the shifter.\n"
		       "  expect %s. Ctrl-C or rerun to change.\n",
		       argv[2], mode ? "HIGH" : "LOW",
		       mode ? "VTREF (3.3 V here)" : "0 V");
	} else {
		printf("%s toggling at ~1 Hz — a meter should read about VTREF/2.\n"
		       "  Ctrl-C to stop (leaves the pin low).\n", argv[2]);
		while (!stop) {
			put(mask); usleep(500000);
			if (stop) break;
			put(0);    usleep(500000);
		}
		put(0);
		printf("\nstopped, pin left low\n");
	}

	libusb_release_interface(dev, 0);
	libusb_close(dev);
	libusb_exit(NULL);
	return 0;
}
