/*
spacenavd - a free software replacement driver for 6dof space-mice.
Copyright (C) 2007-2019 John Tsiombikas <nuclear@member.fsf.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifdef __FreeBSD__

#include "config.h"
#include <dev/usb/usb_ioctl.h>
#include <dev/usb/usbhid.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#include "spnavd.h"
#include "client.h"
#include "dev.h"
#include "dev_usb.h"
#include "event.h"

#define AXES		6
#define IS_DEV_OPEN(dev) ((dev)->fd >= 0)

static void close_hid(struct device *dev)
{
	if(IS_DEV_OPEN(dev)) {
		dev->set_led(dev, 0);
		close(dev->fd);
		dev->fd = -1;
	}
}

static void set_led_hid(struct device *dev, int state)
{
	struct usb_gen_descriptor d = {0};
	uint8_t buf[2];

	/*
	 * TODO: Get stuff from report descriptor:
	 * - Is there an LED?
	 * - How big is the message?
	 * - What bits should be set?
	 */

	if(!IS_DEV_OPEN(dev))
		return;

	buf[0] = 4;
	buf[1] = state ? 15 : 0;
	d.ugd_data = buf;
	d.ugd_maxlen = sizeof buf;
	d.ugd_report_type = UHID_OUTPUT_REPORT;

	if(ioctl(dev->fd, USB_SET_REPORT, &d) == -1) {
		logmsg(LOG_ERR, "Unable to set LED: %s\n", strerror(errno));
	}
}

static uint32_t button_event(struct dev_input *inp, uint32_t last, uint32_t curr)
{
	uint32_t new;
	int b;

	new = last ^ curr;
	b = ffs(new) - 1;

	if(new) {
		inp->type = INP_BUTTON;
		inp->idx = b;
		inp->val = (curr >> b) & 1;
		last ^= (1 << b);
	}
	return last;
}

static uint32_t axis_event(struct dev_input *inp, int16_t *curr, unsigned int *flush)
{
	int axis;

	if((axis = ffs(*flush)) > 0) {
		axis--;
		*flush &= ~(1 << axis);
		if(axis < AXES) {
			inp->type = INP_MOTION;
			inp->idx = axis;
			inp->val = curr[axis];
			return 0;
		}
		inp->type = INP_FLUSH;
		return 0;
	}

	return -1;
}

static int read_hid(struct device *dev, struct dev_input *inp)
{
	uint8_t iev[1024];
	int rdbytes;
	int i;
	static uint32_t last_buttons;
	static uint32_t curr_buttons;
	static int16_t curr_pos[AXES];
	static unsigned int flush = 0;

	if(!IS_DEV_OPEN(dev))
		return -1;

	if(last_buttons != curr_buttons) {
		last_buttons = button_event(inp, last_buttons, curr_buttons);
		return 0;
	}

	if(axis_event(inp, curr_pos, &flush) == 0) {
		return 0;
	}

	do {
		rdbytes = read(dev->fd, &iev, sizeof iev);
	} while(rdbytes == -1 && errno == EINTR);

	/* disconnect? */
	if(rdbytes == -1) {
		if(errno != EAGAIN) {
			logmsg(LOG_ERR, "read error: %s\n", strerror(errno));
			remove_device(dev);
		}
		return -1;
	}

	if(rdbytes > 0) {
		switch(iev[0]) {
		case 1: /* Three axis... X, Y, Z */
			flush = 0x40;
			for(i=0; i<rdbytes/2 && i<AXES; i++) {
				flush |= (1 << i);
				curr_pos[i] = iev[i * 2 + 1] | (iev[i * 2 + 2] << 8);
			}
			return axis_event(inp, curr_pos, &flush);
		case 2: /* Three axis... rX, rY, rZ */
			flush = 0x78;
			curr_pos[3] = iev[1] | (iev[2] << 8);
			curr_pos[4] = iev[3] | (iev[4] << 8);
			curr_pos[5] = iev[5] | (iev[6] << 8);
			return axis_event(inp, curr_pos, &flush);
		case 3: /* Button change event. */
			flush = 1;
			curr_buttons = iev[1] | (iev[2] << 8) | (iev[3] << 16);
			if(last_buttons != curr_buttons) {
				last_buttons = button_event(inp, last_buttons, curr_buttons);
				return 0;
			}
			return -1;
		case 23: /* Battery char level */
			logmsg(LOG_INFO, "Battery level: %%%d\n", iev[1]);
			break;
		default:
			if(verbose) {
				logmsg(LOG_DEBUG, "unhandled event: %d\n", iev[0]);
			}
			break;
		}
	}

	return -1;
}

int open_dev_usb(struct device *dev)
{
	if((dev->fd = open(dev->path, O_RDWR | O_NONBLOCK)) == -1) {
		if((dev->fd = open(dev->path, O_RDONLY | O_NONBLOCK)) == -1) {
			logmsg(LOG_ERR, "failed to open device: %s\n", strerror(errno));
			return -1;
		}
		logmsg(LOG_WARNING, "opened device read-only, LEDs won't work\n");
	}

	if(cfg.led == LED_ON || (cfg.led == LED_AUTO && first_client())) {
		set_led_hid(dev, 1);
	} else {
		/* Some devices start with the LED enabled, make sure to turn it off
		 * explicitly if necessary.
		 *
		 * XXX G.Ebner reports that some devices (SpaceMouse Compact at least)
		 * fail to turn their LED off at startup if it's not turned explicitly
		 * on first. We'll need to investigate further, but it doesn't seem to
		 * cause any visible blinking, so let's leave the redundant call to
		 * enable it first for now. See github pull request #39:
		 * https://github.com/FreeSpacenav/spacenavd/pull/39
		 */
		set_led_hid(dev, 1);
		set_led_hid(dev, 0);
	}

	/* fill the device function pointers */
	dev->close = close_hid;
	dev->read = read_hid;
	dev->set_led = set_led_hid;

	/* TODO until we flesh out the USB code on FreeBSD, let's fill the structure
	 * with fake but plausible information.
	 */
	dev->bnbase = 0;
	dev->num_buttons = 2;
	dev->num_axes = 6;

	return 0;
}

struct usb_dev_info *find_usb_devices(int (*match)(const struct usb_dev_info*))
{
	struct usb_dev_info *node, *devlist = 0;
	struct usb_device_info devinfo;
	glob_t gl;
	size_t si;
	int fd;

	if(verbose) {
		logmsg(LOG_INFO, "Device detection, checking \"/dev/uhid*\"\n");
	}

	if(glob("/dev/uhid*", 0, NULL, &gl) != 0) {
		return devlist;
	}

	for(si=0; si<gl.gl_pathc; si++) {
		logmsg(LOG_INFO, "checking \"%s\"... ", gl.gl_pathv[si]);

		if((fd = open(gl.gl_pathv[si], O_RDWR)) == -1) {
			logmsg(LOG_ERR, "Failed to open \"%s\": %s\n", gl.gl_pathv[si], strerror(errno));
			continue;
		}
		if(ioctl(fd, USB_GET_DEVICEINFO, &devinfo) != -1) {
			if((node = calloc(1, sizeof *node))) {
				node->vendorid = devinfo.udi_vendorNo;
				node->productid = devinfo.udi_productNo;
				node->name = strdup(devinfo.udi_product);
				if(node->name != NULL) {
					node->devfiles[0] = strdup(gl.gl_pathv[si]);
					if(node->devfiles[0] != NULL) {
						node->num_devfiles = 1;
					}
				}
			}
			if(!node || !node->num_devfiles) {
				logmsg(LOG_ERR, "failed to allocate usb device info node: %s\n", strerror(errno));
				free_usb_devices_list(node);
			} else if(verbose) {
				logmsg(LOG_INFO, "found usb device [%x:%x]: \"%s\" (%s) \n", node->vendorid, node->productid,
						node->name ? node->name : "unknown", node->devfiles[0]);
			}
			if(!match || match(node)) {
				if(verbose) {
					logmsg(LOG_INFO, "found usb device: ");
					print_usb_device_info(node);
				}

				node->next = devlist;
				devlist = node;
			}
		}
		close(fd);
	}
	globfree(&gl);

	return devlist;
}

#else
int spacenavd_dev_usb_freebsd_silence_empty_warning;
#endif	/* __FreeBSD__ */
