/*
 * hid-ft260.c - FTDI FT260 HID USB to I2C master bridge
 * Copyright (c) 2018 Moxa, Inc.
 * Wesley Wu <wesleysy.wu@moxa.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

/*
 * The FTDI FT260 chip is a USB HID device which provides an
 * I2C controller for talking to slave devices. The
 * host communicates with the FT260 via raw HID reports.
 *
 * Data Sheet:
 *   http://www.ftdichip.com/Support/Documents/DataSheets/ICs/DS_FT260.pdf
 * Programming Interface Specification:
 *   http://www.ftdichip.com/Support/Documents/ProgramGuides/AN_394_User_Guide_for_FT260.pdf
 */

#include <linux/gpio/driver.h>
#include <linux/hid.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/nls.h>
#include <linux/usb/ch9.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0))
#include <linux/hidraw.h>
#endif

#define FT260_REPORT_MAX_LENGTH			64
/* clk speed range 60k~800kbps */
#define FT260_REPORT_MAX_SPEED			400

/* ID for FTDI:FT260 */
#define USB_VENDOR_ID_FTDI			0x0403
#define USB_DEVICE_ID_FTDI_FT260		0x6030

#define FT260_I2C_STATUS_CONTROLLER_BUSY	0x00
#define FT260_I2C_STATUS_ERROR			0x01
#define FT260_I2C_STATUS_SLAVE_NOT_ACK		0x02
#define FT260_I2C_STATUS_DATA_NOT_ACK		0x04
#define FT260_I2C_STATUS_ARBITRATION_LOST	0x08
#define FT260_I2C_STATUS_CONTROLLER_IDLE	0x10
#define FT260_I2C_STATUS_BUS_BUSY		0x20

enum {
	FT260_F_CHIP_CODE			= 0xA0,
	FT260_F_SYSTEM_SETTING			= 0xA1,
	FT260_F_GPIO				= 0xB0,
	FT260_I_UART_INT_STATUS 		= 0xB1,
	FT260_F_I2C_STATUS			= 0xC0,
	FT260_O_I2C_READ_REQUEST		= 0xC2,
	FT260_IO_I2C_RW_REPORT_BASE		= 0xD0,
	FT260_F_UART_STATUS			= 0xE0,
	FT260_F_UART_RI_DCD_STATUS		= 0xE2,
	FT260_IO_UART_RW_REPORT_BASE		= 0xF0,
};

enum {
	FT260_I2C_RW_FLAG_NONE			= 0,
	FT260_I2C_RW_FLAG_START			= 0x02,
	FT260_I2C_RW_FLAG_REP_START		= 0x03,
	FT260_I2C_RW_FLAG_STOP			= 0x04,
	FT260_I2C_RW_FLAG_START_AND_STOP	= 0x06,
};

enum {
	PCA9535_I2C_CMD_IN_PORT0		= 0x0,
	PCA9535_I2C_CMD_IN_PORT1		= 0x1,
	PCA9535_I2C_CMD_OUT_PORT0		= 0x02,
	PCA9535_I2C_CMD_OUT_PORT1		= 0x03,
	PCA9535_I2C_CMD_CONF_PORT0		= 0x06,
	PCA9535_I2C_CMD_CONF_PORT1		= 0x07,
	PCA9535_I2C_DEV_ADDR0			= 0x26,
	PCA9535_I2C_DEV_ADDR1			= 0x27,
};

struct ft260_write_req_report {
	u8 report;		/* FT260_IO_I2C_RW_REPORT_BASE + (len - 1) / 4 */
	u8 slave_address;	/* 7-bit slave address */
	u8 flag;		/* FT260_I2C_WRITE_FLAG */
	u8 length;
	u8 data[61];
} __packed;

struct ft260_read_req_report {
	u8 report;		/* FT260_O_I2C_READ_REQUEST */
	u8 slave_address;
	u8 flag;
	__le16 length;
} __packed;

/* Number of times to request transfer status before giving up waiting for a
   transfer to complete. This may need to be changed if SMBUS clock, retries,
   or read/write/scl_low timeout settings are changed. */
static const int XFER_STATUS_RETRIES = 10;

/* Time in ms to wait for a CP2112_DATA_READ_RESPONSE or
   CP2112_TRANSFER_STATUS_RESPONSE. */
static const int RESPONSE_TIMEOUT = 50;

static const struct hid_device_id ft260_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_FTDI, USB_DEVICE_ID_FTDI_FT260) },
	{ }
};
MODULE_DEVICE_TABLE(hid, ft260_devices);

struct ft260_device {
	struct i2c_adapter adap;
	struct hid_device *hdev;
	wait_queue_head_t wait;
	u8 read_data[62];
	u8 read_length;
	u8 hwversion;
	int xfer_status;
	atomic_t read_avail;
	struct mutex lock;
};

static int ft260_hid_get(struct hid_device *hdev, unsigned char report_number,
			  u8 *data, size_t count, unsigned char report_type)
{
	u8 *buf;
	int ret;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = hid_hw_raw_request(hdev, report_number, buf, count,
				       report_type, HID_REQ_GET_REPORT);
	memcpy(data, buf, count);
	kfree(buf);
	return ret;
}

static int ft260_hid_output(struct hid_device *hdev, u8 *data, size_t count,
			     unsigned char report_type)
{
	u8 *buf;
	int ret;

	buf = kmemdup(data, count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (report_type == HID_OUTPUT_REPORT)
		ret = hid_hw_output_report(hdev, buf, count);
	else
		ret = hid_hw_raw_request(hdev, buf[0], buf, count, report_type,
				HID_REQ_SET_REPORT);

	kfree(buf);
	return ret;
}

static int ft260_wait(struct ft260_device *dev, atomic_t *avail)
{
	int ret = 0;

	/* We have sent either a CP2112_TRANSFER_STATUS_REQUEST or a
	 * CP2112_DATA_READ_FORCE_SEND and we are waiting for the response to
	 * come in cp2112_raw_event or timeout. There will only be one of these
	 * in flight at any one time. The timeout is extremely large and is a
	 * last resort if the CP2112 has died. If we do timeout we don't expect
	 * to receive the response which would cause data races, it's not like
	 * we can do anything about it anyway.
	 */
	ret = wait_event_interruptible_timeout(dev->wait,
		atomic_read(avail), msecs_to_jiffies(RESPONSE_TIMEOUT));
	if (-ERESTARTSYS == ret)
		return ret;
	if (!ret)
		return -ETIMEDOUT;

	atomic_set(avail, 0);
	return 0;
}

static int ft260_xfer_status(struct ft260_device *dev)
{
	struct hid_device *hdev = dev->hdev;
	u8 buf[5];
	int ret;
	u8 status = 0;

	/* Synchronize IO: Get report in */
	ret = ft260_hid_get(hdev, FT260_F_I2C_STATUS, (u8 *)buf, sizeof(*buf),
			     HID_FEATURE_REPORT);

	dev->xfer_status = -EINVAL;

	if (buf[0] == FT260_F_I2C_STATUS) {
		if (status & (FT260_I2C_STATUS_CONTROLLER_BUSY || FT260_I2C_STATUS_BUS_BUSY)) {
			dev->xfer_status = -EBUSY;
		} else if (status & (FT260_I2C_STATUS_ERROR || FT260_I2C_STATUS_ARBITRATION_LOST)) {
			dev->xfer_status = -EIO;
		} else if (status & (FT260_I2C_STATUS_SLAVE_NOT_ACK || FT260_I2C_STATUS_DATA_NOT_ACK)) {
			dev->xfer_status = -ETIMEDOUT;
		} else {
			dev->xfer_status = 0;
		}
		/* FT260_I2C_STATUS_CONTROLLER_IDLE */
	}

	return dev->xfer_status;
}

static int ft260_read(struct ft260_device *dev, u8 *data, size_t size)
{
	struct hid_device *hdev = dev->hdev;
	int ret;

	if (size > sizeof(dev->read_data))
		size = sizeof(dev->read_data);

	ret = ft260_wait(dev, &dev->read_avail);
	if (ret)
		return ret;

	hid_dbg(hdev, "read %d of %zd bytes requested\n",
		dev->read_length, size);

	if (size > dev->read_length)
		size = dev->read_length;

	mutex_lock(&dev->lock);
	memcpy(data, dev->read_data, size);
	mutex_unlock(&dev->lock);

	return dev->read_length;
}

static int ft260_read_req(void *buf, u8 slave_address, u16 length)
{
	struct ft260_read_req_report *report = buf;

	/* Shall we have a limitation to READ size ? */
	if (length < 1 || length > 512)
		return -EINVAL;

	report->report = FT260_O_I2C_READ_REQUEST;
	report->slave_address = slave_address;
	report->flag = FT260_I2C_RW_FLAG_START_AND_STOP;
	report->length = cpu_to_le16(length);

	return sizeof(*report);
}

static int ft260_write_req(void *buf, u8 slave_address, u8 command,
				u8 *data, u8 data_length)
{
	struct ft260_write_req_report *report = buf;

	if (data_length > sizeof(report->data) - 1)
		return -EINVAL;

	report->report = FT260_IO_I2C_RW_REPORT_BASE;
	report->slave_address = slave_address;
	report->flag = FT260_I2C_RW_FLAG_START_AND_STOP;
	report->length = data_length + 1;
	report->data[0] = command;
	memcpy(&report->data[1], data, data_length);

	return data_length + 5;
}

static int ft260_i2c_write_req(void *buf, u8 slave_address, u8 *data,
				u8 data_length)
{
	struct ft260_write_req_report *report = buf;
	u8 report_id = FT260_IO_I2C_RW_REPORT_BASE;

	if (data_length > sizeof(report->data))
		return -EINVAL;

	report_id += (data_length - 1) / 4;

	report->report = report_id;
	report->flag = FT260_I2C_RW_FLAG_START_AND_STOP;	/* TODO: If data size > 60 */
	report->slave_address = slave_address;
	report->length = data_length;
	memcpy(report->data, data, data_length);
	return data_length + 4;
}

static int ft260_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			   int num)
{
	struct ft260_device *dev = (struct ft260_device *)adap->algo_data;
	struct hid_device *hdev = dev->hdev;
	u8 buf[64];
	ssize_t count;
	ssize_t read_length = 0;
	u8 *read_buf = NULL;
	unsigned int retries;
	int ret;
	struct i2c_msg *cur_msg = NULL;
	int m;

	hid_dbg(hdev, "I2C %d messages\n", num);

	if (num <= 0) {
		hid_err(hdev,
			"Zero I2C transaction....weird\n");
		return -EOPNOTSUPP;
	}

	if (num > 2) {
		hid_err(hdev,
			"Multi-message I2C transactions not supported\n");
		return -EOPNOTSUPP;
	}

	ret = hid_hw_power(hdev, PM_HINT_FULLON);
	if (ret < 0) {
		hid_err(hdev, "power management error: %d\n", ret);
		return ret;
	}

	for (m = 0; m < num; ++m) {
		cur_msg = msgs + m;

		if (cur_msg->flags & I2C_M_RD) {
			hid_dbg(hdev, "I2C read %#04x len %d\n",
				cur_msg->addr, cur_msg->len);
			read_length = cur_msg->len;
			read_buf = cur_msg->buf;
			count = ft260_read_req(buf, cur_msg->addr, cur_msg->len);
			atomic_set(&dev->read_avail, 0);

		} else {
			hid_dbg(hdev, "I2C write %#04x len %d\n",
				cur_msg->addr, cur_msg->len);
			count = ft260_i2c_write_req(buf, cur_msg->addr,
						     cur_msg->buf, cur_msg->len);
		}
		if (count < 0)
			return count;

		ret = ft260_hid_output(hdev, buf, count, HID_OUTPUT_REPORT);
		if (ret < 0) {
			hid_warn(hdev, "Error starting transaction: %d\n", ret);
			goto power_normal;
		}

		for (retries = 0; retries < XFER_STATUS_RETRIES; ++retries) {
			ret = ft260_xfer_status(dev);
			if (-EBUSY == ret)
				continue;
			if (ret < 0)
				goto power_normal;
			break;
		}

		if (XFER_STATUS_RETRIES <= retries) {
			hid_warn(hdev, "Transfer timed out.\n");
			ret = -ETIMEDOUT;
			goto power_normal;
		}

		/* Handle the READ */
		for (count = 0; count < read_length;) {
			ret = ft260_read(dev, read_buf + count, read_length - count);
			if (ret < 0)
				goto power_normal;

			if (ret == 0) {
				hid_err(hdev, "read returned 0\n");
				ret = -EIO;
				goto power_normal;
			}

			count += ret;
			if (count > read_length) {
				/*
				 * The hardware returned too much data.
				 * This is mostly harmless because cp2112_read()
				 * has a limit check so didn't overrun our
				 * buffer.  Nevertheless, we return an error
				 * because something is seriously wrong and
				 * it shouldn't go unnoticed.
				 */
				hid_err(hdev, "long read: %d > %zd\n",
					ret, read_length - count + ret);
				ret = -EIO;
				goto power_normal;
			}
		}
	}

	/* return the number of transferred messages */
	ret = num;

power_normal:
	hid_hw_power(hdev, PM_HINT_NORMAL);
	hid_dbg(hdev, "I2C transfer finished: %d\n", ret);
	return ret;
}

static int ft260_xfer(struct i2c_adapter *adap, u16 addr,
		       unsigned short flags, char read_write, u8 command,
		       int size, union i2c_smbus_data *data)
{
	struct ft260_device *dev = (struct ft260_device *)adap->algo_data;
	struct hid_device *hdev = dev->hdev;
	u8 buf[64];
	u8 reg_data[2];
	ssize_t count;
	size_t read_length = 0;
	__le16 word;
	int ret;

	hid_dbg(hdev, "%s addr 0x%x flags 0x%x cmd 0x%x size %d\n",
		read_write == I2C_SMBUS_WRITE ? "write" : "read",
		addr, flags, command, size);

	switch (size) {
	case I2C_SMBUS_BYTE:
		read_length = 1;
		if (I2C_SMBUS_READ == read_write) {
			count = ft260_read_req(buf, addr, read_length);
		} else {
			count = ft260_write_req(buf, addr, command, &data->byte, read_length);
		}
		break;
	case I2C_SMBUS_BYTE_DATA:
		if (I2C_SMBUS_READ == read_write) {
			read_length = 2;
			/* select channel */
			count = ft260_write_req(buf, addr, command, 0, 0);
			ret = ft260_hid_output(hdev, buf, count, HID_OUTPUT_REPORT);
			if (ret < 0) {
				hid_warn(hdev, "Error starting transaction: %d\n", ret);
				goto power_normal;
			}

			count = ft260_read_req(buf, addr, read_length);
		} else {
			read_length = 1;
			count = ft260_write_req(buf, addr, command, &data->byte, read_length);
			if (count < 0)
				return count;
		}
		break;
	case I2C_SMBUS_WORD_DATA:
		read_length = 2;
		word = cpu_to_le16(data->word);

		if (I2C_SMBUS_READ == read_write) {
			count = ft260_read_req(buf, addr, read_length);
		} else {
			count = ft260_write_req(buf, addr, command, reg_data, read_length);
		}
		break;
	default:
		hid_warn(hdev, "Unsupported transaction %d\n", size);
		return -EOPNOTSUPP;
	}

	if (count < 0)
		return count;

	ret = hid_hw_power(hdev, PM_HINT_FULLON);
	if (ret < 0) {
		hid_err(hdev, "power management error: %d\n", ret);
		return ret;
	}

	/* Send HID report */
	ret = ft260_hid_output(hdev, buf, count, HID_OUTPUT_REPORT);
	if (ret < 0) {
		hid_warn(hdev, "Error starting transaction: %d\n", ret);
		goto power_normal;
	}

	if (I2C_SMBUS_READ == read_write) {
		ret = ft260_read(dev, buf, read_length);

		if (ret < 0)
			goto power_normal;

		if (ret != read_length) {
			hid_warn(hdev, "short read: %d < %zd\n", ret, read_length);
			ret = -EIO;
			goto power_normal;
		}

		switch (size) {
		case I2C_SMBUS_BYTE:
		case I2C_SMBUS_BYTE_DATA:
			data->byte = buf[0];
			break;
		case I2C_SMBUS_WORD_DATA:
			data->word = le16_to_cpup((__le16 *)buf);
			break;
		}
	}

	ret = 0;
power_normal:
	hid_hw_power(hdev, PM_HINT_NORMAL);
	hid_dbg(hdev, "transfer finished: %d\n", ret);
	return ret;
}

static u32 ft260_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA;
}

static const struct i2c_algorithm ft260_i2c_algorithm = {
	.master_xfer	= ft260_i2c_xfer,
	.smbus_xfer	= ft260_xfer,
	.functionality	= ft260_functionality,
};

static int ft260_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct ft260_device *dev;
	u8 buf[13];
	u8 config_buf[4];
	int ret;

	dev = devm_kzalloc(&hdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	mutex_init(&dev->lock);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		return ret;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		return ret;
	}

	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "hw open failed\n");
		goto err_hid_stop;
	}

	ret = hid_hw_power(hdev, PM_HINT_FULLON);
	if (ret < 0) {
		hid_err(hdev, "power management error: %d\n", ret);
		goto err_hid_close;
	}

	ret = ft260_hid_get(hdev, FT260_F_CHIP_CODE, buf, sizeof(buf),
			     HID_FEATURE_REPORT);
	if (ret != sizeof(buf)) {
		hid_err(hdev, "error requesting version\n");
		if (ret >= 0)
			ret = -EIO;
		goto err_power_normal;
	}

	hid_info(hdev, "Part Number: 0x%02X%02X Device Version: 0x%02X%02X\n",
		buf[1], buf[2], buf[3], buf[4]);	/* Wesley: Todo, check the endian ?? */

	/* Set Feature */
	config_buf[0] = 0xA1;
	/* request I2C Clock Speed */
	config_buf[1] = 0x22;
	/* Set I2C clock speed, whose range is from 60K bps to 3400K bps. */
	config_buf[2] = FT260_REPORT_MAX_SPEED & 0x00ff; /* MSB */
	config_buf[3] = FT260_REPORT_MAX_SPEED >> 8; /* LSB */

	ret = ft260_hid_output(hdev, config_buf, sizeof(config_buf), HID_FEATURE_REPORT);
	if (ret != sizeof(config_buf)) {
		hid_err(hdev, "error setting SMBus config\n");
		if (ret >= 0)
			ret = -EIO;
		goto err_power_normal;
	}
	hid_info(hdev, "set FTDI FT260 feature as 0x%02X%02X%02X%02X\n",
		config_buf[0], config_buf[1], config_buf[2], config_buf[3]);

	hid_set_drvdata(hdev, (void *)dev);
	dev->hdev		= hdev;
	dev->adap.owner		= THIS_MODULE;
	dev->adap.class		= I2C_CLASS_HWMON;
	dev->adap.algo		= &ft260_i2c_algorithm;
	dev->adap.algo_data	= dev;
	dev->adap.dev.parent	= &hdev->dev;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0))
	snprintf(dev->adap.name, sizeof(dev->adap.name),
		 "FT260 I2C Bridge on hiddev%d", ((struct hidraw *)hdev->hidraw)->minor);
#else
	snprintf(dev->adap.name, sizeof(dev->adap.name),
		 "FT260 I2C Bridge on hiddev%d", hdev->minor);
#endif

	dev->hwversion = buf[2];		/* Wesley: What should we set for this ?? */
	init_waitqueue_head(&dev->wait);

	hid_device_io_start(hdev);
	ret = i2c_add_adapter(&dev->adap);
	hid_device_io_stop(hdev);

	if (ret) {
		hid_err(hdev, "error registering i2c adapter\n");
		goto err_power_normal;
	}

	hid_dbg(hdev, "adapter registered\n");
	hid_hw_power(hdev, PM_HINT_NORMAL);

	hid_info(hdev, "FT260 driver was probed\n");
	return ret;

err_power_normal:
	hid_hw_power(hdev, PM_HINT_NORMAL);
err_hid_close:
	hid_hw_close(hdev);
err_hid_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void ft260_remove(struct hid_device *hdev)
{
	struct ft260_device *dev = hid_get_drvdata(hdev);
	i2c_del_adapter(&dev->adap);
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static int ft260_raw_event(struct hid_device *hdev, struct hid_report *report,
			    u8 *data, int size)
{
	struct ft260_device *dev = hid_get_drvdata(hdev);

	hid_dbg(hdev, "ft260_raw_event: %02x %02x %02x\n", data[0], data[1], data[2]);

	if (data[0] >= FT260_IO_I2C_RW_REPORT_BASE &&
		data[0] <= (FT260_IO_I2C_RW_REPORT_BASE + 0xE)) {
		/* I2C Input Report */
		dev->read_length = data[1];

		if (dev->read_length > sizeof(dev->read_data))
				dev->read_length = sizeof(dev->read_data);

		mutex_lock(&dev->lock);
		memcpy(dev->read_data, &data[2], dev->read_length);
		mutex_unlock(&dev->lock);

		atomic_set(&dev->read_avail, 1);

	} else if (data[0] >= FT260_IO_UART_RW_REPORT_BASE &&
				data[0] <= (FT260_IO_UART_RW_REPORT_BASE + 0xE)) {
		/* UART Input Report .... not supported */
		hid_err(hdev, "uart input report ... not supported yet\n");
		return 0;
	} else {
		hid_err(hdev, "unknown report\n");
		return 0;
	}

	wake_up_interruptible(&dev->wait);
	return 1;
}

static struct hid_driver ft260_driver = {
	.name		= "ft260",
	.id_table	= ft260_devices,
	.probe		= ft260_probe,
	.remove		= ft260_remove,
	.raw_event	= ft260_raw_event,
};

module_hid_driver(ft260_driver);
MODULE_DESCRIPTION("FTDI FT260 HID USB to I2C master bridge");
MODULE_AUTHOR("Wesley Wu <wesleysy.wu@moxa.com>");
MODULE_LICENSE("GPL");
