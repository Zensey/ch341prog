/*
 * This file is part of the ch341prog project.
 *
 * Copyright (C) 2014 Pluto Yang (yangyj.ee@gmail.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * verbose functionality forked from https://github.com/vSlipenchuk/ch341prog/commit/5afb03fe27b54dbcc88f6584417971d045dd8dab
 *
 */

#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <signal.h>
#include "ch341a.h"

int32_t bulkin_count;
struct libusb_device_handle *devHandle = NULL;
struct sigaction saold;
int force_stop = 0;

void v_print(int mode, int len) ;

/* SIGINT handler */
void sig_int(int signo)
{
    force_stop = 1;
}

/* Configure CH341A, find the device and set the default interface. */
int32_t ch341Configure(uint16_t vid, uint16_t pid)
{
    struct libusb_device *dev;
    int32_t ret;
    struct sigaction sa;

    uint8_t  desc[0x12];

    if (devHandle != NULL) {
        fprintf(stderr, "Call ch341Release before re-configure\n");
        return -1;
    }
    ret = libusb_init(NULL);
    if(ret < 0) {
        fprintf(stderr, "Couldn't initialise libusb\n");
        return -1;
    }

    libusb_set_debug(NULL, 3);

    if(!(devHandle = libusb_open_device_with_vid_pid(NULL, vid, pid))) {
        fprintf(stderr, "Couldn't open device [%04x:%04x].\n", vid, pid);
        return -1;
    }

    if(!(dev = libusb_get_device(devHandle))) {
        fprintf(stderr, "Couldn't get bus number and address.\n");
        goto close_handle;
    }

    if(libusb_kernel_driver_active(devHandle, 0)) {
        ret = libusb_detach_kernel_driver(devHandle, 0);
        if(ret) {
            fprintf(stderr, "Failed to detach kernel driver: '%s'\n", strerror(-ret));
            goto close_handle;
        }
    }

    ret = libusb_claim_interface(devHandle, 0);

    if(ret) {
        fprintf(stderr, "Failed to claim interface 0: '%s'\n", strerror(-ret));
        goto close_handle;
    }

    ret = libusb_get_descriptor(devHandle, LIBUSB_DT_DEVICE, 0x00, desc, 0x12);

    if(ret < 0) {
        fprintf(stderr, "Failed to get device descriptor: '%s'\n", strerror(-ret));
        goto release_interface;
    }

    printf("Device reported its revision [%d.%02d]\n", desc[12], desc[13]);
    sa.sa_handler = &sig_int;
    sa.sa_flags = SA_RESTART;
    sigfillset(&sa.sa_mask);
    if (sigaction(SIGINT, &sa, &saold) == -1) {
        perror("Error: cannot handle SIGINT"); // Should not happen
    }
    return 0;
release_interface:
    libusb_release_interface(devHandle, 0);
close_handle:
    libusb_close(devHandle);
    devHandle = NULL;
    return -1;
}

/* release libusb structure and ready to exit */
int32_t ch341Release(void)
{
    if (devHandle == NULL) return -1;
    libusb_release_interface(devHandle, 0);
    libusb_close(devHandle);
    libusb_exit(NULL);
    devHandle = NULL;
    sigaction(SIGINT, &saold, NULL);
    return 0;
}

/* Helper function for libusb_bulk_transfer, display error message with the caller name */
int32_t usbTransfer(const char * func, uint8_t type, uint8_t* buf, int len)
{
    int32_t ret;
    int transfered;
    if (devHandle == NULL) return -1;
    ret = libusb_bulk_transfer(devHandle, type, buf, len, &transfered, DEFAULT_TIMEOUT);
    if (ret < 0) {
        fprintf(stderr, "%s: Failed to %s %d bytes '%s'\n", func,
                (type == BULK_WRITE_ENDPOINT) ? "write" : "read", len, strerror(-ret));
        return -1;
    }
    return transfered;
}

/*   set the i2c bus speed (speed(b1b0): 0 = 20kHz; 1 = 100kHz, 2 = 400kHz, 3 = 750kHz)
 *   set the spi bus data width(speed(b2): 0 = Single, 1 = Double)  */
int32_t ch341SetStream(uint32_t speed) {
    uint8_t buf[3];

    if (devHandle == NULL) return -1;
    buf[0] = CH341A_CMD_I2C_STREAM;
    buf[1] = CH341A_CMD_I2C_STM_SET | (speed & 0x7);
    buf[2] = CH341A_CMD_I2C_STM_END;

    return usbTransfer(__func__, BULK_WRITE_ENDPOINT, buf, 3);
}

/* ch341 requres LSB first, swap the bit order before send and after receive  */
uint8_t swapByte(uint8_t c)
{
    uint8_t result=0;
    for (int i = 0; i < 8; ++i)
    {
        result = result << 1;
        result |= (c & 1);
        c = c >> 1;
    }
    return result;
}

/* assert or deassert the chip-select pin of the spi device */
void ch341SpiCs(uint8_t *ptr, bool selected)
{
    *ptr++ = CH341A_CMD_UIO_STREAM;
    *ptr++ = CH341A_CMD_UIO_STM_OUT | (selected ? 0x36 : 0x37);
    if (selected)
        *ptr++ = CH341A_CMD_UIO_STM_DIR | 0x3F; // pin direction
    *ptr++ = CH341A_CMD_UIO_STM_END;
}

/* transfer len bytes of data to the spi device */
int32_t ch341SpiStream(uint8_t *out, uint8_t *in, uint32_t len)
{
    uint8_t inBuf[CH341_PACKET_LENGTH], outBuf[CH341_PACKET_LENGTH], *inPtr, *outPtr;
    int32_t ret, packetLen;
    bool done;

    if (devHandle == NULL) return -1;

    ch341SpiCs(outBuf, true);
    ret = usbTransfer(__func__, BULK_WRITE_ENDPOINT, outBuf, 4);
    if (ret < 0) return -1;

    inPtr = in;

    do {
        done=true;
        packetLen=len+1;    // STREAM COMMAND + data length
        if (packetLen>CH341_PACKET_LENGTH) {
            packetLen=CH341_PACKET_LENGTH;
            done=false;
        }
        outPtr = outBuf;
        *outPtr++ = CH341A_CMD_SPI_STREAM;
        for (int i = 0; i < packetLen-1; ++i)
            *outPtr++ = swapByte(*out++);
        ret = usbTransfer(__func__, BULK_WRITE_ENDPOINT, outBuf, packetLen);
        if (ret < 0) return -1;
        ret = usbTransfer(__func__, BULK_READ_ENDPOINT, inBuf, packetLen-1);
        if (ret < 0) return -1;
        len -= ret;

        for (int i = 0; i < ret; ++i) // swap the buffer
            *inPtr++ = swapByte(inBuf[i]);
    } while (!done);

    ch341SpiCs(outBuf, false);
    ret = usbTransfer(__func__, BULK_WRITE_ENDPOINT, outBuf, 3);
    if (ret < 0) return -1;
    return 0;
}


/* read status register */
int32_t ch341ReadStatus(void)
{
    uint8_t out[2];
    uint8_t in[2];
    int32_t ret;

    if (devHandle == NULL) return -1;
    out[0] = 0x05; // Read status
    ret = ch341SpiStream(out, in, 2);
    if (ret < 0) return ret;
    return (in[1]);
}


int32_t ch341WriteEnable(void)
{
    uint8_t out[1];
    uint8_t in[1];
    int32_t ret;

    if (devHandle == NULL) return -1;
    out[0] = 0x06; // Write enable
    ret = ch341SpiStream(out, in, 1);
    if (ret < 0) return ret;
    return 0;
}


/* read the content of SPI device to buf, make sure the buf is big enough before call  */
int32_t ch341SpiRead(uint8_t *buf, uint32_t addr, uint32_t len)
{
    // cmd+addrH+addrL + data
    uint8_t out[3+32];
    uint8_t in[3+32];
    
    if (devHandle == NULL) return -1;
    
    printf("Read started!\n");
    while (len > 0)
    {
	int l=len;
	if (l > 32) l=32;
	
	// Making read command
	printf("Reading 0x%04x\r", addr);
	fflush(stdout);
	memset(out, 0x00, sizeof(out));
	out[0]=0x03;
	out[1]=(addr >> 8) & 0xff;
	out[2]=addr & 0xff;
	int ret=ch341SpiStream(out, in, 3+l);
	if (ret < 0) return ret;
        if (force_stop == 1)
        {
    	    // user hit ctrl+C
            force_stop = 0;
            if (len > 0)
                fprintf(stderr, "\nUser hit Ctrl+C, reading unfinished.\n");
            break;
        }
        
        // Copying data
        memcpy(buf, in+3, l);
        
        // Next data
        buf+=l;
        addr+=l;
        len-=l;
    }
    printf("Read done                          \n");
    return 0;
}

int32_t ch341SpiWrite(uint8_t *buf, uint32_t addr, uint32_t len)
{
    // cmd+addrH+addrL + data
    uint8_t out[3+32];
    uint8_t in[3+32];
    
    if (devHandle == NULL) return -1;
    
    printf("Write started!\n");
    while (len > 0)
    {
	int l=len;
	if (l > 32) l=32;
	
	// Enabling write
	int ret=ch341WriteEnable();
	if (ret < 0) return ret;
	
	// Making read command
	printf("Writing 0x%04x\r", addr);
	fflush(stdout);
	out[0]=0x02;
	out[1]=(addr >> 8) & 0xff;
	out[2]=addr & 0xff;
	memcpy(out+3, buf, l);
	ret=ch341SpiStream(out, in, 3+l);
	if (ret < 0) return ret;
        if (force_stop == 1)
        {
    	    // user hit ctrl+C
            force_stop = 0;
            if (len > 0)
                fprintf(stderr, "\nUser hit Ctrl+C, writing unfinished.\n");
            break;
        }
        
        // Waiting end
        int tries;
        for (tries=0; tries<10; tries++)
        {
    	    ret=ch341ReadStatus();
    	    if (ret < 0) return ret;
    	    if (ret & 0x01)
    	    {
    		//printf("Not ready\n");
    	    } else
    	    {
    		//printf("Ready !!!\n");
    		break;
    	    }
        }
        if (ret & 0x01)
        {
    	    printf("\nFlash not ready\n");
    	    break;
        }
        
        // Next data
        buf+=l;
        addr+=l;
        len-=l;
    }
    printf("Write done                  \n");
    return 0;
}
