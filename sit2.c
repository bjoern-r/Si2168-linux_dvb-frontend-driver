/*
    SIT2  - DVB-T2/T/C demodulator and tuner

    Copyright (C) 2013 Max Nibble <nibble.max@gmail.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <asm/div64.h>
#include "dvb_frontend.h"
#include "sit2_priv.h"
#include "sit2.h"

int sit2_debug = 0;
module_param(sit2_debug, int, 0644);
MODULE_PARM_DESC(sit2_debug, "Activates frontend debugging (default:0)");
#define dprintk(args...) \
	do { \
		if (sit2_debug) \
			printk(KERN_INFO "sit2: " args); \
	} while (0)

/*global state*/
struct sit2_state {
	struct dvb_frontend frontend;
	const struct sit2_config *config;
	struct i2c_adapter *i2c;
	
	bool  isInited;
	u8 demod_addr;
	u8 tuner_addr;
	u8 sndBuffer[64];
	u8 revBuffer[64];
	sit2_tuner_reply tuner_reply;
	sit2_demod_reply demod_reply;

	fe_delivery_system_t current_system;
	int plp_id;
	u32 stream;
	u32 dvbc_symrate;	
};

static u32 sit2_writebytes(struct sit2_state *state, u32 len, u8 *data, bool isTuner)
{
	int ret;
	u32 uret = 0;
	struct i2c_msg w_msg = { .flags = 0 };
	w_msg.addr = (isTuner) ? state->tuner_addr : state->demod_addr;;
	w_msg.buf = data;
	w_msg.len = len;
	ret = i2c_transfer(state->i2c, &w_msg, 1);
	if(ret != 1) {
		printk(KERN_INFO
	     	"%s: error! addr=%x len=%d, ret=%d\n",
	     	__func__, w_msg.addr, len, ret);
	}
	else
		uret = len;		
	return uret;
}

static u32 sit2_readbytes(struct sit2_state *state, u32 len, u8 *data, bool isTuner)
{
	int ret;
	u32 uret = 0;
	struct i2c_msg r_msg = { .flags = I2C_M_RD };
	r_msg.addr = (isTuner) ? state->tuner_addr : state->demod_addr;
	r_msg.len = len;
	r_msg.buf = data;
	ret = i2c_transfer(state->i2c, &r_msg, 1);
	if(ret != 1) {
		printk(KERN_INFO
	     	"%s: error! addr=%x len=%d, ret=%d\n",
	     	__func__, r_msg.addr, len, ret);
	}
	else
		uret = len;		
	return uret;
}

static u8 sit2_tuner_ResponseStatus(struct sit2_state *state, u8 Data)
{
    	state->tuner_reply.tunint = (Data & 0x01) ? 1 : 0;
    	state->tuner_reply.atvint = (Data & 0x02) ? 1 : 0;
    	state->tuner_reply.dtvint = (Data & 0x04) ? 1 : 0;
    	state->tuner_reply.err    = (Data & 0x40) ? 1 : 0;
    	state->tuner_reply.cts    = (Data & 0x80) ? 1 : 0;
  	return (state->tuner_reply.err ? SIT2_ERROR_ERR : SIT2_ERROR_OK);
}

static u8 sit2_demod_ResponseStatus(struct sit2_state *state, u8 Data)
{
	state->demod_reply.ddint   = (Data & 0x01) ? 1 : 0;
	state->demod_reply.scanint = (Data & 0x02) ? 1 : 0;
	state->demod_reply.err     = (Data & 0x40) ? 1 : 0;
	state->demod_reply.cts     = (Data & 0x80) ? 1 : 0;
	return (state->demod_reply.err ? SIT2_ERROR_ERR : SIT2_ERROR_OK);
}

static u8 sit2_pollForResponse(struct sit2_state *state, u32 nbBytes, u8 *pByteBuffer, bool isTuner)
{
	u32 ulCount, ulTick, ulDelay;
	ulCount = 0;
	ulTick = 50;
	ulDelay = 1000/ulTick;
	
	while(ulCount <= ulTick) {
		if (sit2_readbytes(state, nbBytes, pByteBuffer, isTuner) != nbBytes) {
      			dprintk("%s: tuner[%d], readbytes[%d] error!\n", __func__, isTuner, nbBytes);
      			return SIT2_ERROR_POLLING;
    		}
    		/* return response err flag if CTS set */
    		if (pByteBuffer[0] & 0x80)  {
    			if (isTuner)
    				return sit2_tuner_ResponseStatus(state, pByteBuffer[0]);
    			else
      				return sit2_demod_ResponseStatus(state, pByteBuffer[0]);
    		}
    		msleep(ulDelay);
    		ulCount++;
  	}

  	dprintk("%s: tuner[%d], time out error!\n", __func__, isTuner);
  	return SIT2_ERROR_TIMEOUT;
}

static u8 sit2_sendCommand(struct sit2_state *state, u32 sndBytes, u32 revBytes, bool isTuner)
{
	u8 uret = SIT2_ERROR_OK;
	if ((sndBytes > 64) || (revBytes > 64)) {
		printk(KERN_INFO
	     	"%s: error! sndBytes=%x revBytes=%d\n",
	     	__func__, sndBytes, revBytes);
	     	return SIT2_ERROR_PAREMETER;	
	}
	
	if (sit2_writebytes(state, sndBytes, state->sndBuffer, isTuner) != sndBytes) {
		
		dprintk("%s: tuner[%d],writebytes[%d] error!\n", __func__, isTuner, sndBytes);
		return SIT2_ERROR_ERR;
	}
	
	if(revBytes > 0)
		uret = sit2_pollForResponse(state, revBytes, state->revBuffer, isTuner);
	return uret;	
}

static u8 sit2_sendProperty(struct sit2_state *state, u32 prop, u32 data, bool isTuner)
{
	state->sndBuffer[0] = 0x14;
	state->sndBuffer[1] = 0;
	state->sndBuffer[2] = (u8)(prop & 0xff);
	state->sndBuffer[3] = (u8)((prop >> 8) & 0xff);
	state->sndBuffer[4] = (u8)(data & 0xff);
	state->sndBuffer[5] = (u8)((data >> 8) & 0xff);	
	return sit2_sendCommand(state, 6, 4, isTuner);
}

static u8 sit2_startFirmware(struct sit2_state *state, bool isTuner)
{
	state->sndBuffer[0] = 0x01;
	state->sndBuffer[1] = 1;
	return sit2_sendCommand(state, 2, 1, isTuner);	
}

static u8 sit2_demod_tuner_i2c_enable(struct sit2_state *state, u8 onOff)
{
	dprintk("%s, on=%d\n", __func__, onOff);
	state->sndBuffer[0] = 0xc0;
	state->sndBuffer[1] = 13;
	state->sndBuffer[2] = (onOff > 0) ? 1 : 0;
	return sit2_sendCommand(state, 3, 0, false);	
}

static u8 sit2_tuner_xout_enable(struct sit2_state *state, u8 onOff)
{
	state->sndBuffer[0] = 0xc0;
	state->sndBuffer[1] = 0;
	state->sndBuffer[2] = (onOff > 0) ? (3 << 2) : 0;
	return sit2_sendCommand(state, 3, 1, true);
}

static u8 sit2_tuner_enable_FEF(struct sit2_state *state, u8 fef)
{
	u8 uret = SIT2_ERROR_OK;
	if(fef)
		uret = sit2_sendProperty(state, 0x0711, 3, true);
	else
		uret = sit2_sendProperty(state, 0x0711, 1, true);
	return uret;
}

static u8 sit2_tuner_setup_FEFMode(struct sit2_state *state, u8 fef)
{
	u8 uret = SIT2_ERROR_OK;
	state->sndBuffer[0] = 0x12;
	state->sndBuffer[1] = 1;
	state->sndBuffer[2] = 1;
	state->sndBuffer[3] = 1;
	state->sndBuffer[4] = 1;
	state->sndBuffer[5] = 1;	
	uret = sit2_sendCommand(state, 6, 6, true);
	
	sit2_sendProperty(state, 0x070e, 0, true);	
	sit2_sendProperty(state, 0x0708, 0, true);
	
	sit2_tuner_enable_FEF(state, fef);

	return uret;
}

static u8 sit2_tuner_standby(struct sit2_state *state)
{
	state->sndBuffer[0] = 0x16;
	state->sndBuffer[1] = 0;
	return sit2_sendCommand(state, 2, 1, true);
}

static u8 sit2_tuner_powerUp(struct sit2_state *state)
{
	state->sndBuffer[0] = 0xc0;
	state->sndBuffer[1] = 0;
	state->sndBuffer[2] = 0;
	state->sndBuffer[3] = 0;
	state->sndBuffer[4] = 0;
	state->sndBuffer[5] = 1;
	state->sndBuffer[6] = 1;
	state->sndBuffer[7] = 1;
	state->sndBuffer[8] = 1;
	state->sndBuffer[9] = 1;
	state->sndBuffer[10] = 1;
	state->sndBuffer[11] = 2; /* 24MHz */
	state->sndBuffer[12] = 0;
	state->sndBuffer[13] = 0;
	state->sndBuffer[14] = 1;
	
	return sit2_sendCommand(state, 15, 1, true);
}

static u8 sit2_tuner_getStatus(struct sit2_state *state, u8 intack)
{
	state->sndBuffer[0] = 0x42;
	state->sndBuffer[1] = intack & 0x01;
	return sit2_sendCommand(state, 2, 12, true);
}

static u8 sit2_tuner_wakeUp(struct sit2_state *state)
{
	u8 uret, status = 0;
	/* check CTS */
	uret = sit2_pollForResponse(state, 1, &status, true);
	if((uret == SIT2_ERROR_TIMEOUT) || (status & 0x80) != 0x80) {
		printk(KERN_INFO
	     	"%s: error! tuner is not ready.\n",
	     	__func__);		
		return SIT2_ERROR_ERR;
	}
	return SIT2_ERROR_OK;	
}

static u8 sit2_tuner_tuneFreq(struct sit2_state *state, u32 frequency)
{
	u8 uret, status;
	int timeout = 150;
	u32 ulCount, ulTick, ulDelay;
	ulCount = 0;
	ulTick = 3;
	ulDelay = timeout/ulTick;
	
	state->sndBuffer[0] = 0x41;
	state->sndBuffer[1] = 0;
	state->sndBuffer[2] = 0;
	state->sndBuffer[3] = 0;
	
	state->sndBuffer[4] = (u8)(frequency & 0xff);
	state->sndBuffer[5] = (u8)((frequency >> 8) & 0xff);
	state->sndBuffer[6] = (u8)((frequency >> 16) & 0xff);
	state->sndBuffer[7] = (u8)((frequency >> 24) & 0xff);
	
	uret = sit2_sendCommand(state, 8, 1, true);
	if(uret != SIT2_ERROR_OK)
		return uret;
    		
	while(ulCount <= ulTick) {
		uret = sit2_pollForResponse(state, 1, &status, true);
		if(uret != SIT2_ERROR_OK)
			return uret;
		if(state->tuner_reply.tunint)
			break;
		msleep(ulDelay);
		ulCount++;		
	}
	if(state->tuner_reply.tunint == 0)
		return SIT2_ERROR_TIMEOUT;
		
	timeout = 20;
	ulCount = 0;
	ulTick = 2;
	ulDelay = timeout/ulTick;
	while ( ulCount <= ulTick ) {
		uret = sit2_pollForResponse(state, 1, &status, true);
		if(uret != SIT2_ERROR_OK)
			return uret;
		if(state->tuner_reply.dtvint)
			break;
		msleep(ulDelay);
		ulCount++;					
	}	
	if(state->tuner_reply.dtvint == 0)
		return SIT2_ERROR_TIMEOUT;
	
	return SIT2_ERROR_OK;
}

static u8 sit2_tuner_setFreq(struct sit2_state *state, u32 frequency, fe_delivery_system_t delsystem, u8 reqBW)
{
	u8 modulation, bandwidth;
	u8 uret;
	sit2_tuner_enable_FEF(state, 0);
	if (delsystem == SYS_DVBC_ANNEX_A) {
		modulation = 3;
		bandwidth = 8;
	} else {
		modulation = 2;
		bandwidth = (reqBW <= 6) ? 6 : reqBW;
		if(bandwidth > 8)
			bandwidth = 8;
	}
	uret = sit2_sendProperty(state, 0x0703, (modulation << 4) | bandwidth, true);
	if(uret != SIT2_ERROR_OK)
		return uret;
	
	uret = sit2_tuner_tuneFreq(state, frequency);
	if(delsystem == SYS_DVBT2)
		sit2_tuner_enable_FEF(state, 1);
			
	return uret;
}

static u8 sit2_tuner_init(struct sit2_state *state)
{
	u8 uret = SIT2_ERROR_OK;
		
	/* wake up */
	uret = sit2_tuner_wakeUp(state);
	if(uret != SIT2_ERROR_OK)
		return uret;
	/* power up */
	uret = sit2_tuner_powerUp(state);
	if(uret != SIT2_ERROR_OK)
		return uret;
	/* check part info */
	/* load firmware */
	/* start firmare */
	uret = sit2_startFirmware(state, true);
	if(uret != SIT2_ERROR_OK)
		return uret;
	
	/* download default properties */
	/* ATV property */
	sit2_sendProperty(state, 0x0610, 1000, true); /* afc range*/
	sit2_sendProperty(state, 0x0611, 0, true); /* agc speed */
	sit2_sendProperty(state, 0x0623, (0x80 << 8) | 158, true); /* agc speed low rssi */
	sit2_sendProperty(state, 0x0624, 0, true);
	sit2_sendProperty(state, 0x0603, 8, true);
	sit2_sendProperty(state, 0x0607, (200 << 8) | 50, true);
	sit2_sendProperty(state, 0x0601, 1, true);
	sit2_sendProperty(state, 0x0613, (1 << 9) | (1 << 8) | (0 << 1) | 0, true);
	sit2_sendProperty(state, 0x060c, 5000, true);
	sit2_sendProperty(state, 0x060d, (100 << 8) | 148, true);
	sit2_sendProperty(state, 0x0617, 0, true);
	sit2_sendProperty(state, 0x0612, 0, true);
	sit2_sendProperty(state, 0x0605, 0xba, true);
	sit2_sendProperty(state, 0x0604, (1 << 9), true);
	sit2_sendProperty(state, 0x0616, 0, true);
	/* common property */
	sit2_sendProperty(state, 0x0402, 8, true);
	sit2_sendProperty(state, 0x0401, 0, true);
	/* DTV property */
	sit2_sendProperty(state, 0x0711, 0, true); /* agc freeze pin */
	sit2_sendProperty(state, 0x0708, 0, true);
	sit2_sendProperty(state, 0x0702, 1, true);
	sit2_sendProperty(state, 0x0705, (200 << 8) | 50, true);
	sit2_sendProperty(state, 0x070c, 1, true);
	sit2_sendProperty(state, 0x0701, 1, true);
	sit2_sendProperty(state, 0x070d, 0, true);
	sit2_sendProperty(state, 0x070e, 0, true);
	sit2_sendProperty(state, 0x0710, 0, true);
	sit2_sendProperty(state, 0x070a, (1 << 8), true);
	sit2_sendProperty(state, 0x0706, 5000, true);
	sit2_sendProperty(state, 0x0707, (27 << 8) | 148, true);
	sit2_sendProperty(state, 0x0703, (2 << 4) | 8, true);
	sit2_sendProperty(state, 0x0713, (0xff << 8) | 0xff, true);
	sit2_sendProperty(state, 0x070f, 0, true);
	sit2_sendProperty(state, 0x0709, 0, true);
	sit2_sendProperty(state, 0x0704, 0xb0, true);
	sit2_sendProperty(state, 0x0712, 16, true);
	/* tuner property */
	sit2_sendProperty(state, 0x0504, 0x8000, true);
	sit2_sendProperty(state, 0x0501, 1, true);
	sit2_sendProperty(state, 0x0505, (1 << 10) | (1 << 9) | (1 << 8), true);
	sit2_sendProperty(state, 0x0506, 1, true);
	sit2_sendProperty(state, 0x0507, 127, true);
	
	return uret;
}

static u8 sit2_demod_wakeUp(struct sit2_state *state, u8 resetCode, u8 funcCode)
{
	u8 uret = SIT2_ERROR_OK;
	 dprintk("%s, resetCode=%d, funcCode=%d\n", __func__, resetCode, funcCode);
	 /* start clock */
	state->sndBuffer[0] = 0xc0;
	state->sndBuffer[1] = 18;
	state->sndBuffer[2] = 0;
	state->sndBuffer[3] = 12;
	state->sndBuffer[4] = 0;
	state->sndBuffer[5] = 0x0d;
	state->sndBuffer[6] = 22;
	state->sndBuffer[7] = 0;
	state->sndBuffer[8] = 0;
	state->sndBuffer[9] = 0;
	state->sndBuffer[10] = 0;
	state->sndBuffer[11] = 0;
	state->sndBuffer[12] = 0;	
	uret = sit2_sendCommand(state, 13, 0, false);
	if(uret != SIT2_ERROR_OK)
		return uret;
	/* power up */
	dprintk("%s, power up\n", __func__);
	state->sndBuffer[0] = 0xc0;
	state->sndBuffer[1] = 6;
	state->sndBuffer[2] = resetCode;
	state->sndBuffer[3] = 15;
	state->sndBuffer[4] = 0;
	state->sndBuffer[5] = (1 << 5);
	state->sndBuffer[6] = (2 << 4) | (funcCode & 0x0f);
	state->sndBuffer[7] = 1;
	uret = sit2_sendCommand(state, 8, 1, false);
	dprintk("%s, power up[%d]\n", __func__, uret);
	return uret;
}

static u8 sit2_demod_powerDown(struct sit2_state *state)
{
	u8 uret;
	dprintk("%s\n", __func__);
	state->sndBuffer[0] = 0x13;
	uret = sit2_sendCommand(state, 1, 0, false);
	return uret;
}

static u8 sit2_demod_reStart(struct sit2_state *state)
{
	u8 uret;
	state->sndBuffer[0] = 0x85;
	uret = sit2_sendCommand(state, 1, 1, false);
	return uret;
}

static u8 sit2_demod_romId(struct sit2_state *state, u8 *id)
{
	u8 uret;
	state->sndBuffer[0] = 0x02;
	uret = sit2_sendCommand(state, 1, 13, false);
	*id = state->revBuffer[12];
	return uret;
}

static u8 sit2_demod_getStatus(struct sit2_state *state, u8 intack, SIT2_DD_STATUS *pStatus)
{
	u8 uret;
	state->sndBuffer[0] = 0x87;
	state->sndBuffer[1] = intack & 0x01;
	uret = sit2_sendCommand(state, 2, 8, false);
	
	pStatus->pclint = (state->revBuffer[1] >> 1) & 0x01;
	pStatus->dlint = (state->revBuffer[1] >> 2) & 0x01;
	pStatus->berint = (state->revBuffer[1] >> 3) & 0x01;
	pStatus->uncorint = (state->revBuffer[1] >> 4) & 0x01;
	pStatus->rsqint_bit5 = (state->revBuffer[1] >> 5) & 0x01;
	pStatus->rsqint_bit6 = (state->revBuffer[1] >> 6) & 0x01;
	pStatus->rsqint_bit7 = (state->revBuffer[1] >> 7) & 0x01;
	
	pStatus->pcl = (state->revBuffer[2] >> 1) & 0x01;
	pStatus->dl = (state->revBuffer[2] >> 2) & 0x01;
	pStatus->ber = (state->revBuffer[2] >> 3) & 0x01;
	pStatus->uncor = (state->revBuffer[2] >> 4) & 0x01;
	pStatus->rsqstat_bit5 = (state->revBuffer[2] >> 5) & 0x01;
	pStatus->rsqstat_bit6 = (state->revBuffer[2] >> 6) & 0x01;
	pStatus->rsqstat_bit7 = (state->revBuffer[2] >> 7) & 0x01;
	
	pStatus->modulation = state->revBuffer[3] & 0x0f;
	pStatus->ts_bit_rate = (state->revBuffer[5] << 8) | state->revBuffer[4];
	pStatus->ts_clk_freq = (state->revBuffer[7] << 8) | state->revBuffer[6];
		
	return uret;
}

static u8 sit2_demod_getDVBTStatus(struct sit2_state *state, u8 intack)
{
	u8 uret;
	state->sndBuffer[0] = 0xa0;
	state->sndBuffer[1] = intack & 0x01;
	uret = sit2_sendCommand(state, 2, 13, false);
	return uret;
}

static u8 sit2_demod_getDVBT2Status(struct sit2_state *state, u8 intack)
{
	u8 uret;
	state->sndBuffer[0] = 0x50;
	state->sndBuffer[1] = intack & 0x01;
	uret = sit2_sendCommand(state, 2, 14, false);
	return uret;
}

static u8 sit2_demod_getDVBCStatus(struct sit2_state *state, u8 intack)
{
	u8 uret;
	state->sndBuffer[0] = 0x90;
	state->sndBuffer[1] = intack & 0x01;
	uret = sit2_sendCommand(state, 2, 9, false);
	return uret;
}

static u8 sit2_demod_getUncor(struct sit2_state *state, u8 rstcode)
{
	u8 uret;
	state->sndBuffer[0] = 0x84;
	state->sndBuffer[1] = rstcode & 0x01;
	uret = sit2_sendCommand(state, 2, 3, false);
	return uret;
}

static u8 sit2_demod_getBer(struct sit2_state *state, u8 rstcode)
{
	u8 uret;
	state->sndBuffer[0] = 0x82;
	state->sndBuffer[1] = rstcode & 0x01;
	uret = sit2_sendCommand(state, 2, 3, false);
	return uret;
}

static u8 sit2_demod_downloadFW(struct sit2_state *state, u8 fw[], u32 fwSize, u8 nbPerLine)
{
	u8 uret = SIT2_ERROR_OK;
	u32 line, fw_lines, line_left;
	fw_lines = fwSize / nbPerLine;
	line_left = fwSize - fw_lines*nbPerLine;
	if(fw_lines > 0) {
		for(line = 0; line < fw_lines; line++) {
			memcpy(state->sndBuffer, fw + nbPerLine*line, nbPerLine);
			uret = sit2_sendCommand(state, nbPerLine, 1, false);
			if(uret != SIT2_ERROR_OK)
				break;
		}
	}
	if(line_left) {
		memcpy(state->sndBuffer, fw + nbPerLine*fw_lines, line_left);
		uret = sit2_sendCommand(state, line_left, 1, false);
	}	
	return uret;
}

static u8 sit2_demod_setMP(struct sit2_state *state, u8 mp_a, u8 mp_b, u8 mp_c, u8 mp_d)
{
	state->sndBuffer[0] = 0x88;
	state->sndBuffer[1] = mp_a;
	state->sndBuffer[2] = mp_b;
	state->sndBuffer[3] = mp_c;
	state->sndBuffer[4] = mp_d;
	return sit2_sendCommand(state, 5, 5, false);
}

static u8 sit2_demod_setGPIO(struct sit2_state *state, u8 gpMode_0, u8 gpRead_0, u8 gpMode_1, u8 gpRead_1)
{
	state->sndBuffer[0] = 0x12;
	state->sndBuffer[1] = (gpRead_0 << 7) | gpMode_0;
	state->sndBuffer[2] = (gpRead_1 << 7) | gpMode_1;
	return sit2_sendCommand(state, 3, 3, false);
}

static u8 sit2_demod_setExtAGC(struct sit2_state *state, u8 agc1_mode, u8 agc1_inv, u8 agc1_kloop, u8 agc1_min,
				u8 agc2_mode, u8 agc2_inv, u8 agc2_kloop, u8 agc2_min)
{
	state->sndBuffer[0] = 0x89;
	state->sndBuffer[1] = (agc2_inv << 7) | (agc2_mode << 4) | (agc1_inv << 3) | agc1_mode;
	state->sndBuffer[2] = agc1_kloop;
	state->sndBuffer[3] = agc2_kloop;
	state->sndBuffer[4] = agc1_min;
	state->sndBuffer[5] = agc2_min;
	return sit2_sendCommand(state, 6, 3, false);
}

static u8 sit2_demod_setDvbt2FEF(struct sit2_state *state, u8 fef_flag, u8 fef_inv)
{
	state->sndBuffer[0] = 0x51;
	state->sndBuffer[1] = (fef_inv << 3) | fef_flag;
	return sit2_sendCommand(state, 2, 12, false);
}

static u8 sit2_demod_selectPlp(struct sit2_state *state, u8 plp_id, u8 plp_mode)
{
	state->sndBuffer[0] = 0x52;
	state->sndBuffer[1] = plp_id;
	state->sndBuffer[1] = plp_mode;
	return sit2_sendCommand(state, 3, 1, false);
}

static u8 sit2_demod_init(struct sit2_state *state)
{
	u8 uret = SIT2_ERROR_OK;
	u8 ts_mode, ts_clock;
	u8 romid;
	uret = sit2_demod_wakeUp(state, 1, 0);
	if(uret != SIT2_ERROR_OK)
		return uret;

	uret = sit2_demod_romId(state, &romid);
	if(uret != SIT2_ERROR_OK)
		return uret;

	dprintk("%s: start to download ver[%d] patch!\n", __func__, romid);
	if(romid == 2) { /* Ver20 */		
		uret = sit2_demod_downloadFW(state, sit2_patch_2, SIT2_PATCH_2_SIZE, SIT2_PATCH_PER_LINE);
		if(uret != SIT2_ERROR_OK)
			return uret;
		dprintk("%s: download ver[%d] patch sucessfully!\n", __func__, romid);
	} else if (romid == 3) { /* Ver30 */
		uret = sit2_demod_downloadFW(state, sit2_patch_3, SIT2_PATCH_3_SIZE, SIT2_PATCH_PER_LINE);
		if(uret != SIT2_ERROR_OK)
			return uret;
		dprintk("%s: download ver[%d] patch sucessfully!\n", __func__, romid);		
	}
	
	uret = sit2_startFirmware(state, false);
	if(uret != SIT2_ERROR_OK)
		return uret;
		
	/* download default properties */
	sit2_demod_setMP(state, 1, 2, 1, 1);
	sit2_demod_setExtAGC(state, 1, 0, 6, 0, 2, 0, 18, 0);
	sit2_demod_setDvbt2FEF(state, 3, 0);
	sit2_demod_setGPIO(state, 8, 0, 4, 0);
	/* common */
	sit2_sendProperty(state, 0x0401, 0, false);
	/* DD */
	sit2_sendProperty(state, 0x1003, (1 << 4) | 7, false);
	sit2_sendProperty(state, 0x1002, (1 << 4) | 5, false);
	sit2_sendProperty(state, 0x100c, (1 << 4) | 2, false);
	sit2_sendProperty(state, 0x1006, 0x24, false);
	sit2_sendProperty(state, 0x100b, 5000, false);
	sit2_sendProperty(state, 0x1007, 0x2400, false);
	sit2_sendProperty(state, 0x100a, (0 << 9) | (0 << 8) | (2 << 4) | 8, false); /* set modulation */
	sit2_sendProperty(state, 0x1004, (1 << 4) | 5, false);
	sit2_sendProperty(state, 0x1005, (10 << 4) | 1, false);
	sit2_sendProperty(state, 0x100d, 720, false); /* ts clock frequency */
	if (state->config->ts_bus_mode == 1)
		ts_mode = 3;
	else if (state->config->ts_bus_mode == 2)
		ts_mode = 6;
	else
		ts_mode = 0;
	if (state->config->ts_clock_mode == 1)
		ts_clock = 2;
	else
		ts_clock = 1;	
	sit2_sendProperty(state, 0x1001, (0 << 8) | (0 << 7) | (0 << 6) | (ts_clock << 4) | ts_mode, false);	
	sit2_sendProperty(state, 0x1009, (0 << 13) | (1 << 12) | (3 << 10) | (15 << 6) | (3 << 4) | 15, false);
	sit2_sendProperty(state, 0x1008, (0 << 14) | (1 << 13) | (1 << 12) | (3 << 10) | (15 << 6) | (3 << 4) | 15, false);
	/* DVBC */
	sit2_sendProperty(state, 0x1104, 112, false);
	sit2_sendProperty(state, 0x1103, 100, false);
	sit2_sendProperty(state, 0x1101, 0, false);
	sit2_sendProperty(state, 0x1102, 6900, false);
	/* DVBT */
	sit2_sendProperty(state, 0x1203, 130, false);
	sit2_sendProperty(state, 0x1202, 550, false);
	sit2_sendProperty(state, 0x1201, 0, false);
	/* DVBT2 */
	sit2_sendProperty(state, 0x1303, 130, false);
	sit2_sendProperty(state, 0x1301, 550, false);
	sit2_sendProperty(state, 0x1302, (1 << 12) | (1 << 8) | 1, false);	
	/* SCAN */
	sit2_sendProperty(state, 0x0304, 0, false);
	sit2_sendProperty(state, 0x0303, 0, false);
	sit2_sendProperty(state, 0x0308, 0, false);
	sit2_sendProperty(state, 0x0307, (1 << 9) | 1, false);
	sit2_sendProperty(state, 0x0306, 0, false);
	sit2_sendProperty(state, 0x0305, 0, false);
	sit2_sendProperty(state, 0x0301, (3 << 2), false);
	
	return uret;
}

static int sit2_setStandard(struct sit2_state *state, fe_delivery_system_t toSystem)
{
	u8 bandwidth = 8, modulation = 2, spectrum = 0, auto_detect = 1;
	
	dprintk("%s, to=%d, current=%d\n", __func__, toSystem, state->current_system);
	if (toSystem == state->current_system)
		return 0;
	
	switch(toSystem) {
	case SYS_DVBT:
		bandwidth = 8;
		modulation = 2;
		spectrum = 0;
		auto_detect = 1;
		break;
	case SYS_DVBT2:
		bandwidth = 8;
		modulation = 7;
		spectrum = 0;
		auto_detect = 1;
		break;
	case SYS_DVBC_ANNEX_A:
		bandwidth = 8;
		modulation = 3;
		spectrum = 0;
		auto_detect = 0;
		break;
	default:
		printk(KERN_INFO
	     	"%s: error! unsupported system=%d\n",
	     	__func__, toSystem);
	     	break;			
	}
	sit2_demod_tuner_i2c_enable(state, 1);
	if(toSystem == SYS_DVBT2)
		sit2_tuner_setup_FEFMode(state, 1);
	else
		sit2_tuner_setup_FEFMode(state, 0);
	sit2_demod_tuner_i2c_enable(state, 0);
	sit2_sendProperty(state, 0x100a, (auto_detect << 9) | (spectrum << 8) | (modulation << 4) | bandwidth, false);
	sit2_demod_reStart(state);
	state->current_system = toSystem;
	
	return 0;
}

int power_of_n (int n, int m)
{
	int i, p = 1;
	for (i=1; i<= m; i++) {
		p = p*n;
	}
	return p;
}

static int sit2_drv_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct sit2_state *state = fe->demodulator_priv;
	sit2_demod_tuner_i2c_enable(state, 1);
	sit2_tuner_getStatus(state, 0);
	sit2_demod_tuner_i2c_enable(state, 0);
	*strength = state->revBuffer[3] + 128;
	/* scale value to 0x0000-0xffff from 0x0000-0x00ff */
	*strength = *strength * 0xffff / 0x00ff;
	return 0;
}

static int sit2_drv_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	struct sit2_state *state = fe->demodulator_priv;
	
	sit2_demod_getUncor(state, 0);
	*ucblocks = (state->revBuffer[2] << 16) |  state->revBuffer[1];;
	
	return 0;
}

static int sit2_drv_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct sit2_state *state = fe->demodulator_priv;
	
	sit2_demod_getBer(state, 0);
	if(state->revBuffer[1] != 0) { /* to do scale. */
		*ber = state->revBuffer[2]/10/power_of_n(10, state->revBuffer[1]);
	}
	return 0;
}

static int sit2_drv_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct sit2_state *state = fe->demodulator_priv;
	SIT2_DD_STATUS dd_status;
	
	sit2_demod_getStatus(state, 0, &dd_status);
	switch(dd_status.modulation) {
	case 2: /*DVB-T*/
		sit2_demod_getDVBTStatus(state, 0);
		break;
	case 7: /*DVB-T2*/
		sit2_demod_getDVBT2Status(state, 0);
		break;
	case 3: /*DVB-C*/
		sit2_demod_getDVBCStatus(state, 0);		
		break;
	}
	/* report SNR in dB * 10 */
	*snr = state->revBuffer[3]/40;
	return 0;
}

static int sit2_drv_read_status(struct dvb_frontend *fe, fe_status_t *status)
{
	struct sit2_state *state = fe->demodulator_priv;
	SIT2_DD_STATUS dd_status;	
	*status = 0;
	sit2_demod_getStatus(state, 0, &dd_status);
	if(dd_status.pcl)
		*status = FE_HAS_SIGNAL | FE_HAS_CARRIER
		    | FE_HAS_SYNC | FE_HAS_VITERBI;
	if (dd_status.dl)
		*status = FE_HAS_SIGNAL | FE_HAS_CARRIER
		    | FE_HAS_SYNC | FE_HAS_VITERBI | FE_HAS_LOCK;
	return 0;
}

static int sit2_drv_get_frontend_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_HW;
}

static fe_modulation_t sit2_convert_modulation(u8 constellation)
{
	fe_modulation_t ret;
	switch (constellation) {
	case 7:
		ret = QAM_16;
		break;
	case 8:
		ret = QAM_32;
		break;
	case 9:
		ret = QAM_64;
		break;
	case 10:
		ret = QAM_128;
		break;
	case 11:
		ret = QAM_256;
		break;
	case 3:
		ret = QPSK;
		break;
	default:
		ret = QAM_AUTO;
		break;		
	}
	return ret;				
}

static fe_transmit_mode_t sit2_convert_fftcode(u8 fftcode)
{
	fe_transmit_mode_t ret;
	switch (fftcode) {	
	case 10:
		ret = TRANSMISSION_MODE_1K;
		break;
	case 11:
		ret = TRANSMISSION_MODE_2K;
		break;
	case 12:
		ret = TRANSMISSION_MODE_4K;
		break;
	case 13:
		ret = TRANSMISSION_MODE_8K;
		break;
	case 14:
		ret = TRANSMISSION_MODE_16K;
		break;
	case 15:
		ret = TRANSMISSION_MODE_32K;
		break;
	default:
		ret = TRANSMISSION_MODE_AUTO;
	}
	return ret;
}

static fe_guard_interval_t sit2_convert_gicode(int gicode)
{
	fe_guard_interval_t ret;
	switch (gicode) {
	case 1:
		ret = GUARD_INTERVAL_1_32;
		break;
	case 2:
		ret = GUARD_INTERVAL_1_16;
		break;
	case 3:
		ret = GUARD_INTERVAL_1_8;
		break;
	case 4:
		ret = GUARD_INTERVAL_1_4;
		break;
	case 5:
		ret = GUARD_INTERVAL_1_128;
		break;
	case 6:
		ret = GUARD_INTERVAL_19_128;
		break;
	case 7:
		ret = GUARD_INTERVAL_19_256;
		break;		
	default:
		ret = GUARD_INTERVAL_AUTO;
	}
	return ret;
}

static fe_hierarchy_t sit2_convert_hierarchycode(int hierarchycode)
{
	fe_hierarchy_t ret;
	switch (hierarchycode) {
	case 1:
		ret = HIERARCHY_NONE;
		break;
	case 2:
		ret = HIERARCHY_1;
		break;
	case 3:
		ret = HIERARCHY_2;
		break;
	case 5:
		ret = HIERARCHY_4;
		break;
	default:
		ret = HIERARCHY_AUTO;
	}
	return ret;
}

static fe_code_rate_t sit2_convert_coderate(int coderate)
{
	fe_code_rate_t ret;
	switch (coderate) {
	case 1:
		ret = FEC_1_2;
		break;
	case 2:
		ret = FEC_2_3;
		break;
	case 3:
		ret = FEC_3_4;
		break;
	case 4:
		ret = FEC_4_5;
		break;
	case 5:
		ret = FEC_5_6;
		break;
	case 7:
		ret = FEC_7_8;
		break;
	case 13:
		ret = FEC_3_5;
		break;		
	default:
		ret = FEC_AUTO;
	}
	return ret;
}

static int sit2_drv_get_frontend(struct dvb_frontend *fe)
{
	struct sit2_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret = 0;
	SIT2_DD_STATUS dd_status;
	sit2_demod_getStatus(state, 0, &dd_status);
	switch(dd_status.modulation) {
	case 2: /*DVB-T*/
		sit2_demod_getDVBTStatus(state, 0);
		c->modulation = sit2_convert_modulation(state->revBuffer[8] & 0x3f);
		c->transmission_mode = sit2_convert_fftcode(state->revBuffer[10] & 0x0f);
		c->guard_interval = sit2_convert_gicode((state->revBuffer[10] >> 4) & 0x07);
		c->hierarchy = sit2_convert_hierarchycode(state->revBuffer[11] & 0x07);
		c->code_rate_HP = sit2_convert_coderate(state->revBuffer[9] & 0x0f);
		c->code_rate_LP = sit2_convert_coderate((state->revBuffer[9] >> 4) & 0x0f);
		c->inversion = ((state->revBuffer[8] >> 6) & 0x01) ? INVERSION_ON : INVERSION_OFF;
		break;
	case 7: /*DVB-T2*/
		sit2_demod_getDVBT2Status(state, 0);
		c->modulation = sit2_convert_modulation(state->revBuffer[8] & 0x3f);
		c->transmission_mode = sit2_convert_fftcode(state->revBuffer[9] & 0x0f);
		c->guard_interval = sit2_convert_gicode((state->revBuffer[9] >> 4) & 0x07);
		c->fec_inner = sit2_convert_coderate(state->revBuffer[12] & 0x0f);
		c->inversion = ((state->revBuffer[8] >> 6) & 0x01) ? INVERSION_ON : INVERSION_OFF;
		break;
	case 3: /*DVB-C*/
		sit2_demod_getDVBCStatus(state, 0);
		c->symbol_rate = state->dvbc_symrate;
		c->modulation = sit2_convert_modulation(state->revBuffer[8] & 0x3f);
		c->inversion = ((state->revBuffer[8] >> 6) & 0x01) ? INVERSION_ON : INVERSION_OFF;
		break;
	}	
	return ret;
}

static int sit2_drv_set_frontend(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct sit2_state *state = fe->demodulator_priv;
	int req_plp_id = 0;
	u8 uret, req_qam, req_bandwidth = 0;
	u32 max_lock_time = 5000, min_lock_time = 100;
	u32 ulCount, ulTick, ulDelay;
	SIT2_DD_STATUS dd_status;
	bool bLock = false, bSearch = true;
	
	dprintk(
	     "%s: system=%d frequency=%d bandwidth=%d symrate=%d qam=%d stream_id=%d\n",
	     __func__, c->delivery_system, c->frequency, c->bandwidth_hz, c->symbol_rate, c->modulation, c->stream_id);
	     	
	sit2_setStandard(state, c->delivery_system);
	switch (c->modulation) {
	case QAM_16:
		req_qam = 7;
		break;
	case QAM_32:
		req_qam = 8;
		break;
	case QAM_64:
		req_qam = 9;
		break;
	case QAM_128:
		req_qam = 10;
		break;	
	case QAM_256:
		req_qam = 11;
		break;	
	default: /* QAM_AUTO */
		req_qam = 0;
		break;
	}
	if (c->stream_id == NO_STREAM_ID_FILTER)
		req_plp_id = state->plp_id;
	else if ((c->stream_id >= 0) && (c->stream_id <= 255)) {
		req_plp_id = c->stream_id;
		state->plp_id = req_plp_id;
	}
	switch(c->delivery_system) {
	case SYS_DVBT:
	case SYS_DVBT2:
		max_lock_time = 5000;
		min_lock_time = 100;
		sit2_sendProperty(state, 0x1201, state->stream, false);
		if(req_plp_id != -1)
			sit2_demod_selectPlp(state, req_plp_id, 1);
		else
			sit2_demod_selectPlp(state, 0, 0);
		if(c->bandwidth_hz == 1700000)
			req_bandwidth = 2;
		else
			req_bandwidth = (u8)(c->bandwidth_hz/1000000);
		sit2_sendProperty(state, 0x100a, (1 << 9) | (0 << 8) | (15 << 4) | req_bandwidth, false);
		break;
	case SYS_DVBC_ANNEX_A:
		max_lock_time = 2000;
		min_lock_time = 80;
		req_bandwidth = 8;
		state->dvbc_symrate = c->symbol_rate;
		sit2_sendProperty(state, 0x100a, (3 << 4) | req_bandwidth, false);
		sit2_sendProperty(state, 0x1102, c->symbol_rate/1000, false);
		sit2_sendProperty(state, 0x1101, req_qam, false);
		break;
	default:
		dprintk("%s, error! unsupport delivery system - %d!", __func__, c->delivery_system);
		break;
	}
	
	/* tune tuner frequency */
	sit2_demod_tuner_i2c_enable(state, 1);
	sit2_tuner_setFreq(state, c->frequency, c->delivery_system, req_bandwidth);
	sit2_demod_tuner_i2c_enable(state, 0);
	
	sit2_demod_reStart(state);
	
	/* check status */
  	ulCount = 0;
  	ulDelay = 10;
  	ulTick = max_lock_time/ulDelay;
  	msleep(min_lock_time);
  	
  	while(bSearch) {
  		ulCount++;
  		
  		uret = sit2_demod_getStatus(state, 1, &dd_status);
  		switch(c->delivery_system) {
  		case SYS_DVBT:
  		case SYS_DVBT2:
  			if(dd_status.dl) {
  				if((dd_status.modulation == 7) && (c->delivery_system == SYS_DVBT)) {
  					if(req_plp_id != -1)
						sit2_demod_selectPlp(state, req_plp_id, 1);
					else
						sit2_demod_selectPlp(state, 0, 0);
  					msleep(340);
  				}
  				bLock = true;
  				bSearch = false;
  			} else if(dd_status.rsqint_bit5)
  				bSearch = false;
  			break;
  		case SYS_DVBC_ANNEX_A:
  			if(dd_status.dl) {
  				bLock = true;
  				bSearch = false;
  			}
  			break;
  		default:
			dprintk("%s, error! unsupport delivery system - %d!", __func__, c->delivery_system);
			bSearch = false;
			break;
  		}
  		
  		if(bSearch)
  			msleep(10);
  		if (ulCount >= ulTick)
  			bSearch = false;
  	}	

	if (bLock && state->config->start_ctrl)
		state->config->start_ctrl(fe);
	return 0;
}

static int sit2_drv_tune(struct dvb_frontend *fe,
			bool re_tune,
			unsigned int mode_flags,
			unsigned int *delay,
			fe_status_t *status)
{	
	*delay = HZ / 5;	
	if (re_tune) {
		int ret = sit2_drv_set_frontend(fe);
		if (ret)
			return ret;
	}	
	return sit2_drv_read_status(fe, status);
}

static int sit2_drv_init(struct dvb_frontend *fe)
{
	struct sit2_state *state = fe->demodulator_priv;

	dprintk("%s: init=%d\n", __func__, state->isInited);
	
	sit2_demod_tuner_i2c_enable(state, 1);
	
	if(state->isInited) {
		sit2_tuner_wakeUp(state);		
	} else {
		sit2_tuner_init(state);
	}
	sit2_tuner_xout_enable(state, 1);
	
	if(state->isInited) {		
		sit2_demod_wakeUp(state, 8, 1);
	} else {
		sit2_demod_init(state);
		state->isInited = true;
	}	
	
	sit2_demod_tuner_i2c_enable(state, 0);
	return 0;
}

static int sit2_drv_sleep(struct dvb_frontend *fe)
{
	struct sit2_state *state = fe->demodulator_priv;
	
	dprintk("%s: init=%d\n", __func__, state->isInited);
	
	sit2_demod_powerDown(state);
	
	sit2_demod_tuner_i2c_enable(state, 1);
	sit2_tuner_xout_enable(state, 0);
	sit2_tuner_standby(state);
	sit2_demod_tuner_i2c_enable(state, 0);
	
	state->current_system = SYS_UNDEFINED;
	return 0;
}

static void sit2_drv_release(struct dvb_frontend *fe)
{
	struct sit2_state *state = fe->demodulator_priv;
	kfree(state);
}

static const struct dvb_frontend_ops sit2_ops = {
	.delsys = { SYS_DVBT, SYS_DVBT2, SYS_DVBC_ANNEX_A },
	/*.delsys = { SYS_DVBC_ANNEX_A },*/
	/* default: DVB-T/T2 */
	.info = {
		.name = "Sit2 DVB-T2/C",
		.frequency_stepsize = 62500,
		.frequency_min = 48000000,
		.frequency_max = 870000000,
		.symbol_rate_min = 870000,
		.symbol_rate_max = 7500000,
		.caps =	FE_CAN_FEC_1_2			|
			FE_CAN_FEC_2_3			|
			FE_CAN_FEC_3_4			|
			FE_CAN_FEC_5_6			|
			FE_CAN_FEC_7_8			|
			FE_CAN_FEC_AUTO			|
			FE_CAN_QPSK			|
			FE_CAN_QAM_16			|
			FE_CAN_QAM_32			|
			FE_CAN_QAM_64			|
			FE_CAN_QAM_128			|
			FE_CAN_QAM_256			|
			FE_CAN_QAM_AUTO			|
			FE_CAN_TRANSMISSION_MODE_AUTO	|
			FE_CAN_GUARD_INTERVAL_AUTO	|
			FE_CAN_HIERARCHY_AUTO		|
			FE_CAN_MUTE_TS			|
			FE_CAN_2G_MODULATION		|
			FE_CAN_MULTISTREAM
		},

	.release		= sit2_drv_release,
	.init			= sit2_drv_init,
	.sleep			= sit2_drv_sleep,

	.tune			= sit2_drv_tune,
	.set_frontend		= sit2_drv_set_frontend,
	.get_frontend		= sit2_drv_get_frontend,
	.get_frontend_algo	= sit2_drv_get_frontend_algo,

	.read_status		= sit2_drv_read_status,
	.read_snr		= sit2_drv_read_snr,
	.read_ber		= sit2_drv_read_ber,
	.read_ucblocks		= sit2_drv_read_ucblocks,
	.read_signal_strength	= sit2_drv_read_signal_strength,
};

struct dvb_frontend *sit2_attach(const struct sit2_config *config,
		struct i2c_adapter *i2c)
{
	struct sit2_state *state = NULL;
	state = kzalloc(sizeof(struct sit2_state), GFP_KERNEL);
	if (!state) {
		dev_err(&i2c->dev, "%s: kzalloc() failed\n",
				KBUILD_MODNAME);
		goto error;
	}	
	state->config = config;
	state->i2c = i2c;
	state->isInited = false;
	state->demod_addr = SIT2_DEMOD_ADDRESS;
	state->tuner_addr = SIT2_TUNER_ADDRESS;
	state->plp_id = 0;
	state->current_system = SYS_UNDEFINED;
	state->stream = 0;
	
	memcpy(&state->frontend.ops, &sit2_ops,
	       sizeof(struct dvb_frontend_ops));
	state->frontend.demodulator_priv = state;
	return &state->frontend;
error:
	kfree(state);
	return NULL;
}
EXPORT_SYMBOL(sit2_attach);

MODULE_DESCRIPTION("sit2 demodulator driver");
MODULE_AUTHOR("Max Nibble <nibble.max@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.00");
