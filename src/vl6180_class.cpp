/*******************************************************************************
Copyright © 2019, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#include "vl6180_class.h"

#define VL6180_9to7Conv(x) (x)

/* TODO when set all "cached" value with "default init" are updated after init from register read back */
#define REFRESH_CACHED_DATA_AFTER_INIT  1


#define IsValidGPIOFunction(x) ((x) == GPIOx_SELECT_GPIO_INTERRUPT_OUTPUT || (x) == GPIOx_SELECT_OFF)


/** default value ECE factor Molecular */
#define DEF_ECE_FACTOR_M    85
/** default value ECE factor Denominator */
#define DEF_ECE_FACTOR_D    100
/** default value for DMAX Enable */
#define DEF_DMAX_ENABLE     1
/** default ambient tuning factor %x1000 */
#define DEF_AMBIENT_TUNING  80

#define DEF_CROSS_TALK_VALID_HEIGHT_VALUE   20

#define LUXRES_FIX_PREC 8
#define GAIN_FIX_PREC    8  /* ! if not sme as LUX_PREC then :( adjust GetLux */
#define AN_GAIN_MULT    (1 << GAIN_FIX_PREC)

/**
 * ScalerLookUP scaling factor-1 to register #RANGE_SCALER lookup
 */
static const uint16_t ScalerLookUP[]       = {253, 127, 84}; /* lookup table for scaling->scalar 1x2x 3x */
/**
 * scaling factor to Upper limit look up
 */
static const uint16_t UpperLimitLookUP[]   = {185, 370, 580}; /* lookup table for scaling->limit  1x2x3x */


#if VL6180_UPSCALE_SUPPORT == 1
	#define _GetUpscale(...)  1
	#define _SetUpscale(...)
	#define DEF_UPSCALE 1
#elif VL6180_UPSCALE_SUPPORT == 2
	#define _GetUpscale(...)  2
	#define _SetUpscale(...)
	#define DEF_UPSCALE 2
#elif VL6180_UPSCALE_SUPPORT == 3
	#define _GetUpscale(...)  3
	#define _SetUpscale(...)
	#define DEF_UPSCALE 3
#else
	#define DEF_UPSCALE (-(VL6180_UPSCALE_SUPPORT))
	#define _GetUpscale(...) VL6180DevDataGet(Device, UpscaleFactor)
	#define _SetUpscale(Scaling) VL6180DevDataSet(Device, UpscaleFactor, Scaling)
#endif

#define Fix7_2_KCPs(x) ((((uint32_t)(x))*1000)>>7)


#if VL6180_WRAP_AROUND_FILTER_SUPPORT
	#define _IsWrapArroundActive() VL6180DevDataGet(Device, WrapAroundFilterActive)
#else
	#define _IsWrapArroundActive() 0
#endif


#if VL6180_HAVE_DMAX_RANGING
	#define _IsDMaxActive() VL6180DevDataGet(Device, DMaxEnable)
#endif

int VL6180::VL6180_WaitDeviceBooted()
{
	uint8_t FreshOutReset;
	int status;
	do {
		status = VL6180_RdByte(SYSTEM_FRESH_OUT_OF_RESET, &FreshOutReset);
	} while (FreshOutReset != 1 && status == 0);
	return status;
}

int VL6180::VL6180_InitData()
{
	int status, dmax_status ;
	int8_t offset;
	uint8_t FreshOutReset;
	uint32_t CalValue;
	uint16_t u16;
	uint32_t XTalkCompRate_KCps;

	VL6180DevDataSet(Device, EceFactorM, DEF_ECE_FACTOR_M);
	VL6180DevDataSet(Device, EceFactorD, DEF_ECE_FACTOR_D);

	VL6180DevDataSet(Device, RangeIgnore.Enabled, 0);

#ifdef VL6180_HAVE_UPSCALE_DATA
	VL6180DevDataSet(Device, UpscaleFactor,  DEF_UPSCALE);
#endif

#ifdef VL6180_HAVE_WRAP_AROUND_DATA
	VL6180DevDataSet(Device, WrapAroundFilterActive, (VL6180_WRAP_AROUND_FILTER_SUPPORT > 0));
#if VL6180_HAVE_DMAX_RANGING
	VL6180DevDataSet(Device, DMaxEnable, DEF_DMAX_ENABLE);
#endif
#endif

#if VL6180_HAVE_DMAX_RANGING
	_DMax_OneTimeInit();
#endif
	do {

		/* backup offset initial value from nvm these must be done prior any over call that use offset */
		status = VL6180_RdByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, (uint8_t *)&offset);
		if (status) {
			break;
		}
		VL6180DevDataSet(Device, Part2PartOffsetNVM, offset);

		status = VL6180_RdDWord(SYSRANGE_RANGE_IGNORE_THRESHOLD, &CalValue);
		if (status) {
			break;
		}
		if ((CalValue&0xFFFF0000) == 0) {
			CalValue = 0x00CE03F8;
		}
		VL6180DevDataSet(Device, Part2PartAmbNVM, CalValue);

		status = VL6180_RdWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE , &u16);
		if (status) {
			break;
		}
		XTalkCompRate_KCps = Fix7_2_KCPs(u16);
		VL6180DevDataSet(Device, XTalkCompRate_KCps, XTalkCompRate_KCps);

#if VL6180_HAVE_DMAX_RANGING
		dmax_status = _DMax_InitData();
#else
		dmax_status = 0;
#endif
		if (dmax_status < 0) {
			break;
		}

		/* Read or wait for fresh out of reset  */
		status = VL6180_RdByte(SYSTEM_FRESH_OUT_OF_RESET, &FreshOutReset);
		if (status) {
			break;
		}
		if (FreshOutReset != 1 || dmax_status)
			status = CALIBRATION_WARNING;

	} while (0);

	return status;
}

int8_t VL6180::VL6180_GetOffsetCalibrationData()
{
	int8_t offset;
	offset = VL6180DevDataGet(Device, Part2PartOffsetNVM);
	return offset;
}

int  VL6180::VL6180_SetOffsetCalibrationData(int8_t offset)
{
	int status;
	VL6180DevDataSet(Device, Part2PartOffsetNVM, offset);
	offset /= _GetUpscale();
	status = VL6180_WrByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, offset);
	return status;
}

int  VL6180::VL6180_SetXTalkCompensationRate(FixPoint97_t Rate)
{
	int status;
	status = VL6180_WrWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, Rate);
	if (status == 0) {
		uint32_t XTalkCompRate_KCps;
		XTalkCompRate_KCps = Fix7_2_KCPs(Rate);
		VL6180DevDataSet(Device, XTalkCompRate_KCps, XTalkCompRate_KCps);
		/* update dmax whenever xtalk rate changes */
#if VL6180_HAVE_DMAX_RANGING
		status = _DMax_InitData();
#else
		status = 0;
#endif
	}
	return status;
}

int VL6180::VL6180_SetI2CAddress(uint8_t NewAddress)
{
	int status;

	status = VL6180_WrByte(I2C_SLAVE_DEVICE_ADDRESS, NewAddress / 2);
    Device->I2cAddr = NewAddress;

	return status;
}

uint16_t VL6180::VL6180_GetUpperLimit()
{
	uint16_t limit;
	int scaling;

	scaling = _GetUpscale();
	/* FIXME we do assume here _GetUpscale is valid if  user call us prior to init we may overflow the LUT  mem area */
	limit = UpperLimitLookUP[scaling - 1];

	return limit;
}



int VL6180::VL6180_StaticInit()
{
	int status = 0, init_status;

	/* TODO doc When using configurable scaling but using 1x as start condition
	 * load tunning upscale  or not ??? */
	if (_GetUpscale() == 1 && !(VL6180_UPSCALE_SUPPORT < 0))
		init_status = VL6180_RangeStaticInit();
	else
		init_status = VL6180_UpscaleStaticInit();

	if (init_status < 0) {
		goto error;
	}

	if (!status && init_status) {
		status = init_status;
	}
error:
	return status;
}


int VL6180::VL6180_SetGroupParamHold(int Hold)
{
	int status;
	uint8_t value;

	if (Hold)
		value = 1;
	else
		value = 0;
	status = VL6180_WrByte(SYSTEM_GROUPED_PARAMETER_HOLD, value);

	return status;

}

int VL6180::VL6180_Prepare()
{
	int status;

	do {
		status = VL6180_StaticInit();
		if (status < 0)
			break;

		/* set range InterruptMode to new sample */
		status = VL6180_RangeConfigInterrupt(CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY);
		if (status)
			break;

		/* set default threshold */
		status = VL6180_RangeSetRawThresholds(10, 200);
		if (status) {
			break;
		}
	#if VL6180_WRAP_AROUND_FILTER_SUPPORT
		_filter_Init();
	#endif
		/* make sure to reset any left previous condition that can hangs first poll */
		status = VL6180_ClearAllInterrupt();
	} while (0);

	return status;
}


int VL6180::VL6180_RangePollMeasurement(VL6180_RangeData_t *pRangeData)
{
	int status;
	int ClrStatus;
	IntrStatus_t IntStatus;

	/* start single range measurement */


	#if VL6180_SAFE_POLLING_ENTER
	/* if device get stopped with left interrupt uncleared , it is required to clear them now or poll for new condition will never occur*/
	status = VL6180_RangeClearInterrupt();
	if (status) {
		goto done;
	}
	#endif
	/* //![single_shot_snipet] */
	status = VL6180_RangeSetSystemMode(MODE_START_STOP | MODE_SINGLESHOT);
	if (status) {
		goto done;
	}


	/* poll for new sample ready */
	while (1) {
		status = VL6180_RangeGetInterruptStatus(&IntStatus.val);
		if (status) {
			break;
		}
		if (IntStatus.status.Range == RES_INT_STAT_GPIO_NEW_SAMPLE_READY || IntStatus.status.Error != 0) {
			break;
		}

		VL6180_PollDelay();
	}
	/* //![single_shot_snipet] */

	if (!status) {
		status = VL6180_RangeGetMeasurement(pRangeData);
	}

	/*  clear range interrupt source */
	ClrStatus = VL6180_RangeClearInterrupt();
	if (ClrStatus) {
		/*  leave initial status if already in error  */
		if (!status) {
			status = ClrStatus;
		}
	}
done:
	return status;
}

int VL6180::VL6180_GetCachedDWord(uint16_t  index, uint32_t *pValue)
{
#if VL6180_CACHED_REG
	int status;
	uint32_t Value;
	if (VL6180DevDataGet(Device, CacheFilled) != 0 &&
		index >= VL6180_FIRST_CACHED_INDEX  &&
		index <= (VL6180_LAST_CACHED_INDEX - 3)) {
		uint8_t *pBytes = &VL6180DevDataGet(Device, CachedRegs[index - VL6180_FIRST_CACHED_INDEX]);
		Value = ((uint32_t)pBytes[0] << 24) |
				((uint32_t)pBytes[1] << 16) |
				((uint32_t)pBytes[2] << 8) |
				(uint32_t)pBytes[3];
		*pValue = Value;
		status = 0;
	} else {
		status =  VL6180_RdDWord(index, pValue);
	}
	return status;
#else
    return VL6180_RdDWord(index, pValue);
#endif
}

int VL6180::VL6180_GetCachedWord(uint16_t  index, uint16_t *pValue)
{
#if VL6180_CACHED_REG
	int status;
	uint32_t Value;
	if (VL6180DevDataGet(Device, CacheFilled) != 0 &&
		index >= VL6180_FIRST_CACHED_INDEX  &&
		index <= (VL6180_LAST_CACHED_INDEX - 1)) {
		uint8_t *pBytes = &VL6180DevDataGet(Device, CachedRegs[index - VL6180_FIRST_CACHED_INDEX]);
		Value = ((uint32_t)pBytes[0] << 8) | (uint32_t)pBytes[1];
		*pValue = Value;
		status = 0;
	} else {
		status =  VL6180_RdWord(index, pValue);
	}
	return status;
#else
    return VL6180_RdWord(index, pValue);
#endif
}

int VL6180::VL6180_GetCachedByte(uint16_t  index, uint8_t *pValue)
{
#if VL6180_CACHED_REG
	int status;
	uint8_t Value;
	if (VL6180DevDataGet(Device, CacheFilled) != 0 &&
		index >= VL6180_FIRST_CACHED_INDEX &&
		index <= VL6180_LAST_CACHED_INDEX) {
		Value = VL6180DevDataGet(Device, CachedRegs[index - VL6180_FIRST_CACHED_INDEX]);
		*pValue = Value;
		status = 0;
	} else {
		status =  VL6180_RdByte(index, pValue);
	}
	return status;
#else
	return VL6180_RdByte(index, pValue);
#endif
}


int VL6180::_CachedRegs_Fetch()
{
#if VL6180_CACHED_REG
	int status;
	uint8_t *Buffer;
	if (VL6180DevDataGet(Device, CacheFilled) == 0) {
		VL6180DevDataSet(Device, CacheFilled, 1);
		Buffer = &VL6180DevDataGet(Device, CachedRegs[0]);
		status = VL6180_ReadMulti(VL6180_FIRST_CACHED_INDEX, Buffer, VL6180_CACHED_REG_CNT);
	} else {
		status = 0 ;
	}
	return status;
#else
    return 0;
#endif
}

void VL6180::_CachedRegs_Flush()
{
#if VL6180_CACHED_REG
	VL6180DevDataSet(Device, CacheFilled, 0);
#endif
}

int VL6180::VL6180_RangeGetMeasurement(VL6180_RangeData_t *pRangeData)
{
	int status;
	uint16_t RawRate;
	uint8_t RawStatus;

	status = _CachedRegs_Fetch();
	if (status) {
		goto error;
	}
	status = VL6180_RangeGetResult(&pRangeData->range_mm);
	if (!status) {
		status = VL6180_GetCachedWord(RESULT_RANGE_SIGNAL_RATE, &RawRate);
		if (!status) {
			pRangeData->signalRate_mcps = VL6180_9to7Conv(RawRate);
			status = VL6180_GetCachedByte(RESULT_RANGE_STATUS, &RawStatus);
			if (!status) {
				pRangeData->errorStatus = RawStatus >> 4;
			}
	#if VL6180_WRAP_AROUND_FILTER_SUPPORT || VL6180_HAVE_DMAX_RANGING
			status = _GetRateResult(pRangeData);
			if (status)
				goto error;
	#endif
	#if VL6180_WRAP_AROUND_FILTER_SUPPORT
			/* if enabled run filter */
			if (_IsWrapArroundActive()) {
				status = _filter_GetResult(pRangeData);
				if (!status) {
					/* patch the range status and measure if it is filtered */
					if(pRangeData->FilteredData.filterError != NoError) {
						pRangeData->errorStatus = pRangeData->FilteredData.filterError;
						pRangeData->range_mm = pRangeData->FilteredData.range_mm;
					}
				}
			}
	#endif

	#if VL6180_HAVE_DMAX_RANGING
			if (_IsDMaxActive()) {
				_DMax_Compute(pRangeData);
			}
	#endif
		}
	}
error:
	_CachedRegs_Flush();
	return status;
}


int VL6180::VL6180_RangeGetMeasurementIfReady(VL6180_RangeData_t *pRangeData)
{
	int status;
	IntrStatus_t IntStatus;

	status = VL6180_RangeGetInterruptStatus(&IntStatus.val);
	if (status == 0) {
		if (IntStatus.status.Range == RES_INT_STAT_GPIO_NEW_SAMPLE_READY ||
			IntStatus.status.Error != 0) {
			status = VL6180_RangeGetMeasurement(pRangeData);
			if (status == 0) {
				/*  clear range interrupt source */
				status = VL6180_RangeClearInterrupt();
			}
		} else {
			pRangeData->errorStatus = DataNotReady;
		}
	}
	return status;
}

int VL6180::VL6180_FilterSetState(int state)
{
	int status;
	#if VL6180_WRAP_AROUND_FILTER_SUPPORT
	VL6180DevDataSet(Device, WrapAroundFilterActive, state);
	status = 0;
	#else
	status =  NOT_SUPPORTED;
	#endif
	return status;
}

int VL6180::VL6180_FilterGetState()
{
	int status;
	#if VL6180_WRAP_AROUND_FILTER_SUPPORT
	status = VL6180DevDataGet(Device, WrapAroundFilterActive);
	#else
	status = 0;
	#endif
	return status;
}

int VL6180::VL6180_RangeGetResult(int32_t *pRange_mm)
{
	int status;
	uint8_t RawRange;
	int32_t Upscale;

	status = VL6180_GetCachedByte(RESULT_RANGE_VAL, &RawRange);
	if (!status) {
		Upscale = _GetUpscale();
		*pRange_mm = Upscale * (int32_t)RawRange;
	}
	return status;
}

int VL6180::VL6180_RangeSetRawThresholds(uint8_t low, uint8_t high)
{
	int status;
	/* TODO we can optimize here grouping high/low in a word but that's cpu endianness dependent */
	status = VL6180_WrByte(SYSRANGE_THRESH_HIGH, high);
	if (!status) {
		status = VL6180_WrByte(SYSRANGE_THRESH_LOW, low);
	}

	return status;
}

int VL6180::VL6180_RangeSetThresholds(uint16_t low, uint16_t high, int UseSafeParamHold)
{
	int status;
	int scale;
	scale = _GetUpscale();
	if (low > scale * 255 || high > scale * 255) {
		status = INVALID_PARAMS;
	} else {
		do {
			if (UseSafeParamHold) {
				status = VL6180_SetGroupParamHold(1);
				if (status)
					break;
		    }
		    status = VL6180_RangeSetRawThresholds((uint8_t)(low / scale), (uint8_t)(high / scale));
		    if (status) {
				break;
		    }
		    if (UseSafeParamHold) {
				int HoldStatus;
				/* tryt to unset param hold vene if previous fail */
				HoldStatus = VL6180_SetGroupParamHold(0);
				if (!status)
					status = HoldStatus;
		    }
		} while (0);
	}

	return status;
}


int VL6180::VL6180_RangeGetThresholds(uint16_t *low, uint16_t *high)
{
	int status = 0;
	uint8_t RawLow, RawHigh;
	int scale;

	scale = _GetUpscale();
	do {
		if (high != NULL) {
			status = VL6180_RdByte(SYSRANGE_THRESH_HIGH, &RawHigh);
			if (status) {
				break;
			}
			*high = (uint16_t)RawHigh * scale;
		}
		if (low != NULL) {
		    status = VL6180_RdByte(SYSRANGE_THRESH_LOW, &RawLow);
			if (status) {
				break;
		    }
		    *low = (uint16_t)RawLow * scale;
		}
	} while (0);
	return status;
}


int VL6180::VL6180_RangeGetInterruptStatus(uint8_t *pIntStatus)
{
	int status;
	uint8_t IntStatus;
	/* FIXME we are grouping "error" with over status the user must check implicitly for it
	 * not just new sample or over status , that will nevr show up in case of error*/
	status = VL6180_GetCachedByte(RESULT_INTERRUPT_STATUS_GPIO, &IntStatus);
	*pIntStatus = IntStatus & 0xC7;

	return status;
}


int VL6180::VL6180_GetInterruptStatus(uint8_t *IntStatus)
{
	int status;
	status = VL6180_RdByte(RESULT_INTERRUPT_STATUS_GPIO, IntStatus);
	return status;
}

int VL6180::VL6180_ClearInterrupt(uint8_t IntClear)
{
	int status;
	if (IntClear <= 7) {
		status = VL6180_WrByte(SYSTEM_INTERRUPT_CLEAR, IntClear);
	} else {
		status = INVALID_PARAMS;
	}
	return status;
}


int VL6180::VL6180_RangeStaticInit()
{
	int status;

	/* REGISTER_TUNING_SR03_270514_CustomerView.txt */
	VL6180_WrByte(0x0207, 0x01);
	VL6180_WrByte(0x0208, 0x01);
	VL6180_WrByte(0x0096, 0x00);
	VL6180_WrByte(0x0097, 0xfd);
	VL6180_WrByte(0x00e3, 0x00);
	VL6180_WrByte(0x00e4, 0x04);
	VL6180_WrByte(0x00e5, 0x02);
	VL6180_WrByte(0x00e6, 0x01);
	VL6180_WrByte(0x00e7, 0x03);
	VL6180_WrByte(0x00f5, 0x02);
	VL6180_WrByte(0x00d9, 0x05);
	VL6180_WrByte(0x00db, 0xce);
	VL6180_WrByte(0x00dc, 0x03);
	VL6180_WrByte(0x00dd, 0xf8);
	VL6180_WrByte(0x009f, 0x00);
	VL6180_WrByte(0x00a3, 0x3c);
	VL6180_WrByte(0x00b7, 0x00);
	VL6180_WrByte(0x00bb, 0x3c);
	VL6180_WrByte(0x00b2, 0x09);
	VL6180_WrByte(0x00ca, 0x09);
	VL6180_WrByte(0x0198, 0x01);
	VL6180_WrByte(0x01b0, 0x17);
	VL6180_WrByte(0x01ad, 0x00);
	VL6180_WrByte(0x00ff, 0x05);
	VL6180_WrByte(0x0100, 0x05);
	VL6180_WrByte(0x0199, 0x05);
	VL6180_WrByte(0x01a6, 0x1b);
	VL6180_WrByte(0x01ac, 0x3e);
	VL6180_WrByte(0x01a7, 0x1f);
	VL6180_WrByte(0x0030, 0x00);

	/* Recommended : Public registers - See data sheet for more detail */
	VL6180_WrByte(0x0011, 0x10); /* Enables polling for New Sample ready when measurement completes */
	VL6180_WrByte(0x010a, 0x30); /* Set the averaging sample period (compromise between lower noise and increased execution time) */
	VL6180_WrByte(0x003f, 0x46); /* Sets the light and dark gain (upper nibble). Dark gain should not be changed.*/
	VL6180_WrByte(0x0031, 0xFF); /* sets the # of range measurements after which auto calibration of system is performed */
	VL6180_WrByte(0x002e, 0x01); /* perform a single temperature calibration of the ranging sensor */

	/* Optional: Public registers - See data sheet for more detail */
	VL6180_WrByte(0x001b, 0x09); /* Set default ranging inter-measurement period to 100ms */
	VL6180_WrByte(0x0014, 0x24); /* Configures interrupt on New sample ready */


	status = VL6180_RangeSetMaxConvergenceTime(50); /*  Calculate ece value on initialization (use max conv) */

	return status;
}

int VL6180::_UpscaleInitPatch0()
{
	int status;
	uint32_t CalValue = 0;
	CalValue = VL6180DevDataGet(Device, Part2PartAmbNVM);
	status = VL6180_WrDWord(0xDA, CalValue);
	return status;
}

/* only include up-scaling register setting when up-scale support is configured in */
int VL6180::VL6180_UpscaleRegInit()
{
#if VL6180_UPSCALE_SUPPORT != 1
	/*  apply REGISTER_TUNING_ER02_100614_CustomerView.txt */
	VL6180_WrByte(0x0207, 0x01);
	VL6180_WrByte(0x0208, 0x01);
	VL6180_WrByte(0x0096, 0x00);
	VL6180_WrByte(0x0097, 0x54);
	VL6180_WrByte(0x00e3, 0x00);
	VL6180_WrByte(0x00e4, 0x04);
	VL6180_WrByte(0x00e5, 0x02);
	VL6180_WrByte(0x00e6, 0x01);
	VL6180_WrByte(0x00e7, 0x03);
	VL6180_WrByte(0x00f5, 0x02);
	VL6180_WrByte(0x00d9, 0x05);

	_UpscaleInitPatch0();

	VL6180_WrByte(0x009f, 0x00);
	VL6180_WrByte(0x00a3, 0x28);
	VL6180_WrByte(0x00b7, 0x00);
	VL6180_WrByte(0x00bb, 0x28);
	VL6180_WrByte(0x00b2, 0x09);
	VL6180_WrByte(0x00ca, 0x09);
	VL6180_WrByte(0x0198, 0x01);
	VL6180_WrByte(0x01b0, 0x17);
	VL6180_WrByte(0x01ad, 0x00);
	VL6180_WrByte(0x00ff, 0x05);
	VL6180_WrByte(0x0100, 0x05);
	VL6180_WrByte(0x0199, 0x05);
	VL6180_WrByte(0x01a6, 0x1b);
	VL6180_WrByte(0x01ac, 0x3e);
	VL6180_WrByte(0x01a7, 0x1f);
	VL6180_WrByte(0x0030, 0x00);
	VL6180_WrByte(0x0011, 0x10);
	VL6180_WrByte(0x010a, 0x30);
	VL6180_WrByte(0x003f, 0x46);
	VL6180_WrByte(0x0031, 0xFF);
	VL6180_WrByte(0x0040, 0x63);
	VL6180_WrByte(0x002e, 0x01);
	VL6180_WrByte(0x002c, 0xff);
	VL6180_WrByte(0x001b, 0x09);
	VL6180_WrByte(0x003e, 0x31);
	VL6180_WrByte(0x0014, 0x24);
#if VL6180_EXTENDED_RANGE
	VL6180_RangeSetMaxConvergenceTime(63);
#else
	VL6180_RangeSetMaxConvergenceTime(50);
#endif
	return 0;
#else
    return 0;
#endif
}

int VL6180::VL6180_UpscaleSetScaling(uint8_t scaling)
{
	int status;
	uint16_t Scaler;
	uint16_t ValidHeight;
	int8_t  Offset;

#ifdef VL6180_HAVE_UPSCALE_DATA
	#define min_scaling 1
	#define max_scaling (sizeof(ScalerLookUP) / sizeof(ScalerLookUP[0]))
#else
	/* we are in fixed config so only allow configured factor */
	#define min_scaling VL6180_UPSCALE_SUPPORT
	#define max_scaling VL6180_UPSCALE_SUPPORT
#endif

	if (scaling >= min_scaling  && scaling <= max_scaling) {

		Scaler = ScalerLookUP[scaling - 1];
		status = VL6180_WrWord(RANGE_SCALER, Scaler);
		_SetUpscale(scaling);

		/* Apply scaling on  part-2-part offset */
		Offset = VL6180DevDataGet(Device, Part2PartOffsetNVM) / scaling;
		status = VL6180_WrByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, Offset);

		/* Apply scaling on CrossTalkValidHeight */
		if (status == 0) {
			status = VL6180_WrByte(SYSRANGE_CROSSTALK_VALID_HEIGHT,
									DEF_CROSS_TALK_VALID_HEIGHT_VALUE /  scaling);
		}
		/* Apply scaling on RangeIgnore ValidHeight if enabled */
		if ( status == 0) {
			if (  VL6180DevDataGet(Device, RangeIgnore.Enabled) !=0 ) {
				ValidHeight = VL6180DevDataGet(Device, RangeIgnore.ValidHeight);
				ValidHeight  /= _GetUpscale();
				if( ValidHeight > 255 )
					ValidHeight = 255;

				status = VL6180_WrByte(SYSRANGE_RANGE_IGNORE_VALID_HEIGHT,
							(uint8_t)(ValidHeight & 0xFF) );
			}
		}

#if !VL6180_EXTENDED_RANGE
		if (status == 0) {
			status = VL6180_RangeSetEceState(scaling == 1); /* enable ece only at 1x scaling */
		}
		if (status == 0 && !VL6180_EXTENDED_RANGE && scaling != 1) {
			status = NOT_GUARANTEED ;
		}
#endif
	} else {
		status = INVALID_PARAMS;
	}
#undef min_scaling
#undef max_scaling
	return status;
}


int VL6180::VL6180_UpscaleGetScaling()
{
	int status;
	status = _GetUpscale();

	return status;
}


int VL6180::VL6180_UpscaleStaticInit()
{
	/* todo make these a fail macro in case only 1x is suppoted */
	int status;

	do {
		status = VL6180_UpscaleRegInit();
		if (status) {
			break;
		}
#if VL6180_EXTENDED_RANGE
		status = VL6180_RangeSetEceState(0);
		if (status) {
			break;
		}
#endif
	} while (0);
	if (!status) {
		/*  must write the scaler at least once to the device to ensure the scaler is in a known state. */
		status = VL6180_UpscaleSetScaling(_GetUpscale());
		VL6180_WrByte(0x016, 0x00); /* change fresh out of set status to 0 */
	}
	return status;
}


int VL6180::VL6180_SetGPIOxPolarity(int pin, int active_high)
{
	int status;

	if (pin == 0  || pin == 1) {
		uint16_t RegIndex;
		uint8_t  DataSet;
		if (pin == 0)
			RegIndex = SYSTEM_MODE_GPIO0;
		else
			RegIndex = SYSTEM_MODE_GPIO1;

		if (active_high)
		   DataSet = GPIOx_POLARITY_SELECT_MASK;
		else
		   DataSet = 0;

		status = VL6180_UpdateByte(RegIndex, (uint8_t)~GPIOx_POLARITY_SELECT_MASK, DataSet);
	} else {
		status = INVALID_PARAMS;
	}

	return status;
}

int VL6180::VL6180_SetGPIOxFunctionality(int pin, uint8_t functionality)
{
	int status;

	if (((pin == 0)  || (pin == 1))  && IsValidGPIOFunction(functionality)) {
		uint16_t RegIndex;

		if (pin == 0)
			RegIndex = SYSTEM_MODE_GPIO0;
		else
			RegIndex = SYSTEM_MODE_GPIO1;

		status = VL6180_UpdateByte(RegIndex, (uint8_t)~GPIOx_FUNCTIONALITY_SELECT_MASK,
									functionality << GPIOx_FUNCTIONALITY_SELECT_SHIFT);
	} else {
		status = INVALID_PARAMS;
	}

	return status;
}


int VL6180::VL6180_SetupGPIOx(int pin,  uint8_t IntFunction, int  ActiveHigh)
{
	int status;

	if (((pin == 0) || (pin == 1))  && IsValidGPIOFunction(IntFunction)) {
		uint16_t RegIndex;
		uint8_t value = 0;

		if (pin == 0)
		   RegIndex = SYSTEM_MODE_GPIO0;
		else
		   RegIndex = SYSTEM_MODE_GPIO1;

		if (ActiveHigh)
		   value |= GPIOx_POLARITY_SELECT_MASK;

		value |=  IntFunction << GPIOx_FUNCTIONALITY_SELECT_SHIFT;
		status = VL6180_WrByte(RegIndex, value);
	} else {
		status = INVALID_PARAMS;
	}

	return status;
}


int VL6180::VL6180_DisableGPIOxOut(int pin)
{
	int status;

	status = VL6180_SetGPIOxFunctionality(pin, GPIOx_SELECT_OFF);

	return status;
}


int VL6180::VL6180_SetupGPIO1(uint8_t IntFunction, int ActiveHigh)
{
	int status;
	status = VL6180_SetupGPIOx(1, IntFunction, ActiveHigh);
	return status;
}

int VL6180::VL6180_RangeConfigInterrupt(uint8_t ConfigGpioInt)
{
	int status;

	if (ConfigGpioInt <= CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY) {
		status = VL6180_UpdateByte(SYSTEM_INTERRUPT_CONFIG_GPIO,
									(uint8_t)(~CONFIG_GPIO_RANGE_MASK),
									ConfigGpioInt);
	} else {
		status = INVALID_PARAMS;
	}
	return status;
}


int VL6180::VL6180_RangeSetEceFactor(uint16_t  FactorM, uint16_t FactorD)
{
	int status;
	uint8_t u8;

	do {
		/* D cannot be 0 M must be <=D and >= 0 */
		if (FactorM <= FactorD  && FactorD > 0) {
			VL6180DevDataSet(Device, EceFactorM, FactorM);
			VL6180DevDataSet(Device, EceFactorD, FactorD);
			/* read and re-apply max conv time to get new ece factor set */
			status = VL6180_RdByte(SYSRANGE_MAX_CONVERGENCE_TIME, &u8);
			if (status) {
			   break;
			}
			status = VL6180_RangeSetMaxConvergenceTime(u8);
			if (status < 0) {
				break;
			}
		} else {
			status = INVALID_PARAMS;
		}
	} while (0);
	return status;
}

int VL6180::VL6180_RangeSetEceState(int enable)
{
	int status;
	uint8_t or_mask;

	if (enable)
		or_mask = RANGE_CHECK_ECE_ENABLE_MASK;
	else
		or_mask = 0;

	status = VL6180_UpdateByte(SYSRANGE_RANGE_CHECK_ENABLES, ~RANGE_CHECK_ECE_ENABLE_MASK, or_mask);
	return status;
}


int VL6180::VL6180_RangeSetMaxConvergenceTime(uint8_t  MaxConTime_msec)
{
	int status = 0;
	do {
		status = VL6180_WrByte(SYSRANGE_MAX_CONVERGENCE_TIME, MaxConTime_msec);
		if (status) {
			break;
		}
		status = VL6180_RangeSetEarlyConvergenceEestimateThreshold();
		if (status) {
			break;
		}
#if VL6180_HAVE_DMAX_RANGING
		status = _DMax_InitData();
#else
		status = 0;
#endif
	} while (0);
	return status;
}

int VL6180::VL6180_RangeSetInterMeasPeriod(uint32_t  InterMeasTime_msec)
{
	uint8_t SetTime;
	int status;

	do {
		if (InterMeasTime_msec > 2550) {
			status = INVALID_PARAMS;
			break;
		}
		/* doc in not 100% clear and confusing about the limit practically all value are OK but 0
		 * that can hang device in continuous mode */
		if (InterMeasTime_msec < 10) {
			InterMeasTime_msec = 10;
		}
		SetTime = (uint8_t)(InterMeasTime_msec / 10);
		status = VL6180_WrByte(SYSRANGE_INTERMEASUREMENT_PERIOD, SetTime);
		if (!status && SetTime != InterMeasTime_msec / 10) {
			status = MIN_CLIPED;  /* on success change status to clip if it did */
		}
	} while (0);
	return status;
}


int VL6180::VL6180_RangeGetDeviceReady(int *Ready)
{
	int status;
	uint8_t u8;
	status = VL6180_RdByte(RESULT_RANGE_STATUS, &u8);
	if (!status)
		*Ready = u8&RANGE_DEVICE_READY_MASK;
	return status;
}


int VL6180::VL6180_RangeWaitDeviceReady(int MaxLoop)
{
	int status = 0; /* if user specify an invalid <=0 loop count we'll return error */
	int  n;
	uint8_t u8;
	if (MaxLoop < 1) {
		status = INVALID_PARAMS;
	} else {
		for (n = 0; n < MaxLoop ; n++) {
			status = VL6180_RdByte(RESULT_RANGE_STATUS, &u8);
			if (status)
				break;
			u8 = u8 & RANGE_DEVICE_READY_MASK;
			if (u8)
				break;

		}
		if (!status && !u8) {
			status = TIME_OUT;
		}
	}
	return status;
}

int VL6180::VL6180_RangeSetSystemMode(uint8_t  mode)
{
	int status;
	/* FIXME we are not checking device is ready via @a VL6180_RangeWaitDeviceReady
	 * so if called back to back real fast we are not checking
	 * if previous mode "set" got absorbed => bit 0 must be 0 so that it work
	 */
	if (mode <= 3) {
		status = VL6180_WrByte(SYSRANGE_START, mode);
	} else {
		status = INVALID_PARAMS;
	}
	return status;
}


int VL6180::VL6180_RangeStartContinuousMode()
{
	int status;
	status = VL6180_RangeSetSystemMode(MODE_START_STOP | MODE_CONTINUOUS);
	return status;
}

int VL6180::VL6180_RangeStartSingleShot()
{
	int status;
	status = VL6180_RangeSetSystemMode(MODE_START_STOP | MODE_SINGLESHOT);
	return status;
}


int VL6180::VL6180_RangeSetEarlyConvergenceEestimateThreshold()
{
	int status;

	const uint32_t cMicroSecPerMilliSec  = 1000;
	const uint32_t cEceSampleTime_us     = 500;
	uint32_t ece_factor_m          = VL6180DevDataGet(Device, EceFactorM);
	uint32_t ece_factor_d          = VL6180DevDataGet(Device, EceFactorD);
	uint32_t convergTime_us;
	uint32_t fineThresh;
	uint32_t eceThresh;
	uint8_t  u8;
	uint32_t maxConv_ms;
	int32_t AveTime;

	do {
		status = VL6180_RdByte(SYSRANGE_MAX_CONVERGENCE_TIME, &u8);
		if (status) {
			break;
		}
		maxConv_ms = u8;
		AveTime = _GetAveTotalTime();
		if (AveTime < 0) {
			status = -1;
			break;
		}

		convergTime_us = maxConv_ms * cMicroSecPerMilliSec - AveTime;
		status = VL6180_RdDWord(0xB8, &fineThresh);
		if (status) {
			break;
		}
		fineThresh *= 256;
		eceThresh = ece_factor_m * cEceSampleTime_us * fineThresh / (convergTime_us * ece_factor_d);

		status = VL6180_WrWord(SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, (uint16_t)eceThresh);
	} while (0);

	return status;
}


int VL6180::_RangeIgnore_UpdateDevice(){
	int status;
	int enable;
	int threshold;
	int range;
	int or_mask;
	enable= VL6180DevDataGet(Device, RangeIgnore.Enabled);
	if( enable ){
		// if to be nabled program first range value and threshold
		range = VL6180DevDataGet(Device, RangeIgnore.ValidHeight);
		range /= _GetUpscale();
		if( range > 255 )
			range = 255;

		status = VL6180_WrByte(SYSRANGE_RANGE_IGNORE_VALID_HEIGHT,  range);
		if( status ){
			goto done;
		}

		threshold = VL6180DevDataGet(Device, RangeIgnore.IgnoreThreshold);
		status = VL6180_WrWord(SYSRANGE_RANGE_IGNORE_THRESHOLD,  threshold);
		if( status ){
			goto done;
		}
		or_mask = RANGE_CHECK_RANGE_ENABLE_MASK;
	}
	else{
		or_mask = 0;
	}
	status = VL6180_UpdateByte(SYSRANGE_RANGE_CHECK_ENABLES, ~RANGE_CHECK_RANGE_ENABLE_MASK, or_mask);
#if VL6180_HAVE_DMAX_RANGING
	_DMax_InitData();
#endif
done:
	return status;
}

int VL6180::VL6180_RangeIgnoreSetEnable(int EnableState){
	int CurEnable;
	int status=0;

	if( EnableState )
		EnableState = 1;

	CurEnable = VL6180DevDataGet(Device, RangeIgnore.Enabled);
	if( EnableState != CurEnable  ){
		VL6180DevDataSet(Device, RangeIgnore.Enabled, EnableState);
		status = _RangeIgnore_UpdateDevice();
	}
	return status;
}

int VL6180::VL6180_RangeIgnoreConfigure(uint16_t ValidHeight_mm, uint16_t IgnoreThreshold){
	int status;
	int enabled;

	enabled = VL6180DevDataGet(Device, RangeIgnore.Enabled);
	VL6180DevDataSet(Device, RangeIgnore.ValidHeight, ValidHeight_mm);
	VL6180DevDataSet(Device, RangeIgnore.IgnoreThreshold, IgnoreThreshold);
	if(  enabled ){
		status = _RangeIgnore_UpdateDevice();
	}
	else{
		status = 0;
	}
	return status;
}

/*
 * Return >0 = time
 *       <0 1 if fail to get read data from device to compute time
 */
int32_t VL6180::_GetAveTotalTime()
{
	uint32_t cFwOverhead_us = 24;
	uint32_t cVcpSetupTime_us = 70;
	uint32_t cPLL2_StartupDelay_us = 200;
	uint8_t cMeasMask = 0x07;
	uint32_t Samples;
	uint32_t SamplePeriod;
	uint32_t SingleTime_us;
	int32_t TotalAveTime_us;
	uint8_t u8;
	int status;

	status = VL6180_RdByte(0x109, &u8);
	if (status) {
		return -1;
	}
	Samples = u8 & cMeasMask;
	status = VL6180_RdByte(READOUT_AVERAGING_SAMPLE_PERIOD, &u8);
	if (status) {
		return -1;
	}
	SamplePeriod = u8;
	SingleTime_us = cFwOverhead_us + cVcpSetupTime_us + (SamplePeriod * 10);
	TotalAveTime_us = (Samples + 1) * SingleTime_us + cPLL2_StartupDelay_us;

	return TotalAveTime_us;
}

#if VL6180_HAVE_DMAX_RANGING
#define _GetDMaxDataRetSignalAt400mm() VL6180DevDataGet(Device, DMaxData.retSignalAt400mm)
#else
#define _GetDMaxDataRetSignalAt400mm() 375 /* Use a default high value */
#endif


#if VL6180_WRAP_AROUND_FILTER_SUPPORT

#define PRESERVE_DEVICE_ERROR_CODE		/* If uncommented, device error code will be preserved on top of wraparound error code, but this may lead to some error code instability like overflow error <==> RangingFilteringOnGoing error oscillations */
#define SENSITIVE_FILTERING_ON_GOING	/* If uncommented, filter will go back to RangingFilteringOnGoing if it must go through the std dev testing */

#define FILTER_STDDEV_SAMPLES           6
#define MIN_FILTER_STDDEV_SAMPLES       3
#define MIN_FILTER_STDDEV_SAMPLES_AFTER_FLUSH_OR_BYPASS 5
#define STDDEV_BASE_VALUE               150

#define FILTER_INVALID_DISTANCE     65535

#define _FilterData(field) VL6180DevDataGet(Device, FilterData.field)
/*
 * One time init
 */
int VL6180::_filter_Init()
{
	int i;
	_FilterData(MeasurementIndex) = 0;

	_FilterData(Default_ZeroVal) = 0;
	_FilterData(Default_VAVGVal) = 0;
	_FilterData(NoDelay_ZeroVal) = 0;
	_FilterData(NoDelay_VAVGVal) = 0;
	_FilterData(Previous_VAVGDiff) = 0;

	_FilterData(StdFilteredReads) = 0;
	_FilterData(FilteringOnGoingConsecutiveStates) = 0;

	for (i = 0; i < FILTER_NBOF_SAMPLES; i++) {
		_FilterData(LastTrueRange)[i] = FILTER_INVALID_DISTANCE;
		_FilterData(LastReturnRates)[i] = 0;
	}
	_FilterData(MeasurementsSinceLastFlush)=0;
	return 0;
}


uint32_t VL6180::_filter_StdDevDamper(uint32_t AmbientRate,
									uint32_t SignalRate,
									const uint32_t StdDevLimitLowLight,
									const uint32_t StdDevLimitLowLightSNR,
									const uint32_t StdDevLimitHighLight,
									const uint32_t StdDevLimitHighLightSNR)
{
	uint32_t newStdDev;
	uint16_t SNR;

	if (AmbientRate > 0)
		SNR = (uint16_t) ((100 * SignalRate) / AmbientRate);
	else
		SNR = 9999;

	if (SNR >= StdDevLimitLowLightSNR) {
		newStdDev = StdDevLimitLowLight;
	} else {
		if (SNR <= StdDevLimitHighLightSNR)
			newStdDev = StdDevLimitHighLight;
		else {
			newStdDev = (uint32_t)(StdDevLimitHighLight -
									(SNR - StdDevLimitHighLightSNR) *
									(StdDevLimitHighLight - StdDevLimitLowLight) /
									(StdDevLimitLowLightSNR - StdDevLimitHighLightSNR));
		}
	}

	return newStdDev;
}


/*
 * Return <0 on error
 */
int32_t VL6180::_filter_Start(uint16_t m_trueRange_mm,
								uint16_t m_rawRange_mm,
								uint32_t m_rtnSignalRate,
								uint32_t m_rtnAmbientRate,
								uint16_t errorCode)
{
	int status;
	uint16_t m_newTrueRange_mm = 0;
	#if VL6180_HAVE_MULTI_READ
	uint8_t MultiReadBuf[8];
	#endif
	uint16_t i;
	uint16_t bypassFilter = 0;
	uint16_t resetVAVGData = 1;

	uint16_t filterErrorCode = NoError;
	uint16_t filterErrorCodeOnRangingErrorCode = NoError;

	uint16_t registerValue;

	uint32_t register32BitsValue1;
	uint32_t register32BitsValue2;

	uint16_t ValidDistance = 0;
	uint16_t SuspicuousRangingZone = 0;

	uint16_t WrapAroundFlag = 0;
	uint16_t NoWrapAroundFlag = 0;
	uint16_t NoWrapAroundHighConfidenceFlag = 0;

	uint16_t FlushFilter = 0;
	uint32_t RateChange = 0;

	uint16_t StdDevSamplesMinNeeded = 0;
	uint16_t StdDevSamples = 0;
	uint32_t StdDevDistanceSum = 0;
	uint32_t StdDevDistanceMean = 0;
	uint32_t StdDevDistance = 0;
	uint32_t StdDevRateSum = 0;
	uint32_t StdDevRateMean = 0;
	uint32_t StdDevRate = 0;
	uint32_t StdDevLimitWithTargetMove = 0;

	uint32_t VAVGDiff;
	uint32_t IdealVAVGDiff;
	uint32_t MinVAVGDiff;
	uint32_t MaxVAVGDiff;

	/* Filter Parameters */
	static const uint16_t  WrapAroundLowRawRangeLimit = 60;
	static const uint32_t  WrapAroundLowReturnRateLimit_ROM = 800;
	/* Shall be adapted depending on crossTalk */
	static const uint16_t  WrapAroundLowRawRangeLimit2 = 165;
	static const uint32_t  WrapAroundLowReturnRateLimit2_ROM = 180;
	/* Shall be adapted depending on crossTalk and device sensitivity*/
	static const uint32_t  WrapAroundLowRawRangeLimit2SuspicuousAddedSignalRate = 150;


	static const uint32_t  WrapAroundLowReturnRateFilterLimit_ROM = 850;
	/* Shall be adapted depending on crossTalk and device sensitivity*/
	static const uint16_t  WrapAroundHighRawRangeFilterLimit = 350;
	static const uint32_t  WrapAroundHighReturnRateFilterLimit_ROM = 1400;
	/* Shall be adapted depending on crossTalk and device sensitivity*/

	static const uint32_t  WrapAroundMaximumAmbientRateFilterLimit = 15000;

	/*  Temporal filter data and flush values */
	static const uint32_t  MinReturnRateFilterFlush = 75;
	static const uint32_t  MaxReturnRateChangeFilterFlush = 50;

	/* STDDEV values and damper values */
	static const uint32_t  StdDevLimitLowLight = STDDEV_BASE_VALUE;
	static const uint32_t  StdDevLimitLowLightSNR = 30; /* 0.3 */
	static const uint32_t  StdDevLimitHighLight = STDDEV_BASE_VALUE*6;
	static const uint32_t  StdDevLimitHighLightSNR = 5; /* 0.05 */

	static const uint32_t  StdDevHighConfidenceSNRLimit = 8;
	static const uint32_t  StdDevNoWrapDetectedMultiplier = 4;

	static const uint32_t  StdDevMovingTargetStdDevLimit = 90000;

	static const uint32_t  StdDevMovingTargetReturnRateLimit = 3500;
	static const uint32_t  StdDevMovingTargetStdDevForReturnRateLimit = STDDEV_BASE_VALUE*25;

	static const uint32_t  MAX_VAVGDiff_ROM = 1800;
	static const uint32_t  SuspicuousMAX_VAVGDiffRatio = 2;

	/* WrapAroundDetection variables */
	static const uint16_t  WrapAroundNoDelayCheckPeriod = 2;
	static const uint16_t  StdFilteredReadsIncrement = 2;
	static const uint16_t  StdFilteredReadsDecrement = 1;
	static const uint16_t  StdMaxFilteredReads = 4;

	uint32_t SignalRateDMax;
	uint32_t WrapAroundLowReturnRateLimit;
	uint32_t WrapAroundLowReturnRateLimit2;
	uint32_t WrapAroundLowReturnRateFilterLimit;
	uint32_t WrapAroundHighReturnRateFilterLimit;

	uint32_t MAX_VAVGDiff = 1800;

	uint8_t u8;//, u8_2;
	uint32_t XTalkCompRate_KCps;
	uint32_t StdDevLimit = 300;
	uint32_t MaxOrInvalidDistance =   255*_GetUpscale();
	/* #define MaxOrInvalidDistance  (uint16_t) (255 * 3) */

	/* Check if distance is Valid or not */
	switch (errorCode) {
	case Raw_Ranging_Algo_Underflow:
	case Ranging_Algo_Underflow:
		filterErrorCodeOnRangingErrorCode = RangingFiltered; /* If we have to go through filter, mean we have here a wraparound case */
		ValidDistance = 0;
		break;
	case Raw_Ranging_Algo_Overflow:
	case Ranging_Algo_Overflow:
		filterErrorCodeOnRangingErrorCode = RangingFiltered; /* If we have to go through filter, mean we have here a wraparound case */
		//m_trueRange_mm = MaxOrInvalidDistance;
		m_trueRange_mm = 200*_GetUpscale();
		ValidDistance = 1;
		break;
	default:
		if (m_rawRange_mm >= MaxOrInvalidDistance) {
			ValidDistance = 0;
			bypassFilter = 1; /* Bypass the filter in this case as produced distance is not usable (and also the VAVGVal and ZeroVal values) */
		} else {
			ValidDistance = 1;
		}
		break;
	}
	m_newTrueRange_mm = m_trueRange_mm;

	XTalkCompRate_KCps = VL6180DevDataGet(Device, XTalkCompRate_KCps);

	/* Update signal rate limits depending on crosstalk */
	SignalRateDMax = (uint32_t)_GetDMaxDataRetSignalAt400mm() ;
	WrapAroundLowReturnRateLimit = WrapAroundLowReturnRateLimit_ROM  + XTalkCompRate_KCps;
	WrapAroundLowReturnRateLimit2 = ((WrapAroundLowReturnRateLimit2_ROM *
									SignalRateDMax) / 312) +
									XTalkCompRate_KCps;
	WrapAroundLowReturnRateFilterLimit = ((WrapAroundLowReturnRateFilterLimit_ROM *
									SignalRateDMax) / 312) + XTalkCompRate_KCps;
	WrapAroundHighReturnRateFilterLimit = ((WrapAroundHighReturnRateFilterLimit_ROM *
									SignalRateDMax) / 312) + XTalkCompRate_KCps;


	/* Checks on low range data */
	if ((m_rawRange_mm < WrapAroundLowRawRangeLimit) && (m_rtnSignalRate < WrapAroundLowReturnRateLimit)) {
		filterErrorCode = RangingFiltered; /* On this condition, wraparound case is ensured */
		bypassFilter = 1;
	}
	if ((m_rawRange_mm < WrapAroundLowRawRangeLimit2) && (m_rtnSignalRate < WrapAroundLowReturnRateLimit2)) {
		filterErrorCode = RangingFiltered; /* On this condition, wraparound case is ensured */
		bypassFilter = 1;
	}
	if ((m_rawRange_mm < WrapAroundLowRawRangeLimit2) && (m_rtnSignalRate < (WrapAroundLowReturnRateLimit2 + WrapAroundLowRawRangeLimit2SuspicuousAddedSignalRate))) {
		SuspicuousRangingZone = 1; /* On this area, we are in an highly suspicuous wraparound ares, filter parameter will be stengthen */
	}


	/* Checks on Ambient rate level */
	if (m_rtnAmbientRate > WrapAroundMaximumAmbientRateFilterLimit) {
		/* Too high ambient rate */
		FlushFilter = 1;
		bypassFilter = 1;
	}
    
	/*  Checks on Filter flush */
	if (m_rtnSignalRate < MinReturnRateFilterFlush) {
		/* Completely lost target, so flush the filter */
		FlushFilter = 1;
		bypassFilter = 1;
	}
	if (_FilterData(LastReturnRates)[0] != 0) {
		if (m_rtnSignalRate > _FilterData(LastReturnRates)[0])
			RateChange = (100 *
						(m_rtnSignalRate - _FilterData(LastReturnRates)[0])) /
						_FilterData(LastReturnRates)[0];
		else
			RateChange = (100 *
						(_FilterData(LastReturnRates)[0] - m_rtnSignalRate)) /
						_FilterData(LastReturnRates)[0];
	} else
		RateChange = 0;
	if (RateChange > MaxReturnRateChangeFilterFlush) {
		FlushFilter = 1;
	}
	/* TODO optimize filter  using circular buffer */
	if (FlushFilter == 1) {
		_FilterData(MeasurementIndex) = 0;
		for (i = 0; i < FILTER_NBOF_SAMPLES; i++) {
			_FilterData(LastTrueRange)[i] = FILTER_INVALID_DISTANCE;
			_FilterData(LastReturnRates)[i] = 0;
		}
		_FilterData(MeasurementsSinceLastFlush)=0;
	} else {
		for (i = (uint16_t) (FILTER_NBOF_SAMPLES - 1); i > 0; i--) {
			_FilterData(LastTrueRange)[i] = _FilterData(LastTrueRange)[i - 1];
			_FilterData(LastReturnRates)[i] = _FilterData(LastReturnRates)[i - 1];
		}
	}

	if (ValidDistance == 1)
		_FilterData(LastTrueRange)[0] = m_trueRange_mm;
	else
		_FilterData(LastTrueRange)[0] = FILTER_INVALID_DISTANCE;
	_FilterData(LastReturnRates)[0] = m_rtnSignalRate;
	_FilterData(MeasurementsSinceLastFlush)++;

	/* Check if we need to go through the filter or not */
	if (!(((m_rawRange_mm < WrapAroundHighRawRangeFilterLimit) &&
		(m_rtnSignalRate < WrapAroundLowReturnRateFilterLimit)) ||
		((m_rawRange_mm >= WrapAroundHighRawRangeFilterLimit) &&
		(m_rtnSignalRate < WrapAroundHighReturnRateFilterLimit))))
		bypassFilter = 1;
	else {
		/* if some wraparound filtering due to some ranging error code has been detected, update the filter status and bypass the filter */
		if(filterErrorCodeOnRangingErrorCode!=NoError){
#ifndef PRESERVE_DEVICE_ERROR_CODE
			filterErrorCode = filterErrorCodeOnRangingErrorCode;
#else
			if((errorCode==Raw_Ranging_Algo_Underflow) || (errorCode==Ranging_Algo_Underflow)) {
				/* Preserves the error codes except for Raw_Ranging_Algo_Underflow and Ranging_Algo_Underflow */
				filterErrorCode = filterErrorCodeOnRangingErrorCode;
			}
#endif
			bypassFilter = 1;
			resetVAVGData = 0;
		}
	}

	/* Check which kind of measurement has been made */
	status = VL6180_RdByte(0x01AC, &u8);
	if (status) {
		goto done_err;
	}
	registerValue = u8;

	/* Read data for filtering */
#if VL6180_HAVE_MULTI_READ
	status = VL6180_ReadMulti(0x10C, MultiReadBuf, 8); /* read only 8 lsb bits */
	if (status) {
		goto done_err;
	}
	register32BitsValue1 = ((uint32_t) MultiReadBuf[0] << 24)
			+ ((uint32_t) MultiReadBuf[1] << 16)
			+ ((uint32_t) MultiReadBuf[2] << 8)
			+ ((uint32_t) MultiReadBuf[3] << 0);
	register32BitsValue2 = ((uint32_t) MultiReadBuf[4] << 24)
			+ ((uint32_t) MultiReadBuf[5] << 16)
			+ ((uint32_t) MultiReadBuf[6] << 8)
			+ ((uint32_t) MultiReadBuf[7] << 0);
#else
	status = VL6180_RdDWord(0x10C, &register32BitsValue1); /* read 32 bits, lower 17 bits are the one useful */
	if (status) {
		goto done_err;
	}
	status = VL6180_RdDWord(0x0110, &	register32BitsValue2); /* read 32 bits, lower 17 bits are the one useful */
	if (status) {
		goto done_err;
	}
#endif


	if ((FlushFilter == 1) || ((bypassFilter == 1) && (resetVAVGData == 1))) {
		if (registerValue != 0x3E) {
			status = VL6180_WrByte(0x1AC, 0x3E);
			if (status) {
				goto done_err;
			}
		}
		/* Set both Default and NoDelay To same value */
		_FilterData(Default_ZeroVal) = register32BitsValue1;
		_FilterData(Default_VAVGVal) = register32BitsValue2;
		_FilterData(NoDelay_ZeroVal) = register32BitsValue1;
		_FilterData(NoDelay_VAVGVal) = register32BitsValue2;

		_FilterData(MeasurementIndex) = 0;
	} else {
		if (registerValue == 0x3E) {
			_FilterData(Default_ZeroVal) = register32BitsValue1;
			_FilterData(Default_VAVGVal) = register32BitsValue2;
		} else {
			_FilterData(NoDelay_ZeroVal) = register32BitsValue1;
			_FilterData(NoDelay_VAVGVal) = register32BitsValue2;
		}

		if (_FilterData(MeasurementIndex) % WrapAroundNoDelayCheckPeriod == 0) {
			u8 = 0x3C;
			//u8_2 = 0x05;
		} else {
			u8 = 0x3E;
			//u8_2 = 0x01;
		}
		status = VL6180_WrByte(0x01AC, u8);
		if (status) {
			goto done_err;
		}
		_FilterData(MeasurementIndex)++;
	}

	if (bypassFilter == 1) {
		/* Do not go through the filter */

		/* Update filter error code */
		_FilterData(filterError) = filterErrorCode;

		/* Update reported range */
		if(filterErrorCode==RangingFiltered)
			m_newTrueRange_mm = MaxOrInvalidDistance; /* Set to invalid distance */

		return m_newTrueRange_mm;
	}

	/* Computes current VAVGDiff */
	if (_FilterData(Default_VAVGVal) > _FilterData(NoDelay_VAVGVal))
		VAVGDiff = _FilterData(Default_VAVGVal) - _FilterData(NoDelay_VAVGVal);
	else
		VAVGDiff = 0;
	_FilterData(Previous_VAVGDiff) = VAVGDiff;

	if(SuspicuousRangingZone==0)
		MAX_VAVGDiff = MAX_VAVGDiff_ROM;
	else
		/* In suspicuous area, strengthen the filter */
		MAX_VAVGDiff = MAX_VAVGDiff_ROM / SuspicuousMAX_VAVGDiffRatio;

	/* Check the VAVGDiff */
	if (_FilterData(Default_ZeroVal) > _FilterData(NoDelay_ZeroVal))
		IdealVAVGDiff = _FilterData(Default_ZeroVal) - _FilterData(NoDelay_ZeroVal);
	else
		IdealVAVGDiff = _FilterData(NoDelay_ZeroVal) - _FilterData(Default_ZeroVal);
	if (IdealVAVGDiff > MAX_VAVGDiff)
		MinVAVGDiff = IdealVAVGDiff - MAX_VAVGDiff;
	else
		MinVAVGDiff = 0;
	MaxVAVGDiff = IdealVAVGDiff + MAX_VAVGDiff;
	if (VAVGDiff < MinVAVGDiff || VAVGDiff > MaxVAVGDiff) {
		WrapAroundFlag = 1;
		filterErrorCode = RangingFiltered;
	} else {
		/* Go through filtering check */

		if(_FilterData(MeasurementIndex)<=1)
			/* On measurement after a bypass, uses an increase number of samples */
			StdDevSamplesMinNeeded = MIN_FILTER_STDDEV_SAMPLES_AFTER_FLUSH_OR_BYPASS;
		else
			StdDevSamplesMinNeeded = MIN_FILTER_STDDEV_SAMPLES;

		/* StdDevLimit Damper on SNR */
		StdDevLimit = _filter_StdDevDamper(m_rtnAmbientRate, m_rtnSignalRate, StdDevLimitLowLight, StdDevLimitLowLightSNR, StdDevLimitHighLight, StdDevLimitHighLightSNR);

		/* Standard deviations computations */
		StdDevSamples = 0;
		StdDevDistanceSum = 0;
		StdDevDistanceMean = 0;
		StdDevDistance = 0;
		StdDevRateSum = 0;
		StdDevRateMean = 0;
		StdDevRate = 0;
		for (i = 0; (i < FILTER_NBOF_SAMPLES) && (StdDevSamples < FILTER_STDDEV_SAMPLES); i++) {
			if (_FilterData(LastTrueRange)[i] != FILTER_INVALID_DISTANCE) {
				StdDevSamples = (uint16_t) (StdDevSamples + 1);
				StdDevDistanceSum = (uint32_t) (StdDevDistanceSum + _FilterData(LastTrueRange)[i]);
				StdDevRateSum = (uint32_t) (StdDevRateSum + _FilterData(LastReturnRates)[i]);
			}
		}
		if (StdDevSamples > 0) {
			StdDevDistanceMean = (uint32_t) (StdDevDistanceSum / StdDevSamples);
			StdDevRateMean = (uint32_t) (StdDevRateSum / StdDevSamples);
		}
		/* TODO optimize shorten Std dev in aisngle loop computation using sum of x2 - (sum of x)2 */
		StdDevSamples = 0;
		StdDevDistanceSum = 0;
		StdDevRateSum = 0;
		for (i = 0; (i < FILTER_NBOF_SAMPLES) && (StdDevSamples < FILTER_STDDEV_SAMPLES); i++) {
			if (_FilterData(LastTrueRange)[i] != FILTER_INVALID_DISTANCE) {
				StdDevSamples = (uint16_t) (StdDevSamples + 1);
				StdDevDistanceSum = (uint32_t) (StdDevDistanceSum +
									(int)(_FilterData(LastTrueRange)[i] -
											StdDevDistanceMean) *
											(int) (_FilterData(LastTrueRange)[i] -
													StdDevDistanceMean));
				StdDevRateSum = (uint32_t) (StdDevRateSum +
									(int) (_FilterData(LastReturnRates)[i] -
											StdDevRateMean) *
											(int) (_FilterData(LastReturnRates)[i] -
													StdDevRateMean));
			}
		}
		if (StdDevSamples >= StdDevSamplesMinNeeded) {
			StdDevDistance = (uint16_t) (StdDevDistanceSum / StdDevSamples);
			StdDevRate = (uint16_t) (StdDevRateSum / StdDevSamples);
		} else {
			StdDevDistance = 0;
			StdDevRate = 0;
		}

		/* Check Return rate standard deviation */
		if (StdDevRate < StdDevMovingTargetStdDevLimit) {
			if (StdDevSamples < StdDevSamplesMinNeeded) {
				//m_newTrueRange_mm = MaxOrInvalidDistance;
				filterErrorCode = RangingFiltered;
			} else {
				/* Check distance standard deviation */
				if (StdDevRate < StdDevMovingTargetReturnRateLimit)
					StdDevLimitWithTargetMove = StdDevLimit +
						(((StdDevMovingTargetStdDevForReturnRateLimit -
							StdDevLimit) * StdDevRate) /
							StdDevMovingTargetReturnRateLimit);
				else
					StdDevLimitWithTargetMove = StdDevMovingTargetStdDevForReturnRateLimit;

				if(_FilterData(filterError)==NoError){
					/* No wrapAround detected yet, so relax constraints on the std dev */
					StdDevLimitWithTargetMove = StdDevLimitWithTargetMove * StdDevNoWrapDetectedMultiplier;
				}

				if (((StdDevDistance * StdDevHighConfidenceSNRLimit) < StdDevLimit) && (StdDevSamples>=FILTER_STDDEV_SAMPLES)) {
					NoWrapAroundHighConfidenceFlag = 1;
				} else {
					if (StdDevDistance < StdDevLimitWithTargetMove) {
							NoWrapAroundFlag = 1;
						} else {
						WrapAroundFlag = 1;
						filterErrorCode = RangingFiltered;
					}
				}
			}
		} else {
			/* Target moving too fast */
			WrapAroundFlag = 1;
			filterErrorCode = RangingFiltered;
		}
	}

	if (ValidDistance == 0) {
		/* In case of invalid distance */
		if (_FilterData(StdFilteredReads) > 0)
			_FilterData(StdFilteredReads) = (uint16_t) (_FilterData(StdFilteredReads) - 1);
	} else {
		if (WrapAroundFlag == 1) {
			_FilterData(StdFilteredReads) = (uint16_t) (_FilterData(StdFilteredReads) +
											StdFilteredReadsIncrement);
			if (_FilterData(StdFilteredReads) > StdMaxFilteredReads)
				_FilterData(StdFilteredReads) = StdMaxFilteredReads;
		} else {
			if (NoWrapAroundFlag == 1) {
				if (_FilterData(StdFilteredReads) > 0) {
					filterErrorCode = RangingFiltered;
					if (_FilterData(StdFilteredReads) > StdFilteredReadsDecrement)
						_FilterData(StdFilteredReads) = (uint16_t) (_FilterData(StdFilteredReads) -
														StdFilteredReadsDecrement);
					else
						_FilterData(StdFilteredReads) = 0;
				}
			} else {
				if (NoWrapAroundHighConfidenceFlag == 1) {
					_FilterData(StdFilteredReads) = 0;
				}
			}
		}
	}

	/* If we detect a change from no Error to RangingFilteringOnGoing, then it means that
	 * the filter detected a change in te scene, so discard all previous measurements.
	 */
	if((_FilterData(filterError) == NoError) && (filterErrorCode!=NoError)) {
		for (i = 1; i < FILTER_NBOF_SAMPLES; i++) {
			_FilterData(LastTrueRange)[i] = FILTER_INVALID_DISTANCE;
			_FilterData(LastReturnRates)[i] = 0;
		}
	}

	/* Update filter error code */
	_FilterData(filterError) = filterErrorCode;

	/* Update reported range */
	if(filterErrorCode==RangingFiltered)
		m_newTrueRange_mm = MaxOrInvalidDistance; /* Set to invalid distance */

	return m_newTrueRange_mm;
done_err:
	return -1;

#undef MaxOrInvalidDistance
}


int VL6180::_filter_GetResult(VL6180_RangeData_t *pRangeData)
{
	uint32_t m_rawRange_mm = 0;
	int32_t  FilteredRange;
	const uint8_t scaler = _GetUpscale();
	uint8_t u8;
	int status;

	do {
		status = VL6180_GetCachedByte(RESULT_RANGE_RAW, &u8);
		if (status) {
		    break;
		}
		m_rawRange_mm = u8;

		FilteredRange = _filter_Start(pRangeData->range_mm, (m_rawRange_mm * scaler), pRangeData->rtnRate, pRangeData->rtnAmbRate, pRangeData->errorStatus);
		if (FilteredRange < 0) {
		    status = -1;
		    break;
		}
		pRangeData->FilteredData.range_mm = FilteredRange;
		pRangeData->FilteredData.rawRange_mm = m_rawRange_mm * scaler;
		pRangeData->FilteredData.filterError= _FilterData(filterError);
	} while (0);
	return status;
}

#undef _FilterData
#ifdef PRESERVE_DEVICE_ERROR_CODE
#undef PRESERVE_DEVICE_ERROR_CODE
#endif
#ifdef SENSITIVE_FILTERING_ON_GOING
#undef SENSITIVE_FILTERING_ON_GOING
#endif
#undef FILTER_STDDEV_SAMPLES
#undef MIN_FILTER_STDDEV_SAMPLES
#undef MIN_FILTER_STDDEV_SAMPLES_AFTER_FLUSH_OR_BYPASS
#undef STDDEV_BASE_VALUE
#undef FILTER_INVALID_DISTANCE

#endif /* VL6180_WRAP_AROUND_FILTER_SUPPORT */

#ifdef VL6180_HAVE_RATE_DATA

int VL6180::_GetRateResult(VL6180_RangeData_t *pRangeData)
{
	uint32_t m_rtnConvTime = 0;
	uint32_t m_rtnSignalRate = 0;
	uint32_t m_rtnAmbientRate = 0;
	uint32_t m_rtnSignalCount = 0;
	uint32_t m_rtnAmbientCount = 0;
	uint32_t m_refConvTime = 0;
	uint32_t cRtnSignalCountMax = 0x7FFFFFFF;
	uint32_t cDllPeriods = 6;
	uint32_t calcConvTime = 0;

	int status;

	do {
		status = VL6180_GetCachedDWord(RESULT_RANGE_RETURN_SIGNAL_COUNT, &m_rtnSignalCount);
		if (status) {
			break;
		}
		if (m_rtnSignalCount > cRtnSignalCountMax) {
			m_rtnSignalCount = 0;
		}

		status = VL6180_GetCachedDWord(RESULT_RANGE_RETURN_AMB_COUNT, &m_rtnAmbientCount);
		if (status) {
			break;
		}


		status = VL6180_GetCachedDWord(RESULT_RANGE_RETURN_CONV_TIME, &m_rtnConvTime);
		if (status) {
			break;
		}

		status = VL6180_GetCachedDWord(RESULT_RANGE_REFERENCE_CONV_TIME, &m_refConvTime);
		if (status) {
			break;
		}

		pRangeData->rtnConvTime = m_rtnConvTime;
		pRangeData->refConvTime = m_refConvTime;

		calcConvTime = m_refConvTime;
		if (m_rtnConvTime > m_refConvTime) {
			calcConvTime = m_rtnConvTime;
		}
		if (calcConvTime == 0)
			calcConvTime = 63000;

		m_rtnSignalRate = (m_rtnSignalCount * 1000) / calcConvTime;
		m_rtnAmbientRate = (m_rtnAmbientCount * cDllPeriods * 1000) / calcConvTime;

		pRangeData->rtnRate = m_rtnSignalRate;
		pRangeData->rtnAmbRate = m_rtnAmbientRate;


	} while (0);
	return status;
}
#endif /* VL6180_HAVE_RATE_DATA */


int VL6180::VL6180_DMaxSetState(int state)
{
	int status;
#if VL6180_HAVE_DMAX_RANGING
	VL6180DevDataSet(Device, DMaxEnable, state);
	if (state) {
		status = _DMax_InitData();
	} else {
		status = 0;
	}
#else
    (void)state;
	status =  NOT_SUPPORTED;
#endif
	return status;
}

int VL6180::VL6180_DMaxGetState()
{
	int status;
#if VL6180_HAVE_DMAX_RANGING
	status = VL6180DevDataGet(Device, DMaxEnable);
#else
	status = 0;
#endif
	return status;
}


#if VL6180_HAVE_DMAX_RANGING

#define _DMaxData(field) VL6180DevDataGet(Device, DMaxData.field)
/*
 * Convert fix point  x.7 to KCpount per sec
 */

#ifndef VL6180_PLATFORM_PROVIDE_SQRT

/*
 * 32 bit integer square root with not so bad precision (integer result) and is quite fast
 * see http://en.wikipedia.org/wiki/Methods_of_computing_square_roots
 */
uint32_t VL6180::VL6180_SqrtUint32(uint32_t num)
{
	uint32_t res = 0;
	uint32_t bit = 1 << 30; /* The second-to-top bit is set: 1 << 30 for 32 bits */

	/* "bit" starts at the highest power of four <= the argument. */
	while (bit > num)
		bit >>= 2;

	while (bit != 0) {
		if (num >= res + bit) {
		    num -= res + bit;
		    res = (res >> 1) + bit;
		} else
		    res >>= 1;
		bit >>= 2;
	}
	return res;
}
#endif


/* DMax one time init */
void VL6180::_DMax_OneTimeInit()
{
	_DMaxData(ambTuningWindowFactor_K) = DEF_AMBIENT_TUNING;
}


uint32_t VL6180::_DMax_RawValueAtRateKCps(int32_t rate)
{
	uint32_t snrLimit_K;
	int32_t DMaxSq;
	uint32_t RawDMax;
	DMaxFix_t retSignalAt400mm;
	uint32_t ambTuningWindowFactor_K;


	ambTuningWindowFactor_K = _DMaxData(ambTuningWindowFactor_K);
	snrLimit_K              = _DMaxData(snrLimit_K);
	retSignalAt400mm        = _DMaxData(retSignalAt400mm);
	/* 12 to 18 bits Kcps */
	if (rate > 0) {
		DMaxSq = 400 * 400 * 1000 / rate - (400 * 400 / 330);
		/* K of (1/RtnAmb -1/330 )=> 30bit- (12-18)bit  => 12-18 bits*/
		if (DMaxSq <= 0) {
		    RawDMax = 0;
		} else {
		    /* value can be more 32 bit so base on raneg apply
			 * retSignalAt400mm before or after division to presevr accuracy */
		    if (DMaxSq < (2 << 12)) {
				DMaxSq = DMaxSq * retSignalAt400mm /
							(snrLimit_K + ambTuningWindowFactor_K);
				/* max 12 + 12 to 18 -10 => 12-26 bit */
		    } else {
				DMaxSq = DMaxSq / (snrLimit_K + ambTuningWindowFactor_K) * retSignalAt400mm;
				/* 12 to 18 -10 + 12 to 18 *=> 12-26 bit */
		    }
		    RawDMax = VL6180_SqrtUint32(DMaxSq);
		}
	} else {
		RawDMax = 0x7FFFFFFF; /* bigest possibmle 32bit signed value */
	}
	return RawDMax;
}

/*
 * fetch static data from register to avoid re-read
 * precompute all intermediate constant and cliipings
 *
 * to be re-used/call on  changes of :
 *  0x2A
 *  SYSRANGE_MAX_AMBIENT_LEVEL_MULT
 *  Dev Data XtalkComRate_KCPs
 *  SYSRANGE_MAX_CONVERGENCE_TIME
 *  SYSRANGE_RANGE_CHECK_ENABLES    mask RANGE_CHECK_RANGE_ENABLE_MASK
 *  range 0xb8-0xbb (0xbb)
 */
int VL6180::_DMax_InitData()
{
	int status, warning;
	uint8_t u8;
	uint16_t u16;
	uint32_t u32;
	uint32_t Reg2A_KCps;
	uint32_t RegB8;
	uint8_t  MaxConvTime;
	uint32_t XTalkCompRate_KCps;
	uint32_t RangeIgnoreThreshold;
	int32_t minSignalNeeded;
	uint8_t SysRangeCheckEn;
	uint8_t snrLimit;
	static const int  MaxConvTimeAdjust = -4;

	warning = 0;

	do {
		status = VL6180_RdByte(0x02A, &u8);
		if (status) {
		    break;
		}

		if (u8 == 0) {
		    warning = CALIBRATION_WARNING;
		    u8 = 40; /* use a default average value */
		}
		Reg2A_KCps = Fix7_2_KCPs(u8); /* convert to KCPs */

		status = VL6180_RdByte(SYSRANGE_RANGE_CHECK_ENABLES, &SysRangeCheckEn);
		if (status) {
		    break;
		}

		status = VL6180_RdByte(SYSRANGE_MAX_CONVERGENCE_TIME, &MaxConvTime);
		if (status) {
			break;
		}

		status = VL6180_RdDWord(0x0B8, &RegB8);
		if (status) {
		    break;
		}

		status = VL6180_RdByte(SYSRANGE_MAX_AMBIENT_LEVEL_MULT, &snrLimit);
		if (status) {
		    break;
		}
		_DMaxData(snrLimit_K) = (int32_t)16 * 1000 / snrLimit;
		XTalkCompRate_KCps =   VL6180DevDataGet(Device, XTalkCompRate_KCps);

		if (Reg2A_KCps >= XTalkCompRate_KCps) {
		    _DMaxData(retSignalAt400mm) = Reg2A_KCps;
		} else{
		    _DMaxData(retSignalAt400mm) = 0;
			/* Reg2A_K - XTalkCompRate_KCp <0 is invalid */
		}

		/* if xtalk range check is off omit it in snr clipping */
		if (SysRangeCheckEn&RANGE_CHECK_RANGE_ENABLE_MASK) {
		    status = VL6180_RdWord(SYSRANGE_RANGE_IGNORE_THRESHOLD, &u16);
		    if (status) {
				break;
		    }
		    RangeIgnoreThreshold = Fix7_2_KCPs(u16);
		} else{
		    RangeIgnoreThreshold  = 0;
		}

		minSignalNeeded = (RegB8 * 256) / ((int32_t)MaxConvTime + (int32_t)MaxConvTimeAdjust);
		/* KCps 8+8 bit -(1 to 6 bit) => 15-10 bit */
		/* minSignalNeeded = max ( minSignalNeeded,  RangeIgnoreThreshold - XTalkCompRate_KCps) */
		if (minSignalNeeded  <= (int32_t)RangeIgnoreThreshold - (int32_t)XTalkCompRate_KCps)
		    minSignalNeeded  =  RangeIgnoreThreshold - XTalkCompRate_KCps;

		u32 = (minSignalNeeded*(uint32_t)snrLimit) / 16;
		_DMaxData(ClipSnrLimit) = _DMax_RawValueAtRateKCps(u32);
		/* clip to dmax to min signal snr limit rate*/
	} while (0);
	if (!status)
		status = warning;
	return status;
}

int VL6180::_DMax_Compute(VL6180_RangeData_t *pRange)
{
	uint32_t rtnAmbRate;
	int32_t DMax;
	int scaling;
	uint16_t HwLimitAtScale;
	static const uint32_t  rtnAmbLowLimit_KCps = 330 * 1000;

	rtnAmbRate = pRange->rtnAmbRate;
	if (rtnAmbRate  < rtnAmbLowLimit_KCps) {
		DMax = _DMax_RawValueAtRateKCps(rtnAmbRate);
		scaling = _GetUpscale();
		HwLimitAtScale = UpperLimitLookUP[scaling - 1];

		if (DMax > _DMaxData(ClipSnrLimit)) {
		    DMax = _DMaxData(ClipSnrLimit);
		}
		if (DMax > HwLimitAtScale) {
		    DMax = HwLimitAtScale;
		}
		pRange->DMax = DMax;
	} else {
		pRange->DMax = 0;
	}
	return 0;
}

#undef _DMaxData
#undef Fix7_2_KCPs

#endif /* VL6180_HAVE_DMAX_RANGING */

/* Write and read functions from I2C */


int VL6180::VL6180_WrByte(uint16_t index, uint8_t data)
{
   int  status;

   status=VL6180_I2CWrite(Device->I2cAddr, index, &data, 1);
   return status;
}

int VL6180::VL6180_UpdateByte(uint16_t index, uint8_t AndData, uint8_t OrData)
{
   int  status;
   uint8_t buffer = 0;

   /* read data direct onto buffer */
   status = VL6180_I2CRead(Device->I2cAddr, index, &buffer,1);
   if (!status)
   {
      buffer = (buffer & AndData) | OrData;
      status = VL6180_I2CWrite(Device->I2cAddr, index, &buffer, (uint16_t)1);
   }
   return status;
}

int VL6180::VL6180_WrWord(uint16_t index, uint16_t data)
{
   int  status;
   uint8_t buffer[2];

   buffer[0] = data >> 8;
   buffer[1] = data & 0x00FF;
   status=VL6180_I2CWrite(Device->I2cAddr, index, (uint8_t *)buffer, 2);
   return status;
}

int VL6180::VL6180_WrDWord(uint16_t index, uint32_t data)
{
   int  status;
   uint8_t buffer[4];

   buffer[0] = (data >> 24) & 0xFF;
   buffer[1] = (data >> 16) & 0xFF;
   buffer[2] = (data >>  8) & 0xFF;
   buffer[3] = (data >>  0) & 0xFF;
   status=VL6180_I2CWrite(Device->I2cAddr, index, (uint8_t *)buffer, 4);
   return status;
}


int VL6180::VL6180_RdByte(uint16_t index, uint8_t *data)
{
   int  status;

   status = VL6180_I2CRead(Device->I2cAddr, index, data, 1);

   if(status)
      return -1;

   return 0;
}

int VL6180::VL6180_RdWord(uint16_t index, uint16_t *data)
{
   int  status;
   uint8_t buffer[2] = {0,0};

   status = VL6180_I2CRead(Device->I2cAddr, index, buffer, 2);
   if (!status)
   {
      *data = (buffer[0] << 8) + buffer[1];
   }
   return status;

}

int VL6180::VL6180_RdDWord(uint16_t index, uint32_t *data)
{
   int status;
   uint8_t buffer[4] = {0,0,0,0};

   status = VL6180_I2CRead(Device->I2cAddr, index, buffer, 4);
   if(!status)
   {
      *data = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];
   }
   return status;

}

int VL6180::VL6180_I2CWrite(uint8_t DeviceAddr, uint16_t RegisterAddr, uint8_t* pBuffer, uint16_t NumByteToWrite)
{
   dev_i2c->beginTransmission(((uint8_t)(((DeviceAddr) >> 1) & 0x7F)));
   uint8_t buffer[2];
   buffer[0]=(uint8_t) (RegisterAddr>>8);
   buffer[1]=(uint8_t) (RegisterAddr&0xFF);
   dev_i2c->write(buffer, 2);
   for (uint16_t i = 0 ; i < NumByteToWrite ; i++)
      dev_i2c->write(pBuffer[i]);

   dev_i2c->endTransmission(true);
   return 0;
}

int VL6180::VL6180_I2CRead(uint8_t DeviceAddr, uint16_t RegisterAddr, uint8_t* pBuffer, uint16_t NumByteToRead)
{
   int status = 0;
   //Loop until the port is transmitted correctly
   do
   {
      dev_i2c->beginTransmission(((uint8_t)(((DeviceAddr) >> 1) & 0x7F)));
      uint8_t buffer[2];
      buffer[0]=(uint8_t) (RegisterAddr>>8);
      buffer[1]=(uint8_t) (RegisterAddr&0xFF);
      dev_i2c->write(buffer, 2);
      status = dev_i2c->endTransmission(false);
//Fix for some STM32 boards
//Reinitialize th i2c bus with the default parameters
#ifdef ARDUINO_ARCH_STM32
      if (status)
      {
         dev_i2c->end();
         dev_i2c->begin();
      }
#endif
//End of fix
   }
   while(status != 0);

   dev_i2c->requestFrom(((uint8_t)(((DeviceAddr) >> 1) & 0x7F)), (byte) NumByteToRead);

   int i=0;
   while (dev_i2c->available())
   {
      pBuffer[i] = dev_i2c->read();
      i++;
   }

   return 0;
}

int VL6180::VL6180_ReadMulti(uint16_t index, uint8_t *pdata, uint32_t count)
{
   int status;

   status = VL6180_I2CRead(Device->I2cAddr, index, pdata, (uint16_t)count);

   return status;
}

#ifdef __cplusplus
extern "C" {
#endif

/* Low Level Functions: This functions are empty and must be implemented in the platform dependent low level driver */
__attribute__((weak)) void VL6180_PollDelay(void)
{
  delay(1);
}

#ifdef __cplusplus
}
#endif



