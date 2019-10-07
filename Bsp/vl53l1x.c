#include "vl53l1x_reg.h"
#include "vl53l1x.h"
#include "bsp_i2c.h"
#include "bsp_timer.h"

struct vl53l1x_report_s
{
    uint16_t dist;
    uint8_t status;
    uint8_t mode;
};

struct vl53l1x_range_s
{
    uint16_t range_mm;
    uint8_t range_status; //RangeStatus range_status
};

struct vl53l1x_ResultBuffer_s
{
    uint8_t range_status;
    uint8_t stream_count;
    uint16_t dss_actual_effective_spads_sd0;
    uint16_t ambient_count_rate_mcps_sd0;
    uint16_t final_crosstalk_corrected_range_mm_sd0;
    uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
};

struct vl53l1x_range_s ranging_data;
struct vl53l1x_ResultBuffer_s results;

//--------------------------------------------------------------------------------

#define VL53L1_RANGESTATUS_RANGE_VALID 0
/*!<The Range is valid. */
#define VL53L1_RANGESTATUS_SIGMA_FAIL 1
/*!<Sigma Fail. */
#define VL53L1_RANGESTATUS_SIGNAL_FAIL 2
/*!<Signal fail. */
#define VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED 3
/*!<Target is below minimum detection threshold. */
#define VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL 4
/*!<Phase out of valid limits -  different to a wrap exit. */
#define VL53L1_RANGESTATUS_HARDWARE_FAIL 5
/*!<Hardware fail. */
#define VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL 6
/*!<The Range is valid but the wraparound check has not been done. */
#define VL53L1_RANGESTATUS_WRAP_TARGET_FAIL 7
/*!<Wrapped target - no matching phase in other VCSEL period timing. */
#define VL53L1_RANGESTATUS_PROCESSING_FAIL 8
/*!<Internal algo underflow or overflow in lite ranging. */
#define VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL 9
/*!<Specific to lite ranging. */
#define VL53L1_RANGESTATUS_SYNCRONISATION_INT 10
/*!<1st interrupt when starting ranging in back to back mode. Ignore data. */
#define VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE 11
/*!<All Range ok but object is result of multiple pulses merging together.
* Used by RQL for merged pulse detection
*/
#define VL53L1_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL 12
/*!<Used  by RQL  as different to phase fail. */
#define VL53L1_RANGESTATUS_MIN_RANGE_FAIL 13
/*!<User ROI input is not valid e.g. beyond SPAD Array.*/
#define VL53L1_RANGESTATUS_RANGE_INVALID 14
/*!<lld returned valid range but negative value ! */
#define VL53L1_RANGESTATUS_NONE 255

#define MODE_SHORT 0
#define MODE_MEDIUM 1
#define MODE_LONG 2

#define VL53L1X_MODE MODE_MEDIUM

uint16_t fast_osc_frequency;
uint16_t osc_calibrate_val;

uint8_t distance_mode;
uint8_t calibrated = 1;

uint8_t saved_vhv_init;
uint8_t saved_vhv_timeout;

#define writeReg write8
#define writeReg16Bit write16
#define writeReg32Bit write32
#define readReg read8
#define readReg16Bit read16

uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
    return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
    //return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
    return (timeout_mclks >> 12) * macro_period_us + (0x800 >> 12);
}

uint32_t decodeTimeout(uint16_t reg_val)
{
    return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

uint16_t encodeTimeout(uint32_t timeout_mclks)
{
    // encoded format: "(LSByte * 2^MSByte) + 1"
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0)
    {
        ls_byte = timeout_mclks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0)
        {
            ls_byte >>= 1;
            ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    else
    {
        return 0;
    }
}

uint32_t calcMacroPeriod(uint8_t vcsel_period)
{
    // from VL53L1_calc_pll_period_us()
    // fast osc frequency in 4.12 format; PLL period in 0.24 format
    uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;

    // from VL53L1_decode_vcsel_period()
    uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

    // VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
    uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
    macro_period_us >>= 6;
    macro_period_us *= vcsel_period_pclks;
    macro_period_us >>= 6;

    return macro_period_us;
}

void vl53l1x_startContinuous()
{
    // from VL53L1_set_inter_measurement_period_ms()
    uint32_t tmp = osc_calibrate_val * 50;
    writeReg32Bit(VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, tmp);

    writeReg(VL53L1_SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
    writeReg(VL53L1_SYSTEM__MODE_START, 0x40);      // mode_range__timed
}

uint32_t vl53l1x_getMeasurementTimingBudget(void)
{
    // assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
    // enabled: VHV, PHASECAL, DSS1, RANGE

    // VL53L1_get_timeouts_us() begin

    // "Update Macro Period for Range A VCSEL Period"
    uint32_t macro_period_us = calcMacroPeriod(readReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A));

    // "Get Range Timing A timeout"
    u16 tmp = read8(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI) << 8;
    tmp |= read8(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_LO);
    uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(decodeTimeout(tmp), macro_period_us);

    // VL53L1_get_timeouts_us() end

    return 2 * range_config_timeout_us + 4528;
}

u8 vl53l1x_setMeasurementTimingBudget(uint32_t budget_us)
{
    uint32_t range_config_timeout_us;
    uint32_t macro_period_us;
    uint32_t phasecal_timeout_mclks;

    // assumes PresetMode is LOWPOWER_AUTONOMOUS

    if (budget_us <= 4528)
    {
        return 0;
    }

    range_config_timeout_us = budget_us -= 4528;
    if (range_config_timeout_us > 1100000)
    {
        return 0;
    } // FDA_MAX_TIMING_BUDGET_US * 2

    range_config_timeout_us /= 2;

    // VL53L1_calc_timeout_register_values() begin

    // "Update Macro Period for Range A VCSEL Period"
    macro_period_us = calcMacroPeriod(readReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A));

    // "Update Phase timeout - uses Timing A"
    // Timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
    // via VL53L1_get_preset_mode_timing_cfg().
    phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
    if (phasecal_timeout_mclks > 0xFF)
    {
        phasecal_timeout_mclks = 0xFF;
    }
    writeReg(VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);

    // "Update MM Timing A timeout"
    // Timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
    // via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
    // actually ends up with a slightly different value because it gets assigned,
    // retrieved, recalculated with a different macro period, and reassigned,
    // but it probably doesn't matter because it seems like the MM ("mode
    // mitigation"?) sequence steps are disabled in low power auto mode anyway.
    writeReg16Bit(VL53L1_MM_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(timeoutMicrosecondsToMclks(1, macro_period_us)));

    // "Update Range Timing A timeout"
    writeReg16Bit(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

    // "Update Macro Period for Range B VCSEL Period"
    macro_period_us = calcMacroPeriod(readReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B));

    // "Update MM Timing B timeout"
    // (See earlier comment about MM Timing A timeout.)
    writeReg16Bit(VL53L1_MM_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(timeoutMicrosecondsToMclks(1, macro_period_us)));

    // "Update Range Timing B timeout"
    writeReg16Bit(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

    // VL53L1_calc_timeout_register_values() end

    return 1;
}

uint8_t vl53l1x_setDistanceMode(uint8_t mode)
{
    // save existing timing budget
    uint32_t budget_us = vl53l1x_getMeasurementTimingBudget();

    switch (mode)
    {
    case MODE_MEDIUM:
        // from VL53L1_preset_mode_standard_ranging()

        // timing config
        writeReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
        writeReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
        writeReg(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);

        // dynamic config
        writeReg(VL53L1_SD_CONFIG__WOI_SD0, 0x0B);
        writeReg(VL53L1_SD_CONFIG__WOI_SD1, 0x09);
        writeReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 10); // tuning parm default
        writeReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 10); // tuning parm default

        break;

    case MODE_LONG: // long
        // from VL53L1_preset_mode_standard_ranging_long_range()

        // timing config
        writeReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
        writeReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
        writeReg(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);

        // dynamic config
        writeReg(VL53L1_SD_CONFIG__WOI_SD0, 0x0F);
        writeReg(VL53L1_SD_CONFIG__WOI_SD1, 0x0D);
        writeReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 14); // tuning parm default
        writeReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 14); // tuning parm default

        break;

    default:
        // unrecognized mode - do nothing
        return 0;
    }

    // reapply timing budget
    vl53l1x_setMeasurementTimingBudget(budget_us);

    // save mode so it can be returned by getDistanceMode()
    distance_mode = mode;

    return 1;
}

void vl53l1x_init_reg()
{
    // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary

    writeReg(VL53L1_PAD_I2C_HV__EXTSUP_CONFIG, readReg(VL53L1_PAD_I2C_HV__EXTSUP_CONFIG) | 0x01);

    // store oscillator info for later use
    fast_osc_frequency = read16(VL53L1_OSC_MEASURED__FAST_OSC__FREQUENCY);
    osc_calibrate_val = (read8(VL53L1_RESULT__OSC_CALIBRATE_VAL_HI) << 8) | read8(VL53L1_RESULT__OSC_CALIBRATE_VAL_LO);

    //write16(VL53L1_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, 0x0A00); // should already be this value after reset
    write8(VL53L1_GPIO__TIO_HV_STATUS, 0x02);
    write8(VL53L1_SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8);    // tuning parm default
    write8(VL53L1_SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16); // tuning parm default
    write8(VL53L1_ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
    write8(VL53L1_ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
    write8(VL53L1_ALGO__RANGE_MIN_CLIP, 0);               // tuning parm default
    write8(VL53L1_ALGO__CONSISTENCY_CHECK__TOLERANCE, 2); // tuning parm default

    // general config
    write16(VL53L1_SYSTEM__THRESH_RATE_HIGH, 0x0000);
    write16(VL53L1_SYSTEM__THRESH_RATE_LOW, 0x0000);
    write8(VL53L1_DSS_CONFIG__APERTURE_ATTENUATION, 0x38);

    // timing config
    // most of these settings will be determined later by distance and timing
    // budget configuration
    write16(VL53L1_RANGE_CONFIG__SIGMA_THRESH, 360);                  // tuning parm default
    write16(VL53L1_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192); // tuning parm default

    // dynamic config

    write8(VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
    write8(VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
    write8(VL53L1_SD_CONFIG__QUANTIFIER, 2); // tuning parm default

    // VL53L1_preset_mode_standard_ranging() end

    // from VL53L1_preset_mode_timed_ranging_*
    // GPH is 0 after reset, but writing GPH0 and GPH1 above seem to set GPH to 1,
    // and things don't seem to work if we don't set GPH back to 0 (which the API
    // does here).
    write8(VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
    write8(VL53L1_SYSTEM__SEED_CONFIG, 1); // tuning parm default

    // from VL53L1_config_low_power_auto_mode
    write8(VL53L1_SYSTEM__SEQUENCE_CONFIG, 0x8B); // VHV, PHASECAL, DSS1, RANGE
    write16(VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 51200);
    write8(VL53L1_DSS_CONFIG__ROI_MODE_CONTROL, 2); // REQUESTED_EFFFECTIVE_SPADS

    // VL53L1_set_preset_mode() end

    // default to long range, 50 ms timing budget
    // note that this is different than what the API defaults to
    //vl53l1x_setDistanceMode(2);
    vl53l1x_setMeasurementTimingBudget(50000);

    // VL53L1_StaticInit() end

    // the API triggers this change in VL53L1_init_and_start_range() once a
    // measurement is started; assumes MM1 and MM2 are disabled
    u16 tmp = read8(VL53L1_MM_CONFIG__OUTER_OFFSET_MM_HI) << 8;
    tmp |= read8(VL53L1_MM_CONFIG__OUTER_OFFSET_MM_LO);
    write16(VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM, tmp << 2);
}

uint8_t vl53l1x_dataReady(void)
{
    return (readReg(VL53L1_GPIO__TIO_HV_STATUS) & 0x01) == 0;
}

void vl53l1x_readResults(void)
{
    uint8_t buf[17];

    readN(VL53L1_RESULT__RANGE_STATUS, buf, 17);

    results.range_status = buf[0];
    results.stream_count = buf[2];

    results.dss_actual_effective_spads_sd0 = (uint16_t)buf[3] << 8; // high byte
    results.dss_actual_effective_spads_sd0 |= buf[4];               // low byte

    results.ambient_count_rate_mcps_sd0 = (uint16_t)buf[7] << 8; // high byte
    results.ambient_count_rate_mcps_sd0 |= buf[8];               // low byte

    results.final_crosstalk_corrected_range_mm_sd0 = (uint16_t)buf[13] << 8; // high byte
    results.final_crosstalk_corrected_range_mm_sd0 |= buf[14];               // low byte

    results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 = (uint16_t)buf[15] << 8; // high byte
    results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 |= buf[16];               // low byte
}

void vl53l1x_updateDSS(void)
{
    uint16_t spadCount = results.dss_actual_effective_spads_sd0;

    if (spadCount != 0)
    {
        // "Calc total rate per spad"

        uint32_t totalRatePerSpad = (uint32_t)results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 + results.ambient_count_rate_mcps_sd0;

        // "clip to 16 bits"
        if (totalRatePerSpad > 0xFFFF)
        {
            totalRatePerSpad = 0xFFFF;
        }

        // "shift up to take advantage of 32 bits"
        totalRatePerSpad <<= 16;

        totalRatePerSpad /= spadCount;

        if (totalRatePerSpad != 0)
        {
            // "get the target rate and shift up by 16"
            uint32_t requiredSpads = ((uint32_t)0x0A00 << 16) / totalRatePerSpad;

            // "clip to 16 bit"
            if (requiredSpads > 0xFFFF)
            {
                requiredSpads = 0xFFFF;
            }

            // "override DSS config"
            writeReg16Bit(VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);
            // DSS_CONFIG__ROI_MODE_CONTROL should already be set to REQUESTED_EFFFECTIVE_SPADS

            return;
        }
    }

    // If we reached this point, it means something above would have resulted in a
    // divide by zero.
    // "We want to gracefully set a spad target, not just exit with an error"

    // "set target to mid point"
    writeReg16Bit(VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
}

void vl53l1x_getRangingData(void)
{
    // VL53L1_copy_sys_and_core_results_to_range_results() begin

    uint16_t range = results.final_crosstalk_corrected_range_mm_sd0;

    // "apply correction gain"
    // gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
    // Basically, this appears to scale the result by 2011/2048, or about 98%
    // (with the 1024 added for proper rounding).
    ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;

    // VL53L1_copy_sys_and_core_results_to_range_results() end

    // set range_status in ranging_data based on value of RESULT__RANGE_STATUS register
    // mostly based on ConvertStatusLite()
    switch (results.range_status)
    {
    case 17: // MULTCLIPFAIL
    case 2:  // VCSELWATCHDOGTESTFAILURE
    case 1:  // VCSELCONTINUITYTESTFAILURE
    case 3:  // NOVHVVALUEFOUND
        // from SetSimpleData()
        ranging_data.range_status = VL53L1_RANGESTATUS_HARDWARE_FAIL;
        break;

    case 13: // USERROICLIP
        // from SetSimpleData()
        ranging_data.range_status = VL53L1_RANGESTATUS_MIN_RANGE_FAIL;
        break;

    case 18: // GPHSTREAMCOUNT0READY
        ranging_data.range_status = VL53L1_RANGESTATUS_SYNCRONISATION_INT;
        break;

    case 5: // RANGEPHASECHECK
        ranging_data.range_status = VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL;
        break;

    case 4: // MSRCNOTARGET
        ranging_data.range_status = VL53L1_RANGESTATUS_SIGNAL_FAIL;
        break;

    case 6: // SIGMATHRESHOLDCHECK
        ranging_data.range_status = VL53L1_RANGESTATUS_SIGMA_FAIL;
        break;

    case 7: // PHASECONSISTENCY
        ranging_data.range_status = VL53L1_RANGESTATUS_WRAP_TARGET_FAIL;
        break;

    case 12: // RANGEIGNORETHRESHOLD
        ranging_data.range_status = VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL;
        break;

    case 8: // MINCLIP
        ranging_data.range_status = VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED;
        break;

    case 9: // RANGECOMPLETE
        // from VL53L1_copy_sys_and_core_results_to_range_results()
        if (results.stream_count == 0)
        {
            ranging_data.range_status = VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL;
        }
        else
        {
            ranging_data.range_status = VL53L1_RANGESTATUS_RANGE_VALID;
        }
        break;

    default:
        ranging_data.range_status = VL53L1_RANGESTATUS_NONE;
    }

    // from SetSimpleData()
    //ranging_data.peak_signal_count_rate_MCPS = countRateFixedToFloat(results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
    //ranging_data.ambient_count_rate_MCPS = countRateFixedToFloat(results.ambient_count_rate_mcps_sd0);
}

void vl53l1x_setupManualCalibration(void)
{
    // "save original vhv configs"
    saved_vhv_init = readReg(VL53L1_VHV_CONFIG__INIT);
    saved_vhv_timeout = readReg(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND);

    // "disable VHV init"
    writeReg(VL53L1_VHV_CONFIG__INIT, saved_vhv_init & 0x7F);

    // "set loop bound to tuning param"
    writeReg(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
             (saved_vhv_timeout & 0x03) + (3 << 2)); // tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)

    // "override phasecal"
    writeReg(VL53L1_PHASECAL_CONFIG__OVERRIDE, 0x01);
    writeReg(VL53L1_CAL_CONFIG__VCSEL_START, readReg(VL53L1_PHASECAL_RESULT__VCSEL_START));
}

u8 vl53l1x_measure(struct vl53l1x_report_s *pf)
{
    if (vl53l1x_dataReady())
    {
        vl53l1x_readResults();

        if (!calibrated)
        {
            vl53l1x_setupManualCalibration();
            calibrated = 1;
        }

        vl53l1x_updateDSS();
        vl53l1x_getRangingData();
        writeReg(VL53L1_SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range

        pf->dist = ranging_data.range_mm;
        pf->status = ranging_data.range_status;
        pf->mode = VL53L1X_MODE;

        return 0;
    }

    return -1;
}

//错误为1，成功为0
u8 VL53L1X_Init(void)
{

    if (read16(VL53L1_IDENTIFICATION__MODEL_ID) != 0xEACC)
        return 1;

    writeReg(VL53L1_SOFT_RESET, 0x00);
    //延时
    TIMDelay_Nms(100);
    writeReg(VL53L1_SOFT_RESET, 0x01);
    //延时
    TIMDelay_Nms(10);

    //检测系统是否就绪
    if ((read8(VL53L1_FIRMWARE__SYSTEM_STATUS) & 0x01) == 0)
        return 1;

    vl53l1x_init_reg();

    vl53l1x_setDistanceMode(1);
    vl53l1x_setMeasurementTimingBudget(50000);
    vl53l1x_startContinuous();

    return 0;
}

u16 VL53L1X_MEASURE(void)
{
    struct vl53l1x_report_s aa;
    vl53l1x_measure(&aa);
    return aa.dist;
}