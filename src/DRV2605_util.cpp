#include <DRV2605_util.h>

static void stimulation_timer_callback(void *arg);

DRV2605_UTIL &DRV2605_UTIL::getInstance()
{
    static DRV2605_UTIL instance;
    return instance;
}
DRV2605_UTIL::DRV2605_UTIL()
{
}
DRV2605_UTIL::DRV2605_UTIL(DRV2605_Autocal_t auto_config)
{
    m_auto_config = auto_config;
}

void DRV2605_UTIL::set_ratedVoltage(uint8_t ratedVoltage)
{
    Adafruit_DRV2605::writeRegister8(DRV2605_REG_RATEDV, ratedVoltage);
}
void DRV2605_UTIL::set_odClamp(uint8_t odClamp)
{
    Adafruit_DRV2605::writeRegister8(DRV2605_REG_CLAMPV, odClamp);
}
void DRV2605_UTIL::autoCalibrate()
{
    ESP_LOGE("DRV_UTIL", "START autocalibration");
    // DRV2605_REG_FEEDBACK - 0x1A
    // see DRV setup guide 2.2. LRA Auto calibration and C10-100 LRA by Precision Microdrives

    // TODO POWER PIN

    // Write a value of 0x07 to register 0x01. This value moves the DRV2605L device out of STANDBY and places
    // the MODE [2:0] bits in auto - calibration mode.
    Adafruit_DRV2605::setMode(DRV2605_MODE_AUTOCAL);

    // Populate the input parameters required by the auto-calibration engine
    Adafruit_DRV2605::writeRegister8(DRV2605_REG_FEEDBACK,
                                     (DRV_LRA << 7) | (DRV_FB_BRAKE_FACTOR_4X << 4) | (DRV_LOOP_GAIN_MEDIUM << 2) | DRV_BEMF_GAIN_1_365X_15X);
    set_ratedVoltage(m_auto_config.RATED_VOLTAGE);

    set_odClamp(m_auto_config.OD_CLAMP);

    // Drive time(ms) = DRIVE_TIME [4:0] × 0.1 ms + 0.5 ms
    Adafruit_DRV2605::writeRegister8(DRV2605_REG_CONTROL1,
                                     (DRV_STARTUP_BOOST_ON << 7) | (DRV_AC_COUPLE_DC << 5) | (m_auto_config.DRIVE_TIME));
    Adafruit_DRV2605::go();
    // Wait for the auto-calibration to complete
    delay(1200);

    // Read the status register to check if the auto-calibration is complete
    /*
    ESP_LOGD("DRV_UTIL", "Adafruit_DRV2605::readRegister8(DRV2605_REG_STATUS)");
    byte status = Adafruit_DRV2605::readRegister8(DRV2605_REG_STATUS);

    ESP_LOGD("DRV_UTIL", "Status = %d", status);
    while (status & 0x08)
    {

        status = Adafruit_DRV2605::readRegister8(DRV2605_REG_STATUS);
    }
    */

    ESP_LOGE("DRV_UTIL", "END autocalibration");
}

void DRV2605_UTIL::registerDefault()
{

    Adafruit_DRV2605::writeRegister8(DRV2605_REG_MODE,
                                     (DRV_DEV_RESET_OFF << 7) | (DRV_STANDBY_READY << 6) | (DRV2605_MODE_AUTOCAL));
    // Populate the input parameters required by the auto-calibration engine
    Adafruit_DRV2605::writeRegister8(DRV2605_REG_FEEDBACK,
                                     (DRV_LRA << 7) | (DRV_FB_BRAKE_FACTOR_4X << 4) | (DRV_LOOP_GAIN_MEDIUM << 2) | DRV_BEMF_GAIN_1_365X_15X);

    Adafruit_DRV2605::writeRegister8(DRV2605_REG_RATEDV, DRV_RV_LRA_CL_300US_2_0V_RMS);

    Adafruit_DRV2605::writeRegister8(DRV2605_REG_CLAMPV, DRV_LRA_OD_CLAMP_2_05V);
    // Set the closed-loop input to unidirectional
    Adafruit_DRV2605::writeRegister8(DRV2605_REG_CONTROL1,
                                     (DRV_STARTUP_BOOST_ON << 7) | (DRV_AC_COUPLE_DC << 5) | DRV_DRIVE_TIME_19);
    Adafruit_DRV2605::writeRegister8(DRV2605_REG_CONTROL2,
                                     (DRV_BIDIR_INPUT_UNI << 7) | (DRV_BRAKE_STABILIZER_ON << 6) | (DRV_SAMPLE_TIME_300US << 4) | (DRV_BLANKING_TIME_25_75US << 2) | (DRV_IDISS_TIME_25_75US));
    // set the RTP input to unsigned
    Adafruit_DRV2605::writeRegister8(DRV2605_REG_CONTROL3,
                                     (DRV_NG_THRESH_4PER << 6) | (DRV_ERM_OPEN_LOOP << 5) | (DRV_SUPPLY_COMP_DIS_ON << 4) | (DRV_DATA_FORMAT_RTP_UNSIGNED << 3) | (DRV_LRA_DRIVE_MODE_ONCE << 2) | (DRV_NPWM_ANALOG_PWM << 1) | (DRV_LRA_AUTO_RES_MODE));

    Adafruit_DRV2605::writeRegister8(DRV2605_REG_CONTROL4,
                                     (DRV_ZC_DET_TIME_100US << 6) | (DRV_AUTOCAL_TIME_1000MS << 4) | (DRV_OTP_PRG_OFF));
}

/*  Initialization Procedure  */
// 1. After powerup, wait at least 250 μs before the DRV2605L device accepts I2C commands.
// 2. Assert the EN pin (logic high). The EN pin can be asserted any time during or after the 250 μs wait period.
// #TODO
//   3. Perform the steps as described in the Auto Calibration Procedure section. Alternatively, rewrite the results
// from a previous calibration.
// TODO:'
/*
MAKE use drv handle for multi device;

Status				0x00		DeviceID [7:5] Read – 101
          Read(0xA8)	Diag_Result 	[3] Read – 1
                Feedback_Status [2] Read – 0 / 1
                OverTemp 		[1] Read – 0
                OC_Detect 		[0] Read – 0
 Auto-calibration 	0x18 		ACalComp [7:0] Read value and store
 Compensation Results

 Auto-calibration
 Back-EMFResult  		0x19 		ACalBEMF [7:0] Read value and store

 Feedback Control		0x1A 		BEMFGain [1:0] Read bits [1:0] and store
*/
void DRV2605_UTIL::init()
{
    m_auto_config.RATED_VOLTAGE = DRV_L_RV_LRA_CL_300US_2_0V_RMS; // Rated Voltage [7:0] 2 Vrms
    m_auto_config.OD_CLAMP = DRV_L_LRA_OD_CLAMP_2_05V;            // Overdrive Clamp Voltage: ODClamp [7:0] 2.05 Vpeak
    m_auto_config.DRIVE_TIME = 24;                                // Optimum drive time (ms) ≈ 0.5 × LRA Period
    DRV2605_UTIL::init(DRV2605_UTIL::m_auto_config);
};
void DRV2605_UTIL::init(DRV2605_Autocal_t auto_config)
{

    pinMode(m_trig_Right, OUTPUT);
    // pinMode(_trig_Left, OUTPUT);
    pinMode(m_en_Right, OUTPUT);
    pinMode(m_en_Left, OUTPUT);

    digitalWrite(m_trig_Right, LOW);
    // digitalWrite(_trig_Left, LOW);
    digitalWrite(m_en_Right, LOW);
    digitalWrite(m_en_Left, LOW);

    disable();

    gaitGuide.onStimulation([](bool stimDirection)
                            { DRV2605_UTIL::getInstance().stimulate(stimDirection); });

    const esp_timer_create_args_t stimulation_timer_args = {
        .callback = &stimulation_timer_callback,
        /* name is optional, but may help identify the timer when debugging */
        .name = "stimulation_timer"};
    ESP_ERROR_CHECK(esp_timer_create(&stimulation_timer_args, &stimulation_timer));

    // Adafruit_DRV2605::begin();
    bool is_calibrated = false; // #TODO use flash memory and or "Calibration pin"

    if (is_calibrated)
    {
        // # TODO: Read Flash calibration results
    }
    else
    {

        DRV2605_UTIL::autoCalibrate();
    }

    // initialise the device
    // Write the MODE register (address 0x01) to value 0x00 to remove the device from standby mode.
    uint8_t drv_modeSelect = DRV2605_MODE_REALTIME; // DRV2605_MODE_REALTIME;
    Adafruit_DRV2605::writeRegister8(DRV2605_REG_MODE,
                                     (DRV_DEV_RESET_OFF << 7) | (DRV_STANDBY_READY << 6) | (drv_modeSelect));
    // DRV2605_UTIL::useLRA();
    Adafruit_DRV2605::writeRegister8(DRV2605_REG_FEEDBACK,
                                     (DRV_LRA << 7) | (DRV_FB_BRAKE_FACTOR_4X << 4) | (DRV_LOOP_GAIN_MEDIUM << 2) | DRV_BEMF_GAIN_1_365X_15X);

    // 6. If using the embedded ROM library, write the library selection register (address 0x03) to select a library.
    /*There are six ROM libraries in the DRV2605 and each contains 123 effects. Libraries 1–5 are for ERM
      motors and were designed to support various ERM motor types and characteristics. Library 6 is the LRA
      library and uses closed-loop feedback to auto-tune to any LRA actuator. The effects in each library were
      created to achieve the same feel, but the output will appear slightly different to account for differences in
      motor characteristics like startup time, acceleration, and brake time.*/
    Adafruit_DRV2605::selectLibrary(6);

    // TODO add weakup and sleep for device
}

void DRV2605_UTIL::setTrigger(uint8_t Right, uint8_t Left)
{
    m_trig_Right = Right;
    m_trig_Left = Left;

    pinMode(m_trig_Right, OUTPUT);
    pinMode(m_trig_Left, OUTPUT);

    digitalWrite(m_trig_Right, LOW);
    digitalWrite(m_trig_Left, LOW);
}

void DRV2605_UTIL::startRight()
{
    digitalWrite(m_trig_Right, HIGH);
}
void DRV2605_UTIL::stopRight()
{
    digitalWrite(m_trig_Right, LOW);
}
void DRV2605_UTIL::startLeft()
{
    digitalWrite(m_trig_Left, HIGH);
}
void DRV2605_UTIL::stopLeft()
{
    digitalWrite(m_trig_Left, LOW);
}

void DRV2605_UTIL::start()
{
    digitalWrite(m_trig_Right, HIGH);
    digitalWrite(m_trig_Left, HIGH);
}
void DRV2605_UTIL::stop()
{
    digitalWrite(m_trig_Right, LOW);
    digitalWrite(m_trig_Left, LOW);
}

void DRV2605_UTIL::setEnable(uint8_t Right, uint8_t Left)
{
    m_en_Right = Right;
    m_en_Left = Left;

    pinMode(m_en_Right, OUTPUT);
    pinMode(m_en_Left, OUTPUT);

    digitalWrite(m_en_Right, LOW);
    digitalWrite(m_en_Left, LOW);
}

void DRV2605_UTIL::enable()
{
    digitalWrite(m_en_Right, HIGH);
    digitalWrite(m_en_Left, HIGH);
}
void DRV2605_UTIL::disable()
{
    digitalWrite(m_en_Right, LOW);
    digitalWrite(m_en_Left, LOW);
}

void DRV2605_UTIL::enableRight()
{
    digitalWrite(m_en_Right, HIGH);
}
void DRV2605_UTIL::disableRight()
{
    digitalWrite(m_en_Right, LOW);
}
void DRV2605_UTIL::enableLeft()
{
    digitalWrite(m_en_Left, HIGH);
}
void DRV2605_UTIL::disableLeft()
{
    digitalWrite(m_en_Left, LOW);
}

bool DRV2605_UTIL::timescale()
{
    return m_is_timescale_1ms;
}

void DRV2605_UTIL::timescale(bool is_timescale_1ms)
{
    m_is_timescale_1ms = is_timescale_1ms;
    // this overwrites the register with default values, should be changed at some point if multiple DRV on the same i2C is sorted out
    // _is_timescale_5ms is inverted is
    Adafruit_DRV2605::writeRegister8(DRV2605_REG_CONTROL5, 0x80 | (m_is_timescale_1ms << 4));
}

void DRV2605_UTIL::stimulate(bool stimDirection)
{
    log_i("starting stimulation");
    switch (gaitGuide.stimMode())
    {
    case GAITGUIDE_USERMODE_AMPLITUDE:
    {
        if (stimDirection)
        {
            enableRight();
        }
        else
        {
            enableLeft();
        }
        setRealtimeValue(gaitGuide.amp());
        ESP_ERROR_CHECK(esp_timer_start_once(stimulation_timer, gaitGuide.duration() * 1000)); // time in us
        //  ESP_LOGD(TAG_DRV, "SetRealtimeValue = %d for %dms", gaitGuide.amp(), gaitGuide.duration());
        break;
    }
    case GAITGUIDE_USERMODE_EFFECT:
    {
        if (stimDirection)
        {
            enableRight();
            startRight();
        }
        else
        {

            enableLeft();
            startLeft();
        }
        delay(gaitGuide.duration());
        stop();
        disable();
        //   ESP_LOGD(TAG_DRV, "Effect #%d: %s for %dms", gaitGuide.effect(0), drv_effect_string_map[gaitGuide.effect(0)], gaitGuide.duration());
        break;
    }
    case GAITGUIDE_USERMODE_PRESSURE:
    {
        // #TODO remove and clean-up usermodes
        break;
    }
    case GAITGUIDE_USERMODE_DEMO00:
    {
        //** ROB MODE **/

        enable();
        setMode(DRV2605_MODE_EXTTRIGLVL);

        uint8_t effect = DRV_EFF_STRONG_BUZZ_100;
        for (uint8_t i = 0; i < 8; i++)
        {
            setWaveform(i, effect);
        }
        disable();
        while (gaitGuide.stimMode() == GAITGUIDE_USERMODE_DEMO00)
        {
            enableRight();
            // #TODO: move to drv
            log_d("USERDEMO\n---------\n");

            startRight();
            // digitalWrite(TRIGGER_Left, HIGH);
            delay(150);
            // stop
            // writeRegister8(DRV2605_REG_GO, 0);
            stopRight();
            disableRight();
            // digitalWrite(TRIGGER_Left, LOW);
            delay(10000);
        }

        ESP_LOGD(TAG_DRV, "SetRealtimeValue = %d for %dms", gaitGuide.amp(), gaitGuide.duration());
        break;
    }
    default:
        ESP_LOGD(TAG_DRV, "Undefined Usermode");
        break;
    }
}

static void stimulation_timer_callback(void *arg)
{
    DRV2605_UTIL &drv = DRV2605_UTIL::getInstance();
    drv.setRealtimeValue(0x00);
    drv.disable();

    GaitGuide &gaitGuide = GaitGuide::getInstance();
    gaitGuide.goLeft = false;
    gaitGuide.goRight = false;
    log_d("\n--------------STOPPED-------------------\n\n");
}
