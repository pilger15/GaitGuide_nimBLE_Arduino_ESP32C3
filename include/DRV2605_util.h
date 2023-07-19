/*#
The defines and lists have been mostly generated using ChatGPT
*/

#ifndef DRV2605_UTIL_H
#define DRV2605_UTIL_H
#include <Adafruit_DRV2605.h>
#include <DRV2605_EFFECTS.h>
#include <GaitGuide.h>
#include "esp_timer.h"

/*
	"#1: Strong Click - 100%",
	"#2: Strong Click - 60%",
	"#3: Strong Click - 30%",
	"#4: Sharp Click - 100%",
	"#5: Sharp Click - 60%",
	"#6: Sharp Click - 30%",
	"#7: Soft Bump - 100%",
	"#8: Soft Bump - 60%",
	"#9: Soft Bump - 30%",
	"#10: Double Click - 100%",
	"#11: Double Click - 60%",
	"#12: Triple Click - 100%",
	"#13: Soft Fuzz - 60%",
	"#14: Strong Buzz - 100%",
	"#15: 750ms Alert 100%",
	"#16: 1000ms Alert 100%",
	"#17: Strong Click 1 - 100%",
	"#18: Strong Click 2 - 80%",
	"#19: Strong Click 3 - 60%",
	"#20: Strong Click 4 - 30%"



};


*/

typedef struct
{
	uint8_t ERM_LRA : 1;		  // selection will depend on desired actuator
	uint8_t FB_BRAKE_FACTOR : 3;  // A value of 2 is valid for most actuators
	uint8_t LOOP_GAIN : 2;		  // A value of 2 is valid for most actuators
	uint8_t RATED_VOLTAGE : 8;	  // See the Rated Voltage Programming section for calculating the correct register value.
	uint8_t OD_CLAMP : 8;		  // See the Overdrive Voltage-Clamp Programming section for calculating the correct register value.
	uint8_t DRIVE_TIME : 5;		  // See the Drive-Time Programming for calculating the correct register value
	uint8_t STARTUP_BOOST : 1;	  // Control 1: StartupBoost[7] 1 – ON (default)
	uint8_t AC_COUPLE : 1;		  // Control 1: AC_Couple [5] 0 – DC Coupling / Digital Input Modes
	uint8_t BI_DIR_INPUT : 1;	  // Control 2: BiDir_Input [7] 1 – Bi-directional (default)
	uint8_t BRAKE_STABILIZER : 1; // Control 2: BrakeStabilizer [6] 0 – OFF (default)
	uint8_t SAMPLE_TIME : 2;	  // Control 2: SampleTime [5:4] 3 – 300 μs (default)
	uint8_t BLANKING_TIME : 2;	  // Control 2: BlankingTime [3:2] 1 – 25 μs, 75 μs (default)
	uint8_t IDISS_TIME : 2;		  // Control 2: IDissTime [1:0] 1 – 25 μs, 75 μs (default)
	uint8_t NG_THRESH : 2;		  // Control 3: NG_Thresh [7:6] 2 – 4% (default)
	uint8_t ERM_OPEN_LOOP : 1;	  // Control 3: ERM_OpenLoop [5] 0 – Closed Loop (default)
	uint8_t SUPPLY_COMP_DIS : 1;  // Control 3: SupplyCompDis [4] 0 – ON (default)
	uint8_t DATA_FORMAT_RTP : 1;  // Control 3: DataFormat_RTP [3] 0 – Signed (default)
	uint8_t LRA_DRIVE_MODE : 1;	  // Control 3: LRADriveMode [2] 0 – Once per cycle (default)
	uint8_t NPWM_ANALOG : 1;	  // Control 3: nPWM_Analog [1] 0– PWM Input (default)
	uint8_t LRA_OPEN_LOOP : 1;	  // Control 3: LRA_OpenLoop [0] 0 – Auto Resonance On (default)
	uint8_t DEV_RESET : 1;		  // Mode: Dev_Reset [7] 0 – OFF (default)
	uint8_t STANDBY : 1;		  // Mode: STANDBY [6] 0 – Device Ready
	uint8_t MODE : 3;			  // Mode: Mode [2:0] 7 – Auto Calibration
} DRV2605_Autocal_t;

/*
  Code generated by ChatGPT on [2022-01-17].
  Generated in response to the prompt: "make defines from the following list similar to the provided example:
0: 1x will be #define FB_BRAKE_FACTOR_1x 0
and then create structs and populate them with the defines provided and also provide similar list for the loop_gain "
*/
// Warning the following values are  only valid for DRV2605L
// Defines for Rated Voltage @tSample = 300us
// DRV 2604 and 2605
#define DRV_RV_LRA_CL_300US_2_0V_RMS 0x53
#define DRV_RV_LRA_CL_300US_2_1V_RMS 0x57
#define DRV_RV_LRA_CL_300US_2_2V_RMS 0x5B
#define DRV_RV_LRA_CL_300US_2_3V_RMS 0x5F
#define DRV_RV_LRA_CL_300US_2_4V_RMS 0x63
// DRV 2605L
#define DRV_L_RV_LRA_CL_300US_2_0V_RMS 0x53
#define DRV_L_RV_LRA_CL_300US_2_2V_RMS 0x5B
#define DRV_L_RV_LRA_CL_300US_2_4V_RMS 0x64

// Defines for Overdrive Clamp Voltage
#define DRV_LRA_OD_CLAMP_2_05V 0x5D
#define DRV_LRA_OD_CLAMP_2_82V 0x80
#define DRV_LRA_OD_CLAMP_3_02V 0x8A

// DRV 2605L
#define DRV_L_LRA_OD_CLAMP_2_05V 0x60
#define DRV_L_LRA_OD_CLAMP_2_82V 0x84
#define DRV_L_LRA_OD_CLAMP_3_02V 0x8E
// Defines for Feedback Control
#define DRV_ERM 0
#define DRV_LRA 1

#define DRV_FB_BRAKE_FACTOR_1X 0
#define DRV_FB_BRAKE_FACTOR_2X 1
#define DRV_FB_BRAKE_FACTOR_3X 2
#define DRV_FB_BRAKE_FACTOR_4X 3
#define DRV_FB_BRAKE_FACTOR_6X 4
#define DRV_FB_BRAKE_FACTOR_8X 5
#define DRV_FB_BRAKE_FACTOR_16X 6
#define DRV_FB_BRAKE_DISABLED 7
#define DRV_LOOP_GAIN_LOW 0
#define DRV_LOOP_GAIN_MEDIUM 1
#define DRV_LOOP_GAIN_HIGH 2
#define DRV_LOOP_GAIN_VERY_HIGH 3
#define DRV_BEMF_GAIN_0_255X_3_75X 0
#define DRV_BEMF_GAIN_0_7875X_7_5X 1
#define DRV_BEMF_GAIN_1_365X_15X 2 // default
#define DRV_BEMF_GAIN_3X_22_5X 3

// Defines for Control 1
#define DRV_STARTUP_BOOST_ON 1
#define DRV_STARTUP_BOOST_OFF 0

#define DRV_AC_COUPLE_DC 0
#define DRV_AC_COUPLE_AC 1

#define DRV_DRIVE_TIME_19 19
#define DRV_DRIVE_TIME_34 34

// Defines for Control 2
#define DRV_BIDIR_INPUT_BI 1 // default
#define DRV_BIDIR_INPUT_UNI 0

#define DRV_BRAKE_STABILIZER_ON 1 // default
#define DRV_BRAKE_STABILIZER_OFF 0
#define DRV_SAMPLE_TIME_300US 3 // default
#define DRV_SAMPLE_TIME_250US 2
#define DRV_SAMPLE_TIME_200US 1
#define DRV_SAMPLE_TIME_150US 0
#define DRV_BLANKING_TIME_25_75US 1 // default
#define DRV_BLANKING_TIME_15_45US 0
#define DRV_IDISS_TIME_25_75US 1 // default
#define DRV_IDISS_TIME_15_45US 0

// Defines for Control 3
#define DRV_NG_THRESH_DIS 0
#define DRV_NG_THRESH_2PER 1
#define DRV_NG_THRESH_4PER 2
#define DRV_NG_THRESH_8PER 3
#define DRV_ERM_CLOSED_LOOP 0
#define DRV_ERM_OPEN_LOOP 1		 // default
#define DRV_SUPPLY_COMP_DIS_ON 0 // default
#define DRV_SUPPLY_COMP_DIS_OFF 1
#define DRV_DATA_FORMAT_RTP_SIGNED 0 // default
#define DRV_DATA_FORMAT_RTP_UNSIGNED 1
#define DRV_LRA_DRIVE_MODE_ONCE 0 // default
#define DRV_LRA_DRIVE_MODE_TWICE 1
#define DRV_NPWM_ANALOG_PWM 0 // default
#define DRV_NPWM_ANALOG_ANALOG 1
#define DRV_LRA_AUTO_RES_MODE 0 // default
#define DRV_LRA_OPEN_LOOP 1		// default

// Defines for Control 4
#define DRV_ZC_DET_TIME_100US 0 // default
#define DRV_ZC_DET_TIME_200US 1
#define DRV_ZC_DET_TIME_300US 2
#define DRV_ZC_DET_TIME_490US 3
#define DRV_AUTOCAL_TIME_125MS 0
#define DRV_AUTOCAL_TIME_250MS 1
#define DRV_AUTOCAL_TIME_500MS 2 // default
#define DRV_AUTOCAL_TIME_1000MS 3
#define DRV_OTP_PRG_OFF 0 // default
#define DRV_OTP_PRG_ON 1

// Defines for Mode
#define DRV_DEV_RESET_OFF 0
#define DRV_DEV_RESET_ON 1
#define DRV_STANDBY_READY 0
#define DRV_STANDBY_STANDBY 1 // default

#define DRV2605_REG_CONTROL5 0x1F

static const char *TAG_DRV = "DRV2605L";
/**************************************************************************/
/*!
  @brief The DRV2605 driver utility class, extending the Ardafruit library
	the DRV2605_UTIL class controls all drivers
*/
/**************************************************************************/
class DRV2605_UTIL : public Adafruit_DRV2605
{
public:
	static DRV2605_UTIL &getInstance();
	GaitGuide &gaitGuide = GaitGuide::getInstance();

	void initialise();
	void enable(bool Right, bool Left);

	void set_ratedVoltage(uint8_t ratedVoltage);
	void set_odClamp(uint8_t odClamp);
	void autoCalibrate();

	void registerDefault();
	void init();
	void init(DRV2605_Autocal_t auto_config);

	void setTrigger(uint8_t Right, uint8_t Left);

	void startRight();
	void stopRight();

	void startLeft();
	void stopLeft();

	void start();
	void stop();

	void setEnable(uint8_t Right, uint8_t Left);
	void enable();
	void disable();
	void enableRight();
	void disableRight();

	void enableLeft();
	void disableLeft();

	bool timescale();
	void timescale(bool is_timescale_1ms);

	void stimulate(bool stimDirection);

private:
	DRV2605_UTIL(void);
	DRV2605_UTIL(DRV2605_Autocal_t auto_config);
	DRV2605_UTIL &operator=(const DRV2605_UTIL &) = delete;
	// Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
	DRV2605_Autocal_t m_auto_config;

	esp_timer_handle_t stimulation_timer;

	uint8_t m_trig_Right = D3;
	uint8_t m_trig_Left = D4;

	uint8_t m_en_Right = D2;
	uint8_t m_en_Left = D1;

	bool m_is_timescale_1ms = false;
};

#endif // DRV2605_UTIL_H