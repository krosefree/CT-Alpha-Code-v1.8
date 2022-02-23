/* ========================================
 *
 * Copyright Proof Energy, 2021
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Proof Energy.
 *
 * ========================================
*/
#include <project.h>

/***************************************
*        Enumerations
***************************************/

#define MAIN_LOOP_TIME          50		// 50ms loop time (20 Hz), Limited by ADC speed	(varies based on ADC bit depth)
#define STARTUP_GRACE_MS        1000    // (not used) 1 second delay for startup
#define TEMPERATURE_FILTER      10       // 0.1 second cycle (when main loop is 20Hz)

// System States
#define STATE_UNKNOWN			   0
#define STATE_STARTUP			   1
#define STATE_LIGHTOFF			   2
#define STATE_STEADY_STATE		   3
#define STATE_MANUAL_MODE		   4
#define STATE_STANDARD_SHUTDOWN	5
#define STATE_EMERGENCY_SHUTDOWN 6

#define UNKNOWN 0
#define GOOD    1
#define FAULT   2
#define BAD     3

#define FAULT_COUNT     5

#define CHAR_ARRAY_SIZE 300

//#define SENSOR_FAULT_COUNT      25
//#define TEMPERATURE_FAULT_COUNT 25
//#define CURRENT_FAULT_COUNT     25
//#define POWER_FAULT_COUNT       25
//
//#define TEMP_UNKNOWN    0   // power up state
//#define TEMP_GOOD       1   // temp is within required range
//#define TEMP_BAD        2   // temp is outside of valid operating range (open or short-circuit)
//#define TEMP_FAULT      3   // temp is outside of desired operating range (too hot/too cold)
//
//#define CURRENT_UNKNOWN    0   // power up state
//#define CURRENT_GOOD       1   // current is within required range
//#define CURRENT_BAD        2   // current is outside of valid operating range (open or short-circuit)
//#define CURRENT_FAULT      3   // current is outside of desired operating range (too hot/too cold)

#define PUMP_UNKNOWN    0  // power up state
#define PUMP_ON         1  // pump is on/open
#define PUMP_OFF        2  // pump is off/closed
#define PUMP_LOCKED     3  // pump is locked (off)

#define FAN_UNKNOWN     0  // power up state
#define FAN_ON          1  // fan is on/open
#define FAN_OFF         2  // fan is off/closed
#define FAN_LOCKED      3  // fan is locked (off)

#define P_KFI           1  // K-Factor multiplier to amplify P_INC
#define P_KFD           1  // K-Factor multiplier to amplify P_DEC

#define S_KFI           1  // K-Factor multiplier to amplify S_INC
#define S_KFD           1.5  // K-Factor multiplier to amplify S_DEC

#define FAN_DUTY_CYCLE           1024	 // Start at 50%  Valid 8-bit PWM range is 0 to 1024 (0-100%)

#define FAN_SPEED_ADJUSTMENT     25     // TBD RPM change per main loop
#define FAN_SPEED_ADJ_LOW        5      // Reduce derivative rate when Slope Temp is reached
#define FAN_SPD_SLOPE_TEMP_INC   510    // Temp at which to reduce derivative slope
#define FAN_SPD_SLOPE_TEMP_DEC   800    // Temp at which to reduce derivative slope
#define FAN_SPEED_MAXIMUM_RPM    32000  // TBD
#define FAN_STARTUP_SPEED_PRI    10000  // Was 8250 - RK 19Oct
                                        // 29Dec21 - Was 14400
#define FAN_STARTUP_SPEED_SEC    8000   // 
#define FAN_SS_SPEED_PRI         4500   // Primary Fan Steady State Speed
#define FAN_SS_SPEED_SEC         5000   // Secondary Fan Steady State Speed
#define FAN_IDLE_SPEED_PRI       9000   // Initial Primary Floor Speed - Was 12000 - 29Dec21
#define FAN_IDLE_SPEED_SEC       8000   // Initial Secondary Floor Speed
#define FAN_SHUTDOWN_SPEED       8000   // Standard Shutdown Speed Setting
//#define FAN_MINIMUM_RPM         4000   // Lowest Controllable Fan Speed
#define FAN_SPEED_THRESHOLD      0.9    // within 10% of commanded speed is acceptable

#define FUEL_DUTY_OFF			   0     //TBD duty cycle when fuel is off 
#define FUEL_DUTY_STARTUP		   100	//TBD duty cycle startup percentage (0-1000)
#define FUEL_DUTY_FULL			   500	//TBD duty cycle full percentage (0-1000)
#define FUEL_DUTY_MANUAL		   500	//TBD duty cycle manual percentage (0-1000)
#define FUEL_DUTY_SHUTDOWN		   0	   //TBD duty cycle shutdown percentage (0-1000)
#define FUEL_DUTY_ADJUSTMENT     1     //TBD Duty cycle change per main loop
#define FUEL_DUTY_MIN_ADJUSTED   100   //TBD Duty cycle lowest percentage (10%)
#define FUEL_DUTY_MAX_ADJUSTED   1000  //TBD Duty cycle lowest percentage (100%)

// wah: Note that these temps should be X100 (or adjust the comparisons down to X1)
#define PRIMARY_CATALYST_WARM          375   // Temperature to reach for start-up to succeed
#define PRIMARY_CATALYST_HOT           450   // Temperature to reach for light-off to succeed Was 350 - rk 30Sep21
#define PRIMARY_CATALYST_COOLDOWN      300   // Temperature to reach for cooldown
#define PRIMARY_CATALYST_SS_MIN        695   // Lower Limit of Deadband Window
#define PRIMARY_CATALYST_SS_TARGET     700   // Temperature to stay at during steady-state 
#define PRIMARY_CATALYST_SS_MAX        705   // Upper Limit of Deadband Window
#define PRIMARY_CATALYST_OT            850   // Catalyst overtemp level
#define PRIMARY_CATALYST_SS_RANGE      10    // Temperature target range to stay within 
#define SECONDARY_CATALYST_COOLDOWN    400   // Temperature to reach for cooldown
#define SECONDARY_CATALYST_WARM        300   // Temperature to reach for light-off to succeed Added - RK 28Oct21
#define SECONDARY_CATALYST_HOT         700   // Temperature to enable S_DEC amplification
#define SECONDARY_CATALYST_PRIME       850   // Temperature to disable S_DEC amplification
#define SECONDARY_CATALYST_SS_MAX      905   // Maximum Catalyst Temperature
#define SECONDARY_CATALYST_OT          950   // Catalyst overtermp level
#define SECONDARY_CATALYST_SS_TARGET   900   // Temperature to stay at during steady-state 
#define SECONDARY_CATALYST_SS_MIN      895   // Temperature to stay above during steady-state
#define SECONDARY_CATALYST_SS_RANGE    10    // Temperature target range to stay within 
#define TEMP_OFFSET                    700   // Value used to calculate Loop Time
#define BACKFLOW_DETECTED              150   // Value used to detect Back Flow

#define ADC_COUNTS_AT_OPEN_THERMISTOR_3_3V  18256	// ADC counts when the input is at 3.3V (open Thermistor) when ADC 'Other' config is 16-bit (0x475A)

#define X_32BIT_THRESHOLD_HI    1038576
#define X_32BIT_THRESHOLD_LO    10000
#define X_16BIT_THRESHOLD_HI    34535
#define X_16BIT_THRESHOLD_LO    1000

//Thermocouple Correction Factor - Statid
#define GLOW_PLUG_UNKNOWN   0    // power up state
#define GLOW_PLUG_ON        1    // glow plug is on/open
#define GLOW_PLUG_OFF       2    // glow plug is off/closed
#define GLOW_PLUG_LOCKED    3    // glow plug is locked (off)

#define CJ_CHANNEL    		12    // Cold junction temperature
#define VREF_MUX_CHANNEL   13    // Vref mux channel parameter
#define OFFSET_CHANNEL    	14    // Offset channel parameter

#define IS_RATIO            13000   // Current sense ratio of effector FETs
#define IS_SET_RESISTANCE   1200    // Load resistance value on current sense line

#define Thermistor_Coeff_A  0.0008860044
#define Thermistor_Coeff_B  0.000255924
#define Thermistor_Coeff_C  1.418949E-07

// CAN message send rates, in milliseconds
#define CAN_SEND_STATUS_MS			      1000
#define CAN_SEND_THERMOCOUPLE_MS 		250
#define CAN_SEND_EFFECTOR_CURRENT_MS 	100
#define CAN_SEND_HEX_MEDIA_TEMPS_MS 	250
#define CAN_SEND_PRIMARY_FAN_MS 		   100
#define CAN_SEND_SECONDARY_FAN_MS 		100
#define CAN_SEND_RESERVED_MS 			   250
#define CAN_SEND_HOURS_MS 			      1000

// Timeouts in milliseconds
#define STARTUP_TIME_LIMIT_MS		      300000L	// 5 minutes
#define LIGHTOFF_TIME_LIMIT_MS		   300000L	// 5 minutes
#define STEADYSTATE_TIME_LIMIT_MS	   3600000L // 60 minutes

/* Zero degree offset voltage of DS600(509 mV from sensor datasheet) */
#define ZERO_DEGREE_OFFSET_VOLTAGE     509000

/* Temperature coefficient for DS600(6.45 mV/°C from sensor datasheet) */
#define VOLTAGE_PER_DEGREE             645
#define SCALING							   0.1

/***************************************
*        Function Prototypes
***************************************/
void Read_Inputs(void);
void Manage_System_States(void);
void Write_Outputs(void);
void Manage_Watchdog(void);
void Read_Can(void);
void Write_Can(void);
void Continuous_Bit(void);
void Idle(void);		
void Init_Sensors(void);
void Check_Thermocouple_1( );
void Check_Thermocouple_2( );
void Check_Thermocouple_3( );
void Check_Thermocouple_4( );
void Check_Thermistor_In( );  
void Check_Thermistor_Out( );
void Check_Pri_Fan_IS( );         
void Check_Sec_Fan_IS( );         
void Check_CP_IS( );              
void Check_GP_IS( );              
void Check_Inj_IS( );            
void Check_PU_IS( );
void Check_Reserve_Fuel_Switch( );
void Check_Empty_Fuel_Switch( );
void Check_CJC_IC( );
void Check_Heat_Ex_Delta( );

void Start_Up(void);
void Light_Off(void);
void Check_Primary_Oxidizer( );
void Check_Secondary_Oxidizer( );
void Standard_Shutdown(uint8_t);
void Emergency_Shutdown(uint8_t);
void Write_Logfile(void);
void Sensor_Plausibility_Checks(void);
void S_CAT_OT(void);
void P_CAT_OT(void);
void Get_Thermocouple_1_Counts( );
void Get_Thermocouple_2_Counts( );
void Get_Thermocouple_3_Counts( );
void Get_Thermocouple_4_Counts( );
void Get_HEX_In_Counts( );
void Get_HEX_Out_Counts( );
void Get_Pri_Fan_IS_Counts( );
void Get_Sec_Fan_IS_Counts( );
void Get_CP_IS_Counts( );
void Get_GP_IS_Counts( );
void Get_Inj_IS_Counts( );
void Get_PU_IS_Counts( );
float Get_Primary_Temperature( );
float Get_Secondary_Temperature( );
float Get_Mixing_Temperature( );
float Get_EGT_Temperature( );
float Get_HEX_In_Temperature( );
float Get_HEX_Out_Temperature( );
float Get_Pri_Fan_IS( );
float Get_Sec_Fan_IS( );
float Get_Coolant_Pump_IS( );
float Get_Fuel_Pump_IS( );
float Get_Glow_Plug_IS( );
float Get_Injector_IS( );
float Exponential_Filter(float, float, float);
int32_t Get_Offset_Voltage(void);
int32_t Celsius_To_Counts(int32_t, int8_t);
int32_t Get_Pri_Fan_Speed(int32_t);
int32_t Sec_Fan_Speed(int32_t);

void Init_Outputs();
void Read_Inputs();
void Manage_System_States();
void Write_Outputs();
void Manage_Watchdog();
void Calc_FF();
void BackFlow_Det();
void Read_Can();
void Write_Can();
void Idle();
void Init_CAN();
uint8_t can_send_status_msg();
uint8_t can_send_thermocouple_msg();
uint8_t can_send_effector_current_msg();
uint8_t can_send_hex_media_temps_msg();
uint8_t can_send_primary_fan_msg();
uint8_t can_send_secondary_fan_msg();
uint8_t can_send_reserved_msg();
uint8_t can_send_hours_msg();

void write_glow_plug_output();
void update_injector_pwm();
void print_status_page();
void get_typed_keys(void);
void print_debug(char string[CHAR_ARRAY_SIZE]);
void write_coolant_pump_output(void);
void write_fuel_pump_output(void);
void write_pri_fan_output(void);
void write_sec_fan_output(void);

int32 Measure_CJ_Temp(void);

/***************************************
*        Structures
***************************************/
typedef struct tc_temp_type {
   uint16_t status;                // current status of the temperature: TEMP_UNKNOWN, TEMP_GOOD, TEMP_BAD, TEMP_FAULT
   uint16_t error_count;           // number of out-of-range errors
   uint32_t error_high_threshold;  // fixed value for an out-of-range (high) error
   uint32_t error_low_threshold;   // fixed value for an out-of-range (low) error
   uint16_t fault_threshold;       // fault threshold trigger
   uint32_t raw_adc_counts;        // ADC counts
   uint32_t offset_adc_counts;     // offset value
   uint16_t mux;
   float temp_deg_c;
} tc_temp_t;

typedef struct hex_temp_type {
   uint16_t thm_status;               // current status of the temperature: TEMP_UNKNOWN, TEMP_GOOD, TEMP_BAD, TEMP_FAULT
   uint16_t thm_error_count;          // number of out-of-range errors
   uint16_t thm_error_high_threshold; // fixed value for an out-of-range (high) error
   uint16_t thm_error_low_threshold;  // fixed value for an out-of-range (low) error
   uint16_t thm_fault_threshold;      // fault threshold trigger
   uint16_t ref_status;               // current status of the temperature: TEMP_UNKNOWN, TEMP_GOOD, TEMP_BAD, TEMP_FAULT
   uint16_t ref_error_count;          // number of out-of-range errors
   uint16_t ref_error_high_threshold; // fixed value for an out-of-range (high) error
   uint16_t ref_error_low_threshold;  // fixed value for an out-of-range (low) error
   uint16_t ref_fault_threshold;      // fault threshold trigger
   uint32_t raw_thm_adc_counts;       // ADC counts of the thermistor
   uint32_t raw_ref_adc_counts;       // ADC counts of the reference resistor
   uint32_t offset_adc_counts;        // offset value
   uint16_t mux;
   uint32_t resistance;               // thermistor resistance
   float temp_deg_c;
} hex_temp_t;

typedef struct current_sense_type {
   uint16_t status;                // current status of the temperature: CURRENT_UNKNOWN, CURRENT_GOOD, CURRENT_BAD, CURRENT_FAULT
   uint16_t error_count;           // number of out-of-range errors
   uint16_t error_high_threshold;  // fixed value for an out-of-range (high) error
   uint16_t error_low_threshold;   // fixed value for an out-of-range (low) error
   uint16_t fault_threshold;       // fault threshold trigger
   uint16_t raw_adc_counts;        // ADC counts
   uint16_t mux;
   float current_mA;
} current_sense_t;

typedef struct cjc_reference_type {
   uint16_t status;                // current status of the temperature: TEMP_UNKNOWN, TEMP_GOOD, TEMP_BAD, TEMP_FAULT
   uint16_t error_count;           // number of out-of-range errors
   uint16_t error_high_threshold;  // fixed value for an out-of-range (high) error
   uint16_t error_low_threshold;   // fixed value for an out-of-range (low) error
   uint16_t fault_threshold;       // fault threshold trigger
   uint32_t raw_adc_counts;        // ADC counts
   uint16_t mux;
   float temp_deg_c;
} cjc_reference_t;

typedef struct level_switch_input_type {
   uint16_t status;                // current status of the temperature: LEVEL_LOW, LEVEL_HIGH, LEVEL_BAD, LEVEL_FAULT
   uint16_t error_count;           // number of out-of-range errors
} level_switch_input_t;

typedef struct glow_plug_type {
   uint16_t status;				      // current status of the glow plug: GLOW_PLUG_UNKNOWN, GLOW_PLUG_ON, GLOW_PLUG_OFF, GLOW_PLUG_LOCKED
} glow_plug_t;

typedef struct pump_control_type {
   uint16_t status;				      // current status of the pump: PUMP_UNKNOWN, PUMP_GOOD, PUMP_BAD, PUMP_FAULT
   uint16_t cmd_speed_rpm;
   uint16_t measured_speed_rpm;
   uint16_t duty;					   // duty cycle 0-100
} pump_control_t;

typedef struct fan_control_type {
   uint16_t status;				      // current status of the fan: FAN_UNKNOWN, FAN_ON, FAN_OFF, FAN_GOOD, FAN_BAD, FAN_FAULT
   uint16_t cmd_speed_rpm;         // desired fan speed
   uint16_t measured_speed_rpm;    // actual fan speed
	uint16_t shutdown_speed;         // speed setpoint at shutdown
   //uint16_t power_on;            // toggles ProFet on-off
   uint16_t duty;					   // duty cycle 0-100
} fan_control_t;

typedef struct subsystem_type {
   uint16_t status;				      // current status of the subsystem: SYSTEM_UNKNOWN, SYSTEM_GOOD, SYSTEM_BAD, SYSTEM_FAULT
   uint16_t error_count;
} subsystem_t;

typedef struct catalyst_type {
   uint16_t status;				      // current status of the subsystem: SYSTEM_UNKNOWN, SYSTEM_GOOD, SYSTEM_BAD, SYSTEM_FAULT
	uint16_t light_off_temp_required;
	uint32_t light_off_time_limit;
	uint16_t cooldown_temp;
} catalyst_t;

typedef struct injector_control_type {
   uint16_t status;				      // current status of the fan: FAN_UNKNOWN, FAN_GOOD, FAN_BAD, FAN_FAULT
   uint16_t duty;					   // duty cycle 0-100
} injector_control_t;


//typedef struct analog_sensor_t {
//	uint16_t enabled;			   // user enable or disable of sensor from GUI
//	uint16_t status;			   // current status of sensor: SENSOR_UNKNOWN, SENSOR_GOOD, SENSOR_PREALARM, fault_code, SENSOR_BAD, SENSOR_DISABLED
//	uint16_t range_error_count;   // number of readings that have been out-of-range
//	uint16_t low_gas_count;	   // number of readings that indicate a low gas condition
//	uint16_t error_low;		  // fixed value for a out-of-range (low) error
//	uint16_t error_high;	  // fixed value for a out-of-range (high) error 
//	uint16_t zero_counts;	  // for gas level sensor, ADC counts for 0%
//	uint16_t hundred_counts;  // for gas level sensor, ADC counts for 100%
//	uint16_t low_gas_trip;	  // level that below which will trip 'low gas' and prevent gas from being used
//	uint16_t prealarm;		  // level for a prealarm trip
//	uint16_t fault;			  // level for a fault trip
//	uint16_t raw_adc_value;	  // ADC counts
//	int english_value;   // ADC reading converted to English units
//	int metric_value;	  // ADC reading converted to Metric units
//} analog_sensor;

/***************************************
*        Extern declarations
***************************************/
// system variables
extern const uint8_t firmware_high;
extern const uint8_t firmware_low;

extern uint32_t main_timer_ms;
extern uint32_t start_up_timer_ms;
extern uint32_t light_off_timer_ms;
extern uint16_t system_state;
extern uint16_t fault_code;

// sensor variables
extern tc_temp_t primary_temp;
extern tc_temp_t secondary_temp;
extern tc_temp_t mixing_temp;
extern tc_temp_t egt_temp;
extern hex_temp_t hex_inlet_temp;
extern hex_temp_t hex_outlet_temp;
extern current_sense_t cp_current;
extern current_sense_t gp_current;
extern current_sense_t inj_current;
extern current_sense_t pf_current;
extern current_sense_t pu_current;
extern current_sense_t sf_current;
extern cjc_reference_t cjc_reference;
extern fan_control_t pri_fan_speed[2];
extern fan_control_t sec_fan_speed[2];
extern fan_control_t pri_fan_output;
extern fan_control_t sec_fan_output;
extern pump_control_t coolant_pump;
extern pump_control_t fuel_pump;
extern subsystem_t heat_exchanger;
extern subsystem_t primary_air_supply;
extern subsystem_t secondary_air_supply;
extern injector_control_t injector;
extern level_switch_input_t level_switch_1;
extern level_switch_input_t level_switch_2;
extern catalyst_t primary_catalyst;
extern catalyst_t secondary_catalyst;
extern subsystem_t current_consumption;
extern glow_plug_t glow_plug;

extern uint8_t block_shutdown;

extern uint16_t fuel_duty_startup;    
extern uint16_t fuel_duty_full; 	   
extern uint16_t fuel_duty_manual; 	   
extern uint16_t fuel_duty_shutdown;   


/* [] END OF FILE */
