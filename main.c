/* ========================================
 *
 * Copyright Proof Energy, 2021
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Proof Energy.
 *
 * NOITE: FAN MODULES INFO -RK - 04Oct21
 * 1 Fan Module is made up of 2, counter-rotating units. Both units have a unique tach 
 * signal output. In this application, only the OUTLET fan tach is monitored and used 
 * as the feedback signal - the INLET fan signal is not used, since these units rotate
 * at different speeds.
 * =======================================================================================
*/
#include "main.h"
#include <stdio.h>
#include "math.h"
#include "Pri_Fan_Array.h"
#include "Sec_Fan_Array.h"
#include "Thermocouple_1.h"
#include "CAN_1.h"
#include "UART_1.h"
//#include "project.h"

//int32 Thermocouple_1_GetVoltage(int32 temperature) ;


// global variables
const uint8_t firmware_high = 1;	// firmware version v0.2
const uint8_t firmware_low = 8;     // RK 19Jan22

// system variables

uint32_t j;
int16_t  i;
uint32_t n;
uint32_t main_timer_ms;
uint16_t system_state;
uint16_t fault_code;
uint32_t light_off_time_ms;
uint8_t  plot;
uint8_t  slot;

uint16_t P_INC;
uint16_t P_DEC;
uint16_t S_INC;
uint16_t S_DEC;

uint16_t primary_catalyst_light_off_temp_required;
uint16_t secondary_catalyst_light_off_temp_required;

uint16_t primary_fan_light_off_speed = 4500;
uint16_t secondary_fan_light_off_speed = 4500;

uint32_t startup_time_limit_ms;
uint32_t lightoff_time_limit_ms;
uint32_t steadystate_time_limit_ms;

float_t  sec_temp_control;

//Loop Tuning Parameters; P,I,D,A New - RK 15Nov21
/*uint16_t pri_fan_p_term;
uint16_t pri_fan_i_term;
uint16_t pri_fan_d_term;
uint16_t pri_fan_a_term;
uint16_t sec_fan_p_term;
uint16_t sec_fan_i_term;
uint16_t sec_fan_d_term;
uint16_t sec_fan_a_term;*/

//Fuel Parameters
uint16_t fuel_duty_startup;    
uint16_t fuel_duty_full; 	   
uint16_t fuel_duty_manual; 	   
uint16_t fuel_duty_shutdown;   

// sensor variables
tc_temp_t primary_temp;
tc_temp_t secondary_temp;
tc_temp_t mixing_temp;
tc_temp_t egt_temp;
hex_temp_t hex_inlet_temp;
hex_temp_t hex_outlet_temp;
current_sense_t cp_current;
current_sense_t gp_current;
current_sense_t inj_current;
current_sense_t pf_current;
current_sense_t pu_current;
current_sense_t sf_current;
cjc_reference_t cjc_reference;
fan_control_t pri_fan_speed[2];
fan_control_t sec_fan_speed[2];
pump_control_t coolant_pump;
pump_control_t fuel_pump;
subsystem_t heat_exchanger;
subsystem_t primary_air_supply;
subsystem_t secondary_air_supply;
injector_control_t injector;
level_switch_input_t level_switch_1;
level_switch_input_t level_switch_2;
catalyst_t primary_catalyst;
catalyst_t secondary_catalyst;
subsystem_t current_consumption;
glow_plug_t glow_plug;

uint8_t Temperature_Overrides;
uint8_t block_shutdown;

// CAN variables
//CAN_1_RX_MSG can_rx_message;  // undefined
CAN_1_TX_MSG can_tx_message;

uint32_t min_loop_time = 0xFFFFFFFF;	
uint32_t max_loop_time = 0;	
uint32_t last_loop_time = 0;

int32_t offset_microvolts;
int32_t cold_junction_temp;
uint16_t pn2;                 // LUT Primary Fan Speed
int32_t pcorr;                // Correction Factor for pn2 PID Control
uint16_t sn2;                 // LUT Secondary Fan Speed
int32_t scorr;                // Correction Factor for sn2 PID Control

//*****************************************************************************
//* Function:   Main
//*
//* Parameters:
//*   							   
//* Description:  The main control loop
//*
//*****************************************************************************
int main(void)
{
	char str[CHAR_ARRAY_SIZE];

   CyGlobalIntEnable; /* Enable global interrupts. */
   ADC_DelSig_1_IRQ_Enable ();
  	EEPROM_1_Enable();

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    
	isr_millisecond_Start();
   Init_Outputs ();               
   HeartBeat_Start ();
   Opamp_1_Start( );
   INJ_MOD_Start ();
   Pri_Fan_Array_Start ();
   Sec_Fan_Array_Start ();
   ADC_DelSig_1_Start ();
   AMux_1_Start();
   IDAC8_1_Start ();
   Init_Sensors ();
	Init_CAN();

    /* Start UART operation */
   UART_1_Start();
    
	UART_1_PutString("\n\r");
	sprintf(str,"\n\rProof Energy ClearTherm Controller S5209 v%d.%d",firmware_high,firmware_low);
	UART_1_PutString(str);
	UART_1_PutString("\n\r");
	print_status_page();

    Sensor_Plausibility_Checks();   //TBD
    
    while(1)
	{
		Read_Inputs ();      // read ADCs, digital inputs, feedback inputs
		
		Read_Can ();   		 // get new CAN messages

		Manage_System_States ();     // control system states
		
		Write_Outputs ();    // write DAC, power, digital outputs
        
//      Get_Pri_Fan_Speed (pn2);       // Determine Primary Fan Speed via LUT - new 16Jan22
        
      BackFlow_Det ();    // BackFlow Detection - new 10Jan22
		
		Manage_Watchdog ();  // tickle the watchdog to prevent a reset
		
  		Write_Can ();  		 // send updated CAN messages 

		Continuous_Bit ();	 // built-in-test

 		Idle ();			 // manage usb comm, await next 20ms loop  
	}
    
}

//*****************************************************************************
//* Function:   Init_Sensors
//*
//* Usage: 
//*   
//* Description:    Assigns parameter constants to each sensor.
//*
//*****************************************************************************
void Init_Sensors(void){    // assign mux channels to sensors
    primary_temp.mux = 0;
    primary_temp.error_high_threshold = X_16BIT_THRESHOLD_HI;      // assuming 20-bit ADC; 10000 threshold
    primary_temp.error_low_threshold = X_16BIT_THRESHOLD_LO;
    primary_temp.fault_threshold = Celsius_To_Counts(700, primary_temp.mux);    //TBD confirm w/ Kurt what temp is "'too hot"
    
    mixing_temp.mux = 1;
    mixing_temp.error_high_threshold = X_16BIT_THRESHOLD_HI;       // assuming 20-bit ADC; 10000 threshold
    mixing_temp.error_low_threshold = X_16BIT_THRESHOLD_LO;
    mixing_temp.fault_threshold = Celsius_To_Counts(700, mixing_temp.mux);   //TBD assuming same temperature requirements as PO element
    
    secondary_temp.mux = 2;
    secondary_temp.error_high_threshold = X_16BIT_THRESHOLD_HI;    // assuming 20-bit ADC; 10000 threshold
    secondary_temp.error_low_threshold = X_16BIT_THRESHOLD_LO;
    secondary_temp.fault_threshold = Celsius_To_Counts(800, secondary_temp.mux);
    
    egt_temp.mux = 3;
    egt_temp.error_high_threshold = X_16BIT_THRESHOLD_HI;          // assuming 20-bit ADC; 10000 threshold
    egt_temp.error_low_threshold = X_16BIT_THRESHOLD_LO;
    egt_temp.fault_threshold = Celsius_To_Counts(800, egt_temp.mux);          //TBD temperature requirement not specified
    
    hex_inlet_temp.mux = 4;
    hex_inlet_temp.thm_error_high_threshold = X_16BIT_THRESHOLD_HI;    // assuming 16-bit ADC; 1000 threshold
    hex_inlet_temp.thm_error_low_threshold = X_16BIT_THRESHOLD_LO;
    hex_inlet_temp.thm_fault_threshold = Celsius_To_Counts(800,hex_inlet_temp.mux);    //TBD temperature requirement not specified
    hex_inlet_temp.ref_error_high_threshold = X_16BIT_THRESHOLD_HI;    // assuming 16-bit ADC; 1000 threshold
    hex_inlet_temp.ref_error_low_threshold = X_16BIT_THRESHOLD_LO;
    hex_inlet_temp.ref_fault_threshold = Celsius_To_Counts(800,hex_inlet_temp.mux);    //TBD temperature requirement not specified
    
    hex_outlet_temp.mux = 5;
    hex_outlet_temp.thm_error_high_threshold = X_16BIT_THRESHOLD_HI;    // assuming 16-bit ADC; 1000 threshold
    hex_outlet_temp.thm_error_low_threshold = X_16BIT_THRESHOLD_LO;
    hex_outlet_temp.thm_fault_threshold = Celsius_To_Counts(800, hex_outlet_temp.mux);   //TBD temperature requirement not specified
    hex_outlet_temp.ref_error_high_threshold = X_16BIT_THRESHOLD_HI;    // assuming 16-bit ADC; 1000 threshold
    hex_outlet_temp.ref_error_low_threshold = X_16BIT_THRESHOLD_LO;
    hex_outlet_temp.ref_fault_threshold = Celsius_To_Counts(800, hex_outlet_temp.mux);   //TBD temperature requirement not specified
    
    cp_current.mux = 6;
    cp_current.error_high_threshold = X_16BIT_THRESHOLD_HI;    // assuming 16-bit ADC; 1000 threshold
    cp_current.error_low_threshold = 0;		   // 0 mA current is fine
    cp_current.fault_threshold = X_16BIT_THRESHOLD_HI ;                         // TBD until customer defined
    
    gp_current.mux = 7;
    gp_current.error_high_threshold = X_16BIT_THRESHOLD_HI;    // assuming 16-bit ADC; 1000 threshold
    gp_current.error_low_threshold = 0;		   // 0 mA current is fine
    gp_current.fault_threshold = X_16BIT_THRESHOLD_HI ;                         // TBD until customer defined
    
    inj_current.mux = 8;
    inj_current.error_high_threshold = X_16BIT_THRESHOLD_HI;    // assuming 16-bit ADC; 1000 threshold
    inj_current.error_low_threshold = 0;		   // 0 mA current is fine
    inj_current.fault_threshold = X_16BIT_THRESHOLD_HI ;                        // TBD until customer defined
    
    pf_current.mux = 9;
    pf_current.error_high_threshold = X_16BIT_THRESHOLD_HI;    // assuming 16-bit ADC; 1000 threshold
    pf_current.error_low_threshold = 0;		   // 0 mA current is fine
    pf_current.fault_threshold = X_16BIT_THRESHOLD_HI ;                         // TBD until customer defined
    
    pu_current.mux = 10;
    pu_current.error_high_threshold = X_16BIT_THRESHOLD_HI;    // assuming 16-bit ADC; 1000 threshold
    pu_current.error_low_threshold = 0;		   // 0 mA current is fine
    pu_current.fault_threshold = X_16BIT_THRESHOLD_HI ;                         // TBD until customer defined
    
    sf_current.mux = 11;
    sf_current.error_high_threshold = X_16BIT_THRESHOLD_HI;    // assuming 16-bit ADC; 1000 threshold
    sf_current.error_low_threshold = 0;		   // 0 mA current is fine
    sf_current.fault_threshold = X_16BIT_THRESHOLD_HI ;                         // TBD until customer defined
    
    cjc_reference.mux = 12;
    cjc_reference.error_high_threshold = X_16BIT_THRESHOLD_HI;    // assuming 16-bit ADC; 1000 threshold
    cjc_reference.error_low_threshold = X_16BIT_THRESHOLD_LO;
    cjc_reference.fault_threshold = X_16BIT_THRESHOLD_HI ;                      // TBD until customer defined

    primary_air_supply.status = GOOD;              
    secondary_air_supply.status = GOOD;              
	heat_exchanger.status = GOOD;
	primary_catalyst.status	= GOOD;
	secondary_catalyst.status = GOOD;

	primary_catalyst.light_off_temp_required = PRIMARY_CATALYST_HOT;   
	secondary_catalyst.light_off_temp_required = SECONDARY_CATALYST_WARM;

	primary_catalyst.light_off_time_limit = LIGHTOFF_TIME_LIMIT_MS;
	secondary_catalyst.light_off_time_limit = LIGHTOFF_TIME_LIMIT_MS;

	primary_catalyst.cooldown_temp = PRIMARY_CATALYST_COOLDOWN; 
	secondary_catalyst.cooldown_temp = SECONDARY_CATALYST_COOLDOWN; 

	startup_time_limit_ms = STARTUP_TIME_LIMIT_MS;
	lightoff_time_limit_ms = LIGHTOFF_TIME_LIMIT_MS;
	steadystate_time_limit_ms = STEADYSTATE_TIME_LIMIT_MS;

	fuel_duty_startup = FUEL_DUTY_STARTUP;  
	fuel_duty_full = FUEL_DUTY_FULL;		   
	fuel_duty_manual = FUEL_DUTY_MANUAL;	   
	fuel_duty_shutdown = FUEL_DUTY_SHUTDOWN;

}

//*****************************************************************************
//* Function:   Init_Outputs
//*
//* Usage: 
//*   
//* Description:    Configure the outputs to startup as "Off" (Low)
//*
//*****************************************************************************
void Init_Outputs(void){
	//uint8_t i;

/*    GP_IS_Write(1);	   // 0=Off 
    CP_IS_Write(1);	   // 0=Off
    INJ_IS_Write(1);   // 0=Off
    PU_IS_Write(1);	   // 0=Off
    PF_IS_Write(1);	   // 0=Off
    SF_IS_Write(1);	   // 0=Off
    CJCP_Write(1);	   // 0=Off	   */

	// Fan outputs
    /*for (i = 0; i < Pri_Fan_Array_NUMBER_OF_FANS; i++) {
		Pri_Fan_Array_SetDutyCycle(i+1, FAN_DUTY_CYCLE);	   //  Valid range is 0 to 10000
		Sec_Fan_Array_SetDutyCycle(i+1, FAN_DUTY_CYCLE);
    }*/

	// PWM outputs
	injector.duty = FUEL_DUTY_OFF;
    update_injector_pwm( );  

	// Power outputs
	glow_plug.status = GLOW_PLUG_OFF; 
	write_glow_plug_output( );

	coolant_pump.status	= PUMP_OFF;  
	write_coolant_pump_output( );

	fuel_pump.status = PUMP_OFF; 
	write_fuel_pump_output( );
    
    //fan.control.status = FAN_OFF;
    write_pri_fan_output( );
    write_sec_fan_output( );
	
}

//*****************************************************************************
//* Function:   Start_Up
//*
//* Usage: 
//*   
//* Description:    Initiates start-up sequence. Powers up the primary fan, 
//* secondary fan, and the glow plug. Tests if the PO temperature is > 150degC
//* after 5 minutes. If true, fuel pump is enabled and program proceeds to the 
//* light-off sequence. If false, program issues a master fault and enters a 
//* normal shutdown.
//*
//* Notes:          Valid fan rpm range is 5000 to 32,000
//*                 Number of pri fans: 2
//*                 Number of sec fans: 2
//*****************************************************************************
void Start_Up(void) {
   uint8 i;
    
   //CJCP_Write(0);                          // Power on kelvin sense terminal
   PF_IS_Write(1);                         // Power on primary fan array
   SF_IS_Write(1);                         // Power on secondary fan array
   
   /*for (i = 0; i < Pri_Fan_Array_NUMBER_OF_FANS; i++) {
        pri_fan_speed[i].cmd_speed_rpm = primary_fan_light_off_speed;     //TBD 16 SLM needs to be converted to RPM
        sec_fan_speed[i].cmd_speed_rpm = secondary_fan_light_off_speed;     //FAN_STARTUP_SPEED_SEC;       
    }*/
    
   // Fan Speed Setpoints
   for (i = 0; i < Pri_Fan_Array_NUMBER_OF_FANS; i++) {
         pri_fan_speed[i].cmd_speed_rpm = FAN_STARTUP_SPEED_PRI;
         Pri_Fan_Array_SetDesiredSpeed(i + 1,pri_fan_speed[i].cmd_speed_rpm);
      }
        
      for (i = 0; i < Sec_Fan_Array_NUMBER_OF_FANS; i++) {
         sec_fan_speed[i].cmd_speed_rpm = FAN_STARTUP_SPEED_SEC;
         Sec_Fan_Array_SetDesiredSpeed(i + 1,sec_fan_speed[i].cmd_speed_rpm);
      }
    
   // Fan Speed Setpoints
   //pri_fan_speed[i].cmd_speed_rpm = FAN_IDLE_SPEED_PRI;
   //sec_fan_speed[i].cmd_speed_rpm = FAN_IDLE_SPEED_SEC;
   
   /*for (i = 0; i < Pri_Fan_Array_NUMBER_OF_FANS; i++) {
      Pri_Fan_Array_SetDesiredSpeed(i + 1,pri_fan_speed[i].cmd_speed_rpm);
      }     
   for (i = 0; i < Sec_Fan_Array_NUMBER_OF_FANS; i++) {
      Sec_Fan_Array_SetDesiredSpeed(i + 1,sec_fan_speed[i].cmd_speed_rpm);
      }*/
    
	injector.duty = fuel_duty_startup;		// start fuel injector PWM
 
	coolant_pump.status	= PUMP_ON; 
	write_coolant_pump_output( );

	fuel_pump.status = PUMP_ON;
	write_fuel_pump_output( );
	
   /* Glow plug is driven by
   *  BTS50085-1TMB which is ON when input is LO; OFF when HI.  Signal is inverted by N-Channel FET
   */
	glow_plug.status = GLOW_PLUG_ON;
   write_glow_plug_output( );
}

//*****************************************************************************
//* Function:   Light_Off
//*
//* Usage: 
//*   
//* Description:    Initiates light_off sequence. Powers up the primary fan, 
//* secondary fan, and the glow plug. Tests if the PO temperature is > 150degC
//* after 5 minutes. If true, program proceeds to the light-off sequence. If 
//* false, program issues a master fault and enters a normal shutdown.
//*
//* Notes:          Valid fan rpm range is 7500 to 32,000
//*                 Number of pri fans: 2
//*                 Number of sec fans: 2
//*
//*****************************************************************************
void Light_Off(void) {
	uint8_t i;
   // static uint32_t timer;
    
   // CP_IS_Write(0);     // Power on coolant pump
   // PU_IS_Write(0);     // Power on fuel pump
   // INJ_IS_Write(0);    // Power on injector; TBD convert to PWM; set fuel flow rate to 10mL/min

	//injector.duty = fuel_duty_full;		// lightoff fuel injector PWM
    //pri_fan_speed[i].cmd_speed_rpm = primary_fan_light_off_speed;
    //sec_fan_speed[i].cmd_speed_rpm = secondary_fan_light_off_speed;
    
    for (i = 0; i < Pri_Fan_Array_NUMBER_OF_FANS; i++) {
            Pri_Fan_Array_SetDesiredSpeed(i + 1,pri_fan_speed[i].cmd_speed_rpm);
        }
        
        for (i = 0; i < Sec_Fan_Array_NUMBER_OF_FANS; i++) {
            Sec_Fan_Array_SetDesiredSpeed(i + 1,sec_fan_speed[i].cmd_speed_rpm);
        }
    
	/*for (i = 0; i < Pri_Fan_Array_NUMBER_OF_FANS; i++) {
        pri_fan_speed[i].cmd_speed_rpm = FAN_SS_SPEED_PRI;                        //TBD 37 SLM converted to RPM
        Pri_Fan_Array_SetDesiredSpeed(i + 1,pri_fan_speed[i].cmd_speed_rpm);
    }
    
	for (i = 0; i < Sec_Fan_Array_NUMBER_OF_FANS; i++) {
        sec_fan_speed[i].cmd_speed_rpm = FAN_SS_SPEED_SEC;                        //TBD 430 SLM converted to RPM
        Sec_Fan_Array_SetDesiredSpeed(i + 1,sec_fan_speed[i].cmd_speed_rpm);
    }*/
    
    //TBD INCREASE FUEL FLOW RATE to 46mL/min here:
    
    /* Start-up / light-off sequence successful,
    *  proceed to steady-state main loop.
    */
    //return;
}

//*****************************************************************************
//* Function:   Read_Inputs
//*
//* Usage: 
//*   
//* Description:    Reads ADCs, digital inputs, feedback inputs.
//*
//*****************************************************************************
void Read_Inputs(void) {
	uint8_t i;

    // Select the sensor with the highest temperature reading; inlet or outlet
    
   if (secondary_temp.temp_deg_c > mixing_temp.temp_deg_c){
      (sec_temp_control = secondary_temp.temp_deg_c);
       //n = sec_temp_control;
   }
      else {
         (sec_temp_control = mixing_temp.temp_deg_c);
         //n = sec_temp_control;
      }
    
	if(!Temperature_Overrides)
	{
	   Get_Primary_Temperature(&primary_temp);
	   Get_Mixing_Temperature(&mixing_temp);
		Get_Secondary_Temperature(&secondary_temp);
	   Get_EGT_Temperature(&egt_temp);
	   Get_HEX_In_Temperature(&hex_inlet_temp);
	   Get_HEX_Out_Temperature(&hex_outlet_temp);
	}

      for (i = 0; i < Pri_Fan_Array_NUMBER_OF_FANS; i++) {
         pri_fan_speed[i].measured_speed_rpm = Pri_Fan_Array_GetActualSpeed(i + 1);
         sec_fan_speed[i].measured_speed_rpm = Sec_Fan_Array_GetActualSpeed(i + 1);
   	}

      Check_Thermocouple_1(&primary_temp);    
      Check_Thermocouple_2(&mixing_temp);
      Check_Thermocouple_3(&secondary_temp);
      Check_Thermocouple_4(&egt_temp);
      Check_Thermistor_In(&hex_inlet_temp);
      Check_Thermistor_Out(&hex_outlet_temp);
	   Check_Heat_Ex_Delta(&hex_inlet_temp, &hex_outlet_temp);
    
      Get_Pri_Fan_IS(&pf_current);
      Check_Pri_Fan_IS(&pf_current);
    
      Get_Sec_Fan_IS(&sf_current);
      Check_Sec_Fan_IS(&sf_current);
    
      Get_Coolant_Pump_IS(&cp_current);
      Check_CP_IS(&cp_current);
    
      Get_Fuel_Pump_IS(&pu_current);
      Check_PU_IS(&pu_current);
    
      Get_Injector_IS(&inj_current);
      Check_Inj_IS(&inj_current);
    
      Get_Glow_Plug_IS(&gp_current);
      Check_GP_IS(&gp_current);
}

//*****************************************************************************
//* Function:   Manage_System_States
//*
//* Usage: 
//*   
//* Description:  The main state machine  
//*
//*****************************************************************************
void Manage_System_States(void){
	static uint32_t state_timeout_ms;
   
	switch(system_state)
	{
	case(STATE_UNKNOWN):  // should only get here at power up
	default:
		light_off_time_ms = 0;
	   state_timeout_ms = main_timer_ms + startup_time_limit_ms;
		system_state = STATE_STARTUP;
		UART_1_PutString("\n\r*STATE TO STARTUP*");
		break;
      
	case(STATE_STARTUP):
		Start_Up();
		light_off_time_ms = 0;
	   if((primary_temp.temp_deg_c > PRIMARY_CATALYST_WARM)) 
	   {
	    	state_timeout_ms = main_timer_ms + lightoff_time_limit_ms;
			system_state = STATE_LIGHTOFF;
			UART_1_PutString("\n\r*STATE TO LIGHT-OFF*");            
	   }
		   else if(main_timer_ms > state_timeout_ms)
		   {
			   Standard_Shutdown(11);
		   }
		break;
      
	case(STATE_LIGHTOFF):
		Light_Off();
		light_off_time_ms = 0;
      plot = 0;
      slot = 0;
      
    	   // Emergency Shutdown - Prevent CAT Melting
         if (primary_temp.temp_deg_c >= 800){            // New - 12jan22 kr - Was 850
         UART_1_PutString("\n\r*PRI. CAT OVER TEMP!*");  // Prevents Cat Melting during light-off
         block_shutdown = 0;
         system_state = STATE_EMERGENCY_SHUTDOWN;
         }           
        
         if(main_timer_ms > state_timeout_ms) // Has LightOff timeout been exceeded?
			   {
			   Standard_Shutdown(12); // Yes - Shutdown
			   }
		      else if ((primary_temp.temp_deg_c > primary_catalyst.light_off_temp_required) &&
                    (sec_temp_control > secondary_catalyst.light_off_temp_required)) {   // No - keep going until primary && secondary temp. are met
 
	    	   state_timeout_ms = main_timer_ms + STEADYSTATE_TIME_LIMIT_MS;
			   system_state = STATE_STEADY_STATE;
			   light_off_time_ms = main_timer_ms;
			   injector.duty = fuel_duty_full;
            glow_plug.status = GLOW_PLUG_OFF;
            write_glow_plug_output( );          // Power off glow plug    
			   UART_1_PutString("\n\r*STATE TO STEADY STATE*");
			   }
		break;
         
	case(STATE_STEADY_STATE):    
	    Check_Primary_Oxidizer(&primary_temp);
	    Check_Secondary_Oxidizer(&secondary_temp);    
    	if((primary_temp.temp_deg_c < (PRIMARY_CATALYST_SS_MIN)) && (main_timer_ms > state_timeout_ms))
			{
			system_state = STATE_STANDARD_SHUTDOWN;
			UART_1_PutString("\n\r*STATE TO STANDARD SHUTDOWN*");
			}
    	if((sec_temp_control < (SECONDARY_CATALYST_SS_MIN)) && (main_timer_ms > state_timeout_ms))
			{
			system_state = STATE_STANDARD_SHUTDOWN;
			UART_1_PutString("\n\r*STATE TO STANDARD SHUTDOWN*");
			}
		break;
         
	case(STATE_MANUAL_MODE):
		// perform no system state changes, block shutdowns
		break;
   
	case(STATE_STANDARD_SHUTDOWN):
		Standard_Shutdown(0);
		break;
      
	case(STATE_EMERGENCY_SHUTDOWN):
		Emergency_Shutdown(0);
		break;
	}

   if((coolant_pump.status == PUMP_LOCKED)
      || (heat_exchanger.status == FAULT)
      || (heat_exchanger.status == BAD)
      || ((primary_air_supply.status == FAULT) && (secondary_air_supply.status == FAULT))
      || ((primary_air_supply.status == FAULT) && (secondary_air_supply.status == BAD))
      || ((primary_air_supply.status == BAD) && (secondary_air_supply.status == FAULT))
      || ((primary_air_supply.status == BAD) && (secondary_air_supply.status == BAD))){   
         Emergency_Shutdown(32);
   }
    
   if((fuel_pump.status == PUMP_LOCKED)
      || (injector.status == FAULT)
      || (level_switch_1.status == FAULT)
      || (level_switch_2.status == FAULT)
      || (hex_inlet_temp.thm_status == FAULT)
      || (hex_inlet_temp.ref_status == FAULT)
      || (hex_outlet_temp.thm_status == FAULT)
      || (hex_outlet_temp.ref_status == FAULT)
      || (primary_temp.status == FAULT)
      || (mixing_temp.status == FAULT)
      || (secondary_temp.status == FAULT)
      || (egt_temp.status == FAULT)
      || (primary_catalyst.status == FAULT)
      || (secondary_catalyst.status == FAULT)
		|| (gp_current.status == FAULT)
		|| (cp_current.status == FAULT)
		|| (inj_current.status == FAULT)
		|| (pf_current.status == FAULT)
		|| (sf_current.status == FAULT)
		|| (pu_current.status == FAULT)
      || (current_consumption.status == FAULT)
      || (injector.status == BAD)
      || (level_switch_1.status == BAD)
      || (level_switch_2.status == BAD)
      || (hex_inlet_temp.thm_status == BAD)
      || (hex_inlet_temp.ref_status == BAD)
      || (hex_outlet_temp.thm_status == BAD)
      || (hex_outlet_temp.ref_status == BAD)
      || (primary_temp.status == BAD)
      || (mixing_temp.status == BAD)
      || (secondary_temp.status == BAD)
      || (egt_temp.status == BAD)
      || (primary_catalyst.status == BAD)
      || (secondary_catalyst.status == BAD)
		|| (gp_current.status == BAD)
		|| (cp_current.status == BAD)
		|| (inj_current.status == BAD)
		|| (pf_current.status == BAD)
		|| (sf_current.status == BAD)
		|| (pu_current.status == BAD)
      || (current_consumption.status == BAD)){   
        Standard_Shutdown(13);
   }
    
    Write_Logfile();
}

//*****************************************************************************
//* Function:   Write_Outputs
//*
//* Usage: 
//*   
//* Description:    
//*
//*****************************************************************************
void Write_Outputs(void){
	// PWM outputs
   update_injector_pwm( );  

	// Power outputs
	write_glow_plug_output( );
	write_coolant_pump_output( );
	write_fuel_pump_output( );
   write_pri_fan_output( );
   write_sec_fan_output( );
}

//*****************************************************************************
//* Function:   Manage_Watchdog
//*
//* Usage: 
//*   
//* Description:    
//*
//*****************************************************************************
void Manage_Watchdog(void){
     // TBD
}

//*****************************************************************************
//* Function:   Read_Can
//*
//* Usage: 
//*   
//* Description:    
//*
//*****************************************************************************
void Read_Can(void){
		  // TBD
}

//*****************************************************************************
//* Function:   Write_Can
//*
//* Usage: 
//*   
//* Description:    
//*
//*****************************************************************************
void Write_Can(void){
	static uint32_t next_send_status_ms = CAN_SEND_STATUS_MS;
	static uint32_t next_send_thermocouple_ms = CAN_SEND_THERMOCOUPLE_MS;
	static uint32_t next_send_effector_current_ms = CAN_SEND_EFFECTOR_CURRENT_MS;
	static uint32_t next_send_hex_media_temps_ms = CAN_SEND_HEX_MEDIA_TEMPS_MS;
	static uint32_t next_send_primary_fan_ms = CAN_SEND_PRIMARY_FAN_MS;
	static uint32_t next_send_secondary_fan_ms = CAN_SEND_SECONDARY_FAN_MS;
	static uint32_t next_send_reserved_ms = CAN_SEND_RESERVED_MS;
	static uint32_t next_send_hours_ms = CAN_SEND_HOURS_MS;

	if(main_timer_ms >= next_send_status_ms)
		{
		can_send_status_msg();
		next_send_status_ms = main_timer_ms + CAN_SEND_STATUS_MS;
    	}

	if(main_timer_ms >= next_send_thermocouple_ms)
		{
		can_send_thermocouple_msg();
		next_send_thermocouple_ms = main_timer_ms + CAN_SEND_THERMOCOUPLE_MS;
    	}

	if(main_timer_ms >= next_send_effector_current_ms)
		{
		can_send_effector_current_msg();
		next_send_effector_current_ms = main_timer_ms + CAN_SEND_EFFECTOR_CURRENT_MS;
    	}

	if(main_timer_ms >= next_send_hex_media_temps_ms)
		{
		can_send_hex_media_temps_msg();
		next_send_hex_media_temps_ms = main_timer_ms + CAN_SEND_HEX_MEDIA_TEMPS_MS;
    	}

	if(main_timer_ms >= next_send_primary_fan_ms)
		{
		can_send_primary_fan_msg();
		next_send_primary_fan_ms = main_timer_ms + CAN_SEND_PRIMARY_FAN_MS;
    	}

	if(main_timer_ms >= next_send_secondary_fan_ms)
		{
		can_send_secondary_fan_msg();
		next_send_secondary_fan_ms = main_timer_ms + CAN_SEND_SECONDARY_FAN_MS;
    	}

	if(main_timer_ms >= next_send_reserved_ms)
		{
		can_send_reserved_msg();
		next_send_reserved_ms = main_timer_ms + CAN_SEND_RESERVED_MS;
    	}
		
	if(main_timer_ms >= next_send_hours_ms)
		{
		can_send_hours_msg();
		next_send_hours_ms = main_timer_ms + CAN_SEND_HOURS_MS;
    	}
}

//*****************************************************************************
//* Function:   Idle
//*
//* Usage: 
//*   
//* Description:    
//*
//*****************************************************************************
void Idle(void){
    static uint32_t last_main_timer_ms;
	static uint32_t delta_loop_time_ms;

	get_typed_keys();
    while(main_timer_ms < (last_main_timer_ms + MAIN_LOOP_TIME) )
    	;   //DO NOTHING
    
	delta_loop_time_ms = main_timer_ms - last_main_timer_ms;
	
	if(delta_loop_time_ms < min_loop_time)
		min_loop_time = delta_loop_time_ms;	

	if(delta_loop_time_ms > max_loop_time)
		max_loop_time = delta_loop_time_ms;	

	last_loop_time = delta_loop_time_ms;

    last_main_timer_ms = main_timer_ms;
}

//*****************************************************************************
// Calculate Feed-Forward Terms
// New - 4Jan22 - rk
//*****************************************************************************
void Calc_FF(void)
{

}    

//*****************************************************************************
// Test for BackFlow Condition
// New - 10Jan22 - rk
//*****************************************************************************
void BackFlow_Det(void)
{
    if (egt_temp.temp_deg_c > BACKFLOW_DETECTED){
            UART_1_PutString("\n\t\v\r*BACKFLOW DETECTED!*");
            block_shutdown = 0;
            Temperature_Overrides = 0;
            system_state = STATE_EMERGENCY_SHUTDOWN;
    }
}

//*****************************************************************************
//* Function:   get_typed_keys
//*
//* Usage: 
//*   
//* Description:    
//*
//*****************************************************************************
void get_typed_keys(void)
{								
	uint8_t readStatus;
	uint8 TypedChar;

    readStatus = UART_1_RXSTATUS_REG;
    if((readStatus & UART_1_RX_STS_FIFO_NOTEMPTY) != 0u)
		{
		TypedChar = UART_1_ReadRxData();
		switch(TypedChar)
			{
			case('A'):
			case('a'):
				UART_1_PutString("\n\rA-LightOff");
				Temperature_Overrides = 1;
	        	primary_temp.temp_deg_c = 210;      // Was 100 
	        	mixing_temp.temp_deg_c = 300;
	        	secondary_temp.temp_deg_c = 400;    // Was 100
				//egt_temp.temp_deg_c = 600;
				//hex_inlet_temp.temp_deg_c = 100;
				//hex_outlet_temp.temp_deg_c = 100;                
				system_state = STATE_STARTUP;
				fault_code = 0;
				break;
			case('B'):
			case('b'):
				UART_1_PutString("\n\rB-Profile B");
                Temperature_Overrides = 1;
	        	primary_temp.temp_deg_c = 550; 	  // > 150 C
                secondary_temp.temp_deg_c = 450;  // New
                mixing_temp.temp_deg_c = 550;
				break;
			case('C'):
			case('c'):
				UART_1_PutString("\n\rC-Profile C");
                Temperature_Overrides = 1;
	        	primary_temp.temp_deg_c = 690; 	  // Was 410, > 400 C
	        	secondary_temp.temp_deg_c = 850; 	  // > 400 C
                mixing_temp.temp_deg_c = 1000;
				break;
			case('D'):
			case('d'):
				UART_1_PutString("\n\rD-Profile D");
                Temperature_Overrides = 1;
	        	primary_temp.temp_deg_c = 700; 	   // 600 C
	        	secondary_temp.temp_deg_c = 900;   // 700 C
                mixing_temp.temp_deg_c = 875;
				break;
			case('E'):
			case('e'):
				UART_1_PutString("\n\rE-Profile E");
                Temperature_Overrides = 1;
	        	primary_temp.temp_deg_c = 710; 	   // 710 C
	        	secondary_temp.temp_deg_c = 920;  // 700 C
				break;
			case('F'):
			case('f'):
				UART_1_PutString("\n\rF-Profile F");
                Temperature_Overrides = 1;
	        	primary_temp.temp_deg_c = 660; 	   // 660 C
	        	secondary_temp.temp_deg_c = 915;   // 915 C
                Temperature_Overrides = 0;
				break;
            case('U'):
            case('u'):
	        	UART_1_PutString("\n\rU-Quiescent");
                Temperature_Overrides = 1;
	        	primary_temp.temp_deg_c = 700; 	   // 700 C
	        	secondary_temp.temp_deg_c = 900;   // 900 C
				break;
            case('V'):
            case('v'):
	        	UART_1_PutString("\n\rV-Profile V");
                Temperature_Overrides = 1;
	        	primary_temp.temp_deg_c = 685; 	   // 700 C
	        	secondary_temp.temp_deg_c = 885;   // 860 C
				break;
            case('L'):
            case('l'):
	        	UART_1_PutString("\n\rL");
                Temperature_Overrides = 1;
	        	primary_temp.temp_deg_c = 750; 	   // 700 C
	        	secondary_temp.temp_deg_c = 1200;   // 860 C
				break;
			case('G'):
			case('g'):
				UART_1_PutString("\n\rG");
	        	system_state = STATE_STANDARD_SHUTDOWN;
                Temperature_Overrides = 0;
				break;
            case('P'):
            case('p'):
                UART_1_PutString("\n\rP");
                primary_catalyst.light_off_temp_required = 500;
                break;
			case('W'):
			case('w'):
				block_shutdown = 1;
				UART_1_PutString("\n\rW-Block Shutdown");
                Temperature_Overrides = 0;
				break;
			case('X'):
			case('x'):
				UART_1_PutString("\n\rX-E-Stop");
				block_shutdown = 0;
                Temperature_Overrides = 0;
				break;
			case('Y'):
			case('y'):
				UART_1_PutString("\n\rY");
				Temperature_Overrides = 0;
				break;
			case('Z'):
			case('z'):
				UART_1_PutString("\n\rZ-Restart SM");
				system_state = STATE_STARTUP;
				fault_code = 0;
				Init_Sensors();
				max_loop_time = 0;
				min_loop_time = 0xFFFFFFFF;
                Temperature_Overrides = 0;
				break;
			default:
	 //			UART_1_PutString(TypedChar);
				break;
			}
		}
}
//*****************************************************************************
//* Function:  print_status_page
//*
//* Usage: Transmit Debug Information
//*   
//* Description:  send the status details out the RS232 port
//*   
//*****************************************************************************
void print_status_page(void)
{
	char str[CHAR_ARRAY_SIZE]; 
    int a,b,c,d,e;

	// Also send a status message through the RS232 port
	// using smaller messages since compiler throws warning with too many parameters

	// print the system timer (milliseconds)
	sprintf(str,"\n\r\n\r%udms - State:",main_timer_ms);
	UART_1_PutString(str);

	// print the system state name
	switch(system_state)
	{
	case(STATE_UNKNOWN):  // should only get here at power up
	default:
		UART_1_PutString("UNKNOWN");
		break;
	case(STATE_STARTUP):
		UART_1_PutString("STARTUP");
		break;
	case(STATE_LIGHTOFF):
		UART_1_PutString("LIGHTOFF");
		break;
	case(STATE_STEADY_STATE):
		UART_1_PutString("STEADY_STATE");
		break;
	case(STATE_MANUAL_MODE):
		UART_1_PutString("MANUAL_MODE");
		break;
	case(STATE_STANDARD_SHUTDOWN):
		UART_1_PutString("STANDARD_SHUTDOWN");
		break;
	case(STATE_EMERGENCY_SHUTDOWN):
		UART_1_PutString("EMERGENCY_SHUTDOWN");
		break;
	}

	// print the fault status
	sprintf(str,", Last Fault:%d",fault_code); 
	UART_1_PutString(str);
	
	// print the temperatures
	/*#if 1
	sprintf(str,"\n\rPri:%d C, Sec:%d C, Mix:%d C, EGT:%d C, HexIn:%d C, HexOut:%d C, CJC: %d C", 
        (int)primary_temp.temp_deg_c,
        (int)secondary_temp.temp_deg_c,
        (int)mixing_temp.temp_deg_c,
		(int)egt_temp.temp_deg_c,
		(int)hex_inlet_temp.temp_deg_c,
		(int)hex_outlet_temp.temp_deg_c,
        (int)cold_junction_temp);
	UART_1_PutString(str);
	#else
	sprintf(str,"\n\rPri:%08lX->%d C, offset:%08lX %lduV, CJ:%ld C, Sec:%08lX->%d C", 
        (long)primary_temp.raw_adc_counts,(int)primary_temp.temp_deg_c,
        (long)primary_temp.offset_adc_counts,offset_microvolts,
        (long)cold_junction_temp,
        (long)secondary_temp.raw_adc_counts,(int)secondary_temp.temp_deg_c);
	UART_1_PutString(str);
	sprintf(str,"\n\rMix:%08lX->%d C, EGT:%08lX->%d C, HexIn:%08lX->%d C, HexOut:%08lX->%d C", 
        (long)mixing_temp.raw_adc_counts,(int)mixing_temp.temp_deg_c,
		(long)egt_temp.raw_adc_counts,(int)egt_temp.temp_deg_c,
		(long)hex_inlet_temp.raw_thm_adc_counts,(int)hex_inlet_temp.temp_deg_c,
		(long)hex_outlet_temp.raw_thm_adc_counts,(int)hex_outlet_temp.temp_deg_c);
	UART_1_PutString(str);
	#endif*/
	
	// print the commanded and actual RPMs 
	/*sprintf(str,"\n\rFan Speed Cmd->Act - PriFan:%d->%d/%d RPM, SecFan: %d->%d/%d RPM",
		pri_fan_speed[0].cmd_speed_rpm,pri_fan_speed[0].measured_speed_rpm,pri_fan_speed[1].measured_speed_rpm,
		sec_fan_speed[0].cmd_speed_rpm,sec_fan_speed[0].measured_speed_rpm,sec_fan_speed[1].measured_speed_rpm);
	UART_1_PutString(str);

	// print the currents
	sprintf(str,"\n\rCurrents - CP:%d mA, GP:%d mA, IJ:%d mA",
		(int)cp_current.current_mA,(int)gp_current.current_mA,(int)inj_current.current_mA);
	UART_1_PutString(str);

	// print the currents
	sprintf(str,"\n\rCurrents - PF:%d mA, FP:%d mA, SF:%d mA",
		(int)pf_current.current_mA,(int)pu_current.current_mA,(int)sf_current.current_mA);
	UART_1_PutString(str);

	// print the power outputs on (1)/off (0)
	sprintf(str,"\n\rOutputs - Cool:%d, Fuel:%d, Glow:%d, Inj:%d %%",
		(coolant_pump.status == PUMP_ON),(fuel_pump.status == PUMP_ON),(glow_plug.status == GLOW_PLUG_ON),injector.duty/10);
	UART_1_PutString(str);

	// print statuses of subsystems
	sprintf(str,"\n\rStatus - Hex:%d, Pri Air:%d, Sec Air:%d, Pri Cat:%d, Sec Cat:%d",
		heat_exchanger.status,primary_air_supply.status,secondary_air_supply.status,
		primary_catalyst.status,secondary_catalyst.status);
	UART_1_PutString(str);

	// print the main loop timing
	sprintf(str,"\n\rMain Loop Time - Last:%u, Min:%u, Max:%u ms", 
		last_loop_time, min_loop_time, max_loop_time);
	UART_1_PutString(str);

}*/

// print the temperatures
      #if 1
      sprintf(str,"\n\rPri:%d C, SecI:%d C, SecO:%d C, EGT:%d C, HexIn:%d C, HexOut:%d C, CJC: %d C", 
        (int)primary_temp.temp_deg_c,
        (int)secondary_temp.temp_deg_c,
        (int)mixing_temp.temp_deg_c,
        (int)egt_temp.temp_deg_c,
        (int)hex_inlet_temp.temp_deg_c,
        (int)hex_outlet_temp.temp_deg_c,
        (int)cold_junction_temp);
      UART_1_PutString(str);
      #else
      sprintf(str,"\n\rPri:%08lX->%d C, offset:%08lX %lduV, CJ:%ld C, Sec:%08lX->%d C", 
        (long)primary_temp.raw_adc_counts,(int)primary_temp.temp_deg_c,
        (long)primary_temp.offset_adc_counts,offset_microvolts,
        (long)cold_junction_temp,
        (long)secondary_temp.raw_adc_counts,(int)secondary_temp.temp_deg_c);
      UART_1_PutString(str);
      sprintf(str,"\n\rMix:%08lX->%d C, EGT:%08lX->%d C, HexIn:%08lX->%d C, HexOut:%08lX->%d C", 
        (long)mixing_temp.raw_adc_counts,(int)mixing_temp.temp_deg_c,
            (long)egt_temp.raw_adc_counts,(int)egt_temp.temp_deg_c,
            (long)hex_inlet_temp.raw_thm_adc_counts,(int)hex_inlet_temp.temp_deg_c,
            (long)hex_outlet_temp.raw_thm_adc_counts,(int)hex_outlet_temp.temp_deg_c);
      UART_1_PutString(str);
      #endif
    
   // print catalyst light-off temperature settings
    
   a = primary_catalyst.light_off_temp_required;
   b = primary_catalyst.light_off_time_limit;
   c = sec_temp_control;
   d = secondary_catalyst.light_off_temp_required;
   e = secondary_catalyst.light_off_time_limit;
    
      sprintf(str,"\n\rPC-LO:%d, PC-TO:%d, STC:%ld, SC-LO:%d, SC-TO:%d",
         (int)a,(int)b,(long)c,(int)d,(int)e);
       
      UART_1_PutString(str);
      
   // print the commanded and actual Fan Speeds 
      sprintf(str,"\n\rFan Speed Cmd->Act - PriFan:%d->%d/%d RPM, SecFan: %d->%d/%d RPM",
      pri_fan_speed[0].cmd_speed_rpm,pri_fan_speed[0].measured_speed_rpm,pri_fan_speed[1].measured_speed_rpm,
      sec_fan_speed[0].cmd_speed_rpm,sec_fan_speed[0].measured_speed_rpm,sec_fan_speed[1].measured_speed_rpm);
      UART_1_PutString(str);

   // print calculated Feed Forward Terms
      sprintf(str,"\n\rPID Correction: P_INC: %ld, P_DEC: %ld, Pn2: %d, S_INC: %ld, S_DEC: %ld, Sn2: %d",
         (long)P_INC,(long)P_DEC,(int)pn2,(long)S_INC,(long)S_DEC,(int)sn2);
      UART_1_PutString(str);
    
   // print the currents
   a = cp_current.current_mA;
   b = gp_current.current_mA;
   c = inj_current.current_mA;
  #if 1
      sprintf(str,"\n\rCurrents - Cool:%d mA, Glow:%d mA, Inj:%d ma",
         (int)a,(int)b,(int)c);
      UART_1_PutString(str);
   #else
    sprintf(str,"\n\rCurrents - Cool:%lX, Glow:%lX, Inj:%1X, PINC:%d, SINC:%d",
            (long)cp_current.raw_adc_counts,(long)gp_current.raw_adc_counts,(int)inj_current.raw_adc_counts,(int)d,(int)e);    
        
   #endif

   // print the currents
   a = pf_current.current_mA;
   b = pu_current.current_mA;
   c = sf_current.current_mA;

      sprintf(str,"\n\rCurrents - Pri:%d mA, Fuel:%d mA, Sec:%d mA",
         (int)a,(int)b,(int)c);
      UART_1_PutString(str);

      // print the power outputs on (1)/off (0)
      sprintf(str,"\n\rOutputs - Cool:%d, Fuel:%d, Glow:%d, Inj:%d %%",
         (coolant_pump.status == PUMP_ON),(fuel_pump.status == PUMP_ON),(glow_plug.status == GLOW_PLUG_ON),injector.duty/10);
      UART_1_PutString(str);

      // print statuses of subsystems
      /*sprintf(str,"\n\rStatus - Hex:%d, Pri Air:%d, Sec Air:%d, Pri Cat:%d, Sec Cat:%d",
            heat_exchanger.status,primary_air_supply.status,secondary_air_supply.status,
            primary_catalyst.status,secondary_catalyst.status);
      UART_1_PutString(str);*/

      // print the main loop timing
      sprintf(str,"\n\rMain Loop Time - Last:%ulms, Min:%ulms, Max:%ulms", 
            last_loop_time, min_loop_time, max_loop_time);
      UART_1_PutString(str);
}
    
//*****************************************************************************
//* Function:   Get_Thermocouple_1_Counts
//*
//* Parameters:
//*     sensor: pointer to the struct members for PO temperature sensor
//*   
//* Description:    Reads the ADC counts from Thermocouple 1
//*
//*****************************************************************************
void Get_Thermocouple_1_Counts(tc_temp_t * sensor) {
   AMux_1_FastSelect(sensor->mux);
   ADC_DelSig_1_SelectConfiguration(ADC_DelSig_1_ThermoCouple, 1);         // selects config; starts ADC conversion
   ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_WAIT_FOR_RESULT);             // loops until the conversion is done
   sensor->raw_adc_counts = ADC_DelSig_1_Read32( );                        //Read PO element counts (TC1)
   ADC_DelSig_1_StopConvert( );
}

//*****************************************************************************
//* Function:   Get_Primary_Temperature
//*
//* Parameters:
//*     sensor: pointer to the struct members for PO temperature sensor
//*   
//* Description:    Returns the PO element temperature
//*
//*****************************************************************************
float Get_Primary_Temperature(tc_temp_t * sensor){
    int32_t thermocouple_voltage;

	/* get ADC counts */
   cold_junction_temp = Measure_CJ_Temp();
	offset_microvolts = Primary_T_GetVoltage(((int32)cold_junction_temp * 2.65) * 100);	 // must send as 1/100 degree C per bit 
	ADC_DelSig_1_SetOffset(0); //wah  offset_microvolts);             // configures ADC conversion to adjust for offset

	Get_Thermocouple_1_Counts(sensor);	 // must be last conversion of the 2, or ADC_DelSig_1_CountsTo_uVolts will return for wrong configuration!
  
   /* Convert counts to microvolts */
   thermocouple_voltage = ADC_DelSig_1_CountsTo_uVolts(sensor->raw_adc_counts);
   thermocouple_voltage -= offset_microvolts;
	 
   /* Read and filter thermocouple temperature */
      sensor->temp_deg_c = Exponential_Filter(sensor->temp_deg_c,
         (float)(Primary_T_GetTemperature(thermocouple_voltage) / 100),
         TEMPERATURE_FILTER);
    
   return sensor->temp_deg_c;
}

//*****************************************************************************
//* Function:   Get_Thermocouple_3_Counts
//*
//* Parameters:
//*     sensor: pointer to the struct members for TO temperature sensor
//*   
//* Description:    Reads the ADC counts from Thermocouple 3
//*
//*****************************************************************************
void Get_Thermocouple_3_Counts(tc_temp_t * sensor) {
   AMux_1_FastSelect(sensor->mux);
   ADC_DelSig_1_SelectConfiguration(ADC_DelSig_1_ThermoCouple, 1);         // selects config; starts ADC conversion
//    ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_WAIT_FOR_RESULT);             // loops until the conversion is done
//    ADC_DelSig_1_StopConvert( );
   sensor->raw_adc_counts = ADC_DelSig_1_Read32( );                        //Read TO element counts (TC3)
}

//*****************************************************************************
//* Function:   Get_Secondary_Temperature
//*
//* Parameters:
//*     sensor: pointer to the struct members for TO temperature sensor
//*   
//* Description:    Returns the TO element temperature
//*
//*****************************************************************************
float Get_Secondary_Temperature(tc_temp_t * sensor) {
   int32_t thermocouple_voltage;
    
    /* Get ADC counts */
   Get_Thermocouple_3_Counts(sensor);
    
   /* Convert counts to microvolts */
// only retrieve in primary!    ADC_DelSig_1_SetOffset((int32_t)Get_Offset_Voltage( ));             // configures ADC conversion to adjust for offset
   thermocouple_voltage = ADC_DelSig_1_CountsTo_uVolts(sensor->raw_adc_counts);
   thermocouple_voltage -= offset_microvolts;
    
   /* Read and filter thermocouple temperature */
   sensor->temp_deg_c = Exponential_Filter(sensor->temp_deg_c,
      (Secondary_T_GetTemperature(thermocouple_voltage) / 100), 
         TEMPERATURE_FILTER);
    
   return sensor->temp_deg_c;
}

//*****************************************************************************
//* Function:   Get_Thermocouple_2_Counts
//*
//* Parameters:
//*     sensor: pointer to the struct members for mixing chamber temperature sensor
//*   
//* Description:    Reads the ADC counts from Thermocouple 2
//*
//*****************************************************************************
void Get_Thermocouple_2_Counts(tc_temp_t * sensor) {
   AMux_1_FastSelect(sensor->mux);
   ADC_DelSig_1_SelectConfiguration(ADC_DelSig_1_ThermoCouple, 1);         // selects config; starts ADC conversion
//    ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_WAIT_FOR_RESULT);             // loops until the conversion is done
//    ADC_DelSig_1_StopConvert( );
   sensor->raw_adc_counts = ADC_DelSig_1_Read32( );                        //Read mixing chamber counts (TC2)
}

//*****************************************************************************
//* Function:   Get_Mixing_Temperature
//*
//* Parameters:
//*     sensor: pointer to the struct members for mixing chamber temperature sensor
//*   
//* Description:    Returns the mixing chamber temperature
//*
//*****************************************************************************
float Get_Mixing_Temperature(tc_temp_t * sensor) {
   int32_t thermocouple_voltage;
    
   /* Get ADC counts */
   Get_Thermocouple_2_Counts(sensor);
    
   /* Convert counts to microvolts */
// only retrieve in primary!    ADC_DelSig_1_SetOffset((int32_t)Get_Offset_Voltage( ));             // configures ADC conversion to adjust for offset
   thermocouple_voltage = ADC_DelSig_1_CountsTo_uVolts(sensor->raw_adc_counts);
   thermocouple_voltage -= offset_microvolts;
    
   /* Read and filter thermocouple temperature */
   sensor->temp_deg_c = Exponential_Filter(sensor->temp_deg_c,
      (Mixing_T_GetTemperature(thermocouple_voltage) / 100), 
         TEMPERATURE_FILTER);
    
   return sensor->temp_deg_c;
}

//*****************************************************************************
//* Function:   Get_Thermocouple_4_Counts
//*
//* Parameters:
//*     sensor: pointer to the struct members for EGT temperature sensor
//*   
//* Description:    Reads the ADC counts from Thermocouple 4
//*
//*****************************************************************************
void Get_Thermocouple_4_Counts(tc_temp_t * sensor) {
   AMux_1_FastSelect(sensor->mux);
   ADC_DelSig_1_SelectConfiguration(ADC_DelSig_1_ThermoCouple, 1);         // selects config; starts ADC conversion
//    ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_WAIT_FOR_RESULT);             // loops until the conversion is done
//    ADC_DelSig_1_StopConvert( );
   sensor->raw_adc_counts = ADC_DelSig_1_Read32( );                        //Read EGT counts (TC4)
}

/*******************************************************************************
*  Function Name: Measure_CJ_Temp
********************************************************************************
* Summary:
*  This function is used to measure the DS600 IC temperature which is used as
*  cold junction compensation for thermocouple.
*
* Parameters:  
* void:
*
* Return: 
* int32: Temperature in °C
*
* Theory:
* Cold Junction IC DS600 has temperature coefficient of 6.45mV/°C with 
* 509mv offset at 25°C.
* Analog voltage output from IC is converted to temperature as follows:
* Temp = (Vout - 509mV)/6.45mV
*
* Side Effects:
* None
*******************************************************************************/
int32 Measure_CJ_Temp(void)
{
  	/* Cold junction IC ADC count measurement*/
   int32 ICResult;
   
   /* Cold Junction micro volts converted from ADC reading */
   int32 ICUVolts;
    
   /* Cold Jn temperature measured using DS600 */
   int32 ICTemp;
	
	/* Variable used to calculate VSSA ground reference voltage */
   //int32 offset;
    
   /* Read cold junction voltage */
   AMux_1_FastSelect(CJ_CHANNEL);
   ADC_DelSig_1_SelectConfiguration(ADC_DelSig_1_Others, 1);         // selects config; starts ADC conversion
   ADC_DelSig_1_StartConvert();
   ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_WAIT_FOR_RESULT);
   ADC_DelSig_1_StopConvert();
   ICResult = ADC_DelSig_1_GetResult32();
	
	/* Measure and subtract offset*/
	//offset = Get_Offset_Voltage();
	//ICResult -= offset;
	
	/* Convert counts to microvolts*/
   ICUVolts = ADC_DelSig_1_CountsTo_uVolts(ICResult);
    
   /* Measure cold junction temperature in Celsius from parameters from the DS600 
	* Temp Sensor 509 mV at 0 degrees C, divided by 6.45 mV per degree C. The macros
	* (SCALING, VOLTAGE_PER_DEGREE and ZERO_DEGREE_OFFSET_VOLTAGE have been 
	* manipulated such that the resultant temperature is in 100th of a degree Celsius*/
    
	ICTemp = ((ICUVolts - ZERO_DEGREE_OFFSET_VOLTAGE) * SCALING) / VOLTAGE_PER_DEGREE;

   return ICTemp;
}

//*****************************************************************************
//* Function:   Get_EGT_Temperature
//*
//* Parameters:
//*     sensor: pointer to the struct members for the EGT temperature sensor
//*   
//* Description:    Returns the exhaust gas temperature
//*
//*****************************************************************************
float Get_EGT_Temperature(tc_temp_t * sensor) {
   int32_t thermocouple_voltage;
    
   /* Get Adc counts */
	Get_Thermocouple_4_Counts(sensor);
    
   /* Convert counts to microvolts */
// only retrieve in primary!    ADC_DelSig_1_SetOffset((int32_t)Get_Offset_Voltage( ));             // configures ADC conversion to adjust for offset
   thermocouple_voltage = ADC_DelSig_1_CountsTo_uVolts(sensor->raw_adc_counts);
   thermocouple_voltage -= offset_microvolts;
    
   /* Read and filter thermocouple temperature */
   sensor->temp_deg_c = Exponential_Filter(sensor->temp_deg_c,
      (Exhaust_T_GetTemperature(thermocouple_voltage) / 100), 
         TEMPERATURE_FILTER);
    
   return sensor->temp_deg_c;
}

//*****************************************************************************
//* Function:   Get_HEX_In_Counts
//*
//* Parameters:
//*     sensor: pointer to the struct members for the HEX inlet temperature sensor
//*   
//* Description:    Reads the ADC counts from HEX/Thermistor IN
//*
//*****************************************************************************
void Get_HEX_In_Counts(hex_temp_t * sensor) {
   /* Read ADC across thermistor */
   // AMux_1_FastSelect(sensor->mux);
   ADC_DelSig_1_StopConvert( );
	AMux_1_FastSelect(sensor->mux);

   ADC_DelSig_1_SelectConfiguration(ADC_DelSig_1_Others, 1);           // selects config; starts ADC conversion
//    ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_WAIT_FOR_RESULT);         // loops until the conversion is done
//    ADC_DelSig_1_StopConvert( );
   sensor->raw_thm_adc_counts = ADC_DelSig_1_Read32( );
    
   /* Read ADC across reference resistor */
   AMux_1_FastSelect(VREF_MUX_CHANNEL);
   ADC_DelSig_1_StartConvert( );                                       // starts the conversion; no need to change config
//    ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_WAIT_FOR_RESULT); 
//    ADC_DelSig_1_StopConvert( );
   sensor->raw_ref_adc_counts = ADC_DelSig_1_Read32( );
}

//*****************************************************************************
//* Function:   Get_HEX_In_Temperature
//*
//* Parameters:
//*     sensor: pointer to the struct members for the HEX inlet temperature sensor
//*   
//* Description:    Returns the heat exchange inlet temperature
//*
//*****************************************************************************
float Get_HEX_In_Temperature(hex_temp_t * sensor){
//    int32_t thermistor_voltage;
//    int32_t ref_resistor_voltage;
    
   /* Get ADC counts */
   Get_HEX_In_Counts(sensor);
    
   /* Convert counts to microvolts */
//    ADC_DelSig_1_SetOffset((int32_t)Get_Offset_Voltage( ));             // configures ADC conversion to adjust for offset
//    thermistor_voltage = ADC_DelSig_1_CountsTo_uVolts(sensor->raw_thm_adc_counts);
//    ref_resistor_voltage = ADC_DelSig_1_CountsTo_uVolts(sensor->raw_ref_adc_counts);
        
   /* Read thermistor resistance */
   sensor->resistance = HEX_In_GetResistance(ADC_COUNTS_AT_OPEN_THERMISTOR_3_3V - sensor->raw_thm_adc_counts, sensor->raw_thm_adc_counts);
    
   /* Read thermistor temperature*/
   sensor->temp_deg_c = Exponential_Filter(sensor->temp_deg_c,
      (HEX_In_GetTemperature(sensor->resistance) / 100), 
         TEMPERATURE_FILTER);
    
   return sensor->temp_deg_c;
}

//*****************************************************************************
//* Function:   Get_HEX_Out_Counts
//*
//* Parameters:
//*     sensor: pointer to the struct members for the HEX outlet temperature sensor
//*   
//* Description:    Reads the ADC counts from HEX/Thermistor OUT
//*
//*****************************************************************************
void Get_HEX_Out_Counts(hex_temp_t * sensor) {
   /* Read ADC across thermistor */
   // AMux_1_FastSelect(sensor->mux);
	AMux_1_FastSelect(sensor->mux);
   ADC_DelSig_1_SelectConfiguration(ADC_DelSig_1_Others, 1);           // selects config; starts ADC conversion 
//    ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_WAIT_FOR_RESULT);         // loops until the conversion is done
//    ADC_DelSig_1_StopConvert( );
   sensor->raw_thm_adc_counts = ADC_DelSig_1_Read32( );
    
   /* Read ADC across reference resistor */
   AMux_1_FastSelect(OFFSET_CHANNEL);
   ADC_DelSig_1_StartConvert( );                                       // starts the conversion; no need to change config
//    ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_WAIT_FOR_RESULT); 
//    ADC_DelSig_1_StopConvert( );
   sensor->raw_ref_adc_counts = ADC_DelSig_1_Read32( );
}

//*****************************************************************************
//* Function:   Get_HEX_Out_Temperature
//*
//* Parameters:
//*     sensor: pointer to the struct members for the HEX outlet temperature sensor
//*   
//* Description:    Returns the heat exchange outlet temperature
//*
//*****************************************************************************
float Get_HEX_Out_Temperature(hex_temp_t * sensor){
//    int32_t thermistor_voltage;
//    int32_t ref_resistor_voltage;
    
   /* Get ADC counts */
   Get_HEX_Out_Counts(sensor);
    
//    /* Convert counts to microvolts */
//    ADC_DelSig_1_SetOffset((int32_t) Get_Offset_Voltage( ));             // configures ADC conversion to adjust for offset
//    thermistor_voltage = ADC_DelSig_1_CountsTo_uVolts(sensor->raw_thm_adc_counts);
//    ref_resistor_voltage = ADC_DelSig_1_CountsTo_uVolts(sensor->raw_ref_adc_counts);
    
   /* Read thermistor resistance */
   sensor->resistance = HEX_Out_GetResistance(ADC_COUNTS_AT_OPEN_THERMISTOR_3_3V - sensor->raw_thm_adc_counts, sensor->raw_thm_adc_counts);
    
   /* Read thermistor temperature*/
   sensor->temp_deg_c = Exponential_Filter(sensor->temp_deg_c,
      (HEX_Out_GetTemperature(sensor->resistance) / 100), 
         TEMPERATURE_FILTER);
    
   return sensor->temp_deg_c;
}

//*****************************************************************************
//* Function:   Get_Pri_Fan_IS_Counts
//*
//* Parameters:
//*     sensor: pointer to the struct members for the pri fan current sensor
//*   
//* Description:    Reads the ADC counts from pri fan current sensor
//*
//*****************************************************************************
void Get_Pri_Fan_IS_Counts(current_sense_t * sensor) {
   AMux_1_FastSelect(sensor->mux);
   ADC_DelSig_1_SelectConfiguration(ADC_DelSig_1_Others, 1);           // selects config; starts ADC conversion
//    ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_WAIT_FOR_RESULT);         // loops until the conversion is done
//    ADC_DelSig_1_StopConvert( );
   sensor->raw_adc_counts = ADC_DelSig_1_Read16( );
}

//*****************************************************************************
//* Function:   Get_Pri_Fan_IS
//*
//* Usage: 
//*   
//* Description:   Returns the current consumption feedback from the primary
//* fan array.
//*
//*****************************************************************************
float Get_Pri_Fan_IS(current_sense_t * sensor) {
   int16_t load_voltage;
    
    /* Get ADC counts */
	Get_Pri_Fan_IS_Counts(sensor);
    
    /* Convert counts to current */
// only retrieve in primary!    ADC_DelSig_1_SetOffset((int32_t)Get_Offset_Voltage( ));             // configures ADC conversion to adjust for offset
   load_voltage = ADC_DelSig_1_CountsTo_Volts(sensor->raw_adc_counts);
   sensor->current_mA = (load_voltage * 10); //RK - New algo from Pete
    
   /* TBD filter current result */
    
   return sensor->current_mA;
}

//*****************************************************************************
//* Function:   Get_Sec_Fan_IS_Counts
//*
//* Parameters:
//*     sensor: pointer to the struct members for the sec fan current sensor
//*   
//* Description:    Reads the ADC counts from sec fan current sensor
//*
//*****************************************************************************
void Get_Sec_Fan_IS_Counts(current_sense_t * sensor) {
   AMux_1_FastSelect(sensor->mux);
   ADC_DelSig_1_SelectConfiguration(ADC_DelSig_1_Others, 1);           // selects config; starts ADC conversion
//    ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_WAIT_FOR_RESULT);         // loops until the conversion is done
//    ADC_DelSig_1_StopConvert( );
   sensor->raw_adc_counts = ADC_DelSig_1_Read16( );
}

//*****************************************************************************
//* Function:   Get_Sec_Fan_IS
//*
//* Parameters:
//*     sensor: pointer to the struct members for the sec fan current sensor
//*   
//* Description:   Returns the current consumption feedback from the secondary
//* fan array.
//*
//*****************************************************************************
float Get_Sec_Fan_IS(current_sense_t * sensor) {
   int16_t load_voltage;
    
    /* Get ADC counts */
	Get_Sec_Fan_IS_Counts(sensor);
    
   /* Convert counts to current */
// only retrieve in primary!    ADC_DelSig_1_SetOffset((int32_t)Get_Offset_Voltage( ));             // configures ADC conversion to adjust for offset
   load_voltage = ADC_DelSig_1_CountsTo_Volts(sensor->raw_adc_counts);
   sensor->current_mA = ((load_voltage / IS_SET_RESISTANCE) * IS_RATIO) * 1000;
    
   /* TBD filter current result */
        
   return sensor->current_mA;
}

//*****************************************************************************
//* Function:   Get_CP_IS_Counts
//*
//* Parameters:
//*     sensor: pointer to the struct members for the coolant pump current sensor
//*   
//* Description:    Reads the ADC counts from coolant pump current sensor
//*
//*****************************************************************************
void Get_CP_IS_Counts(current_sense_t * sensor) {
   AMux_1_FastSelect(sensor->mux);
   ADC_DelSig_1_SelectConfiguration(ADC_DelSig_1_Others, 1);           // selects config; starts ADC conversion
//    ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_WAIT_FOR_RESULT);         // loops until the conversion is done
//    ADC_DelSig_1_StopConvert( );
   sensor->raw_adc_counts = ADC_DelSig_1_Read16( );
}

//*****************************************************************************
//* Function:   Get_Coolant_Pump_IS
//*
//* Parameters:
//*     sensor: pointer to the struct members for the coolant pump current sensor
//*   
//* Description:   Returns the current consumption feedback from the coolant
//* pump.
//*
//*****************************************************************************
float Get_Coolant_Pump_IS(current_sense_t * sensor) {
    int16_t load_voltage;
    
   /* Get ADC counts */
	Get_CP_IS_Counts(sensor);
    
   /* Convert counts to current */
// only retrieve in primary!    ADC_DelSig_1_SetOffset((int32_t)Get_Offset_Voltage( ));             // configures ADC conversion to adjust for offset
   load_voltage = ADC_DelSig_1_CountsTo_Volts(sensor->raw_adc_counts);
   sensor->current_mA = ((load_voltage / IS_SET_RESISTANCE) * IS_RATIO) * 1000;
    
   /* TBD filter current result */
        
   return sensor->current_mA;
}

//*****************************************************************************
//* Function:   Get_GP_IS_Counts
//*
//* Parameters:
//*     sensor: pointer to the struct members for the glow plug current sensor
//*   
//* Description:    Reads the ADC counts from glow plug current sensor
//*
//*****************************************************************************
void Get_GP_IS_Counts(current_sense_t * sensor) {
   AMux_1_FastSelect(sensor->mux);
   ADC_DelSig_1_SelectConfiguration(ADC_DelSig_1_Others, 1);           // selects config; starts ADC conversion
//    ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_WAIT_FOR_RESULT);         // loops until the conversion is done
//    ADC_DelSig_1_StopConvert( );
   sensor->raw_adc_counts = ADC_DelSig_1_Read16( );
}

//*****************************************************************************
//* Function:   Get_Glow_Plug_IS
//*
//* Parameters:
//*     sensor: pointer to the struct members for the glow plug current sensor
//*   
//* Description:   Returns the current consumption feedback from the glow plug.
//*
//*****************************************************************************
float Get_Glow_Plug_IS(current_sense_t * sensor) {
   int16_t load_voltage;
    
   /* Get ADC counts */
	Get_GP_IS_Counts(sensor);
    
   /* Convert counts to current */
// only retrieve in primary!    ADC_DelSig_1_SetOffset((int32_t)Get_Offset_Voltage( ));             // configures ADC conversion to adjust for offset
   load_voltage = ADC_DelSig_1_CountsTo_Volts(sensor->raw_adc_counts);
   sensor->current_mA = ((load_voltage / IS_SET_RESISTANCE) * IS_RATIO) * 1000;
    
   /* TBD filter current result */
        
   return sensor->current_mA;
}

//*****************************************************************************
//* Function:   Get_Inj_IS_Counts
//*
//* Parameters:
//*     sensor: pointer to the struct members for the injector current sensor
//*   
//* Description:    Reads the ADC counts from injector current sensor
//*
//*****************************************************************************
void Get_Inj_IS_Counts(current_sense_t * sensor) {
   AMux_1_FastSelect(sensor->mux);
   ADC_DelSig_1_SelectConfiguration(ADC_DelSig_1_Others, 1);           // selects config; starts ADC conversion
//    ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_WAIT_FOR_RESULT);         // loops until the conversion is done
//    ADC_DelSig_1_StopConvert( );
   sensor->raw_adc_counts = ADC_DelSig_1_Read16( );
}

//*****************************************************************************
//* Function:   Get_Injector_IS
//*
//* Parameters:
//*     sensor: pointer to the struct members for the injector current sensor
//*   
//* Description:   Returns the current consumption feedback from the injector.
//*
//*****************************************************************************
float Get_Injector_IS(current_sense_t * sensor) {
   int16_t load_voltage;
    
   /* Get ADC counts */
	Get_Inj_IS_Counts(sensor);
    
   /* Convert counts to current */
// only retrieve in primary!    ADC_DelSig_1_SetOffset((int32_t)Get_Offset_Voltage( ));             // configures ADC conversion to adjust for offset
   load_voltage = ADC_DelSig_1_CountsTo_Volts(sensor->raw_adc_counts);
   sensor->current_mA = ((load_voltage / IS_SET_RESISTANCE) * IS_RATIO) * 1000;
    
   /* TBD filter current result */
        
   return sensor->current_mA;
}

//*****************************************************************************
//* Function:   Get_PU_IS_Counts
//*
//* Parameters:
//*     sensor: pointer to the struct members for the fuel pump current sensor
//*   
//* Description:    Reads the ADC counts from fuel pump current sensor
//*
//*****************************************************************************
void Get_PU_IS_Counts(current_sense_t * sensor) {
   AMux_1_FastSelect(sensor->mux);
   ADC_DelSig_1_SelectConfiguration(ADC_DelSig_1_Others, 1);           // selects config; starts ADC conversion
//    ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_WAIT_FOR_RESULT);         // loops until the conversion is done
//    ADC_DelSig_1_StopConvert( );
   sensor->raw_adc_counts = ADC_DelSig_1_Read16( );
}

//*****************************************************************************
//* Function:   Get_Fuel_Pump_IS
//*
//* Parameters:
//*     sensor: pointer to the struct members for the fuel pump current sensor
//*   
//* Description:   Returns the current consumption feedback from the fuel pump.
//*
//*****************************************************************************
float Get_Fuel_Pump_IS(current_sense_t * sensor) {
   int16_t load_voltage;
    
   /* Get ADC counts */
	Get_PU_IS_Counts(sensor);
    
   /* Convert counts to current */
// only retrieve in primary!    ADC_DelSig_1_SetOffset((int32_t)Get_Offset_Voltage( ));             // configures ADC conversion to adjust for offset
   load_voltage = ADC_DelSig_1_CountsTo_Volts(sensor->raw_adc_counts);
   sensor->current_mA = ((load_voltage / IS_SET_RESISTANCE) * IS_RATIO) * 1000;
    
   /* TBD filter current result */
       
   return sensor->current_mA;
}

//*****************************************************************************
//* Function:   Get_Offset_Voltage
//*
//* Parameters: none
//*   
//* Description:   Returns the Vssa(GND) ref value
//*
//*****************************************************************************
int32_t Get_Offset_Voltage(void) {
   int32_t result;
    
   AMux_1_FastSelect(OFFSET_CHANNEL); 
   ADC_DelSig_1_StartConvert( );
   ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_WAIT_FOR_RESULT);         // loops until the conversion is done
   result = ADC_DelSig_1_Read32( );
   ADC_DelSig_1_StopConvert( );

   primary_temp.offset_adc_counts = result;
    
   return result;
}

//*****************************************************************************
//* Function:   Check_Primary_Oxidizer
//*
//* Usage: 
//*   
//* Description:    
//*
//*****************************************************************************
void Check_Primary_Oxidizer(tc_temp_t * sensor) {
   static uint32_t timer_ms;
   uint16 j;

//    static uint16_t last_pri_rpm;
   j = ((sensor->temp_deg_c)/10);

   // Calculate Primary Additive Delta for PID loop & Set Boundaries
   P_INC = (PRIMARY_CATALYST_SS_TARGET - sensor->temp_deg_c); // Primary Catalyst is too cold
   P_DEC = (sensor->temp_deg_c - PRIMARY_CATALYST_SS_TARGET); // Primary Catalyst is too cold
   
   // Calculate single term for Positive PID correction - multiplier is the K-factor
   if (P_INC <= 0){  // Additive Value Min.
       P_INC  = 0;
      }
      else if (P_INC >= 500){   // Additive Value Max.
         P_INC  = 500;
         }
   if (P_INC >= 140){
       P_INC *= P_KFI;
      }
   // Calculate single term for Negative PID correction - multiplier is the K-factor
   if (P_DEC <= 0){  // Subtractive Value Min.
       P_DEC  = 0;
      }
      else if (P_DEC >= 500){   //Subtractive Value Max.
            P_DEC  = 500;
         }
   if ((P_DEC >= 200) && (sensor->temp_deg_c <= 550)){ // This brackets the agressive response so as not to affect outside of bracket 
        P_DEC *= P_KFD;
      }
         
   static const uint32_t pn2_tab[100] = {
      //                10       20       30       40       50       60       70       80       90    [Deg. C] Original RK 11Feb22
   //   /*000*/  10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,
   //   /*100*/  10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,
   //   /*200*/  10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,
   //   /*300*/  10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,
   //   /*400*/  10200,   10400,   10600,   10800,   11000,   11200,   11400,   11600,   11800,   12000,
   //   /*500*/  12100,   12200,   12300,   12400,   12500,   12600,   12700,   12800,   12900,   13000,
   //   /*600*/  13100,   13200,   13300,   13400,   13600,   13800,   14000,   14200,   14400,   14500,
   //   /*700*/  14550,   14500,   14450,   14400,   14350,   14300,   14250,   14200,   14150,   14100,
   //   /*800*/  14000,   13500,   13000,   12500,   12000,   11500,   11000,   10500,   10000,   10000,
   //   /*900*/  10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000
      
       //                10       20       30       40       50       60       70       80       90    [Deg. C] Modified RK 21Feb22
      /*000*/  10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,
      /*100*/  10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,
      /*200*/  10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,
      /*300*/  10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,
      /*400*/  10200,   10400,   10600,   10800,   11000,   11200,   11400,   11600,   11800,   12000,
      /*500*/  12600,   12800,   13000,   13200,   13500,   13500,   13500,   13500,   13500,   13500,
      /*600*/  13600,   13700,   13800,   13900,   14000,   14100,   14200,   14300,   14400,   14500,
      /*700*/  14500,   14450,   14400,   14350,   14300,   14250,   14200,   14200,   14200,   14200,
      /*800*/  14000,   13500,   13000,   12500,   12000,   11500,   11000,   10500,   10000,   10000,
      /*900*/  10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000
      };
      pn2 = pn2_tab[j];
      
   // E-Stop if Primary CAT is in Danger of Melting 
   if (sensor->temp_deg_c >= PRIMARY_CATALYST_OT){
      UART_1_PutString("\n\n\t\r*PRI. CAT OVER TEMP!*");
      block_shutdown = 0;
      system_state = STATE_EMERGENCY_SHUTDOWN;    
      }
   
	// Primary temperature is good (695-705 C) Deadband
   if ((sensor->temp_deg_c >= PRIMARY_CATALYST_SS_MIN) && 
      (sensor->temp_deg_c <= PRIMARY_CATALYST_SS_MAX)) {
       timer_ms = main_timer_ms;                                                          // maintain existing fan and fuel flows
      // Has the commanded fan speed been reached? 
         for (i = 0; i < 20; i++) {
            if ((pri_fan_speed[0].measured_speed_rpm < (pri_fan_speed[0].cmd_speed_rpm + 50)) && // Actual speed less than commanded speed +50 &&
               (pri_fan_speed[0].measured_speed_rpm > (pri_fan_speed[0].cmd_speed_rpm - 50))){  // Actual speed greater than command speed -50 
                primary_air_supply.error_count++;                          // if yes, increment error count
               }
            else {
               primary_air_supply.error_count = 0; 
               primary_air_supply.status = GOOD;                   // no faults
            }
            // require x readings in a row to fail
            if (primary_air_supply.error_count >= FAULT_COUNT) {
                primary_air_supply.error_count = FAULT_COUNT;   // prevent wrap
                primary_air_supply.status = FAULT;              // flag the fault
            }
         }     
      }
      
      // Primary is too cold
         if ((sensor->temp_deg_c >= PRIMARY_CATALYST_HOT) && (sensor->temp_deg_c <= PRIMARY_CATALYST_SS_MIN)) {
            if ((main_timer_ms - timer_ms) >= STEADYSTATE_TIME_LIMIT_MS) {
               if (sensor->status == BAD) {       //check if sensor is bad
                // TBD Set Master Fault here
                // Standard_Shutdown( );
               }
            }
    	      for (i = 0; i < 20; i++) {			                            // increase fan speed
               if (pri_fan_speed[0].measured_speed_rpm + P_INC < FAN_SPEED_MAXIMUM_RPM) {    // verify speed command is less than max. speed
                   pri_fan_speed[0].cmd_speed_rpm = (P_INC + pn2);                             // if yes, keep increasing 
               }
               else {
                     pri_fan_speed[0].cmd_speed_rpm = FAN_SPEED_MAXIMUM_RPM;                             // if no, command maximum speed
               }
               pri_fan_speed[0].cmd_speed_rpm = (pn2 + P_INC);
            }
         }
              
         // Primary is too hot
            else if ((sensor->temp_deg_c >= PRIMARY_CATALYST_SS_MAX) && (sensor->temp_deg_c < PRIMARY_CATALYST_OT)) {
               if ((main_timer_ms - timer_ms) >= STEADYSTATE_TIME_LIMIT_MS) {
                  if (sensor->status == BAD) {       //check if sensor is bad
                  // TBD Set Master Fault here
                  // Standard_Shutdown( );
                  }
               }
    	         for (i = 0; i < 20; i++) {			                            // increase fan speed
                  if (pri_fan_speed[0].measured_speed_rpm - P_DEC < pn2) {    // verify speed command is less than max. speed
                      pri_fan_speed[0].cmd_speed_rpm = (pn2 - P_DEC);                             // if yes, keep decreasing 
                  }
                  else {
                     pri_fan_speed[0].cmd_speed_rpm = FAN_STARTUP_SPEED_PRI;                             // if no, command maximum speed
                  }
                  pri_fan_speed[0].cmd_speed_rpm = (pn2 - P_DEC);
               }
            Pri_Fan_Array_SetDesiredSpeed (0, pri_fan_speed[0].cmd_speed_rpm);    // Write speed request to controller
         }
        
		// decrease fuel flow
         if (injector.duty - FAN_SPEED_ADJUSTMENT < FUEL_DUTY_MIN_ADJUSTED) {
    		injector.duty = FUEL_DUTY_MIN_ADJUSTED; // increase fuel flow rate 
         }
            else {
    		   injector.duty -= FUEL_DUTY_ADJUSTMENT; // decrease fuel flow rate 
            }
               
		// increase fuel flow
        if (injector.duty + FAN_SPEED_ADJUSTMENT > FUEL_DUTY_MAX_ADJUSTED) {
    		injector.duty = FUEL_DUTY_MAX_ADJUSTED; // increase fuel flow rate 
         }
            else {
    		   injector.duty += FUEL_DUTY_ADJUSTMENT; // increase fuel flow rate 
            }
      return; 
   }      
   
 
//*****************************************************************************
//* Function:   Check_Secondary_Oxidizer
//*
//* Usage: 
//*   
//* Description:    
//*
//*****************************************************************************
void Check_Secondary_Oxidizer(void) {
   static uint32_t timer_ms;
   uint16 n;
   uint16 i;
   
   n = (sec_temp_control/10);
   //static uint16_t last_sec_rpm;                                      
  
   // Calculate Secondary Additive Deltas for PID loop & Set Boundaries
   S_INC = (sec_temp_control - SECONDARY_CATALYST_SS_TARGET); // Primary Catalyst is too hot 
   S_DEC = (SECONDARY_CATALYST_SS_TARGET - sec_temp_control); // Primary Catalyst is too cold                                                                
   
   // Calculate term for Positive PID correction - S_KFI is a multiplier
   if (S_INC >= 300){         // Additive Value Max.
       S_INC  = 200;
      }
      else if (S_INC <= 0){   // Additive Value Min.
         S_INC  = 0;
         }
   if (S_INC >= 35 ){
       S_INC *= S_KFI;
      }
   // Calculate term for Negative PID correction - S_KFD is a multiplier
   if (S_DEC >= 300){         // Subtractive Value Max.
       S_DEC  = 200;
      }
      else if (S_DEC <= 0){   // Subtractive Value Min.
         S_DEC  = 0;
         }
    
   // Look-Up Table for Secondary Air Speed Setpoint  
      {      
      static const uint32_t sn2_tab[100] = {
      //                   10       20       30       40       50       60       70       80       90    [Deg. C] Modified RK 21Feb22
      /*000*/   8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,
      /*100*/   8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,
      /*200*/   8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,
      /*300*/  11500,   11500,   11500,   11500,   11500,   11500,   11500,   11500,   11500,   11500,
      /*400*/  11500,   11500,   11500,   11500,   11500,   11500,   11500,   11500,   11500,   11500,
      /*500*/  11500,   11500,   11500,   11500,   11500,   11500,   11500,   11500,   11500,   11500,
      /*600*/  11500,   11500,   11500,   11500,   11500,   11500,   11500,   11500,   11500,   11500,
      /*700*/  11600,   11700,   11800,   11900,   12000,   12000,   12000,   12000,   12000,   12000,
      /*800*/  12100,   12200,   12300,   12400,   12500,   12600,   12700,   12800,   12900,   13000,
      /*900*/  13000,   13200,   13400,   13600,   13800,   14000,   14200,   14400,   14600,   14800
      
   //                        10       20       30       40       50       60       70       80       90    [Deg. C] Step Function RK 11Feb22
   //   /*000*/   8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,
   //   /*100*/   8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,
   //   /*200*/   8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,    8000,
   //   /*300*/  11500,   11500,   11500,   11500,   11500,   11500,   11500,   11500,   11500,   11500,
   //   /*400*/  11500,   11500,   11600,   11800,   12000,   12200,   12400,   12600,   12800,   13000,
   //   /*500*/  13000,   13000,   13000,   13000,   13000,   13000,   13000,   13000,   13000,   13000,
   //   /*600*/  13000,   13000,   13000,   13000,   13000,   13000,   13000,   13000,   13000,   13000,
   //   /*700*/  13000,   13000,   13000,   13000,   13000,   13000,   13000,   13000,   13000,   13000,
   //   /*800*/  13000,   13000,   13000,   13000,   13000,   13000,   13000,   13000,   13000,   13000,
   //   /*900*/  13000,   13000,   13200,   13400,   13600,   13800,   14000,   14200,   14400,   14600
         };
         sn2 = sn2_tab[n];
         sec_fan_speed[0].cmd_speed_rpm = sn2;
      }
      
   // E-Stop if Secondary CAT is in Danger of Melting 
   if (sec_temp_control >= SECONDARY_CATALYST_OT){
      UART_1_PutString("\n\n\t\r*SEC. CAT OVER TEMP!*");
      block_shutdown = 0;
      system_state = STATE_EMERGENCY_SHUTDOWN;
      }   
 
   if ((sec_temp_control >= SECONDARY_CATALYST_HOT) && (sec_temp_control <= SECONDARY_CATALYST_PRIME)) { // This brackets the agressive response so as not to affect outside of bracket
      S_DEC *= S_KFD;
      }
   
	// Secondary is fine
   if ((sec_temp_control > SECONDARY_CATALYST_SS_MIN) &&
      (sec_temp_control < SECONDARY_CATALYST_SS_MAX)) {
       timer_ms = main_timer_ms;                                       //maintain existing fan and fuel flows
      
		// Has the commanded fan speed been reached? 
        for (i = 0; i < 20; i++) {
            if (sec_fan_speed[0].measured_speed_rpm < (sec_fan_speed[0].cmd_speed_rpm)) {// Actual speed less than commanded speed +50 &&
               secondary_air_supply.error_count++;                          // if yes, increment error count
            }   
            else {
                secondary_air_supply.error_count = 0;
                secondary_air_supply.status = GOOD;
            }
            // require x readings in a row to fail
            if (secondary_air_supply.error_count >= FAULT_COUNT) {
                secondary_air_supply.error_count = FAULT_COUNT;     // prevent wrap
                secondary_air_supply.status = FAULT;                // flag the fault
            }
         }         
      }

      // Secondary is too cold 
      else if ((sec_temp_control >= SECONDARY_CATALYST_WARM) && (sec_temp_control < SECONDARY_CATALYST_SS_MAX)) {
         /*if (main_timer_ms - timer_ms >= STEADYSTATE_TIME_LIMIT_MS) {
            if (sensor->status == BAD) {           //check if sensor is bad
                  // TBD Set Master Fault here
                  // Standard Shutdown( );
               }*/
               //decrease fan rpm
               for (i = 0; i < S_DEC; i++) {
                  if ((sec_fan_speed[0].cmd_speed_rpm - (S_DEC) < FAN_IDLE_SPEED_SEC)) {
                      (sec_fan_speed[0].cmd_speed_rpm = (sn2 - i));
                     }
                     else {
                        sec_fan_speed[0].cmd_speed_rpm = FAN_IDLE_SPEED_SEC;
                     }               
                     Sec_Fan_Array_SetDesiredSpeed (0, sec_fan_speed[0].cmd_speed_rpm + (sn2 - i));    // Write speed request to controller
                     //Sec_Fan_Array_SetDesiredSpeed(0,sec_fan_speed[0].cmd_speed_rpm);
                  }
               //}
            }
   
	   // Secondary is too hot 
      else if ((sec_temp_control > SECONDARY_CATALYST_SS_MAX) && (sec_temp_control < SECONDARY_CATALYST_OT)){
         /*if (main_timer_ms - timer_ms <= STEADYSTATE_TIME_LIMIT_MS) {
            if (sensor->status == BAD) {           //check if sensor is bad
               // TBD Set Master Fault here
               // Standard Shutdown( );
               }*/
               for (i = 0; i < S_INC; i++) {
                  if ((sec_fan_speed[0].cmd_speed_rpm + (sn2 + S_INC) < FAN_SPEED_MAXIMUM_RPM)) {
                      (sec_fan_speed[0].cmd_speed_rpm = (sn2 + i));
                     }
                     else {
                        sec_fan_speed[0].cmd_speed_rpm = FAN_SPEED_MAXIMUM_RPM;   
                     }
                     Sec_Fan_Array_SetDesiredSpeed (0, sec_fan_speed[0].cmd_speed_rpm + (sn2 + i));    // Write speed request to controller
                  } 
               //}
            }
         
		// increase fuel flow
         if (injector.duty + FAN_SPEED_ADJUSTMENT > FUEL_DUTY_MAX_ADJUSTED) {
    		    injector.duty = FUEL_DUTY_MAX_ADJUSTED; // increase fuel flow rate 
            }
            else {
    		   injector.duty += FUEL_DUTY_ADJUSTMENT; // increase fuel flow rate 
            }
      // decrease fuel flow
         if (injector.duty - FAN_SPEED_ADJUSTMENT < FUEL_DUTY_MIN_ADJUSTED) {
    		    injector.duty = FUEL_DUTY_MIN_ADJUSTED; // increase fuel flow rate 
            }
            else {
    		   injector.duty -= FUEL_DUTY_ADJUSTMENT; // decrease fuel flow rate 
            }
         return;
   }
//}
//*****************************************************************************
//* Function:  Check_Thermocouple_1
//*.
//* Parameters:
//*     sensor: pointer to the struct members for PO temperature sensor
//*   
//* Description:
//*
//*****************************************************************************
void Check_Thermocouple_1(tc_temp_t * sensor) {
	// is user forcing us to ignore this sensor?
	// or has the CPU just powered up? (grace period)
//	if(main_timer_ms < STARTUP_GRACE_MS)
//		{
//		sensor->error_count = 0;
//		sensor->status = GOOD;		 // force it as 'good'
//		}
    // read adc counts
    Get_Thermocouple_1_Counts(sensor);
    
	// is sensor out of the valid range?
	if((sensor->raw_adc_counts < sensor->error_low_threshold) || 
			(sensor->raw_adc_counts > sensor->error_high_threshold))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;  // prevent wrap!
			if(sensor->status != BAD)
				{
				sensor->status = BAD;	 // mark it as bad
				print_debug("\n\rTC1 Bad");
				}
			}
		}
	// is sensor into fault range?
	else if((sensor->fault_threshold != 0) && 
			(sensor->raw_adc_counts > sensor->fault_threshold) && 
			(sensor->status != BAD))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;	// prevent wrap!
			if(sensor->status != FAULT)
				{
				sensor->status = FAULT;  // flag the fault
				print_debug("\n\rTC1 Fault");
				}
			}
		}
	// recovery from some bad status counts, but not prealarms or faults 
	else if((sensor->status == UNKNOWN) ||
			(sensor->status == GOOD))
		{
		sensor->error_count = 0;
		sensor->status = GOOD;		 // continue as good
		}
}

//*****************************************************************************
//* Function:  Check_Thermocouple_2
//*
//* Parameters:
//*     sensor: pointer to the struct members for mix chamber temperature sensor
//*   
//* Description:
//*
//*****************************************************************************
void Check_Thermocouple_2(tc_temp_t * sensor) {
	// is user forcing us to ignore this sensor?
	// or has the CPU just powered up? (grace period)
//	if(main_timer_ms < STARTUP_GRACE_MS)
//		{
//		sensor->error_count = 0;
//		sensor->status = GOOD;		 // force it as 'good'
//		}
    // read adc counts
    Get_Thermocouple_2_Counts(sensor);
    
	// is sensor out of the valid range?
	if((sensor->raw_adc_counts < sensor->error_low_threshold) || 
			(sensor->raw_adc_counts > sensor->error_high_threshold))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;  // prevent wrap!
			if(sensor->status != BAD)
				{
				sensor->status = BAD;	 // mark it as bad
				print_debug("\n\rTC2 Bad");
				}
			}
		}
	// is sensor into fault range?
	else if((sensor->fault_threshold != 0) && 
			(sensor->raw_adc_counts > sensor->fault_threshold) && 
			(sensor->status != BAD))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;	// prevent wrap!
			if(sensor->status != FAULT)
				{
				sensor->status = FAULT;  // flag the fault
				print_debug("\n\rTC2 Fault");
				}
			}
		}
	// recovery from some bad status counts, but not prealarms or faults 
	else if((sensor->status == UNKNOWN) ||
			(sensor->status == GOOD))
		{
		sensor->error_count = 0;
		sensor->status = GOOD;		 // continue as good
		}
   }

//*****************************************************************************
//* Function:  Check_Thermocouple_3
//*
//* Parameters:
//*     sensor: pointer to the struct members for TO temperature sensor
//*   
//* Description:
//*
//*****************************************************************************
void Check_Thermocouple_3(tc_temp_t * sensor) {
	// is user forcing us to ignore this sensor?
	// or has the CPU just powered up? (grace period)
//	if(main_timer_ms < STARTUP_GRACE_MS)
//		{
//		sensor->error_count = 0;
//		sensor->status = GOOD;		 // force it as 'good'
//		}
    // read adc counts
    Get_Thermocouple_3_Counts(sensor);
    
	// is sensor out of the valid range?
	if((sensor->raw_adc_counts < sensor->error_low_threshold) || 
			(sensor->raw_adc_counts > sensor->error_high_threshold))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;  // prevent wrap!
			if(sensor->status != BAD)
				{
				sensor->status = BAD;	 // mark it as bad
				print_debug("\n\rTC3 Bad");
				}
			}
		}
	// is sensor into fault range?
	else if((sensor->fault_threshold != 0) && 
			(sensor->raw_adc_counts > sensor->fault_threshold) && 
			(sensor->status != BAD))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;	// prevent wrap!
			if(sensor->status != BAD)
				{
				sensor->status = BAD;  // flag the fault
				print_debug("\n\rTC3 BAD");
				}
			}
		}
	// recovery from some bad status counts, but not prealarms or faults 
	else if((sensor->status == UNKNOWN) ||
			(sensor->status == GOOD))
		{
		sensor->error_count = 0;
		sensor->status = GOOD;		 // continue as good
		}
}

//*****************************************************************************
//* Function:  Check_Thermocouple_4
//*
//* Parameters:
//*     sensor: pointer to the struct members for EGT temperature sensor
//*   
//* Description:
//*
//*****************************************************************************
void Check_Thermocouple_4(tc_temp_t * sensor) {
	// is user forcing us to ignore this sensor?
	// or has the CPU just powered up? (grace period)
//	if(main_timer_ms < STARTUP_GRACE_MS)
//		{
//		sensor->error_count = 0;
//		sensor->status = GOOD;		 // force it as 'good'
//		}
    // read adc counts
    Get_Thermocouple_4_Counts(sensor);
    
	// is sensor out of the valid range?
	if((sensor->raw_adc_counts < sensor->error_low_threshold) || 
			(sensor->raw_adc_counts > sensor->error_high_threshold))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;  // prevent wrap!
			if(sensor->status != BAD)
				{
				sensor->status = BAD;	 // mark it as bad
				print_debug("\n\rTC4 Bad");
				}
			}
		}
	// is sensor into fault range?
	else if((sensor->fault_threshold != 0) && 
			(sensor->raw_adc_counts > sensor->fault_threshold) && 
			(sensor->status != BAD))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;	// prevent wrap!
			if(sensor->status != FAULT)
				{
				sensor->status = FAULT;	 // mark it as bad
				print_debug("\n\rTC4 Fault");
				}
			}
		}
	// recovery from some bad status counts, but not prealarms or faults 
	else if((sensor->status == UNKNOWN) ||
			(sensor->status == GOOD))
		{
		sensor->error_count = 0;
		sensor->status = GOOD;		 // continue as good
		}
   }

//*****************************************************************************
//* Function:  Check_Thermistor_In
//*
//* Parameters:
//*     sensor: pointer to the struct members for HEX inlet temperature sensor
//*   
//* Description:
//*
//*****************************************************************************
void Check_Thermistor_In(hex_temp_t * sensor) {
    // read adc counts
    Get_HEX_In_Counts(sensor);
    
    /* Verify thermistor count value */
	// is sensor out of the valid range?
	if((sensor->raw_thm_adc_counts < sensor->thm_error_low_threshold) || 
			(sensor->raw_thm_adc_counts > sensor->thm_error_high_threshold))
		{
		// require x readings in a row to fail
		sensor->thm_error_count++;
		if(sensor->thm_error_count >= FAULT_COUNT)
			{
			sensor->thm_error_count = FAULT_COUNT;  // prevent wrap!
			if(sensor->thm_status != BAD)
				{
				sensor->thm_status = BAD;	 // mark it as bad
				print_debug("\n\rThrmIn Bad");
				}
			}
		}
	// is sensor into fault range?
	else if((sensor->thm_fault_threshold != 0) && 
			(sensor->raw_thm_adc_counts > sensor->thm_fault_threshold) && 
			(sensor->thm_status != BAD))
		{
		// require x readings in a row to fail
		sensor->thm_error_count++;
		if(sensor->thm_error_count >= FAULT_COUNT)
			{
			sensor->thm_error_count = FAULT_COUNT;	// prevent wrap!
			if(sensor->thm_status != FAULT)
				{
				sensor->thm_status = FAULT;  // flag the fault
				print_debug("\n\rThrmIn Fault");
				}
			}
		}
	// recovery from some bad status counts, but not prealarms or faults 
	else if((sensor->thm_status == UNKNOWN) ||
			(sensor->thm_status == GOOD))
		{
		sensor->thm_error_count = 0;
		sensor->thm_status = GOOD;		 // continue as good
		}
        
    /* Verify reference resistor count value */
	// is sensor out of the valid range?
	if((sensor->raw_ref_adc_counts < sensor->ref_error_low_threshold) || 
			(sensor->raw_ref_adc_counts > sensor->ref_error_high_threshold))
		{
		// require x readings in a row to fail
		sensor->ref_error_count++;
		if(sensor->ref_error_count >= FAULT_COUNT)
			{
			sensor->ref_error_count = FAULT_COUNT;  // prevent wrap!
			if(sensor->ref_status != BAD)
				{
				sensor->ref_status = BAD;  // flag the bad
				print_debug("\n\rRef Bad");
				}
			}
		}
	// is sensor into fault range?
	else if((sensor->ref_fault_threshold != 0) && 
			(sensor->raw_ref_adc_counts > sensor->ref_fault_threshold) && 
			(sensor->ref_status != BAD))
		{
		// require x readings in a row to fail
		sensor->ref_error_count++;
		if(sensor->ref_error_count >= FAULT_COUNT)
			{
			sensor->ref_error_count = FAULT_COUNT;	// prevent wrap!
			if(sensor->ref_status != FAULT)
				{
				sensor->ref_status = FAULT;  // flag the bad
				print_debug("\n\rRef Fault");
				}
			}
		}
	// recovery from some bad status counts, but not prealarms or faults 
	else if((sensor->ref_status == UNKNOWN) ||
			(sensor->ref_status == GOOD))
		{
		sensor->ref_error_count = 0;
		sensor->ref_status = GOOD;		 // continue as good
		}
}

//*****************************************************************************
//* Function:  Check_Thermistor_Out
//*
//* Parameters:
//*     sensor: pointer to the struct members for HEX outlet temperature sensor
//*   
//* Description:
//*
//*****************************************************************************
void Check_Thermistor_Out(hex_temp_t * sensor) {
    // read adc counts
    Get_HEX_Out_Counts(sensor);
    
    /* Verify thermistor count value */
	// is sensor out of the valid range?
	if((sensor->raw_thm_adc_counts < sensor->thm_error_low_threshold) || 
			(sensor->raw_thm_adc_counts > sensor->thm_error_high_threshold))
		{
		// require x readings in a row to fail
		sensor->thm_error_count++;
		if(sensor->thm_error_count >= FAULT_COUNT)
			{
			sensor->thm_error_count = FAULT_COUNT;  // prevent wrap!
			if(sensor->thm_status != BAD)
				{
				sensor->thm_status = BAD;	 // mark it as bad
				print_debug("\n\rThrmOut Bad");
				}
			}
		}
	// is sensor into fault range?
	else if((sensor->thm_fault_threshold != 0) && 
			(sensor->raw_thm_adc_counts > sensor->thm_fault_threshold) && 
			(sensor->thm_status != BAD))
		{
		// require x readings in a row to fail
		sensor->thm_error_count++;
		if(sensor->thm_error_count >= FAULT_COUNT)
			{
			sensor->thm_error_count = FAULT_COUNT;	// prevent wrap!
			if(sensor->thm_status != FAULT)
				{
				sensor->thm_status = FAULT;	 // mark it as bad
				print_debug("\n\rThrmOut Fault");
				}
			}
		}
	// recovery from some bad status counts, but not prealarms or faults 
	else if((sensor->thm_status == UNKNOWN) ||
			(sensor->thm_status == GOOD))
		{
		sensor->thm_error_count = 0;
		sensor->thm_status = GOOD;		 // continue as good
		}
        
    /* Verify reference resistor count value */
	// is sensor out of the valid range?
	if((sensor->raw_ref_adc_counts < sensor->ref_error_low_threshold) || 
			(sensor->raw_ref_adc_counts > sensor->ref_error_high_threshold))
		{
		// require x readings in a row to fail
		sensor->ref_error_count++;
		if(sensor->ref_error_count >= FAULT_COUNT)
			{
			sensor->ref_error_count = FAULT_COUNT;  // prevent wrap!
			if(sensor->ref_status != BAD)
				{
				sensor->ref_status = BAD;	 // mark it as bad
				print_debug("\n\rRef Bad");
				}
			}
		}
	// is sensor into fault range?
	else if((sensor->ref_fault_threshold != 0) && 
			(sensor->raw_ref_adc_counts > sensor->ref_fault_threshold) && 
			(sensor->ref_status != BAD))
		{
		// require x readings in a row to fail
		sensor->ref_error_count++;
		if(sensor->ref_error_count >= FAULT_COUNT)
			{
			sensor->ref_error_count = FAULT_COUNT;	// prevent wrap!
			if(sensor->ref_status != FAULT)
				{
				sensor->ref_status = FAULT;	 // mark it as bad
				print_debug("\n\rRef Fault");
				}
			}
		}
	// recovery from some bad status counts, but not prealarms or faults 
	else if((sensor->ref_status == UNKNOWN) ||
			(sensor->ref_status == GOOD))
		{
		sensor->ref_error_count = 0;
		sensor->ref_status = GOOD;		 // continue as good
		}
}

//*****************************************************************************
//* Function:  Check_Pri_Fan_IS
//*
//* Parameters:
//*     sensor: pointer to the struct members for pri fan current sensor
//*   
//* Description:
//*
//*****************************************************************************
void Check_Pri_Fan_IS(current_sense_t * sensor) {
    // read adc counts
    Get_Pri_Fan_IS_Counts(sensor);
    
	// is sensor out of the valid range?
	if((sensor->raw_adc_counts < sensor->error_low_threshold) || 
			(sensor->raw_adc_counts > sensor->error_high_threshold))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;  // prevent wrap!
			if(sensor->status != BAD)
				{
				UART_1_PutString("\n\rPriFan Current Bad");
				}
			sensor->status = BAD;	 // mark it as bad
			}
		}
	// is sensor into fault range?
	else if((sensor->fault_threshold != 0) && 
			(sensor->raw_adc_counts > sensor->fault_threshold) && 
			(sensor->status != BAD))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;	// prevent wrap!
			sensor->status = FAULT;  // flag the fault
			UART_1_PutString("\n\rPriFan Current Fault");
			}
		}
	// recovery from some bad status counts, but not prealarms or faults 
	else if((sensor->status == UNKNOWN) ||
			(sensor->status == GOOD))
		{
		sensor->error_count = 0;
		sensor->status = GOOD;		 // continue as good
		}
}

//*****************************************************************************
//* Function:  Check_Sec_Fan_IS
//*
//* Parameters:
//*     sensor: pointer to the struct members for sec fan current sensor
//*   
//* Description:
//*
//*****************************************************************************
void Check_Sec_Fan_IS(current_sense_t * sensor) {
    // read adc counts
    Get_Sec_Fan_IS_Counts(sensor);
    
	// is sensor out of the valid range?
	if((sensor->raw_adc_counts < sensor->error_low_threshold) || 
			(sensor->raw_adc_counts > sensor->error_high_threshold))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;  // prevent wrap!
			if(sensor->status != BAD)
				{
				UART_1_PutString("\n\rSecFan Current Bad");
				}
			sensor->status = BAD;	 // mark it as bad
			}
		}
	// is sensor into fault range?
	else if((sensor->fault_threshold != 0) && 
			(sensor->raw_adc_counts > sensor->fault_threshold) && 
			(sensor->status != BAD))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;	// prevent wrap!
			sensor->status = FAULT;  // flag the fault
			UART_1_PutString("\n\rSecFan Current Fault");
			}
		}
	// recovery from some bad status counts, but not prealarms or faults 
	else if((sensor->status == UNKNOWN) ||
			(sensor->status == GOOD))
		{
		sensor->error_count = 0;
		sensor->status = GOOD;		 // continue as good
		}
}

//*****************************************************************************
//* Function:  Check_CP_IS
//*
//* Parameters:
//*     sensor: pointer to the struct members for coolant pump current sensor
//*   
//* Description:
//*
//*****************************************************************************
void Check_CP_IS(current_sense_t * sensor) {
    // read adc counts
    Get_Coolant_Pump_IS(sensor);
    
	// is sensor out of the valid range?
	if((sensor->raw_adc_counts < sensor->error_low_threshold) || 
			(sensor->raw_adc_counts > sensor->error_high_threshold))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;  // prevent wrap!
			if(sensor->status != BAD)
				{
				UART_1_PutString("\n\rCP Current Bad");
				}
			sensor->status = BAD;	 // mark it as bad
			}
		}
	// is sensor into fault range?
	else if((sensor->fault_threshold != 0) && 
			(sensor->raw_adc_counts > sensor->fault_threshold) && 
			(sensor->status != BAD))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;	// prevent wrap!
			sensor->status = FAULT;  // flag the fault
			UART_1_PutString("\n\rCP Current Fault");
			}
		}
	// recovery from some bad status counts, but not prealarms or faults 
	else if((sensor->status == UNKNOWN) ||
			(sensor->status == GOOD))
		{
		sensor->error_count = 0;
		sensor->status = GOOD;		 // continue as good
		}
}

//*****************************************************************************
//* Function:  Check_GP_IS
//*
//* Parameters:
//*     sensor: pointer to the struct members for glow plug current sensor
//*   
//* Description:
//*
//*****************************************************************************
void Check_GP_IS(current_sense_t * sensor) {
    // read adc counts
    Get_Glow_Plug_IS(sensor);
    
	// is sensor out of the valid range?
	if((sensor->raw_adc_counts < sensor->error_low_threshold) || 
			(sensor->raw_adc_counts > sensor->error_high_threshold))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;  // prevent wrap!
			if(sensor->status != BAD)
				{
				UART_1_PutString("\n\rGP Current Bad");
				}
			sensor->status = BAD;	 // mark it as bad
			}
		}
	// is sensor into fault range?
	else if((sensor->fault_threshold != 0) && 
			(sensor->raw_adc_counts > sensor->fault_threshold) && 
			(sensor->status != BAD))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;	// prevent wrap!
			sensor->status = FAULT;  // flag the fault
			UART_1_PutString("\n\rGP Current Fault");
			}
		}
	// recovery from some bad status counts, but not prealarms or faults 
	else if((sensor->status == UNKNOWN) ||
			(sensor->status == GOOD))
		{
		sensor->error_count = 0;
		sensor->status = GOOD;		 // continue as good
		}
}

//*****************************************************************************
//* Function:  Check_Inj_IS
//*
//* Parameters:
//*     sensor: pointer to the struct members for injector current sensor
//*   
//* Description:
//*
//*****************************************************************************
void Check_Inj_IS(current_sense_t * sensor) {
    // read adc counts
    Get_Inj_IS_Counts(sensor);
    
	// is sensor out of the valid range?
	if((sensor->raw_adc_counts < sensor->error_low_threshold) || 
			(sensor->raw_adc_counts > sensor->error_high_threshold))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;  // prevent wrap!
			if(sensor->status != BAD)
				{
				UART_1_PutString("\n\rInj Current Bad");
				}
			sensor->status = BAD;	 // mark it as bad
			}
		}
	// is sensor into fault range?
	else if((sensor->fault_threshold != 0) && 
			(sensor->raw_adc_counts > sensor->fault_threshold) && 
			(sensor->status != BAD))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;	// prevent wrap!
			sensor->status = FAULT;  // flag the fault
			UART_1_PutString("\n\rInj Current Fault");
			}
		}
	// recovery from some bad status counts, but not prealarms or faults 
	else if((sensor->status == UNKNOWN) ||
			(sensor->status == GOOD))
		{
		sensor->error_count = 0;
		sensor->status = GOOD;		 // continue as good
		}
}

//*****************************************************************************
//* Function:  Check_PU_IS
//*
//* Parameters:
//*     sensor: pointer to the struct members for fuel pump current sensor
//*   
//* Description:
//*
//*****************************************************************************
void Check_PU_IS(current_sense_t * sensor) {
    // read adc counts
    Get_PU_IS_Counts(sensor);
    
	// is sensor out of the valid range?
	if((sensor->raw_adc_counts < sensor->error_low_threshold) || 
			(sensor->raw_adc_counts > sensor->error_high_threshold))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;  // prevent wrap!
			if(sensor->status != BAD)
				{
				UART_1_PutString("\n\rPU Current Bad");
				}
			sensor->status = BAD;	 // mark it as bad
			}
		}
	// is sensor into fault range?
	else if((sensor->fault_threshold != 0) && 
			(sensor->raw_adc_counts > sensor->fault_threshold) && 
			(sensor->status != BAD))
		{
		// require x readings in a row to fail
		sensor->error_count++;
		if(sensor->error_count >= FAULT_COUNT)
			{
			sensor->error_count = FAULT_COUNT;	// prevent wrap!
			sensor->status = FAULT;  // flag the fault
			UART_1_PutString("\n\rPU Current Fault");
			}
		}
	// recovery from some bad status counts, but not prealarms or faults 
	else if((sensor->status == UNKNOWN) ||
			(sensor->status == GOOD))
		{
		sensor->error_count = 0;
		sensor->status = GOOD;		 // continue as good
		}
}

//*****************************************************************************
//* Function:  Check_Heat_Ex_Delta
//*
//* Parameters:
//*     sensor: pointer to the struct members for fuel pump current sensor
//*   
//* Description:
//*
//*****************************************************************************
void Check_Heat_Ex_Delta(hex_temp_t * thmi, hex_temp_t * thmo) {
    float delta;
    
    delta = fabsf(thmi->temp_deg_c - thmo->temp_deg_c);
    
    if(delta < 15 || delta > 30) {
        heat_exchanger.error_count++;
    }
    else {
        heat_exchanger.error_count = 0;
        heat_exchanger.status = GOOD;
    }
    
    if(heat_exchanger.error_count >= FAULT_COUNT){
        heat_exchanger.status = FAULT;
    }
}

//*****************************************************************************
//* Function:  Exponential_Filter
//*
//* Usage: 
//*   
//* Description:    Performs an exponential filter on the data, weighted towards the average
//* the average by 'cycles' amount.
//* If cycles = 75, the output is (new + old * 74)/75, and has a 0.5sec time
//* constant (step function in to 63% out) at 20ms per loop.
//*
//*****************************************************************************
float Exponential_Filter(float average, float target, float cycles) {
	float new_average;

	// keeps a running average by adding on 1/Tau * new value
    //	if(cycles != 0)	// if zero, ignore filtering	
	// if cycles time is zero or existing average is zero, ignore filtering
	if((cycles != 0) && (average > 0.01))	
		{
		new_average = (target + average * (cycles - 1.0)) / cycles;
		}
	else
		{
		new_average = target;		// no filtering, input direct to output
		}
	return(new_average);
}

//*****************************************************************************
//* Function:  Check_Reserve_Fuel_Switch
//*
//* Parameters:
//*     sensor: pointer to the struct members for fuel pump current sensor
//*   
//* Description:
//*
//*****************************************************************************
void Check_Reserve_Fuel_Switch( ) {
    // TBD
    
}

//*****************************************************************************
//* Function:  Check_Empty_Fuel_Switch
//*
//* Parameters:
//*     sensor: pointer to the struct members for fuel pump current sensor
//*   
//* Description:
//*
//*****************************************************************************
void Check_Empty_Fuel_Switch( ) {
    // TBD
    
    
}

//*****************************************************************************
//* Function:  Check_CJC_IC
//*
//* Parameters:
//*     sensor: pointer to the struct members for fuel pump current sensor
//*   
//* Description:
//*
//*****************************************************************************
void Check_CJC_IC( ) {
    // TBD
    // uint8_t feedback;
    // feedback = CJCP_Read( );
    // if(feedback)
}

//*****************************************************************************
//* Function:  Celsius_To_Counts
//*
//* Usage: 
//*   
//* Description:    Converts celsius to ADC counts.
//*   
//*****************************************************************************
int32_t Celsius_To_Counts(int32_t celsius, int8_t sensor) {
	int32_t counts;
    float resThermistor;
    float uVolts;
    float kelvin;
    float x, y;
    
    switch (sensor) {
        case 0: uVolts = Primary_T_GetVoltage(celsius / 100);
                //counts = (uVolts / 3.3) * 1048575;                // if using 20-bit ADC
                //counts = (uVolts / 3.3) * 65535;                    // if using 16-bit ADC
                counts = (uVolts / 3.3) * 262144;                   // if using 18-bit ADC
                break;
                
        case 1: uVolts = Mixing_T_GetVoltage(celsius / 100);
                //counts = (uVolts / 3.3) * 1048575;                // if using 20-bit ADC
                //counts = (uVolts / 3.3) * 65535;                    // if using 16-bit ADC
                counts = (uVolts / 3.3) * 262144;                   // if using 18-bit ADC
                break;
                
        case 2: uVolts = Secondary_T_GetVoltage(celsius / 100);
                //counts = (uVolts / 3.3) * 1048575;                // if using 20-bit ADC
                //counts = (uVolts / 3.3) * 65535;                    // if using 16-bit ADC
                counts = (uVolts / 3.3) * 262144;                   // if using 18-bit ADC
                break;
                
        case 3: uVolts = Exhaust_T_GetVoltage(celsius / 100);
                //counts = (uVolts / 3.3) * 1048575;                // if using 20-bit ADC
                //counts = (uVolts / 3.3) * 65535;                    // if using 16-bit ADC
                counts = (uVolts / 3.3) * 262144;                   // if using 18-bit ADC
                break;
                
        /* Using the Steinhart-Hart equation to get resistance from a known temperature */
        case 4: kelvin = celsius + HEX_In_K2C; 
                x = 1 / Thermistor_Coeff_C * (Thermistor_Coeff_A - 1 / kelvin);
                y = sqrt(powf(Thermistor_Coeff_B / (Thermistor_Coeff_C * 3), 3) + pow(x / 2, 2));
                resThermistor = expf(cbrtf(y - (x / 2)) - cbrtf(y + (x / 2)));
                
                counts = celsius;   // TBD
                
                break;
        
        /* Using the Steinhart-Hart equation to get resistance from a known temperature */
        case 5: kelvin = celsius + HEX_In_K2C;
                x = 1 / Thermistor_Coeff_C * (Thermistor_Coeff_A - 1 / kelvin);
                y = sqrtf(powf(Thermistor_Coeff_B / (Thermistor_Coeff_C * 3), 3) + powf(x / 2, 2));
                resThermistor = expf(cbrtf(y - (x / 2)) - cbrtf(y + (x / 2)));
                
                counts = celsius;   // TBD
                
                break;
        
        default: counts = 0;
    }

	return counts; 
}

//*****************************************************************************
//* Function:  Init_CAN
//*
//* Usage: 
//*   
//* Description:  Initialize the CAN controller, send the first status message
//*   
//*****************************************************************************
void Init_CAN(void)
{
	CAN_1_SetOpMode(CAN_1_ACTIVE_RUN_MODE);

	CAN_1_Start();

	can_send_status_msg();	  // send status message at power up
}

//*****************************************************************************
//* Function:  can_send_status_msg
//*
//* Usage: 
//*   
//* Description:  send the status message via CAN
//*   
//*****************************************************************************
uint8_t can_send_status_msg(void)
{	
	uint8_t result;

	print_status_page();

    result = CAN_1_SendMsgStatus();

	return result;
}

//*****************************************************************************
//* Function:  can_send_thermocouple_msg
//*
//* Usage: 
//*   
//* Description:  send the thermocouple message via CAN
//*   
//*****************************************************************************
uint8_t can_send_thermocouple_msg(void)
{	
	uint8_t result;

	// can_tx_message.id = 0x18FFA080;
	CAN_1_TX_DATA_BYTE1(CAN_1_TX_MAILBOX_Thermocouples) = (uint16_t)primary_temp.temp_deg_c;			// wah: manage sign on negative temperatures!
	CAN_1_TX_DATA_BYTE2(CAN_1_TX_MAILBOX_Thermocouples) = ((uint16_t)primary_temp.temp_deg_c) >> 8;
	CAN_1_TX_DATA_BYTE3(CAN_1_TX_MAILBOX_Thermocouples) = (uint16_t)mixing_temp.temp_deg_c;
	CAN_1_TX_DATA_BYTE4(CAN_1_TX_MAILBOX_Thermocouples) = ((uint16_t)mixing_temp.temp_deg_c) >> 8;
	CAN_1_TX_DATA_BYTE5(CAN_1_TX_MAILBOX_Thermocouples) = (uint16_t)secondary_temp.temp_deg_c;
	CAN_1_TX_DATA_BYTE6(CAN_1_TX_MAILBOX_Thermocouples) = ((uint16_t)secondary_temp.temp_deg_c) >> 8;
	CAN_1_TX_DATA_BYTE7(CAN_1_TX_MAILBOX_Thermocouples) = (uint16_t)egt_temp.temp_deg_c;
	CAN_1_TX_DATA_BYTE8(CAN_1_TX_MAILBOX_Thermocouples) = ((uint16_t)egt_temp.temp_deg_c) >> 8;

    result = CAN_1_SendMsgThermocouples();

	return result;
}

//*****************************************************************************
//* Function:  can_send_effector_current_msg
//*
//* Usage: 
//*   
//* Description:  Send the effector current message via CAN
//*   
//*****************************************************************************
uint8_t can_send_effector_current_msg(void)
{	
	uint8_t result;

	// can_tx_message.id = 0x18FFA180;
	CAN_1_TX_DATA_BYTE1(CAN_1_TX_MAILBOX_Effectors) = inj_current.current_mA / 100;  // convert from mA to 1/10 Amps
	CAN_1_TX_DATA_BYTE2(CAN_1_TX_MAILBOX_Effectors) = gp_current.current_mA / 100;
	CAN_1_TX_DATA_BYTE3(CAN_1_TX_MAILBOX_Effectors) = pf_current.current_mA / 100;
	CAN_1_TX_DATA_BYTE4(CAN_1_TX_MAILBOX_Effectors) = sf_current.current_mA / 100;
	CAN_1_TX_DATA_BYTE5(CAN_1_TX_MAILBOX_Effectors) = pu_current.current_mA / 100;
	CAN_1_TX_DATA_BYTE6(CAN_1_TX_MAILBOX_Effectors) = cp_current.current_mA / 100;
	CAN_1_TX_DATA_BYTE7(CAN_1_TX_MAILBOX_Effectors) = 0;
	CAN_1_TX_DATA_BYTE8(CAN_1_TX_MAILBOX_Effectors) = 0;

    result = CAN_1_SendMsgEffectors();

	return result;
}

//*****************************************************************************
//* Function:  can_send_hex_media_temps_msg
//*
//* Usage: 
//*   
//* Description:  Send the heat exchanger media temps message via CAN
//*   
//*****************************************************************************
uint8_t can_send_hex_media_temps_msg(void)
{	
	uint8_t result;

	// can_tx_message.id = 0x18FFA37E;
	CAN_1_TX_DATA_BYTE1(CAN_1_TX_MAILBOX_HexTemps) = (uint16_t)hex_inlet_temp.temp_deg_c;		  // wah: manage sign on negative temperatures!
	CAN_1_TX_DATA_BYTE2(CAN_1_TX_MAILBOX_HexTemps) = ((uint16_t)hex_inlet_temp.temp_deg_c) >> 8;
	CAN_1_TX_DATA_BYTE3(CAN_1_TX_MAILBOX_HexTemps) = (uint16_t)hex_outlet_temp.temp_deg_c;
	CAN_1_TX_DATA_BYTE4(CAN_1_TX_MAILBOX_HexTemps) = ((uint16_t)hex_outlet_temp.temp_deg_c) >> 8;
	CAN_1_TX_DATA_BYTE5(CAN_1_TX_MAILBOX_HexTemps) = 0;
	CAN_1_TX_DATA_BYTE6(CAN_1_TX_MAILBOX_HexTemps) = 0;
	CAN_1_TX_DATA_BYTE7(CAN_1_TX_MAILBOX_HexTemps) = 0;
	CAN_1_TX_DATA_BYTE8(CAN_1_TX_MAILBOX_HexTemps) = 0;

    result = CAN_1_SendMsgHexTemps();

	return result;
}

//*****************************************************************************
//* Function:  can_send_primary_fan_msg
//*
//* Usage: 
//*   
//* Description:  Send the primary fan message via CAN
//*   
//*****************************************************************************
uint8_t can_send_primary_fan_msg(void)
{	
	uint8_t result;

	// can_tx_message.id = 0x18FFA47D;
	CAN_1_TX_DATA_BYTE1(CAN_1_TX_MAILBOX_PriFans) = pri_fan_speed[0].measured_speed_rpm;
	CAN_1_TX_DATA_BYTE2(CAN_1_TX_MAILBOX_PriFans) = pri_fan_speed[0].measured_speed_rpm >> 8;
	CAN_1_TX_DATA_BYTE3(CAN_1_TX_MAILBOX_PriFans) = pri_fan_speed[1].measured_speed_rpm;
	CAN_1_TX_DATA_BYTE4(CAN_1_TX_MAILBOX_PriFans) = pri_fan_speed[1].measured_speed_rpm >> 8;
	CAN_1_TX_DATA_BYTE5(CAN_1_TX_MAILBOX_PriFans) = pri_fan_speed[2].measured_speed_rpm;
	CAN_1_TX_DATA_BYTE6(CAN_1_TX_MAILBOX_PriFans) = pri_fan_speed[2].measured_speed_rpm >> 8;
	CAN_1_TX_DATA_BYTE7(CAN_1_TX_MAILBOX_PriFans) = 0;
	CAN_1_TX_DATA_BYTE8(CAN_1_TX_MAILBOX_PriFans) = 0;

    result = CAN_1_SendMsgPriFans();

	return result;
}

//*****************************************************************************
//* Function:  can_send_secondary_fan_msg
//*
//* Usage: 
//*   
//* Description:  Send the secondary fan message via CAN
//*   
//*****************************************************************************
uint8_t can_send_secondary_fan_msg(void)
{	
	uint8_t result;

	// can_tx_message.id = 0x18FFA57C;
	CAN_1_TX_DATA_BYTE1(CAN_1_TX_MAILBOX_SecFans) = sec_fan_speed[0].measured_speed_rpm;
	CAN_1_TX_DATA_BYTE2(CAN_1_TX_MAILBOX_SecFans) = sec_fan_speed[0].measured_speed_rpm >> 8;
	CAN_1_TX_DATA_BYTE3(CAN_1_TX_MAILBOX_SecFans) = sec_fan_speed[1].measured_speed_rpm;
	CAN_1_TX_DATA_BYTE4(CAN_1_TX_MAILBOX_SecFans) = sec_fan_speed[1].measured_speed_rpm >> 8;
	CAN_1_TX_DATA_BYTE5(CAN_1_TX_MAILBOX_SecFans) = sec_fan_speed[2].measured_speed_rpm;
	CAN_1_TX_DATA_BYTE6(CAN_1_TX_MAILBOX_SecFans) = sec_fan_speed[2].measured_speed_rpm >> 8;
	CAN_1_TX_DATA_BYTE7(CAN_1_TX_MAILBOX_SecFans) = 0;
	CAN_1_TX_DATA_BYTE8(CAN_1_TX_MAILBOX_SecFans) = 0;

    result = CAN_1_SendMsgSecFans();

	return result;
}

//*****************************************************************************
//* Function:  can_send_reserved_msg
//*
//* Usage: 
//*   
//* Description:  Send the reserved message via CAN
//*   
//*****************************************************************************
uint8_t can_send_reserved_msg(void)
{	
	uint8_t result;

	// can_tx_message.id = 0x18FFA67B;
	CAN_1_TX_DATA_BYTE1(CAN_1_TX_MAILBOX_Reserved) = 0;	 // all bytes unused
	CAN_1_TX_DATA_BYTE2(CAN_1_TX_MAILBOX_Reserved) = 0;
	CAN_1_TX_DATA_BYTE3(CAN_1_TX_MAILBOX_Reserved) = 0;
	CAN_1_TX_DATA_BYTE4(CAN_1_TX_MAILBOX_Reserved) = 0;
	CAN_1_TX_DATA_BYTE5(CAN_1_TX_MAILBOX_Reserved) = 0;
	CAN_1_TX_DATA_BYTE6(CAN_1_TX_MAILBOX_Reserved) = 0;
	CAN_1_TX_DATA_BYTE7(CAN_1_TX_MAILBOX_Reserved) = 0;
	CAN_1_TX_DATA_BYTE8(CAN_1_TX_MAILBOX_Reserved) = 0;

    result = CAN_1_SendMsgReserved();

	return result;
}

//*****************************************************************************
//* Function:  can_send_hours_msg
//*
//* Usage: 
//*   
//* Description:  Send the hours since light-off message via CAN
//*   
//*****************************************************************************
uint8_t can_send_hours_msg(void)
{	
	uint8_t result;
	uint32_t seconds_since_light_off;

	// if non zero
	if(light_off_time_ms)
		{ 
		seconds_since_light_off = (main_timer_ms - light_off_time_ms)/1000;
		}
	// otherwise if 0, lightoff hasn't happened yet
	else
		{
		seconds_since_light_off = 0;		
		}

	// can_tx_message.id = 0x18FFAC75;
	CAN_1_TX_DATA_BYTE1(CAN_1_TX_MAILBOX_Hours) = seconds_since_light_off;	 
	CAN_1_TX_DATA_BYTE2(CAN_1_TX_MAILBOX_Hours) = seconds_since_light_off >> 8;
	CAN_1_TX_DATA_BYTE3(CAN_1_TX_MAILBOX_Hours) = seconds_since_light_off >> 16;
	CAN_1_TX_DATA_BYTE4(CAN_1_TX_MAILBOX_Hours) = seconds_since_light_off >> 24;
	CAN_1_TX_DATA_BYTE5(CAN_1_TX_MAILBOX_Hours) = 0;
	CAN_1_TX_DATA_BYTE6(CAN_1_TX_MAILBOX_Hours) = 0;
	CAN_1_TX_DATA_BYTE7(CAN_1_TX_MAILBOX_Hours) = 0;
	CAN_1_TX_DATA_BYTE8(CAN_1_TX_MAILBOX_Hours) = 0;

    result = CAN_1_SendMsgHours();

	return result;
}

//*****************************************************************************
//* Function:  Sensor_Plausibility_Checks
//*
//* Usage: 
//*   
//* Description:  Perform sensor plausibility checks
//*   
//*****************************************************************************
void Sensor_Plausibility_Checks(void)
{
	Check_Thermocouple_1(&primary_temp);
    Check_Thermocouple_2(&mixing_temp);
    Check_Thermocouple_3(&secondary_temp);
    Check_Thermocouple_4(&egt_temp);
    Check_Thermistor_In(&hex_inlet_temp);
    Check_Thermistor_Out(&hex_outlet_temp);
    Check_Pri_Fan_IS(&pf_current);
    Check_Sec_Fan_IS(&sf_current);
    Check_CP_IS(&cp_current);
    Check_GP_IS(&gp_current);
    Check_Inj_IS(&inj_current);
    Check_PU_IS(&pu_current);
    Check_Reserve_Fuel_Switch( );       // TBD
    Check_Empty_Fuel_Switch( );         // TBD
    Check_CJC_IC( );                    // TBD
}

//*****************************************************************************
//* Function:  write_glow_plug_output
//*
//* Usage: 
//*   
//* Description:  Write the glow plug output
//*   
//*****************************************************************************
void write_glow_plug_output(void)
{
	if(glow_plug.status == GLOW_PLUG_ON)
	{
    	GlowPlug_Write(1);     // Power on glow plug
	}
	else
	{
    	GlowPlug_Write(0);     // Power off glow plug
	}
}

//*****************************************************************************
//* Function:  write_coolant_pump_output
//*
//* Usage: 
//*   
//* Description:  Write the coolant pump output
//*   
//*****************************************************************************
void write_coolant_pump_output(void)
{
	if(coolant_pump.status == PUMP_ON)
	{
    	CoolantPump_Write(1);     // Power on coolant pump
	}
	else
	{
    	CoolantPump_Write(0);     // Power off coolant pump
	}
}

//*****************************************************************************
//* Function:  write_primary_fan_output
//*
//* Usage: 
//*   
//* Description:  Write the primary fan output
//*   
//*****************************************************************************
void write_pri_fan_output(void)
{
	if(pri_fan_speed[0].status == FAN_ON)
	{
    	Pri_Fan_Write(1);     // Power on primary fan
	}
	else
	{
    	Pri_Fan_Write(0);     // Power off primary fan
	}
}

//*****************************************************************************
//* Function:  write_secondary_fan_output
//*
//* Usage: 
//*   
//* Description:  Write the secondary fan output
//*   
//*****************************************************************************
void write_sec_fan_output(void)
{
	if(sec_fan_speed[0].status == FAN_ON)
	{
    	Sec_Fan_Write(1);     // Power on secondary fan
	}
	else
	{
    	Sec_Fan_Write(0);     // Power off secondary fan
	}
}

//*****************************************************************************
//* Function:  write_fuel_pump_output
//*
//* Usage: 
//*   
//* Description:  Write the fuel pump output
//*   
//*****************************************************************************
void write_fuel_pump_output(void)
{
	if(fuel_pump.status == PUMP_ON)
	{
    	FuelPump_Write(1);     // Power on fuel pump
	}
	else
	{
    	FuelPump_Write(0);     // Power off fuel pump
	}
}

//*****************************************************************************
//* Function:  update_injector_pwm
//*
//* Usage: 
//*   
//* Description:  Write the injector output
//*   
//*****************************************************************************
void update_injector_pwm(void)
{
	INJ_MOD_WritePeriod(1000);				// PWM period of 1000cnts is 10ms, 100Hz (in 100's of microseconds, 0-65535 cnts at 100kHz clk)
	INJ_MOD_WriteCompare(injector.duty);	// compare/duty output, 0 is off, 1000 is fully on (when 10ms period, above)
}

//*****************************************************************************
//* Function:  Standard_Shutdown
//*
//* Usage: 
//*   
//* Description:  Initiates standard shutdown
//*   
//*****************************************************************************
void Standard_Shutdown(uint8_t shutdown_reason)
{
	// if already in a shutdown, keep that shutdown method
	if(!block_shutdown)
		{
		if(!shutdown_reason)
			fault_code = shutdown_reason;

		if((system_state != STATE_STANDARD_SHUTDOWN) && (system_state != STATE_EMERGENCY_SHUTDOWN))
		{
			system_state = STATE_STANDARD_SHUTDOWN;
			UART_1_PutString("\n\r*STATE TO STANDARD SHUTDOWN*");
		}

		if((primary_temp.temp_deg_c > primary_catalyst.cooldown_temp) || (secondary_temp.temp_deg_c > secondary_catalyst.cooldown_temp)) 
		{
			//FUEL RATE = SHUTDOWN SPEED
			fuel_pump.cmd_speed_rpm	= 0;   // Not used
			fuel_pump.status = PUMP_OFF;
			injector.duty = fuel_duty_shutdown;

			pri_fan_speed[0].cmd_speed_rpm = pri_fan_speed[0].shutdown_speed;
			pri_fan_speed[1].cmd_speed_rpm = pri_fan_speed[1].shutdown_speed;
			/*pri_fan_speed[2].cmd_speed_rpm = pri_fan_speed[2].shutdown_speed;*/ //Only monitor-control 2 fan speeds
			sec_fan_speed[0].cmd_speed_rpm = sec_fan_speed[0].shutdown_speed;
			sec_fan_speed[1].cmd_speed_rpm = sec_fan_speed[1].shutdown_speed;
			/*sec_fan_speed[2].cmd_speed_rpm = sec_fan_speed[2].shutdown_speed;*/ //Only monitor-control 2 fan speeds
		}
		else
		{
			fuel_pump.cmd_speed_rpm	= 0;
			pri_fan_speed[0].cmd_speed_rpm = pri_fan_speed[0].shutdown_speed;
			pri_fan_speed[1].cmd_speed_rpm = pri_fan_speed[1].shutdown_speed;
			//pri_fan_speed[2].cmd_speed_rpm = 0;
			pri_fan_speed[0].status = FAN_OFF;
			pri_fan_speed[1].status = FAN_OFF;
			//pri_fan_speed[2].status = FAN_OFF;
			sec_fan_speed[0].cmd_speed_rpm = sec_fan_speed[0].shutdown_speed;
			sec_fan_speed[1].cmd_speed_rpm = sec_fan_speed[1].shutdown_speed;
			//sec_fan_speed[2].cmd_speed_rpm = 0;
			sec_fan_speed[0].status = FAN_OFF;
			sec_fan_speed[1].status = FAN_OFF;
			//sec_fan_speed[2].status = FAN_OFF;
			coolant_pump.status = PUMP_OFF;
			Init_Outputs();
		}
	}
        
        for (i = 0; i < Pri_Fan_Array_NUMBER_OF_FANS; i++) {
            Pri_Fan_Array_SetDesiredSpeed(i + 1,pri_fan_speed[i].cmd_speed_rpm);
        }
        
        for (i = 0; i < Sec_Fan_Array_NUMBER_OF_FANS; i++) {
            Sec_Fan_Array_SetDesiredSpeed(i + 1,sec_fan_speed[i].cmd_speed_rpm);
        }
}

//*****************************************************************************
//* Function:  Emergency_Shutdown
//*
//* Usage: 
//*   
//* Description:  Initiates an Emergency Shutdown (all off immediately)
//*   
//*****************************************************************************
void Emergency_Shutdown(uint8_t shutdown_reason)
{

	if(!block_shutdown && (system_state != STATE_EMERGENCY_SHUTDOWN)) //Was = !State_Emergency_Shutdown
	{
		if(!shutdown_reason)
			{
			fault_code = shutdown_reason;
			}

		Init_Outputs();
		system_state = STATE_EMERGENCY_SHUTDOWN;
		fuel_pump.status = PUMP_OFF;
		injector.duty = fuel_duty_shutdown;
		pri_fan_speed[0].cmd_speed_rpm = pri_fan_speed[0].shutdown_speed;
		pri_fan_speed[1].cmd_speed_rpm = pri_fan_speed[1].shutdown_speed;
		//pri_fan_speed[2].cmd_speed_rpm = 0;
		pri_fan_speed[0].status = FAN_OFF;
		pri_fan_speed[1].status = FAN_OFF;
		//pri_fan_speed[2].status = FAN_OFF;
		sec_fan_speed[0].cmd_speed_rpm = sec_fan_speed[0].shutdown_speed;
		sec_fan_speed[1].cmd_speed_rpm = sec_fan_speed[1].shutdown_speed;
		//sec_fan_speed[2].cmd_speed_rpm = 0;
		sec_fan_speed[0].status = FAN_OFF;
		sec_fan_speed[1].status = FAN_OFF;
		//sec_fan_speed[2].status = FAN_OFF;
		coolant_pump.status = PUMP_OFF;
        UART_1_PutString("\n\r*EMERGENCY SHUTDOWN*");
      	}
        for (i = 0; i < Pri_Fan_Array_NUMBER_OF_FANS; i++) {
            Pri_Fan_Array_SetDesiredSpeed(i + 1,pri_fan_speed[i].cmd_speed_rpm);
        }
        
        for (i = 0; i < Sec_Fan_Array_NUMBER_OF_FANS; i++) {
            Sec_Fan_Array_SetDesiredSpeed(i + 1,sec_fan_speed[i].cmd_speed_rpm);
        }
}

//*****************************************************************************
//* Function:  Write_Logfile
//*
//* Usage: 
//*   
//* Description:  Writes a snapshot of operational data to the EEPROM
//*   
//*****************************************************************************
void Write_Logfile(void)
{
	static uint16_t logging_addr;
	uint8_t log_data[16];

	logging_addr++;
	log_data[0] = logging_addr >> 8;
	log_data[1] = logging_addr;
	log_data[2] = main_timer_ms >> 24;
	log_data[3] = main_timer_ms >> 16;
	log_data[4] = main_timer_ms >> 8;
	log_data[5] = main_timer_ms;
	log_data[6] = system_state;
	log_data[7] = fault_code;
	log_data[8] = primary_temp.temp_deg_c;
	log_data[9] = secondary_temp.temp_deg_c;
	log_data[10] = mixing_temp.temp_deg_c;
	log_data[11] = egt_temp.temp_deg_c;
	log_data[12] = hex_inlet_temp.temp_deg_c;
	log_data[13] = hex_outlet_temp.temp_deg_c;
	log_data[14] = 0;
	log_data[15] = 0;

	// write to EEPROM
	EEPROM_1_UpdateTemperature();

	// write next row of 16 bytes
	EEPROM_1_Write(log_data,logging_addr);
}

//*****************************************************************************
//* Function:  Continuous_Bit
//*
//* Usage: 
//*   
//* Description:  (Future) Perform continuous built-in-test functions
//*   
//*****************************************************************************
void Continuous_Bit(void)
{
	// TBD
}

//*****************************************************************************
//* Function:  Continuous_Bit
//*
//* Usage: 
//*   
//* Description:  (Future) Perform continuous built-in-test functions
//*   
//*****************************************************************************
void print_debug(char8 string[CHAR_ARRAY_SIZE])
{
#if PRINT_DEBUG
	UART_1_PutString(string);
#endif
}


/* [] END OF FILE */
