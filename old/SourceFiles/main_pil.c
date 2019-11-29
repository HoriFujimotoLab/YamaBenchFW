/************************************************************************************
SINGLE MOTOR FIRMWARE
---------------------
RENAME MAIN_PIL (processor in the loop)
Descr.:		main file for single motor setup control
Boards:		PE-Expert3, C6713A DSP + Hardware
System:		single PMSM experimental setup
Author:		Thomas Beauduin, University of Tokyo, 2015
*************************************************************************************/
#include "system_fsm.h"

// TIMER COMP
#define ALPHA (0.2)
#define	PEV_BDN	0	
#define	INV_CH	0


interrupt void system_tint0(void);

void main(void)
{
	int_disable();

	// TIMER INIT
	timer0_init(TS);
	timer0_init_vector(system_tint0);
	timer0_start();
	timer0_enable_int();
	watch_init();
	watch_data();

	// TEST CODE INIT
	system_fsm_init();
	motor_enc_init();
	motor_adc_init();
	stage_adc_init();
	stage_lin_init();
	stage_enc_init();

	// INVERTER INIT
	pev_inverter_init(PEV_BDN, INV_CH, FC, DT);
	pev_inverter_set_uvw(PEV_BDN, INV_CH, 0.0, 0.0, 0.0);
	wait(TC);
	pev_inverter_start_pwm(PEV_BDN, INV_CH);

	int_enable();

	//SYSTEM RUN
	while (1){ system_fsm_mode(); }
}


void system_tint0(void)
{
	// TEST CODE
	// ---------

	// put the code to test in here and wave the timer
	// please change actuator output function to output 0 (to inverter)
	// this is test for timer not for exciting the system



	// TIMER COUNT
	// -----------
	t0 = timer0_read();
	t0_a = ALPHA * (t0 * 17.7777e-3) + (1 - ALPHA) * t0_a;
	if (msr >= 0 && msr < NROFT) { msr++; }
	watch_data_8ch();
}

