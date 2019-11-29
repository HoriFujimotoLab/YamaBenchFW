/************************************************************************************
SINGLE MOTOR FIRMWARE
---------------------
RENAME MAIN_PIL (processor in the loop)
Descr.:		main time test file for motor bench setup
Boards:		PE-Expert3, C6713A DSP + Hardware
Author:		Thomas Beauduin, University of Tokyo, 2016
*************************************************************************************/
#include "system_fsm.h"

// TIMER COMP
#define ALPHA (0.2)
#define	PEV_BDN	0	
#define	INV_CH	0
int tt0 = 0;


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

	// TEST CODE INIT
	system_fsm_init();
	motor_enc_init(0);
	motor_enc_init(1);
	setup_adc_init();

/*
	// INVERTER INIT
	pev_inverter_init(PEV_BDN, 0, FC, DT);								// init inv para
	pev_inverter_init(PEV_BDN, 1, FC, DT);
	pev_inverter_set_uvw(PEV_BDN, 0, 0.0, 0.0, 0.0);					// set modulation type
	pev_inverter_set_uvw(PEV_BDN, 1, 0.0, 0.0, 0.0);
	wait(TC);															// wait 1 carrier cycle [us]
	pev_inverter_start_pwm(PEV_BDN, 0);									// start gate signal (pwm)
	pev_inverter_start_pwm(PEV_BDN, 1);
*/
	int_enable();

	//SYSTEM RUN
	while (1){ //system_fsm_mode(); 
			}
}


void system_tint0(void)
{
	// TEST CODE
	// ---------

	// SENSOR READ
	motor_enc_elec(0, &theta_ex);
	motor_enc_elec(1, &theta_ey);
	setup_adc_read(0, &vdc_adx, &idc_adx, &iu_adx, &iw_adx);
	setup_adc_read(1, &vdc_ady, &idc_ady, &iu_ady, &iw_ady);


	// CURRENT CONTROL - X&Y AXES
//	if (sysmode_e == SYS_INI || sysmode_e == SYS_RUN)
//	{
		current_ctrl_uw2ab(iu_adx, iw_adx, &ia_adx, &ib_adx);
		current_ctrl_uw2ab(iu_ady, iw_ady, &ia_ady, &ib_ady);
		current_ctrl_ab2dq(ia_adx, ib_adx, theta_ex, &id_adx, &iq_adx);
		current_ctrl_ab2dq(ia_ady, ib_ady, theta_ey, &id_ady, &iq_ady);
		current_ctrl_zcpi(iq_refx, id_adx, iq_adx, &vd_refx, &vq_refx);
		current_ctrl_zcpi(iq_refy, id_ady, iq_ady, &vd_refy, &vq_refy);
		current_ctrl_dec(omega_max, id_adx, iq_adx, &vd_refx, &vq_refx);
		current_ctrl_dec(omega_may, id_ady, iq_ady, &vd_refy, &vq_refy);
		current_ctrl_dq2ab(vd_refx, vq_refx, theta_ex, &va_refx, &vb_refx);
		current_ctrl_dq2ab(vd_refy, vq_refy, theta_ey, &va_refy, &vb_refy);
		current_ctrl_ab2uvw(va_refx, vb_refx, &vu_refx, &vv_refx, &vw_refx);
		current_ctrl_ab2uvw(va_refy, vb_refy, &vu_refy, &vv_refy, &vw_refy);
	 	motor_inv_pwm(0, 0, 0, 0, vdc_adx); 
		motor_inv_pwm(1, 0, 0, 0, vdc_ady); 
//	}


	// TIMER COUNT
	// -----------
	tt0 = timer0_read();
	t0_a = ALPHA * (tt0 * 17.7777e-3) + (1 - ALPHA) * t0_a;
	if (msr >= 0 && msr < NROFT) { msr++; }
	watch_data_8ch();
}

