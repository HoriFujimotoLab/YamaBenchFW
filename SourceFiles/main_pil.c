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

	// SENSOR READ
	motor_enc_elec(&theta_e);
	stage_lin_read(&pos_t, &vel_t, &vel_ta);
	stage_enc_read(&theta_s, &omega_s, &omega_sa);
	motor_enc_read(&theta_m, &omega_m, &omega_ma);
	stage_adc_read(0, &vdc_ad, &idc_ad, &iu_ad, &iw_ad);
	stage_adc_read(1, &disp_s1, &disp_s2, &disp_m1, &disp_m2);
	stage_adc_read(2, &acc_mx, &acc_tx, &acc_tz, &acc_sx);

	// MOTION CTRL
	if (sysmode_e == SYS_INI)
	{
		if (set > 0 && calib != 1)	 	 { vref = VSET; }
		if (home_ad > 0.5 && calib != 1) { vref = 0; system_fsm_reset(); calib = 1; }
		if (pos_t < set && calib == 1)	 { vref = -VSET; }
		if (pos_t > set && calib == 1)   { vref = 0; set = -1; calib = 0; }
		if (msr >= 0 && msr < NROFT)	 { msr++; }
		motion_ctrl_vpi(vref, omega_ma, &iq_ref);
		//iq_ref = vref;
	}
	if (sysmode_e == SYS_RUN)
	{
		if (msr >= 0 && msr < NROFT) { motion_ctrl_ref(reftype_e, Aref, Fref, &iq_ref); msr++; }
		else	 					 { motion_ctrl_ref(REF_OFF, Aref, Fref, &iq_ref); }
	}

	// DRIVE CONTROL
	if (sysmode_e == SYS_INI || sysmode_e == SYS_RUN)
	{
		drive_ctrl_uw2ab(iu_ad, iw_ad, &ia_ad, &ib_ad);
		drive_ctrl_ab2dq(ia_ad, ib_ad, theta_e, &id_ad, &iq_ad);
		drive_ctrl_zcpi(iq_ref, id_ad, iq_ad, &vd_ref, &vq_ref);
		drive_ctrl_dec(omega_ma, id_ad, iq_ad, &vd_ref, &vq_ref);
		drive_ctrl_dq2ab(vd_ref, vq_ref, theta_e, &va_ref, &vb_ref);
		drive_ctrl_ab2uvw(va_ref, vb_ref, &vu_ref, &vv_ref, &vw_ref);
		motor_inv_pwm(vu_ref, vv_ref, vw_ref, vdc_ad);
	}
	else{ motor_inv_pwm(0, 0, 0, vdc_ad); }



	// TIMER COUNT
	// -----------
	t0 = timer0_read();
	t0_a = ALPHA * (t0 * 17.7777e-3) + (1 - ALPHA) * t0_a;
	if (msr >= 0 && msr < NROFT) { msr++; }
	watch_data_8ch();
}

