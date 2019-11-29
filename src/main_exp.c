/************************************************************************************
MULTI INTERRUPT FIRMWARE
-------------------------
Descr.:		main file for multiple interrupt experiments
Boards:		PE-Expert3, C6713A DSP + System Hardware
System:		Main-file of modular firmware structure
Author:		Thomas Beauduin, University of Tokyo, 2016
*************************************************************************************/
#include "system_fsm.h"
#include "ctrl_math.h"

void system_init(void);
interrupt void system_tint0(void);
interrupt void system_cint5(void);

///////////////////////////////////////////////////////
int mode = 0;
int first = 0;
double t_x = 0.0 ;

float step_size = 0.0;

float xpd[1]  = { 0.0 };
float xQ[1]  = { 0.0 };
float xDOBM[1]  = { 0.0 };
float xDOBL[1]  = { 0.0 };
float ximp[1] = { 0.0 }:

float theta_my_offset = 0.0;
float theta_mx_offset = 0.0;
float omega_my_r = 0.0;

float Dt = 0.0;
float Kt = 0.172668006987608;
float Q_out = 0.0;
float DOBM_out = 0.0;
float DOBL_out = 0.0;
float torque_ref = 0.0;
float imp_ref = 0.0;
float omega_ref = 0.0;

float Kimp=1.5;
float Kwp=1.0;
//////////////////////////////////////////////////////


void main(void)
{
	// SYSTEM INIT
	int_disable();
	system_init();
	int_enable();

	//SYSTEM RUN
	while (1){ system_fsm_mode(); }
}


void system_tint0(void)
{
	unsigned int regs[2];
	time++;

	// SENSOR READ
	motor_enc_read(0, &theta_mx, &omega_mx, &omega_max);
	motor_enc_read(1, &theta_my, &omega_my, &omega_may);
	setup_adc_read(2, &torque_ad, &temp1, &temp2, &temp3);

	// MULTI-INT ON
	regs[0] = CSR;
	regs[1] = IRP;
	int_enable();

	// MOTION CTRL
	if (sysmode_e == SYS_STP){ }
	if (sysmode_e == SYS_INI){ }
	if (sysmode_e == SYS_RUN)
	{
		//if (msr >= 0 && msr < NROFT) { motion_ctrl_ref(1, Aref, Fref, &xref); msr++; }
		//else	 					 { motion_ctrl_ref(REF_OFF, Aref, Fref, &xref); }
		//iq_refx = xref;	// open-loop
		
		
		if (mode == 1){ //////////Proposesd method//////////
		if(first == 0){
			// OFFSET
			theta_my_offset = theta_my;
			theta_mx_offset = theta_mx;
			first = 1;
		}
		
		if( (t_x >= 1.0) && (t_x <= 1.2)) {
			iq_refy = -step_size;
		}
		else{
			iq_refy = 0.0;
		}
		omega_my_r = -omega_my;
		Dt = theta_mx - (-theta_my);
		ctrl_math_output(Cpd[0], xpd, Dpd[0], Dt, iq_refx, 1, 1, 1);
	    ctrl_math_state(Apd[0], xpd, Bpd[0], Dt, xpd, 1, 1);
	    iq_refx = -iq_refx;
	    if (fabsf(*iq_ref) > I_PK) { *iq_ref = sign(*iq_ref) * I_PK; }		// limit torque
	    }
	    
	    if (mode == 2){ //////////Impedance control//////////
	    if(first == 0){
			// OFFSET
			theta_my_offset = theta_my;
			theta_mx_offset = theta_mx;
			first = 1;
		}
		
		if( (t_x >= 1.0) && (t_x <= 1.2)) {
			iq_refy = -step_size;
		}
		else{
			iq_refy = 0.0;
		}
		omega_my_r = -omega_my;
	    // Double DOB
	    float torque_ref = iq_refx * Kt;
		ctrl_math_output(C_Q[0], xQ, D_Q[0], torque_ref, Q_out, 1, 1, 1);
		ctrl_math_state(A_Q[0], xQ, B_Q[0], torque_ref, xQ, 1, 1);
		ctrl_math_output(C_DOBM[0], xDOBM, D_DOBM[0], omega_mx, DOBM_out, 1, 1, 1);
		ctrl_math_state(A_DOBM[0], xDOBM, B_DOBM[0], omega_mx, xDOBM, 1, 1);
		Ts_est = Q_out - DOBM_out;
		ctrl_math_output(C_DOBL[0], xDOBL, D_DOBL[0], omega_my_r, DOBL_out, 1, 1, 1);
		ctrl_math_state(A_DOBL[0], xDOBL, B_DOBL[0], omega_my_r, xDOBL, 1, 1);
		dL_est = DOBL_out - Ts_est;
		
		//model impedance
		ctrl_math_output(C_imp[0], ximp, D_imp[0], dL_est, imp_ref, 1, 1, 1);
		ctrl_math_state(A_imp[0], ximp, B_imp[0], dL_est, ximp, 1, 1);
		iq_refx = (imp_ref - omega_mx) * Kimp;
		if (fabsf(*iq_refx) > I_PK) { *iq_refx = sign(*iq_refx) * I_PK; }		// limit torque	
		}
		
		if (mode == 3){ /////////Velocity P ////////////
		if(t_x >= 2.0){
			omega_ref = step_size;
		}
		else{
			omega_ref = 0.0;
		}
		iq_refx = Kwp * (omega_ref - omega_max);
	    }
	    
		t_x += (TS*1e-6);
	}

	// MULTI-INT OFF
	int_disable();
	CSR = regs[0];
	IRP = regs[1];
	watch_data_8ch();
}


void system_cint5(void)
{
	// SENSOR READ
	motor_enc_elec(0, &theta_ex);
	motor_enc_elec(1, &theta_ey);
	setup_adc_read(0, &vdc_adx, &idc_adx, &iu_adx, &iw_adx);
	setup_adc_read(1, &vdc_ady, &idc_ady, &iu_ady, &iw_ady);

	// CURRENT CONTROL - X&Y AXES
	if (sysmode_e == SYS_INI || sysmode_e == SYS_RUN)
	{
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
		motor_inv_pwm(0, vu_refx, vv_refx, vw_refx, vdc_adx);
		motor_inv_pwm(1, vu_refy, vv_refy, vw_refy, vdc_ady);
	}
	else{ motor_inv_pwm(0, 0, 0, 0, vdc_adx); 
		  motor_inv_pwm(1, 0, 0, 0, vdc_ady); }
}


void system_init(void)
{
	// SENSORS
	watch_init();
	motor_adc_init();
	setup_adc_init();
	motor_enc_init(0);
	motor_enc_init(1);

	// DRIVE CTRL
	int5_init_vector(system_cint5);
	motor_inv_init(0);
	int5_enable_int();

	// MOTION CTRL
	timer0_init(TS);
	timer0_init_vector(system_tint0);
	timer0_start();
	timer0_enable_int();
	system_fsm_init();
}

// NOTES:
// ------

