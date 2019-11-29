/************************************************************************************
MULTI INTERRUPT FIRMWARE
-------------------------
Descr.:        main file for multiple interrupt experiments
Boards:        PE-Expert3, C6713A DSP + System Hardware
System:        Main-file of modular firmware structure
Author:        Thomas Beauduin, University of Tokyo, 2016
*************************************************************************************/

#include "system_fsm.h"
#include "ctrl_math.h"

#include "data/multisine_1_250.h"

int iteration = 0;
float input_gain = 0;

void system_init(void);
interrupt void system_tint0(void);
interrupt void system_cint5(void);

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
        if (msr >= 0 && msr < NROFS*iteration) {
            iq_refx = input_gain*refvec[msr%NROFS];
            iq_refy = 0;
            msr++;
        }
        else{
            iq_refx = 0;
            iq_refy = 0;
        }
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
        current_ctrl_zcpi(0, iq_refx, id_adx, iq_adx, &vd_refx, &vq_refx);
        current_ctrl_zcpi(1, iq_refy, id_ady, iq_ady, &vd_refy, &vq_refy);
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
