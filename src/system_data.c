/************************************************************************************
SYSTEM GLOBAL DATA MODULE
-------------------------
Descr.:		Header with experimental setup global data
Boards:		PE-Expert3, MWPE-C6713A DSP, MW-WAVE2 acquisition
System:		Motor Bench experimental setup (S-PMSM drives)
Author:		Thomas Beauduin, University of Tokyo, March 2016
*************************************************************************************/
 #include "system_data.h"

// GLOBAL SYSTEM VAR
int msr = -1, cnt = 0;				// measurement counters
int set = -1, calib = 0;			// calibration counters
float time = 0.0, t0=0.0, t0_a = 0.0;
enum mode sysmode_e = SYS_STP;

// GLOBAL COMMAND VAR
float Aref = 0.0, Fref = 0.0;
enum ref reftype_e = REF_OFF;

// GLOBAL CTRL VAR
float vref = 0.0, xref = 0.0;
float uptc = 0.0, yptc = 0.0;
float vd_refx = 0.0, vq_refx = 0.0, id_refx = 0.0, iq_refx = 0.0;
float vd_refy = 0.0, vq_refy = 0.0, id_refy = 0.0, iq_refy = 0.0;
enum fb fbtype_e = FB_OFF;
enum ff fftype_e = FF_OFF;
enum dob dobtype_e = DOB_OFF;
enum fric frictype_e = FRIC_OFF;


// GLOBAL MSR VAR
float vdc_adx, idc_adx, theta_ex;
float iu_adx, iw_adx, ia_adx, ib_adx, id_adx, iq_adx;
float vu_adx, vw_adx, va_adx, vb_adx, vd_adx, vq_adx;
float vdc_ady, idc_ady, theta_ey;
float iu_ady, iw_ady, ia_ady, ib_ady, id_ady, iq_ady;
float vu_ady, vw_ady, va_ady, vb_ady, vd_ady, vq_ady;
float va_refx, vb_refx, vu_refx, vv_refx, vw_refx;
float va_refy, vb_refy, vu_refy, vv_refy, vw_refy;

float theta_mx, theta_mox, theta_mdx, omega_mx, omega_max = 0.0;
float theta_my, theta_moy, theta_mdy, omega_my, omega_may = 0.0;
float torque_ad, temp1, temp2, temp3;
int test = 0.0;


