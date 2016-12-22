#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include <signal.h>

#include "tcp_func.h"
#include "io_func.h"


#ifndef DEBUG
//#define DEBUG
#endif //DEBUG

#ifndef DEBUG2
//#define DEBUG2
#endif //DEBUG2

//#define MULTI_CMD

#define TRUE 1
#define FALSE 0
#define NG 0
#define OK 1
#define NEW (45)
#define OLD (23)
#define PI 3.141592654

#ifndef OFF
#define OFF 0
#endif

#ifndef ON
#define ON 1
#endif 

//general
#define DRV_NMB 4
#define STR_NMB 4
#define PTL_NMB 2
#define TTL_MTR_NMB (DRV_NMB+STR_NMB+PTL_NMB)
#define DRV_DRV_NMB(ch) ((ch) + 1) 
#define STR_DRV_NMB(ch) ((ch) + DRV_NMB + 1)
#define PTL_DRV_NMB(ch) ((ch) + DRV_NMB + STR_NMB + 1)
#define PWL_ANG_1 ((float)(45.0/180.0*(float)PI))
#define PWL_ANG_2 ((float)(-45.0/180.0*(float)PI))
#define PWL_ANG_3 ((float)(-45.0/180.0*(float)PI))
#define PWL_ANG_4 ((float)(45.0/180.0*(float)PI))
#define WHL_BAS_DST (0.3)
#define CTR_SPV_NMB (4)
#define FLT_TO_INT_PWM_CFC (10) //float to int change on PWM ratio
#define INT_TO_FLT_PWM_CFC ((float)1.0/10.0)
#define WAIT_STR_ANG_LMT (5.0/180.0*PI) //rad

#define GIGAZERO 1000000000
#define MEGAZERO 1000000
#define KILOZERO 1000
#define FIR_FLT_NMB 40

//resolver relations
#define RU_COUNT_OF_ONE_ROUND 262144 //max value of counter
#define RU_ANGLE_OF_ONE_ROUND 360.0
#define RU_SIZE_OF_COMMAND 6 //data size[byte]
#define RU_ERROR_CODE (-1) 
#define RU_CNT_DIR_CFC_PAN (-1)
#define RU_CNT_DIR_CFC_TLT (-1)
#define RU_TLT_NMN_OFS (0.0)
#define RU_TLT_FIX_ANG_DEG (11.0) //DEG nominal -25 deg
#define RU_OFS_PAN (-224.89 - 4.33 - 54.22 - 4.0 + 4.44) //deg
#define RU_OFS_TLT (-61.21 - 11.50 + 0.0*(float)RU_TLT_FIX_ANG_DEG) //deg
#define RU_TLT_NGT_ANG_LMT_DEG (-15.0 - 0.0*(float)RU_TLT_FIX_ANG_DEG) //deg
#define RU_TLT_PST_ANG_LMT_DEG (60.0 - 0.0*(float)RU_TLT_FIX_ANG_DEG) //deg
#define RU_PAN_NGT_ANG_LMT_DEG (-90.0) //deg
#define RU_PAN_PST_ANG_LMT_DEG (90.0) //deg
#define RU_TLT_ORD (1)
#define RU_PAN_ORD (0)
#define RU_RNG_BUF_SIZ (RU_SIZE_OF_COMMAND+2)
#define RU_TRG_THR_DEG (360/32 * 0.8)
#define RU_TRG_WDT_DEG (20.0)
#define RU_REV_OFS_DEG (360/32)
#define RU_TRG_NSS_REG (3.0) //deg

//DRV motor spec //339152 RE25, 406762 GP26A
#define DRV_MTR_MAX_TRQ_SPD (9690) //rpm
#define DRV_MTR_MIN_TRQ_SPD (10900) //rpm
#define DRV_HDR_GEA_RAT (80)
#define DRV_PLN_GEA_RAT (19)
#define DRV_GEA_RAT (DRV_HDR_GEA_RAT*DRV_PLN_GEA_RAT)
#define DRV_ENC_CNT_BUNSYU (1)
#define DRV_ENC_CNT_PER_RND (128*DRV_ENC_CNT_BUNSYU)
#define DRV_WHL_RDS (0.115)
#define DRV_WHL_DMT ((float)DRV_WHL_RDS*2.0)
#define DRV_NMN_SLP_RAT (0.1) //nominal slippage ratio
#define DRV_LNR_SPD ((float)(DRV_WHL_RDS*DRV_MTR_MAX_TRQ_SPD/DRV_GEA_RAT*2.0*PI/60.0*(1.0-DRV_NMN_SLP_RAT))) //m/s @ duty 100%
#define DRV_MAX_DTY (100.0) //%
#define DRV_LNR_SPD_PER_DTY ((float)(DRV_LNR_SPD/DRV_MAX_DTY))
#define DRV_NSN_DTY_RNG (10.0)
#define DRV_MAX_DST (100.0) //meters
#define DRV_TIM_ADJ_CFC (1/1)
#define DRV_ROT_DIR_1 (1)
#define DRV_ROT_DIR_2 (-1)
#define DRV_ROT_DIR_3 (1)
#define DRV_ROT_DIR_4 (-1)
#define DRV_ACC_DRT (2.0) //sec, driving acceleration duration 

#define DRV_PID_GAIN_P (144.3)
#define DRV_PID_GAIN_I (0.0)
#define DRV_PID_GAIN_D (26.8)
#define DRV_AMP_CFC_NGT_1 (1.0-0.0)
#define DRV_AMP_CFC_NGT_2 (1.0-0.0)
#define DRV_AMP_CFC_NGT_3 (1.0-0.0)
#define DRV_AMP_CFC_NGT_4 (1.0-0.0)
#define DRV_AMP_CFC_PST_1 (1.0-0.0)
#define DRV_AMP_CFC_PST_2 (1.0-0.0)
#define DRV_AMP_CFC_PST_3 (1.0-0.0)
#define DRV_AMP_CFC_PST_4 (1.0-0.0)

//STR motor spec //339152 RE25, 406128 GP26A
#define STR_MTR_MAX_TRQ_SPD (9690) //rpm
#define STR_MTR_MIN_TRQ_SPD (10900) //rpm
#define STR_HRD_GEA_RAT (80)
#define STR_PLN_GEA_RAT (71)
#define STR_GEA_RAT (STR_HD_GEA_RAT*STR_PLN_GEA_RAT)
#define STR_ENC_CNT_BUNSYU (1)
#define STR_ENC_CNT_PER_RND (128*STR_ENC_CNT_BUNSYU)
//#1 gain
//#define STR_PID_GAIN_P (144.3)
//#define STR_PID_GAIN_I (0.0)
//#define STR_PID_GAIN_D (26.8)
//#2 gain
#define STR_PID_GAIN_P (561.3358)
#define STR_PID_GAIN_I (0.0)
#define STR_PID_GAIN_D (52.8796)

#define STR_PTN_ROT_DIR (-1.0)
#define STR_MTR_ROT_DIR (1.0)
#define STR_PWM_NSN_LMT_DEG (0.01) //deg
#define STR_PWM_NSN_LMT_RAD ((float)STR_PWM_NSN_LMT_DEG/180.0*PI) //rad
#define STR_ANG_LMT_RAD ((float)PI/2.0)
#define STR_ANG_LMT_DEG ((float)90.0)

//IncOrder spec
#define STR_PTN_MAX_VLT (10.0)
#define STR_PTN_MIN_VLT (0.5)
#define STR_PTN_MAX_ANG (360.0)
#define STR_PTN_OFS_VLT ((float)((STR_PTN_MAX_VLT + STR_PTN_MIN_VLT)/2.0)) //zero point
#define STR_PTN_CRC_VLT_TO_ANG_DEG ((float)(STR_PTN_MAX_ANG/(STR_PTN_MAX_VLT - STR_PTN_MIN_VLT)))
#define STR_PTN_CRC_VLT_TO_ANG_RAD ((float)STR_PTN_CRC_VLT_TO_ANG_DEG/180.0*PI)

///////////////////
//need to measure//
///////////////////
#define STR_PTN_OFS_VLT_1 (7.053750 - (1.0/(float)STR_PTN_CRC_VLT_TO_ANG_DEG)) //V
#define STR_PTN_OFS_VLT_2 (3.456375 + (4.0/(float)STR_PTN_CRC_VLT_TO_ANG_DEG)) //V
#define STR_PTN_OFS_VLT_3 (3.425625 + 0.0) //V
#define STR_PTN_OFS_VLT_4 (7.056188 + (2.0/(float)STR_PTN_CRC_VLT_TO_ANG_DEG)) //V
///////////////////
//need to measure//
///////////////////

//US digital spec
/*
#define STR_PTN_MAX_VLT (5.0)
#define STR_PTN_MIN_VLT (0.0)
#define STR_PTN_MAX_ANG (360.0)
#define STR_PTN_OFS_VLT ((float)((STR_PTN_MAX_VLT + STR_PTN_MIN_VLT)/2.0)) //zero point
#define STR_PTN_CRC_VLT_TO_ANG ((float)(STR_PTN_MAX_ANG/(STR_PTN_MAX_VLT - STR_PTN_MIN_VLT)))
*/

//PTL motor spec //339152 RE25, 406762 GP26A
#define PTL_MTR_MAX_TRQ_SPD (9690) //rpm
#define PTL_MTR_MIN_TRQ_SPD (10900) //rpm
#define PTL_HRD_GEA_RAT (80)
#define PTL_PLN_GEA_RAT (19)
#define PTL_GEA_RAT (PTL_HD_GEA_RAT*PTL_PLN_GEA_RAT)
#define PTL_ENC_CNT_BUNSYU (1)
#define PTL_ENC_CNT_PER_RND (128*PTL_ENC_CNT_BUNSYU)
#define PTL_PAN_MTR_ROT_DIR (-1)
#define PTL_TLT_MTR_ROT_DIR (1)
//#1 good gain
#define PTL_PAN_PID_GAIN_P (126.78)
#define PTL_PAN_PID_GAIN_I (0.0)
#define PTL_PAN_PID_GAIN_D (52.88)
//#2g ain 
//#define PTL_PAN_PID_GAIN_P (196.9951)
//#define PTL_PAN_PID_GAIN_I (0.0)
//#define PTL_PAN_PID_GAIN_D (65.9152)
//#3 gain
//#define PTL_PAN_PID_GAIN_P (282.6163)
//#define PTL_PAN_PID_GAIN_I (0.0)
//#define PTL_PAN_PID_GAIN_D (78.950)

//#1 good gain
#define PTL_TLT_PID_GAIN_P (126.78)
#define PTL_TLT_PID_GAIN_I (0.0)
#define PTL_TLT_PID_GAIN_D (52.88)
//#2 gain
//#define PTL_TLT_PID_GAIN_P (282.6163)
//#define PTL_TLT_PID_GAIN_I (0.0)
//#define PTL_TLT_PID_GAIN_D (78.950)

#define PTL_PWM_NSN_LMT_DEG (0.01) //deg
#define PTL_PWM_NSN_LMT_RAD ((float)(PTL_PWM_NSN_LMT_DEG/180.0*PI)) //rad
#define PTL_TLT_ANG_PST_LMT_RAD ((float)(RU_TLT_PST_ANG_LMT_DEG*PI/180.0))
#define PTL_TLT_ANG_NGT_LMT_RAD ((float)(RU_TLT_NGT_ANG_LMT_DEG*PI/180.0))
#define PTL_PAN_ANG_PST_LMT_RAD ((float)(RU_PAN_PST_ANG_LMT_DEG*PI/180.0))
#define PTL_PAN_ANG_NGT_LMT_RAD ((float)(RU_PAN_NGT_ANG_LMT_DEG*PI/180.0))

//serial relations
#define SERIAL_PORT_0 "/dev/ttyS0" //RS232C, for Resolver of pan
#define SERIAL_PORT_1 "/dev/ttyS1" //RS232C, for Resolver of tilt
#define SERIAL_PORT_2 "/dev/ttyS2" //N.A.
#define SERIAL_PORT_3 "/dev/ttyS3" //N.A.
#define SERIAL_PORT_4 "/dev/ttyS4" //Force torque sensor #1
#define SERIAL_PORT_5 "/dev/ttyS5" //Force torque sensor #2
#define SERIAL_PORT_6 "/dev/ttyS6" //Force torque sensor #3
#define SERIAL_PORT_7 "/dev/ttyS7" //Force torque sensor #4
//#define SERIAL_PORT_USB0 "/dev/ttyUSB0" //motor driver
#define SERIAL_PORT_USB0 "/dev/ttyMC1" //motor driver
//#define SERIAL_PORT_USB1 "/dev/ttyUSB1" //motor driver
#define SERIAL_PORT_USB1 "/dev/ttyMC2" //motor driver
#define SERIAL_BAUDRATE_MDV B115200
#define SERIAL_BAUDRATE_RU B38400
#define SERIAL_BAUDRATE_NMN B115200
#define SBUF_CMD_SIZE 255
#define SBUF_TLM_SIZE 255

//motor driver relations
#define MDV_CMD_ENC_CLR 0x15 //encoder clear
#define MDV_CMD_DTY_SET 0x21 //duty set
#define MDV_CMD_TLM_SND 0x22 //control parameter get
#define MDV_CMD_NON	    0x00 
#define MDV_CMD_ENC_CLR_DAT_SIZ 0x01
#define MDV_CMD_DTY_SET_DAT_SIZ 0x05
#define MDV_CMD_TLM_SND_DAT_SIZ 0x02
#define MDV_SCN_CMD_STT 0
#define MDV_SCN_CMD_DTY 1000
#define MDV_SCN_CMD_TMP 2000
#define MDV_SCN_CMD_CRR 3000
#define MDV_SCN_CMD_ABS_ENC_CNT 4000
#define MDV_SCN_CMD_RLT_ENC_CNT 5000
#define MDV_TLM_ENC_CLR 0x95
#define MDV_TLM_DTY_SET 0xA1
#define MDV_TLM_TLM_SND 0xA2
#define MDV_TLM_ENC_CLR_DAT_SIZ 0x01
#define MDV_TLM_DTY_SET_DAT_SIZ 0x05
#define MDV_TLM_TLM_SND_DAT_SIZ_00 3
#define MDV_TLM_TLM_SND_DAT_SIZ_01 4
#define MDV_TLM_TLM_SND_DAT_SIZ_02 4
#define MDV_TLM_TLM_SND_DAT_SIZ_03 4
#define MDV_TLM_TLM_SND_DAT_SIZ_04 6
#define MDV_TLM_TLM_SND_DAT_SIZ_05 6

//thread relations
#define RU_ACQ_RATE (50)//Hz
#define CTR_SPV_RATE (20)//Hz
#define DRV_CTR_RATE (10)//Hz
#define STR_CTR_RATE (10)//Hz
#define PTL_CTR_RATE (10)//Hz
#define FIL_CTR_RATE (5)//Hz
#define MTR_HK_RATE (4)//Hz
#define SERIAL_OUT_RATE (100) //Hz

#define THR_NMB 100
#define _POSIX_SOURCE 1


//global variables
extern add ad_data[AD_CHN_NMB];

volatile int STOP=FALSE;
int serial_fd[10], cmd_fd;                            //
struct termios oldtio[10], newtio[10];    //
pthread_t thrd_num[THR_NMB];
pthread_mutex_t pan_tlt_mutex[2];
static int rslv_ring_length[2];
char rslv_ring_buf[2][RU_RNG_BUF_SIZ];
FILE *fp_ad, *fp_rslv_pan, *fp_rslv_tlt, *fp_hk;

typedef struct{
	float str_trg[STR_NMB];
	float str_pos[STR_NMB];	
	float str_ang[STR_NMB];
	float str_crt[STR_NMB];
	int str_dty[STR_NMB]; //steering driver duty	
	
	float drv_trg[DRV_NMB];
	float drv_pos[DRV_NMB];
	float drv_ang[DRV_NMB];
	float drv_crt[DRV_NMB];
	int drv_dty[DRV_NMB];
	
	float ptl_trg[PTL_NMB];
	float ptl_pos[PTL_NMB];
	float ptl_ang[PTL_NMB];
	float ptl_crt[PTL_NMB];
	int ptl_dty[PTL_NMB];
	
	float dst_trg;
	float thrm_trg;
	int elec_busv;
	int max_pwm;  //-1000 ~ 1000, nominal 500
	int cmd_c;    //d~V
	
	int lst_cmd_num; //last command number
	int prv_cmd_num; //previous command number	
	
	//telemetry relations
	unsigned short mdv_tmp_int[TTL_MTR_NMB];
	short ctr_dty_int[TTL_MTR_NMB];
	short mtr_crr_int[TTL_MTR_NMB];
	int abs_enc_cnt[TTL_MTR_NMB];
	int rlt_enc_cnt[TTL_MTR_NMB];
	
	float mtr_crr_flt[TTL_MTR_NMB];
	float ctr_dty_flt[TTL_MTR_NMB];
	float mdv_tmp_flt[TTL_MTR_NMB];
	
}motor_data;

motor_data rcv_eth, snd_srl, act_spv; 

struct my_time {
	time_t tim; // yyyymmddhhmmss
	long msec;  // milli sec
};

//motor driver number vs drive/steering/pan tilt control
int mdv_ch[DRV_NMB+STR_NMB+PTL_NMB] = {2, 8, 3, 10, 4, 11, 6, 12, 1, 7}; //#5,9 out
int mdv_srl_ch[DRV_NMB+STR_NMB+PTL_NMB] = {0, 1, 0, 1, 0, 1, 0, 1, 0, 1};

//prototype declration
void rcg_mdv_tlm(int ch, motor_data *tlm, char *buf, int sbuf_siz);


//tools
float fir_scd_filt(float crt_val, int flt_num){
	
	static float mem_dat[FIR_FLT_NMB][3];
	static int cnt=0;
	int i=0, j=0;
	
	if(cnt==0){
		for(i=0;i<FIR_FLT_NMB;i++){
			for(j=0;j<3;j++){
				mem_dat[i][j] = 0.0;
			}
		}
		
		cnt++;
	}
	
	mem_dat[flt_num][2]=mem_dat[flt_num][1];
	mem_dat[flt_num][1]=mem_dat[flt_num][0];
	mem_dat[flt_num][0]=crt_val;
	
	return (mem_dat[flt_num][2]+2*mem_dat[flt_num][1]+mem_dat[flt_num][0])/4.0;
}



//serial relations
unsigned short serial_checksum(char *addr, int leng){
	
		int nleft=leng; //1byte order
		int sum=0;
		char *w=addr;
		unsigned short answer=0;
	
		while(nleft > 0)
		{
			sum+=*w++;
			nleft-=1;
		}
		sum=(sum>>16) + (sum&0xffff); //addition of upper and lower value
		sum+=(sum>>16); //adding carry value
		answer=sum; //inversion of value
		return(answer);
}


void init_serial(int srl_num){
		
		int res;
		int fd;
		struct termios old_dummy;
				
		if(srl_num==0){ //resolver 1
			fd = open(SERIAL_PORT_0, O_RDWR | O_NOCTTY | O_NONBLOCK);
		}else if(srl_num==1){ //resolver 2
			fd = open(SERIAL_PORT_1, O_RDWR | O_NOCTTY | O_NONBLOCK);
		}else if(srl_num==2){
			fd = open(SERIAL_PORT_2, O_RDWR | O_NOCTTY | O_NONBLOCK);
		}else if(srl_num==3){
			fd = open(SERIAL_PORT_3, O_RDWR | O_NOCTTY | O_NONBLOCK);
		}else if(srl_num==4){
			fd = open(SERIAL_PORT_4, O_RDWR | O_NOCTTY | O_NONBLOCK);
		}else if(srl_num==5){
			fd = open(SERIAL_PORT_5, O_RDWR | O_NOCTTY | O_NONBLOCK);
		}else if(srl_num==6){
			fd = open(SERIAL_PORT_6, O_RDWR | O_NOCTTY | O_NONBLOCK);
		}else if(srl_num==7){
			fd = open(SERIAL_PORT_7, O_RDWR | O_NOCTTY | O_NONBLOCK);
		}else if(srl_num==8){ //motor driver, remove non_block
			fd = open(SERIAL_PORT_USB0, O_RDWR | O_NOCTTY | O_NONBLOCK);
			//fd = open(SERIAL_PORT_USB0, O_RDWR | O_NOCTTY);
			usleep(2000);
		}else if(srl_num==9){ //motor driver, remove non_block
			fd = open(SERIAL_PORT_USB1, O_RDWR | O_NOCTTY | O_NONBLOCK);
			//fd = open(SERIAL_PORT_USB1, O_RDWR | O_NOCTTY);
			usleep(2000);
		}else{
			fd = open(SERIAL_PORT_0, O_RDWR | O_NOCTTY | O_NONBLOCK);
		}		
		//fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);	 
	    
	    //setup global variables
	    serial_fd[srl_num] = fd;
	    
	    if(fd<0){
			printf("fail to open serial for driver #%d\n", srl_num);
			//return;  // added by otsu
		}else{
			printf("open serial port #%d \n", srl_num);
		}
			
	    //res=tcgetattr(fd, &oldtio[srl_num]);       // 現在のシリアルポートの設定を待避させる
	    res=tcgetattr(fd, &old_dummy);
	    if(res) printf("tcgetattr error \n");
	
	    memset(&newtio[srl_num], 0, sizeof(newtio[srl_num])); 
	    newtio[srl_num] = old_dummy;                  // ポートの設定をコピー
	    newtio[srl_num].c_cflag = CS8 | CLOCAL | CREAD; 
	    
	    if((srl_num==8) || (srl_num==9)){
			newtio[srl_num].c_iflag = ICRNL | IGNPAR;
			newtio[srl_num].c_cc[VMIN]     = 1;
			newtio[srl_num].c_cc[VEOF]     = 0;
			newtio[srl_num].c_cc[VEOL]     = 0; 
			newtio[srl_num].c_oflag = 0;
			newtio[srl_num].c_lflag = 0;
		}
	    //newtio.c_oflag = 0;
	    //newtio.c_lflag = 0;
	
	    //newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */ 
	    //newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
	    //newtio.c_cc[VERASE]   = 0;     /* del */
	    //newtio.c_cc[VKILL]    = 0;     /* @ */
	    //newtio.c_cc[VEOF]     = 0;     /* Ctrl-d */
	    newtio[srl_num].c_cc[VTIME]    = 0;     /* キャラクタ間タイマを使わない */
	    //newtio.c_cc[VMIN]     = 1;     /* 1文字来るまで，読み込みをブロックする */
	    //newtio.c_cc[VSWTC]    = 0;     /* '\0' */
	    //newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */ 
	    //newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
	    //newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
	    //newtio.c_cc[VEOL]     = 0;     /* '\0' */
	    //newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
	    //newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
	    //newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
	    //newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
	    //newtio.c_cc[VEOL2]    = 0;     /* '\0' */
	
		if((srl_num==8) || (srl_num==9)){
		    res = cfsetispeed(&newtio[srl_num], SERIAL_BAUDRATE_MDV);
		    if(res < 0) printf("cfsetspeed error \n");
		    
		    res = cfsetospeed(&newtio[srl_num], SERIAL_BAUDRATE_MDV);
		    if(res < 0) printf("cfsetspeed error \n");
		}else if((srl_num==0) || (srl_num==1)){
			res = cfsetispeed(&newtio[srl_num], SERIAL_BAUDRATE_RU);
		    if(res < 0) printf("cfsetspeed error \n");
		    
		    res = cfsetospeed(&newtio[srl_num], SERIAL_BAUDRATE_RU);
		    if(res < 0) printf("cfsetspeed error \n");
		}else{
		    res = cfsetispeed(&newtio[srl_num], SERIAL_BAUDRATE_NMN);
		    if(res < 0) printf("cfsetspeed error \n");
		    
		    res = cfsetospeed(&newtio[srl_num], SERIAL_BAUDRATE_NMN);
		    if(res < 0) printf("cfsetspeed error \n");
		}
		
	    res = tcflush(fd, TCIFLUSH);
	    if(res < 0) printf("tcflush error \n");
		
	    res = tcsetattr(fd, TCSANOW, &newtio[srl_num]);       // ポートの設定を有効にする
	    if(res) printf("tcsetattr error \n");
	    
	    oldtio[srl_num] = old_dummy;
	
}


static struct my_time *get_now_tm(){
	
	struct my_time *qt;
	struct tm *tmp;
	struct timeval tv;

	qt=(struct my_time*)malloc(sizeof(struct my_time));
	
	if(qt == NULL)return NULL;
	
	gettimeofday(&tv,NULL);
	tmp=localtime(&tv.tv_sec);
	qt->tim=mktime(tmp);
	qt->msec=tv.tv_usec/1000;

	return(qt);
}

static inline long my_tm_cmptime(struct my_time* now_t,struct my_time* prev_t){
	long diff_sec;
	long diff_msec;
	diff_sec=difftime(now_t->tim,prev_t->tim);
	diff_msec=now_t->msec-prev_t->msec;

	return(diff_sec*1000+diff_msec);
}

void *make_file(void *arg){ //making save file data
	
	struct timespec bfr_clck, aft_clck;
	struct timespec req, res_t;
	int time_usec_bfr=0, time_usec_aft=0, i=0;
		
	printf("Filing thread start \n");
	
	clock_gettime(CLOCK_MONOTONIC, &bfr_clck);
	clock_gettime(CLOCK_MONOTONIC, &aft_clck);
	req.tv_sec = 0;
	req.tv_nsec = 0;
		
	printf("fprintf start \n");

	fprintf(fp_hk,"month,day,hour,min,sec,pan[rad],tlt[rad],str_1[rad],str_2[rad],str_3[rad],str_4[rad],drv_1[rad],drv_2[rad],drv_3[rad],drv_4[rad]\n");
	
	for(;;){
				
		///////////////////////////////////////////
		//start clock//
		if(clock_gettime(CLOCK_MONOTONIC, &bfr_clck) !=0){
			printf("error: clock_gettime() \n");
		}		
		time_usec_bfr = (int)(bfr_clck.tv_nsec)/KILOZERO;
		//end start clock//		
		///////////////////////////////////////////
			
		////////////
		//userland//
		////////////
		
		//printf("Filing for loop \n");
		
		{//time save
			time_t timer;
			struct tm *t_st;
			static int prv_tm_sec=-1, nn=0;
			
			time(&timer);
			t_st = localtime(&timer);
					
			//fprintf(sns->var.fp[n],"%4.3lf\t", (var_tim-kep_tim));
			fprintf(fp_hk,"%1d,%1d,%1d,%1d,",
			t_st->tm_mon+1,t_st->tm_mday, 
			t_st->tm_hour, t_st->tm_min);
			
			if(prv_tm_sec != (t_st->tm_sec)){
				nn=0;
			}else{
				nn++;
			}
			fprintf(fp_hk, "%2.4f,",(float)(t_st->tm_sec)+(float)nn*(1.0/(float)FIL_CTR_RATE)); //msec		
			
			prv_tm_sec = t_st->tm_sec;
		}
		
		fprintf(fp_hk, "%3.4f,",act_spv.ptl_ang[0]); //pan
		fprintf(fp_hk, "%3.4f,",act_spv.ptl_ang[1]); //tlt
		
		for(i=0;i<STR_NMB;i++){
			fprintf(fp_hk, "%3.4f,",act_spv.str_ang[i]); //rad
		}

		for(i=0;i<DRV_NMB;i++){
			fprintf(fp_hk, "%3.4f,",act_spv.drv_ang[i]); //rad
		}

		fprintf(fp_hk, "\n");
		
		////////////
		//user end//
		////////////
		
		///////////////////////////////////////////
		//end clock//
		clock_gettime(CLOCK_MONOTONIC, &aft_clck);
		time_usec_aft = (int)(aft_clck.tv_nsec)/KILOZERO;
				
		if(time_usec_aft < time_usec_bfr){
			req.tv_nsec = GIGAZERO/FIL_CTR_RATE - KILOZERO*( 1 * MEGAZERO + time_usec_aft - time_usec_bfr);
		}else{
			req.tv_nsec = GIGAZERO/FIL_CTR_RATE - KILOZERO*( time_usec_aft - time_usec_bfr );
		}		
		
		if(req.tv_nsec > 0){
			nanosleep(&req, &res_t);
		}
		//end end clock//				
		///////////////////////////////////////////
		
#ifdef DEBUG
		if(time_usec_aft < time_usec_bfr){
			printf("[make_file] (irr) total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr )/KILOZERO, ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr ));
		}else{
			printf("[make_file] total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( time_usec_aft - time_usec_bfr )/KILOZERO, ( time_usec_aft - time_usec_bfr ));
		}
#endif	//DEBUG	
		
		
	}//for(;;)
	
}


void close_serial(int srl_num){
	    ioctl(serial_fd[srl_num], TCSETS, &oldtio[srl_num]);       // ポートの設定を元に戻す
	    close(serial_fd[srl_num]);                        // デバイスのクローズ
}	

void close_files(){
	
	fclose(fp_ad);
	fclose((fp_rslv_pan));
	fclose((fp_rslv_tlt));
	fclose(fp_hk);

}

void close_thread_function(){

	int i=0;
	
	for(i=0;i<2;i++){
		pthread_mutex_destroy(&pan_tlt_mutex[i]);
	}
}

void sig_fnc(){
	
	cmd_exe('f',0.0);
	sleep(1);
	close_thread_function();
	close_files();
	close_serial(0);
	close_serial(1);
	close_serial(8);
	close_serial(9);
	
	close(cfd);
	printf("\n Close: all done!! \n");
	
	exit(0);
}




void *get_rslv_dat_pan(void *arg){ //0:pan , 1:tlt

/*	struct timespec bfr_clck, aft_clck;
	struct timespec req, res_t;
	int time_usec_bfr=0, time_usec_aft=0;
*/
	char buf[SBUF_TLM_SIZE];
	int res=0, i=0;
	static int wp=0;
	static int pan_tlt_num=0;

	printf(" get_rslv_pan_dat thread start \n");	
		
	memset(&rslv_ring_length[pan_tlt_num], 0 ,sizeof(int));
	memset(&rslv_ring_buf[pan_tlt_num][0],0,RU_RNG_BUF_SIZ);

/*	clock_gettime(CLOCK_MONOTONIC, &bfr_clck);
	clock_gettime(CLOCK_MONOTONIC, &aft_clck);
	req.tv_sec = 0;
	req.tv_nsec = 0;
*/	
	for(;;){
				
		///////////////////////////////////////////
		//start clock//
/*		if(clock_gettime(CLOCK_MONOTONIC, &bfr_clck) !=0){
			printf("error: clock_gettime() \n");
		}		
		time_usec_bfr = (int)(bfr_clck.tv_nsec)/KILOZERO;
*/		//end start clock//		
		///////////////////////////////////////////

			
		////////////
		//userland//
		////////////
		res = read(serial_fd[pan_tlt_num], buf, SBUF_TLM_SIZE);
		
		/*if(res==0){
			//printf("read 0 (#%d) \n", pan_tlt_num);
			continue;
		}else if(res<0){
			//printf("read error in the function of get resolver data \n");
			continue;
		}*/
		
		if(res>0){
			pthread_mutex_lock( &pan_tlt_mutex[pan_tlt_num]);
			
			for(i=0;i<res;i++){
				rslv_ring_buf[pan_tlt_num][wp] = buf[i];
				wp = (wp+1)%RU_RNG_BUF_SIZ;
				rslv_ring_length[pan_tlt_num]++;
			}
			
			pthread_mutex_unlock( &pan_tlt_mutex[pan_tlt_num]);
			
			//printf("write pan data wp = %d: rslv_ring_length = %d \n", wp, rslv_ring_length[pan_tlt_num]);
			
		}
		////////////
		//user end//
		////////////

		
		///////////////////////////////////////////
		//end clock//
/*		clock_gettime(CLOCK_MONOTONIC, &aft_clck);
		time_usec_aft = (int)(aft_clck.tv_nsec)/KILOZERO;
				
		if(time_usec_aft < time_usec_bfr){
			req.tv_nsec = GIGAZERO/RU_ACQ_RATE - KILOZERO*( 1 * MEGAZERO + time_usec_aft - time_usec_bfr);
		}else{
			req.tv_nsec = GIGAZERO/RU_ACQ_RATE - KILOZERO*( time_usec_aft - time_usec_bfr );
		}		
		
		if(req.tv_nsec > 0){
			nanosleep(&req, &res_t);
		}
		//end end clock//				
		///////////////////////////////////////////
		
#ifdef DEBUG
		if(time_usec_aft < time_usec_bfr){
			printf("[pan hdl] (irr) total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr )/KILOZERO, ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr ));
		}else{
			printf("[pan hdl] total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( time_usec_aft - time_usec_bfr )/KILOZERO, ( time_usec_aft - time_usec_bfr ));
		}
#endif	//DEBUG	
*/

	}
		

}


void *get_rslv_dat_tlt(void *arg){ //0:pan , 1:tlt

	struct timespec bfr_clck, aft_clck;
	struct timespec req, res_t;
	int time_usec_bfr=0, time_usec_aft=0;

	char buf[SBUF_TLM_SIZE];
	int res=0, i=0;
	static int wp=0;
	static int pan_tlt_num=1; //different point

	memset(&rslv_ring_length[pan_tlt_num], 0 ,sizeof(int));
	memset(&rslv_ring_buf[pan_tlt_num][0],0,RU_RNG_BUF_SIZ);

	printf(" get_rslv_tlt_dat thread start \n");	
	
/*	clock_gettime(CLOCK_MONOTONIC, &bfr_clck);
	clock_gettime(CLOCK_MONOTONIC, &aft_clck);
	req.tv_sec = 0;
	req.tv_nsec = 0;
*/	
	for(;;){
				
		///////////////////////////////////////////
		//start clock//
/*		if(clock_gettime(CLOCK_MONOTONIC, &bfr_clck) !=0){
			printf("error: clock_gettime() \n");
		}		
		time_usec_bfr = (int)(bfr_clck.tv_nsec)/KILOZERO;
*/		//end start clock//		
		///////////////////////////////////////////

			
		////////////
		//userland//
		////////////
		res = read(serial_fd[pan_tlt_num], buf,SBUF_TLM_SIZE);
		
		/*
		if(res==0){
			//printf("read 0 (#%d) \n", pan_tlt_num);
			//continue;
		}else if(res<0){
			//printf("read error in the function of get resolver data \n");
			//continue;
		}*/
		
		if(res>0){
			pthread_mutex_lock( &pan_tlt_mutex[pan_tlt_num]);
			
			for(i=0;i<res;i++){
				rslv_ring_buf[pan_tlt_num][wp] = buf[i];
				wp = (wp+1)%RU_RNG_BUF_SIZ;
				rslv_ring_length[pan_tlt_num]++;
			}
			
			pthread_mutex_unlock( &pan_tlt_mutex[pan_tlt_num]);
			
			//printf("write tlt data wp = %d: rslv_ring_length = %d \n", wp, rslv_ring_length[pan_tlt_num]);
		}
		////////////
		//user end//
		////////////

		
		///////////////////////////////////////////
		//end clock//
/*		clock_gettime(CLOCK_MONOTONIC, &aft_clck);
		time_usec_aft = (int)(aft_clck.tv_nsec)/KILOZERO;
				
		if(time_usec_aft < time_usec_bfr){
			req.tv_nsec = GIGAZERO/RU_ACQ_RATE - KILOZERO*( 1 * MEGAZERO + time_usec_aft - time_usec_bfr);
		}else{
			req.tv_nsec = GIGAZERO/RU_ACQ_RATE - KILOZERO*( time_usec_aft - time_usec_bfr );
		}		
		
		if(req.tv_nsec > 0){
			nanosleep(&req, &res_t);
		}
		//end end clock//				
		///////////////////////////////////////////
		
#ifdef DEBUG
		if(time_usec_aft < time_usec_bfr){
			printf("[tilt hdl] (irr) total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr )/KILOZERO, ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr ));
		}else{
			printf("[tilt hdl] total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( time_usec_aft - time_usec_bfr )/KILOZERO, ( time_usec_aft - time_usec_bfr ));
		}
#endif	//DEBUG	
*/

	}

}


float cnv_cnt_to_ang(int ncount, int pan_tlt_num){
	
	float temp=0;
	
	if(pan_tlt_num==RU_PAN_ORD){
		temp = (float)(RU_CNT_DIR_CFC_PAN * ncount)/((float)RU_COUNT_OF_ONE_ROUND - 1.0)
		*(float)RU_ANGLE_OF_ONE_ROUND - (float)RU_OFS_PAN;
	}else if(pan_tlt_num==RU_TLT_ORD){
		temp = (float)(RU_CNT_DIR_CFC_TLT * ncount)/((float)RU_COUNT_OF_ONE_ROUND - 1.0)
		*(float)RU_ANGLE_OF_ONE_ROUND - (float)RU_OFS_TLT;			
	}
	
	return temp;

}


long get_rslv_val(char* rdata, int pan_tlt_num){
	
	static char buf[RU_SIZE_OF_COMMAND+1];
	static int pcount[2]={0,0};
	//long tmp_data[RU_SIZE_OF_COMMAND];
	//int i=0;
	
	if(pcount[pan_tlt_num]==0)
		memset(buf, 0, RU_SIZE_OF_COMMAND);
	
	//if((rdata[0]=='\r') || (rdata[0]=='\n' || (rdata[0]==0x0a))){
	if(rdata[0]==0x0a){		
		pcount[pan_tlt_num]=0;
		return RU_ERROR_CODE;
	}
	
	buf[pcount[pan_tlt_num]] = rdata[0];
	pcount[pan_tlt_num]++;
	//printf("%x ", rdata[0]);
	
	if(pcount[pan_tlt_num] == RU_SIZE_OF_COMMAND){
		
		//printf("/%d \n",pan_tlt_num);
		
		buf[RU_SIZE_OF_COMMAND] = '\n';
		pcount[pan_tlt_num] = 0;
		
		//printf("atol date = %ld (%d) \n", atol(&buf[0]), pan_tlt_num);
		return atol(&buf[0]);
		
	}else{
		
		return RU_ERROR_CODE;
	}
	
}


int rslv_trig_dtc(float crt_ang, int pan_tlt_num){ //resolver date jump up detection. 
	
	static float prv_ang[PTL_NMB]={0.0,0.0}; //degree
	
	if(fabsf(prv_ang[pan_tlt_num] - crt_ang)>(float)RU_TRG_THR_DEG){
		if((prv_ang[pan_tlt_num] - crt_ang)>0.0){//downward 
			prv_ang[pan_tlt_num] = crt_ang;
			return 1;
		}else if((prv_ang[pan_tlt_num] - crt_ang)<0.0){//upward 
			prv_ang[pan_tlt_num] = crt_ang;
			return -1;
		}else{
			return 0;
		}
		
	}else{
		prv_ang[pan_tlt_num] = crt_ang;
		return 0;
	}
	
}


float rev_rslv_sig(float crt_ang_deg, int pan_tlt_num){
	
	float rev_sig=0.0;
	int cnt=0, ptn;
	static int amp[PTL_NMB]={0, 0};
	static int crt_flg[PTL_NMB]={0, 0}; //flagment for in(1) or out(0)
	static float trig_point[PTL_NMB] = {0.0, 0.0};
	
	ptn = pan_tlt_num;
	
	//detection trigger
	//cnt=rslv_trig_dtc(crt_ang_deg, ptn);
	
	//start flagment
	if((crt_flg[ptn]==0) && (cnt != 0)){
		crt_flg[ptn] = 1;
		amp[ptn] = cnt;
		trig_point[ptn] = crt_ang_deg; //deg
		//printf("\n Trigger in at %f deg\n\n", trig_point);
	}else if((crt_flg[ptn]==1) && (cnt != 0)){
		crt_flg[ptn] = 0;
		amp[ptn] = 0;
		//printf("\n Trigger out \n\n");
	}
	
	//duration check
	if((fabsf(crt_ang_deg - trig_point[ptn]) > (float)RU_TRG_WDT_DEG) && (crt_flg[ptn]==1)){
		crt_flg[ptn] = 0;
		amp[ptn] = 0;
		//printf("\n Trigger out by duration over \n\n");
	}
	
	//nonsensitive area
	if(fabsf(crt_ang_deg - 0.0) < (float)RU_TRG_NSS_REG){
		crt_flg[ptn] = 0;
		amp[ptn] = 0;
		//printf("\n Trigger out by nonsensitive area \n\n");
	}	
	
	//revision of data
	if(crt_flg[ptn]==1){
		rev_sig = crt_ang_deg + amp[ptn]* (float)RU_REV_OFS_DEG; 		
	}else{
		rev_sig = crt_ang_deg;
	}
	
	return rev_sig;
}


float get_rslv(int pan_tlt_num){
	
	long rslv_cnt=RU_ERROR_CODE;
	float rslv_ang=628.0;
	char buf;
	static int rslv_read_ptr=0;
	int i=0;
	
	//static int ptl_rot_dir=0; // 0: negative, 1:positive
	
	while(1){

		//if(rslv_ring_length[pan_tlt_num]>0){
			//pthread_mutex_lock( &pan_tlt_mutex[pan_tlt_num]);
			//rslv_ring_length[pan_tlt_num]--;
			//buf = (char)rslv_ring_buf[pan_tlt_num][rslv_read_ptr];
			//pthread_mutex_unlock( &pan_tlt_mutex[pan_tlt_num]);
					
			//rslv_cnt = get_rslv_val(&buf, pan_tlt_num);
			
			rslv_cnt = get_rslv_val(&rslv_ring_buf[pan_tlt_num][rslv_read_ptr], pan_tlt_num);
			rslv_read_ptr = (rslv_read_ptr+1)%RU_RNG_BUF_SIZ;

		//}	
		
		if(rslv_cnt != RU_ERROR_CODE){
			break;
		}
	}
	
	rslv_ang = (float)cnv_cnt_to_ang((int)rslv_cnt, pan_tlt_num); //deg
	
	rslv_ang = rev_rslv_sig(rslv_ang, pan_tlt_num); //deg
	
	//printf("Current pan/tl[%d] angle = %3.3f deg\n", pan_tlt_num, rslv_ang);
	//data filing
	/*if(pan_tlt_num==0){
		fprintf(fp_rslv_pan, "%3.3f\n", rslv_ang); //deg
	}else if(pan_tlt_num==1){
		fprintf(fp_rslv_tlt, "%3.3f\n", rslv_ang); //deg
	}*/
	

	//limit 0 to 360 deg >> -180 to 180 deg
	if((rslv_ang > 180.0)){
		rslv_ang = -360.0 + rslv_ang; //deg
	}else if((rslv_ang < -180.0)){
		rslv_ang = 360.0 + rslv_ang; //deg
	}
	
	return(rslv_ang*(float)PI/180.0); //rad

}


void wait_str_ang_zero(){
	
	int i=0, j=0;
	float tmp_ang=0.0;
	
	
	for(i=0;i<(2<<24);i++){

		for(j=0;j<STR_NMB;j++){
			tmp_ang = tmp_ang + fabsf(act_spv.str_ang[j]);
		}
		
		if(tmp_ang < WAIT_STR_ANG_LMT){
			break;
		}
		
		tmp_ang=0.0;
	}
}


void cmd_exe(char c, float trg)
{
	int i=0, num=0;
	static int test_mod = NG, emst_mod = NG, state_i=OFF;
	
	if(fabsf(trg)>360.0){
		trg = 0.0;
	}
	
	//printf("Command char = %s \n", &c);
	//printf("Command float = %f \n", trg);
	
	if(c=='s'){//steering
			
		if(fabsf(trg)>=90.0){
			trg = trg/fabsf(trg) * 90.0;
		}
		
		if(state_i==OFF){
			for(i=0;i<STR_NMB;i++){
				rcv_eth.str_trg[i]= trg*PI/180.0;
			}
		}

	}else if(c=='u'){//2WD steering
			
		if(fabsf(trg)>=90.0){
			trg = trg/fabsf(trg) * 90.0;
		}
		
		if(state_i==OFF){
			for(i=0;i<STR_NMB;i++){
				if(i<2){
					rcv_eth.str_trg[i]= trg*PI/180.0;
				}else{
					rcv_eth.str_trg[i]= 0.0;
				}
			}		
		}
			
	}else if(c=='d'){//drive
				
		if(fabsf(rcv_eth.str_trg[0]-rcv_eth.str_trg[1]) < 0.001){//constraint
		//if(state_i==OFF){	
			if(fabsf(trg)>=DRV_MAX_DST){ //meter
				trg = trg/fabsf(trg) * (float)DRV_MAX_DST;
			}
		
			for(i=0;i<DRV_NMB;i++){
				rcv_eth.drv_trg[i] = trg;
			}
						
		}
			
	}else if(c=='a'){ //pan motor drive
		
		if(trg>=RU_PAN_PST_ANG_LMT_DEG){
			trg = (float)RU_PAN_PST_ANG_LMT_DEG;
		}else if(trg<RU_PAN_NGT_ANG_LMT_DEG){
			trg = (float)RU_PAN_NGT_ANG_LMT_DEG;
		}		
	
		rcv_eth.ptl_trg[0]= trg*PI/180.0;

	}else if(c=='o'){ //pan motor drive
		
		//if(trg >= (trg/fabsf(trg)*90.0 - (float)RU_TLT_FIX_ANG_DEG)){
		//	trg = trg/fabsf(trg) * 90.0 - (float)RU_TLT_FIX_ANG_DEG;
		//}
		
		if(trg <= RU_TLT_NGT_ANG_LMT_DEG){
			trg = (float)RU_TLT_NGT_ANG_LMT_DEG;
		}else if(trg > RU_TLT_PST_ANG_LMT_DEG){
			trg = (float)RU_TLT_PST_ANG_LMT_DEG;
		}
		
		rcv_eth.ptl_trg[1]= trg*PI/180.0;

	}else if(c=='n'){ //displaying ad data for potentio offset cal
			 
		printf("\n--PTL current angle--\n");
		for(i=0;i<PTL_NMB;i++){
			printf("angles[%d] = %3.2f deg\n", i, get_rslv(i)*180.0/PI);
		}
		
		printf("\n\n--STR potentio voltage--\n");
		for(i=0;i<STR_NMB;i++){
			printf("#%d current potentio voltage/angle = %3.3f [V] / %3.3f [deg] \n",
			i+1, ad_data[i].vlt, act_spv.str_ang[i]*180.0/(float)PI);
			//fprintf(fp_ad, "\n---- latest line ---- \n");
			//fprintf(fp_ad, "%2.3f \n", ad_data[i].vlt);
		}
		printf("\n");
		
						
	}else if(c=='p'){ //maximum velocity set
			
		if(trg>DRV_MAX_DTY){
			trg = DRV_MAX_DTY;
		}else if(trg<(-1.0*DRV_MAX_DTY)){
			trg = -1.0*DRV_MAX_DTY;
		}else if( (trg<DRV_NSN_DTY_RNG) && (trg > (-1.0*DRV_NSN_DTY_RNG))){
			trg = trg/fabsf(trg)*DRV_NSN_DTY_RNG;
		}
		
		rcv_eth.max_pwm = trg*FLT_TO_INT_PWM_CFC; 
		
		
	}else if(c=='e'){ //emergency stop
		
		//drive stop
		for(i=0;i<DRV_NMB;i++){
			rcv_eth.drv_trg[i]=0.0;
		}
		
		if(emst_mod==NG){
			emst_mod=OK;
		}else if(emst_mod==OK){
			emst_mod=NG;
		}
					
	}else if(c=='i'){ //turn in place (initialization)
		
		//initialize driving 			
		for(i=0;i<DRV_NMB;i++){
			rcv_eth.drv_trg[i]=0.0;
		}			
		
		//initialize steering 
		rcv_eth.str_trg[0]=(float)PWL_ANG_1;
		rcv_eth.str_trg[1]=(float)PWL_ANG_2;
		rcv_eth.str_trg[2]=(float)PWL_ANG_3;
		rcv_eth.str_trg[3]=(float)PWL_ANG_4;
		
		//printf("steering target for pinwheeling = %f rad \n", rcv_eth.str_trg[0]);
		
		state_i = ON;
		
	}else if(c=='r'){ //turn in place (rotation)
		
		float tmp_ang;
		
		tmp_ang = WHL_BAS_DST;

		if(state_i==ON){
			//front wheel
			rcv_eth.drv_trg[0] = trg*(float)PI/180*tmp_ang;
			rcv_eth.drv_trg[1] = -trg*(float)PI/180*tmp_ang;
		
			//rear wheel
			rcv_eth.drv_trg[2] = trg*(float)PI/180*tmp_ang;
			rcv_eth.drv_trg[3] = -trg*(float)PI/180*tmp_ang;
		}else{
			printf("Impossible! Check current steering angle \n");
		}
		
	}else if(c=='f'){ // finishing
	
		//drive stop
		for(i=0;i<DRV_NMB;i++){
			rcv_eth.drv_trg[i]=0.0;
		}
		
		//steering stop
		for(i=0;i<STR_NMB;i++){
			rcv_eth.str_trg[i]=0.0;
		}
		
		//pan tilt reset
		for(i=0;i<PTL_NMB;i++){
			
			if(i==1){ //tilt
				rcv_eth.ptl_trg[i]=(float)RU_TLT_FIX_ANG_DEG/180.0*(float)PI;
			}else{ //pan
				rcv_eth.ptl_trg[i]=0.0;
			}
			
		}
		
		wait_str_ang_zero();
		
		state_i = OFF;

	}else if(c=='t'){ //test mode set
		
		if(test_mod==OK){
			test_mod=NG;
		}else if(test_mod==NG){
			test_mod=OK;
		}

	}else if(c=='g'){ 
		
		//printf("\n No comment! \n");

	}else if(c=='q'){ //question means help
				
/*		printf("\n\n ---- command help by q---- \n\n");			
		printf("\n-Operation relations-\n");
		printf("u***: Steer 2 wheels with *** deg\n");
		printf("s***: Steer 4 wheels with *** deg\n");
		printf("d***: Drive 4 wheels with *** meters\n");
		printf("p***: Set PWM value with *** percent\n");
		printf("e: Stop here and Go in/out emergency stop mode\n");
		printf("e + h***,j***,k***: \n #5 wheel rotation preperation 'h', change angle with 'j***' deg, and stay here 'k'\n");
		printf("f: Finish to STR and DRV and pan tilt\n");
		printf("\n-HK relations-\n");
		printf("n: Display HK data\n");
		printf("a: ***: Pan motor moves with *** deg\n");
		printf("o: ***: Tilt motor moves with *** deg\n");
		printf("i: Initialize turn in palce\n");
		printf("\n-Turn in palce relations-\n");
		printf("r***: Turn in palce with *** deg\n");
		printf("g: Show secret command\n");
		printf("\n-TEST relations-\n");
		printf("t: Go in/out test mode\n");
		printf("t + h***,j***,k***,l***: Move each steering motor: #1,2,3,4 with *** deg\n");
		printf("z***,x***,c***,v***: Move each driving motor: #1,2,3,4 with *** meter\n");
		//printf("t + z***: Preparation for pan sweep\n");
		//printf("t + x***: Perform pan sweep\n");
		//printf("t + c***: Perform tilt sweep\n");
*/
		
	}else{
	
		//steering origin setup			
		for(i=0;i<STR_NMB;i++){
			rcv_eth.str_trg[i]=0.0*PI/180;
		}
					
		//driving stop
		for(i=0;i<DRV_NMB;i++){
			rcv_eth.drv_trg[i] = 0.0*PI/180;
		}
		
		
	}//else
	
	
	//test mode
	if((test_mod == OK) && (emst_mod != OK)){
	
		if(c=='h' || c=='j' || c=='k' || c=='l'){//each steering
		
			if(fabsf(trg)>=90.0){
				trg = trg/fabsf(trg) * 90.0;
			}
		
			if(c=='h'){ 
				num = 0;
			}else if(c=='j'){
				num = 1;
			}else if(c=='k'){
				num = 2;
			}else if(c=='l'){
				num = 3;
			}else if(c=='+'){
				num = 4;			
			}
			if(num<STR_NMB){
				rcv_eth.str_trg[num]=trg*PI/180.0;
			}
						
			num=STR_NMB;
	
		}else if(c=='z' || c=='x' || c=='c' || c=='v'){//each driving
		
			if(fabsf(trg)>=100.0){
				trg = trg/fabsf(trg) * 100.0;
			}
		
			if(c=='z'){
				num = 0;
			}else if(c=='x'){
				num = 1;
			}else if(c=='c'){
				num = 2;
			}else if(c=='v'){
				num = 3;
			}else if(c=='>'){				
				num = 4;
			}
			if(num<DRV_NMB){
				rcv_eth.drv_trg[num]=trg;
			}
			
			num=DRV_NMB;
						
		}//if(c=='h' || c=='j' || c=='k' || c=='l'){//each steering
		
	}//if(test_mod == OK)
		
}


void *cmd_hdl(void *arg){ //main in command/telemetry handling
	
	int i=0;
	unsigned char rdata[EBUF_CMD_SIZ];
	char cmd_chr[1];
	float cmd_val=0.0;
	
	memset(rdata, 0, EBUF_CMD_SIZ);
	
	for(;;)
	{
	
		if(i==0){
			if((cmd_fd=tcp_listen(&GND))!=0)
			{	
				i++;
			}
		}


		if(i==1){
			if(rcv_cmd(cmd_fd, &cmd_chr[0], &cmd_val)==OK) //command recognition
			{
				printf("receved data character : %c and value=%f \n", cmd_chr[0], cmd_val);
				
				//printf("you are in command handling \n");
				cmd_exe(cmd_chr[0], cmd_val); //command execution
			}
		}
			
		cmd_chr[0]=0;
		cmd_val = 0.0;
		
	}//for(;;)

	close(cmd_fd);	
	
}


//motor driver relations
char mdv_cmd_tlm_dat_siz(int num){
	
	int scn_num = 0;
	
	scn_num = (int)(num/1000);
	num = num%1000;
	
	if(num==MDV_CMD_ENC_CLR){

		return (char)MDV_CMD_ENC_CLR_DAT_SIZ;

	}else if(num==MDV_CMD_DTY_SET){

		return (char)MDV_CMD_DTY_SET_DAT_SIZ;

	}else if(num==MDV_CMD_TLM_SND){

		return (char)MDV_CMD_TLM_SND_DAT_SIZ;

	}else if(num==MDV_TLM_ENC_CLR){

		return (char)MDV_TLM_ENC_CLR_DAT_SIZ;

	}else if(num==MDV_TLM_DTY_SET){

		return (char)MDV_TLM_DTY_SET_DAT_SIZ;

	}else if(num==MDV_TLM_TLM_SND){

		if(scn_num==0){
			return MDV_TLM_TLM_SND_DAT_SIZ_00;
		}else if(scn_num==1){
			return MDV_TLM_TLM_SND_DAT_SIZ_01;
		}else if(scn_num==2){
			return MDV_TLM_TLM_SND_DAT_SIZ_02;
		}else if(scn_num==3){
			return MDV_TLM_TLM_SND_DAT_SIZ_03;
		}else if(scn_num==4){
			return MDV_TLM_TLM_SND_DAT_SIZ_04;
		}else if(scn_num==5){
			return MDV_TLM_TLM_SND_DAT_SIZ_05;
		}else{
			return 0;
		}		
	}else{
		return 0;
	}
	
}


int make_mdv_cmd(int mdv_num, int cmd_num, char *buf){
	
	int n=0, num=0;
	unsigned short UH_tmp=0;
	int scnd_cmd_num = 0;
	
	memset(buf, 0, SBUF_CMD_SIZE);
	
	//analyze second command
	scnd_cmd_num = (int)(cmd_num/1000);
	cmd_num = (int)(cmd_num%1000);
	
	//Header
	buf[0] = 0x02;  
	n++;
		
	//driver ID	
	num = mdv_num+1;
	//buf[n] = (char)num;		
	buf[n] = (char)mdv_ch[num-1];
	n++;
	
	//data size
    buf[n] = mdv_cmd_tlm_dat_siz(cmd_num);	
    n++;
    
    //command ID
    buf[n] = (char)cmd_num;	
	n++;
	
	if(cmd_num==(int)MDV_CMD_DTY_SET){
		if(mdv_num<DRV_NMB){
			
/*			if(snd_srl.drv_dty[mdv_num]>500){
				snd_srl.drv_dty[mdv_num] = 500;
			}else if(snd_srl.drv_dty[mdv_num]<-500){
				snd_srl.drv_dty[mdv_num] = -500;
			}
*/	
			itoc4(snd_srl.drv_dty[mdv_num], &buf[n]);
	
		}else if(mdv_num<(STR_NMB+DRV_NMB)){
	
//		if(fabsf(act_spv.str_ang[mdv_num-DRV_NMB])<=(float)PI/2.0){
			itoc4(snd_srl.str_dty[mdv_num-DRV_NMB], &buf[n]);
//		}else{
//			itoc4((int)0, &buf[n]);
//		}
			
		}else if(mdv_num<TTL_MTR_NMB){
	
			itoc4(snd_srl.ptl_dty[mdv_num - DRV_NMB - STR_NMB], &buf[n]);
	
		}
		
		
		n=n+4;
			
	}else if(cmd_num==(int)MDV_CMD_TLM_SND){
		
		buf[n] = (char)scnd_cmd_num;
		n++;
		//printf("\n in make motor driver command for telemetry: #%d:%d \n", (char)cmd_num, (char)scnd_cmd_num);
		
	}else if(cmd_num==(int)MDV_CMD_ENC_CLR){

		buf[n] = (char)scnd_cmd_num;
		n++;
	}	
	
    UH_tmp=serial_checksum(&buf[1],n-1);    
    buf[n]=ustoc1(UH_tmp);   //check sum
    n++;
    buf[n]=0;
    n++;
    
    return n;
}

void emrg_stop(){ //emergency stop command
	
	int j=0, res=0, num_cmd_dat=0;
	char cmd_buf[SBUF_CMD_SIZE];
	
	memset(cmd_buf, 0, SBUF_CMD_SIZE);
	
	//set duty 0
/*	for(j=0;j<TTL_MTR_NMB;j++){	
		if(j<DRV_NMB){
			snd_srl.drv_dty[j] = 0.0;
		}else if(j<DRV_NMB+STR_NMB){
			snd_srl.str_dty[j-DRV_NMB] = 0.0;
		}else{
			snd_srl.ptl_dty[j-DRV_NMB-STR_NMB] = 0.0;
		}
		num_cmd_dat = make_mdv_cmd(j, (int)MDV_CMD_DTY_SET ,cmd_buf);
		res = write(serial_fd[8], cmd_buf, num_cmd_dat);
	}
*/

	//set target 0
	for(j=0;j<TTL_MTR_NMB;j++){	
		if(j<DRV_NMB){
			rcv_eth.drv_trg[j] = 0.0;
		}else if(j<DRV_NMB+STR_NMB){
			rcv_eth.str_trg[j] = 0.0;
		}else{
			rcv_eth.ptl_trg[j] = 0.0;
		}
	}
	
	sleep(1);
			
}

int chk_cmd_ack(unsigned char *cmd_buf, unsigned char *tlm_buf, int res){
	
	const int PVn = 8;
	int tmp_cmd_val=123, tmp_tlm_val=321;
	int flg=0;
	
	if(res >= PVn){

		if(cmd_buf[3] == (char)MDV_CMD_ENC_CLR){
			printf("encoder clear \n");
			return 1;
		}

		tmp_cmd_val = c4toi(&cmd_buf[4]);
		tmp_tlm_val = c4toi(&tlm_buf[4]);
		
		if(tmp_cmd_val == tmp_tlm_val){
			flg = 1;
		}else{
			flg = 0;
		}
		
	
	}else{
		flg = 0;
	}
	
	return flg;
}


void eth_dat_print(int num, char *buf){

	int j=0;
	
	for(j=0; j<num; j++)
		printf("%x ", buf[j]);
	
	printf(": %d\n", num);		

	
}


void snd_cmd_mdv(motor_data *cmd){ //send command to motor drivers
	
	int i=0,j=0,n=0,res, num_cmd_dat;
	char cmd_buf[SBUF_CMD_SIZE], tlm_buf[SBUF_TLM_SIZE];
	unsigned short UH_tmp=0;
	static int cnt=0;
	int srl_flg = 0, slc_srl_nmb=8;
	
	memset(cmd_buf, 0, SBUF_CMD_SIZE);
	memset(tlm_buf, 0, SBUF_TLM_SIZE);
	
	
	for(i=0;i<TTL_MTR_NMB;i++){
	//for(i=TTL_MTR_NMB-1;i>=0;i--){			
		
		num_cmd_dat = make_mdv_cmd(i, cmd->lst_cmd_num ,cmd_buf);
		
		//select serial port
		//left or right
		srl_flg = mdv_srl_ch[i];
		
		if(srl_flg==0){
			slc_srl_nmb=8;
		}else if(srl_flg==1){
			slc_srl_nmb=9;
		}
		
		//printf("\n open serial port #%d \n",slc_srl_nmb);


		// retry writing for n times on error
		int retry = 2;
		while (retry-- > 0) {
			res = write(serial_fd[slc_srl_nmb], cmd_buf, num_cmd_dat);		
			usleep(2000);

			if (res < 0) {
				printf("Write error for #%d: %s\n\n", slc_srl_nmb, strerror(errno));
				//close_serial(slc_srl_nmb);
				//init_serial(slc_srl_nmb);
				usleep(100);
			} else {
				break;
			}
		}

#ifdef DEBUG2
        //printf write data
        printf("Write #[%d]: ", i);
		eth_dat_print(res, cmd_buf);
#endif //DEBUG2				

		//emergency com
		if(res<0){
			//printf("write error");
			
			//reboot serial port for USB
			close_serial(slc_srl_nmb);
			
			usleep(100);
			
			init_serial(slc_srl_nmb);
			
			//emrg_stop();
			
			printf("\n\n --Emergency stop at SERIAL COM ERROR-- \n\n");	
			
			break;
		}	

/*		
		usleep(1000);

		//absorb ACK
		res = read(serial_fd[8], tlm_buf, SBUF_TLM_SIZE);
        tlm_buf[res]=0;

        
#ifdef DEBUG2
        //printf read data
        printf("Read #[%d]: ", i);
		eth_dat_print(res, tlm_buf);
#endif //DEBUG2

		if(chk_cmd_ack((unsigned char *)cmd_buf, (unsigned char *)tlm_buf, res)==1){
			i++;
		}else{
			if(res<0){
				close_serial(8);
			
				usleep(1000);
			
				init_serial(8);
				
				emrg_stop();
			
				printf("\n\n --Emergency stop at SERIAL TLM ERROR-- \n\n");	
			
				break;
			}
		}
*/		
		usleep(1000);

	} //for i
	
	
	cmd->prv_cmd_num = cmd->lst_cmd_num;
	cmd->lst_cmd_num = MDV_CMD_NON;
	
	
	/////////////////////////
	//encoder telemetry get//
	/////////////////////////
/*
	if(cnt%5==0){
		
		cmd->lst_cmd_num = MDV_CMD_TLM_SND + MDV_SCN_CMD_ABS_ENC_CNT;					
		//printf("Command ID = %d \n", (cmd->lst_cmd_num));
		
		for(i=0;i<TTL_MTR_NMB;i++){			
		
			num_cmd_dat = make_mdv_cmd(i, cmd->lst_cmd_num , cmd_buf);

			//select serial port
			//left or right
			srl_flg = mdv_srl_ch[i];
		
			if(srl_flg==0){
				slc_srl_nmb=8;
			}else if(srl_flg==1){
				slc_srl_nmb=9;
			}
				
			int retry = 4;
			while (retry-- > 0) {
				res = write(serial_fd[slc_srl_nmb], cmd_buf, num_cmd_dat);		
				usleep(3000);

				if (res < 0) {
					printf("Write error for #%d: %s\n\n", slc_srl_nmb, strerror(errno));
					sleep(1);
				} else {
					break;
				}
			}
			
			res = read(serial_fd[slc_srl_nmb], tlm_buf, SBUF_TLM_SIZE);
			
			if(res<0){
				printf("read error in telemetry \n");
			}else{
				tlm_buf[res]=0;
			}
			
			//data store
			if(res>3){
				rcg_mdv_tlm(i, cmd, tlm_buf, res);
			}

			usleep(10000);

		}
		cmd->prv_cmd_num = snd_srl.lst_cmd_num;
		cmd->lst_cmd_num = MDV_CMD_NON;
	}
*/
	
	cnt++;
	
	/////////////////////////
	//encoder telemetry end//
	/////////////////////////
	
	
	//if(cnt%1000==0){
	//		close_serial(8);
	//		
	//		usleep(1000);
	//		
	//		init_serial(8);
	//		
	//		cnt=1;
	//}
	
}


void rcg_mdv_tlm(int ch, motor_data *tlm, char *buf, int sbuf_siz){

	int scn_cmd_num=0;
	int dat_siz=0;
	
	scn_cmd_num = (int)((tlm->lst_cmd_num)/1000) * 1000;
	dat_siz = (int)mdv_cmd_tlm_dat_siz(tlm->lst_cmd_num);
	
	//if(sbuf_siz>(dat_siz+3)){
		
		//printf("\n scn_cmd_num = %d at rcg_mdv_tlm \n", scn_cmd_num);

		if(scn_cmd_num == MDV_SCN_CMD_ABS_ENC_CNT){
			
			/*
			if(c4toui(&buf[5])<2147483648){
				tlm->abs_enc_cnt[ch] = (int)c4toui(&buf[5]);
			}else{
				tlm->abs_enc_cnt[ch] = (int)c4toui(&buf[5])-4294967296;
			}*/

			tlm->abs_enc_cnt[ch] = (int)c4toi(&buf[5]);
			
			printf("\n Encoder absolute value [%d] = %d \n", ch,(int)(tlm->abs_enc_cnt));
				
		}else if(scn_cmd_num == MDV_SCN_CMD_RLT_ENC_CNT){
			
			/*
			if(c4toui(&buf[5])<2147483648){
				tlm->rlt_enc_cnt[ch] = (int)c4toui(&buf[5]);
			}else{
				tlm->rlt_enc_cnt[ch] = (int)c4toui(&buf[5])-4294967296;
			}*/

			tlm->rlt_enc_cnt[ch] = (int)c4toi(&buf[5]);			
			
			printf("\n Encoder relative value [%d] = %d \n", ch,(int)(tlm->rlt_enc_cnt));
			
		}else if(scn_cmd_num == MDV_SCN_CMD_CRR){
	
			if(c2tous(&buf[5])<32768){
				tlm->mtr_crr_int[ch] = (short)c2tous(&buf[5]);
			}else{
				tlm->mtr_crr_int[ch] = (short)c2tous(&buf[5])-65536;
			}
			
			tlm->mtr_crr_flt[ch] = (float)(130.0/3228.0)*((tlm->mtr_crr_int[ch])-3972) + 100.0;
		
		}else if(scn_cmd_num == MDV_SCN_CMD_TMP){
	
			tlm->mdv_tmp_int[ch] = c2tous(&buf[5]);
			
			tlm->mdv_tmp_flt[ch] = (float)(24.0/4095.0)*((tlm->mdv_tmp_int[ch])-2047) + 12.0;
			
		}else if(scn_cmd_num == MDV_SCN_CMD_DTY){
			
			if(c2tous(&buf[5])<32768){
				tlm->ctr_dty_int[ch] = (short)c2tous(&buf[5]);
			}else{
				tlm->ctr_dty_int[ch] = (short)c2tous(&buf[5])-65536;
			}
			
			tlm->ctr_dty_flt[ch] = (float)(tlm->ctr_dty_int[ch])/10.0;
		
		}else{
			
			//printf("No telemetry was received \n");
			
		}
	//}
	
	
}


void rcv_tlm_mdv(motor_data *tlm){
	
	int i=0, res, num_cmd_dat, num_tlm_dat;
	char cmd_buf[SBUF_CMD_SIZE], tlm_buf[SBUF_TLM_SIZE];
	int srl_flg = 0, slc_srl_nmb=8;
	
	memset(cmd_buf, 0, SBUF_CMD_SIZE);
	memset(tlm_buf, 0, SBUF_TLM_SIZE);
		
	for(i=0;i<TTL_MTR_NMB;i++){			
		
		num_cmd_dat = make_mdv_cmd(i, tlm->lst_cmd_num ,cmd_buf);

		//select serial port
		//left or right
		srl_flg = mdv_srl_ch[i];
		
		if(srl_flg==0){
			slc_srl_nmb=8;
		}else if(srl_flg==1){
			slc_srl_nmb=9;
		}
		
		res = write(serial_fd[slc_srl_nmb], cmd_buf, num_cmd_dat);
		usleep(1000);
		
		if(res<0){
			//printf("write error in telemetry \n");
		}
		
		res = read(serial_fd[slc_srl_nmb], tlm_buf, SBUF_TLM_SIZE);
		
		if(res<0){
			//printf("read error in telemetry \n");
		}else{
			tlm_buf[res]=0;
		}
		
		//data store
		if(res>3){
			rcg_mdv_tlm(i, tlm, tlm_buf, res);
		}
		
	}
	
	tlm->prv_cmd_num = tlm->lst_cmd_num;
	tlm->lst_cmd_num = MDV_CMD_NON;
	
}


void init_mdv(){
	
	int i=0;
	
	//encoder clear		
	//snd_srl.lst_cmd_num = (int)MDV_CMD_ENC_CLR;
	//snd_cmd_mdv(&snd_srl);
	//printf("Encoder Clear \n");
		
	//set all target 0
	snd_srl.lst_cmd_num = (int)MDV_CMD_DTY_SET;
	snd_cmd_mdv(&snd_srl);
	printf("Set all target 0 or fixed value \n");

}


void *mtr_hdl(void *arg){
	
	int i=0;
	struct timespec bfr_clck, aft_clck;
	struct timespec req, res;
	int time_usec_bfr=0, time_usec_aft=0;
	
	clock_gettime(CLOCK_MONOTONIC, &bfr_clck);
	clock_gettime(CLOCK_MONOTONIC, &aft_clck);
	req.tv_sec = 0;
	req.tv_nsec = 0;
	
	for(;;){
				
		///////////////////////////////////////////
		//start clock//
		if(clock_gettime(CLOCK_MONOTONIC, &bfr_clck) !=0){
			printf("error: clock_gettime() \n");
		}		
		time_usec_bfr = (int)(bfr_clck.tv_nsec)/KILOZERO;
		//end start clock//		
		///////////////////////////////////////////

			
		////////////
		//userland//
		////////////
		
		//send command of motor control
		snd_srl.lst_cmd_num=MDV_CMD_DTY_SET;
		snd_cmd_mdv(&snd_srl);

		////////////
		//user end//
		////////////

		
		///////////////////////////////////////////
		//end clock//
		clock_gettime(CLOCK_MONOTONIC, &aft_clck);
		time_usec_aft = (int)(aft_clck.tv_nsec)/KILOZERO;
				
		if(time_usec_aft < time_usec_bfr){
			req.tv_nsec = GIGAZERO/SERIAL_OUT_RATE - KILOZERO*( 1 * MEGAZERO + time_usec_aft - time_usec_bfr);
			//printf("aft = %d, bfr = %d \n", time_usec_aft, time_usec_bfr);
		}else{
			req.tv_nsec = GIGAZERO/SERIAL_OUT_RATE - KILOZERO*( time_usec_aft - time_usec_bfr );
		}		
		
		if(req.tv_nsec > 0){
			nanosleep(&req, &res);
		}
		//end end clock//				
		///////////////////////////////////////////
		
#ifdef DEBUG
		if(time_usec_aft < time_usec_bfr){
			printf("[serial cmd hdl] (irr) total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr )/KILOZERO, ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr ));
		}else{
			printf("[serial cmd hdl] total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( time_usec_aft - time_usec_bfr )/KILOZERO, ( time_usec_aft - time_usec_bfr ));
		}
#endif //DEBUG		
		
					
		
	}
	
	
}


void *mtr_hk_hdl(void *arg){
	
	int i=0;
	struct timespec bfr_clck, aft_clck;
	struct timespec req, res;
	int time_usec_bfr=0, time_usec_aft=0;
	
	clock_gettime(CLOCK_MONOTONIC, &bfr_clck);
	clock_gettime(CLOCK_MONOTONIC, &aft_clck);
	req.tv_sec = 0;
	req.tv_nsec = 0;
	
	for(;;){
				
		///////////////////////////////////////////
		//start clock//
		if(clock_gettime(CLOCK_MONOTONIC, &bfr_clck) !=0){
			printf("error: clock_gettime() \n");
		}		
		time_usec_bfr = (int)(bfr_clck.tv_nsec)/KILOZERO;
		//end start clock//		
		///////////////////////////////////////////

			
		////////////
		//userland//
		////////////

		
		//absolute encoder data
		snd_srl.lst_cmd_num=MDV_CMD_TLM_SND + MDV_SCN_CMD_ABS_ENC_CNT;					
		rcv_tlm_mdv(&snd_srl);
		
/*		//current
		snd_srl.lst_cmd_num=MDV_CMD_TLM_SND + MDV_SCN_CMD_CRR;					
		rcv_tlm_mdv(&snd_srl);		
		//temperature
		snd_srl.lst_cmd_num=MDV_CMD_TLM_SND + MDV_SCN_CMD_TMP;					
		rcv_tlm_mdv(&snd_srl);
		//duty
		snd_srl.lst_cmd_num=MDV_CMD_TLM_SND + MDV_SCN_CMD_DTY;					
		rcv_tlm_mdv(&snd_srl);
*/		////////////
		//user end//
		////////////

		
		///////////////////////////////////////////
		//end clock//
		clock_gettime(CLOCK_MONOTONIC, &aft_clck);
		time_usec_aft = (int)(aft_clck.tv_nsec)/KILOZERO;
				
		if(time_usec_aft < time_usec_bfr){
			req.tv_nsec = GIGAZERO/MTR_HK_RATE - KILOZERO*( 1 * MEGAZERO + time_usec_aft - time_usec_bfr);
			//printf("aft = %d, bfr = %d \n", time_usec_aft, time_usec_bfr);
		}else{
			req.tv_nsec = GIGAZERO/MTR_HK_RATE - KILOZERO*( time_usec_aft - time_usec_bfr );
		}		
		
		if(req.tv_nsec > 0){
			nanosleep(&req, &res);
		}
		//end end clock//				
		///////////////////////////////////////////
		
#ifdef DEBUG
		if(time_usec_aft < time_usec_bfr){
			printf("[serial tlm hdl] (irr) total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr )/KILOZERO, ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr ));
		}else{
			printf("[serial tlm hdl] total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( time_usec_aft - time_usec_bfr )/KILOZERO, ( time_usec_aft - time_usec_bfr ));
		}
#endif //DEBUG		
		
					
		
	}
	
	
}


void *get_add_prd(void *arg){ //periodically AD data get
	
	int i=0;
	struct timespec bfr_clck, aft_clck;
	struct timespec req, res;
	int time_usec_bfr=0, time_usec_aft=0;
	
	clock_gettime(CLOCK_MONOTONIC, &bfr_clck);
	clock_gettime(CLOCK_MONOTONIC, &aft_clck);
	req.tv_sec = 0;
	req.tv_nsec = 0;
	
	for(;;){
				
		///////////////////////////////////////////
		//start clock//
		if(clock_gettime(CLOCK_MONOTONIC, &bfr_clck) !=0){
			printf("error: clock_gettime() \n");
		}		
		time_usec_bfr = (int)(bfr_clck.tv_nsec)/KILOZERO;
		//end start clock//		
		///////////////////////////////////////////

			
		////////////
		//userland//
		////////////
		get_add();			
		////////////
		//user end//
		////////////

		
		///////////////////////////////////////////
		//end clock//
		clock_gettime(CLOCK_MONOTONIC, &aft_clck);
		time_usec_aft = (int)(aft_clck.tv_nsec)/KILOZERO;
				
		if(time_usec_aft < time_usec_bfr){
			req.tv_nsec = GIGAZERO/ADD_ACQ_RATE - KILOZERO*( 1 * MEGAZERO + time_usec_aft - time_usec_bfr);
			//printf("aft = %d, bfr = %d \n", time_usec_aft, time_usec_bfr);
		}else{
			req.tv_nsec = GIGAZERO/ADD_ACQ_RATE - KILOZERO*( time_usec_aft - time_usec_bfr );
		}		
		
		if(req.tv_nsec > 0){
			nanosleep(&req, &res);
		}
		//end end clock//				
		///////////////////////////////////////////
		
#ifdef DEBUG
		if(time_usec_aft < time_usec_bfr){
			printf("[add hdl] (irr) total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr )/KILOZERO, ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr ));
		}else{
			printf("[add hdl] total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( time_usec_aft - time_usec_bfr )/KILOZERO, ( time_usec_aft - time_usec_bfr ));
		}
#endif	//DEBUG	
		
		
	}
	
}


void *ptl_ctr(void *arg){ //pan tilt control

	static int tmp_cnt=0;
	int ptn = 0; //pan tilt number
	float crt_trg=0.0, crt_ang=0.0;
	float tmp_pwm=0.0, max_pwm=500.0, crt_vel=0.0;
	static float err_i[PTL_NMB]={0.0, 0.0};
	static float prv_ang[STR_NMB]={0.0, 0.0, 0.0, 0.0};
	static long itg_cnt=0;
	
	struct timespec bfr_clck, aft_clck;
	struct timespec req, res_t;
	int time_usec_bfr=0, time_usec_aft=0;
	
	tmp_cnt++; 
	ptn = tmp_cnt - 1; //0: pan, 1: tilt
	printf("#%d pan/tlt thread start \n", ptn);
	
	clock_gettime(CLOCK_MONOTONIC, &bfr_clck);
	clock_gettime(CLOCK_MONOTONIC, &aft_clck);
	req.tv_sec = 0;
	req.tv_nsec = 0;
	
	for(;;){
				
		///////////////////////////////////////////
		//start clock//
		if(clock_gettime(CLOCK_MONOTONIC, &bfr_clck) !=0){
			printf("error: clock_gettime() \n");
		}		
		time_usec_bfr = (int)(bfr_clck.tv_nsec)/KILOZERO;
		//end start clock//		
		///////////////////////////////////////////
			
		////////////
		//userland//
		////////////
	
		
		//maximum pwm ratio
		max_pwm = act_spv.max_pwm;
		
		//get resolver angle
		crt_ang = act_spv.ptl_ang[ptn]; 
			
		//get current target
		crt_trg = act_spv.ptl_trg[ptn];
		
		if(ptn==1){
			if((crt_trg>((float)PTL_TLT_ANG_PST_LMT_RAD))){
				crt_trg = (float)PTL_TLT_ANG_PST_LMT_RAD;
			}else if((crt_trg<((float)PTL_TLT_ANG_NGT_LMT_RAD))){
				crt_trg = (float)PTL_TLT_ANG_NGT_LMT_RAD;	
			}
		}else if(ptn==0){
			if((crt_trg>((float)PTL_PAN_ANG_PST_LMT_RAD))){
				crt_trg = (float)PTL_PAN_ANG_PST_LMT_RAD;
			}else if((crt_trg<((float)PTL_PAN_ANG_NGT_LMT_RAD))){
				crt_trg = (float)PTL_PAN_ANG_NGT_LMT_RAD;	
			}
		}

		//calcualte integration of error
		/*if(itg_cnt%(2<<16)==0){
			err_i[ptn]=0.0;
			itg_cnt=1;
		}else{
			err_i[ptn] = err_i[ptn] + (crt_trg - crt_ang) * 1.0/(float)PTL_CTR_RATE;
			itg_cnt++;
		}*/
		
		//calculation derivative
		crt_vel = (crt_ang - prv_ang[ptn]) * 1.0/(float)PTL_CTR_RATE; 
			
		//set ptl control duty
		if(ptn==0){			
			tmp_pwm = (float)PTL_PAN_MTR_ROT_DIR*
			(float)((PTL_PAN_PID_GAIN_P*(crt_trg - crt_ang)
					+PTL_PAN_PID_GAIN_I*err_i[ptn]
					+PTL_PAN_PID_GAIN_D*crt_vel)
					*FLT_TO_INT_PWM_CFC);
		}else if(ptn==1){
			tmp_pwm = (float)PTL_TLT_MTR_ROT_DIR*
			(float)((PTL_TLT_PID_GAIN_P*(crt_trg - crt_ang)
					+PTL_TLT_PID_GAIN_I*err_i[ptn]
					+PTL_TLT_PID_GAIN_D*crt_vel)
					*FLT_TO_INT_PWM_CFC);
		}
		
		if(fabsf(tmp_pwm)>max_pwm){
			tmp_pwm = tmp_pwm/fabsf(tmp_pwm)*max_pwm;
		}
		
		if(fabsf(crt_trg - crt_ang)<PTL_PWM_NSN_LMT_RAD){
			tmp_pwm = 0.0;
		}
		
		act_spv.ptl_dty[ptn] = (int)tmp_pwm;
		
		//printf("max_pwm/ctr_pwm = %f/%f, crt_trg=%f, crt_ang=%f, act_spv.ptl_dty[%d]=%d \n", 
		//max_pwm, tmp_pwm, crt_trg, crt_ang, ptn, act_spv.ptl_dty[ptn]);
		
		//for next loop
		prv_ang[ptn] = crt_ang;
		
		////////////
		//user end//
		////////////
		
		///////////////////////////////////////////
		//end clock//
		clock_gettime(CLOCK_MONOTONIC, &aft_clck);
		time_usec_aft = (int)(aft_clck.tv_nsec)/KILOZERO;
				
		if(time_usec_aft < time_usec_bfr){
			req.tv_nsec = GIGAZERO/PTL_CTR_RATE - KILOZERO*( 1 * MEGAZERO + time_usec_aft - time_usec_bfr);
		}else{
			req.tv_nsec = GIGAZERO/PTL_CTR_RATE - KILOZERO*( time_usec_aft - time_usec_bfr );
		}		
		
		if(req.tv_nsec > 0){
			nanosleep(&req, &res_t);
		}
		//end end clock//				
		///////////////////////////////////////////
		
#ifdef DEBUG
		if(time_usec_aft < time_usec_bfr){
			printf("[ptl hdl:%d] (irr) total = %f msec . worktime=%d usec \n",ptn,(float)req.tv_nsec/MEGAZERO + ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr )/KILOZERO, ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr ));
		}else{
			printf("[ptl_hdl:%d] total = %f msec . worktime=%d usec \n",ptn,(float)req.tv_nsec/MEGAZERO + ( time_usec_aft - time_usec_bfr )/KILOZERO, ( time_usec_aft - time_usec_bfr ));
		}
#endif	//DEBUG	
		
		
	}//for(;;)
	
}


void *drv_ctr(void *arg){ //driving control

	struct timespec bfr_clck, aft_clck;
	struct timespec req, res_t;
	int time_usec_bfr=0, time_usec_aft=0;
	
	static int tmp_cnt=0, drv_rot_dir[DRV_NMB];
	int dvn = 0; //steering number
	float crt_trg=0.0, crt_ang=0.0;
	float tmp_pwm=0.0, max_pwm=500.0;
	
	float crt_vel=0.0;
	static float err_i[DRV_NMB]={0.0, 0.0, 0.0, 0.0};
	static float prv_ang[DRV_NMB]={0.0, 0.0, 0.0, 0.0};
	static long itg_cnt=0;

	tmp_cnt++; 
	dvn = tmp_cnt - 1; //0~3 steering number
	printf("#%d driving thread start \n", dvn);	
	
	if(dvn==0){
		drv_rot_dir[0] = (int)DRV_ROT_DIR_1;
	}else if(dvn==1){
		drv_rot_dir[1] = (int)DRV_ROT_DIR_2;
	}else if(dvn==2){
		drv_rot_dir[2] = (int)DRV_ROT_DIR_3;
	}else if(dvn==3){
		drv_rot_dir[3] = (int)DRV_ROT_DIR_4;
	}
	
	clock_gettime(CLOCK_MONOTONIC, &bfr_clck);
	clock_gettime(CLOCK_MONOTONIC, &aft_clck);
	req.tv_sec = 0;
	req.tv_nsec = 0;
	
	
	for(;;){
				
		///////////////////////////////////////////
		//start clock//
		if(clock_gettime(CLOCK_MONOTONIC, &bfr_clck) !=0){
			printf("error: clock_gettime() \n");
		}		
		time_usec_bfr = (int)(bfr_clck.tv_nsec)/KILOZERO;
		//end start clock//		
		///////////////////////////////////////////
			
		////////////
		//userland//
		////////////
		//maximum pwm ratio
		max_pwm = act_spv.max_pwm;
		
		//get angle
		crt_ang = act_spv.drv_ang[dvn]; 
			
		//get current target
		crt_trg = act_spv.drv_trg[dvn];
		
		//calcualte integration of error
		/*if(itg_cnt%(2<<16)==0){
			err_i[dvn]=0.0;
			itg_cnt=1;
		}else{
			//err_i[dvn] = err_i[dvn] + (crt_trg - crt_ang) * 1.0/(float)DRV_CTR_RATE;
			err_i[dvn] = 0.0;
			itg_cnt++;
		}*/
		err_i[dvn] = 0.0;
		
		//calculation derivative
		crt_vel = (crt_ang - prv_ang[dvn]) * 1.0/(float)DRV_CTR_RATE; 
		
		//set control duty
		tmp_pwm = drv_rot_dir[dvn]*
				(float)(DRV_PID_GAIN_P*(crt_trg - crt_ang)
				+DRV_PID_GAIN_I*err_i[dvn]
				+DRV_PID_GAIN_D*crt_vel)
				*FLT_TO_INT_PWM_CFC;
		
		//printf("Driving pwm = %f, crt_vel/crt_trg = %f/%f (cang=%f) rad/s, err_i=%f \n", tmp_pwm, crt_vel, crt_trg, crt_ang, err_i[dvn]);
		//tmp_pwm = drv_rot_dir[dvn]*max_pwm;
		
		if(fabsf(tmp_pwm)>max_pwm){
			tmp_pwm = tmp_pwm/fabsf(tmp_pwm)*max_pwm;
		}
				
		act_spv.drv_dty[dvn] = (int)tmp_pwm;
		
		//for next loop
		prv_ang[dvn] = crt_ang;
		
		////////////
		//user end//
		////////////
		
		///////////////////////////////////////////
		//end clock//
		clock_gettime(CLOCK_MONOTONIC, &aft_clck);
		time_usec_aft = (int)(aft_clck.tv_nsec)/KILOZERO;
				
		if(time_usec_aft < time_usec_bfr){
			req.tv_nsec = GIGAZERO/DRV_CTR_RATE - KILOZERO*( 1 * MEGAZERO + time_usec_aft - time_usec_bfr);
		}else{
			req.tv_nsec = GIGAZERO/DRV_CTR_RATE - KILOZERO*( time_usec_aft - time_usec_bfr );
		}		
		
		if(req.tv_nsec > 0){
			nanosleep(&req, &res_t);
		}
		//end end clock//				
		///////////////////////////////////////////
		
#ifdef DEBUG
		if(time_usec_aft < time_usec_bfr){
			printf("[drv hdl:%d] (irr) total = %f msec . worktime=%d usec \n",ptn,(float)req.tv_nsec/MEGAZERO + ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr )/KILOZERO, ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr ));
		}else{
			printf("[drv_hdl:%d] total = %f msec . worktime=%d usec \n",ptn,(float)req.tv_nsec/MEGAZERO + ( time_usec_aft - time_usec_bfr )/KILOZERO, ( time_usec_aft - time_usec_bfr ));
		}
#endif	//DEBUG	
		
			
	}//for(;;)	
}


void *str_ctr(void *arg){ //steering control

	struct timespec bfr_clck, aft_clck;
	struct timespec req, res_t;
	int time_usec_bfr=0, time_usec_aft=0;

	static int tmp_cnt=0;
	int i=0;
	int stn = 0; //steering number
	float crt_trg=0.0, crt_ang=0.0;
	static float max_pwm=500.0;
	float tmp_pwm=0.0, crt_vel=0.0;
	static float err_i[STR_NMB]={0.0, 0.0, 0.0, 0.0};
	static float prv_ang[STR_NMB]={0.0, 0.0, 0.0, 0.0};
	static long itg_cnt=0;
	
	tmp_cnt++; 
	stn = tmp_cnt - 1; //0~3 steering number
	printf("#%d steering thread start \n", stn);	
	
	clock_gettime(CLOCK_MONOTONIC, &bfr_clck);
	clock_gettime(CLOCK_MONOTONIC, &aft_clck);
	req.tv_sec = 0;
	req.tv_nsec = 0;
	
	for(;;){
				
		///////////////////////////////////////////
		//start clock//
		if(clock_gettime(CLOCK_MONOTONIC, &bfr_clck) !=0){
			printf("error: clock_gettime() \n");
		}		
		time_usec_bfr = (int)(bfr_clck.tv_nsec)/KILOZERO;
		//end start clock//		
		///////////////////////////////////////////
			
		////////////
		//userland//
		////////////
		
		//maximum pwm ratio
		max_pwm = act_spv.max_pwm;
		
		//get angle
		crt_ang = act_spv.str_ang[stn]; //rad
		
		//get current target
		crt_trg = act_spv.str_trg[stn]; //rad

		//calcualte integration of error
		//if(itg_cnt%(2<<16)==0){
		//	err_i[stn]=0.0;
		//	itg_cnt=1;
		//}else{
		//	err_i[stn] = err_i[stn] + (crt_trg - crt_ang) * 1.0/(float)STR_CTR_RATE;
		//	itg_cnt++;
		//}
		err_i[stn] = 0.0;
		
		//calculation derivative
		crt_vel = (crt_ang - prv_ang[stn]) * 1.0/(float)STR_CTR_RATE; 
		
		//set control duty
		tmp_pwm = STR_MTR_ROT_DIR*
			(float)(STR_PID_GAIN_P*(crt_trg - crt_ang)
					+STR_PID_GAIN_I*err_i[stn]
					+STR_PID_GAIN_D*crt_vel)
					*FLT_TO_INT_PWM_CFC;
		
		if(fabsf(tmp_pwm)>max_pwm){
			tmp_pwm = tmp_pwm/fabsf(tmp_pwm)*max_pwm;
		}

		if(fabsf(crt_trg - crt_ang)<STR_PWM_NSN_LMT_RAD){
			tmp_pwm = 0.0;
		}
		
		
		//tmp_pwm = fir_scd_filt(tmp_pwm, 20+stn);
		
		/*if(stn==1){
			printf("pwm[1] = %f \n", tmp_pwm);
		}*/
		
		act_spv.str_dty[stn] = (int)tmp_pwm; //float to int change
		
		//for next loop
		prv_ang[stn] = crt_ang;

		////////////
		//user end//
		////////////
		
		///////////////////////////////////////////
		//end clock//
		clock_gettime(CLOCK_MONOTONIC, &aft_clck);
		time_usec_aft = (int)(aft_clck.tv_nsec)/KILOZERO;
				
		if(time_usec_aft < time_usec_bfr){
			req.tv_nsec = GIGAZERO/STR_CTR_RATE - KILOZERO*( 1 * MEGAZERO + time_usec_aft - time_usec_bfr);
		}else{
			req.tv_nsec = GIGAZERO/STR_CTR_RATE - KILOZERO*( time_usec_aft - time_usec_bfr );
		}		
		
		if(req.tv_nsec > 0){
			nanosleep(&req, &res_t);
		}
		//end end clock//				
		///////////////////////////////////////////
		
#ifdef DEBUG
		if(time_usec_aft < time_usec_bfr){
			printf("[str hdl:%d] (irr) total = %f msec . worktime=%d usec \n",ptn,(float)req.tv_nsec/MEGAZERO + ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr )/KILOZERO, ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr ));
		}else{
			printf("[str_hdl:%d] total = %f msec . worktime=%d usec \n",ptn,(float)req.tv_nsec/MEGAZERO + ( time_usec_aft - time_usec_bfr )/KILOZERO, ( time_usec_aft - time_usec_bfr ));
		}
#endif	//DEBUG	
		
		
	}//for(;;)
	
}


void *ctr_spv(void *arg){ //supervisor for control, syncronization, data handling from eth and serial 

	int i=0, ctr_n = 0, j=0;
	float temp=0.0;
	static int thrd_n = 0;
	static float crt_add[STR_NMB], prv_add[STR_NMB], prv2_add[STR_NMB];
	float tmp_ofs=0.0, tmp_add=0.0;

	struct timespec bfr_clck, aft_clck;
	struct timespec req, res_t;
	int time_usec_bfr=0, time_usec_aft=0;
	float tmp_str_ang[STR_NMB];

	static float drv_tim=0;
	static int drv_upd_flg=NEW, tim_cnt=0, tim_cnt_inv=0;
	static float old_trg=0.0;
	
	thrd_n++;
	ctr_n = thrd_n - 1;
	printf("#%d ctr_spv thread start \n", ctr_n);	
	for(i=0;i<STR_NMB;i++){
		crt_add[i] = 0.0;
		prv_add[i] = 0.0;
		prv2_add[i] = 0.0;
	}
	
	clock_gettime(CLOCK_MONOTONIC, &bfr_clck);
	clock_gettime(CLOCK_MONOTONIC, &aft_clck);
	req.tv_sec = 0;
	req.tv_nsec = 0;
	
	for(;;){
				
		///////////////////////////////////////////
		//start clock//
		if(clock_gettime(CLOCK_MONOTONIC, &bfr_clck) !=0){
			printf("error: clock_gettime() \n");
		}		
		time_usec_bfr = (int)(bfr_clck.tv_nsec)/KILOZERO;
		//end start clock//		
		///////////////////////////////////////////
			
		////////////
		//userland//
		////////////
		act_spv.max_pwm = rcv_eth.max_pwm;

		if(ctr_n==0){
			//pan tilt control supervise
			//printf("Pan/tlt control supervise \n");
			for(i=0;i<PTL_NMB;i++){
				act_spv.ptl_ang[i] = get_rslv(i);
				act_spv.ptl_trg[i] = rcv_eth.ptl_trg[i];
				snd_srl.ptl_dty[i] = act_spv.ptl_dty[i];
				//printf("act_spv.ptl_trg[%d] = %f (at ctr_spv)\n", i, act_spv.ptl_trg[i]);
				//printf("Current PAN/TLT angle[%d] = %2.2f deg \n", i, 
					//act_spv.ptl_ang[i]*180.0/PI);
					
				if(i==1){
					if((act_spv.ptl_ang[i]>((float)PTL_TLT_ANG_PST_LMT_RAD+0.1)) || 
					(act_spv.ptl_ang[i]<((float)PTL_TLT_ANG_NGT_LMT_RAD-0.1))){
						//emrg_stop();
						//printf("\n\n --Emergency stop at TLT overrange-- \n\n");	
					}
				}else if(i==0){
					if((act_spv.ptl_ang[i]>((float)PTL_PAN_ANG_PST_LMT_RAD+0.1)) || 
					(act_spv.ptl_ang[i]<((float)PTL_PAN_ANG_NGT_LMT_RAD-0.1))){
						//emrg_stop();	
						//printf("\n\n --Emergency stop at PAN overraneg-- \n\n");	
					}
				}

			} //for
			

		}
		
		
		if(ctr_n==1){
			//steering control supervise
			//printf("Steering control supervise \n");
			for(i=0;i<STR_NMB;i++){
								
				if(i==0){
					tmp_ofs = (float)STR_PTN_OFS_VLT_1;
				}else if(i==1){
					tmp_ofs = (float)STR_PTN_OFS_VLT_2;
				}else if(i==2){
					tmp_ofs = (float)STR_PTN_OFS_VLT_3;
				}else if(i==3){
					tmp_ofs = (float)STR_PTN_OFS_VLT_4;
				}else{
					tmp_ofs = 0.0;
				}
				
				crt_add[i] = (ad_data[i].vlt - tmp_ofs);
				
				//tmp_add = (crt_add[i]+2*prv_add[i]+prv2_add[i])/4.0;
				//tmp_add = fir_scd_filt(crt_add[i], i+DRV_NMB);
				tmp_add = crt_add[i];
				
				tmp_str_ang[i] = STR_PTN_ROT_DIR*(float)STR_PTN_CRC_VLT_TO_ANG_RAD*tmp_add;  //rad
				//if(tmp_str_ang[i]<0.0){
				//	tmp_str_ang[i] 
				//}
				//if(i==1)
					//printf("Current angle[%d] = %2.2f deg \n", i, tmp_str_ang[i]*180.0/PI);
				act_spv.str_ang[i] = tmp_str_ang[i]; //rad
				
				if(fabsf(act_spv.str_ang[i])>((float)STR_ANG_LMT_RAD+0.1)){
					//emrg_stop();
					printf("\n\n --Emergency stop at STR overraneg-- \n\n");	
				}
				
				/*if(fabsf(rcv_eth.str_trg[i])>((float)STR_ANG_LMT_RAD)){
					rcv_eth.str_trg[i] = rcv_eth.str_trg[i]/fabsf(rcv_eth.str_trg[i]) * 
					((float)STR_ANG_LMT_RAD);
				}*/

				act_spv.str_trg[i] = rcv_eth.str_trg[i];
				snd_srl.str_dty[i] = act_spv.str_dty[i];

				
				prv2_add[i] = prv_add[i];
				prv_add[i] = crt_add[i];

			}
		}


		if(ctr_n==2){	
			//driving control supervise
			//printf("Driving control supervise \n");
			
			if(drv_upd_flg==NEW){
			
				if(act_spv.max_pwm!=0.0){ 
					drv_tim = fabsf(rcv_eth.drv_trg[0])
					/((float)DRV_LNR_SPD*act_spv.max_pwm*(float)INT_TO_FLT_PWM_CFC/100.0); 
				}else{
					drv_tim = 0.0;
				}
								
				for(i=0;i<DRV_NMB;i++){
					act_spv.drv_ang[i] = 0.0; //no encoder information
					//act_spv.drv_ang[i] = snd_srl.abs_enc_cnt[i]/(float)DRV_ENC_CNT_PER_RND*2.0*(float)PI;
					act_spv.drv_trg[i] = rcv_eth.drv_trg[i]/fabsf(rcv_eth.drv_trg[i])
							*(float)DRV_MAX_DST/DRV_WHL_RDS*DRV_GEA_RAT;
				}
				
				old_trg = rcv_eth.drv_trg[0];
				tim_cnt = (int)(drv_tim*CTR_SPV_RATE*DRV_TIM_ADJ_CFC);
				tim_cnt_inv = 0;
				drv_upd_flg = OLD;
			
				
			}
			
			if(drv_upd_flg==OLD){
							
				tim_cnt--;
				tim_cnt_inv++;
				
				//printf("drivin time coun updated = %d ", tim_cnt);	

				if(tim_cnt>0){
					
					for(i=0;i<DRV_NMB;i++){
				//		//gradually accelerating
						if(tim_cnt_inv<=(int)(DRV_ACC_DRT*CTR_SPV_RATE)){
							snd_srl.drv_dty[i] = (int)((float)((float)(tim_cnt_inv)/(float)((float)DRV_ACC_DRT*(float)CTR_SPV_RATE))*(float)act_spv.drv_dty[i]);
							//printf("[%d]Input drive target = %d / %d [deg]\n",tim_cnt_inv, snd_srl.drv_dty[i],act_spv.drv_dty[i]);
						}else{
							snd_srl.drv_dty[i] = act_spv.drv_dty[i];
							//usleep(1000);
							//printf("MAX::[%d]Input drive target = %d [deg]\n",tim_cnt, act_spv.drv_dty[i]);
						}
					}
					
				}else{

					for(i=0;i<DRV_NMB;i++){
						act_spv.drv_trg[i] = 0.0;
						snd_srl.drv_dty[i] = 0.0;
					}

				} //if(tim_cnt>0)


				if(old_trg != rcv_eth.drv_trg[0]){
					
					drv_upd_flg = NEW;
				
				}

			} //if OLD
			
		}//if(ctr_n==2)
		
		////////////
		//user end//
		////////////

		
		///////////////////////////////////////////
		//end clock//
		clock_gettime(CLOCK_MONOTONIC, &aft_clck);
		time_usec_aft = (int)(aft_clck.tv_nsec)/KILOZERO;
				
		if(time_usec_aft < time_usec_bfr){
			req.tv_nsec = GIGAZERO/CTR_SPV_RATE - KILOZERO*( 1 * MEGAZERO + time_usec_aft - time_usec_bfr);
		}else{
			req.tv_nsec = GIGAZERO/CTR_SPV_RATE - KILOZERO*( time_usec_aft - time_usec_bfr );
		}		
		
		if(req.tv_nsec > 0){
			nanosleep(&req, &res_t);
		}
		//end end clock//				
		///////////////////////////////////////////
		
#ifdef DEBUG
		if(ctr_n==0){
			if(time_usec_aft < time_usec_bfr){
				printf("[ptl_spv hdl] (irr) total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr )/KILOZERO, ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr ));
			}else{
				printf("[ptl_spv hdl] total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( time_usec_aft - time_usec_bfr )/KILOZERO, ( time_usec_aft - time_usec_bfr ));
			}
		}
		
		if(ctr_n==1){
			if(time_usec_aft < time_usec_bfr){
				printf("[str_spv hdl] (irr) total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr )/KILOZERO, ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr ));
			}else{
				printf("[str_spv hdl] total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( time_usec_aft - time_usec_bfr )/KILOZERO, ( time_usec_aft - time_usec_bfr ));
			}
		}
		
		if(ctr_n==2){
			if(time_usec_aft < time_usec_bfr){
				printf("[drv_spv hdl] (irr) total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr )/KILOZERO, ( 1 * MEGAZERO + time_usec_aft - time_usec_bfr ));
			}else{
				printf("[drv_spv hdl] total = %f msec . worktime=%d usec \n",(float)req.tv_nsec/MEGAZERO + ( time_usec_aft - time_usec_bfr )/KILOZERO, ( time_usec_aft - time_usec_bfr ));
			}
		}
#endif	//DEBUG	
		
	}//for(;;)

}


void init_thread_function(){
	
	int i=0,iret[THR_NMB], j=0;
	
	//motor output
	iret[i] = pthread_create(&thrd_num[i], NULL, (void *)mtr_hdl, (void*) &i);
	i++;
	
	//motor hk acquisition
	//iret[i] = pthread_create(&thrd_num[i], NULL, (void *)mtr_hk_hdl, (void*) &i);
	//i++;	
		
	//resolver data get
	iret[i] = pthread_create(&thrd_num[i], NULL, (void *)get_rslv_dat_pan, (void*) &i);
	i++;
	
	iret[i] = pthread_create(&thrd_num[i], NULL, (void *)get_rslv_dat_tlt, (void*) &i);
	i++;
	
	//add data get
	iret[i] = pthread_create(&thrd_num[i], NULL, (void *)get_add_prd, (void*) &i);
	i++;
		
	//pan/tilt, drive, steering motor control
	for(j=0;j<DRV_NMB;j++){
		iret[i] = pthread_create(&thrd_num[i], NULL, (void *)drv_ctr, (void*) &i);
		i++;
		usleep(1000);
	}

	for(j=0;j<PTL_NMB;j++){
		iret[i] = pthread_create(&thrd_num[i], NULL, (void *)ptl_ctr, (void*) &i);
		i++;
		usleep(1000);
	}
	
	for(j=0;j<STR_NMB;j++){
		iret[i] = pthread_create(&thrd_num[i], NULL, (void *)str_ctr, (void*) &i);
		i++;
		usleep(1000);
	}


	//control supervisor
	for(j=0;j<CTR_SPV_NMB;j++){
		iret[i] = pthread_create(&thrd_num[i], NULL, (void *)ctr_spv, (void*) &i);
		i++;
		usleep(1000);
	}

	//command handling and communication 
	iret[i] = pthread_create(&thrd_num[i], NULL, (void *)cmd_hdl, (void*) &i);
	i++;
	
	//data acqisition

	//making save file
	iret[i] = pthread_create(&thrd_num[i], NULL, (void *)make_file, (void*) &i);
	i++;
	

	//mutex lock for pan/tilt data
	for(i=0;i<2;i++){
		pthread_mutex_init(&pan_tlt_mutex[i], NULL);
	}
	
	//pan tilt control
	
	//for(i=0;i<2;i++){
	//	pthread_join(thrd_num[i], NULL);
	//}	

	
}

	
void init_all(){
	
	int i=0, j=0;
	motor_data *tmp;
	
	//set 0 to variables
	memset(ad_data, 0, sizeof(int)*AD_CHN_NMB);
	
	for(j=0;j<3;j++){
		if(j==0) tmp = &rcv_eth;
		if(j==1) tmp = &act_spv;
		if(j==2) tmp = &snd_srl;
		
		for(i=0;i<DRV_NMB;i++){
			tmp->drv_trg[i]=0.0;
			tmp->drv_pos[i]=0.0;
			tmp->drv_pos[i]=0.0;
			tmp->drv_ang[i]=0.0;
			tmp->drv_crt[i]=0.0;
			tmp->drv_dty[i]=0.0;
		}
	
		for(i=0;i<STR_NMB;i++){
			tmp->str_trg[i]=0.0;
			tmp->str_pos[i]=0.0;	
			tmp->str_ang[i]=0.0;
			tmp->str_crt[i]=0.0;
			tmp->str_dty[i]=0.0; //steering driver duty	
		}
	
	
		for(i=0;i<PTL_NMB;i++){
			if(i==1){
				tmp->ptl_trg[i]=(float)RU_TLT_FIX_ANG_DEG/180.0*(float)PI;
			}else{
				tmp->ptl_trg[i]=0.0;
			}
			tmp->ptl_pos[i]=0.0;
			tmp->ptl_ang[i]=0.0;
			tmp->ptl_crt[i]=0.0;
			tmp->ptl_dty[i]=0.0;
		}
		
		tmp->dst_trg = 0.0;
		tmp->thrm_trg=0.0;
		tmp->elec_busv=0;
		tmp->max_pwm=500;
		tmp->cmd_c=0;
		tmp->lst_cmd_num=0;
		
		for(i=0;i<PTL_NMB;i++){
			tmp->mdv_tmp_int[i]=0;	
			tmp->ctr_dty_int[i]=0;
			tmp->mtr_crr_int[i]=0;
			tmp->abs_enc_cnt[i]=0;
			tmp->rlt_enc_cnt[i]=0;
			tmp->mtr_crr_flt[i]=0;
			tmp->ctr_dty_flt[i]=0;
			tmp->mdv_tmp_flt[i]=0;
		}
	
	}
	
	//file save
	if((fp_ad=fopen("ad_data.txt", "w"))==NULL){
		printf("ad_data.txt file open error\n");
	}
	
	if(((fp_rslv_pan)=fopen("rslv_pan_data.txt", "w"))==NULL){
		printf("rslv_pan_data.txt file open error\n");
		
	}

	if(((fp_rslv_tlt)=fopen("rslv_tlt_data.txt", "w"))==NULL){
		printf("rslv_tlt_data.txt file open error\n");
		
	}
	
	if(((fp_hk)=fopen("mc_hk_data.csv", "w"))==NULL){
		printf("mc_hk_data.csv file open error\n");
		
	}
	
	//AD board initializing
	init_add_prt();
	
	//serial port initializaing
	for(i=0;i<2;i++){
		init_serial(i);
	}
	init_serial(8); //USB0>>MC1
	init_serial(9); //USB1>>MC2
	
	//motor driver initializing
	init_mdv();
	
	//ethernet port initializing
	//>> initialized in charged thread function
	
	//last call
	init_thread_function();
	
	//pan tilt set current position
	act_spv.ptl_trg[0]=get_rslv(0); //pan
	act_spv.ptl_trg[1]=get_rslv(1); //tilt
		
}
