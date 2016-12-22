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
#include <pthread.h>

#include "tcp_func.h"

#define RU_COUNT_OF_ONE_ROUND 262144 //max value of counter
#define RU_ANGLE_OF_ONE_ROUND 360.0
#define RU_SIZE_OF_COMMAND 6 //data size[byte]
#define RU_ERROR_CODE (-1) 

#define SERIAL_PORT_0 "/dev/ttyS0" //RS232C, for Resolver of pan
#define SERIAL_PORT_1 "/dev/ttyS1" //RS232C, for Resolver of tilt
#define SERIAL_PORT_2 "/dev/ttyS2" //N.A.
#define SERIAL_PORT_3 "/dev/ttyS3" //N.A.
#define SERIAL_PORT_4 "/dev/ttyS4" //Force torque sensor #1
#define SERIAL_PORT_5 "/dev/ttyS5" //Force torque sensor #2
#define SERIAL_PORT_6 "/dev/ttyS6" //Force torque sensor #3
#define SERIAL_PORT_7 "/dev/ttyS7" //Force torque sensor #4
#define SERIAL_PORT_USB0 "/dev/ttyUSB0" //motor driver

#define SERIAL_BAUDRATE B38400
#define _POSIX_SOURCE 1
#define SBUF_CMD_SIZE 255
#define SBUF_TLM_SIZE 4096
#define TRUE 1
#define FALSE 0
#define DRV_SPEED 170

#define THR_NMB 10

volatile int STOP=FALSE;
int serial_fd[9], cmd_fd;                            //
struct termios oldtio[9], newtio;    //
pthread_t thrd_num[THR_NMB];
pthread_mutex_t pan_tlt_mutex[2];
static int rslv_ring_length[2];
char rslv_ring_buf[2][SBUF_TLM_SIZE];

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
				
		if(srl_num==0){
			fd = open(SERIAL_PORT_0, O_RDWR | O_NOCTTY | O_NONBLOCK);
		}else if(srl_num==1){
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
		}else if(srl_num==8){
			fd = open(SERIAL_PORT_USB0, O_RDWR | O_NOCTTY | O_NONBLOCK);
		}else{
			fd = open(SERIAL_PORT_0, O_RDWR | O_NOCTTY | O_NONBLOCK);
		}		
		//fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);	 
	    
	    //setup global variables
	    serial_fd[srl_num] = fd;
	    
	    if(fd<0)
	    {
			printf("fail to open serial for driver \n");
		}else{
			printf("open serial port #%d \n", srl_num);
		}
			
	    //res=tcgetattr(fd, &oldtio[srl_num]);       // 現在のシリアルポートの設定を待避させる
	    res=tcgetattr(fd, &old_dummy);
	    if(res) printf("tcgetattr error \n");
	
	    memset(&newtio, 0, sizeof(newtio)); 
	    newtio = old_dummy;                  // ポートの設定をコピー
	    newtio.c_cflag = CS8 | CLOCAL | CREAD; 
	    //newtio.c_iflag = ICRNL | IGNPAR;
	    //newtio.c_oflag = 0;
	    //newtio.c_lflag = 0;
	
	    //newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */ 
	    //newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
	    //newtio.c_cc[VERASE]   = 0;     /* del */
	    //newtio.c_cc[VKILL]    = 0;     /* @ */
	    //newtio.c_cc[VEOF]     = 0;     /* Ctrl-d */
	    newtio.c_cc[VTIME]    = 100;     /* キャラクタ間タイマを使わない */
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
	
	    res = cfsetispeed(&newtio, SERIAL_BAUDRATE);
	    if(res < 0) printf("cfsetspeed error \n");
	    
	    res = cfsetospeed(&newtio, SERIAL_BAUDRATE);
	    if(res < 0) printf("cfsetspeed error \n");
	
	    res = tcflush(fd, TCIFLUSH);
	    if(res < 0) printf("tcflush error \n");
		
	    res = tcsetattr(fd, TCSANOW, &newtio);       // ポートの設定を有効にする
	    if(res) printf("tcsetattr error \n");
	    
	    oldtio[srl_num] = old_dummy;
	
}


void close_serial(int srl_num){
	    ioctl(serial_fd[srl_num], TCSETS, &oldtio[srl_num]);       // ポートの設定を元に戻す
	    close(serial_fd[srl_num]);                        // デバイスのクローズ
}	


void close_thread_function(){

	int i=0;
	
	for(i=0;i<2;i++){
		pthread_mutex_destroy(&pan_tlt_mutex[i]);
	}
}

int get_rslv_val(char* rdata){
	
	char buf[RU_SIZE_OF_COMMAND+1];
	static int pcount=0;
	
	if(pcount==0)
		memset(buf, 0, RU_SIZE_OF_COMMAND);
	
	if((rdata[0]=='\r') || (rdata[0]=='\n' || (rdata[0]==0x0a))){
		pcount=0;
		return RU_ERROR_CODE;
	}
	
	buf[pcount] = rdata[0];
	pcount ++;
	
	if(pcount == RU_SIZE_OF_COMMAND){
		
		buf[RU_SIZE_OF_COMMAND] = '\n';
		pcount = 0;
		
		return atol(&buf[0]);
		
	}else{
		
		return RU_ERROR_CODE;
	}
	
}


void *get_rslv_dat_pan(void *arg){ //0:pan , 1:tlt

	char buf[SBUF_TLM_SIZE];
	int res=0, i=0;
	static int wp=0;
	static int pan_tlt_num=0;
	
	//pan_tlt_num = (int *) arg;
	
	//pan_tlt_num++;
	
	//printf("pan_tlt_num=%d",pan_tlt_num);
	
	memset(&rslv_ring_length[pan_tlt_num], 0 ,sizeof(int));
	memset(&rslv_ring_buf[pan_tlt_num][0],0,SBUF_TLM_SIZE);

	while(1){
		res = read(serial_fd[pan_tlt_num], buf, SBUF_TLM_SIZE);
		
		if(res==0){
			//printf("read 0 (#%d) \n", pan_tlt_num);
			continue;
		}else if(res<0){
			//printf("read error in the function of get resolver data \n");
			continue;
		}
		
		pthread_mutex_lock( &pan_tlt_mutex[pan_tlt_num]);
		
		for(i=0;i<res;i++){
			rslv_ring_buf[pan_tlt_num][wp] = buf[i];
			wp = (wp+1)%SBUF_TLM_SIZE;
			rslv_ring_length[pan_tlt_num]++;
		}
		
		pthread_mutex_unlock( &pan_tlt_mutex[pan_tlt_num]);
		 
	}
}


void *get_rslv_dat_tlt(void *arg){ //0:pan , 1:tlt

	char buf[SBUF_TLM_SIZE];
	int res=0, i=0;
	static int wp=0;
	static int pan_tlt_num=1;
	
	//pan_tlt_num = (int *) arg;
	
	//pan_tlt_num++;
	
	//printf("pan_tlt_num=%d",pan_tlt_num);
	
	memset(&rslv_ring_length[pan_tlt_num], 0 ,sizeof(int));
	memset(&rslv_ring_buf[pan_tlt_num][0],0,SBUF_TLM_SIZE);

	while(1){
		res = read(serial_fd[pan_tlt_num], buf, SBUF_TLM_SIZE);
		
		if(res==0){
			//printf("1 in the rslve_ring_length loop (#%d) \n", pan_tlt_num);
			continue;
		}else if(res<0){
			//printf("read error in the function of get resolver data \n");
			continue;
		}
		
		pthread_mutex_lock( &pan_tlt_mutex[pan_tlt_num]);
		
		//printf("res = %d, 0x", res);
		for(i=0;i<res;i++){
			rslv_ring_buf[pan_tlt_num][wp] = buf[i];
			wp = (wp+1)%SBUF_TLM_SIZE;
			rslv_ring_length[pan_tlt_num]++;
			//printf("%x ", buf[i]);
		}
		//printf("\n\n");
		
		pthread_mutex_unlock( &pan_tlt_mutex[pan_tlt_num]);
		 
	}
}

float cnv_cnt_to_ang(int ncount){
	
	return((float)(ncount)/((float)RU_COUNT_OF_ONE_ROUND) - 1.0)*(float)RU_ANGLE_OF_ONE_ROUND;

}


float get_rslv(int pan_tlt_num){
	
	int rslv_cnt=RU_ERROR_CODE;
	float rslv_ang=628.0;
	char buf;
	static int rslv_read_ptr=0;
	int i=0;
	
	while(1){

		//printf("wait for rslve_ring_length > 0 (#%d) \n", pan_tlt_num);
		if(rslv_ring_length[pan_tlt_num]>0){
			//printf("1 in the rslve_ring_length loop (#%d) \n", pan_tlt_num);
			pthread_mutex_lock( &pan_tlt_mutex[pan_tlt_num]);
			//printf("2 in the rslve_ring_length loop (#%d) \n", pan_tlt_num);
			rslv_ring_length[pan_tlt_num]--;
			buf = (char)rslv_ring_buf[pan_tlt_num][rslv_read_ptr];
			printf("buf = %x (%d) \n", buf, pan_tlt_num);			
			pthread_mutex_unlock( &pan_tlt_mutex[pan_tlt_num]);
			
			rslv_read_ptr = (rslv_read_ptr+1)%SBUF_TLM_SIZE;
			printf("rslv_read_ptr = %d (%d) \n", rslv_read_ptr, pan_tlt_num);
			
			rslv_cnt = get_rslv_val(&buf);
			
		}	
		
		if(rslv_cnt != RU_ERROR_CODE){
			//printf("rslv_cnt = %d (%d) \n", rslv_cnt, pan_tlt_num);
			break;
		}
	}
	
	rslv_ang = (float)cnv_cnt_to_ang((int)rslv_cnt);
	printf("rslv_ang = %f (%d) \n", rslv_ang, pan_tlt_num);
	
	return rslv_ang;

}


void init_thread_function(){
	
	char *message[2]={"Pan","Tlt"};
	int i=0,iret[THR_NMB];
	
	printf("before thread create \n");

	//for(i=0;i<2;i++){
		iret[i] = pthread_create(&thrd_num[i], NULL, (void *)get_rslv_dat_pan, (void*) &i);
		iret[i] = pthread_create(&thrd_num[i], NULL, (void *)get_rslv_dat_tlt, (void*) &i);
	//}
	
	//printf("before thread join \n");
	 
	//for(i=0;i<2;i++){
	//	pthread_join(thrd_num[i], NULL);
	//}	

	printf("before thread mutex \n");
	
	for(i=0;i<2;i++){
		pthread_mutex_init(&pan_tlt_mutex[i], NULL);
	}
}


void serial_hdl(){
	
	char buf[SBUF_TLM_SIZE];
	int j, res;
	
	for(;;){
		res = read(serial_fd[0], buf, SBUF_TLM_SIZE);
		
		if(res<0){
			printf("read error \n");
		}else{		
			printf("read \n");
	        buf[res]=0;
	
			for(j=0; j<res; j++)
				printf("%x ", buf[j]);

				printf(": %d\n", res);
		}
		
	
	}
	
	
}


//command handling

void cmd_hdl(){ //main in command/telemetry handling
	
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


		//command handling
		if(i==1){
			if(rcv_cmd(cmd_fd, &cmd_chr[0], &cmd_val)==OK) //command recognition
			{
				printf("receved data character : %c and value=%f \n", cmd_chr[0], cmd_val);
				
				//printf("you are in command handling \n");
				//cmd_exe(); //command execution
				//cmd_cnt++;
			}
		}
	
		//tptr[(int) arg].thread_count++;
		
		cmd_chr[0]=0;
		cmd_val = 0.0;
		
	}//for(;;)

	close(cmd_fd);	
	
	//printf("Command handlign thread --FINISHED \n");
}
