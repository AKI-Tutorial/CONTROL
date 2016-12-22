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


#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE B115200
#define _POSIX_SOURCE 1

#define FALSE 0
#define TRUE 1
#define DATA_MAX_SIZE 255

#define DRV_SPEED 170

unsigned short checksum(char *tmp, int leng);
char ustoc1(unsigned short usd);
void itoc4(int id, char *cd);
unsigned short c2tous(char *cd);

volatile int STOP=FALSE;

int main(int argc, char *argv[]){

    char buf[DATA_MAX_SIZE];                    /* バッファ */
    int fd, i,j, res;                            /* ファイルディスクリプタ */
    unsigned short UH_tmp=0;
    struct termios oldtio, newtio;    /* シリアル通信設定 */

    printf("data set \n");
    
    //fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
    fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);	 
    if(fd<0) printf("fail to open serial for driver \n");

    printf("serial open \n");
    
    res=tcgetattr(fd, &oldtio);       /* 現在のシリアルポートの設定を待避させる */
    if(res) printf("tcgetattr error \n");

    memset(&newtio, 0, sizeof(newtio)); 
    newtio = oldtio;                  /* ポートの設定をコピー */
    newtio.c_cflag = CS8 | CLOCAL | CREAD; 
    newtio.c_iflag = ICRNL | IGNPAR;
    //newtio.c_iflag = IGNPAR | ICANON;
    newtio.c_oflag = 0;
    //newtio.c_lflag = ICANON;
    newtio.c_lflag = 0;

    //newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */ 
    //newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
    //newtio.c_cc[VERASE]   = 0;     /* del */
    //newtio.c_cc[VKILL]    = 0;     /* @ */
    newtio.c_cc[VEOF]     = 0;     /* Ctrl-d */
    newtio.c_cc[VTIME]    = 0;     /* キャラクタ間タイマを使わない */
    newtio.c_cc[VMIN]     = 1;     /* 1文字来るまで，読み込みをブロックする */
    //newtio.c_cc[VSWTC]    = 0;     /* '\0' */
    //newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */ 
    //newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
    //newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
    newtio.c_cc[VEOL]     = 0;     /* '\0' */
    //newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
    //newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
    //newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
    //newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
    //newtio.c_cc[VEOL2]    = 0;     /* '\0' */

    res = cfsetispeed(&newtio, BAUDRATE);
    if(res < 0) printf("cfsetspeed error \n");
    res = cfsetospeed(&newtio, BAUDRATE);
    if(res < 0) printf("cfsetspeed error \n");

    res = tcflush(fd, TCIFLUSH);
    if(res < 0) printf("tcflush error \n");

    res = tcsetattr(fd, TCSANOW, &newtio);       /* ポートの設定を有効にする */
    if(res) printf("tcsetattr error \n");

    printf("serial set \n");

    //encoder clear
    for(i=0;i<2;i++){
   	
   	buf[0]=0x02;
	if(i==0){
		buf[1]=0x01;
	}else{
		buf[1]=0x02;
	}
    	buf[2]=0x01;
    	buf[3]=0x15;
       	UH_tmp=checksum(&buf[1],3);    
    	buf[4]=ustoc1(UH_tmp);   
    	buf[5]=0;
 
 	res=write(fd, buf, 6);     

	for(j=0; j<5; j++)
	   printf("%x ", buf[j]);
	printf(": %d\n", res);


       	res = read(fd, buf, DATA_MAX_SIZE);
        printf("read \n");
        buf[res]=0;

	for(j=0; j<res; j++)
	   printf("%x ", buf[j]);
	printf(": %d\n", res);

    }


    //periodical response start
    for(i=0;i<2;i++){
   	
   	buf[0]=0x02;
	if(i==0){
		buf[1]=0x01;
	}else{
		buf[1]=0x02;
	}
    	buf[2]=0x03;
    	buf[3]=0x23;
      	buf[4]=0x01;
	buf[5]=0x01;
       	UH_tmp=checksum(&buf[1],5);    
    	buf[6]=ustoc1(UH_tmp);   
    	buf[7]=0;
 
 	res=write(fd, buf, 8);     

	for(j=0; j<7; j++)
	   printf("%x ", buf[j]);
	printf(": %d\n", res);


       	res = read(fd, buf, DATA_MAX_SIZE);
        printf("read \n");
        buf[res]=0;

	for(j=0; j<res; j++)
	   printf("%x ", buf[j]);
	printf(": %d\n", res);
    }


    printf(":\n\n");
    
    { int temp_speed=(int)DRV_SPEED;
    while(temp_speed>-100){
    
    //make drive 
    for(i=0;i<2;i++){
   	
   	buf[0]=0x02;
	if(i==0){
		buf[1]=0x01;
	}else{
		buf[1]=0x02;
	}
    	buf[2]=0x05;
    	buf[3]=0x21;
    	itoc4(temp_speed, &buf[4]);
    	UH_tmp=checksum(&buf[1],7);    
    	buf[8]=ustoc1(UH_tmp);   
    	buf[9]=0;
 
 	res=write(fd, buf, 10);     

	for(j=0; j<9; j++)
	   printf("%x ", buf[j]);
	printf(": %d\n", res);


       	res = read(fd, buf, DATA_MAX_SIZE);
        printf("read \n");
        buf[res]=0;

	for(j=0; j<res; j++)
	   printf("%x ", buf[j]);
	printf(": %d\n", res);

   	printf(":\n");


	//read control parameter
  	buf[0]=0x02;
	if(i==0){
		buf[1]=0x01;
	}else{
		buf[1]=0x02;
	}
    	buf[2]=0x02;
    	buf[3]=0x22;
    	buf[4]=0x01;
    	UH_tmp=checksum(&buf[1],4);    
    	buf[5]=ustoc1(UH_tmp);   
    	buf[6]=0;
       
        res=write(fd, buf, 7);

       	res = read(fd, buf, DATA_MAX_SIZE);
        printf("read \n");
        buf[res]=0;

	for(j=0; j<res; j++)
	   printf("%x ", buf[j]);
	printf(": %d\n", res);
	
	if(res>3){
		if(c2tous(&buf[5])<32768){
			printf("Duty=%d\n",c2tous(&buf[5]));
		}else{
			printf("Duty=%d\n",c2tous(&buf[5])-65536);
		}
	}	

    }

	temp_speed--;

    }//while
    }//middle pa



    printf(":\n\n");
    
    

    //read line
/*    for(i=0;i<1;i++){
       	res = read(fd, buf, DATA_MAX_SIZE);
        printf("read \n");
        buf[res]=0;

	for(j=0; j<res; j++)
	   printf("%x ", buf[j]);
	printf(": %d\n", res);

	if(res>3){
		if(buf[3]==3){
			printf("Duty=%d\n",c2tous(&buf[4]));

		}else if(buf[3]==4){

			if(buf[4]==1){
				printf("Duty=%d\n",c2tous(&buf[5]));
			}else if(buf[4]==0){
				printf("Duty=-%d\n",c2tous(&buf[5]));
			}

		}
	}
    }
*/

    //make stop
    for(i=0;i<2;i++){ //double command

    	buf[0]=0x02;
	if(i==0){
    		buf[1]=0x01;
	}else{
		buf[1]=0x02;
	}
    	buf[2]=0x02;
    	buf[3]=0x20;
    	buf[4]=0x00;
    	UH_tmp=checksum(&buf[1],4);
    	buf[5]=ustoc1(UH_tmp);
    	buf[6]=0;

   	res=write(fd, buf, 6);
    
       	res = read(fd, buf, DATA_MAX_SIZE);
        printf("read \n");
        buf[res]=0;

	for(j=0; j<res; j++)
	   printf("%x ", buf[j]);
	printf(": %d\n", res);
	
    }

    printf("control end \n");

    ioctl(fd, TCSETS, &oldtio);       /* ポートの設定を元に戻す */
    close(fd);                        /* デバイスのクローズ */

    return 0;
}


unsigned short checksum(char *addr, int leng){

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

char ustoc1(unsigned short usd) //little endian version
{
	void *p;
	
	p=&usd;
	return(*((unsigned short *)p+0));
}

void itoc4(int id, char *cd)
{
	int i;
	union{
		int f;
		struct{
			char c[4];
		}i;
	}j;
	
	j.f=id;
	for(i=0;i<4;i++)
		*(cd+i)=j.i.c[i];
}

unsigned short c2tous(char *cd) //little endian version
{
	return((*(cd+1) << 8) + (*(cd+0) & 0xff));
}
