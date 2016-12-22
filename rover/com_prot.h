/* com_prot.h */
/* M6 Motor Controller FPGA Programming */
/* programmed by M. Otsuki 2009/June */

//-------------------//
//--  include      --//
//-------------------//

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>

#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/param.h>
#include <sys/time.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/resource.h>

//#include "main_aux.h"

//-------------------//
//--  define      --//
//-------------------//

#define EBUF_MAX_SIZ (255) 	// bytes: Ethernet data number
#define EBUF_HSIZE (12)	// Ethernet header byte size

//Ethernet relations
#define FRM_IND 0x0020f3fa00000000
#define FRM_IND_SIZ 8

#define PORT_NO 13000

#define ARM_IP  "192.168.201.11"
#define GND_IP  "192.168.201.81"

#define EBUF_TLM_SIZ (EBUF_HSIZE+sizeof(char)+sizeof(float))
#define EBUF_CMD_SIZ (EBUF_HSIZE+sizeof(char)+sizeof(float))

#define NG 0
#define OK 1

//readline 

#define MAXLINE 1

//-------------------//
//--  prototype    --//
//-------------------//

//Communication Data type//
typedef struct{
	unsigned char frm_ind[8];//0x0020f3fa00000000
	unsigned short id;	//1-7
	unsigned short upd_cnt;	//0-65535
}sgl_hdr;

typedef struct{
	const char addr[16];
	const unsigned short portn;
}tcp_info;

typedef struct{
	int rl_cnt;
	char *rl_bufptr;	//initializing with 0
	char rl_buf[MAXLINE]; 	//initializing with rl_buf
}Rline;

//-------------------//
//--  functions    --//
//-------------------//
void ftoc4(float fd, unsigned char *cd)
{
	int i;
	union{
		float f;
		struct{
			unsigned char c[4];
		}i;
	}j;
	
	j.f=fd;
	for(i=0;i<4;i++) 
		*(cd+i)=j.i.c[i];
}

float c4tof(unsigned char *cd)
{
	int i;
	union{
		float f;
		struct{
			unsigned char c[4];
		}i;
	}j;

	for(i=0;i<4;i++) 
		j.i.c[i]=*(cd+i);

	return(j.f);
}

void itoc4(int id, unsigned char *cd)
{
	int i;
	union{
		int f;
		struct{
			unsigned char c[4];
		}i;
	}j;
	
	j.f=id;
	for(i=0;i<4;i++)
		*(cd+i)=j.i.c[i];
}

int c4toi(unsigned char *cd)
{
	int i;
	union{
		int f;
		struct{
			unsigned char c[4];
		}i;
	}j;

	for(i=0;i<4;i++)
		j.i.c[i]=*(cd+i);

	return(j.f);
}

unsigned int c4toui(unsigned char *cd)
{
	int i;
	union{
		unsigned int f;
		struct{
			unsigned char c[4];
		}i;
	}j;

	for(i=0;i<4;i++)
		j.i.c[i]=*(cd+i);

	return(j.f);
}

void ustoc2(unsigned short usd, unsigned char *cd) //little endian version
{
	void* p;
	
	p=&usd;
	
	*(cd+1)=*((unsigned char *)p+1);
	*(cd+0)=*((unsigned char *)p+0);
}

unsigned short c2tous(unsigned char *cd) //little endian version
{
	return((*(cd+1) << 8) + (*(cd+0) & 0xff));
}

int c1toi(unsigned char cd)
{
	return((int)cd);
}

unsigned char itoc1(int usd) //little endian version
{
	void* p;
	
	p=&usd;
	return(*((unsigned char *)p+0));
}

unsigned short c1tous(unsigned char cd)
{
	return((unsigned short)cd);
}

unsigned char ustoc1(unsigned short usd) //little endian version
{
	void* p;
	
	p=&usd;
	return(*((unsigned char *)p+0));
}

void printf_eth_data(unsigned char *data, int csize)
{
	int i=0;

	printf("\n 0x");
	
	for(i=0;i<csize;i++){
		printf("%x ", *(data+i));
	}

	printf(" dsize=%d \n", csize);
	
}


unsigned short char_cksum(unsigned char *addr, int leng) //unsigned char data[256], leng = sizeof(data)
{
	int nleft=leng; //1byte order
	int sum=0;
	unsigned char *w=addr;
	unsigned short answer=0;

	while(nleft > 0)
	{
		sum+=*w++;
		nleft-=1;
	}
	sum=(sum>>16) + (sum&0xffff); //addition of upper and lower value
	sum+=(sum>>16); //adding carry value
	answer=~sum; //inversion of value
	return(answer);
}


unsigned short shortswap(unsigned char *tmp)
{
	return((unsigned short)(tmp[1]) + (unsigned short)(tmp[0]<<8));
}

int longswap(unsigned char *tmp)
{
	return((int)(tmp[3]) + (int)(tmp[2]<<8) + 
			(int)(tmp[1]<<16) + (int)(tmp[0]<<24));
}

unsigned short shortnoswap(unsigned char *tmp)
{
	return((unsigned short)(tmp[0]) + (unsigned short)(tmp[1]<<8));
}

int longnoswap(unsigned char *tmp)
{
	return((int)(tmp[0]) + (int)(tmp[1]<<8) + 
			(int)(tmp[2]<<16) + (int)(tmp[3]<<24));
}

/*
 * trb_act_com_whl_fid_pos
 */
 


int fid_pos(char *buf, int siz, int *pos)
{
  int i;

  for (i=0; i<siz; i++){
    if(buf[i] == (char)0x00){
      goto trb_act_com_whl_rcv_chk_2nd;
    }
  }
  *pos = 0;
  return NG;

trb_act_com_whl_rcv_chk_2nd:
  for (i++; i<siz; i++){
    if (buf[i] == (char)0x20){
      goto trb_act_com_whl_rcv_chk_3rd;
    }
  }
  *pos = 1;
  return NG;

trb_act_com_whl_rcv_chk_3rd:
  for (i++; i<siz; i++){
    if (buf[i] == (char)0xF3){
      goto trb_act_com_whl_rcv_chk_4th;
    }
  }
  *pos = 2;
  return NG;

trb_act_com_whl_rcv_chk_4th:
  for (i++; i<siz; i++){
    if (buf[i] == (char)0xFA){
      goto trb_act_com_whl_rcv_chk_5th;
    }
  }
  *pos = 3;
  return NG;

trb_act_com_whl_rcv_chk_5th:
  for (i++; i<siz; i++){
    if (buf[i] == (char)0x0){
      goto trb_act_com_whl_rcv_chk_6th;
    }
  }
  *pos = 4;
  return NG;

trb_act_com_whl_rcv_chk_6th:
  for (i++; i<siz; i++){
    if (buf[i] == (char)0x0){
      goto trb_act_com_whl_rcv_chk_7th;
    }
  }
  *pos = 5;
  return NG;

trb_act_com_whl_rcv_chk_7th:
  for (i++; i<siz; i++){
    if (buf[i] == (char)0x0){
      goto trb_act_com_whl_rcv_chk_8th;
    }
  }
  *pos = 6;
  return NG;

trb_act_com_whl_rcv_chk_8th:
  for (i++; i<siz; i++){
    if (buf[i] == (char)0x0){
      goto trb_act_com_whl_rcv_chk_end;
    }
  }
  *pos = 7;
  return NG;

trb_act_com_whl_rcv_chk_end:
  *pos = i-7;
  return OK;
}












//--------------------------olds----------------------------//

/*

void ftous2(float fd, unsigned short *usd)
{
	union{
       float f;
       struct{

            unsigned short ym, yl;
      }i;
    }j;
   
   	j.f = fd;
   	*(usd+0) = j.i.ym;
   	*(usd+1) = j.i.yl;	
}
	
float us2tof(unsigned short usd1, unsigned short usd2)
{
	union{
       float f;
       struct{
            unsigned short ym, yl;
      }i;
    }j;
   
   	j.i.ym = usd1;
	j.i.yl = usd2;
	
	return(j.f);
}

float ultof(int cd)
{
	union{
       float f;
       struct{
            int ul;
      }i;
    }j;
    
      j.i.ul=cd;
      
      return(j.f);
}

int intof(int cd)
{
	union{
       int f;
       struct{
            int ul;
      }i;
    }j;
    
      j.i.ul=cd;
      
      return(j.f);
}

void ultoc4(int ld, unsigned char *cd)
{
	int i;
	union{
       		int l;
       		struct{
            		unsigned char c[4];
      		}i;
    	}j;
      
      j.l=ld;
      for(i=0;i<4;i++)
      	*(cd+i)=j.i.c[i];
}

void ltoc4(long ld, unsigned char *cd)
{
	int i;
	union{
       		long l;
       		struct{
            		unsigned char c[4];
      		}i;
    	}j;
      
      j.l=ld;
      for(i=0;i<4;i++)
      	*(cd+i)=j.i.c[i];
}

int c4toul(unsigned char *cd)
{
	int i;
	union{
       int l;
       struct{
            unsigned char c[4];
      }i;
    }j;
    
    for(i=0;i<4;i++)
      j.i.c[i]=*(cd+i);
      
      return(j.l);
}

long c4tol(unsigned char *cd)
{
	int i;
	union{
       long l;
       struct{
            unsigned char c[4];
      }i;
    }j;
    
    for(i=0;i<4;i++)
      j.i.c[i]=*(cd+i);
      
      return(j.l);
}

unsigned short ctous(unsigned char cd)
{
	return((unsigned short)cd);
}

unsigned char ustoc(unsigned short usd)
{
	void *p;
	
	p=&usd;
	return(*((unsigned char *)p+0));
}

void stoc2(short usd, unsigned char *cd)
{
	void *p;
	
	p=&usd;
	
	*(cd+1)=*((unsigned char *)p+0);
	*(cd+0)=*((unsigned char *)p+1);
}

short c2tos(unsigned char *cd)
{
	return((*(cd+0) << 8) + (*(cd+1) & 0xff));
}

void printf_disp(void *p, int num)
{
	int i;
	for(i=0;i<num;i++)
		printf("#%d: 0x%x\t",i+1,*((unsigned char *)(p+i)));
	printf("\n");
}

void get_eth_header(unsigned char *get_data, struct sig_header *psh)
{
	psh->frmt_id = ctous(*(get_data+0));
	psh->sender_id = ctous(*(get_data+1));
	psh->type = ctous(*(get_data+2));
	psh->leng = ctous(*(get_data+3));
}

void make_eth_header(struct sig_header *psh, unsigned char *get_data)
{
	*(get_data+0) = ustoc(psh->frmt_id);
	*(get_data+1) = ustoc(psh->sender_id);
	*(get_data+2) = ustoc(psh->type);
	*(get_data+3) = ustoc(psh->leng);
}

*/

//calculating checksum
/*unsigned short us_cksum(unsigned short *addr, int leng) //unsigned short data[128], leng = sizeof(data)
{
	int nleft=leng; //1byte order
	int sum=0;
	unsigned short *w=addr;
	unsigned short answer=0;

	while(nleft > 1)
	{
		sum+=*w++;
		nleft-= 2;
	}

	sum=(sum>>16) + (sum&0xffff); //addition of upper and lower value
	sum+=(sum>>16); //adding carry value
	answer=~sum; //inversion of value
	return(answer);
}*/

/*
int pck_com_dat_long(struct sig_header *sigh, long *pckdata, unsigned char *sdata)
{
	int i,bsize=sigh->leng+EBUF_HSIZE;
	unsigned short chksum, uswap;
	unsigned char c[4];
	int lswap;
	
	make_eth_header(sigh, sdata);
	
	//due to type ID//
	for(i=0;i<(sigh->leng/sizeof(long));i++)
	{
		ltoc4(*(pckdata+i),c);
		lswap=longswap(c);
		ultoc4(lswap,(sdata+sizeof(long)*i+sizeof(struct sig_header)/sizeof(unsigned short)));
	}
	
	chksum=char_cksum(sdata,bsize-sizeof(unsigned short));
	uswap=shortswap((unsigned char *)&chksum);
	ustoc2(uswap,(sdata+bsize-sizeof(unsigned short)));

	//for(i=0;i<bsize;i++)
	//	printf("#%d send_data pack=0x%x \n",i+1,*(sdata+i));
	return(bsize);
}
*/

/*int pck_com_dat_s(struct sig_header *sigh, short *pckdata, unsigned char *sdata)
{
	int i,bsize=sigh->leng+EBUF_HSIZE;
	unsigned short chksum, uswap;
	unsigned char c[2];
	
	make_eth_header(sigh, sdata);
	
	//due to type ID//
	for(i=0;i<(sigh->leng/sizeof(unsigned short));i++)
	{
		stoc2(*(pckdata+i),c);
		uswap=shortswap(c);
		ustoc2(uswap,(sdata+sizeof(unsigned short)*i+sizeof(struct sig_header)/sizeof(unsigned short)));
	}
	
	chksum=char_cksum(sdata,bsize-sizeof(unsigned short));
	uswap=shortswap((unsigned char *)&chksum);
	ustoc2(uswap,(sdata+bsize-sizeof(unsigned short)));

	//for(i=0;i<bsize;i++)
	//	printf("#%d send_data pack=0x%x \n",i+1,*(sdata+i));
	return(bsize);
}


int pck_com_dat_us(struct sig_header *sigh, unsigned short *pckdata, unsigned char *sdata)
{
	int i,bsize=sigh->leng+EBUF_HSIZE;
	unsigned short chksum, uswap;
	unsigned char c[2];
	
	make_eth_header(sigh, sdata);
	
	//due to type ID//
	for(i=0;i<(sigh->leng/sizeof(unsigned short));i++)
	{
		ustoc2(*(pckdata+i),c);
		uswap=shortswap(c);
		ustoc2(uswap,(sdata+sizeof(unsigned short)*i+sizeof(struct sig_header)/sizeof(unsigned short)));
	}
	
	chksum=char_cksum(sdata,bsize-sizeof(unsigned short));
	uswap=shortswap((unsigned char *)&chksum);
	ustoc2(uswap,(sdata+bsize-sizeof(unsigned short)));

	//for(i=0;i<bsize;i++)
	//	printf("#%d send_data pack=0x%x \n",i+1,*(sdata+i));
	return(bsize);
}

int pck_com_dat(struct sig_header *sigh, float *pckdata, unsigned char *sdata)
{
	int i,bsize=sigh->leng+EBUF_HSIZE;
	int fswap;
	unsigned short chksum, uswap;
	unsigned char c[4];
	
	make_eth_header(sigh, sdata);
	
	//due to type ID//
	for(i=0;i<(sigh->leng/sizeof(float));i++)
	{
		ftoc4(*(pckdata+i),c);
		fswap=longswap(c);
		ultoc4(fswap,(sdata+sizeof(float)*i+sizeof(struct sig_header)/sizeof(unsigned short)));
	}
	
	chksum=char_cksum(sdata,bsize-sizeof(unsigned short));
	uswap=shortswap((unsigned char *)&chksum);
	ustoc2(uswap,(sdata+bsize-sizeof(unsigned short)));

	//for(i=0;i<bsize;i++)
	//	printf("#%d send_data pack=0x%x \n",i+1,*(sdata+i));
	return(bsize);
}

int pck_com_dat_int(struct sig_header *sigh, int *pckdata, unsigned char *sdata)
{
	int i,bsize=sigh->leng+EBUF_HSIZE;
	int fswap;
	unsigned short chksum, uswap;
	unsigned char c[4];
	
	make_eth_header(sigh, sdata);
	
	//due to type ID//
	for(i=0;i<(sigh->leng/sizeof(int));i++)
	{
		intoc4(*(pckdata+i),c);
		fswap=longswap(c);
		ultoc4(fswap,(sdata+sizeof(int)*i+sizeof(struct sig_header)/sizeof(unsigned short)));
	}
	
	chksum=char_cksum(sdata,bsize-sizeof(unsigned short));
	uswap=shortswap((unsigned char *)&chksum);
	ustoc2(uswap,(sdata+bsize-sizeof(unsigned short)));

	//for(i=0;i<bsize;i++)
	//	printf("#%d send_data pack=0x%x \n",i+1,*(sdata+i));
	return(bsize);
}*/

/*
void unpck_com_dat_long(unsigned char *get_data, struct sig_header *sig_r, long *rdata)
{
	int i,asize;
	int lswap;
	unsigned short uswap,temp;
	unsigned char c[sizeof(long)];

	//get header
	get_eth_header(get_data,sig_r);
	asize=sig_r->leng+EBUF_HSIZE;
	
	//get check sum
	temp=c2tous(get_data+asize-sizeof(unsigned short));
	uswap=shortswap((unsigned char *)&temp);
	
	//due to type ID//
	//if((char_cksum(get_data,asize-sizeof(unsigned short))==uswap)==1)
	//{
		for(i=0;i<(sig_r->leng/sizeof(long));i++)
		{
			lswap=longswap(get_data+sizeof(long)*i+sizeof(struct sig_header)/sizeof(unsigned short)); 
			ultoc4(lswap,c);
			*(rdata+i) = c4tol(c);
		}		
		//}
		
	//printf("chksum 0x%x \n", uswap);
	//printf("chksum 0x%x \n", char_cksum(get_data,asize-sizeof(unsigned short)));
}
*/

/*
void unpck_com_dat_us(unsigned char *get_data, struct sig_header *sig_r, unsigned short *rdata)
{
	int i,asize;
	unsigned short uswap,temp;
	unsigned char c[sizeof(unsigned short)];

	//get header
	get_eth_header(get_data,sig_r);
	asize=sig_r->leng+EBUF_HSIZE;
	
	//get check sum
	temp=c2tous(get_data+asize-sizeof(unsigned short));
	uswap=shortswap((unsigned char *)&temp);
	
	//due to type ID//
	//if((char_cksum(get_data,asize-sizeof(unsigned short))==uswap)==1)
	//{
	for(i=0;i<(sig_r->leng/sizeof(unsigned short));i++)
	{
		uswap=shortswap(get_data+sizeof(unsigned short)*i+sizeof(struct sig_header)/sizeof(unsigned short));
		ustoc2(uswap,c);
		*(rdata+i) = c2tous(c);
		//printf("xd=%d yd=%d",*rdata+0, *rdata+1);
	}
	
	//printf("chksum 0x%x \n", uswap);
	//printf("chksum 0x%x \n", char_cksum(get_data,asize-sizeof(unsigned short)));
}


void unpck_com_dat(unsigned char *get_data, struct sig_header *sig_r, float *rdata)
{
	int i,asize;
	int fswap;
	unsigned short uswap,temp;
	unsigned char c[sizeof(float)];

	//get header
	get_eth_header(get_data,sig_r);
	asize=sig_r->leng+EBUF_HSIZE;
	
	//get check sum
	temp=c2tous(get_data+asize-sizeof(unsigned short));
	uswap=shortswap((unsigned char *)&temp);
	
	//due to type ID//
	//if((char_cksum(get_data,asize-sizeof(unsigned short))==uswap)==1)
	//{
		for(i=0;i<(sig_r->leng/sizeof(float));i++)
		{
			fswap=longswap(get_data+sizeof(float)*i+sizeof(struct sig_header)/sizeof(unsigned short)); 
			ultoc4(fswap,c);
			*(rdata+i) = c4tof(c);
			*(rdata+i) = c4tof(get_data+4*i+4);
		}		
		//}
		
	//printf("chksum 0x%x \n", uswap);
	//printf("chksum 0x%x \n", char_cksum(get_data,asize-sizeof(unsigned short)));
}

void unpck_com_dat_mix(unsigned char *get_data, struct sig_header *sig_r, short *rdata1, float *rdata2)
{
	int asize;
	int fswap2;
	unsigned short fswap1;
	unsigned short uswap,temp;
	unsigned char c1[sizeof(short)],c2[sizeof(float)];

	//get header
	get_eth_header(get_data,sig_r);
	asize=sig_r->leng+EBUF_HSIZE;
	
	//get check sum
	temp=c2tous(get_data+asize-sizeof(unsigned short));
	uswap=shortswap((unsigned char *)&temp);
	
	//due to type ID//
	fswap1=shortswap(get_data+sizeof(struct sig_header)/sizeof(unsigned short));  
	ustoc2(fswap1,c1);

	fswap2=longswap(get_data+sizeof(short)+sizeof(struct sig_header)/sizeof(unsigned short));  
	ultoc4(fswap2,c2);

	*rdata1 = c2tos(c1);
	*rdata2 = c4tof(c2);
}

void unpck_com_dat_mix2(unsigned char *get_data, struct sig_header *sig_r, int *rdata1, float *rdata2)
{
	int asize;
	int fswap2;
	unsigned short fswap1;
	unsigned short uswap,temp;
	unsigned char c1[sizeof(int)],c2[sizeof(float)];

	//get header
	get_eth_header(get_data,sig_r);
	asize=sig_r->leng+EBUF_HSIZE;
	
	//get check sum
	temp=c2tous(get_data+asize-sizeof(unsigned short));
	uswap=shortswap((unsigned char *)&temp);
	
	//due to type ID//
	fswap1=longswap(get_data+sizeof(struct sig_header)/sizeof(unsigned short));  
	ultoc4(fswap1,c1);

	fswap2=longswap(get_data+sizeof(short)+sizeof(struct sig_header)/sizeof(unsigned short));  
	ultoc4(fswap2,c2);

	*rdata1 = c4toin(c1);
	*rdata2 = c4tof(c2);
}*/

//---------------------//
//under review of usage//
//---------------------//
/*
void chngt_carry(struct sig_header head, float *fdata, 
unsigned short unum, unsigned short cksum, unsigned char *cdata)
{ 
	int i;
	
	*(cdata+0)=ustoc(head.frmt_id);
	*(cdata+1)=ustoc(head.sender_id);
	*(cdata+2)=ustoc(head.type);
	*(cdata+3)=ustoc(head.type);
	
	for(i=0;i<unum;i++)
		ftoc4(fdata[i],(cdata+4*i+4));
		
	ustoc2(cksum,(cdata+4*unum+4));
}

*/


