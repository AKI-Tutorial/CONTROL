//io_func.h

#include <unistd.h>
#include <sys/io.h>
#include <stdio.h>
#include <stdint.h>
#include <getopt.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#define AD_BAS_ADR 		(0x0380)
#define AD_CHN(offset)	((offset) + AD_BAS_ADR)
#define AD_ALL_ON_MASK 	(0x16)
#define AD_4CH_ON_MASK 	(0x16) //ch1-4 on
#define AD_CHN_NMB 		(8)
#define AD_RNG_DIS 		(0x00)
#define AD_RNG_PM_3V 	(0x01)
#define AD_RNG_M_5V		(0x02)
#define AD_RNG_P_5V		(0x03)
#define AD_RNG_PM_5V 	(0x04)
#define AD_RNG_M_10V	(0x05)
#define AD_RNG_P_10V	(0x06)
#define AD_RNG_PM_10V 	(0x07)
#define AD_RNG_DIS_DIF	(0x08)
#define AD_RNG_DIF_PM_5V	(0x09)
#define AD_RNG_DIS_PM_10V	(0x0a)
#define AD_RNG_DIS_PM_20V	(0x0b)
#define AD_RNG_MSK		(AD_RNG_P_10V)

#define AD_HLF_DAT_SIZ 	(8)
#define AD_FLL_DAT_SIZ 	(16)

#define AD_ABS_VAL_5V	(6.144)
#define AD_ABS_VAL_3V 	(3.072)
#define AD_ABS_VAL_10V 	(12.288)

#define AD_FLL_BIT 		(65536)
#define AD_HLF_BIT 		(32768)

#define ADD_ACQ_RATE	20//Hz

#define POT_NMB 		4
#define POT_OUT_MIN 	(0.5)
#define POT_OUT_MAX 	(10)
#define POT_CFC 		(360.0/(float)(POT_OUT_MAX-POT_OUT_MIN)) //deg/V

//memmap relations
#define AD_DEVICE "/dev/mem"
#define A460_PC104_IO_8_BASE (0xb2000000)
#define A460_PC104_IO_8_SIZE (0x10000)

//miscellaneous
#define AD_ON 	1
#define AD_OFF 	0

//read, write function definition
static void *isoadc16_base;
#define isoadc16_readb(addr) ({*(volatile uint8_t *)((uintptr_t)isoadc16_base + addr);})
#define isoadc16_writeb(addr,data) ({ *(volatile uint8_t *)((uintptr_t)isoadc16_base + addr) = data;})

typedef struct{
	unsigned char _H, _L;
	unsigned short _HL;
	float vlt;
	float eng;
}add;

add ad_data[AD_CHN_NMB];


//functions

/*
static inline uint8_t _get_status(void){
	
	volatile uint8_t stat;
	
	stat = isoadc16_readb(AD_BAS_ADR);
	
	return stat;
}
*/

static int xmmap(void){
	
	int fd;
	
	fd = open(AD_DEVICE, O_RDWR | O_SYNC);
	
	if(fd<0){
		perror("open");
		return -1;
	}
	
	isoadc16_base = mmap(0x0, A460_PC104_IO_8_SIZE, PROT_READ | 
	PROT_WRITE, MAP_SHARED, fd, A460_PC104_IO_8_BASE);
	
	printf("0x %x \n", (unsigned int)isoadc16_base);
	
	if(isoadc16_base==MAP_FAILED){
		perror("mmap");
		return -1;
	}
	
	close(fd);
	
	return 0;
}


static void xmunmap(void){
	
	munmap(isoadc16_base, A460_PC104_IO_8_SIZE);

}


void init_add_prt(){

	int ch=0;
	int error_no, ret=0;
	
	ret = xmmap();
	if(ret) perror("init add prt of xmmap");
	else printf("success xmmap \n");
	
	ret = atexit(xmunmap);
	if(ret) perror("init add port of atexit");
	else printf("success xmunmap \n");
		
	/*	
	error_no=ioperm((unsigned long)(AD_BAS_ADR), (unsigned long)AD_ALL_ON_MASK, (int)AD_ON);

	if(error_no==-1){
		printf("ioperm error %d \n", error_no);
		if(error_no==EPERM) printf("error EPERM");
		if(error_no==EINVAL) printf("error EINVAL");
		if(error_no==EIO) printf("error EIO");
		if(error_no==ENOMEM) printf("error ENOMEM");
	}else{
		printf("after ioperm \n");
		
	}*/
	
	
	for(ch=0;ch<AD_CHN_NMB;ch++){
		
		//outb((unsigned char)AD_RNG_MSK, (unsigned short int)(AD_BAS_ADR+2*ch));
		//isoadc16_writeb((AD_BAS_ADR+2*ch), (uint8_t)AD_RNG_MSK);
		isoadc16_writeb(AD_CHN(2*ch), (uint8_t)AD_RNG_MSK);
		
		printf("write AD port in ch%d: %x / %x\n", ch+0, AD_CHN(2*ch), AD_RNG_MSK);
	}
	
	
}


void get_add_ch(int ch, add *ad_raw_data){
	
	//volatile uint8_t *stat_H, *stat_L;
	static int i[8], j=0;
	
	if(j==0){
		memset(i, 0 , 8);
		j++;
	}

	//memset(&(ad_raw_data->_H), 0, sizeof(unsigned char));
	//memset(&(ad_raw_data->_L), 0, sizeof(unsigned char));
	//memset(&(ad_raw_data->_HL), 0, sizeof(unsigned short));
	
	//ad_raw_data->vlt=0.0;
	//ad_raw_data->eng=0.0;
	
	ad_raw_data->_L = (unsigned char)isoadc16_readb(AD_CHN(2*ch));
	ad_raw_data->_H = (unsigned char)isoadc16_readb(AD_CHN(2*ch+1));
	
	/*
	*stat_L = (volatile uint8_t) isoadc16_readb(AD_CHN(2*ch));
	*stat_H = (volatile uint8_t) isoadc16_readb(AD_CHN(2*ch+1));
	
	ad_raw_data->_L = (unsigned char)*stat_L;
	ad_raw_data->_H = (unsigned char)*stat_H;
	*/
	
	ad_raw_data->_HL = ((ad_raw_data->_H) << AD_HLF_DAT_SIZ) | (ad_raw_data->_L);
	
	/*if((i[ch]%10000==1)){
		printf("Read channel[%d] = 0x%x+%x, value=%d \n",ch+1, AD_CHN(2*ch), AD_CHN(2*ch+1),ad_raw_data->_HL);
	}*/
	
	i[ch]++;
	/*
	ad_raw_data->_L = inb((unsigned short int)(AD_BAS_ADR+2*ch));
	ad_raw_data->_H = inb((unsigned short int)(AD_BAS_ADR+2*ch+1));
	ad_raw_data->_HL = (ad_raw_data->_H << AD_HLF_DAT_SIZ) | ad_raw_data->_L;
	*/
	//printf("after inb \n");
}


void get_add_all(add *ad_raw_data){
	
	int ch=0;
	static int cnt=0;
	
	if(cnt==0){
		memset(&(ad_raw_data->_H), 0, sizeof(unsigned char));
		memset(&(ad_raw_data->_L), 0, sizeof(unsigned char));
		memset(&(ad_raw_data->_HL), 0, sizeof(unsigned short));
		
		ad_raw_data->vlt=0.0;
		ad_raw_data->eng=0.0;
		
		cnt++;
	}		
	
	for(ch=0;ch<AD_CHN_NMB;ch++){
		get_add_ch(ch, &ad_raw_data[ch]);
	}
	//printf("after get_add_ch \n");
}


void get_add(){
	
	int ch=0, i=0;
	static int cnt=0;

	if(cnt==0){
		for(i=0;i<AD_CHN_NMB;i++){
			memset(&ad_data[i]._H, 0, sizeof(unsigned char));
			memset(&ad_data[i]._L, 0, sizeof(unsigned char));
			memset(&ad_data[i]._HL, 0, sizeof(unsigned short));
			
			ad_data[i].vlt=0.0;
			ad_data[i].eng=0.0;
		}
		cnt++;
	}
	
	
	//printf("before get add all \n");
	
	get_add_all(ad_data);
	
	//printf("after get add all \n");
	
	for(ch=0;ch<AD_CHN_NMB;ch++){
		
		if(AD_RNG_MSK==AD_RNG_PM_10V){

			ad_data[ch].vlt = (ad_data[ch]._HL - AD_HLF_BIT)*(2*AD_ABS_VAL_10V/AD_FLL_BIT);

		}else if(AD_RNG_MSK==AD_RNG_PM_5V){

			ad_data[ch].vlt = (ad_data[ch]._HL - AD_HLF_BIT)*(2*AD_ABS_VAL_5V/AD_FLL_BIT);

		}else if(AD_RNG_MSK==AD_RNG_PM_3V){

			ad_data[ch].vlt = (ad_data[ch]._HL - AD_HLF_BIT)*(2*AD_ABS_VAL_3V/AD_FLL_BIT);

		}else if(AD_RNG_MSK==AD_RNG_P_10V){

			ad_data[ch].vlt = ad_data[ch]._HL * (AD_ABS_VAL_10V/AD_FLL_BIT);

		}else if(AD_RNG_MSK==AD_RNG_P_5V){

			ad_data[ch].vlt = ad_data[ch]._HL * (AD_ABS_VAL_5V/AD_FLL_BIT);

		}else if(AD_RNG_MSK==AD_RNG_M_10V){

			ad_data[ch].vlt = -1 * (AD_FLL_BIT-ad_data[ch]._HL)*(AD_ABS_VAL_10V/AD_FLL_BIT);

		}else if(AD_RNG_MSK==AD_RNG_M_5V){

			ad_data[ch].vlt = -1 * (AD_FLL_BIT-ad_data[ch]._HL)*(AD_ABS_VAL_5V/AD_FLL_BIT);

		}else{

			ad_data[ch].vlt = 0.0;

		}
		
	}
	
}


void get_eng_ch(int ch, add *ad_tmp){
	
	if(ch<POT_NMB){
		ad_tmp->eng = 1.0 * ( ad_tmp->vlt - (float)POT_OUT_MIN) * (float)POT_CFC + 0.0;
	}else{
		ad_tmp->eng = 1.0 * (ad_tmp->vlt - (float)POT_OUT_MIN) + 0.0;
	}

}

void get_eng_all(){
	
	int ch=0;
	
	for(ch=0;ch<AD_CHN_NMB;ch++){
		get_eng_ch(ch, &ad_data[ch]);
	}
}
