/* tcp_func.h */
/* M6 Motor Controller FPGA Programming */
/* programmed by M. Otsuki 2009/June */

#include "com_prot.h"
#include <arpa/inet.h>
#include <netinet/in.h>

#define THR_TCP_NMB (100)

//tcp_info 
tcp_info
ARM = {ARM_IP, PORT_NO+0},
GND = {GND_IP, PORT_NO+0};


/* MC is always server */

//---------------------//
//--Ethernet relations-//
//---------------------//

static pthread_key_t rl_key;
static pthread_once_t rl_once=PTHREAD_ONCE_INIT;

int srvfd;
struct sockaddr_in clt;

pthread_t thrd_tcp_num[THR_TCP_NMB];
int rcv_cmd(int, char*, float*);
extern void cmd_exe(char, float);
int cfd;

void *func_accept(void *arg){

	socklen_t cltlen;
	int cfd, rft;
	static int ch=0;
	char cmd_chr[1];
	float cmd_val=0.0;

	ch++;
	
	cltlen = sizeof(clt);
	if((cfd = accept(srvfd, (struct sockaddr *) &clt, &cltlen))< 0){
		printf("until accept \n");
	}else{
		printf("after accept[%d] \n", ch);
		rft=pthread_create(&thrd_tcp_num[THR_TCP_NMB-ch], NULL, (void *)func_accept, (void*) &cfd);
	}
	
	for(;;){
		
		if(rcv_cmd(cfd, &cmd_chr[0], &cmd_val)==OK) //command recognition
		{
			printf("receved data character : %c and value=%f \n", cmd_chr[0], cmd_val);
					
			//printf("you are in command handling \n");
			cmd_exe(cmd_chr[0], cmd_val); //command execution
		}
		
		cmd_chr[0]=0;
		cmd_val = 0.0;
	}			

}

int tcp_listen(tcp_info *info) //for server
{
	//int srvfd;
	struct sockaddr_in srvaddr;
	const int one=1;
	socklen_t cltlen;
	//struct sockaddr_in clt;
	
	srvfd = socket(AF_INET, SOCK_STREAM, 0);	
	printf("socket \n");
	if(setsockopt(srvfd, SOL_SOCKET, SO_REUSEADDR, (char *)&one, sizeof(int))<0){
		printf("until socket option \n ");
		return(0);
	}
	printf("setsockopt \n");
	
	memset(&srvaddr, 0, sizeof(srvaddr)); 				//substituting 0 into structure
	srvaddr.sin_family = AF_INET;					//Internet Address Family
	srvaddr.sin_addr.s_addr = INADDR_ANY;
	//srvaddr.sin_addr.s_addr = inet_addr(info->addr);	//Server IP Address
	//printf("hallo world 1 \n");
	//srvaddr.sin_port = htons(info->portn);
	srvaddr.sin_port = shortswap((unsigned char *) &(info->portn));				//Server Port number
	//printf("after swap = 0x%x, before swap = 0x%x",srvaddr.sin_port, info->portn);

	// Bind to server(local) address //
	if(bind(srvfd, (struct sockaddr *) &srvaddr, sizeof(srvaddr)) < 0){
		printf("until bind \n");
		return(0);
	}
	printf("bind \n");
	
	// Listen to client //
	if(listen(srvfd, 1) < 0){
		printf("until listen \n");
		return(0);
	}
	printf("listen \n");
	
	// accept //	
	printf("before accept \n");
	cltlen = sizeof(clt);
	if((cfd=accept(srvfd, (struct sockaddr *) &clt, &cltlen))< 0){
		printf("until accept \n");
		return(0);
	}else{
		printf("after accept \n");
		pthread_create(&thrd_tcp_num[THR_TCP_NMB], NULL, (void *)func_accept, (void*) &cfd);
	}
	return(cfd);
}


int tcp_connect(tcp_info *info)
{	
	int cltfd=0;
	struct sockaddr_in cltaddr;
	const int one=1;
	
	cltfd = socket(AF_INET, SOCK_STREAM, 0);	
	if(setsockopt(cltfd, SOL_SOCKET, SO_REUSEADDR, (char *)&one, sizeof(int))<0)
		return(0);

	memset(&cltaddr,0,sizeof(cltaddr)); 				//substituting 0 into structure
	cltaddr.sin_family = AF_INET;					//Internet Address Family
	cltaddr.sin_addr.s_addr = inet_addr(info->addr);	//Server IP Address
	cltaddr.sin_port = shortswap((unsigned char *) &(info->portn));				//Server Port number
	
	// Connecting Server //
	if(connect(cltfd, (struct sockaddr *) &cltaddr, sizeof(cltaddr)) < 0 )
		return(0);
		
	return(cltfd);
}


static void readline_destructor(void *ptr)
{
	free(ptr);
}

static void readline_once(void){
	pthread_key_create(&rl_key,readline_destructor);
}

static ssize_t my_read(Rline *tsd, int fd, char *ptr)
{
	if(tsd->rl_cnt <=0){
again:
		if((tsd->rl_cnt = read(fd,tsd->rl_buf, MAXLINE))<0){
			if(errno == EINTR)
				goto again;
			return(-1);
		}else if(tsd->rl_cnt ==0)
			return(0);
		tsd->rl_bufptr = tsd->rl_buf;
	}
	tsd->rl_cnt--;
	*ptr=*tsd->rl_bufptr++;
	return(1);
}

ssize_t readline(int fd, void* vptr, size_t maxlen) 
{
	int n,rc;
	//inttlen;
	char c, *ptr;
	Rline *tsd;
	
	maxlen = maxlen;
	
	pthread_once(&rl_once,readline_once);
	
	if((tsd=pthread_getspecific(rl_key))==NULL){
		tsd=calloc(1,sizeof(Rline));
		pthread_setspecific(rl_key,tsd);
	}

	//get data length
	ptr=vptr;
/*
	for(n=1;n<(sizeof(EBUF_HDR_SIZ)-sizeof(UH)+1);n++){
		if((rc=my_read(tsd,fd,&c))==1){
			*ptr++=c;
		}else if(rc==0){
			if(n==1)
				return(0); //EOF, no data
			else
				break;	//EOF, get data
		}else
			return(-1); //error
	}	
	tlen=c1toi(*(ptr-1)); //little endian version
*/	//prinTf("%d \n", tlen);
	
	//read residual data
	for(n=1;n<(int)EBUF_CMD_SIZ+1;n++){
		if((rc=my_read(tsd,fd,&c))==1){
			*ptr++=c;
		}else if(rc==0){
			if(n==1)
				return(0); //EOF, no data
			else
				break;	//EOF, get data
		}else
			return(-1); //error
	}
	return(n-1); //return number of data
}


void rst_head(sgl_hdr *head){ //reset header
	
	head->id=0;
	head->upd_cnt=(unsigned short)0;

}


void make_cmd_head(unsigned char *sdata, unsigned short cmder_id){ //make telemetry head
	
	int n=0;
	static unsigned short upd_cnt=0;
	
	sdata[n]=0x00;	n++;
	sdata[n]=0x20;	n++;
	sdata[n]=0xf3;	n++;
	sdata[n]=0xfa;	n++;
	sdata[n]=0x00;	n++;
	sdata[n]=0x00;	n++;
	sdata[n]=0x00;	n++;
	sdata[n]=0x00;	n++;
	
	ustoc2(cmder_id, &sdata[n]);
	n += sizeof(unsigned short);
	
	ustoc2(upd_cnt, &sdata[n]);
	
	upd_cnt++;
	
}


void make_data(unsigned char* cmd_char, float cmd_val, 
unsigned short cmdr_id, unsigned char *sdata){ //for making command data
	
	int n=0;
	
	make_cmd_head(sdata, cmdr_id);
	n = EBUF_HSIZE;
	
	sdata[n] = *cmd_char;
	n += sizeof(unsigned char);

	ftoc4(cmd_val, &sdata[n]);
	n += sizeof(float);

}


int init_cmd_tcp(unsigned char *cmd_data, tcp_info COMID){
	int com_fd=0;
	
	//cmd_data = (unsigned char *) calloc( EBUF_CMD_SIZ, sizeof( unsigned char ) );
	
	memset(cmd_data, 0, EBUF_CMD_SIZ);
	
	if((com_fd=tcp_connect(&COMID))!=0){
		printf("\n\n TCP connection established \n\n");
	}else{
		printf("\n\n Fail to make a connection with the server");
		com_fd = -1;
	}
		
	return com_fd;
}	

void snd_cmd(unsigned char *sdata, int buf_siz, int connfd){

	int n=0, num=0;
		 	
	while(n < buf_siz){
		if((num=write(connfd, sdata, EBUF_CMD_SIZ)) < 0){
			memset(sdata, 0, EBUF_CMD_SIZ);
			printf("write failed \n");
			break;
		}
				
		n += num;
	}
	
}

void get_head(char *rdata, sgl_hdr *rdt_hdr){ //recognize header in command
	
	int n=0;
	
	n += FRM_IND_SIZ;
	
	rdt_hdr->id=c2tous(rdata+n);
	n += sizeof(unsigned short);
	rdt_hdr->upd_cnt=c2tous(rdata+n);
	n += sizeof(unsigned short);
	
	//printf_eth_data(rdata);

}

int rcv_cmd(int fd, char *temp_c, float *temp_f) 
{ 
	int rc, flag=NG, t=0;
	unsigned char rdata[EBUF_CMD_SIZ];
	sgl_hdr rdt_hdr;
	
	memset(rdata, 0, EBUF_CMD_SIZ);
	temp_c[0]=0;
	*temp_f=0.0;
	
	if(fd!=0){
		
		rc=readline(fd, rdata, EBUF_CMD_SIZ);
		
		if(rc>0){
			flag=(int)OK;
		}else if(rc==0){
			flag=(int)NG;			
		}else{
			flag=(int)NG;
		}
	}	
	
	if(flag==OK){
		
		get_head(rdata, &rdt_hdr); //header data convert
		
		printf_eth_data(rdata,rc);
		
		//check frame indicator	
		if(fid_pos(rdata, EBUF_CMD_SIZ, &t)==OK){
			temp_c[0] = rdata[EBUF_HSIZE];
			*temp_f = c4tof(&rdata[EBUF_HSIZE+1]);
			
			return OK;			
		}else{
			*temp_f = 0.0;
			temp_c[0] = 0;
			return NG;
		}
			
	}else if(flag==NG){
		temp_c[0]=0;
		*temp_f=0.0;
		return NG;
	}else{
		temp_c[0]=0;
		*temp_f=0.0;
		return NG;
	}
	
}


/*
void printf_eth_data(unsigned char *data, int csize)
{
	int i=0;

	printf("\n 0x");
	
	for(i=0;i<csize;i++){
		printf("%x ", *(data+i));
	}

	printf(" dsize=%d \n", csize);
	
}
*/
