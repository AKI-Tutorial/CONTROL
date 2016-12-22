//main function 
#include "exe_main.h"


int main(int argc, char *argv[]){
	
	int i=0, j=1;
	float rslv_data[2]={0.0,0.0};
	
	printf("Into initializing \n");
	//initializing
	init_all();
		
	//printf("Into while loop \n");
	while(1){

		for(i=0;i<2;i++){
			rslv_data[i] = get_rslv(i);
		}
		printf("angles = %3.2f, %3.2f deg\n", rslv_data[0]*180.0/PI, rslv_data[1]*180.0/PI);
		
		usleep(1000000);
	}
/*
		get_add();
		
		for(i=0;i<AD_CHN_NMB;i++){
			if(j%100000==1)
				printf("ad_data[%d].vlt=%f \n", i, ad_data[i].vlt);
				j=1; //reset			
		}

		j++;
		//printf("in the loop of while");
	}
*/	
	/*while(1){
		
		get_add();
		
		for(i=0;i<AD_CHN_NMB;i++){
			if((j%100000==1))
				printf("ad_data[%d].vlt=%f \n", i, ad_data[i].vlt);			
		}
		
		j++;
	}*/
	
	for(;;){
		signal(SIGINT, sig_fnc);
	}

    return 0;
}
