
/***************************** Include files *******************************/
#include <stdio.h>
#include <math.h>
#include "node_sync.h"
#include "rf24l01.h"
#include "delay.h"
/*****************************    Defines    *******************************/

#define NODE 1	// Should be appiled for this node
/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/
static struct RBS_INFO rbs;			// top node for all infomation
static unsigned char Tx_Buffer[20]={0};	
static unsigned char wearH=0;

const double C = 299792458.0;       // Speed of light
const double tsfreq = 499.2e6 * 128;  // Timestamp counter frequency



int RBS(const exlcm_sync_t * msg)
/*****************************************************************************
*   Input    :
*   Output   :
*   Function :
******************************************************************************/ 	
{
	exlcm_sync_t my_data;		// create a struct to LCM
	static double firstx[ALL_NODES+1],firsty[ALL_NODES+1],secondx[ALL_NODES+1],secondy[ALL_NODES+1];
	static double slope[ALL_NODES+1],y_intercept[ALL_NODES+1];
	static double sync_store[ALL_NODES+1]={0,0,0};
	double result1,result2;
	switch(msg->operation)		// statemachine of handling LCM packets
		{
			case BEACONSENDER:	// case when beacon message is recieved	
				if(NODE == 1)
				{
					NRF24L01_TX_Mode();
					my_data = construct_msg(NODE,ALL_NODES,RECIEVER_TO_RECIEVER,rbs.realtime,msg->sync_time,msg->tag_id); // define send struct
					//Delay_ms(1);
					memcpy(Tx_Buffer,&my_data,sizeof(my_data));
					NRF24L01_TxPacket(Tx_Buffer);//send message
					NRF24L01_RX_Mode();//back to receive mode
				}
				break;	
			case RECIEVER_TO_RECIEVER:	// case when nodes inform each other
				if(msg->sender != NODE)   		// ONly run RBS for others nodes  
				{
					switch (msg->sync_time)
					{
						case 1:
							firstx[msg->sender]=rbs.realtime;
							firsty[msg->sender]=msg->timestamp-rbs.realtime;
						//	printf("firstx is %f,firsty is %f from %d\r\n",firstx[msg->sender],firsty[msg->sender],msg->sender);
							break;
						case 2:
							secondx[msg->sender]=rbs.realtime;
							secondy[msg->sender]=msg->timestamp-rbs.realtime;
							if(firsty[msg->sender]==secondy[msg->sender]||firstx[msg->sender]==secondx[msg->sender])
							{
								printf("2 ERROR!\r\n");
								break;
							}	
							slope[msg->sender]=(secondy[msg->sender]-firsty[msg->sender])/(secondx[msg->sender]-firstx[msg->sender]);
							y_intercept[msg->sender]=secondy[msg->sender]-slope[msg->sender]*secondx[msg->sender];
							if(NODE==ALL_NODES&&msg->sender==NODE-1)
							{
								result1=sync_store[msg->sender-1]-(sync_store[NODE]*slope[msg->sender-1]+y_intercept[msg->sender-1]+sync_store[NODE]);
								result1=result1*C/tsfreq;
								result2=sync_store[msg->sender]-(sync_store[NODE]*slope[msg->sender]+y_intercept[msg->sender]+sync_store[NODE]);
								result2=result2*C/tsfreq;
								if(result1>100||result1<-100||result2>100||result2<-100)
									break;
								printf("[3:%d:0:%d][%d:%d:%f:%d][%d:%d:%f:%d]\r\n",msg->tag_id,wearH,msg->sender,msg->tag_id,result2,wearH,
								msg->sender-1,msg->tag_id,result1,wearH);
							}
//							if(NODE==ALL_NODES)
//							{
//								result1=sync_store[msg->sender]-(sync_store[NODE]*slope[msg->sender]+y_intercept[msg->sender]+sync_store[NODE]);
//								result1=result1*C/tsfreq;
//								printf("[3:%d:0:0][%d:%d:%f:0]\r\n",msg->tag_id,msg->sender,msg->tag_id,result1);
//							}
							break;
						case 3:
							if(firsty[msg->sender]==secondy[msg->sender]||firstx[msg->sender]==secondx[msg->sender])
							{
								printf("3 ERROR!\r\n");
								break;
							}	
							result1=rbs.realtime*slope[msg->sender]+y_intercept[msg->sender]+rbs.realtime;
						//	printf("receive timestamp is %f,,this time is %f,calculate is %f\r\n",msg->timestamp,rbs.realtime,result);
						//	if(msg->timestamp-result>200||msg->timestamp-result<-200)
					//		printf("firstx %f secondx %f firsty %f secondy %f\r\n",firstx[msg->sender],secondx[msg->sender],firsty[msg->sender],secondy[msg->sender]);
					//	printf("warning: received is:%f,calculated is:%f\r\n",msg->timestamp,result);
						  printf("different between node %d and node %d is:%f\r\n",msg->sender,NODE,msg->timestamp-result1);
					//		printf("slope is:%2.10f,y_intercept is:%f\r\n",slope[msg->sender],y_intercept[msg->sender]);
						//	printf("--------------------------------------------------\r\n");
							break;
						default:
							break;
					}						
				}
				if(msg->sender == (NODE-1)&&(NODE!=ALL_NODES))
				{
					NRF24L01_TX_Mode();
					my_data = construct_msg(NODE,ALL_NODES,RECIEVER_TO_RECIEVER,rbs.realtime,msg->sync_time,msg->tag_id); // define send struct
					//Delay_ms(1);
					memcpy(Tx_Buffer,&my_data,sizeof(my_data));
					NRF24L01_TxPacket(Tx_Buffer);//send message
					NRF24L01_RX_Mode();//back to receive mode
				}
				break;
			case SYNC:
				if(msg->sender == (NODE-1)&&(NODE!=ALL_NODES))
				{
					NRF24L01_TX_Mode();
					my_data = construct_msg(NODE,ALL_NODES,SYNC,rbs.realtime,1,msg->tag_id); // define send struct
					memcpy(Tx_Buffer,&my_data,sizeof(my_data));
					NRF24L01_TxPacket(Tx_Buffer);//send message
					NRF24L01_RX_Mode();
				}else if(NODE==ALL_NODES){
					if(msg->sync_time==0)
					{
						sync_store[NODE]=rbs.realtime;
						break;
					}
					sync_store[msg->sender]=msg->timestamp;
				}
				break;
			default:
				break;
			}
	return 0;
}

exlcm_sync_t construct_msg(int8_t sender, int8_t receiver, 
				int8_t operation, double timestamp, int8_t sync_time, int8_t tag_id)
/*****************************************************************************
*   Input    :
*   Output   :
*   Function :
******************************************************************************/ 
{
	exlcm_sync_t my_data; 
	
		my_data.sender = sender;
		my_data.receiver = receiver;
		my_data.operation = operation;
		my_data.timestamp = timestamp;
		my_data.sync_time = sync_time;
		my_data.tag_id = tag_id;
	return my_data;
}


void init_variables(void)
/*****************************************************************************
*   Input    :
*   Output   :
*   Function :
******************************************************************************/ 
{
	int i;
	for(i=0; i < ALL_NODES;i++)		// resetting datafields
	{
		rbs.nodes[i].n = 0;
		rbs.nodes[i].reach_max = 0;
//		rbs.nodes[i].end->next = NULL;
	}
}	

void set_RBS_realtime(double realtime)
{
	rbs.realtime= realtime;
}

void set_wear_flag(unsigned char wear)
{
	wearH=wear;
}

double format5to8(unsigned char * timestamp)
{
	unsigned long long timestamp8 = 0;
	int j;
	for (j = 5 ; j >= 0 ; j --)
    {
        timestamp8 = (timestamp8 << 8) + timestamp[j] ;
    }
	return (double)timestamp8;
}

double KalmanFilter_slope(double ResrcData, double ProcessNiose_Q,double MeasureNoise_R, double InitialPrediction ,int8_t operation)
{
        double R = MeasureNoise_R;
        double Q = ProcessNiose_Q;

        static double x_last;

        double x_mid = x_last;
        double x_now;

        static double p_last;

        double p_mid ;
        double p_now;
        double kg; 
				if(operation==CLEARALL)
				{
					x_last=0;
					p_last=0;
					return 0;
				}else if(operation==PRENUM)
				{
					return x_last;
				}
				if(x_last==0)
					x_last=InitialPrediction;
        x_mid=x_last; 
        p_mid=p_last+Q; 
        kg=p_mid/(p_mid+R); 
        x_now=x_mid+kg*(ResrcData-x_mid);
               
        p_now=(1-kg)*p_mid;     

        p_last = p_now; 
        x_last = x_now; 

        return x_now;               
}

double KalmanFilter_y(double ResrcData, double ProcessNiose_Q,double MeasureNoise_R , double InitialPrediction ,int8_t operation)
{
        double R = MeasureNoise_R;
        double Q = ProcessNiose_Q;

        static double x_last;

        double x_mid = x_last;
        double x_now;

        static double p_last;

        double p_mid ;
        double p_now;
        double kg;
				if(operation==CLEARALL)
				{
					x_last=0;
					p_last=0;
					return 0;
				}else if(operation==PRENUM)
				{
					return x_last;
				}	
				if(x_last==0)
					x_last=InitialPrediction;
        x_mid=x_last; 
        p_mid=p_last+Q; 
        kg=p_mid/(p_mid+R); 
        x_now=x_mid+kg*(ResrcData-x_mid);
               
        p_now=(1-kg)*p_mid;     

        p_last = p_now; 
        x_last = x_now; 

        return x_now;               
}
