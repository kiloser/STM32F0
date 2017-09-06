
#ifndef _node_sync_h
#define _node_sync_h

#include "stdint.h"
/***************************** Include files *******************************/

/*****************************    Defines    *******************************/



// statemachine for RBS
#define BEACONSENDER 0
#define	RECIEVER_TO_RECIEVER 1
#define SYNC	2

#define IDLE	0


// sender and receiver
#define ALL_NODES 3		// How many nodes in the network
#define BEACON_NODE 0		// beacon node

#define TRIALS 50			// How many packets stored

//Inserting and getting
#define REMOTETIME 	1
#define LOCALTIME 	2
#define OFFSET 		3
#define REACHMAX	4
#define DATASIZE	5
#define SCALETIME   6
#define OFFSETMEAN	7
#define LSLR		8
#define NEWTIMELSLR	9
#define NEWTIMEMEAN	10

#define ZERO 0
#define ONE 1

#define CLEARALL 1
#define PRENUM 2 


typedef struct _exlcm_sync_t exlcm_sync_t;
struct _exlcm_sync_t
{
    int8_t     sender;
    int8_t     receiver;
    int8_t     operation;
    double     timestamp;
		int8_t     sync_time;
		int8_t     tag_id;
};


struct NODE_INFO	// store data for each TRIAL
{
	struct NODE_INFO *next;		// next element in the list
	double remote_time;			// time for the remote node
	double local_time;			// time for the current node
	double offset;				// a node offset to the current node
	double scale_time;			// time for the current noce scaled
	double offset_mean;			// result from mean offset
	double lslr;				// result from LSLR
};
struct NODES		// store data for each NODE
{
		int number_of_node;		// number of the node
		int n;				// current sample
		int reach_max;		// When TRIALS is reached
		struct NODE_INFO info[TRIALS];	// stored data for each TRIALS
		struct NODE_INFO *end;			// GET the oldest TRIAL	
		struct NODE_INFO *front;			// GET the newest TRIAL	
};

struct RBS_INFO
{
	double realtime;		// time for current node
	double realtime_0;		// first get time for current node used by scale
	struct NODE_INFO *end;	// get oldest TRIAL
	struct NODES nodes[ALL_NODES];	// all nodes
};


/*****************************   Functions   *******************************/


exlcm_sync_t construct_msg(int8_t, int8_t, int8_t, double, int8_t, int8_t);
/*****************************************************************************
*   Input    : -
*   Output   : -
*   Function : Test function
******************************************************************************/



int RBS(const exlcm_sync_t *);
/*****************************************************************************
*   Input    : -
*   Output   : -
*   Function : Test function
******************************************************************************/

void init_variables(void);

void set_RBS_realtime(double realtime);
void set_wear_flag(unsigned char wear);
double format5to8(unsigned char * timestamp);
double KalmanFilter_slope(double ResrcData, double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction,int8_t operation);
double KalmanFilter_y(double ResrcData, double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction,int8_t operation);

/****************************** End Of Module *******************************/
#endif
