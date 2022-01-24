#ifndef __FSM_H
#define __FSM_H

#include <rtthread.h>

#define FSM_MAX_EVT (8)
#define FSM_MAX_TIMER_EVT  (4)

typedef enum
{
    STATE_RET_HANDLE,
    STATE_RET_IGNORE,
    STATE_RET_TRAN,
}R_state_t;


#define S_HANDLE() (STATE_RET_HANDLE)
#define S_IGNORE() (STATE_RET_IGNORE)
#define S_TRAN(tar)      ( ((fsm_t*)me)->cur_state = (state_handle_t)tar ,  STATE_RET_TRAN)

enum
{
    ENTER_EVT =1,
    EXIT_EVT,
    TIMEOUT_EVT,
    USER_EVT,
};

typedef struct fsm fsm_t;
typedef  R_state_t (*state_handle_t)(fsm_t* me, uint8_t evt); 

struct fsm 
{
    struct fsm*  next;
	state_handle_t cur_state;
	state_handle_t last_state;
	
	void (*timeout_callback)(void *parameter);
	
    /*evt fifo*/
	struct fifo
	{
		uint8_t buf[FSM_MAX_EVT];
		uint8_t head;
		uint8_t tail;
		uint8_t num;
	}evt_fifo;
	
	/*timer evt*/
	struct 
	{
		rt_timer_t timer;
		uint8_t evt;
		uint8_t status;
	}timer_evt[FSM_MAX_TIMER_EVT];
};


void fsm_init(fsm_t* me , state_handle_t init);
void fsm_polling(void);

void fsm_set_timeout_evt(fsm_t* me , uint32_t timeout,uint8_t evt);
void fsm_stop_timeout_evt(fsm_t* me,uint8_t evt);
int fsm_evt_post(fsm_t* me ,uint8_t evt);


#endif // !__FSM_H


