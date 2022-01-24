
//#include "fsm.h"


//#define FSM_ASSERT(me)    do{if(me==NULL){ while(1);}}while(0)
//#define hw_interrupt_disable  BoardEnableIrq 
//#define hw_interrupt_enable   BoardDisableIrq

//static const char *NAME = "FSM";
//static fsm_t active_fsm;
//static void fsm_timer_callback(void* param);

///*---------------------------------------------------------------!
// * \brief Initializes fsm and set the init state
// *
// * \param [IN] fsm itself
// * \param [IN] the initial state of fsm
// * \param [out] NULL
// ----------------------------------------------------------------*/
//void fsm_init(fsm_t* me , state_handle_t init)
//{
//    FSM_ASSERT(me);
//    FSM_ASSERT(init);
//	
//    me->cur_state = init;
//    me->last_state = NULL;

//    me->evt_fifo.head = 0;
//	me->evt_fifo.tail = 0;
//	me->evt_fifo.num = 0;
//	rt_memset(me->evt_fifo.buf , 0 , FSM_MAX_EVT);
//	
//	me->timeout_callback = fsm_timer_callback;
//	
//	/*timer evt*/
//	for(uint8_t i=0;i<FSM_MAX_TIMER_EVT;i++)
//	{
//		rt_timer_init(&me->timer_evt[i].timer ,"fsm",, RT_NULL);
//		me->timer_evt[i].status = 0;
//		me->timer_evt[i].evt = 0;
//	}

//	/*add new fsm*/
//    fsm_t* tail_fsm = &active_fsm;
//    while(tail_fsm->next != NULL)
//    {
//        tail_fsm = tail_fsm->next;
//    }
//    tail_fsm->next = me;
//    me->next =NULL;
//	
//	/*enter init state*/
//    me->cur_state(me,ENTER_EVT);
//}
///*---------------------------------------------------------------!
// * \brief post evt to fsm evt fifo, fsm get the evt by polling
// *
// * \param [IN] fsm itself
// * \param [IN] the  evt need to be sent 
// * \param [out] result of post
// ----------------------------------------------------------------*/
//int fsm_evt_post(fsm_t* me ,uint8_t evt)
//{
//    FSM_ASSERT(me);

//    if(me->evt_fifo.num < FSM_MAX_EVT)
//    {
//		hw_interrupt_disable();
//        me->evt_fifo.buf[me->evt_fifo.tail] = evt;
//        me->evt_fifo.tail = (me->evt_fifo.tail + 1) % FSM_MAX_EVT;
//        me->evt_fifo.num ++;
//		hw_interrupt_enable();
//        return 0;
//    }
//    return -1;
//}
///*---------------------------------------------------------------!
// * \brief get evt from fsm evt fifo
// *
// * \param [IN] fsm itself
// * \param [IN] the  pointer of evt
// * \param [out] Whether the event was successfully obtained
// ----------------------------------------------------------------*/
//static int fsm_evt_get(fsm_t* me,uint8_t* evt)
//{
//    FSM_ASSERT(me);
//    if(me->evt_fifo.num > 0)
//    {
//	  	hw_interrupt_disable();
//        *evt =  me->evt_fifo.buf[me->evt_fifo.head];
//        me->evt_fifo.head = (me->evt_fifo.head+ 1) % FSM_MAX_EVT;
//        me->evt_fifo.num --;
//		hw_interrupt_enable();
//        return 0;
//    }
//    return -1;
//}
///*---------------------------------------------------------------!
// * \brief set the irq evt ,The event will be handled in the timer 
// * interrupt callback.This means that you can get a precise timing 
// *	event , You can't spend too much time on event processing because 
// *	event processing takes place in an interrupt.Each fsm only allows 
// *	1 irq events to occur simultaneously.
// * \param [IN] fsm itself
// * \param [IN] The exact delay time for the event to arrive
// * \param [IN] The evt
// * \param [out] NULL
// ----------------------------------------------------------------*/
//void fsm_set_irq_evt(fsm_t* me , uint32_t timeout , uint8_t evt)
//{
//  	FSM_ASSERT(me);
// 	hw_interrupt_disable();
//  	me->irq_evt.status = true;
//	me->irq_evt.evt = evt;
//	TimerSetValue(&me->irq_evt.timer, timeout);
//	TimerStart(&me->irq_evt.timer);
//	hw_interrupt_enable();
//}

//void fsm_stop_irq_evt(fsm_t* me)
//{
//	FSM_ASSERT(me);
//	hw_interrupt_disable();
//	if(me->irq_evt.status == true)
//	{
//		me->irq_evt.status = false;
//		me->irq_evt.evt = 0;
//		TimerStop(&me->irq_evt.timer);
//	}
//	hw_interrupt_enable();
//}

///*---------------------------------------------------------------!
// * \brief set the timer evt ,the timer evt is also obtained by 
// *  polling,As a result, the time is biased, affected by the system 
//	polling cycle, and does not respond as timely as interrupt events.
//	Each fsm allows 4 timed events to occur simultaneously
// * \param [IN] fsm itself
// * \param [IN] The exact delay time for the event to arrive
// * \param [IN] The evt
// * \param [out] NULL
// ----------------------------------------------------------------*/
//void fsm_set_timeout_evt(fsm_t* me , uint32_t timeout , uint8_t evt)
//{	
//	for(uint8_t i=0;i<FSM_MAX_TIMER_EVT;i++)
//	{
//		/*find the first free evt timer*/
//		if(me->timer_evt[i].status == false)
//		{	
//			hw_interrupt_disable();
//			me->timer_evt[i].status = true;
//			me->timer_evt[i].evt = evt;
//			TimerSetValue(&me->timer_evt[i].timer, timeout);
//			TimerStart(&me->timer_evt[i].timer);
//			hw_interrupt_enable();
//			return;
//		}
//	}
//	LOG(DEBUG,"no free evt timer!");
//}
///*---------------------------------------------------------------!
// * \brief When a timed event occurs in a state, it may be necessary
//	to stop the timed event before exiting the state due to some 
//	unexpected event occurring that causes the system to switch 
//	the state
// * \param [IN] fsm itself
// * \param [IN] Timing events that need to be cleaned up
// * \param [out] NULL
// ----------------------------------------------------------------*/
//void fsm_stop_timeout_evt(fsm_t* me , uint8_t evt)
//{
//  	for(uint8_t i=0;i<FSM_MAX_TIMER_EVT;i++)
//	{
//	  	/*find the same evt timer*/
//	  	if(evt == me->timer_evt[i].evt)
//		{
//		    hw_interrupt_disable();
//		    me->timer_evt[i].status = false;
//			me->timer_evt[i].evt = 0;
//			TimerStop(&me->timer_evt[i].timer);
//			hw_interrupt_enable();
//		}
//	}
//}

//void fsm_clean_timeout_evt(fsm_t* me)
//{
//	for(uint8_t i=0;i<FSM_MAX_TIMER_EVT;i++)
//	{
//		if(me->timer_evt[i].status == true)
//		{
//			hw_interrupt_disable();
//		    me->timer_evt[i].status = false;
//			me->timer_evt[i].evt = 0;
//			TimerStop(&me->timer_evt[i].timer);
//			hw_interrupt_enable();
//		}
//	}
//}
///*---------------------------------------------------------------!
// * \brief fsm evt dispatch
// *
// * \param [IN] fsm itself
// * \param [IN] the evt get from evt-fifo or irq evt or timer evt
// * \param [out] NULL
// ----------------------------------------------------------------*/

//static void fsm_evt_dispatch(fsm_t* me,uint8_t evt)
//{
//	state_handle_t last = me->cur_state;
//		
//	R_state_t ret =  me->cur_state(me , evt);
//	if(ret == STATE_RET_TRAN)
//	{
//		me->last_state = last;
//		me->last_state(me,EXIT_EVT);
//		fsm_clean_timeout_evt(me);
//		me->cur_state (me,ENTER_EVT);
//	}
//}
///*---------------------------------------------------------------!
// * \brief This function polls event FIFO and timed events for 
//	all FSM and dispatches them
// *
// * \param [IN] NULL
// * \param [IN] NULL
// * \param [out] NULL
// ----------------------------------------------------------------*/
//void fsm_polling(void)
//{
//    fsm_t* cur_fsm = &active_fsm;
//	uint8_t evt;

//    while (cur_fsm->next != NULL)
//    {
//        cur_fsm = cur_fsm->next;
//		
//		/*get all evt and dispatch*/
//		while(fsm_evt_get(cur_fsm,&evt) == 0)
//    	{
//        	fsm_evt_dispatch(cur_fsm,evt);
//    	}
//		/*get all timeout evt and dispatch*/
//		for(uint8_t i=0;i<FSM_MAX_TIMER_EVT;i++)
//		{
//			if(cur_fsm->timer_evt[i].status == true)
//			{
//				if(TimerGetFlag(&cur_fsm->timer_evt[i].timer))
//				{
//					cur_fsm->timer_evt[i].status = false;
//					evt = cur_fsm->timer_evt[i].evt;
//					cur_fsm->timer_evt[i].evt = 0;
//					
//					fsm_evt_dispatch(cur_fsm ,  evt);
//				}
//			}
//		}
//    }
//}
///*---------------------------------------------------------------!
// * \brief Interrupt event callback, all state machines share the 
//    same callback event, each state machine only exists one available 
//	interrupt event, only one interrupt event will occur at the same 
//    time, therefore, the user's interrupt event processing must be as 
//    short as possible
//	TODO£º **issue---> If a state switch occurs in an interrupt event, 
//		   it causes an existing polling event to be processed in the 
//           next state  (insure no state switch occurs in interrupt event)
// * \param [IN] NULL
// * \param [IN] NULL
// * \param [out] NULL
// ----------------------------------------------------------------*/
//static void fsm_timer_callback(void* param)
//{
//  	uint8_t evt;

//	fsm_t* cur_fsm = &active_fsm;
//	while (cur_fsm->next != NULL)
//    {
//        cur_fsm = cur_fsm->next;
//		if(cur_fsm->irq_evt.status == true)
//		{
//			if(TimerGetFlag(&cur_fsm->irq_evt.timer))
//			{
//			  	/*weather need to dispatch the fifo evt*/
//				cur_fsm->irq_evt.status = false;
//				evt = cur_fsm->irq_evt.evt;
//				cur_fsm->irq_evt.evt = 0;
//				
//				fsm_evt_dispatch(cur_fsm ,  evt);
//				return;
//			}
//		}
//	}

//}

///*------------------------end--------------------------------*/