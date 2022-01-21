/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-08-23     balanceTWK   first version
 */

#ifndef __PULSE_ENCODER_CONFIG_H__
#define __PULSE_ENCODER_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif


#ifdef BSP_USING_PULSE_ENCODER1
  
#define BSP_ENCODER1_TIM         	TIM5
#define BSP_ENCODER1_TIM_IRQ     	TIM5_IRQn
#define BSP_ENCODER1_TIM_IRQ_HANDLE TIM5_IRQHandler


#ifndef PULSE_ENCODER1_CONFIG
#define PULSE_ENCODER1_CONFIG                          \
    {                                                  \
       .tim_handler.Instance    = BSP_ENCODER1_TIM,                \
       .encoder_irqn            = BSP_ENCODER1_TIM_IRQ,  \
       .name                    = "pulse1"             \
    }
#endif /* PULSE_ENCODER1_CONFIG */
#endif /* BSP_USING_PULSE_ENCODER1 */

#ifdef BSP_USING_PULSE_ENCODER2
	
#define BSP_ENCODER2_TIM         		TIM8
#define BSP_ENCODER2_TIM_IRQ     		TIM8_CC_IRQn	
#define BSP_ENCODER2_TIM_IRQ_HANDLE 	TIM8_CC_IRQHandler
	

#ifndef PULSE_ENCODER2_CONFIG
#define PULSE_ENCODER2_CONFIG                          \
    {                                                  \
       .tim_handler.Instance    = BSP_ENCODER2_TIM,                \
       .encoder_irqn            = BSP_ENCODER2_TIM_IRQ,           \
       .name                    = "pulse2"             \
    }
#endif /* PULSE_ENCODER2_CONFIG */
#endif /* BSP_USING_PULSE_ENCODER2 */

#ifdef BSP_USING_PULSE_ENCODER3

#define BSP_ENCODER3_TIM         TIM3
#define BSP_ENCODER3_TIM_IRQ     TIM3_IRQn		
#define BSP_ENCODER3_TIM_IRQ_HANDLE TIM3_IRQHandler
	
#ifndef PULSE_ENCODER3_CONFIG
#define PULSE_ENCODER3_CONFIG                          \
    {                                                  \
       .tim_handler.Instance    = BSP_ENCODER3_TIM,                \
       .encoder_irqn            = BSP_ENCODER3_TIM_IRQ,           \
       .name                    = "pulse3"             \
    }
#endif /* PULSE_ENCODER3_CONFIG */
#endif /* BSP_USING_PULSE_ENCODER3 */

#ifdef BSP_USING_PULSE_ENCODER4
	
#define BSP_ENCODER4_TIM         TIM4
#define BSP_ENCODER4_TIM_IRQ     TIM4_IRQn	
#define BSP_ENCODER4_TIM_IRQ_HANDLE TIM4_IRQHandler

#ifndef PULSE_ENCODER4_CONFIG
#define PULSE_ENCODER4_CONFIG                          \
    {                                                  \
       .tim_handler.Instance    = BSP_ENCODER4_TIM,                \
       .encoder_irqn            = BSP_ENCODER4_TIM_IRQ,           \
       .name                    = "pulse4"             \
    }
#endif /* PULSE_ENCODER4_CONFIG */
#endif /* BSP_USING_PULSE_ENCODER4 */

#ifdef __cplusplus
}
#endif

#endif /* __PULSE_ENCODER_CONFIG_H__ */
