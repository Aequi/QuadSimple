#ifndef __HAL_PWM_H__
#define __HAL_PWM_H__

#include <stdint.h>

typedef enum {
    PWM_CHANNEL_0,
    PWM_CHANNEL_1,
    PWM_CHANNEL_2,
    PWM_CHANNEL_3,
} PwmChannel;

void halPwmInit(void);
void halPwmSetChannelValue(PwmChannel pwmChannel, uint32_t value);


#endif /* __HAL_PWM_H__ */
