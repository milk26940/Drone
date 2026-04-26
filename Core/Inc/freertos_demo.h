#ifndef __FREERTOS_DEMO_H
#define __FREERTOS_DEMO_H

#include <stdint.h>
#include "../../common/Com_config.h"

extern volatile uint8_t rc_connected;
extern volatile Flight_State flight_state;

void freertos_start(void);
void LED_Task(void *pvParameters);
void set_rc_connected(uint8_t connected);

#endif
