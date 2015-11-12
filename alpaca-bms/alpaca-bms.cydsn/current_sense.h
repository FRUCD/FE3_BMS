#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <project.h>
#include <stdint.h>

void current_init(void);
void set_current_interval(uint16_t interval);
int16_t get_current(void);



#endif
