#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include <project.h>
#include "data.h"
#include "LTC68041.h"

typedef enum{
    NO_ERROR=0x0000,
    CHARGEMODE=0x0001,
    PACK_TEMP_OVER=0x0002,
    STACK_FUSE_BROKEN=0x0004,
    PACK_TEMP_UNDER=0x0008,
    LOW_SOC = 0x0010,
    CRITICAL_SOC = 0x0020,
    IMBALANCE = 0x0040,
    COM_FAILURE = 0x0080,
    NEG_CONT_CLOSED = 0x0100,
    POS_CONT_CLOSED = 0x0200,
    ISO_FAULT = 0x0400,
    CELL_VOLT_OVER = 0x0800,
    CELL_VOLT_UNDER = 0x1000,
    CHARGE_HAULT = 0x2000,
    FULL = 0x4000,
    PRECHARGE_CLOSED = 0x8000
}BMS_STATUS;

    
    
    
void can_send_temp(uint8_t temp_index,
    uint8_t temp_node,
    uint16_t temp_c,
    uint32_t temp_raw,
    uint8_t HI_temp_c);

void can_send_volt(uint8_t cell_index,
    uint8_t cell_node,
    uint16_t cell_voltage,
    uint32_t pack_voltage);

void can_send_current();

void can_send_status(uint8_t SOC_P,
                    uint8_t AH,
                    BMS_STATUS status,
                    uint8_t stack,
                    uint8_t cell,
                    uint16_t value16);
uint8_t can_test_send();
void can_init();

#endif // CAN_MANAGER_H
