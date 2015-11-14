/* LICENSE
	cell_interface.h and cell_interface.c are derivatives of source from Linear
	Technology Corp.(LTC)
*/

/************************************



***********************************************************/



#ifndef CELL_INTERFACE_H
#define CELL_INTERFACE_H

    #include <stdint.h>
    #include <project.h>
    #include "can_manager.h"
   
    #define ERROR_VOLTAGE_LIMIT (3u)
    #define ERROR_TEMPERATURE_LIMIT (3u)
    #define FUSE_BAD_LIMIT (3u)
    #define BAD_FILTER_LIMIT (5u)
    
    #define CELL_ENABLE (0x1cf)
    #define OVER_VOLTAGE (4200u)
    #define UNDER_VOLTAGE (2000u)
    #define STACK_VOLT_DIFF_LIMIT (30000u)
    #define CRITICAL_TEMP_L (2000u)          //0.2V
    #define CRITICAL_TEMP_H (10213u)             //1.0213V  10213
    #define BAD_THERM_LIMIT (8u)
    #define SOC_CALI_HIGH (110000u)     //High cali point at 110V
    #define SOC_SOC_HIGH  (60000u)      //manually set it in mAh
    #define SOC_CALI_LOW (80000u)     //Low Cali point at 80V
    #define SOC_SOC_LOW   (10000u)      //manually set it in mAh
    #define SOC_FULL_CAP (75000u)     //let's say, 75,000mAh
    #define SOC_FULL      (115000u)   //when voltage reaches 115V, consider it full
    
    #define N_OF_CELL (84u)
    #define N_OF_TEMP (60u)
    #define N_OF_NODE (6u)
    #define N_OF_STACK (3u)

    //#define DEBUG_LCD 0

    #define OVER_TEMP (90u)             //now it just for debug purpose
    #define UNDER_TEMP (0u)

    #define THERM_CELL (0u)
    #define THERM_BOARD (0u)


    // bms_status
    #define NO_ERROR 0x0000
    #define CHARGEMODE 0x0001
    #define PACK_TEMP_OVER 0x0002
    #define STACK_FUSE_BROKEN 0x0004
    #define PACK_TEMP_UNDER 0x0008
    #define LOW_SOC   0x0010
    #define CRITICAL_SOC   0x0020
    #define IMBALANCE   0x0040
    #define COM_FAILURE   0x0080
    #define NEG_CONT_CLOSED   0x0100
    #define POS_CONT_CLOSED   0x0200
    #define ISO_FAULT   0x0400
    #define CELL_VOLT_OVER   0x0800
    #define CELL_VOLT_UNDER   0x1000
    #define CHARGE_HAULT   0x2000
    #define FULL   0x4000
    #define PRECHARGE_CLOSED   0x8000

  

//new data stucture

typedef enum {
  NORMAL =0,
  WARNING =1,
  FAULT =2,
}BAT_HEALTH;


typedef struct
{
  uint16_t err;
  uint8_t bad_cell;
  uint8_t bad_node;

}BAT_ERR_t;

typedef struct 
{
  uint16_t voltage;
  uint8_t bad_counter;
  uint8_t bad_type;
}BAT_CELL_t;

typedef struct
{
  uint16_t temp_raw;
  uint8_t temp_c;
  uint8_t bad_counter;
  uint8_t type;
  uint8_t bad_type;
}BAT_TEMP_t;

typedef struct
{
  BAT_CELL_t *cells[14];
  BAT_TEMP_t *temps[10];
  uint16_t over_temp;
  uint16_t under_temp;
  uint16_t over_voltage;
  uint16_t under_voltage;
}BAT_NODE_t;

typedef struct 
{
  BAT_NODE_t *nodes[2];
  uint32_t voltage;
  uint8_t bad_counter;
}BAT_STACK_t;

typedef struct
{
  BAT_STACK_t *stacks[3];
  BAT_NODE_t *nodes[6];
  uint32_t voltage;
  uint16_t current;
  uint8_t fuse_fault;
  uint16_t status;
  BAT_HEALTH health;
  uint32_t current_charge;
  uint8_t SOC_cali_flag;
  uint8_t HI_temp_c;
  uint8_t HI_temp_raw;
}BAT_PACK_t;

typedef struct 
{
  uint8_t percent_SOC;
  uint32_t absolute_SOC;
}BAT_SOC_t;



/**
 * @initialize. In case need to setup anything, write them in it.
 *
 * @param no input parameters.
 * @return 1 if everything is OK. 0 for hard failure.
 */
void  bms_init();




/**
 * @check config register
 *
 * @param no input parameters.
 * @return 1 if everything is OK. 0 for hard failure.
 */
void check_cfg();


/**
 * @wake all BMSs up, waiting for commands
 *
 * @param no input parameters.
 * @return 1 if everything is OK. 0 for hard failure.
 */
void  wake_up();

/**
 * @check if chips are exist without and error
 *
 * @param no input parameters.
 * @return 1 if everything is OK. 0 for hard failure.
 */
void check_chips();


/**
 * @check if cells are existed 
 *
 * @param no input parameters.
 * @return 1 if everything is OK. 0 for hard failure.
 */
uint8_t check_cells();

/**
 * @check every cells if voltages are in safe range
 *
 * @param no input parameters.
 * @return 1 if everything is OK. 0 for hard failure.
 */
uint8_t get_cell_volt();

/**
 * @check every cells if temperature are in safe range
 *
 * @param no input parameters.
 * @return 1 if everything is OK. 0 for hard failure.
 */
uint8_t get_cell_temp();

/**
 * @balance each cell
 *
 * @param no input parameters.
 * @return 1 if everything is OK. 0 for hard failure.
 */
//void balance_cells();

/**
 * @update voltage and detect error
 *
 * @param 1 input parameters, which is raw cell_codes.
 * @return NULL.
 */
void update_volt(uint16_t cell_codes[TOTAL_IC][12]);

/**
 * @check voltage and detect error
 *
 * @param no input
 * @return NULL.
 */
void check_volt();

/**
 * @update temperature and detect error
 *
 * @param 1 input parameters, which is raw aux_codes.
 * @return NULL.
 */
void update_temp(uint16_t aux_codes[TOTAL_IC][6]);

/**
 * @check temperature and detect error
 *
 * @param no input param
 * @return NULL.
 */
void check_temp();


/**
 * @initial mypack
 *
 * @param no input parameters.
 * @return NULL.
 */
void mypack_init();

/**
 * @cell balancing
 *
 * @param no input parameters.
 * @return NULL.
 */
void balance_cell();

/**
 * @check is fuse is broken
 *
 * @param no input parameters. (use global mypack)
 * @return NULL.
 */
void check_stack_fuse();

/**
 * @check is fuse is broken
 *
 * @param no input parameters. (use global mypack)
 * @return NULL.
 */
void bat_err_add(uint16_t, uint8_t, uint8_t);

uint8_t temp_transfer(uint16_t);

void voltage_compensation();

/**
 * @once other function requested the soc, it returned
 *
 * @param no input parameters. (use global mypack)
 * @return NULL.
 */
BAT_SOC_t get_soc();

/**
 * @update soc in short time period
 *
 * @param no input parameters. (use global mypack)
 * @return NULL.
 */
void update_soc();

#endif // CELL_INTERFACE_H
