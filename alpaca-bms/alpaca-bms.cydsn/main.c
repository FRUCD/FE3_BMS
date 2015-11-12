#include <project.h>
#include <stdint.h>

#include "cell_interface.h"
#include "current_sense.h"
#include "WDT.h"
#include "data.h"
#include "can_manager.h"
#include "uart-terminal.h"
#include "BMS_monitor.h"

typedef enum 
{
	BMS_BOOTUP,
	BMS_NORMAL,
	BMS_CHARGEMODE,
	BMS_RACINGMODE,
	BMS_DEBUGMODE,
	BMS_SLEEPMODE,
	BMS_FAULT
}BMS_MODE;


volatile uint8_t CAN_UPDATE_FLAG=0;
extern volatile BAT_PACK_t bat_pack;
extern volatile BAT_ERR_t* bat_err_array;
extern volatile uint8_t bat_err_index;
extern volatile uint8_t bat_err_index_loop;
volatile uint8_t CAN_DEBUG=0;
volatile uint8_t RACING_FLAG=0;    // this flag should be set in CAN handler
BAT_SOC_t bat_soc;




CY_ISR(CAN_UPDATE_Handler){
    Can_Update_Timer_STATUS;
    CAN_UPDATE_FLAG = 1;
}

CY_ISR(current_update_Handler){
    current_Timer_STATUS;
	update_soc();
	return;
}


void process_event(){
    // heartbeat
    can_send_status(bat_soc.percent_SOC,
    	0,
    	bat_pack.status,
    	0,0,0);
}

void process_failure_helper(BAT_ERR_t err){
	switch(err.err){
		case CELL_VOLT_OVER:
		case CELL_VOLT_UNDER:
			can_send_volt(err.bad_cell,
				err.bad_node,
				bat_pack.nodes[err.bad_node]->cells[err.bad_cell]->voltage,
				bat_pack.voltage);
			break;
		
		case PACK_TEMP_OVER:
		case PACK_TEMP_UNDER:
			// waiting for CAN mesg been defined clearly
			break;

	}
	return;
}

void process_failure(){
	uint8_t i=0;
	// broadcast error in inverse chrognxxxical order
	if (bat_err_index_loop){
		// start from bat_err_index back to 0
		for (i=bat_err_index;i>=0;i--){
			process_failure_helper(bat_err_array[i]);
		}
		// start from index=99 to bat_err_index+1
		for (i=99;i>bat_err_index;i--){
			process_failure_helper(bat_err_array[i]);
		}
	}else{
		// start from bat_err_index back to 0
		for (i=bat_err_index;i>=0;i--){
			process_failure_helper(bat_err_array[i]);
		}
	}
}

int main(void)
{
	// Initialize state machine
	BMS_MODE bms_status = BMS_BOOTUP;
	uint32_t system_interval = 100;

	while(1){
		switch (bms_status){
			case BMS_BOOTUP:
				Can_Update_ISR_StartEx(CAN_UPDATE_Handler);
                current_update_ISR_StartEx(current_update_Handler);
				Can_Update_Timer_Start();
                current_Timer_Start();
				can_init();
				
				// TODO Watchdog Timer
			    CyWdtStart(CYWDT_1024_TICKS,CYWDT_LPMODE_NOCHANGE);

				// Initialize
				bms_init();
				mypack_init();
				current_init();
			    //monitor_init();
			    
			    //enable global interrupt
			    CyGlobalIntEnable;
		    
			    //some variables and states
			    OK_SIG_Write(1);
		       //terminal_run();
				break;

			case BMS_NORMAL:
			    OK_SIG_Write(1);
			    //check_cfg();  //CANNOT be finished, because 
				//check_cells();// TODO This function will be finished in get_cell_volt/check stack fuse
		        get_cell_volt();// TODO Get voltage
				check_stack_fuse(); // TODO: check if stacks are disconnected
				get_cell_temp();// TODO Get temperature
				get_current(); // TODO get current reading from sensor
				bat_soc = get_soc(); // TODO calculate SOC()
				// because it is normal mode, just set a median length current reading interval
				set_current_interval(100);
				system_interval = 1000;

				if (bat_pack.health == FAULT){
					bms_status = BMS_FAULT;
				}
				if (bat_pack.status || CHARGEMODE){
					bms_status = BMS_CHARGEMODE;
				}else if(RACING_FLAG){
					bms_status = BMS_RACINGMODE;
				}
				break;

			case BMS_CHARGEMODE:
				OK_SIG_Write(1);

				//check_cfg();  //CANNOT be finished, because 
				//check_cells();// TODO This function will be finished in get_cell_volt/check stack fuse
		        get_cell_volt();// TODO Get voltage
				check_stack_fuse(); // TODO: check if stacks are disconnected
				get_cell_temp();// TODO Get temperature
				get_current(); // TODO get current reading from sensor
				bat_soc = get_soc(); // TODO calculate SOC()
				// because it is normal mode, just set a median length current reading interval
				set_current_interval(100);
				system_interval = 5000;



				if (!(bat_pack.status || CHARGEMODE)){
					bms_status = CHARGEMODE;
				}
				break;

			case BMS_RACINGMODE:
				OK_SIG_Write(1);

				//check_cfg();  //CANNOT be finished, because 
				//check_cells();// TODO This function will be finished in get_cell_volt/check stack fuse
		        get_cell_volt();// TODO Get voltage
				check_stack_fuse(); // TODO: check if stacks are disconnected
				get_cell_temp();// TODO Get temperature
				get_current(); // TODO get current reading from sensor
				bat_soc = get_soc(); // TODO calculate SOC()

				system_interval = 200;
				set_current_interval(1);

				if (bat_pack.health == FAULT){
					bms_status = BMS_FAULT;
				}
				if (!RACING_FLAG){
					bms_status = BMS_NORMAL;
				}
				break;

			case BMS_SLEEPMODE:
				OK_SIG_Write(1);
				break;

			case BMS_FAULT:
				OK_SIG_Write(0u);
				// send

				bms_status = BMS_FAULT;
				system_interval = 100;
				process_failure();
			default:
				bms_status = BMS_FAULT;

		}
		CyWdtClear();
		process_event();
		CyDelay(system_interval);
	}
} // main()
