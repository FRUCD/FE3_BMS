/* LICENSE
	cell_interface.h and cell_interface.c are derivatives of source from Linear
	Technology Corp.(LTC)
*/

/************************************
REVISION HISTORY
$Revision: 1000 $
$Date: 2013-07-15 

Copyright (c) 2013, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.

Copyright 2013 Linear Technology Corp. (LTC)
***********************************************************/



#include "cell_interface.h"
#include "LTC68041.h"
#include "math.h"

#include <stdlib.h>



extern volatile uint8_t CAN_UPDATE_FLAG;
extern volatile BMS_STATUS fatal_err;
extern volatile BMS_STATUS warning_err;
extern volatile uint8_t error_IC;
extern volatile uint8_t error_CHIP;
volatile uint8_t error_voltage_count=0;
volatile uint8_t error_temperature_count=0;
volatile BATTERYPACK mypack;
// FE3 new structure
volatile BAT_CELL_t bat_cell[N_OF_CELL];
volatile BAT_TEMP_t bat_temp[N_OF_TEMP];
volatile BAT_NODE_t bat_node[N_OF_NODE];
volatile BAT_STACK_t bat_stack[N_OF_STACK];
volatile BAT_ERR_t bat_err;

extern volatile uint8_t CAN_DEBUG;
volatile uint8_t bad_therm=0;


/**
 * @initialize. In case need to setup anything, write them in it.
 *
 * @param no input parameters.
 * @return 1 if everything is OK. 0 for hard failure.
 */
void  bms_init(){
    //setup SS pin
    SS_SetDriveMode(SS_DM_RES_UP);
    LTC68_Start();
    LTC6804_initialize();
    LTC6804_wrcfg(TOTAL_IC,tx_cfg);
}


/**
 * @wake all BMSs up, waiting for commands
 *
 * @param no input parameters.
 * @return 1 if everything is OK. 0 for hard failure.
 */
void  wake_up(){
    wakeup_sleep();
}

void check_cfg(){
    //DEBUG_UART_PutString("Enter Check_CFG\n");
    int i=0;
    wakeup_sleep();
    LTC6804_rdcfg(TOTAL_IC,rx_cfg);
    //LCD_Position(1u,0u);
    for (i=0;i<8;i++){
        if (rx_cfg[i] != tx_cfg[i]){
              fatal_err = COM_FAILURE;
            return;
        }
    }
}



void check_chips(){
    
}// check_chips()



uint8_t get_cell_volt(){
    LTC68_ClearFIFO();
   // DEBUG_UART_PutString("Enter GET_CELL_VOLT\n");
    int error;
    wakeup_sleep();
    LTC6804_adcv();
    CyDelay(10);
    wakeup_sleep();
    error = LTC6804_rdcv(0, TOTAL_IC,cell_codes); // Set to read back all cell voltage registers
    if (error == -1)
    {
        #ifdef DEBUG_LCD
            LCD_Position(0u,10u);
            LCD_PrintString("ERROR");
        #endif
       return 1;
    }
    
    //get information
    update_volt(cell_codes);
    
    
    //check error
    check_voltage();
    if (mypack.status!=NORMAL){
        if (mypack.bad_cell_index>0){
            return 1;
        }
    }
    
    return 0;
}// get_cell_volt()


uint8_t get_cell_temp(){
    int error;
    wakeup_sleep();
    LTC6804_adax();
    CyDelay(3);  
    wakeup_sleep();
    error = LTC6804_rdaux(0,TOTAL_IC,aux_codes); // Set to read back all aux registers
    if (error == -1)
    {
        #ifdef DEBUG_LCD
        LCD_Position(0u,10u);
        LCD_PrintString("ERROR");
        #endif
        return 1;
    }
 

    //get information
    update_temp(aux_codes);

    //check error
    check_temp();
    //check error
    if (mypack.status!=NORMAL){
        if (mypack.bad_temp_index>0){
            return 1;
        }
    }

   
   
    #ifdef DEBUG_LCD
        LCD_Position(1u,10u);
        print_cells(aux_codes[0][0]);
        LCD_Position(0u,10u);
        LCD_PrintString("OK");
    #endif
    return 0;
}// get_cell_temp()



uint8_t check_cells(){ 
    //using ADOW
  uint16_t cell_pu[TOTAL_IC][12];
  uint16_t cell_pd[TOTAL_IC][12];
  int error;
  uint8_t i_IC=0;
  uint8_t i_cell=0;

  wakeup_sleep();

  LTC6804_adow(ADOW_PUP_UP);
  error = LTC6804_rdcv(0, TOTAL_IC,cell_pu); // Set to read back all cell voltage registers

  wakeup_sleep();

  LTC6804_adow(ADOW_PUP_DOWN);
  error = LTC6804_rdcv(0, TOTAL_IC,cell_pd); // Set to read back all cell voltage registers

  if (error==-1){
    return 1;
    }

  for (i_IC=0;i_IC<TOTAL_IC;i_IC++){
    for (i_cell=0;i_cell<12;i_cell++){
      if ((((int16_t)cell_pu[i_IC][i_cell+1]-(int16_t)cell_pd[i_IC][i_cell+1]) < -400) && (CELL_ENABLE&(0x1<<i_cell))){
        fatal_err |= CELL_VOLT_UNDER;
        //LCD_Position(1u,0u);
        //LCD_PrintString("big ");
        return 1;
      }
      if (cell_pu[i_IC][0]==0){
        fatal_err |= CELL_VOLT_UNDER;
        //LCD_Position(1u,0u);
        //LCD_PrintString("eq 0");
        return 1;
      }
      if (cell_pd[i_IC][11]==0){
        fatal_err |= CELL_VOLT_UNDER;
        return 1;
      }
    }

  }
    return 0;

}// check_cells()


void update_volt(uint16_t cell_codes[TOTAL_IC][12]){
    uint8_t cell=0;
    uint8_t raw_cell=0;
    uint8_t node = 0;
    uint8_t ic=0;
    uint32_t stack_volt=0;
    uint8_t stack=0;
    uint16_t voltage;
    uint32_t temp_volt;

    //log in voltage data
    for (ic=0;ic<TOTAL_IC;ic++){
        for (raw_cell=0;raw_cell<12;raw_cell++){
            if ((CELL_ENABLE & (0x1<<raw_cell))){
                bat_cell[cell].voltage = cell_codes[ic][raw_cell];
                cell++;
            }
        }
        cell=0;
    }

    // voltage_compensation();
    
    // FE3 update 3 stacks voltage
    node = 0;
    cell = 0;
    stack = 0;
    temp_volt = 0;
    for (stack = 0; stack<3; stack++){
        for (node = 0; node<2; node++){
            for (cell = 0; cell<14; cell++){
                temp_volt += (uint32_t)(bat_stack[stack].node[node]->cells[cell]->voltage);
            }
        }
        bat_stack[stack].voltage = temp_volt;
    }

    // FE3 update pack voltage
    stack = 0;
    for (stack = 0; stack<3; stack++){
        temp_volt += bat_pack.stacks[stack]->voltage;
    }
    bat_pack.voltage = temp_volt;

}

void check_voltage(){
    uint8_t cell = 0;
    uint8_t temp = 0;
    uint16_t voltage16 = 0;
    uint32_t voltage32 = 0;
    // update each cell
    for (cell = 0; cell<N_OF_CELL; cell++){
        voltage16 = bat_cell[cell].voltage;
        if (voltage > (uint16_t)OVER_VOLTAGE){
            bat_cell[cell].bad_counter++;
            bat_cell[cell].bad_type = 1;
        }else if (voltage < (uint16_t)UNDER_VOLTAGE){
            bat_cell[cell].bad_counter++;
            bat_cell[cell].bad_type = 0;
        }else{
            if (bat_cell[cell].bad_counter>0){
                bat_cell[cell].bad_counter--;
            }           
        }
    }

    // update node
    for (node = 0; node< N_OF_NODE; node++){
        for (cell = 0; cell<14; cell++){
            if (bat_node[node].cells[cell]->bad_counter > ERROR_VOLTAGE_LIMIT){
                if (bat_node[node].cells[cell]->bad_type == 0){
                    bad_node[node].under_voltage |= (1u<<cell);
                }else{
                    bad_node[node].over_voltage |= (1u<<cell);
                }
            }
        }
    }

    // update pack of cell voltage error
    for (node = 0; stack < N_OF_NODE; node++){
        if (bat_pack.nodes[node].over_voltage != 0){
            bat_pack.status |= CELL_VOLT_OVER;
            bat_err_add(CELL_VOLT_OVER, bat_node[node].over_voltage, node);
        }

        if (bat_pack.nodes[node].under_voltage != 0){
            bat_pack.status  |= CELL_VOLT_UNDER;
            bat_err_add(CELL_VOLT_UNDER, bat_node[node].under_voltage, node);
        }
    }
        
}

                


void update_temp(uint16_t aux_codes[TOTAL_IC][6]){
    uint8_t cell=0;
    uint8_t ic=0;
    uint8_t stack=0;
    uint16_t temp;
    uint8_t i=0;
    uint8_t temp_found=0;



    // FE3 new data structure
    // update node
    temp = 0;
    for (ic = 0; ic < 12; ic++){
        for (i = 0; i < 6; i++){
            bat_temp[temp].temp = aux_codes[ic][i];
        }
    }

}


void check_temp()){
    uint8_t temp;
    uint8_t node;
    uint16_t temp16;

    // check temp
    for (node = 0; node<N_OF_TEMP; node++){
        temp16 = bat_temp[node].temp;
        if (temp16 > (uint16_t)CRITICAL_TEMP_H){
            bat_temp[temp].bad_counter++;
            bat_temp[temp].bad_type = 1;
        }else if (temp16 < (uint16_t)CRITICAL_TEMP_L){
            bat_temp[temp].bad_counter++;
            bat_temp[temp].bad_type = 0;
        }else{
            if (bat_temp[temp].bad_counter>0){
                bat_temp[temp].bad_counter--;
            }           
        }
    }

    // update node
    for (node = 0; node< N_OF_NODE; node++){
        for (temp = 0; temp<10; temp++){
            if (bat_node[node].temps[temp]->bad_counter > ERROR_TEMPERATURE_LIMIT){
                if (bat_node[node].temps[temp]->bad_type == 0){
                    bad_node[node].under_temp |= (1u<<cell);
                }else{
                    bad_node[node].over_temp |= (1u<<cell);
                }
            }
        }
    }

    // update pack of cell voltage error
    for (node = 0; stack < N_OF_NODE; node++){
        if (bat_pack.nodes[node].over_temp != 0){
            bat_pack.status |= PACK_TEMP_OVER;
            bat_err_add(PACK_TEMP_OVER, bat_node[node].over_temp, node);
        }

        if (bat_pack.nodes[node].under_voltage != 0){
            bat_pack.status  |= PACK_TEMP_UNDER;
            bat_err_add(PACK_TEMP_UNDER, bat_node[node].under_temp, node);
        }
    }
}




void mypack_init(){
    uint8_t stack=0;
    uint8_t cell=0;
    uint8_t node=0;
	// mypack.status = NORMAL;
	// mypack.bad_cell_index =0;
	// mypack.bad_temp_index =0;
	// mypack.fuse_fault=0;
	// mypack.voltage =0;

	// //memset(mypack.bad_temp, 0, sizeof(mypack.temp));
 //    for (stack=0;stack<3;stack++){
 //        for (cell=0;cell<20;cell++){
 //            mypack.bad_temp[stack][cell]=0;
 //        }
 //    }

    // FE3 new data structure
    // initialize cells and temps
    for (cell = 0; cell<N_OF_CELL;cell++){
        bat_cell[cell].voltage = 0;
        bat_cell[cell].bad_volt_counter = 0;
    }
    for (temp = 0; temp < N_OF_TEMP ; temp++){
        bat_temp[temp].temp = 0;
        bat_temp[temp].bad_temp_counter = 0;
        bat_temp[temp].bad_type = 0;
        if ((temp%10) < 5){
            bat_temp[temp].type = THERM_BOARD;
        }
        else{
            bat_temp[temp].type = THERM_CELL;
        }
    }

    // register node
    for (node = 0; node < N_OF_NODE; node++){
        for (cell = 0; cell < 14; cell++){    
            bat_node[node].cells[cell] = &(bat_cell[node*14+cell]);
        }
        for (temp = 0; temp < 10; temp++){
            bat_node[node].temps[temp] = &(bat_temp[node*10+temp]);
        }
        bat_node.over_temp = 0;
        bat_node.under_temp = 0;
        bat_node.over_voltage = 0;
        bat_node.under_voltage = 0;
    }
    // register stack
    for (stack = 0;stack < N_OF_STACK; stack++){
        bat_stack[stack].nodes[0] = &(bat_stack[stack*2]);
        bat_stack[stack].nodes[1] = &(bat_stack[stack*2+1]);
        bat_stack[stack].voltage = 0;
    }
    // register pac
    for (stack = 0; stack< 3; stack++){
        bat_pack.stacks[stack] = &(bat_stack[stack]);
    }
    for (node = 0; node<6; node++){
        bat_pack.nodes[node] = &(bat_node[node]);
    }
    bat_pack.voltage = 0;
    bat_pack.current = 0;
    bat_pack.status = 0; 
    
    
}



void check_stack_fuse()
{
	uint8_t stack=0;

	int delta_0_1, delta_1_2, delta_2_0;

	// compute delta
	delta_0_1 = (int)bat_stack[0].voltage - (int)bat_stack[1].voltage;
	delta_1_2 = (int)bat_stack[1].voltage - (int)bat_stack[2].voltage;
	delta_2_0 = (int)bat_stack[2].voltage - (int)bat_stack[0].voltage;

	// absolute value of delta
	if(delta_0_1 < 0) delta_0_1 *= -1;
	if(delta_1_2 < 0) delta_1_2 *= -1;
	if(delta_2_0 < 0) delta_2_0 *= -1;

	// Comparisons to stack limits
	if(delta_0_1 > STACK_VOLT_DIFF_LIMIT)
		bat_stack[0].bad_counter++;
	else
		if(mypack.stack[0].bad_counter > 0)
			bat_stack[0].bad_counter--;

	if(delta_1_2 > STACK_VOLT_DIFF_LIMIT)
		bat_stack[1].bad_counter++;
	else
		if(bat_stack[1].bad_counter > 0)
			bat_stack[1].bad_counter--;

	if(delta_2_0 > STACK_VOLT_DIFF_LIMIT)
		bat_stack[2].bad_counter++;
	else
		if(bat_stack[2].bad_counter > 0)
			bat_stack[2].bad_counter--;


	stack=0;
	for (stack =0;stack<3;stack++){
		if (bat_stack[stack].bad_counter>FUSE_BAD_LIMIT){
			bat_pack.status |= STACK_FUSE_BROKEN;
			fatal_err |= STACK_FUSE_BROKEN;

		if(bat_stack[0].bad_counter > FUSE_BAD_LIMIT &&
			bat_stack[1].bad_counter > FUSE_BAD_LIMIT)
		{
			bat_pack.fuse_fault = 1;
		} // if fuse on stack 1 fails

		if(bat_stack[1].bad_counter > FUSE_BAD_LIMIT &&
			bat_stack[2].bad_counter > FUSE_BAD_LIMIT)
		{
			bat_pack.fuse_fault = 2;
		} // if fuse on stack 2 fails

		if(bat_stack[2].bad_counter > FUSE_BAD_LIMIT &&
			bat_stack[0].bad_counter > FUSE_BAD_LIMIT)
		{
			bat_pack.fuse_fault = 0;
		} // if fuse on stack 0 fails

		}  
	}
}



uint8_t temp_transfer(uint16_t raw){
    //using 1/T = 1/T0 +(1/B)(R/R0)
    //V = raw/0xffff*5
    //R is R=10K(5-V)/V;
    //translate raw reading to C temperature
    //B25=3900
    //B75=3936
    float vcc=5.0;
    float V=((float)raw*vcc/0xffff);
    float R=10000.0*(vcc-V)/V;
    float R0=10000.0;
    float oneOverT=(1/298.15)+(1/3950.0)*(log(R/R0));
    return ((uint8_t)(floor(1/oneOverT)));
}
//void balance_cells(){}// balance_cells()

void voltage_compensation(){
    //should compsensation to top and bottom cells
    float dV = 500;         //in 0.0001V
    float temp = 0;
    float d=0;
    uint8_t stack=0;
    float dT=0;
    
    for (stack=0;stack<3;stack++){
        //calculate voltage across interface
        if (temp_transfer(mypack.stack[stack].value16) > 25){
            dT = (float)(temp_transfer(mypack.stack[stack].value16) - 25);
            temp = dT*0.017+1.4;
            d = (temp/1.4)*dV;
        }else{
            dT = (float)(25-temp_transfer(mypack.stack[stack].value16));
            temp = 1.4-dT*0.017;
            d = (temp/1.4)*dV;
        }
        if (d>800){
            d=800;
        }else if(d<500){
            d=500;
        }
        mypack.cell[stack][0][0].value16+=(uint16_t)d;
        mypack.cell[stack][1][6].value16+=(uint16_t)d;
        mypack.cell[stack][2][0].value16+=(uint16_t)d;
        mypack.cell[stack][3][6].value16+=(uint16_t)d;
        
    }
    
}
