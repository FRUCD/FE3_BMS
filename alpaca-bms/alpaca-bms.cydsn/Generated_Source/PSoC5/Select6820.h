/*******************************************************************************
* File Name: Select6820.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_Select6820_H) /* Pins Select6820_H */
#define CY_PINS_Select6820_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Select6820_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 Select6820__PORT == 15 && ((Select6820__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    Select6820_Write(uint8 value);
void    Select6820_SetDriveMode(uint8 mode);
uint8   Select6820_ReadDataReg(void);
uint8   Select6820_Read(void);
void    Select6820_SetInterruptMode(uint16 position, uint16 mode);
uint8   Select6820_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the Select6820_SetDriveMode() function.
     *  @{
     */
        #define Select6820_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define Select6820_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define Select6820_DM_RES_UP          PIN_DM_RES_UP
        #define Select6820_DM_RES_DWN         PIN_DM_RES_DWN
        #define Select6820_DM_OD_LO           PIN_DM_OD_LO
        #define Select6820_DM_OD_HI           PIN_DM_OD_HI
        #define Select6820_DM_STRONG          PIN_DM_STRONG
        #define Select6820_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define Select6820_MASK               Select6820__MASK
#define Select6820_SHIFT              Select6820__SHIFT
#define Select6820_WIDTH              1u

/* Interrupt constants */
#if defined(Select6820__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in Select6820_SetInterruptMode() function.
     *  @{
     */
        #define Select6820_INTR_NONE      (uint16)(0x0000u)
        #define Select6820_INTR_RISING    (uint16)(0x0001u)
        #define Select6820_INTR_FALLING   (uint16)(0x0002u)
        #define Select6820_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define Select6820_INTR_MASK      (0x01u) 
#endif /* (Select6820__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Select6820_PS                     (* (reg8 *) Select6820__PS)
/* Data Register */
#define Select6820_DR                     (* (reg8 *) Select6820__DR)
/* Port Number */
#define Select6820_PRT_NUM                (* (reg8 *) Select6820__PRT) 
/* Connect to Analog Globals */                                                  
#define Select6820_AG                     (* (reg8 *) Select6820__AG)                       
/* Analog MUX bux enable */
#define Select6820_AMUX                   (* (reg8 *) Select6820__AMUX) 
/* Bidirectional Enable */                                                        
#define Select6820_BIE                    (* (reg8 *) Select6820__BIE)
/* Bit-mask for Aliased Register Access */
#define Select6820_BIT_MASK               (* (reg8 *) Select6820__BIT_MASK)
/* Bypass Enable */
#define Select6820_BYP                    (* (reg8 *) Select6820__BYP)
/* Port wide control signals */                                                   
#define Select6820_CTL                    (* (reg8 *) Select6820__CTL)
/* Drive Modes */
#define Select6820_DM0                    (* (reg8 *) Select6820__DM0) 
#define Select6820_DM1                    (* (reg8 *) Select6820__DM1)
#define Select6820_DM2                    (* (reg8 *) Select6820__DM2) 
/* Input Buffer Disable Override */
#define Select6820_INP_DIS                (* (reg8 *) Select6820__INP_DIS)
/* LCD Common or Segment Drive */
#define Select6820_LCD_COM_SEG            (* (reg8 *) Select6820__LCD_COM_SEG)
/* Enable Segment LCD */
#define Select6820_LCD_EN                 (* (reg8 *) Select6820__LCD_EN)
/* Slew Rate Control */
#define Select6820_SLW                    (* (reg8 *) Select6820__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Select6820_PRTDSI__CAPS_SEL       (* (reg8 *) Select6820__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Select6820_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Select6820__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Select6820_PRTDSI__OE_SEL0        (* (reg8 *) Select6820__PRTDSI__OE_SEL0) 
#define Select6820_PRTDSI__OE_SEL1        (* (reg8 *) Select6820__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Select6820_PRTDSI__OUT_SEL0       (* (reg8 *) Select6820__PRTDSI__OUT_SEL0) 
#define Select6820_PRTDSI__OUT_SEL1       (* (reg8 *) Select6820__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Select6820_PRTDSI__SYNC_OUT       (* (reg8 *) Select6820__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(Select6820__SIO_CFG)
    #define Select6820_SIO_HYST_EN        (* (reg8 *) Select6820__SIO_HYST_EN)
    #define Select6820_SIO_REG_HIFREQ     (* (reg8 *) Select6820__SIO_REG_HIFREQ)
    #define Select6820_SIO_CFG            (* (reg8 *) Select6820__SIO_CFG)
    #define Select6820_SIO_DIFF           (* (reg8 *) Select6820__SIO_DIFF)
#endif /* (Select6820__SIO_CFG) */

/* Interrupt Registers */
#if defined(Select6820__INTSTAT)
    #define Select6820_INTSTAT            (* (reg8 *) Select6820__INTSTAT)
    #define Select6820_SNAP               (* (reg8 *) Select6820__SNAP)
    
	#define Select6820_0_INTTYPE_REG 		(* (reg8 *) Select6820__0__INTTYPE)
#endif /* (Select6820__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_Select6820_H */


/* [] END OF FILE */
