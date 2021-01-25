/**********************************************************************
 * $Id$		sgpio_uart.h		2012-09-03
 *//**
 * @file		sgpio_uart.h
 * @brief	Contains all macro definitions and function prototypes
 * 			support for UART firmware library on lpc43xx
 * @version	1.0
 * @date		3. Sep. 2012
 * @author	NXP MCU SW Application Team
 *
 * Copyright(C) 2012, NXP Semiconductor
 * All rights reserved.
 *
 ***********************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors’
 * relevant copyright in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 **********************************************************************/

/* Peripheral group ----------------------------------------------------------- */
/** @defgroup SGPIO_UART
 * @ingroup LPC4300CMSIS_FwLib_Drivers
 * @{
 */

#ifndef __SGPIO_UART_H
#define __SGPIO_UART_H

/* Includes ------------------------------------------------------------------- */
//#include "LPC43xx.h"
#include "lpc_types.h"
//#include "sgpio.h"


#ifdef __cplusplus
extern "C"
{
#endif

    /* Public Macros -------------------------------------------------------------- */
    /** @defgroup SGPIO_UART_Public_Macros  SGPIO_UART Public Macros
     * @{
     */


    /**
     * @}
     */

    /* Private Macros ------------------------------------------------------------- */
    /** @defgroup SGPIO_UART_Private_Macros SGPIO_UART Private Macros
     * @{
     */

    /* --------------------- BIT DEFINITIONS -------------------------------------- */

    /**
     * @}
     */


    /* Public Types --------------------------------------------------------------- */
    /** @defgroup SGPIO_UART_Public_Types SGPIO_UART Public Types
     * @{
     */

    typedef struct {
        int	TxPin;
        int	TxSlice;
        int	RxPin;
        int	RxSlice;
    } mySGPIO_Type;

    /***********************************************************************
     * @brief UART enumeration
     **********************************************************************/
    typedef enum {
        SGPIO_UART_BAUDRATE_9600 = 9600,     		
        SGPIO_UART_BAUDRATE_19200 = 19200,		     			
        SGPIO_UART_BAUDRATE_38400 = 38400,     
        SGPIO_UART_BAUDRATE_57600 = 57600,
        SGPIO_UART_BAUDRATE_115200 = 115200
    } SGPIO_UART_BAUDRATE_Type;

    /**
     * @brief UART Databit type definitions
     */
    typedef enum {
        SGPIO_UART_DATABIT_5		= 5,     		/*!< UART 5 bit data mode */
        SGPIO_UART_DATABIT_6,		     			/*!< UART 6 bit data mode */
        SGPIO_UART_DATABIT_7,		     			/*!< UART 7 bit data mode */
        SGPIO_UART_DATABIT_8		     			/*!< UART 8 bit data mode */
    } SGPIO_UART_DATABIT_Type;

    /**
     * @brief UART Stop bit type definitions
     */
    typedef enum {
        SGPIO_UART_STOPBIT_1		= (1),   					/*!< UART 1 Stop Bits Select */
        SGPIO_UART_STOPBIT_2		 							/*!< UART Two Stop Bits Select */
    } SGPIO_UART_STOPBIT_Type;

    /**
     * @brief UART Parity type definitions
     */
    typedef enum {
        SGPIO_UART_PARITY_NONE 	= 0,					/*!< No parity */
        SGPIO_UART_PARITY_ODD = 1,	 						/*!< Odd parity */
        SGPIO_UART_PARITY_EVEN = 1, 							/*!< Even parity */
        SGPIO_UART_PARITY_SP_1	= 1, 							/*!< Forced "1" stick parity */
        SGPIO_UART_PARITY_SP_0 = 1							/*!< Forced "0" stick parity */
    } SGPIO_UART_PARITY_Type;

    /********************************************************************//**
     * @brief UART Configuration Structure definition
     **********************************************************************/
    typedef struct {
        uint32_t Baud_rate;   		/**< UART baud rate */
        SGPIO_UART_PARITY_Type Parity;    	/**< Parity selection, should be:
                                          - SGPIO_UART_PARITY_NONE: No parity
                                          - SGPIO_UART_PARITY_ODD: Odd parity
                                          - SGPIO_UART_PARITY_EVEN: Even parity
                                          - SGPIO_UART_PARITY_SP_1: Forced "1" stick parity
                                          - SGPIO_UART_PARITY_SP_0: Forced "0" stick parity
                                          */
        SGPIO_UART_DATABIT_Type Databits;   /**< Number of data bits, should be:
                                        - SGPIO_UART_DATABIT_5: UART 5 bit data mode
                                        - SGPIO_UART_DATABIT_6: UART 6 bit data mode
                                        - SGPIO_UART_DATABIT_7: UART 7 bit data mode
                                        - SGPIO_UART_DATABIT_8: UART 8 bit data mode
                                        */
        SGPIO_UART_STOPBIT_Type Stopbits;   /**< Number of stop bits, should be:
                                        - SGPIO_UART_STOPBIT_1: UART 1 Stop Bits Select
                                        - SGPIO_UART_STOPBIT_2: UART 2 Stop Bits Select
                                        */
    } SGPIO_UART_CFG_Type;


    /**
     * @}
     */


    /* Public Functions ----------------------------------------------------------- */
    /** @defgroup UART_Public_Functions UART Public Functions
     * @{
     */
    /* SGPIO UART Init functions --------------------------------------------------*/
    Status SGPIO_UART_setclk(int SGPIO_Txslice, int SGPIO_Rxslice, SGPIO_UART_CFG_Type *UART_ConfigStruct);
    void SGPIO_UART_Tx_Init(int SGPIO_TxPin, int SGPIO_slice);
    void SGPIO_UART_Rx_Init(int SGPIO_RxPin, int SGPIO_slice);
    void SGPIO_UART_ConfigStructInit(SGPIO_UART_CFG_Type *UART_InitStruct);

    /* SGPIO UART Send/Receive functions -------------------------------------------------*/
    void SGPIO_UART_SendByte(int SGPIO_slice, uint8_t Data);
    uint8_t SGPIO_UART_ReceiveByte(int SGPIO_slice);
    uint32_t SGPIO_UART_Send(int SGPIO_slice, uint8_t *txbuf, uint32_t buflen);
    uint32_t SGPIO_UART_Receive(int SGPIO_slice, uint8_t *rxbuf,	uint32_t buflen);
    uint32_t SGPIO_UART_SendFull(int SGPIO_slice, uint8_t *txbuf, uint32_t buflen, uint32_t txlen);
    uint32_t SGPIO_UART_RxFull(int SGPIO_slice, uint8_t *rxbuf, uint32_t rxlen);

    /* UART operate functions -------------------------------------------------------*/
    void SGPIO_UART_Setmode(int SGPIO_slice, FunctionalState NewState);

    /**
     * @}
     */


#ifdef __cplusplus
}
#endif


#endif /* __SGPIO_UART_H */

/**
 * @}
 */

