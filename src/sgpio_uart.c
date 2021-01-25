/**********************************************************************
 * $Id$		sgpio_uart.c		2012-11-12
 *//**
 * @file		spgio_uart.c
 * @brief	Contains all functions support for UART firmware library on lpc43xx
 * @version	1.1
 * @date		12. Nov. 2012
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
/** @addtogroup SGPIO_UART
 * @{
 */

/* Includes ------------------------------------------------------------------- */
#include "sgpio_uart.h"
#include "lpc43xx_cgu.h"

#ifdef SGPIO_UART

#define MAX_PRESET_VALUE	(1<<12)

#define OVER_SAMPLING		1
#if	OVER_SAMPLING
#define OVERSAMPLING_NUM	3	//sample 3 times for one bit
#else
#define OVERSAMPLING_NUM	1
#endif

extern mySGPIO_Type	mySGPIO;

static uint32_t TxFrame, RxFrame, RxStarted ;
static uint32_t FrameLen;
/* Private Functions ---------------------------------------------------------- */


/* End of Private Functions ---------------------------------------------------- */


/* Public Functions ----------------------------------------------------------- */
/** @addtogroup SGPIO_UART_Public_Functions
 * @{
 */

/*****************************************************************************//**
 * @brief		1. Fills each UART_InitStruct member with its default value:
 * 					- 9600 bps (can be 19200, 38400, 57600, 115200)
 * 					- 8-bit data
 * 					- 1 Stopbit
 * 					- None Parity
 *					2. Initialize frame size
 * @param[in]	UART_InitStruct Pointer to a SGPIO_UART_CFG_Type structure which will
 * 				be initialized.
 * @return		None
 *******************************************************************************/
void SGPIO_UART_ConfigStructInit(SGPIO_UART_CFG_Type *UART_InitStruct)
{
    UART_InitStruct->Baud_rate = UART_BAUDRATE_9600;
    UART_InitStruct->Databits = UART_DATABIT_8;
    UART_InitStruct->Parity = UART_PARITY_NONE;
    UART_InitStruct->Stopbits = UART_STOPBIT_1;

    FrameLen = 1 + UART_InitStruct->Databits + UART_InitStruct->Parity + UART_InitStruct->Stopbits;
}

/********************************************************************//**
 * @brief		set the SGPIO UART clock according to the specified
 *               parameters in the UART_ConfigStruct.
 * @param[in]	SGPIO_Txslice	SGPIO slice selected as UART Tx, can be
 *						one of the enum type of SGPIO_Slice.
 * @param[in] SGPIO_Rxslice SGPIO slice selected as UART Rx, can be
 *						one of the enum type of SGPIO_Slice.					
 * @param[in]	UART_ConfigStruct Pointer to a UART_CFG_Type structure
 *            that contains the configuration information for SPIO
 *						UART	
 * @return 		Result SUCCESS or ERROR
 *********************************************************************/
Status SGPIO_UART_setclk(int SGPIO_Txslice, int SGPIO_Rxslice, SGPIO_UART_CFG_Type *UART_ConfigStruct)
{
    uint32_t tmp, a, b, clk;

    LPC_SGPIO->CTRL_ENABLED &= (~(1<<SGPIO_Txslice));
    LPC_SGPIO->CTRL_ENABLED &= (~(1<<SGPIO_Rxslice));
    /* Set up peripheral clock for SGPIO module */
    CGU_EntityConnect(CGU_CLKSRC_PLL1, CGU_BASE_PERIPH);//SGPIO CLK: from PPL1

    /* Config SGPIO according to the config struct */
    //shift clock by baudrate
    clk = CGU_GetPCLKFrequency(CGU_PERIPHERAL_M4CORE);//clk=PLL
    tmp = clk/UART_ConfigStruct->Baud_rate-1;//PRESET = SGPIO_CLK/shift_clk - 1
    if(!(tmp < MAX_PRESET_VALUE)) {
        CGU_EntityConnect(CGU_CLKSRC_PLL1, CGU_CLKSRC_IDIVE);
        a = MAX_PRESET_VALUE*UART_ConfigStruct->Baud_rate;
        b = clk/a + 1;
        CGU_SetDIV(CGU_CLKSRC_IDIVE, b);
        CGU_EntityConnect(CGU_CLKSRC_IDIVE, CGU_BASE_PERIPH);//SGPIO CLK
        tmp = clk/(b*UART_ConfigStruct->Baud_rate)-1;
    } 
    LPC_SGPIO->PRESET[SGPIO_Txslice] = tmp;
    LPC_SGPIO->COUNT[SGPIO_Txslice] = tmp;

    tmp /= OVERSAMPLING_NUM;
    LPC_SGPIO->PRESET[SGPIO_Rxslice] = tmp;
    LPC_SGPIO->COUNT[SGPIO_Rxslice] = tmp;

    //swap clock
    tmp = FrameLen - 1;
    LPC_SGPIO->POS[SGPIO_Txslice] = tmp | (tmp << 8);

    tmp = FrameLen * OVERSAMPLING_NUM - 1;
    LPC_SGPIO->POS[SGPIO_Rxslice] = tmp | (tmp << 8);

    LPC_SGPIO->SLICE_MUX_CFG[SGPIO_Txslice] = 0<<2 | 0<<6; // Clk from COUNTER, 1b shift per clck
    LPC_SGPIO->SGPIO_MUX_CFG[SGPIO_Txslice] = 0<<5; // Enable Clk

    LPC_SGPIO->SLICE_MUX_CFG[SGPIO_Rxslice] = 0<<2 | 0<<6; //COUNTgenCLK, 1b shift per clck
    LPC_SGPIO->SGPIO_MUX_CFG[SGPIO_Rxslice] = 0<<5; // Enable Clk

    return SUCCESS;
}
void SGPIO_UART_EnTxIRQ(int SGPIO_slice)
{
    LPC_SGPIO->SET_EN_1 = 1<<SGPIO_slice;
}
void SGPIO_UART_DisTxIRQ(int SGPIO_slice)
{
    LPC_SGPIO->CLR_EN_1 = 1<<SGPIO_slice;
}
/********************************************************************//**
 * @brief		Initialize the SGPIO as UART Tx
 * @param[in]	SGPIO_TxPin	SGPIO pin for Tx, can be
 *						one of the enum type of SGPIO_Pin.
 * @param[in]	SGPIO_slice SGPIO slice for Tx, can be
 *						one of the enum type of SGPIO_Slice.
 * @return 		None
 *********************************************************************/
void SGPIO_UART_Tx_Init(int SGPIO_TxPin, int SGPIO_slice)
{
    //original data
    LPC_SGPIO->REG[SGPIO_slice] = 0xffffffff;
    LPC_SGPIO->REG_SS[SGPIO_slice] = 0xffffffff;	

    //setup interrupt
    NVIC_EnableIRQ(SGPIO_IINT_IRQn);
    SGPIO_UART_EnTxIRQ(SGPIO_slice);
    TxFrame = 0;

    //set out mode
    LPC_SGPIO->OUT_MUX_CFG[SGPIO_slice]   = 0 | 0<<4;		// 1b out, gpio_oe
    LPC_SGPIO->GPIO_OENREG |= 1<<SGPIO_TxPin;		// Enable SGPIO output
}
/********************************************************************//**
 * @brief		Configure SGPIO to match start bit when UART Rx
 * @param[in]	SGPIO_slice SGPIO slice for Rx, can be
 *						one of the enum type of SGPIO_Slice.
 * @return 		None
 *********************************************************************/
static void SGPIO_UART_Rx_Match(int SGPIO_slice)
{
    LPC_SGPIO->REG_SS[SGPIO_slice] = (0xffffffff>>OVERSAMPLING_NUM);
    LPC_SGPIO->REG[SGPIO_slice] = 0x0;

    LPC_SGPIO->SLICE_MUX_CFG[SGPIO_slice] |= 1<<0; //pattern match mode
    LPC_SGPIO->SET_EN_2 = 1<<SGPIO_slice;
}
/********************************************************************//**
 * @brief		Configure SGPIO to capture data bit after start bit matched
 *					when UART Rx
 * @param[in]	SGPIO_slice SGPIO slice for Rx, can be
 *						one of the enum type of SGPIO_Slice.
 * @return 		None
 *********************************************************************/
static void SGPIO_UART_Rx_Capture(int SGPIO_slice)
{
    uint32_t tmp;

    LPC_SGPIO->CLR_EN_2 = 1<<SGPIO_slice;
    tmp = (FrameLen -1) * OVERSAMPLING_NUM - 2;// 2: one more shift clock generated while being macthed
    LPC_SGPIO->POS[SGPIO_slice] = tmp | (tmp << 8);
    LPC_SGPIO->SLICE_MUX_CFG[SGPIO_slice] &= (~(1<<0)); //remove pattern match mode
    LPC_SGPIO->CTR_STATUS_1 = (1<<SGPIO_slice);
    while(LPC_SGPIO->STATUS_1 & (1<<SGPIO_slice));
    LPC_SGPIO->SET_EN_1 = 1<<SGPIO_slice;
}

/********************************************************************//**
 * @brief		Initialize the SGPIO as UART Rx
 * @param[in]	SGPIO_RxPin	SGPIO pin for Rx, can be
 *						one of the enum type of SGPIO_Pin.
 * @param[in]	SGPIO_slice SGPIO slice for Rx, can be
 *						one of the enum type of SGPIO_Slice.
 * @return 		None
 *********************************************************************/
void SGPIO_UART_Rx_Init(int SGPIO_RxPin, int SGPIO_slice)
{
    //set input mode
    LPC_SGPIO->GPIO_OENREG |= 0<<SGPIO_RxPin;

    //setup interrupt
    LPC_SGPIO->CLR_EN_0 = 1<<SGPIO_slice;
    NVIC_EnableIRQ(SGPIO_IINT_IRQn);
    RxStarted = 0;
    RxFrame = 0;

    //ready to match start bit
    SGPIO_UART_Rx_Match(SGPIO_slice);
}

/*********************************************************************//**
 * @brief		Enable/Disable transfer (Tx/Rx) on SPGIO slice
 * @param[in]	SGPIO_slice SGPIO slice for transfer, can be
 *						one of the enum type of SGPIO_Slice.
 * @param[in]	NewState New State of transfer function, should be:
 * 					- ENABLE	:Enable this function
 *					- DISABLE	:Disable this function
 * @return none
 **********************************************************************/
void SGPIO_UART_Setmode(int SGPIO_slice, FunctionalState NewState)
{

    if (NewState == ENABLE)
    {
        LPC_SGPIO->CTRL_ENABLED |= 1<<SGPIO_slice;
    }	else {
        LPC_SGPIO->CTRL_ENABLED &= (~(1<<SGPIO_slice));
    }
}

/*********************************************************************//**
 * @brief		Enter idle state for Tx function
 * @param[in]	SGPIO_slice SGPIO slice for transmission, can be
 *						one of the enum type of SGPIO_Slice.
 * @return none
 **********************************************************************/
void SGPIO_UART_Enter_SendIdle(int SGPIO_slice)
{
    LPC_SGPIO->REG_SS[SGPIO_slice] = 0xffffffff;	
    while(TxFrame==0);
    TxFrame = 0;
}	

/*********************************************************************//**
 * @brief		Transmit a single data via SGPIO UART
 * @param[in]	SGPIO_slice SGPIO slice for Tx, can be
 *						one of the enum type of SGPIO_Slice.
 * @param[in]	Data	Data to transmit (must be 8-bit long)
 * @return 		None
 **********************************************************************/
void SGPIO_UART_SendByte(int SGPIO_slice, uint8_t Data)
{
    uint32_t tmp;
    tmp = 0x00 | (Data<<1) | (0x01<<9);//10b = start bit(0) + data + stop bit 1
    LPC_SGPIO->REG_SS[SGPIO_slice] = tmp;	
}
/*********************************************************************//**
 * @brief		Receive a single data via SGPIO UART
 * @param[in]	SGPIO_slice SGPIO slice for Rx, can be
 *						one of the enum type of SGPIO_Slice.
 * @return 		Data received
 **********************************************************************/
uint8_t SGPIO_UART_ReceiveByte(int SGPIO_slice)
{
    uint32_t tmp, shift_bit;

    tmp = LPC_SGPIO->REG_SS[SGPIO_slice];
    shift_bit = (FrameLen-1)*OVERSAMPLING_NUM;
    tmp = tmp>>(32-shift_bit);

#if OVER_SAMPLING	
    {
        uint32_t db[OVERSAMPLING_NUM];
        uint8_t	i,j;

        shift_bit -= OVERSAMPLING_NUM;
        i = 0;
        while(i<shift_bit) {
            //get 3 sampling value of one bit
            db[0] = (tmp>>i)&0x00000001;
            db[1] = (tmp>>(1+i))&0x00000001;
            db[2] = (tmp>>(2+i))&0x00000001;
            //compare the sampling values and use the one same to another in three
            j = i/OVERSAMPLING_NUM;
            tmp &= ~(1<<j);
            if(db[0] == db[1]) {
                tmp |= (db[0]<<j);
            } else if(db[0] == db[2]) {
                tmp |= (db[0]<<j);
            } else {
                tmp |= (db[1]<<j);
            }

            i += OVERSAMPLING_NUM;
        }
    }
#endif	
    return (uint8_t)(tmp&0xff);
}

/*********************************************************************//**
 * @brief		Send a block of data via SGPIO UART in half duplex mode
 * @param[in]	SGPIO_slice SGPIO slice for Tx, can be
 *						one of the enum type of SGPIO_Slice.
 * @param[in]	txbuf 	Pointer to Transmit buffer
 * @param[in]	buflen 	Length to be Transmitted
 * @return 		Number of bytes sent.
 *
 * Note: The function won't exit until the length to be sent is up
 **********************************************************************/
uint32_t SGPIO_UART_Send(int SGPIO_slice, uint8_t *txbuf, uint32_t buflen)
{
    uint32_t bToSend, bSent;
    uint8_t *pChar = txbuf;

    bToSend = buflen;
    bSent = 0;
    //SGPIO_UART_EnTxIRQ(SGPIO_slice);
    SGPIO_UART_Setmode(SGPIO_slice, ENABLE);
    while(bToSend){				
        SGPIO_UART_SendByte(SGPIO_slice, (*pChar++));
        while (TxFrame==0);
        bSent++;
        bToSend--;
        TxFrame = 0;		
    }
    SGPIO_UART_Enter_SendIdle(SGPIO_slice);
    //SGPIO_UART_DisTxIRQ(SGPIO_slice);
    SGPIO_UART_Setmode(SGPIO_slice, DISABLE);
    return bSent;
}

/*********************************************************************//**
 * @brief		Receive a block of data via SGPIO UART in half duplex mode
 * @param[in] SGPIO_slice SGPIO slice for Rx, can be
 *						one of the enum type of SGPIO_Slice.
 * @param[out]	rxbuf 	Pointer to Received buffer
 * @param[in]	buflen 	Length to be Received
 * @return 		Number of bytes received
 *
 * Note: The function won't exit until the length to be received is up		
 **********************************************************************/
uint32_t SGPIO_UART_Receive(int SGPIO_slice, uint8_t *rxbuf,	uint32_t buflen)
{
    uint32_t bToRecv, bRecv;
    uint8_t *pChar = rxbuf;

    bToRecv = buflen;
    bRecv = 0;

    while(bToRecv) {
        while (RxFrame==0);		
        (*pChar++) = SGPIO_UART_ReceiveByte(SGPIO_slice);
        RxFrame = 0;
        RxStarted = 0;
        bToRecv--;
        bRecv++;
        SGPIO_UART_Rx_Match(SGPIO_slice);
    }

    return bRecv;
}
/*********************************************************************//**
 * @brief		Send a block of data via SGPIO UART in full duplex mode
 * @param[in]	SGPIO_slice SGPIO slice for Tx, can be
 *						one of the enum type of SGPIO_Slice.
 * @param[in]	txbuf 	Pointer to Transmit buffer
 * @param[in]	buflen 	Length to be Transmitted
 * @param[in] txlen		Length sent last time
 * @return 		Number of bytes sent.
 *
 * Note: The function will exit after write a single data to register
 **********************************************************************/
uint32_t SGPIO_UART_SendFull(int SGPIO_slice, uint8_t *txbuf, uint32_t buflen, uint32_t txlen)
{
    uint32_t bToSend, bSent;
    uint8_t *pChar = txbuf;

    bToSend = buflen - txlen;
    if(!txlen) {
        //SGPIO_UART_EnTxIRQ(SGPIO_slice);
        SGPIO_UART_Setmode(SGPIO_slice, ENABLE);
    }
    bSent = txlen;
    while(bToSend){				
        SGPIO_UART_SendByte(SGPIO_slice, (*(pChar+bSent)));
        if(TxFrame==0) return bSent;
        bSent++;
        bToSend--;
        TxFrame = 0;		
    }
    SGPIO_UART_Enter_SendIdle(SGPIO_slice);
    //SGPIO_UART_DisTxIRQ(SGPIO_slice);
    SGPIO_UART_Setmode(SGPIO_slice, DISABLE);
    return bSent;
}
/*********************************************************************//**
 * @brief		Receive a block of data via SGPIO UART in full duplex mode
 * @param[in]	SGPIO_slice SGPIO slice for Rx, can be
 *						one of the enum type of SGPIO_Slice.
 * @param[in]	rxbuf 	Pointer to Received buffer
 * @param[in] rxlen		Length Received last time
 * @return 		Number of bytes Received.
 *
 * Note: The function will exit if no data received
 **********************************************************************/
uint32_t SGPIO_UART_RxFull(int SGPIO_slice, uint8_t *rxbuf, uint32_t rxlen)
{
    uint32_t bRecv;
    uint8_t *pChar = rxbuf;

    bRecv = rxlen;
    if(RxFrame){		
        (*(pChar+bRecv)) = SGPIO_UART_ReceiveByte(SGPIO_slice);
        SGPIO_UART_Rx_Match(SGPIO_slice);
        RxFrame = 0;
        RxStarted = 0;
        bRecv++;
    }
    return bRecv;
}

/*********************************************************************//**
 * @brief		SGPIO irq handler for Tx and Rx (match&capture data)
 * @param[in]	None
 * @return 		None
 *
 * Note: globle structure mySGPIO is used
 **********************************************************************/
void SGPIO_IRQHandler(void)
{
    unsigned int status;

    if(RxStarted) {
        status = LPC_SGPIO->STATUS_1;
        if (status & (1<<mySGPIO.RxSlice)) {
            LPC_SGPIO->CTR_STATUS_1 = (1<<mySGPIO.RxSlice);
            while(LPC_SGPIO->STATUS_1 & (1<<mySGPIO.RxSlice));
            LPC_SGPIO->CLR_EN_1 = 1<<mySGPIO.RxSlice;//stop interrupt handler when rx over
            RxFrame = 1;
            RxStarted = 0;
        }
    } else {// for match data
        status = LPC_SGPIO->STATUS_2;
        if (LPC_SGPIO->STATUS_2 & (1<<mySGPIO.RxSlice)) {		
            LPC_SGPIO->CTR_STATUS_2 = (1<<mySGPIO.RxSlice);
            while(LPC_SGPIO->STATUS_2 & (1<<mySGPIO.RxSlice));
            SGPIO_UART_Rx_Capture(mySGPIO.RxSlice);
            RxStarted=1;//matched
        }
    }

    status = LPC_SGPIO->STATUS_1;
    if (status & (1<<mySGPIO.TxSlice)) {
        LPC_SGPIO->CTR_STATUS_1 = (1<<mySGPIO.TxSlice);
        while(LPC_SGPIO->STATUS_1 & (1<<mySGPIO.TxSlice));
        TxFrame	= 1;
    } 
}

#endif /* SGPIO_UART */

/**
 * @}
 */

/**
 * @}
 */
/* --------------------------------- End Of File ------------------------------ */
