/* 
 * File:   modbus_slave.h
 * Author: Dell05
 *
 * Created on 25 January, 2021, 12:42 PM
 */

#ifndef MODBUS_SLAVE_H
#define	MODBUS_SLAVE_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "mcc_generated_files/pin_manager.h"
    
    
#define TxMode458() ENDI_485_SetHigh();
#define RxMode458() ENDI_485_SetLow();

#define EnTxModbus()  PIE1bits.TXIE  = 1;    TXSTAbits.TXEN = 1
#define DisTxModbus() PIE1bits.TXIE  = 0;    TXSTAbits.TXEN = 0

#define initTimoutValue()   TMR1L   =   0x00;   TMR1H   =   0xE0; PIR1bits.TMR1IF = 0;

#define DEFAULT_RX_QUERY_LENGTH         8

#define READ_HOLDING_REGISTERS          3U
#define WRITE_SINGLE_HOLDING_REGISTER   6U
#define WRITE_MULTIPLE_HOLDING_REGISTER 16U    

typedef union
{
    unsigned int	all;
    struct	MODBUS_BITS
    {
        unsigned int    busyProcessing:1;
        unsigned int    processTimeOut:1;
        unsigned int	processSuccess:1;
        unsigned int	error:1;
        unsigned int	responseReceived:1;
        unsigned int	rsved1:1;
        unsigned int	rsved2:1;
        unsigned int	rsved3:1;
        unsigned int	rsved4:1;
        unsigned int	rsved5:1;
        unsigned int	rsved6:1;
        unsigned int	rsved7:1;
        unsigned int	rsved8:1;
        unsigned int	rsved9:1;
        unsigned int	rsved10:1;
        unsigned int	rsved11:1;
    }bitValue;
}MODBUS_FLAG;

typedef struct
{
//    uint16_t	StartAddress;
//    uint16_t	TotalRegisters;
    uint16_t    ModbusHoldingRegister[8];
    uint8_t     ModbusFrame[22];
    uint8_t     TxPointer;
    uint8_t     RxPointer;
    uint8_t     TotalTxValue;
    uint8_t     TotalRxValue;

    MODBUS_FLAG     flags;
//    MODBUS_FUNCTION	function;
    uint8_t     address;
//    uint16_t	ResponseTime;
//    uint16_t	ResponseTimeout;
}MODBUS_SLAVE_DATA;

extern volatile MODBUS_SLAVE_DATA	modbusSlaveData;

void InitModbusSlave(uint8_t u8SlaveId);
void calculate_crc(uint8_t crcreg);
void mbus_rx_handler(void);
void mbus_tx_handler(void);
void mbus_timer_handler(void);
void ModbusSlaveProcessReceivedQuery(void);

#ifdef	__cplusplus
}
#endif

#endif	/* MODBUS_SLAVE_H */

