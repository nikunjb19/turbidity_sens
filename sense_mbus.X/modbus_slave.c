#include <xc.h>
#include "modbus_slave.h"
#include "mcc_generated_files/pin_manager.h"

//  Global Variables 
static uint16_t crcModbusSlave = 0xFFFF;

//static uint8_t modbus_rx_query[DEFAULT_RX_QUERY_LENGTH] = {0};
//static uint8_t modbus_tx_resp[22] = {0};

volatile MODBUS_SLAVE_DATA	modbusSlaveData;

#define reInitModbusRxPointer()     modbusSlaveData.RxPointer   = 0;    modbusSlaveData.TotalRxValue    = 8;
#define startTimoutTimer()          T1CONbits.TMR1ON = 1
#define stopTimoutTimer()           T1CONbits.TMR1ON = 0

void InitModbusSlave(uint8_t u8SlaveId)
{
    modbusSlaveData.flags.all = 0U;

    modbusSlaveData.address = u8SlaveId;
//    modbusSlaveData.StartAddress = 0U;
    modbusSlaveData.RxPointer = 0U;
    modbusSlaveData.TotalRxValue = DEFAULT_RX_QUERY_LENGTH;
    RxMode458();
}

void calculate_crc(uint8_t crcreg)
{
    uint16_t crcbit;

    crcModbusSlave  ^= crcreg;
    for(uint8_t i=0; i<=7; i++)
    {
        crcbit = 0;
        if((crcModbusSlave & 0x0001) == 0x0001)
            crcbit = 1;

        crcModbusSlave	 >>= 1;
        if(crcbit)
            crcModbusSlave  ^= 0xa001;
    }
}

void mbus_rx_handler(void)
{
    initTimoutValue();
    modbusSlaveData.ModbusFrame[modbusSlaveData.RxPointer++] = RCREG;

    if(modbusSlaveData.RxPointer == 7U)
    {
        if(modbusSlaveData.ModbusFrame[1U] == WRITE_MULTIPLE_HOLDING_REGISTER)
        {
            modbusSlaveData.TotalRxValue += (modbusSlaveData.ModbusFrame[6U]+1U);
        }
    }

    if(modbusSlaveData.RxPointer == modbusSlaveData.TotalRxValue)
    {
        ModbusSlaveProcessReceivedQuery();
    }
}

void mbus_tx_handler(void)
{
    initTimoutValue();
    if(modbusSlaveData.TxPointer < modbusSlaveData.TotalTxValue)
    {
        TXREG  = modbusSlaveData.ModbusFrame[modbusSlaveData.TxPointer++];
    }
    if(modbusSlaveData.TxPointer == modbusSlaveData.TotalTxValue)
    {
        while(!TXSTAbits.TRMT);
        DisTxModbus();

        reInitModbusRxPointer();
        RxMode458();
        startTimoutTimer();
    }
}

void mbus_timer_handler(void)
{
    initTimoutValue();
    reInitModbusRxPointer();
}

void ModbusSlaveProcessReceivedQuery(void)
{
   unsigned int i, j, k;

    crcModbusSlave = 0xFFFF;
    for(i=0; i<modbusSlaveData.TotalRxValue-2; i++)
    {
        calculate_crc(modbusSlaveData.ModbusFrame[i]);
    }

    if(modbusSlaveData.ModbusFrame[i++] !=  (crcModbusSlave & 0xFF))
    {
        modbusSlaveData.flags.bitValue.error = 1;
        return;
    }

    if(modbusSlaveData.ModbusFrame[i] !=  (crcModbusSlave>>8))
    {
        modbusSlaveData.flags.bitValue.error = 1;
        return;
    }

    if(modbusSlaveData.ModbusFrame[0] != modbusSlaveData.address)
    {
        return;
    }
    modbusSlaveData.TotalTxValue = 0;
    switch(modbusSlaveData.ModbusFrame[1])
    {
        case READ_HOLDING_REGISTERS:
            modbusSlaveData.TotalTxValue = 2;
            j = (modbusSlaveData.ModbusFrame[2] << 8U) | modbusSlaveData.ModbusFrame[3];
            
            modbusSlaveData.ModbusFrame[modbusSlaveData.TotalTxValue++] = modbusSlaveData.ModbusFrame[5]<<1;
            uint8_t u8ReadReg = modbusSlaveData.ModbusFrame[5];
            for(i=j; i<(u8ReadReg+j); i++)
            {
                modbusSlaveData.ModbusFrame[modbusSlaveData.TotalTxValue++] = (modbusSlaveData.ModbusHoldingRegister[i] >> 8U) & 0xFF;
                modbusSlaveData.ModbusFrame[modbusSlaveData.TotalTxValue++] = modbusSlaveData.ModbusHoldingRegister[i] & 0xFF;
            }
            break;

        case WRITE_SINGLE_HOLDING_REGISTER:
            i = modbusSlaveData.ModbusFrame[2];
            i <<= 8U;
            i |= modbusSlaveData.ModbusFrame[3];
            
            uint16_t u16Data = modbusSlaveData.ModbusFrame[4];
            u16Data <<= 8U;
            u16Data |= modbusSlaveData.ModbusFrame[5];
            modbusSlaveData.ModbusHoldingRegister[i] = u16Data;
            modbusSlaveData.TotalTxValue = 6;
            break;

        case WRITE_MULTIPLE_HOLDING_REGISTER:
            j = modbusSlaveData.ModbusFrame[3]<<1;
            k = 7;
            for(i=j; i<(modbusSlaveData.ModbusFrame[6]+j); i++)
            {
                modbusSlaveData.ModbusHoldingRegister[i] = modbusSlaveData.ModbusFrame[k++];
            }
            modbusSlaveData.TotalTxValue = 6;
            break;
        default:
            
            break;
    }

    crcModbusSlave = 0xFFFF;
    for(i=0; i<modbusSlaveData.TotalTxValue; i++)
    {
        calculate_crc(modbusSlaveData.ModbusFrame[i]);
    }
    modbusSlaveData.ModbusFrame[modbusSlaveData.TotalTxValue++] = crcModbusSlave & 0xFF;
    modbusSlaveData.ModbusFrame[modbusSlaveData.TotalTxValue++] = crcModbusSlave >> 8;

    stopTimoutTimer();
    modbusSlaveData.TxPointer = 0;
    TxMode458();
    EnTxModbus();
}