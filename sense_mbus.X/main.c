/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC16F1509
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/examples/i2c_master_example.h"
#include "modbus_slave.h"
#include "math.h"

/*
                         Main application
 */
const HEFregion[32] __at(0x1FE0);

#define MCP3422_ADDRESS             0x68// I2C slave address

#define MCP3422_CONFIG_CH1          0x80
#define MCP3422_CONFIG_CH2          0xA0

#define MCP3422_CONFIG_12BIT        0x00
#define MCP3422_CONFIG_14BIT        0x04
#define MCP3422_CONFIG_16BIT        0x08
#define MCP3422_CONFIG_18BIT        0x0C
#define MCP3422_CONFIG_GAIN1        0x00
#define MCP3422_CONFIG_GAIN2        0x01
#define MCP3422_CONFIG_GAIN4        0x02
#define MCP3422_CONFIG_GAIN8        0x03
#define MCP3422_CONFIG_CONTINUOUS   0x10
#define MCP3422_CONFIG_ONESHOT      0x00

#define MCP3422_CONFIG              (MCP3422_CONFIG_CONTINUOUS | MCP3422_CONFIG_GAIN1 | MCP3422_CONFIG_18BIT)
#define MCP3422_CONVERSION_TIME_MS  420// Conversion time for 18-bit mode (80ms)

#define MAGIC_WORD1             0x3FFFU
#define MAGIC_WORD2             0x3400U
#define DEF_TD_ZERO             0
#define DEF_TD_SPAN             400
#define DEF_TD_ZERO_CAL_VAL     900
#define DEF_TD_SPAN_CAL_VAL     1800
#define DEF_SLAVE_ID            1

#define _NTC_R_SERIES         33000.0f

#define _NTC_R_NOMINAL        10000.0f
#define _NTC_TEMP_NOMINAL     25.0f
#define _NTC_ADC_MAX          5.0
#define _NTC_BETA             3950

#define FLASH_ADDRESS    0x1FE0

float tempNTC = 0;
bool bCalMode = false;

long map(float x, long in_min, long in_max, long out_min, long out_max);
static void MCP3422_Write(uint8_t data);
static void MCP3422_Read(uint8_t *data, uint8_t count);
static float MCP3422_ReadVoltage(uint8_t channel);
void get_ntc_temperature(float adcVoltage);


long map(float x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void MCP3422_Write(uint8_t data)
{
    I2C_WriteNBytes(MCP3422_ADDRESS,&data,1U);
}

static void MCP3422_Read(uint8_t *data, uint8_t count)
{
    I2C_ReadNBytes(MCP3422_ADDRESS, data, count);
}

static float MCP3422_ReadVoltage(uint8_t channel)
{
    uint32_t adc_value = 0U;
    uint8_t data[3] = {0U};
    
    MCP3422_Write((MCP3422_CONFIG | channel));
    __delay_ms(MCP3422_CONVERSION_TIME_MS); // Wait for conversion to complete
    
    MCP3422_Read(data, 3U);
    adc_value = data[0];
    adc_value <<= 8U;
    adc_value |= data[1];
    adc_value <<= 8U;
    adc_value |= data[2]; // Combine the three data bytes into a 24-bit ADC value
    
    // Convert 18-bit ADC data to voltage
    return ((float)adc_value * 0.015625f);
}

void get_ntc_temperature(float adcVoltage)
{
    adcVoltage /= 1000.0f;
    float rntc = (float)_NTC_R_SERIES / (((float)_NTC_ADC_MAX / (float)adcVoltage ) - 1.0f);
    float temp;
    temp = rntc / (float)_NTC_R_NOMINAL; 
    temp = logf(temp);
    temp /= (float)_NTC_BETA;
    temp += 1.0f / ((float)_NTC_TEMP_NOMINAL + 273.15f);
    temp = 1.0f / temp;
    temp -= 273.15f;
    tempNTC = temp;
}

void main(void)
{
    float fNtcVoltage = 0.0f;
    float fTdVoltage = 0.0f;
    uint16_t u16TdVal = 0.0f;
    uint16_t u16CalBuffer[8] = {0U};
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    __delay_ms(10);

    u16CalBuffer[0] = FLASH_ReadWord(FLASH_ADDRESS);
    u16CalBuffer[1] = FLASH_ReadWord(FLASH_ADDRESS+1);
    u16CalBuffer[2] = FLASH_ReadWord(FLASH_ADDRESS+2);
    u16CalBuffer[3] = FLASH_ReadWord(FLASH_ADDRESS+3);
//    u16SlaveId = FLASH_ReadWord(SLAVE_ID);

    if((MAGIC_WORD1 == u16CalBuffer[0]) || (MAGIC_WORD2 == u16CalBuffer[0]))
    {
        u16CalBuffer[0] = DEF_TD_ZERO;
    }
    if((MAGIC_WORD1 == u16CalBuffer[2]) || (MAGIC_WORD2 == u16CalBuffer[2]))
    {
        u16CalBuffer[2] = DEF_TD_SPAN;
    }
    if((MAGIC_WORD1 == u16CalBuffer[1]) || (MAGIC_WORD2 == u16CalBuffer[1]))
    {
        u16CalBuffer[1] = DEF_TD_ZERO_CAL_VAL;
    }
    if((MAGIC_WORD1 == u16CalBuffer[3]) || (MAGIC_WORD2 == u16CalBuffer[3]))
    {
        u16CalBuffer[3] = DEF_TD_SPAN_CAL_VAL;
    }
//    if((MAGIC_WORD1 == u16SlaveId) || (MAGIC_WORD2 == u16SlaveId))
//    {
//        u16SlaveId = DEF_SLAVE_ID;
//    }

//    InitModbusSlave(u16SlaveId);
    InitModbusSlave(1U);
    __delay_ms(10);
    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();
    
    // Disable the Global Interrupts
//    INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    while (1)
    {
        // Feed the watchdog timer
        CLRWDT();
        // Add your application code
        fTdVoltage = MCP3422_ReadVoltage(MCP3422_CONFIG_CH1); // Read from channel 1
        fNtcVoltage = MCP3422_ReadVoltage(MCP3422_CONFIG_CH2);// Read from channel 2
        LED_Toggle();
        get_ntc_temperature(fNtcVoltage);
        
        if((modbusSlaveData.ModbusHoldingRegister[4U] == 9588U) && (bCalMode == false))
        {
            modbusSlaveData.ModbusHoldingRegister[4U] = 1U;
//            modbusSlaveData.ModbusHoldingRegister[7U] = u16SlaveId;
            bCalMode = true;
        }
        else if((modbusSlaveData.ModbusHoldingRegister[4U] == 4633U) && (bCalMode == true))
        {
            modbusSlaveData.ModbusHoldingRegister[4U] = 0U;
            INTERRUPT_GlobalInterruptDisable();
            INTERRUPT_PeripheralInterruptDisable();
            FLASH_WriteBlock(FLASH_ADDRESS,&u16CalBuffer);
            __delay_ms(50);
            INTERRUPT_GlobalInterruptEnable();
            INTERRUPT_PeripheralInterruptEnable();
            bCalMode = false;
        }
        else
        {
            modbusSlaveData.ModbusHoldingRegister[4U] = 1U;
            if(true == bCalMode)
            {
                switch(modbusSlaveData.ModbusHoldingRegister[5U])
                {
                    case 1:
                        u16CalBuffer[0] = modbusSlaveData.ModbusHoldingRegister[6U];
                        u16CalBuffer[1] = (uint16_t)fTdVoltage;
                        break;
                    case 2:
                        u16CalBuffer[2] = modbusSlaveData.ModbusHoldingRegister[6U];
                        u16CalBuffer[3] = (uint16_t)fTdVoltage;
                        break;
                    default:
                        break;
                }
//                u16SlaveId = modbusSlaveData.ModbusHoldingRegister[7U];
            }
            else
            {
                modbusSlaveData.ModbusHoldingRegister[4U] = 0U;
                modbusSlaveData.ModbusHoldingRegister[5U] = 0U;
                modbusSlaveData.ModbusHoldingRegister[6U] = 0U;
//                modbusSlaveData.ModbusHoldingRegister[7U] = 0U;
            }
        }
        
        if(false == bCalMode)
        {
            if((fTdVoltage >= u16CalBuffer[1]) && (fTdVoltage <= u16CalBuffer[3]))
            {
                u16TdVal = map(fTdVoltage,u16CalBuffer[1],u16CalBuffer[3],u16CalBuffer[0],u16CalBuffer[2]);
            }
            else
            {
                if(fTdVoltage < u16CalBuffer[1])
                {
                    u16TdVal = u16CalBuffer[0];
                }
                else if(fTdVoltage > u16CalBuffer[3])
                {
                    u16TdVal = u16CalBuffer[2];
                }
            }
           
            /* move data into 485 after map*/
            modbusSlaveData.ModbusHoldingRegister[0U] = u16TdVal;/* <! Turbidity */
            modbusSlaveData.ModbusHoldingRegister[1U] = (uint16_t)fTdVoltage;/* <! Turbidity mV*/
            modbusSlaveData.ModbusHoldingRegister[2U] = (uint16_t)(tempNTC * 100.0f);/* <! NTC Temp */
            modbusSlaveData.ModbusHoldingRegister[3U] = (uint16_t)fNtcVoltage;/* <! NTC Temp mV */
#if 0
            modbusSlaveData.ModbusHoldingRegister[4U] = u16CalBuffer[0];/* <! Turbidity */
            modbusSlaveData.ModbusHoldingRegister[5U] = u16CalBuffer[1];/* <! Turbidity mV*/
            modbusSlaveData.ModbusHoldingRegister[6U] = u16CalBuffer[2];/* <! NTC Temp */
            modbusSlaveData.ModbusHoldingRegister[7U] = u16CalBuffer[3];/* <! NTC Temp mV */
#endif
        }
        else
        {
            modbusSlaveData.ModbusHoldingRegister[0U] = 0U;/* <! Turbidity */
            modbusSlaveData.ModbusHoldingRegister[1U] = (uint16_t)fTdVoltage;/* <! Turbidity mV*/
            modbusSlaveData.ModbusHoldingRegister[2U] = (uint16_t)(tempNTC * 100.0f);/* <! NTC Temp */
            modbusSlaveData.ModbusHoldingRegister[3U] = (uint16_t)fNtcVoltage;/* <! NTC Temp mV */            
        }
        
    }
}

/**
 End of File
*/