#include <stdint.h>
#include <stdbool.h>
#include "tm4c1290nczad.h"
#include "inc/hw_gpio.h"
#include "inc/hw_flash.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ssi.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/flash.h"
#include "bl_packet.h"
#include "driverlib/debug.h"                // Macros for assisting debug of the driver library
#include "driverlib/sysctl.h"               // Defines and macros for System Control API of DriverLib
#include "driverlib/interrupt.h"            // Defines and macros for NVIC Controller API of DriverLib
#include "driverlib/timer.h"                // Defines and macros for Timer API of driverLib
#include "driverlib/gpio.h"                 // Defines and macros for GPIO API of DriverLib
#include "driverlib/pin_map.h"              // Mapping of peripherals to pins for all parts
#include "driverlib/uart.h"                 // Defines and Macros for the UART
#include "driverlib/rom.h"                  // Defines and macros for ROM API of driverLib
#include "driverlib/rom_map.h"

#define UART_TIME_OUT 1000 // 1 second time out
extern volatile uint32_t msTicks;
uint32_t ui32ProcRatio;
uint32_t g_ui32SysClock;

#define RECV_BUFFER_SIZE 512                //接收最大字节512
unsigned char data_buf[RECV_BUFFER_SIZE];   //定义一个接受区

#define COMMAND_DOWNLOAD        0x21

#define COMMAND_RUN             0x22

#define COMMAND_GET_STATUS      0x23


#define COMMAND_SEND_DATA       0x24

#define COMMAND_RESET           0x25

#define COMMAND_RET_SUCCESS     0x40

#define COMMAND_RET_UNKNOWN_CMD 0x41

#define COMMAND_RET_INVALID_CMD 0x42

#define COMMAND_RET_INVALID_ADR 0x43

#define COMMAND_RET_FLASH_FAIL  0x44



#define COMMAND_RET_CRC_FAIL    0x45


#define COMMAND_ACK             0xcc

#define COMMAND_NAK             0x33

//Enable UART0 clock
    void UartConfigure()
    {
        g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                         SYSCTL_CFG_VCO_480),80000000);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//uart0对应的外设引脚为PA0，PA1
        //设置PA0，PA1为uart引脚
        GPIOPinConfigure(GPIO_PA0_U0RX);
        GPIOPinConfigure(GPIO_PA1_U0TX);
        GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
        //设置波特率115200，数据位 8 ，校验位 None ，停止位 1 ，8——N——1模式
        UARTConfigSetExpClk(UART0_BASE, g_ui32SysClock, 115200,
                                (UART_CONFIG_WLEN_8 |  UART_CONFIG_STOP_ONE |
                                 UART_CONFIG_PAR_NONE));
        //开启uart0中断
        IntEnable(INT_UART0);
        UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);

    }

    void UARTReceive(uint8_t *pui8Data, uint32_t ui32Size)
    {
        //
        // Send out the number of bytes requested.
        //
        while(ui32Size--)
        {
            //
            // Wait for the FIFO to not be empty.
            //
            while((HWREG(UART0_BASE + UART_O_FR) & UART_FR_RXFE))
            {
            }

            //
            // Receive a byte from the UART.
            //
            *pui8Data++ = HWREG(UART0_BASE + UART_O_DR);
        }
    }
    void UARTFlush(void)
    {
        //
        // Wait for the UART FIFO to empty and then wait for the shifter to get the
        // bytes out the port.
        //
        while(!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_TXFE))
        {
        }

        //
        // Wait for the FIFO to not be busy so that the shifter completes.
        //
        while((HWREG(UART0_BASE + UART_O_FR) & UART_FR_BUSY))
        {
        }
    }

    void UARTSend(const uint8_t *pui8Data, uint32_t ui32Size)
    {
        //
        // Transmit the number of bytes requested on the UART port.
        //
        while(ui32Size--)
        {
            //
            // Make sure that the transmit FIFO is not full.
            //
            while((HWREG(UART0_BASE + UART_O_FR) & UART_FR_TXFF))
            {

            }

            //
            // Send out the next byte.
            //
            HWREG(UART0_BASE + UART_O_DR) = *pui8Data++;
        }

        //
        // Wait until the UART is done transmitting.
        //
        UARTFlush();
    }
int main(void)
{
    UartConfigure();
    while(1)
    {
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);
        uint8_t a[2] = {0x11,0x22};
        UARTSend(a,2);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0);
    }
}
