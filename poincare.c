/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== spiloopback.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>

/* Example/Board Header files */
#include "Board.h"

/* Custom header files */
#include "ecg2_hal.h"
#include "ecg2_hw.h"

// Compiler constants
#define SPI_MSG_LENGTH    26
#define TASKSTACKSIZE     768

// Local functions protetypes
double calculate_ecg_channel( unsigned char *buffer, unsigned short index, double refv, double gain, double offset_voltage );
void setup_ecg(void);

/* Allocate buffers in .dma section of memory for concerto devices */
#ifdef MWARE
#pragma DATA_SECTION(g_spi_RxBuffer, ".dma");
#pragma DATA_SECTION(g_spi_TxBuffer, ".dma");
//#pragma DATA_SECTION(slaveRxBuffer, ".dma");
//#pragma DATA_SECTION(slaveTxBuffer, ".dma");
#endif


// Task related elements
Task_Struct task0Struct, task1Struct;
Char task0Stack[TASKSTACKSIZE], task1Stack[TASKSTACKSIZE];

// Semaphores
Semaphore_Struct adc_drdy_sem_struct;
Semaphore_Params sem_params;
Semaphore_Handle adc_drdy_sem_handle;

// specific ADC constants
const num_bytes_sample = 19;       // number of bytes in ADS1194 sample in data continuous mode
const double channel_gain = 20.00; // amplifier gain
const double vref = 2400.00;       // reference voltage in millivolts

// variables for ADC
unsigned char ecg_data_sample[num_bytes_sample];  // one sample data from ADS1194
double channel1_voltage; // channel 1 millivolts
double channel2_voltage; // channel 2 millivolts
double channel3_voltage; // channel 3 millivolts
double channel4_voltage; // channel 4 millivolts
double channel1_voltage_offset; // channel 1 offset millivolts
double channel2_voltage_offset; // channel 2 offset millivolts
double channel3_voltage_offset; // channel 3 offset millivolts
double channel4_voltage_offset; // channel 4 offset millivolts
UInt32 g_time_ticks;	// used to store some events time (in clock ticks)


// SPI related variables
SPI_Handle g_spi0_handle;
unsigned char g_spi_RxBuffer[SPI_MSG_LENGTH];
unsigned char g_spi_TxBuffer[SPI_MSG_LENGTH];


/* *  ======== slaveTaskFxn ========
 *  Task function for slave task.
 */
Void slaveTaskFxn (UArg arg0, UArg arg1)
{
	while(1){
		;
	}
}
/*
 *  ======== masterTaskFxn ========
 *  Task function for master task.
 */
Void masterTaskFxn (UArg arg0, UArg arg1)
{
	SPI_Params spi_params;
	SPI_Transaction spi_transaction;
    UART_Params uartParams;
    UART_Handle uart_handle;

    char final_string[20];
    char time_string[20];
    double time_value = 0.0;
    int size;

	// Print some system information

    //--------------------------------------------------------------------
    // Initialize SPI handle as  master
    SPI_Params_init(&spi_params);
    spi_params.bitRate = 4000000;
    spi_params.mode =  SPI_MASTER;
    spi_params.transferMode = SPI_MODE_BLOCKING;
    spi_params.frameFormat = SPI_POL0_PHA1;// SCLK steady zero, data capture on second clock transition
    g_spi0_handle = SPI_open(Board_SPI0, &spi_params);
    System_printf("SPI Data size: %d\n", spi_params.dataSize);
    if (g_spi0_handle == NULL) {
        System_abort("Error initializing SPI\n");
    }
    else {
        System_printf("SPI initialized\n");
    }

    System_flush();

    // Initialize UART
    UART_Params_init(&uartParams);
    uartParams.readMode = UART_MODE_BLOCKING;
    uartParams.writeMode = UART_MODE_BLOCKING;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_NEWLINE;
    uartParams.baudRate = 9600;
    uart_handle = UART_open(EK_TM4C1294XL_UART0, &uartParams);

    // ret = UART_write(uart_handle, (uint8_t *)buffer, size);
    //



    //--------------------------------------------------------------------
    // Initialize and configure the ECG2 ADC
    // issue RESET pulse
    GPIO_write(ADC_RTS, 0);
    User_delay_us(100);
    GPIO_write(ADC_RTS, 1);
    Task_sleep(1000);

    // device is in RDATAC mode, set it to SDATAC mode to edit registers
    ecg2_hal_send_command(SDATAC_COMMAND);
    setup_ecg();

    // Start continuosly reading from the ECG ADC
    while (1)
    {
    	// Wait for ADS1194 device to prepare output data. Data is ready every 8 milliseconds
        GPIO_enableInt(ADC_DRDY);
        Semaphore_pend(adc_drdy_sem_handle, BIOS_WAIT_FOREVER);// Wait for ADS1194 device to prepare output data.
        GPIO_disableInt(ADC_DRDY);

        User_delay_us(5);
        // TODO: Use SPI_transfer function (done)
        spi_transaction.count = num_bytes_sample;
        spi_transaction.rxBuf = ecg_data_sample;
        spi_transaction.txBuf = NULL;
        SPI_transfer(g_spi0_handle, &spi_transaction);
        //for (i=0; i<num_bytes_sample; i++)  // read ADS1194 output data, one sample
        //ecg_data_sample[i] = SPI_Read(0);

        //time_value += 8.0;
        //time_value = (double) g_time_ticks*1.0e-3;

        // calculate input voltage
        // voltage LL RA - channel 2 is usually used for simple ECG
        channel1_voltage = calculate_ecg_channel( ecg_data_sample, 3, vref, channel_gain, channel1_voltage_offset );
        channel2_voltage = calculate_ecg_channel( ecg_data_sample, 5, vref, channel_gain, channel2_voltage_offset );
        sprintf(final_string, "%.2f", channel2_voltage);  // convert values to string and send to MikroPlot
        strcat(final_string, ",");
        //sprintf(time_string, "%.2f", time_value);
        sprintf(time_string, "%ul", g_time_ticks);
        strcat(final_string, time_string);
        strcat(final_string, "\n");
        size = strlen(final_string);
        UART_write(uart_handle, (uint8_t *)final_string, size);
        //Uart_Write_Text(final_string);
        //Uart_Write_Text("\r\n");

        // voltage LL LA
        channel3_voltage = calculate_ecg_channel( ecg_data_sample, 7, vref, channel_gain, channel3_voltage_offset );

        // voltage from temperature sensor
        channel4_voltage = calculate_ecg_channel( ecg_data_sample, 9, vref, channel_gain, channel4_voltage_offset );
    }
}

/*
 *  ======== gpioADC_DRDYFxn ========
 *  Callback function for the GPIO interrupt on ADC_DRDY input.
 */
void gpioADC_DRDYFxn(unsigned int index)
{
	g_time_ticks = Clock_getTicks();
    /* Clear the GPIO interrupt and toggle an LED */
    GPIO_toggle(Board_LED1);
    Semaphore_post(adc_drdy_sem_handle);
}

//--------------------------------------------------------------------
// Calculate voltage for an ECG ADC channel
double calculate_ecg_channel( unsigned char *buffer, unsigned short index, double refV, double gain, double offset_voltage )
{
    int adc_value = 0;
    adc_value = 0;
    adc_value = buffer[index];
    adc_value <<= 8;
    adc_value |= buffer[index + 1];
    return ( ((double)adc_value * (refV / (32768 - 1))) / gain ) - offset_voltage;
}

//--------------------------------------------------------------------
// ECG2 configuration function
void setup_ecg(void)
{
    uint8_t tempctr = 0;
    SPI_Transaction spi_transaction;


    ecg2_hal_send_command(SDATAC_COMMAND);
    ecg2_oscillator_clock_enable(true);

    //no test signal, default value
    ecg2_set_output_data_rate(SPS_125);
    ecg2_set_test_source(TEST_SOURCE_EXTERNAL);

    // RDL internal and enabled, reference voltage is 2.4V, RLD signal source is internal
    ecg2_power_down_reference_buffer_enable(true);
    ecg2_set_reference_voltage(VREF_2_4V);
    ecg2_rld_measurement_enable(true);
    ecg2_rldref_source_select(RLDEF_SIGNAL_INTERNAL);
    ecg2_rld_buffer_enable(true);

    // lead-off is in DC mode and using pull up and down resistors, comparator thresholds are set to 70% and 30%
    // seting LOFF register
    ecg2_lead_off_comparator_threshold_set(POSITIVE_70);
    ecg2_vlead_off_enable(false);
    ecg2_ilead_off_magnitude_set(NA_4);
    ecg2_flead_off_frequency_set(3);

    // channel 1 settings register
    // channel is on and gain is 12, input shorted for offset measurements
    ecg2_configure_channel(1, false, 0, 1);

    // channel 2 settings register
    // channel is on and gain is 12, input shorted for offset measurements
    ecg2_configure_channel(2, false, 0, 1);

    // channel 3 settings register
    // channel is on and gain is 12, input shorted for offset measurements
    ecg2_configure_channel(3, false, 0, 1);

    // channel 4 settings register
    // channel is on and gain is 12, input shorted for offset measurements
    ecg2_configure_channel(4, false, 0, 1);

    // channel 2 is used for RDL
    ecg2_right_leg_positive_drive_set(2);

    // channel 2 is used for RDL
    ecg2_right_leg_negative_drive_set(2);

    // channel 3P uses pull-up resistor to detect LL lead-off, channel 1P uses pull-up resistor to detect LA lead-off,
    ecg2_lead_off_positive_channel_select(5);

    // channel 2N uses pull-down resistor to detect RA lead-off
    ecg2_lead_off_negative_channel_select(2);

    // no flip
    ecg2_lead_off_current_direction_select(0);

    // continuous conversion mode, WCT not connected to RLD, LOFF comparators enabled
    ecg2_lead_off_comparator_enable(true);

    // activate conversion to read and calculate offset
    ecg2_hal_send_command(START_COMMAND); // send START command
    User_delay_us(100);
    ecg2_hal_send_command(RDATA_COMMAND); // enable read data once
    GPIO_write(ADC_CS, 0); // chip select
    User_delay_us(1);

    GPIO_enableInt(ADC_DRDY); //
    Semaphore_pend(adc_drdy_sem_handle, BIOS_WAIT_FOREVER);// Wait for ADS1194 device to prepare output data.
    GPIO_disableInt(ADC_DRDY);
    User_delay_us(5);

    // TODO: Use SPI_transfer function
    spi_transaction.count = num_bytes_sample;
    spi_transaction.rxBuf = ecg_data_sample;
    spi_transaction.txBuf = NULL;
    SPI_transfer(g_spi0_handle, &spi_transaction);
    //for (tempctr = 0; tempctr < num_bytes_sample; tempctr++){
    //	ecg_data_sample[tempctr] = SPI_Read(0);	// read ADS1194 output data, one sample
    //}

    // calculate offset
    // offset voltage LA RA
    channel1_voltage_offset = calculate_ecg_channel( ecg_data_sample, 3, vref, channel_gain, 0 );

    // offset voltage LL RA
    channel2_voltage_offset = calculate_ecg_channel( ecg_data_sample, 5, vref, channel_gain, 0 );

    // offset voltage LL LA
    channel3_voltage_offset = calculate_ecg_channel( ecg_data_sample, 7, vref, channel_gain, 0 );

    // offset voltage
    channel4_voltage_offset = calculate_ecg_channel( ecg_data_sample, 9, vref, channel_gain, 0 );
    GPIO_write(ADC_CS, 1);
    User_delay_us(10);

    // stop conversion
    ecg2_hal_send_command(STOP_COMMAND); // send STOP command
    User_delay_us(100);
    ecg2_hal_send_command(SDATAC_COMMAND); // SDATAC mode

    // activate conversion
    // channel is on and gain is 12, normal electrode input
    ecg2_configure_channel(1, false, 6, 0);

    // channel is on and gain is 12, normal electrode input
    ecg2_configure_channel(2, false, 6, 0);

    // channel is on and gain is 12, normal electrode input
    ecg2_configure_channel(3, false, 6, 0);

    // channel is on and gain is 12, temperature sensor
    ecg2_configure_channel(4, false, 6, 4);

    ecg2_hal_send_command(START_COMMAND); // send START command
    User_delay_us(100);
    ecg2_hal_send_command(RDATAC_COMMAND); // enable read data in continuous mode
    GPIO_write(ADC_CS, 0); // chip select to continuosly clock out data
    User_delay_us(1);
}


/*
 *  ======== main ========
 */
int main(void)
{
    /* Construct BIOS objects */
    Task_Params taskParams;

    /* Call board init functions. */
    Board_initGeneral();
    Board_initGPIO();
    Board_initSPI();

    /* Construct master/slave Task threads */
    Task_Params_init(&taskParams);
    taskParams.priority = 1;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)masterTaskFxn, &taskParams, NULL);

    taskParams.stack = &task1Stack;
    taskParams.priority = 2;
    Task_construct(&task1Struct, (Task_FuncPtr)slaveTaskFxn, &taskParams, NULL);

    //
    /* install ADC_DRDY callback */
    GPIO_setCallback(ADC_DRDY, gpioADC_DRDYFxn);

    // Semaphore posting events from ADC_DRDY callback
    Semaphore_Params_init(&sem_params);
    sem_params.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&adc_drdy_sem_struct, 0, &sem_params);
    adc_drdy_sem_handle = Semaphore_handle(&adc_drdy_sem_struct);


    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    /* Start BIOS */
    BIOS_start();

    return (0);
}

