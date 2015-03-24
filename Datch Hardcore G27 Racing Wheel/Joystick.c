/*
             LUFA Library
     Copyright (C) Dean Camera, 2014.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2014  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the Joystick demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */

#include "Joystick.h"
#include "analog.h"
//#include "encoder.h"

int8_t xChange = 0;
float n=1;
int32_t wheel=0;
int16_t zAxis=0, zrAxis=0, slAxis=0, gearX=512, gearY=512;
volatile int16_t ffb=128;
int8_t* ffbreport;

void axesInit(){
	n=0.9;
	wheel=2048/n;
	zAxis=0;
	zrAxis=0;
	slAxis=0;
	
	};

/** Buffer to hold the previously generated HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevJoystickHIDReportBuffer[sizeof(USB_JoystickReport_Data_t)];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Joystick_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber              = INTERFACE_ID_Joystick,
				.ReportINEndpoint             =
					{
						.Address              = JOYSTICK_EPADDR,
						.Size                 = JOYSTICK_EPSIZE,
						.Banks                = 1,
					},
				.PrevReportINBuffer           = PrevJoystickHIDReportBuffer,
				.PrevReportINBufferSize       = sizeof(PrevJoystickHIDReportBuffer),
			},
	};

void InitPWM()
{
	//Set 8-bit fast PWM mode
	TCCR0A|=(1<<WGM00)|(1<<WGM01)|(1<<CS00)|(1<<COM0A1);
	
	TCCR0B = 1 << CS00; //Enable counter with no prescaling
	
	//Initialize port B as output
	DDRB|=(1 << PB7);
}

void SetPWMOutput(uint8_t duty)
{
	if(duty==0)PORTB&=~(1<<PB7);
	else OCR0A=duty;
}

//Прерывание для энкодера
ISR(INT2_vect){
	if((PIND & (1<<PD2))!= 0){
		if((PIND & (1<<PD1))!= 0) xChange++;
		else xChange--;
	}
	if((PIND & (1<<PD2))== 0){
		if((PIND & (1<<PD1))!= 0) xChange--;
		else xChange++;
	}
}
/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();
	axesInit();
	
	//Activate PWM
	InitPWM();
	
	//Activate Encoder
	//ENC_InitEncoder();
	
	//Прерывания энкодера
	EIMSK = ( 1 << INT2 );
	EICRA = (0<<ISC21) | (1<<ISC20);
	MCUCR |= (0<<ISC21)|(1<<ISC20);
	sei();
	
	GlobalInterruptEnable();

	for (;;)
	{
		HID_Device_USBTask(&Joystick_HID_Interface);
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

	/* Hardware Initialization */
	USB_Init();

	//Outputs
	DDRC|=(1<<PC7);
	DDRC|=(1<<PC6);
	DDRB|=(1<<PB1);
	DDRB|=(1<<PB6);
	DDRB|=(1<<PB5);
	DDRD|=(1<<PD0);

	//Inputs
	//DDRB&=~(1<<PB1);
	//DDRB&=~(1<<PB3);
	DDRB&=~(1<<PB4);
	DDRD&=~(1<<PD7);
	DDRD&=~(1<<PD6);
	DDRD&=~(1<<PD4);
	DDRD&=~(1<<PD3);
	DDRD&=~(1<<PD2);
	//DDRD&=~(1<<PD0);
	DDRF&=~(1<<PF7);
	DDRF&=~(1<<PF6);
	DDRF&=~(1<<PF5);
	DDRF&=~(1<<PF4);
	DDRF&=~(1<<PF1);
	DDRF&=~(1<<PF0);
	

}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Joystick_HID_Interface);

	USB_Device_EnableSOFEvents();
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	HID_Device_ProcessControlRequest(&Joystick_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Joystick_HID_Interface);
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
	int16_t setffb;
	uint8_t dpadLeftOn=0, dpadUpOn=0, dpadRightOn=0, dpadDownOn=0;
	USB_JoystickReport_Data_t* JoystickReport = (USB_JoystickReport_Data_t*)ReportData;

		
		//Задаём значение поворота руля
		wheel = wheel + xChange;
		xChange = 0;//Обнуляем изменение
		
		
		setffb=ffb;//Получаем отдачу
		if(setffb==128||setffb==127)setffb=128;//Выравниваем центр
		
		
		//((wheel*n)>4095) ? 4095 : (((wheel*n)<0) ? 0 : wheel*n)
		//Проверяем не повёрнут ли руль до упора
		
		if((wheel*n)<127){
			setffb = 255-wheel*n;
			if(setffb>255)setffb = 255;
		}
		if((wheel*n)>4095-127){
		setffb = 4095-wheel*n;
		if(setffb<0)setffb = 0;
		}
		
		//Разбираем силу и направление вращения
		//задаём порты для выбора направления вращения
		if(setffb>128){
			PORTB|=(1<<PB1);
			PORTD&=~(1<<PD0);
			setffb = (setffb-128)*2;
		}
		else{
			if(setffb==128){
				PORTB&=~(1<<PB1);
				PORTD&=~(1<<PD0);
				setffb = 1;
			}
			else{
				PORTB&=~(1<<PB1);
				PORTD|=(1<<PD0);
				setffb = (127-setffb)*2;
			}
		}
		SetPWMOutput(setffb);
		
		zAxis = adc_read(PF7);
		zrAxis= adc_read(PF6);
		slAxis= adc_read(PF5);
		gearX= adc_read(PF1);
		gearY= adc_read(PF4);
		
		//Отправка осей
		JoystickReport->x_axis = ((wheel*n)>4095) ? 4095 : (((wheel*n)<0) ? 0 : wheel*n);
		JoystickReport->z_axis = (zAxis>250 ? (zAxis<900 ? (zAxis-250) :650) : 0)*6.3;
		JoystickReport->zr_axis= 4095-(zrAxis>200 ? (zrAxis<900 ? (zrAxis-200) : 700): 0)*5.85;
		JoystickReport->sl_axis= (slAxis>204 ? (slAxis-204) : 0)*5;
		JoystickReport->y_axis = 2048+(JoystickReport->z_axis/2)-(JoystickReport->zr_axis/2);
		
		//Проверка портов и отправка кнопок
		PORTC|=(1<<PC7);
		PORTC|=(1<<PC6);
		PORTB|=(1<<PB6);
		PORTB|=(1<<PB5);
		
		//выкл кроме PORTC&=~(1<<PC7);
		PORTC&=~(1<<PC6);
		PORTB&=~(1<<PB6);
		PORTB&=~(1<<PB5);
		Delay_MS(1);
		if((PINB&(1<<PB4))!=0) JoystickReport->_1btn = 1;
		else JoystickReport->_1btn = 0;
		if((PIND&(1<<PD7))!=0) JoystickReport->_2btn = 1;
		else JoystickReport->_2btn = 0;
		if((PIND&(1<<PD6))!=0) JoystickReport->_3btn = 1;
		else JoystickReport->_3btn = 0;
		if((PIND&(1<<PD4))!=0) JoystickReport->_5btn = 1;
		else JoystickReport->_5btn = 0;
		//все вкл 
		PORTC|=(1<<PC6);
		PORTB|=(1<<PB6);
		PORTB|=(1<<PB5);
		
		//выкл кроме PORTC&=~(1<<PC6);
		PORTC&=~(1<<PC7);
		PORTB&=~(1<<PB6);
		PORTB&=~(1<<PB5);
		Delay_MS(1);
		if((PINB&(1<<PB4))!=0) JoystickReport->_4btn = 1;
		else JoystickReport->_4btn = 0;
		if((PIND&(1<<PD7))!=0) JoystickReport->_16btn = 1;
		else JoystickReport->_16btn = 0;
		if((PIND&(1<<PD6))!=0) JoystickReport->_17btn = 1;
		else JoystickReport->_17btn = 0;
		if((PIND&(1<<PD4))!=0) JoystickReport->_6btn = 1;
		else JoystickReport->_6btn = 0;
		//все вкл
		PORTC|=(1<<PC7);
		PORTB|=(1<<PB6);
		PORTB|=(1<<PB5);
		
		//выкл кроме PORTB&=~(1<<PB6);
		PORTC&=~(1<<PC7);
		PORTC&=~(1<<PC6);
		PORTB&=~(1<<PB5);
		Delay_MS(1);
		if((PINB&(1<<PB4))!=0) JoystickReport->_18btn = 1;
		else JoystickReport->_18btn = 0;
		if((PIND&(1<<PD7))!=0) JoystickReport->_19btn = 1;
		else JoystickReport->_19btn = 0;
		if((PIND&(1<<PD6))!=0) dpadLeftOn = 1;
		else dpadLeftOn = 0;
		if((PIND&(1<<PD4))!=0) JoystickReport->_7btn = 1;
		else JoystickReport->_7btn = 0;
		//все вкл
		PORTC|=(1<<PC7);
		PORTC|=(1<<PC6);
		PORTB|=(1<<PB5);
		
		//выкл кроме PORTB&=~(1<<PB5);
		PORTC&=~(1<<PC7);
		PORTC&=~(1<<PC6);
		PORTB&=~(1<<PB6);
		Delay_MS(1);
		if((PINB&(1<<PB4))!=0) dpadUpOn = 1;
		else dpadUpOn = 0;
		if((PIND&(1<<PD7))!=0) dpadRightOn = 1;
		else dpadRightOn = 0;
		if((PIND&(1<<PD6))!=0) dpadDownOn = 1;
		else dpadDownOn = 0;
		if((PIND&(1<<PD4))!=0) JoystickReport->_8btn = 1;
		else JoystickReport->_8btn = 0;
		//все вкл
		PORTC|=(1<<PC7);
		PORTC|=(1<<PC6);
		PORTB|=(1<<PB6);
		
		//Отправка КПП
		
		//1st gear
		if(gearY>690 && gearX>585) JoystickReport->_9btn=1;
		else JoystickReport->_9btn=0;
		//2nd gear
		if(gearY<615 && gearX>585) JoystickReport->_10btn=1;
		else JoystickReport->_10btn=0;
		//3rd gear
		if(gearY>690 && gearX<585 && gearX>530) JoystickReport->_11btn=1;
		else JoystickReport->_11btn=0;
		//4th gear
		if(gearY<615 && gearX<585 && gearX>530) JoystickReport->_12btn=1;
		else JoystickReport->_12btn=0;
		//5th gear
		if(gearY>690 && gearX<530 && gearX>480) JoystickReport->_13btn=1;
		else JoystickReport->_13btn=0;
		//6th gear
		if(gearY<615 && gearX<530 && gearX>480) JoystickReport->_14btn=1;
		else JoystickReport->_14btn=0;
		//Reverse gear
		if(gearY<615 && gearX<480) JoystickReport->_15btn=1;
		else JoystickReport->_15btn=0;
		
		//Отправка dPad
		JoystickReport->direction = 8;
		if (dpadUpOn == 1){
			if (dpadLeftOn == 1){
				JoystickReport->direction = 7;
			}
			else if (dpadRightOn == 1){
				JoystickReport->direction = 1;
			}
			else
			JoystickReport->direction = 0;
		}
		else if (dpadDownOn == 1){
			if (dpadLeftOn == 1){
				JoystickReport->direction = 5;
			}
			else if (dpadRightOn == 1){
				JoystickReport->direction = 3;
			}
			else
			JoystickReport->direction = 4;
		}
		else if (dpadLeftOn == 1){
			JoystickReport->direction = 6;
		}
		else if (dpadRightOn == 1){
			JoystickReport->direction = 2;
		}
		
		
		
		*ReportSize = sizeof(USB_JoystickReport_Data_t);
		return true;
  }


/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
	// Unused (but mandatory for the HID class driver) in this demo, since there are no Host->Device reports
	
	ffbreport = ReportData;
	
	if(ffbreport[0]==0x11)ffb = ffbreport[2];
	if(ffbreport[0]==0x13)ffb = 128;//Обнуляем отдачу
}

