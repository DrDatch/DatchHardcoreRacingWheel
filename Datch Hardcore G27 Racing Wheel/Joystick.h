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
 *  Header file for Joystick.c.
 */

#ifndef _JOYSTICK_H_
#define _JOYSTICK_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/wdt.h>
		#include <avr/power.h>
		#include <avr/interrupt.h>
		#include <string.h>

		#include "Descriptors.h"

		#include <LUFA/Drivers/Board/Joystick.h>
		#include <LUFA/Drivers/Board/LEDs.h>
		#include <LUFA/Drivers/Board/Buttons.h>
		#include <LUFA/Drivers/USB/USB.h>
		#include <LUFA/Platform/Platform.h>

	/* Type Defines: */
		/** Type define for the joystick HID report structure, for creating and sending HID reports to the host PC.
		 *  This mirrors the layout described to the host in the HID report descriptor, in Descriptors.c.
		 */
		typedef struct
		{
			// digital buttons, 0 = off, 1 = on

			uint8_t _1btn : 1;
			uint8_t _2btn : 1;
			uint8_t _3btn : 1;
			uint8_t _4btn : 1;

			uint8_t _5btn : 1;
			uint8_t _6btn : 1;
			uint8_t _7btn : 1;
			uint8_t _8btn : 1;

			uint8_t _9btn : 1;
			uint8_t _10btn : 1;
			uint8_t _11btn : 1;
			uint8_t _12btn : 1;
			
			uint8_t _13btn : 1;
			uint8_t _14btn : 1;
			uint8_t _15btn : 1;
			uint8_t _16btn : 1;
			
			uint8_t _17btn : 1;
			uint8_t _18btn : 1;
			uint8_t _19btn : 1;
			
			uint8_t : 5;

			// digital direction, use the dir_* constants(enum)
			// 8 = center, 0 = up, 1 = up/right, 2 = right, 3 = right/down
			// 4 = down, 5 = down/left, 6 = left, 7 = left/up

			uint8_t direction;

			// left and right analog sticks, 0x00 left/up, 0x80 middle, 0xff right/down

			uint16_t x_axis;
			uint16_t y_axis;
			uint16_t z_axis;
			uint16_t zr_axis;
			uint16_t sl_axis;
		} USB_JoystickReport_Data_t;

	/* Macros: */

	/* Function Prototypes: */
		void SetupHardware(void);

		void EVENT_USB_Device_Connect(void);
		void EVENT_USB_Device_Disconnect(void);
		void EVENT_USB_Device_ConfigurationChanged(void);
		void EVENT_USB_Device_ControlRequest(void);
		void EVENT_USB_Device_StartOfFrame(void);

		bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
		                                         uint8_t* const ReportID,
		                                         const uint8_t ReportType,
		                                         void* ReportData,
		                                         uint16_t* const ReportSize);
		void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
		                                          const uint8_t ReportID,
		                                          const uint8_t ReportType,
		                                          const void* ReportData,
		                                          const uint16_t ReportSize);

#endif

