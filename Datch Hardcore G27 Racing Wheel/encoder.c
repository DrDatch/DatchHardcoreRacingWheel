/*#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "encoder.h"

#define SetBit(port, bit) port|= (1<<bit)
#define ClearBit(port, bit) port&= ~(1<<bit)

//это дл€ нагл€дности кода
#define b00000011 3
#define b11010010 210
#define b11100001 225

volatile int bufEnc = 0; //буфер энкодера

//функци€ инициализации
//__________________________________________
void ENC_InitEncoder(void)
{
  ClearBit(DDRB, PB3); //вход
  ClearBit(DDRB, PB1);
  SetBit(PORTB, PB3);//вкл подт€гивающий резистор
  SetBit(PORTB, PB1);
}

//функци€ опроса энкодера
//___________________________________________
int ENC_PollEncoder(void)
{
static unsigned char stateEnc; 	//хранит последовательность состо€ний энкодера
unsigned char tmp;  
unsigned char currentState = 0;

//провер€ем состо€ние выводов микроконтроллера
if ((PINB & (1<<PB3))!= 0) {SetBit(currentState,0);}
if ((PINB & (1<<PB1))!= 0) {SetBit(currentState,1);}

//если равно предыдущему, то выходим
tmp = stateEnc;
if (currentState == (tmp & b00000011)) return 0;

//если не равно, то сдвигаем и сохран€ем в озу
tmp = (tmp<<2)|currentState;
stateEnc = tmp;

//сравниваем получившуюс€ последовательность
if (tmp == b11100001) return  1;
if (tmp == b11010010) return -1;
return 0;
}

//функци€ возвращающа€ значение буфера энкодера
//_____________________________________________
int ENC_GetStateEncoder(void)
{
  int tmp = bufEnc;
  bufEnc = 0;
  return tmp;
}


*/