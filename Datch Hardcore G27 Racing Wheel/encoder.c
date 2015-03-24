/*#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "encoder.h"

#define SetBit(port, bit) port|= (1<<bit)
#define ClearBit(port, bit) port&= ~(1<<bit)

//��� ��� ����������� ����
#define b00000011 3
#define b11010010 210
#define b11100001 225

volatile int bufEnc = 0; //����� ��������

//������� �������������
//__________________________________________
void ENC_InitEncoder(void)
{
  ClearBit(DDRB, PB3); //����
  ClearBit(DDRB, PB1);
  SetBit(PORTB, PB3);//��� ������������� ��������
  SetBit(PORTB, PB1);
}

//������� ������ ��������
//___________________________________________
int ENC_PollEncoder(void)
{
static unsigned char stateEnc; 	//������ ������������������ ��������� ��������
unsigned char tmp;  
unsigned char currentState = 0;

//��������� ��������� ������� ����������������
if ((PINB & (1<<PB3))!= 0) {SetBit(currentState,0);}
if ((PINB & (1<<PB1))!= 0) {SetBit(currentState,1);}

//���� ����� �����������, �� �������
tmp = stateEnc;
if (currentState == (tmp & b00000011)) return 0;

//���� �� �����, �� �������� � ��������� � ���
tmp = (tmp<<2)|currentState;
stateEnc = tmp;

//���������� ������������ ������������������
if (tmp == b11100001) return  1;
if (tmp == b11010010) return -1;
return 0;
}

//������� ������������ �������� ������ ��������
//_____________________________________________
int ENC_GetStateEncoder(void)
{
  int tmp = bufEnc;
  bufEnc = 0;
  return tmp;
}


*/