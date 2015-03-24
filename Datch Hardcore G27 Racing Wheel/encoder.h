#ifndef	encoder_h
#define	encoder_h
//_________________________________________
/*//порт и выводы к которым подключен энкодер
#define PORT_Enc 	PORTF
#define PIN_Enc 	PINF
#define DDR_Enc 	DDRF
#define Pin1_Enc 	PF1
#define Pin2_Enc 	PF0
//______________________
#define RIGHT_SPIN 1 
#define LEFT_SPIN -1*/

void ENC_InitEncoder(void);
int ENC_PollEncoder(void);
int ENC_GetStateEncoder(void);
#endif  //encoder_h
