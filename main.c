#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x.h"
#include "misc.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "stm32f10x_tim.h"
#define BUFFERSIZE 512
#define LEFT_COUNT()            TIM2->CNT
#define RIGHT_COUNT()           TIM4->CNT
char czlig[5] = "zlig";
char czwyl[5] = "zwyl";
char ckont[5] = "kont";
char chelp[5] = "help";
char czlid[5] = "zlid";
char cpwm[5] = "pwm=";
char cpwm2[6] = "pwm2=";
char cpuls[6] = "puls=";
char cstop[5] = "stop";
char cdwyl[5] = "dwyl";
char ckwad[5] = "kwad";
char cipwmin[7] = "ipwmin";
char cpwminput[9] = "pwminput";
char ckwaread[8] = "kwaread";
__IO float IC2Value = 0;
__IO float DutyCycle = 0;
__IO float Frequency = 0;
unsigned int kanal, pwmwartosckanalu, pwmwartoscokres, pwmwartoscdzielnika,
		mojq = 0, mojw = 0, moje = 0, skok = 0;
unsigned long int konter3, konter5;
unsigned char konterW3[6] = ("0\0"), konterW5[6] = ("0\0"), wypelnienie[6] , czestotliwosc[8] , fwd[8] = ("0\0");
volatile uint8_t rx_buffer[BUFFERSIZE], tx_buffer[BUFFERSIZE];
volatile uint16_t rx_wr_index = 0, rx_rd_index = 0, tx_wr_index = 0,
		tx_rd_index = 0;
volatile char command_line[24];
uint8_t pwmdioda[] = { "pwm=" },pwmdioda2[] = { "pwm2="}, kpuls[] = { "puls=" }, kstop[] = { "stop" },
		pwmokres[5], pwmkanal[5], pwmdzielnikzegara[8];
int pomoc = 0;
uint8_t prze = 0, pul = 0, pwu = 0, pwk = 0, zbi = 0, pwin = 0;
void usart();
void czyszczenie();
void parse();
void szukaniekomendy();
void sendString();
void pwminput();
void pwm();
void zwyklylicznik();
void zliczanieD();
void zliczanieU();
void stop();
void dwylacz();
void kwadratura1();
void jedenpuls();
void cut_command_line(int a);

void TIM2_IRQHandler() {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		prze = 1;
	}
	if (TIM_GetITStatus(TIM2, TIM_OPMode_Single) == SET) {
		TIM_ClearITPendingBit(TIM2, TIM_OPMode_Single);
		pul = 1;
	}
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		zbi = 1;
	}
}

void TIM3_IRQHandler() {
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	pwin = 1;
}
void jedenpuls() {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	 TIM_OCInitTypeDef TIM_OCInitStructure;
	 NVIC_InitTypeDef nvic;
	 GPIO_InitTypeDef GPIO_InitStructure;

	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	 GPIO_StructInit(&GPIO_InitStructure);
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	 GPIO_Init(GPIOA, &GPIO_InitStructure);

	 TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	 TIM_TimeBaseStructure.TIM_Prescaler = 64000 - 1;
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	 TIM_TimeBaseStructure.TIM_Period = pwmwartoscokres - 1;
	 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	 TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	 TIM_OCStructInit(&TIM_OCInitStructure);
	 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	 TIM_OCInitStructure.TIM_Pulse = pwmwartosckanalu;
	 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	 TIM_OC1Init(TIM2, &TIM_OCInitStructure);

	 TIM_ITConfig(TIM2, TIM_OPMode_Single | TIM_IT_CC1, ENABLE);
	 TIM_SelectOnePulseMode(TIM2, TIM_OPMode_Single);
	 TIM_Cmd(TIM2, ENABLE);

	 nvic.NVIC_IRQChannel = TIM2_IRQn;
	 nvic.NVIC_IRQChannelPreemptionPriority = 0;
	 nvic.NVIC_IRQChannelSubPriority = 0;
	 nvic.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&nvic);
}


void zliczanieU() {
	TIM_TimeBaseInitTypeDef tim;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseStructInit(&tim);
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Prescaler = 64000;
	tim.TIM_Period = 10000;
	tim.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &tim);
	TIM_Cmd(TIM3, ENABLE);
}
void stop() {
	TIM_Cmd(TIM2, DISABLE);
	TIM_Cmd(TIM3, DISABLE);
	TIM_Cmd(TIM4, DISABLE);
	dwylacz();
}
void dwylacz() {
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);
}
void zliczanieD() {
	TIM_TimeBaseInitTypeDef tim;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructInit(&tim);
	tim.TIM_CounterMode = TIM_CounterMode_Down;
	tim.TIM_Prescaler = 64000;
	tim.TIM_Period = 10000;
	tim.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &tim);
	TIM_Cmd(TIM2, ENABLE);
}

void zwyklylicznik() {
	GPIO_InitTypeDef gpio;
	TIM_TimeBaseInitTypeDef tim;
	NVIC_InitTypeDef nvic;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_6;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &gpio);

	TIM_TimeBaseStructInit(&tim);
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Prescaler = 64000 - 1;
	tim.TIM_Period = 2000 - 1;
	TIM_TimeBaseInit(TIM2, &tim);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
}

void pwm() {
	GPIO_InitTypeDef gpio;
	TIM_TimeBaseInitTypeDef tim;
	TIM_OCInitTypeDef channel;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_6;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);

	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Prescaler = pwmwartoscdzielnika - 1;
	tim.TIM_Period = pwmwartoscokres;
	tim.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM4, &tim);

	channel.TIM_OCMode = TIM_OCMode_PWM1;
	channel.TIM_OutputState = TIM_OutputState_Enable;
	channel.TIM_OCPolarity = TIM_OCPolarity_High;
	channel.TIM_Pulse = pwmwartosckanalu;
	TIM_OC1Init(TIM4, &channel);

	TIM_Cmd(TIM4, ENABLE);
}

void pwminput() {
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);

	TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);

	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);

	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

	TIM_Cmd(TIM3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

int main(void) {
	usart();
	sendString(
			(unsigned char *) "\bLista komend dostepna pod polecenie help;\r\n");
	while (1) {
		if (rx_wr_index != rx_rd_index) {
			szukaniekomendy();
		}
		if (prze) {
			if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5))
				GPIO_ResetBits(GPIOA, GPIO_Pin_5);
			else
				GPIO_SetBits(GPIOA, GPIO_Pin_5);
			prze = 0;
		}
		if (pul) {
			sendString((unsigned char *) "PulsDown\r\n");
			GPIO_ResetBits(GPIOA, GPIO_Pin_5);
			pul = 0;
		}
		if (zbi) {
			sendString((unsigned char *) "PulsUp\r\n");
			GPIO_SetBits(GPIOA, GPIO_Pin_5);
			zbi = 0;
		}

		if (pwin) {
			IC2Value = TIM_GetCapture2(TIM3);
			if (IC2Value != 0) {
				DutyCycle = (TIM_GetCapture1(TIM3) * 100) / IC2Value;
				Frequency = SystemCoreClock / IC2Value;
			} else {
				DutyCycle = 0;
				Frequency = 0;
			}
			pwin = 0;
		}
	}
}

void sendString(char * string) {
	uint16_t id = tx_wr_index;
	while (*string) {
		tx_buffer[id++] = *string++;
		if (id == BUFFERSIZE)
			id = 0;
	}
	__disable_irq();
	if ((tx_wr_index == tx_rd_index)
			&& (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == SET)) {
		tx_wr_index = id;
		USART_SendData(USART2, tx_buffer[tx_rd_index++]);
		if (tx_rd_index == BUFFERSIZE)
			tx_rd_index = 0;
		USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	} else
		tx_wr_index = id;
	__enable_irq();
}

void usart() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO;
	GPIO.GPIO_Pin = GPIO_Pin_3;
	GPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO);

	GPIO.GPIO_Pin = GPIO_Pin_2;
	GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO);

	USART_InitTypeDef USART;
	USART.USART_BaudRate = 9600;
	USART.USART_WordLength = USART_WordLength_8b;
	USART.USART_StopBits = USART_StopBits_1;
	USART.USART_Parity = USART_Parity_No;
	USART.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART);

	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void USART2_IRQHandler(void) {
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
		rx_buffer[rx_wr_index] = USART_ReceiveData(USART2);
		rx_wr_index++;
		if (rx_wr_index == BUFFERSIZE) {
			rx_wr_index = 0;
		}
	}
	if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
		if (tx_wr_index != tx_rd_index) {
			USART_SendData(USART2, tx_buffer[tx_rd_index++]);
			if (tx_rd_index == BUFFERSIZE) {
				tx_rd_index = 0;
			}
		} else {
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
		}
	}
}

void szukaniekomendy() {
	if (rx_wr_index != rx_rd_index) {
		while (rx_wr_index != rx_rd_index) {
			if (rx_buffer[rx_rd_index] != ';') {
				command_line[pomoc] = rx_buffer[rx_rd_index];
				pomoc++;
				if (pomoc >= 24)
					pomoc = 0;
			} else {
				parse();
				czyszczenie();
				pomoc = 0;
			}
			rx_rd_index++;
			if (rx_rd_index >= 512)
				rx_rd_index = 0;
		}
	}
}

void parse() {
	if (strncmp(command_line, czlig, 5) == 0) {
		sendString((unsigned char *) "Uruchomiono zliczanie w gore!\r\n");
		zliczanieU();
	} else if (strncmp(command_line, cstop, 5) == 0) {
		sendString((unsigned char *) "Zatrzymano liczniki!\r\n");
		stop();
	} else if (strncmp(command_line, czwyl, 5) == 0) {
		sendString((unsigned char *) "Zwykly licznik - co 2s dioda!\r\n");
		zwyklylicznik();
	} else if (strncmp(command_line, kpuls, 5) == 0) {
		cut_command_line(5);
		mojq = 0, moje = 0, skok = 0;
		for (int c = 0; c < 24; c++) {
			if ((command_line[c] != ' ') && (skok == 0)) {
				pwmokres[mojq++] = command_line[c];
				if (command_line[c + 1] == ',') {
					skok = 1;
				}
			} else if ((command_line[c] != " ") && (skok == 1)
					&& (command_line[c] != ',') && (command_line[c] != '\0')) {
				pwmkanal[moje++] = command_line[c];
			}
		}
		pwmwartosckanalu = atoi(pwmkanal);
		pwmwartoscokres = atoi(pwmokres);
		stop();
		jedenpuls();
	} else if (strncmp(command_line, ckont, 5) == 0) {
		konter3 = TIM_GetCounter(TIM3);
		konter5 = TIM_GetCounter(TIM2);
		sprintf((char *) konterW3, "%2i\0", konter3);
		sprintf((char *) konterW5, "%2i\0", konter5);
		sendString((unsigned char *) "Zliczanie w gore jest na ");
		sendString(konterW3);
		sendString((unsigned char *) "\n\r");
		sendString((unsigned char *) "Zliczanie w dol jest na ");
		sendString(konterW5);
		sendString((unsigned char *) "\n\r");
	} else if (strncmp(command_line, chelp, 5) == 0) {
		sendString(
				(unsigned char *) "zlig - Zliczanie w gore\r\nzlid - Zliczanie w dol\r\nkont - Aktualna wartosc zliczania");
		sendString(
				(unsigned char *) "stop - Zatrzymanie licznikow\r\nzwyl - Zwykly licznik co 2s dioda\r\npuls=okres,wartosc_kanalu - Jeden puls\r\n");
		sendString(
				(unsigned char *) "pwminput - PWM Input\r\npwm=dzielnik_zegara,okres,wartosc_kanalu - PWM\r\nkwad - Sygnal kwadraturowy\r\n");
		sendString(
				(unsigned char *) "ipwmin - Wyswietlenie czestotliwosci i wypelnienia PWM\r\ndwyl - Wylaczenie diody\r\nkwaread - Wyswietlanie z kwadraturowego");
	} else if (strncmp(command_line, czlid, 5) == 0) {
		sendString((unsigned char *) "Uruchomiono zliczanie w dol!\r\n");
		zliczanieD();
	} else if (strncmp(command_line, cpwminput, 9) == 0) {
		sendString((unsigned char *) "Uruchomiono PWM Input!\r\n");
		pwminput();
	} else if (strstr(command_line, cpwm)) {
		cut_command_line(4);
		mojq = 0, mojw = 0, moje = 0;
		skok = 0;
		for (int c = 0; c < 24; c++) {
			if ((command_line[c] != ' ') && (skok == 0)) {
				pwmdzielnikzegara[mojq++] = command_line[c];
				if (command_line[c + 1] == ',') {
					skok = 1;
				}
			} else if ((command_line[c] != " ") && (skok == 1)
					&& (command_line[c] != ',')) {
				pwmokres[mojw++] = command_line[c];
				if (command_line[c + 1] == ',') {
					skok = 2;
				}
			} else if ((command_line[c] != " ") && (skok == 2)
					&& (command_line[c] != ',')) {
				pwmkanal[moje++] = command_line[c];
			}
		}
		pwmwartosckanalu = atoi(pwmkanal);
		pwmwartoscdzielnika = atoi(pwmdzielnikzegara);
		pwmwartoscokres = atoi(pwmokres);
		stop();
		pwm();
	} else if (strstr(command_line, cpwm2)) {
		cut_command_line(5);
		mojq = 0, mojw = 0, moje = 0;
		skok = 0;
		for (int c = 0; c < 24; c++) {
			if ((command_line[c] != ' ') && (skok == 0)) {
				pwmdzielnikzegara[mojq++] = command_line[c];
				if (command_line[c + 1] == ',') {
					skok = 1;
				}
			} else if ((command_line[c] != " ") && (skok == 1)
					&& (command_line[c] != ',')) {
				pwmkanal[mojw++] = command_line[c];


		}}
double pomoc=0;
		pwmwartosckanalu = (atoi(pwmkanal)*10);
		pomoc=(atoi(pwmdzielnikzegara)*1000);
		pwmwartoscdzielnika = (72000/pomoc);
		pwmwartoscokres = 1000;



		stop();
		pwm();
	}else if (strncmp(command_line, ckwad, 5) == 0) {
		sendString((unsigned char *) "Uruchomiono sygnal kwadraturowy v1\r\n");
		kwadratura1();
	} else if (strncmp(command_line, cipwmin, 7) == 0) {
		sprintf((char *) wypelnienie, "%f", DutyCycle);
		sprintf((char *) czestotliwosc, "%f", Frequency);
		sendString((unsigned char *) "Czestotliwosc: ");
		sendString((unsigned char *) czestotliwosc);
		sendString((unsigned char *) " Hz\r\n");
		sendString((unsigned char *) "Wypelnienie: ");
		sendString((unsigned char *) wypelnienie);
		sendString((unsigned char *) " %\r\n");
	} else if (strncmp(command_line, ckwaread, 8) == 0) {
		encodersRead();
	} else {
		sendString((unsigned char*) "Zla komenda\r\n");
	}
}
void czyszczenie() {
	for (int i = 0; i < 24; i++) {
		command_line[i] = '\0';
	}
}
void cut_command_line(int a) {
	for (int i = 0; i < a; i++) {
		command_line[i] = ' ';
	}
}
/*
 void jedenpuls() {
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

	GPIO_Init(GPIOB, &GPIO_InitStructure);
	TIM_TimeBaseStructure.TIM_Period = pwmwartoscokres-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 72000;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse =  pwmwartosckanalu;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);

	TIM_ICStructInit(&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0;

	TIM_ICInit(TIM4, &TIM_ICInitStructure);

	TIM_SelectOnePulseMode(TIM4, TIM_OPMode_Single);

	TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);

	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Trigger);

 }*/
//available to the rest of the code
//speeds
volatile int16_t leftCount;
volatile int16_t rightCount;
volatile int16_t fwdCount;
volatile int16_t rotCount;
//distances
volatile int32_t leftTotal;
volatile int32_t rightTotal;
volatile int32_t fwdTotal;
volatile int32_t rotTotal;

// local variables
static volatile int16_t oldLeftEncoder;
static volatile int16_t oldRightEncoder;
static volatile int16_t leftEncoder;
static volatile int16_t rightEncoder;
static volatile int16_t encoderSum;
static volatile int16_t encoderDiff;
void encodersRead(void) {
	oldLeftEncoder = leftEncoder;
	oldRightEncoder = rightEncoder;
	leftCount = leftEncoder - oldLeftEncoder;
	rightCount = rightEncoder - oldRightEncoder;
	fwdCount = leftCount + rightCount;
	rotCount = -(leftCount - rightCount);
	fwdTotal += fwdCount;
	rotTotal += rotCount;
	leftTotal += leftCount;
	rightTotal += rightCount;
	sprintf((char *) fwd, "%2i\0", fwdCount);
	sendString(konterW5);
}
void encodersReset(void) {
	__disable_irq();
	oldLeftEncoder = 0;
	oldRightEncoder = 0;
	leftTotal = 0;
	rightTotal = 0;
	fwdTotal = 0;
	rotTotal = 0;
	encodersRead();
	__enable_irq();
}

void kwadratura1() {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
			TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_SetAutoreload(TIM2, 0xffff);
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
			TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_SetAutoreload(TIM4, 0xffff);

	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	encodersReset();
}
