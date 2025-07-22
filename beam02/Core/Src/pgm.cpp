/*
 * pgm.cpp
 *
 *  Created on: Jul 20, 2025
 *      Author: louis
 */


#include "main.h"
#include <math.h>
#include <cstdio>

//#include "arm_math.h"
/*
extern DAC_HandleTypeDef hdac;
*/

template <typename T> T square(const T t) { return t*t; }


template <int size, typename T> struct Buf {
  T buffer[size];
  long int sum = 0;
  long int var = 0;
  unsigned int p;
  Buf() : p(0) {
	  for (int i = 0; i < size; i++) put(i);
  }
  void put(const T &t) {
    p = (p+1)%size;
    const auto cc = buffer[p];
    sum -= cc;
    var -= square(cc-mean());
	buffer[p] = t;
    sum += t;
    var += square(t-mean());
  }
  const T queue() const { return buffer[(p+1)%size]; }
  const T mean() const { return sum/size; }
  const T ecart_type() const { return sqrtf(var/size);  }
  const T head() const { return buffer[p]; }
};

const float speed_sound_m_sec = 340.;
const float inter_like_dist = 0.1;
const int number_adc_cycles = 144;
const int mega = 1000 * 1000;
const int freq_clock = 72 * mega; // MHz
const int adc_freq_clock = freq_clock / 2; // PCLK2 divided by 2
const float T_sec = 1. / adc_freq_clock;


const float sample_rate = 1. / (T_sec * number_adc_cycles * 4) * 4;
const float T_sample = 1. / sample_rate;
const float dist = 0.1;
const float duration_dist = dist / speed_sound_m_sec;
const float n_samples = duration_dist / T_sample;
const int n_samples_i = (int)n_samples;
Buf<n_samples_i*3, int> buf3;
Buf<n_samples_i*2, int> buf2;
Buf<n_samples_i, int> buf1;
Buf<n_samples_i, int> buf_out;


auto smn  = buf3.mean();
auto sec  = buf3.ecart_type();

long int  adc1 = 0;
GPIO_PinState bvalue;
long int b = 0;

extern "C" long int acc;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern DAC_HandleTypeDef hdac;
extern DMA_HandleTypeDef hdma_dac2;

// extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
extern "C" int pgm_loop() ;
extern "C" int pgm_init();
extern "C" int _write(int file, char *ptr, int len);

const int NN = 1024*10;
uint32_t AD_RES_BUFFER[NN];
uint32_t DA_RES_BUFFER[NN];

int pgm_init() {

	printf("coucou\n");

	HAL_ADC_Start_DMA(&hadc1, AD_RES_BUFFER, NN);

	HAL_TIM_Base_Start_IT(&htim6);

	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, DA_RES_BUFFER, NN, DAC_ALIGN_12B_R);

	return 0;
}

int wwww = 55;
int dac1 = 0;
int dac2 = 0;

int _writeXXX(int file, char *ptr, int len) {
	for (int DataIdx = 0; DataIdx < len; DataIdx ++) {
		ITM_SendChar(*ptr++);
	}
	wwww++;
	return len;
}

extern "C"  void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    //do_dac(&dma_buffer[DMA_BUFFER_SIZE]);
	dac1 ++;
}


extern "C"  void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    //do_dac(&dma_buffer[0]);
	dac2 ++;
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Conversion Complete & DMA Transfer Complete As Well
    // So The AD_RES_BUFFER Is Now Updated & Let's Move Values To The PWM CCRx
    // Update The PWM Channels With Latest ADC Scan Conversion Results
	/*
    TIM2->CCR1 = (AD_RES_BUFFER[0] << 4);  // ADC CH6 -> PWM CH1
    TIM2->CCR2 = (AD_RES_BUFFER[1] << 4);  // ADC CH7 -> PWM CH2
    TIM2->CCR3 = (AD_RES_BUFFER[2] << 4);  // ADC CH8 -> PWM CH3
    TIM2->CCR4 = (AD_RES_BUFFER[3] << 4);  // ADC CH9 -> PWM CH4
    */
    adc1 ++;
    bvalue = (GPIO_PinState)(adc1 % 2);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, bvalue);
    for (int i = 0; i < NN; i+=4) {
    	buf3.put(AD_RES_BUFFER[i+3]);
    	buf2.put(AD_RES_BUFFER[i+2]);
    	buf1.put(AD_RES_BUFFER[i+1]);
    	int g = (AD_RES_BUFFER[i] + buf3.queue() + buf2.queue() + buf1.queue())/4;
    	buf_out.put(g);
    }
}

int pgm_loop()
{
	int hh = buf3.head();
	int qq = buf3.queue();
	buf3.put(-1);
	int hh1 = buf3.head();
	int qq1 = buf3.queue();
	//a++;
	//acc ++;
	while(1) {
		int v = AD_RES_BUFFER[0]/10;
		auto mn  = buf3.mean();
		auto ec = buf3.ecart_type();
		v = buf3.ecart_type();
		b++;
		if (1) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (GPIO_PinState)0);
			HAL_Delay(v);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (GPIO_PinState)1);
			HAL_Delay(v);
		}
        printf("coucou\n"); fflush(stdout);

		b++;
		//acc ++;
		//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, v);
	}
	return 0;
}


