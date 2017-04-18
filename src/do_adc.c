#include "ch.h"
#include "hal.h"
#include "pwm.h"

#define ADC_GRP1_NUM_CHANNELS   1
#define ADC_GRP1_BUF_DEPTH      8


static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 8 samples of 1 channel, SW triggered.
 * Channels:    IN8.
 */
static const ADCConversionGroup adcgrpcfg1 = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  NULL,
  adcerrorcallback,
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3),
  0,                        /* SMPR2 */
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  0,                        /* SQR2 */
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN8)
};


void InitADC(void) {
  /*
   * Activates the ADC1 driver and the temperature sensor.
   */
  adcStart(&ADCD1, NULL);
  adcSTM32EnableTSVREFE();

}

int DoADC(void)
{
  /*
   * Linear conversion.
   */
  adcConvert(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);

  return *samples1;
}

#define ADC_GRP2_NUM_CHANNELS   14
#define ADC_GRP2_BUF_DEPTH      1

static adcsample_t samples2[ADC_GRP2_NUM_CHANNELS * ADC_GRP2_BUF_DEPTH];

size_t nx = 0, ny = 0;

static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

  (void)adcp;
  if (samples1 == buffer) {
    nx += n;
  }
  else {
    ny += n;
  }
}


/*
 * ADC conversion group.
 * Mode:        Continuous, 16 samples of 8 channels, SW triggered.
 * Channels:    IN11, IN12, IN11, IN12, IN11, IN12, Sensor, VRef.
 */
static const ADCConversionGroup adcgrpcfg2 = {
  FALSE,
  ADC_GRP2_NUM_CHANNELS,
  adccallback,
  adcerrorcallback,
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  ADC_SMPR1_SMP_AN10(ADC_SAMPLE_56) | ADC_SMPR1_SMP_AN11(ADC_SAMPLE_56) |
  ADC_SMPR1_SMP_AN12(ADC_SAMPLE_56) | ADC_SMPR1_SMP_AN13(ADC_SAMPLE_56) |
  ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_56) | ADC_SMPR1_SMP_VREF(ADC_SAMPLE_56), /* SMPR1*/
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_56) |
  ADC_SMPR2_SMP_AN1(ADC_SAMPLE_56) | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_56) |
  ADC_SMPR2_SMP_AN3(ADC_SAMPLE_56) | ADC_SMPR2_SMP_AN4(ADC_SAMPLE_56) |
  ADC_SMPR2_SMP_AN5(ADC_SAMPLE_56) | ADC_SMPR2_SMP_AN6(ADC_SAMPLE_56) |
  ADC_SMPR2_SMP_AN9(ADC_SAMPLE_56) ,     /* SMPR2 */
  ADC_SQR1_NUM_CH(ADC_GRP2_NUM_CHANNELS) |
  ADC_SQR1_SQ14_N(ADC_CHANNEL_SENSOR) | ADC_SQR1_SQ13_N(ADC_CHANNEL_IN13),
  ADC_SQR2_SQ12_N(ADC_CHANNEL_IN12) | ADC_SQR2_SQ11_N(ADC_CHANNEL_IN11) |
  ADC_SQR2_SQ10_N(ADC_CHANNEL_IN10) | ADC_SQR2_SQ9_N(ADC_CHANNEL_IN9) |
  ADC_SQR2_SQ8_N(ADC_CHANNEL_IN8)   | ADC_SQR2_SQ7_N(ADC_CHANNEL_IN6),
  ADC_SQR3_SQ6_N(ADC_CHANNEL_IN5)   | ADC_SQR3_SQ5_N(ADC_CHANNEL_IN4) |
  ADC_SQR3_SQ4_N(ADC_CHANNEL_IN3)   | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN2) |
  ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1)   | ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0)
};

adcsample_t *ReadADCs(void) {

  /*
   * Linear conversion.
   */
  adcConvert(&ADCD1, &adcgrpcfg2, samples2, ADC_GRP2_BUF_DEPTH);

  return samples2;
}
