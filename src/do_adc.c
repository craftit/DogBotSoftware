#include "ch.h"
#include "hal.h"
#include "pwm.h"
#include "chbsem.h"

#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"

void InitADC(void) {
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  /* ADC Common configuration *************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_6Cycles;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
  ADC_CommonInit(&ADC_CommonInitStructure);

  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_Init(ADC2, &ADC_InitStructure);
  ADC_Init(ADC3, &ADC_InitStructure);

  // ADC1
  // NOTE:  Channels have to setup in order of rank.
  ADC_InjectedChannelConfig(ADC1,ADC_Channel_10, 1, ADC_SampleTime_3Cycles);
  ADC_InjectedChannelConfig(ADC1,ADC_Channel_3, 2, ADC_SampleTime_3Cycles);
  ADC_InjectedSequencerLengthConfig(ADC1,2);

  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
  ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Falling);

  // ADC2

  ADC_InjectedChannelConfig(ADC2,ADC_Channel_11, 1, ADC_SampleTime_3Cycles);
  ADC_InjectedChannelConfig(ADC2,ADC_Channel_4, 2, ADC_SampleTime_3Cycles);
  ADC_InjectedSequencerLengthConfig(ADC2,2);

  ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T1_TRGO);
  ADC_ExternalTrigInjectedConvEdgeConfig(ADC2, ADC_ExternalTrigInjecConvEdge_Falling);

  // ADC3

  ADC_InjectedChannelConfig(ADC3,ADC_Channel_12, 1, ADC_SampleTime_3Cycles);
  ADC_InjectedChannelConfig(ADC3,ADC_Channel_5, 2, ADC_SampleTime_3Cycles);
  ADC_InjectedSequencerLengthConfig(ADC3,2);

  ADC_ExternalTrigInjectedConvConfig(ADC3, ADC_ExternalTrigInjecConv_T1_TRGO);
  ADC_ExternalTrigInjectedConvEdgeConfig(ADC3, ADC_ExternalTrigInjecConvEdge_Falling);

  nvicEnableVector(STM32_ADC_NUMBER, STM32_ADC_IRQ_PRIORITY);

  ADC_ITConfig(ADC1,ADC_IT_JEOC,ENABLE);
  ADC_ITConfig(ADC2,ADC_IT_JEOC,ENABLE);
  ADC_ITConfig(ADC3,ADC_IT_JEOC,ENABLE);
  ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);

  palSetPad(GPIOB, GPIOB_PIN12); // on

  ADC_Cmd(ADC1,ENABLE);
  ADC_Cmd(ADC2,ENABLE);
  ADC_Cmd(ADC3,ENABLE);

}

int g_currentSample = 0;
static uint8_t g_samplesDone[16];
static uint16_t g_samples[16];

int16_t g_currentADCValue[3];
uint16_t g_hall[3];
int g_adcInjCount = 0;


BSEMAPHORE_DECL(g_adcInjectedDataReady,0);
BSEMAPHORE_DECL(g_adcDataReady,0);

OSAL_IRQ_HANDLER(STM32_ADC_HANDLER) {
  //uint32_t sr;

  OSAL_IRQ_PROLOGUE();


#if 1
  int count  = 0;
  if(ADC_GetITStatus(ADC1, ADC_IT_JEOC) == SET) {
    ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    g_currentADCValue[2] = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1);
    g_hall[0] = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2);
    count++;
  }

  if(ADC_GetITStatus(ADC2, ADC_IT_JEOC) == SET) {
    ADC_ClearITPendingBit(ADC2, ADC_IT_JEOC);
    g_currentADCValue[1] = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_1);
    g_hall[1] = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_2);
    count++;
  }

  if(ADC_GetITStatus(ADC3, ADC_IT_JEOC) == SET) {
    ADC_ClearITPendingBit(ADC3, ADC_IT_JEOC);
    g_currentADCValue[0] = ADC_GetInjectedConversionValue(ADC3,ADC_InjectedChannel_1);
    g_hall[2] = ADC_GetInjectedConversionValue(ADC3,ADC_InjectedChannel_2);
    count++;
  }

  if(ADC_GetITStatus(ADC1, ADC_IT_EOC) == SET) {
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    g_samples[g_currentSample] = ADC_GetConversionValue(ADC1);
    g_samplesDone[g_currentSample] = 1;
    ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
    chBSemSignalI(&g_adcDataReady);
    //adc_inj_int_handler();
  }

  if(ADC_GetITStatus(ADC1, ADC_IT_OVR) == SET) {
    ADC_ClearITPendingBit(ADC1, ADC_IT_OVR);
  }

  if(count > 0) {
    g_adcInjCount = count;
    palSetPad(GPIOB, GPIOB_PIN12); // Flag data captured
    chBSemSignalI(&g_adcInjectedDataReady);
  }
#endif
  OSAL_IRQ_EPILOGUE();
}


uint16_t *ReadADCs(void) {

  static int g_convSeq[13] =  {
      ADC_Channel_0,ADC_Channel_1,ADC_Channel_2,
      ADC_Channel_3,ADC_Channel_4,ADC_Channel_5,
      ADC_Channel_6,ADC_Channel_8,ADC_Channel_9,
      ADC_Channel_10,ADC_Channel_11,ADC_Channel_12,
      ADC_Channel_15
  };

  for(int i = 0;i < 12;i++) {
    ADC_RegularChannelConfig(ADC1,g_convSeq[i],1,ADC_SampleTime_3Cycles);
    g_currentSample = i;
    ADC_SoftwareStartConv(ADC1);
    g_samplesDone[i] = 0;
    msg_t state = chBSemWaitTimeout(&g_adcDataReady,1000);
    if(state != MSG_OK)
      break; // Timeout or Reset
    //g_samples[i] = ADC_GetConversionValue(ADC1);
    //ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
  }

  return g_samples;
}

int DoADC(void)
{


  return g_samples[0];
}


#if 0
/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

}

#endif


#if 0
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
#endif
