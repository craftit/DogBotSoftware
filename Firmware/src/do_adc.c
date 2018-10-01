#include "ch.h"
#include "hal.h"
#include "pwm.h"
#include "chbsem.h"
#include "math.h"
#include "bmc.h"

#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"

/* ADC JEXTEN mask */
#define CR2_JEXTEN_RESET          ((uint32_t)0xFFCFFFFF)

#define SMPR1_SMP_SET             ((uint32_t)0x00000007)
#define SMPR2_SMP_SET             ((uint32_t)0x00000007)

void ADC_InjectedChannelSampleTime(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t ADC_SampleTime)
{
  uint32_t tmpreg1 = 0, tmpreg2 = 0;
  /* if ADC_Channel_10 ... ADC_Channel_18 is selected */
  if (ADC_Channel > ADC_Channel_9)
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SMPR1;
    /* Calculate the mask to clear */
    tmpreg2 = SMPR1_SMP_SET << (3*(ADC_Channel - 10));
    /* Clear the old sample time */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    tmpreg2 = (uint32_t)ADC_SampleTime << (3*(ADC_Channel - 10));
    /* Set the new sample time */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADCx->SMPR1 = tmpreg1;
  }
  else /* ADC_Channel include in ADC_Channel_[0..9] */
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SMPR2;
    /* Calculate the mask to clear */
    tmpreg2 = SMPR2_SMP_SET << (3 * ADC_Channel);
    /* Clear the old sample time */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    tmpreg2 = (uint32_t)ADC_SampleTime << (3 * ADC_Channel);
    /* Set the new sample time */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADCx->SMPR2 = tmpreg1;
  }
}

void ADCSampleCurrentAndHall(void)
{
  // ADC1

  // See page 427 of the reference manual.
  // Set this register by hand, I don't trust the above functions, though they do set the sample
  // times.
  ADC1->JSQR = (1 << 20) | (ADC_Channel_10 << (5 * 2)) |  (ADC_Channel_5 << (5 * 3));


  // ADC2

  ADC2->JSQR = (1 << 20) | (ADC_Channel_11 << (5 * 2)) |  (ADC_Channel_4 << (5 * 3));


  // ADC3

  ADC3->JSQR = (1 << 20) | (ADC_Channel_12 << (5 * 2)) |  (ADC_Channel_3 << (5 * 3));


}

void ADCSampleVoltage(void) {
  // ADC1

  // See page 427 of the reference manual.
  // Set this register by hand, I don't trust the above functions, though they do set the sample
  // times.
  ADC1->JSQR = (1 << 20) | (ADC_Channel_10 << (5 * 2)) |  (ADC_Channel_6 << (5 * 3));


  // ADC2

  ADC2->JSQR = (1 << 20) | (ADC_Channel_11 << (5 * 2)) |  (ADC_Channel_9 << (5 * 3));


  // ADC3

  ADC3->JSQR = (1 << 20) | (ADC_Channel_12 << (5 * 2)) |  (ADC_Channel_13 << (5 * 3));

}

void StopADC(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, FALSE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, FALSE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, FALSE);

}

void InitADC(void)
{
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

  // Setup regular channel for converting the supply voltage
  ADC_RegularChannelConfig(ADC2,ADC_Channel_6,1,ADC_SampleTime_3Cycles);

  // These are just to setup the cycle times.

  // See page 564 of the reference manual for the Tim1 source setup.

  {
    // ADC1
    // NOTE:  Channels have to setup in order of rank.
    ADC_InjectedSequencerLengthConfig(ADC1,2);
    ADC_InjectedChannelSampleTime(ADC1,ADC_Channel_10, ADC_SampleTime_3Cycles); // Hall A
    ADC_InjectedChannelSampleTime(ADC1,ADC_Channel_5,  ADC_SampleTime_3Cycles); // ISenseC
    ADC_InjectedChannelSampleTime(ADC1,ADC_Channel_6,  ADC_SampleTime_3Cycles); // VSupply

    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
    ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Falling);

    // ADC2

    ADC_InjectedSequencerLengthConfig(ADC2,2);
    ADC_InjectedChannelSampleTime(ADC2,ADC_Channel_11, ADC_SampleTime_3Cycles); // Hall B
    ADC_InjectedChannelSampleTime(ADC2,ADC_Channel_4,  ADC_SampleTime_3Cycles); // ISenseB
    ADC_InjectedChannelSampleTime(ADC2,ADC_Channel_0,  ADC_SampleTime_3Cycles);  // VSenseA
    ADC_InjectedChannelSampleTime(ADC2,ADC_Channel_13, ADC_SampleTime_3Cycles); // Temp Motor

    ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T1_TRGO);
    ADC_ExternalTrigInjectedConvEdgeConfig(ADC2, ADC_ExternalTrigInjecConvEdge_Falling);

    // ADC3

    ADC_InjectedSequencerLengthConfig(ADC3,2);
    ADC_InjectedChannelSampleTime(ADC3,ADC_Channel_12, ADC_SampleTime_3Cycles);   // Hall C
    ADC_InjectedChannelSampleTime(ADC3,ADC_Channel_3,  ADC_SampleTime_3Cycles);   // ISenseA
    ADC_InjectedChannelSampleTime(ADC3,ADC_Channel_1,  ADC_SampleTime_3Cycles);   // VSenseB
    ADC_InjectedChannelSampleTime(ADC3,ADC_Channel_2,  ADC_SampleTime_3Cycles);   // VSenseC
    ADC_InjectedChannelSampleTime(ADC3,ADC_Channel_9,  ADC_SampleTime_3Cycles);   // Temp Driver

    ADC_ExternalTrigInjectedConvConfig(ADC3, ADC_ExternalTrigInjecConv_T1_TRGO);
    ADC_ExternalTrigInjectedConvEdgeConfig(ADC3, ADC_ExternalTrigInjecConvEdge_Falling);

    ADCSampleCurrentAndHall();
  }


  nvicEnableVector(STM32_ADC_NUMBER, STM32_ADC_IRQ_PRIORITY);

  ADC_ITConfig(ADC1,ADC_IT_JEOC,ENABLE);
  ADC_ITConfig(ADC2,ADC_IT_JEOC,ENABLE);
  ADC_ITConfig(ADC3,ADC_IT_JEOC,ENABLE);
  ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);

  ADC_Cmd(ADC1,ENABLE);
  ADC_Cmd(ADC2,ENABLE);
  ADC_Cmd(ADC3,ENABLE);

}

int g_currentSample = 0;
int g_adcTickCount = 0;
static uint8_t g_samplesDone[16];
static uint16_t g_samples[16];

int16_t g_currentADCValue[3];
int16_t g_supplyADCValue = 0;
int16_t g_driverTempADCValue = 0;
int16_t g_motorTempADCValue = 0;
uint16_t g_hall[3];
int g_adcInjCount = 0;
float g_supplyVoltageScale = 1.0;

BSEMAPHORE_DECL(g_adcInjectedDataReady,0);
BSEMAPHORE_DECL(g_adcDataReady,0);

inline bool IsRisingEdge(ADC_TypeDef* ADCx) {
  uint32_t tmpreg = ADCx->CR2;
  return (tmpreg & ~CR2_JEXTEN_RESET) == ADC_ExternalTrigInjecConvEdge_Rising;
}

OSAL_IRQ_HANDLER(STM32_ADC_HANDLER) {
  //uint32_t sr;

  OSAL_IRQ_PROLOGUE();

#if 1
  int count  = 0;

  bool isRisingEdge = false;// IsRisingEdge(ADC1);
#if 0
  {
    stm32_tim_t *tim = (stm32_tim_t *)TIM1_BASE;
    isRisingEdge = tim->CR1 & STM32_TIM_CR1_DIR;
  }
#endif


  if(ADC_GetITStatus(ADC1, ADC_IT_JEOC) == SET) {
    ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    if(isRisingEdge) {
      g_supplyADCValue = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2);
    } else {
      g_currentADCValue[0] = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2);
    }
    g_hall[0] = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1);
    count++;
  }

  if(ADC_GetITStatus(ADC2, ADC_IT_JEOC) == SET) {
    ADC_ClearITPendingBit(ADC2, ADC_IT_JEOC);
    if(isRisingEdge) {
      g_driverTempADCValue = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_2);
    } else {
      g_currentADCValue[1] = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_2);
    }
    g_hall[1] = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_1);
    count++;
  }

  if(ADC_GetITStatus(ADC3, ADC_IT_JEOC) == SET) {
    ADC_ClearITPendingBit(ADC3, ADC_IT_JEOC);
    if(isRisingEdge) {
      g_motorTempADCValue = ADC_GetInjectedConversionValue(ADC3,ADC_InjectedChannel_2);
    } else {
      g_currentADCValue[2] = ADC_GetInjectedConversionValue(ADC3,ADC_InjectedChannel_2);
    }
    g_hall[2] = ADC_GetInjectedConversionValue(ADC3,ADC_InjectedChannel_1);
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

  if(ADC_GetITStatus(ADC2, ADC_IT_EOC) == SET) {
    ADC_ClearITPendingBit(ADC2, ADC_IT_EOC);
  }

  if(ADC_GetITStatus(ADC3, ADC_IT_EOC) == SET) {
    ADC_ClearITPendingBit(ADC3, ADC_IT_EOC);
  }

  if(count > 0) {
    g_adcInjCount = count;
    if(!isRisingEdge) {
      //palSetPad(GPIOB, GPIOB_PIN12); // Flag data captured
      g_adcTickCount++;
      //ADCSampleVoltage(); // Sample voltage on next rising edge.
      chBSemSignalI(&g_adcInjectedDataReady);
    } else {
      //palClearPad(GPIOB, GPIOB_PIN12); // Turn output off to measure timing
      // Sample current next
      ADCSampleCurrentAndHall();
    }

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
    ADC_RegularChannelConfig(ADC1,g_convSeq[i],1,ADC_SampleTime_15Cycles);
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


uint16_t ReadADC(uint8_t ADC_Channel)
{
  ADC_RegularChannelConfig(ADC1,ADC_Channel,1,ADC_SampleTime_3Cycles);
  g_currentSample = 0;

  // Make sure we don't get out of step.
  chBSemWaitTimeout(&g_adcDataReady,TIME_IMMEDIATE);

  ADC_SoftwareStartConv(ADC1);
  g_samplesDone[0] = 0;
  msg_t state = chBSemWaitTimeout(&g_adcDataReady,1000);
  if(state != MSG_OK)
    return 0;
  return g_samples[0];
}

float Read5VRailVoltage(void)
{
  uint16_t val = ReadADC(ADC_Channel_8);
  return val * 2.0 * 3.3f/((float)(1<<12));
}

float ReadSupplyVoltage(void)
{
  uint16_t val = ReadADC(ADC_Channel_6);

  return ((float) val) * ((3.0+39.0)/3.0) * 3.3f/((float)(1<<12)) * g_supplyVoltageScale;
}

float ReadDriveTemperature(void)
{
  uint16_t adcVal = ReadADC(ADC_Channel_9);

  const float refResistor = 10e3;
  float res =  refResistor * adcVal / ((1<<12) - (adcVal+1));

  const float thermistorNominal = 10e3;
  const float temperatureNominal = 25;
  const float thermistorK = 3430;

  float temp = 1.0/((logf(res / thermistorNominal) / thermistorK) + 1.0 / (temperatureNominal + 273.15)) - 273.15;
  return temp;
}

float ReadMotorTemperature(void)
{
  uint16_t adcVal = ReadADC(ADC_Channel_13);

  const float refResistor = 10e3;
  float res = refResistor * (((float) (1<<12)/(adcVal+1.0f)) - 1.0f);

  const float thermistorNominal = 10e3;
  const float temperatureNominal = 25;
  const float thermistorK = 3435;

  float temp = 1.0/((logf(res / thermistorNominal) / thermistorK) + 1.0 / (temperatureNominal + 273.15)) - 273.15;
  return temp;
}

int DoADC(void)
{


  return g_samples[0];
}


