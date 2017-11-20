#ifndef DRV8503_HEADER
#define DRV8503_HEADER 1

#ifdef __cplusplus
extern "C" {
#endif

extern uint16_t Drv8503ReadStatus(void);
extern uint16_t Drv8503ReadRegister(uint16_t addr);
extern uint16_t Drv8503SetRegister(uint16_t addr,uint16_t value);

extern uint16_t Drv8503Test(void);
extern void InitDrv8503(void);

// ------------------------------------------------------
// Register 0x5 :

// ------------------------------------------------------
// Register 0x9 : IC Operation


#define DRV8503_REG_IC_CONTROL   (0x9)

#define DRV8503_IC_VCPH_UV_4V9   (0U)
#define DRV8503_IC_VCPH_UV_4V6   (1U)
#define DRV8503_IC_CLR_FLTS      (1U<<1)
#define DRV8503_IC_SLEEP         (1U<<2)
#define DRV8503_IC_WD_EN         (1U<<3)
#define DRV8503_IC_DIS_SNS_OCP   (1U<<4)

#define DRV8503_IC_WD_DLY_10ms  (0b00 << 5)
#define DRV8503_IC_WD_DLY_20ms  (0b01 << 5)
#define DRV8503_IC_WD_DLY_50ms  (0b10 << 5)
#define DRV8503_IC_WD_DLY_100ms (0b11 << 5)

#define DRV8503_IC_EN_SNS_CLAMP   (1U<<7)
#define DRV8503_IC_DIS_GDRV_FAULT (1U<<8)
#define DRV8503_IC_DIS_PVDD_UVLO2 (1U<<9)
#define DRV8503_IC_FLIP_OTSD    (1U<<10)

// ------------------------------------------------------
// Register 0xA : Shunt Amplifier Control
#define DRV8503_REG_SHUNT_AMPLIFIER_CONTROL (0xAU)

// Blanking
#define DRV8503_CS_BLANK_OFF   (0U)
#define DRV8503_CS_BLANK_0_5US (1U << 6)  // 0.5 us
#define DRV8503_CS_BLANK_2_5US (2U << 6)   // 2.5 us
#define DRV8503_CS_BLANK_10US  (3U << 6)   // 10us

// Cal mode for each amplifier, sets input to 0V to calibrate the DC offset.

#define DRV8503_DC_CAL_CH1 (1U<<8)
#define DRV8503_DC_CAL_CH2 (1U<<9)
#define DRV8503_DC_CAL_CH3 (1U<<10)

// Gain for shunt 3

#define DRV8503_GAIN_CS3_10 (0U<<4)
#define DRV8503_GAIN_CS3_20 (1U<<4)
#define DRV8503_GAIN_CS3_40 (2U<<4)
#define DRV8503_GAIN_CS3_80 (3U<<4)

// Gain for shunt 2

#define DRV8503_GAIN_CS2_10 (0U<<2)
#define DRV8503_GAIN_CS2_20 (1U<<2)
#define DRV8503_GAIN_CS2_40 (2U<<2)
#define DRV8503_GAIN_CS2_80 (3U<<2)

// Gain for shunt 1  V/V

#define DRV8503_GAIN_CS1_10 (0U)
#define DRV8503_GAIN_CS1_20 (1U)
#define DRV8503_GAIN_CS1_40 (2U)
#define DRV8503_GAIN_CS1_80 (3U)

#define DRV8503_PEAK_SOURCE_20mA        (0b0000)
#define DRV8503_PEAK_SOURCE_30mA        (0b0001)
#define DRV8503_PEAK_SOURCE_40mA        (0b0010)
#define DRV8503_PEAK_SOURCE_50mA        (0b0011)
#define DRV8503_PEAK_SOURCE_60mA        (0b0100)
#define DRV8503_PEAK_SOURCE_70mA        (0b0101)
#define DRV8503_PEAK_SOURCE_80mA        (0b0110)
#define DRV8503_PEAK_SOURCE_250mA       (0b0111)
#define DRV8503_PEAK_SOURCE_500mA       (0b1000)
#define DRV8503_PEAK_SOURCE_750mA       (0b1001)
#define DRV8503_PEAK_SOURCE_1000mA      (0b1010)
#define DRV8503_PEAK_SOURCE_1250mA      (0b1011)

#define DRV8503_PEAK_SINK_10mA          (0b0000 << 4)
#define DRV8503_PEAK_SINK_20mA          (0b0001 << 4)
#define DRV8503_PEAK_SINK_30mA          (0b0010 << 4)
#define DRV8503_PEAK_SINK_40mA          (0b0011 << 4)
#define DRV8503_PEAK_SINK_50mA          (0b0100 << 4)
#define DRV8503_PEAK_SINK_60mA          (0b0101 << 4)
#define DRV8503_PEAK_SINK_70mA          (0b0110 << 4)
#define DRV8503_PEAK_SINK_125mA         (0b0111 << 4)
#define DRV8503_PEAK_SINK_250mA         (0b1000 << 4)
#define DRV8503_PEAK_SINK_500mA         (0b1001 << 4)
#define DRV8503_PEAK_SINK_750mA         (0b1010 << 4)
#define DRV8503_PEAK_SINK_1000mA        (0b1011 << 4)

#define DRV8503_SOURCE_TIME_220ns       (0b00 << 8)
#define DRV8503_SOURCE_TIME_440ns       (0b01 << 8)
#define DRV8503_SOURCE_TIME_880ns       (0b10 << 8)
#define DRV8503_SOURCE_TIME_1780ns      (0b11 << 8)

#define DRV8503_REG_DRIVER_HS     (0x5)
#define DRV8503_REG_DRIVER_LS     (0x6)


#define DRV8503_VDS_THRESHOLD_0V060         (0b00000 << 3)
#define DRV8503_VDS_THRESHOLD_0V068         (0b00001 << 3)
#define DRV8503_VDS_THRESHOLD_0V076         (0b00010 << 3)
#define DRV8503_VDS_THRESHOLD_0V086         (0b00011 << 3)
#define DRV8503_VDS_THRESHOLD_0V097         (0b00100 << 3)
#define DRV8503_VDS_THRESHOLD_0V109         (0b00101 << 3)
#define DRV8503_VDS_THRESHOLD_0V123         (0b00110 << 3)
#define DRV8503_VDS_THRESHOLD_0V138         (0b00111 << 3)
#define DRV8503_VDS_THRESHOLD_0V155         (0b01000 << 3)
#define DRV8503_VDS_THRESHOLD_0V175         (0b01001 << 3)
#define DRV8503_VDS_THRESHOLD_0V197         (0b01010 << 3)
#define DRV8503_VDS_THRESHOLD_0V222         (0b01011 << 3)
#define DRV8503_VDS_THRESHOLD_0V250         (0b01100 << 3)
#define DRV8503_VDS_THRESHOLD_0V282         (0b01101 << 3)
#define DRV8503_VDS_THRESHOLD_0V317         (0b01110 << 3)
#define DRV8503_VDS_THRESHOLD_0V358         (0b01111 << 3)
#define DRV8503_VDS_THRESHOLD_0V403         (0b10000 << 3)
#define DRV8503_VDS_THRESHOLD_0V454         (0b10001 << 3)
#define DRV8503_VDS_THRESHOLD_0V511         (0b10010 << 3)
#define DRV8503_VDS_THRESHOLD_0V576         (0b10011 << 3)
#define DRV8503_VDS_THRESHOLD_0V648         (0b10100 << 3)
#define DRV8503_VDS_THRESHOLD_0V730         (0b10101 << 3)
#define DRV8503_VDS_THRESHOLD_0V822         (0b10110 << 3)
#define DRV8503_VDS_THRESHOLD_0V926         (0b10111 << 3)
#define DRV8503_VDS_THRESHOLD_1V043         (0b11000 << 3)
#define DRV8503_VDS_THRESHOLD_1V175         (0b11001 << 3)
#define DRV8503_VDS_THRESHOLD_1V324         (0b11010 << 3)
#define DRV8503_VDS_THRESHOLD_1V491         (0b11011 << 3)
#define DRV8503_VDS_THRESHOLD_1V679         (0b11100 << 3)
#define DRV8503_VDS_THRESHOLD_1V892         (0b11101 << 3)
#define DRV8503_VDS_THRESHOLD_2V131         (0b11110 << 3)

#define DRV8503_VDS_SHUT_DOWN               (0b000)
#define DRV8503_VDS_REPORT_ONLY             (0b001)
#define DRV8503_VDS_PROTECTION_DISABLE      (0b010)

#define DRV8503_REG_VDS_SENSE_CONTROL     (0xC)




#ifdef __cplusplus
}
#endif

#endif
