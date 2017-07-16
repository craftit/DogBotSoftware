#ifndef DRV8503_HEADER
#define DRV8503_HEADER 1


extern uint16_t Drv8503ReadStatus(void);
extern uint16_t Drv8503ReadRegister(uint16_t addr);
extern uint16_t Drv8503SetRegister(uint16_t addr,uint16_t value);

extern uint16_t Drv8503Test(void);

// ------------------------------------------------------
// Register 0x5 :

// ------------------------------------------------------
// Register 0x9 : IC Operation

#define DRV8503_FLIP_OTSD  (1U<<10)
#define DRV8503_EN_SNS_CLAMP (1U<<7)
#define DRV8503_CLR_FLTS (1U<<2)

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


#endif
