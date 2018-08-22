#ifndef DRV8305_HEADER
#define DRV8305_HEADER 1

#ifdef __cplusplus
extern "C" {
#endif

extern uint16_t Drv8305ReadStatus(void);
extern uint16_t Drv8305ReadRegister(uint16_t addr);
extern uint16_t Drv8305SetRegister(uint16_t addr,uint16_t value);

extern uint16_t Drv8305Test(void);
extern void InitDrv8305(void);

// ------------------------------------------------------
// Register 0x1 :

#define DRV8305_REG_WARNING     (0x1)

#define DRV8305_WARN_FAULT      (1U<<10)
#define DRV8305_WARN_TEMP_175C  (1U<<8)
#define DRV8305_WARN_PVDD_UVFL  (1U<<7)
#define DRV8305_WARN_PVDD_OVFL  (1U<<6)
#define DRV8305_WARN_VDS_STATUS (1U<<5)

#define DRV8305_WARN_VCHP_UVFL  (1U<<4)
#define DRV8305_WARN_TEMP_105C  (1U<<3)
#define DRV8305_WARN_TEMP_125C  (1U<<2)
#define DRV8305_WARN_TEMP_135C  (1U<<1)
#define DRV8305_WARN_OVERTEMP   (1U<<0)

// ------------------------------------------------------
// Register 0x5 :


// ------------------------------------------------------
// Register 0x9 : IC Operation


#define DRV8305_REG_IC_CONTROL   (0x9)

#define DRV8305_IC_VCPH_UV_4V9   (0U)
#define DRV8305_IC_VCPH_UV_4V6   (1U)
#define DRV8305_IC_CLR_FLTS      (1U<<1)
#define DRV8305_IC_SLEEP         (1U<<2)
#define DRV8305_IC_WD_EN         (1U<<3)
#define DRV8305_IC_DIS_SNS_OCP   (1U<<4)

#define DRV8305_IC_WD_DLY_10ms  (0b00 << 5)
#define DRV8305_IC_WD_DLY_20ms  (0b01 << 5)
#define DRV8305_IC_WD_DLY_50ms  (0b10 << 5)
#define DRV8305_IC_WD_DLY_100ms (0b11 << 5)

#define DRV8305_IC_EN_SNS_CLAMP   (1U<<7)
#define DRV8305_IC_DIS_GDRV_FAULT (1U<<8)
#define DRV8305_IC_DIS_PVDD_UVLO2 (1U<<9)
#define DRV8305_IC_FLIP_OTSD    (1U<<10)

// ------------------------------------------------------
// Register 0xA : Shunt Amplifier Control
#define DRV8305_REG_SHUNT_AMPLIFIER_CONTROL (0xAU)

// Blanking
#define DRV8305_CS_BLANK_OFF   (0U)
#define DRV8305_CS_BLANK_0_5US (1U << 6)  // 0.5 us
#define DRV8305_CS_BLANK_2_5US (2U << 6)   // 2.5 us
#define DRV8305_CS_BLANK_10US  (3U << 6)   // 10us

// Cal mode for each amplifier, sets input to 0V to calibrate the DC offset.

#define DRV8305_DC_CAL_CH1 (1U<<8)
#define DRV8305_DC_CAL_CH2 (1U<<9)
#define DRV8305_DC_CAL_CH3 (1U<<10)

// Gain for shunt 3

#define DRV8305_GAIN_CS3_10 (0U<<4)
#define DRV8305_GAIN_CS3_20 (1U<<4)
#define DRV8305_GAIN_CS3_40 (2U<<4)
#define DRV8305_GAIN_CS3_80 (3U<<4)

// Gain for shunt 2

#define DRV8305_GAIN_CS2_10 (0U<<2)
#define DRV8305_GAIN_CS2_20 (1U<<2)
#define DRV8305_GAIN_CS2_40 (2U<<2)
#define DRV8305_GAIN_CS2_80 (3U<<2)

// Gain for shunt 1  V/V

#define DRV8305_GAIN_CS1_10 (0U)
#define DRV8305_GAIN_CS1_20 (1U)
#define DRV8305_GAIN_CS1_40 (2U)
#define DRV8305_GAIN_CS1_80 (3U)

#define DRV8305_PEAK_SINK_20mA        (0b0000 << 4)
#define DRV8305_PEAK_SINK_30mA        (0b0001 << 4)
#define DRV8305_PEAK_SINK_40mA        (0b0010 << 4)
#define DRV8305_PEAK_SINK_50mA        (0b0011 << 4)
#define DRV8305_PEAK_SINK_60mA        (0b0100 << 4)
#define DRV8305_PEAK_SINK_70mA        (0b0101 << 4)
#define DRV8305_PEAK_SINK_80mA        (0b0110 << 4)
#define DRV8305_PEAK_SINK_250mA       (0b0111 << 4)
#define DRV8305_PEAK_SINK_500mA       (0b1000 << 4)
#define DRV8305_PEAK_SINK_750mA       (0b1001 << 4)
#define DRV8305_PEAK_SINK_1000mA      (0b1010 << 4)
#define DRV8305_PEAK_SINK_1250mA      (0b1011 << 4)

#define DRV8305_PEAK_SOURCE_10mA          (0b0000)
#define DRV8305_PEAK_SOURCE_20mA          (0b0001)
#define DRV8305_PEAK_SOURCE_30mA          (0b0010)
#define DRV8305_PEAK_SOURCE_40mA          (0b0011)
#define DRV8305_PEAK_SOURCE_50mA          (0b0100)
#define DRV8305_PEAK_SOURCE_60mA          (0b0101)
#define DRV8305_PEAK_SOURCE_70mA          (0b0110)
#define DRV8305_PEAK_SOURCE_125mA         (0b0111)
#define DRV8305_PEAK_SOURCE_250mA         (0b1000)
#define DRV8305_PEAK_SOURCE_500mA         (0b1001)
#define DRV8305_PEAK_SOURCE_750mA         (0b1010)
#define DRV8305_PEAK_SOURCE_1000mA        (0b1011)

#define DRV8305_SOURCE_TIME_220ns       (0b00 << 8)
#define DRV8305_SOURCE_TIME_440ns       (0b01 << 8)
#define DRV8305_SOURCE_TIME_880ns       (0b10 << 8)
#define DRV8305_SOURCE_TIME_1780ns      (0b11 << 8)

#define DRV8305_REG_DRIVER_HS     (0x5)
#define DRV8305_REG_DRIVER_LS     (0x6)


#define DRV8305_VDS_THRESHOLD_0V060         (0b00000 << 3)
#define DRV8305_VDS_THRESHOLD_0V068         (0b00001 << 3)
#define DRV8305_VDS_THRESHOLD_0V076         (0b00010 << 3)
#define DRV8305_VDS_THRESHOLD_0V086         (0b00011 << 3)
#define DRV8305_VDS_THRESHOLD_0V097         (0b00100 << 3)
#define DRV8305_VDS_THRESHOLD_0V109         (0b00101 << 3)
#define DRV8305_VDS_THRESHOLD_0V123         (0b00110 << 3)
#define DRV8305_VDS_THRESHOLD_0V138         (0b00111 << 3)
#define DRV8305_VDS_THRESHOLD_0V155         (0b01000 << 3)
#define DRV8305_VDS_THRESHOLD_0V175         (0b01001 << 3)
#define DRV8305_VDS_THRESHOLD_0V197         (0b01010 << 3)
#define DRV8305_VDS_THRESHOLD_0V222         (0b01011 << 3)
#define DRV8305_VDS_THRESHOLD_0V250         (0b01100 << 3)
#define DRV8305_VDS_THRESHOLD_0V282         (0b01101 << 3)
#define DRV8305_VDS_THRESHOLD_0V317         (0b01110 << 3)
#define DRV8305_VDS_THRESHOLD_0V358         (0b01111 << 3)
#define DRV8305_VDS_THRESHOLD_0V403         (0b10000 << 3)
#define DRV8305_VDS_THRESHOLD_0V454         (0b10001 << 3)
#define DRV8305_VDS_THRESHOLD_0V511         (0b10010 << 3)
#define DRV8305_VDS_THRESHOLD_0V576         (0b10011 << 3)
#define DRV8305_VDS_THRESHOLD_0V648         (0b10100 << 3)
#define DRV8305_VDS_THRESHOLD_0V730         (0b10101 << 3)
#define DRV8305_VDS_THRESHOLD_0V822         (0b10110 << 3)
#define DRV8305_VDS_THRESHOLD_0V926         (0b10111 << 3)
#define DRV8305_VDS_THRESHOLD_1V043         (0b11000 << 3)
#define DRV8305_VDS_THRESHOLD_1V175         (0b11001 << 3)
#define DRV8305_VDS_THRESHOLD_1V324         (0b11010 << 3)
#define DRV8305_VDS_THRESHOLD_1V491         (0b11011 << 3)
#define DRV8305_VDS_THRESHOLD_1V679         (0b11100 << 3)
#define DRV8305_VDS_THRESHOLD_1V892         (0b11101 << 3)
#define DRV8305_VDS_THRESHOLD_2V131         (0b11110 << 3)

#define DRV8305_VDS_SHUT_DOWN               (0b000)
#define DRV8305_VDS_REPORT_ONLY             (0b001)
#define DRV8305_VDS_PROTECTION_DISABLE      (0b010)

#define DRV8305_REG_GATE_DRIVE_CONTROL (0x7)


#define DRV8305_COMM_OPTION_ACTIVE_FREE_WHEEL   (0b1 << 9)
#define DRV8305_COMM_OPTION_DIODE_FREE_WHEEL    (0b0 << 9)

#define DRV8305_PWM_MODE_6_INPUT (0b00 << 7)
#define DRV8305_PWM_MODE_3_INPUT (0b01 << 7)
#define DRV8305_PWM_MODE_1_INPUT (0b10 << 7)
#define DRV8305_PWM_MODE_6B_INPUT (0b11 << 7)

#define DRV8305_DEAD_TIME_35ns   (0b000 << 4)
#define DRV8305_DEAD_TIME_52ns   (0b001 << 4)
#define DRV8305_DEAD_TIME_88ns   (0b010 << 4)
#define DRV8305_DEAD_TIME_440ns  (0b011 << 4)
#define DRV8305_DEAD_TIME_880ns  (0b100 << 4)
#define DRV8305_DEAD_TIME_1760ns (0b101 << 4)
#define DRV8305_DEAD_TIME_3520ns (0b110 << 4)
#define DRV8305_DEAD_TIME_5280ns (0b111 << 4)

#define DRV8305_VDS_TBLANK_0us    (0b00 << 2)
#define DRV8305_VDS_TBLANK_1_75us (0b01 << 2)
#define DRV8305_VDS_TBLANK_3_5us  (0b10 << 2)
#define DRV8305_VDS_TBLANK_7us    (0b11 << 2)

#define DRV8305_VDS_TVDS_0us    (0b00)
#define DRV8305_VDS_TVDS_1_75us (0b01)
#define DRV8305_VDS_TVDS_3_5us  (0b10)
#define DRV8305_VDS_TVDS_7us    (0b11)

#define DRV8305_REG_VDS_SENSE_CONTROL     (0xC)

#ifdef __cplusplus
}
#endif

#endif
