#ifndef DRV8320_HEADER
#define DRV8320_HEADER 1

#ifdef __cplusplus
extern "C" {
#endif

extern uint16_t Drv8320ReadStatus(void);
extern uint16_t Drv8320ReadRegister(uint16_t addr);
extern uint16_t Drv8320SetRegister(uint16_t addr,uint16_t value);

extern uint16_t Drv8320Test(void);
extern void InitDrv8320(void);

// ------------------------------------------------------
// Register 0x1 :

#define DRV8320_REG_FAULT1     (0x0)

#define DRV8320_FAULT1_FAULT      (1U<<10)
#define DRV8320_FAULT1_VDS_OCP    (1U<<9)
#define DRV8320_FAULT1_GDF        (1U<<8)
#define DRV8320_FAULT1_PVDD_UVFL  (1U<<7)
#define DRV8320_FAULT1_OTSD       (1U<<6)
#define DRV8320_FAULT1_VDS_HA     (1U<<5)

#define DRV8320_FAULT1_VDS_LA     (1U<<4)
#define DRV8320_FAULT1_VDS_HB     (1U<<3)
#define DRV8320_FAULT1_VDS_LB     (1U<<2)
#define DRV8320_FAULT1_VDS_HC     (1U<<1)
#define DRV8320_FAULT1_VDS_LC     (1U<<0)

#define DRV8320_REG_FAULT2     (0x1)

#define DRV8320_FAULT2_SA_OC      (1U<<10)
#define DRV8320_FAULT2_SB_OC      (1U<<9)
#define DRV8320_FAULT2_SC_OC      (1U<<8)
#define DRV8320_FAULT2_OTW        (1U<<7)
#define DRV8320_FAULT2_CPUV       (1U<<6)
#define DRV8320_FAULT2_VGS_HA     (1U<<5)

#define DRV8320_FAULT2_VGS_LA     (1U<<4)
#define DRV8320_FAULT2_VGS_HB     (1U<<3)
#define DRV8320_FAULT2_VGS_LB     (1U<<2)
#define DRV8320_FAULT2_VGS_HC     (1U<<1)
#define DRV8320_FAULT2_VGS_LC     (1U<<0)


// ------------------------------------------------------
// Register 0x2 : Drive Control


#define DRV8320_REG_DRIVE_CONTROL   (0x2)

#define DRV8320_DC_DIS_CPUV       (1U<<9)
#define DRV8320_DC_DIS_GDF        (1U<<8)
#define DRV8320_DC_OTW_REP        (1U<<7)

#define DRV8320_DC_PWM6x          (00U<<5)
#define DRV8320_DC_PWM3x          (01U<<5)
#define DRV8320_DC_PWM1x          (10U<<5)
#define DRV8320_DC_PWMIND         (11U<<5)

#define DRV8320_DC_1PWM_COM       (1U<<4)
#define DRV8320_DC_1PWM_DIR       (1U<<3)
#define DRV8320_DC_COAST          (1U<<2)
#define DRV8320_DC_BRAKE          (1U<<1)

#define DRV8320_DC_CLR_FLTS      (1U<<0)

// ------------------------------------------------------
// Register 0x3 : Gate Drive high side

#define DRV8320_GATE_DRIVE_HS   (0x3)

#define DRV8320_GATEDRIVE_LOCK                      (0b110 << 8)
#define DRV8320_GATEDRIVE_UNLOCK                    (0b011 << 8)

#define DRV8320_PEAK_SOURCE_10mA          (0b0000 << 4)
#define DRV8320_PEAK_SOURCE_30mA          (0b0001 << 4)
#define DRV8320_PEAK_SOURCE_60mA          (0b0010 << 4)
#define DRV8320_PEAK_SOURCE_80mA          (0b0011 << 4)
#define DRV8320_PEAK_SOURCE_120mA         (0b0100 << 4)
#define DRV8320_PEAK_SOURCE_140mA         (0b0101 << 4)
#define DRV8320_PEAK_SOURCE_170mA         (0b0110 << 4)
#define DRV8320_PEAK_SOURCE_190mA         (0b0111 << 4)
#define DRV8320_PEAK_SOURCE_260mA         (0b1000 << 4)
#define DRV8320_PEAK_SOURCE_330mA         (0b1001 << 4)
#define DRV8320_PEAK_SOURCE_370mA         (0b1010 << 4)
#define DRV8320_PEAK_SOURCE_440mA         (0b1011 << 4)
#define DRV8320_PEAK_SOURCE_570mA         (0b1100 << 4)
#define DRV8320_PEAK_SOURCE_680mA         (0b1101 << 4)
#define DRV8320_PEAK_SOURCE_820mA         (0b1110 << 4)
#define DRV8320_PEAK_SOURCE_1000mA        (0b1111 << 4)


#define DRV8320_PEAK_SINK_20mA        (0b0000 << 0)
#define DRV8320_PEAK_SINK_60mA        (0b0001 << 0)
#define DRV8320_PEAK_SINK_120mA       (0b0010 << 0)
#define DRV8320_PEAK_SINK_160mA       (0b0011 << 0)
#define DRV8320_PEAK_SINK_240mA       (0b0100 << 0)
#define DRV8320_PEAK_SINK_280mA       (0b0101 << 0)
#define DRV8320_PEAK_SINK_340mA       (0b0110 << 0)
#define DRV8320_PEAK_SINK_380mA       (0b0111 << 0)
#define DRV8320_PEAK_SINK_520mA       (0b1000 << 0)
#define DRV8320_PEAK_SINK_660mA       (0b1001 << 0)
#define DRV8320_PEAK_SINK_740mA       (0b1010 << 0)
#define DRV8320_PEAK_SINK_880mA       (0b1011 << 0)
#define DRV8320_PEAK_SINK_1140mA      (0b1100 << 0)
#define DRV8320_PEAK_SINK_1360mA      (0b1101 << 0)
#define DRV8320_PEAK_SINK_1640mA      (0b1110 << 0)
#define DRV8320_PEAK_SINK_2000mA      (0b1111 << 0)

// ------------------------------------------------------
// Register 0x4 : Gate Drive low side

#define DRV8320_GATE_DRIVE_LS   (0x4)

#define DRV8320_DRIVE_TIME_500ns       (0b00 << 8)
#define DRV8320_DRIVE_TIME_1000ns      (0b01 << 8)
#define DRV8320_DRIVE_TIME_2000ns      (0b10 << 8)
#define DRV8320_DRIVE_TIME_4000ns      (0b11 << 8)

// ------------------------------------------------------
// Register 0x5 : OCP Control

#define DRV8320_OCP_CONTROL   (0x5)

#define DRV8320_TRETRY_4ms           (0b0 << 10)
#define DRV8320_TRETRY_50us          (0b1 << 10)


#define DRV8320_DEAD_TIME_50ns       (0b00 << 8)
#define DRV8320_DEAD_TIME_100ns      (0b01 << 8)
#define DRV8320_DEAD_TIME_200ns      (0b10 << 8)
#define DRV8320_DEAD_TIME_400ns      (0b11 << 8)

#define DRV8320_OCP_MODE_LATCHED     (0b00 << 6)
#define DRV8320_OCP_MODE_RETRY       (0b01 << 6)
#define DRV8320_OCP_MODE_REPORT      (0b10 << 6)
#define DRV8320_OCP_MODE_DISABLE     (0b11 << 6)

#define DRV8320_OCP_DEG_2us          (0b00 << 4)
#define DRV8320_OCP_DEG_4us          (0b01 << 4)
#define DRV8320_OCP_DEG_6us          (0b10 << 4)
#define DRV8320_OCP_DEG_8us          (0b11 << 4)

#define DRV8320_VDS_LVL_0V06         (0b0000 << 0)
#define DRV8320_VDS_LVL_0V13         (0b0001 << 0)
#define DRV8320_VDS_LVL_0V20         (0b0010 << 0)
#define DRV8320_VDS_LVL_0V26         (0b0011 << 0)
#define DRV8320_VDS_LVL_0V31         (0b0100 << 0)
#define DRV8320_VDS_LVL_0V45         (0b0101 << 0)
#define DRV8320_VDS_LVL_0V53         (0b0110 << 0)
#define DRV8320_VDS_LVL_0V60         (0b0111 << 0)
#define DRV8320_VDS_LVL_0V68         (0b1000 << 0)
#define DRV8320_VDS_LVL_0V75         (0b1001 << 0)
#define DRV8320_VDS_LVL_0V94         (0b1010 << 3)
#define DRV8320_VDS_LVL_1V13         (0b1011 << 3)
#define DRV8320_VDS_LVL_1V30         (0b1100 << 3)
#define DRV8320_VDS_LVL_1V50         (0b1101 << 3)
#define DRV8320_VDS_LVL_1V70         (0b1110 << 3)
#define DRV8320_VDS_LVL_1V88         (0b1111 << 3)

#ifdef __cplusplus
}
#endif

#endif
