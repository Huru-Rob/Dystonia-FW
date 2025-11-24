#ifndef __AFE4960
#define __AFE4960

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrfx_gpiote.h"
#include "board_config.h"
#include "AFE4960_ex.h"

#define AFE_DEBUG_OUT
#define AFE_I2C_ADD 0x5A

#define EEG1_INA_GAIN (INA_GAIN_12)
#define EEG2_INA_GAIN (BIOZ_INA_GAIN_2)
#undef ECG1_SHORTED
#undef ECG2_SHORTED


#define AFE_COMMON_CONTROL0     (0<<FIFO_EN_BITPOS)
#define AFE_COMMON_CONTROL2     (3<<INT_MUX_GPIO2_1_BITPOS)
#define AFE_PASSIVE_CONFIG_MODE_SEL ( \
                                (1<<EN_ECG_RX_BITPOS) | \
                                (0x4A << ACQ_MODE_SEL_BITPOS) ) 
#define AFE_ACTIVE_CONFIG_MODE_SEL ( \
                                (1<<EN_ECG_RX_BITPOS) | \
                                (0x4A << ACQ_MODE_SEL_BITPOS) | \
                                (1<<EN_BIOZ_RX_BITPOS) | \
                                (1<<EN_BIOZ_TX_BITPOS) \
                            )
#define AFE_COMMON_ACT_CTRL 0x000000
#define AFE_COMMON_CTRL_PDN     ( \
                                (1<<DIS_PD_VCM_BITPOS) | \
                                (1<<DIS_PD_REFSYS_BITPOS) | \
                                (1<<DIS_ACTIVE_BIOZ_TX_BITPOS) \
                                )

#define AFE_COMMON_RAC_CONTROL  ( \
                                (0<<TIMER_ENABLE_BITPOS) | \
                                (0<<RAC_COUNTER_ENABLE_BITPOS))

#define AFE_COMMON_CONTROL4    0x000000
// #define AFE_COMMON_DESIGNID 0x000000 //readonly reg
#define AFE_COMMON_CONTROL7     (1<<EN_GPIO2_OUT_BITPOS)
#define AFE_COMMON_CONTROL8    0x000000 
#define AFE_COMMON_WATERMARK_LEVEL     (2)
#define AFE_COMMON_CONTROL9 ( \
                                ((AFE_COMMON_WATERMARK_LEVEL-1)<<REG_WM_FIFO_BITPOS) | \
                                (2<<INT_MUX_ADC_RDY_1_BITPOS))
#define AFE_COMMON_CONTROL10    (1<<DIS_BUF_PDN_ON_ADC_BITPOS)
#define AFE_COMMON_CONTROL13   0x000000
#define AFE_COMMON_CONTROL19 0x000000
#define AFE_COMMON_CONTROL20 ( \
                                (1<<MODE_EN_FRAME_SYNC_BITPOS) \
                            )
// #define AFE_COMMON_READ_AC_LEAD 0x000000 //read only 
#define AFE_COMMON_AC_DEMOD ( \
                                (0x66<<AC_LEAD_DEMOD_CFG_BITPOS) | \
                                (0xAAAA<<ECG_SAMPLE_SIGN_BITPOS))
#define AFE_COMMON_MASK_INT_REG ( \
                                (1<<MASK_PACE_VALID_INT_BITPOS) | \
                                (1<<MASK_DISABLE2_BITPOS) | \
                                (1<<MASK_DISABLE1_BITPOS) | \
                                (1<<MASK_AC_LEAD_OFF_BITPOS) | \
                                (1<<MASK_AC_LEAD_ON_BITPOS) | \
                                (MASK_DC_LEAD_DET_BITPOS))

#define AFE_COMMON_CONTROL25   ( \
                                (0<<EN_DATA_MARKER_BITPOS) \
                            )

#if AFE_CLOCK_SYNC == 1
#define EN_OSCL_SYNC 1
#else
#define EN_OSCL_SYNC 0
#endif

#define AFE_COMMON_CONFIG_CLK_CKT_MIX1 ( \
                                (EN_OSCL_SYNC<<EN_OSCL_SYNC_BITPOS) \
                            )
#define AFE_COMMON_CONFIG_CLK_DIV_MIX1 0x000000

#if AFE_OVERSAMPLING == 1
#define EN_DEC 1
#define DEC_FACTOR (DEC_FACTOR_2)
#define COUNT_RAC (COUNT_RAC_1000SA_SEC)
#else
#define EN_DEC 0
#define DEC_FACTOR (DEC_FACTOR_4)
#define COUNT_RAC (COUNT_RAC_1000SA_SEC)
#endif

#define AFE_COMMON_CONFIG_PRPCT_MIX1 (COUNT_RAC<<COUNT_RAC_BITPOS) 

#define AFE_PASSIVE_CONFIG_TS_1TO8_MIX1 ( \
                                (CONFIG_TSM_ECG<<CONFIG_TS0_BITPOS) | \
                                (CONFIG_TSM_DUMMY<<CONFIG_TS1_BITPOS) | \
                                (CONFIG_TSM_DUMMY<<CONFIG_TS2_BITPOS) | \
                                (CONFIG_TSM_DUMMY<<CONFIG_TS3_BITPOS) \
                            )
#define AFE_ACTIVE_CONFIG_TS_1TO8_MIX1 ( \
                                (CONFIG_TSM_ECG<<CONFIG_TS0_BITPOS) | \
                                (CONFIG_TSM_DUMMY<<CONFIG_TS1_BITPOS) | \
                                (CONFIG_TSM_BIOZ<<CONFIG_TS2_BITPOS) | \
                                (CONFIG_TSM_DUMMY<<CONFIG_TS3_BITPOS) \
                            )

#define AFE_COMMON_CONFIG_TS_9TO16_MIX1 0x000000

#define NUM_BSAW (1)
#define NUM_ESAW (1)
#define NUM_TS (4)
#define REG_NUM_ESAW (NUM_ESAW-1)
#define REG_NUM_BSAW (NUM_BSAW-1)
#define REG_NUM_TS (NUM_TS-1)
#define AFE_COMMON_CONFIG_NUM_TS_MIX1 ( \
                                (REG_NUM_TS<<REG_NUM_TS_BITPOS) | \
                                (REG_NUM_ESAW<<REG_NUM_ESAW_BITPOS) | \
                                (REG_NUM_BSAW<<REG_NUM_BSAW_BITPOS) \
                                )
#ifdef ECG1_SHORTED
#define SHORT_ECG_INM_RLD (1) // Short the channel inputs to RLD
#define SHORT_ECG_INP_RLD (1)
#define SW_ECG_INP_RESP (0) // No RESP inputs are selected when the channel is shorted
#define SW_ECG_INM_RESP (0)
#else
#define SHORT_ECG_INM_RLD (0) // Don't short the channel inputs to RLD
#define SHORT_ECG_INP_RLD (0)
#define SW_ECG_INP_RESP (SW_RESP1) // Connect the channel to RESP1,2 
#define SW_ECG_INM_RESP (SW_RESP2)
#endif
#define AFE_COMMON_CONFIG_ECG1_MIX1 ( \
                                (EEG1_INA_GAIN<<ECG_INA_GAIN_BITPOS) | \
                                (SW_ECG4<<SW_RLD_BITPOS) | \
                                (0<<SW_ECG_INM_ECG_BITPOS) | \
                                (0<<SW_ECG_INP_ECG_BITPOS) | \
                                (1<<CONFIG_RLD_AS_UGB_BITPOS) | \
                                (SHORT_ECG_INM_RLD<<SHORT_ECG_INM_RLD_BITPOS) | \
                                (SHORT_ECG_INP_RLD<<SHORT_ECG_INP_RLD_BITPOS) \
                                )

#define AFE_COMMON_CONFIG_ECG2_MIX1 ( \
                                (EN_DEC<<EN_DEC_ECG_BITPOS) | \
                                (DEC_FACTOR<<ECG_DEC_FACTOR_BITPOS) | \
                                (SW_ECG_INP_RESP<<SW_ECG_INP_RESP_BITPOS) | \
                                (SW_ECG_INM_RESP<<SW_ECG_INM_RESP_BITPOS) \
                                )

#define AFE_PASSIVE_CONFIG_ECG_CH2 ( \
                                (0<<EN_ECG2_BITPOS) | \
                                (1<<SEL_IN_BIOZ_RX_BITPOS) | \
                                (1<<RECONFIGURE_BIOZ_LPF_BITPOS) \
                                )
#define AFE_ACTIVE_CONFIG_ECG_CH2 ( \
                                (1<<EN_ECG2_BITPOS) | \
                                (1<<SEL_IN_BIOZ_RX_BITPOS) | \
                                (1<<RECONFIGURE_BIOZ_LPF_BITPOS) \
                                )

#ifdef ECG2_SHORTED
#define SW_BIOZ_RXP (0) // Connect BIOZ_RXP to VCM when shorting
#define SW_BIOZ_RXM (0) // Connect BIOZ_RXM to VCM when shorting
#define SHORT_BIOZ_INP_RLD (1) // Also short BIOZ_INP to RLD
#define SHORT_BIOZ_INM_RLD (1) // Also short BIOZ_INM to RLD
#else
#define SW_BIOZ_RXP (SW_BIOZ_IN_ECG1) // Connect channel to ECG1,2 
#define SW_BIOZ_RXM (SW_BIOZ_IN_ECG2)
#define SHORT_BIOZ_INP_RLD (0) // Don't short to RLD
#define SHORT_BIOZ_INM_RLD (0)
#endif
#define AFE_COMMON_CONFIG_BIA_SW_MATRIX_STATIC_MIX1 ( \
                                (SW_BIOZ_RXP<<SW_BIOZ_RXP_BITPOS) | \
                                (SW_BIOZ_RXM<<SW_BIOZ_RXM_BITPOS) \
                                )
#define AFE_COMMON_CONFIG_BIA_EXC_MIX1 0x000000 
#define AFE_COMMON_CONFIG_BIA_TX_MIX1 0x000000

#define AFE_COMMON_CONFIG_BIA_RX_MIX1 ( \
                                (SHORT_BIOZ_INP_RLD<<SHORT_BIOZ_INP_RLD_BITPOS) | \
                                (SHORT_BIOZ_INM_RLD<<SHORT_BIOZ_INM_RLD_BITPOS) | \
                                (EEG2_INA_GAIN<<BIOZ_INA_GAIN_BITPOS) | \
                                (BIOZ_LPF_BW_370HZ<<BIOZ_LPF_BW_BITPOS) | \
                                (1<<BYP_CHPF1_BIOZ_BITPOS) | \
                                (1<<DIS_RHPF1_BIOZ_BITPOS) \
                                ) 
#define AFE_PASSIVE_CONFIG_BIA_PROC_MIX1 ( \
                                (EN_DEC<<EN_DEC_BIOZ_BITPOS) | \
                                (DEC_FACTOR<<BIOZ_DEC_FACTOR_BITPOS) \
                            ) 
#define AFE_ACTIVE_CONFIG_BIA_PROC_MIX1 ( \
                                (EN_DEC<<EN_DEC_BIOZ_BITPOS) | \
                                (DEC_FACTOR<<BIOZ_DEC_FACTOR_BITPOS) | \
                                (3<<PROCESS_BSAW_BITPOS) \
                            )
#define AFE_COMMON_CONFIG_ELECTRODE_BIAS1_MIX1 ( \
                                (1<<SW_RBIAS1_ECGP_BITPOS) | \
                                (1<<SW_RBIAS2_ECGM_BITPOS) | \
                                (1<<SW_RBIAS1_RLD_BITPOS) | \
                                (1<<SW_RBIAS2_RLD_BITPOS) \
                            )
#define AFE_COMMON_CONFIG_ELECTRODE_BIAS2_MIX1 ( \
                                (RBIAS_25MOHM<<SEL_RBIAS_CH1_BITPOS)  \
                            )
                                

#define LEAD_DET_WIDTH   (COUNT_RAC_TO_LEAD_DET_WIDTH(COUNT_RAC, NUM_ESAW))
#define AFE_COMMON_CONFIG_COMP_ANA_MIX1 ( \
                                (LEAD_DET_THR_H_1_20V<<LEAD_DET_THR_H_BITPOS) | \
                                (LEAD_DET_THR_L_0_60V<<LEAD_DET_THR_L_BITPOS) | \
                                ((LEAD_DET_WIDTH<<LEAD_DET_WIDTH_BITPOS)&LEAD_DET_WIDTH_MASK) | \
                                (1<<AC_LEAD_DET_CLK_PHASE_BITPOS) \
                                )
#define AFE_COMMON_CONFIG_IACTIVE_MIX1 ( \
                                (1<<EN_IBIAS_BITPOS) | \
                                (1<<EN_AC_LEAD_DET_CLK_BITPOS) | \
                                (1<<SW_IBIAS_AC_BITPOS) | \
                                (IBIAS_50NA<<IBIAS_AMPL_BITPOS) | \
                                (IBIAS_BIAS_AMPL_FOR_50NA<<IBIAS_BIAS_AMPL_BITPOS) \
                            )

#define AC_LEAD_DET_THR_H   (0x300) // Pure guess
#define AC_LEAD_DET_THR_L   (0x100) // Pure guess
#define AFE_COMMON_CONFIG_COMP_DIG_MIX1 ( \
                                (AC_LEAD_DET_THR_H<<AC_LEAD_DET_THR_H_BITPOS) | \
                                (AC_LEAD_DET_THR_L<<AC_LEAD_DET_THR_L_BITPOS) )

// #define AFE_COMMON_CONFIG_PACE_DETECT 0x000000
// #define AFE_COMMON_CONFIG_PACE_SIG_CHAIN 0x000000
// #define AFE_COMMON_CONFIG_PACE_DIG_REG1 0x000000
// #define AFE_COMMON_CONFIG_PACE_DIG_REG2 0x000000
// #define AFE_COMMON_CONFIG_PACE_DIG_REG3 0x000000
// #define AFE_COMMON_READ_PACE_STATUS 0x000000 //read only
// #define AFE_COMMON_READ_PACE_STATUS2 0x000000 //read only
// #define AFE_COMMON_CONFIG_DDS 0x000000

#define AFE_COMMON_CONFIG_AC_DEMOD ( \
                                (0x66<<AC_LEAD_DEMOD_CFG_BITPOS) | \
                                (0xAAAA<<ECG_SAMPLE_SIGN_BITPOS) \
                            )


#define AFE_FIFO_SIZE (128) //samples

typedef struct {
    uint8_t     addr;
    uint32_t    value;
} afe_init_t;


#define SCALE_AMPLITUDE_TO_KOHMS (11.623/1000.0)
#define AFE_GOOD_CONNECTION_RESISTANCE_KOHMS (100.0)
#define AFE_GOOD_CONNECTION_AMPLITUDE (AFE_GOOD_CONNECTION_RESISTANCE_KOHMS / SCALE_AMPLITUDE_TO_KOHMS)
#define AC_LEAD_DETECT_VALID_BITPOS (0)
#define AFE_MODE_BITPOS (1)
#define AFE_MODE_MASK (3<<AFE_MODE_BITPOS)
#define DC_LEAD_DETECT_VALID_BITPOS (4)
#define DC_LEAD_DETECT_ELECTRODE_1_BITPOS (6)
#define DC_LEAD_DETECT_ELECTRODE_2_BITPOS (7)


int32_t AFE4960_init(afe_mode_e mode);
int32_t AFE4960_write_reg(uint8_t addr, uint32_t value);
int32_t AFE4960_read_reg(uint8_t addr, uint32_t *value);

#endif