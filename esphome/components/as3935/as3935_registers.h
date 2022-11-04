#pragma once

namespace esphome {
namespace as3935 {

/* ----- Registers configs ----- */
/* register 0x00 */
constexpr uint8_t AFE_GB_REG_NB = 0x00;
constexpr uint8_t AFE_GB_MASK = 0x3E;
constexpr uint8_t AFE_GB_BIT_POS = 1U;
/* AFE_GB values */
constexpr uint8_t AFE_GB_VAL_INDOOR = 18U;
constexpr uint8_t AFE_GB_VAL_OUTDOOR = 14U;

/* register 0x01 */
constexpr uint8_t NF_LEV_REG_NB = 0x01;
constexpr uint8_t NF_LEV_MASK = 0x70;
constexpr uint8_t NF_LEV_BIT_POS = 4U;

constexpr uint8_t WDTH_REG_NB = 0x01;
constexpr uint8_t WDTH_MASK = 0x0F;
constexpr uint8_t WDTH_BIT_POS = 0U;

/* register 0x02 */
constexpr uint8_t CL_STAT_REG_NB = 0x02;
constexpr uint8_t CL_STAT_MASK = 0x40;
constexpr uint8_t CL_STAT_BIT_POS = 6U;

constexpr uint8_t MIN_NUM_LIGH_REG_NB = 0x02;
constexpr uint8_t MIN_NUM_LIGH_MASK = 0x30;
constexpr uint8_t MIN_NUM_LIGH_BIT_POS = 4U;

constexpr uint8_t SREJ_REG_NB = 0x02;
constexpr uint8_t SREJ_MASK = 0x0F;
constexpr uint8_t SREJ_BIT_POS = 0U;

/* register 0x03 */
constexpr uint8_t LCO_FDIV_REG_NB = 0x03;
constexpr uint8_t LCO_FDIV_MASK = 0xC0;
constexpr uint8_t LCO_FDIV_BIT_POS = 6U;

constexpr uint8_t MASK_DIST_REG_NB = 0x03;
constexpr uint8_t MASK_DIST_MASK = 0x20;
constexpr uint8_t MASK_DIST_BIT_POS = 5U;

constexpr uint8_t INT_REG_NB = 0x03;
constexpr uint8_t INT_MASK = 0x0F;
constexpr uint8_t INT_BIT_POS = 0U;
/* flag position (on bitfield) */
constexpr uint8_t INT_NH_FLAG_POS = 0x01;
constexpr uint8_t INT_D_FLAG_POS = 0x04;
constexpr uint8_t INT_L_FLAG_POS = 0x08;
constexpr uint8_t INT_ALL_FLAGS_MASK = INT_NH_FLAG_POS + INT_D_FLAG_POS + INT_L_FLAG_POS;


/* register 0x04 */
constexpr uint8_t S_SIG_L_REG_NB = 0x04;
constexpr uint8_t S_SIG_L_MASK = 0xFF;
constexpr uint8_t S_SIG_L_BIT_POS = 0U;

/* register 0x05 */
constexpr uint8_t S_SIG_M_REG_NB = 0x05;
constexpr uint8_t S_SIG_M_MASK = 0xFF;
constexpr uint8_t S_SIG_M_BIT_POS = 0U;

/* register 0x06 */
constexpr uint8_t S_SIG_MM_REG_NB = 0x06;
constexpr uint8_t S_SIG_MM_MASK = 0x1F;
constexpr uint8_t S_SIG_MM_BIT_POS = 0U;

/* register 0x07 */
constexpr uint8_t DISTANCE_REG_NB = 0x07;
constexpr uint8_t DISTANCE_MASK = 0x3F;
constexpr uint8_t DISTANCE_BIT_POS = 0U;

/* register 0x08 */
constexpr uint8_t DISP_xxCO_REG_NB = 0x08;
constexpr uint8_t DISP_xxCO_MASK = 0xE0;
constexpr uint8_t DISP_xxCO_BIT_POS = 5U;

constexpr uint8_t DISP_TUN_CAP_REG_NB = 0x08;
constexpr uint8_t TUN_CAP_MASK = 0x0F;
constexpr uint8_t TUN_CAP_BIT_POS = 0U;

/* register 0x3A */
constexpr uint8_t TRCO_CALIB_REG_NB = 0x3A;
constexpr uint8_t TRCO_CALIB_MASK = 0xC0;
constexpr uint8_t TRCO_CALIB_BIT_POS = 6U;

/* register 0x3B */
constexpr uint8_t SRCO_CALIB_REG_NB = 0x3B;
constexpr uint8_t SRCO_CALIB_MASK = 0xC0;
constexpr uint8_t SRCO_CALIB_BIT_POS = 6U;

}  // namespace as3935
}  // namespace esphome
