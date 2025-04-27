#ifndef BQ40Z50_FG_H
#define BQ40Z50_FG_H

#define BQ25980_MANUFACTURER "Texas Instruments"

#define bq_info	pr_info
#define bq_dbg	pr_debug
#define bq_err	pr_info
#define bq_log	pr_err

#define INVALID_REG_ADDR       0xFF

#define FG_FLAGS_FD             BIT(4)   // Fully Discharged (完全耗尽状态)
#define FG_FLAGS_FC             BIT(5)   // Fully Charged
#define FG_FLAGS_DSG            BIT(6)   // Discharging or Relax
#define FG_FLAGS_RCA            BIT(9)   // Remaining Capacity Alarm

/* Manufacturer Access Commands */
#define BQ40Z50_CMD_MANUFACTURER_ACCESS              0x00
#define BQ40Z50_CMD_MANUFACTURER_BLOCK_ACCESS        0x44

/* Standard SBS Commands */
#define BQ40Z50_CMD_REMAINING_CAPACITY_ALARM         0x01
#define BQ40Z50_CMD_REMAINING_TIME_ALARM             0x02
#define BQ40Z50_CMD_BATTERY_MODE                     0x03
#define BQ40Z50_CMD_AT_RATE                          0x04
#define BQ40Z50_CMD_AT_RATE_TIME_TO_FULL             0x05
#define BQ40Z50_CMD_AT_RATE_TIME_TO_EMPTY            0x06
#define BQ40Z50_CMD_AT_RATE_OK                       0x07
#define BQ40Z50_CMD_TEMPERATURE                      0x08
#define BQ40Z50_CMD_VOLTAGE                          0x09
#define BQ40Z50_CMD_CURRENT                          0x0A
#define BQ40Z50_CMD_AVERAGE_CURRENT                  0x0B
#define BQ40Z50_CMD_MAX_ERROR                        0x0C
#define BQ40Z50_CMD_RELATIVE_STATE_OF_CHARGE         0x0D
#define BQ40Z50_CMD_ABSOLUTE_STATE_OF_CHARGE         0x0E
#define BQ40Z50_CMD_REMAINING_CAPACITY               0x0F
#define BQ40Z50_CMD_FULL_CHARGE_CAPACITY             0x10
#define BQ40Z50_CMD_RUN_TIME_TO_EMPTY                0x11
#define BQ40Z50_CMD_AVERAGE_TIME_TO_EMPTY            0x12
#define BQ40Z50_CMD_AVERAGE_TIME_TO_FULL             0x13
#define BQ40Z50_CMD_CHARGING_CURRENT                 0x14
#define BQ40Z50_CMD_CHARGING_VOLTAGE                 0x15
#define BQ40Z50_CMD_BATTERY_STATUS                   0x16
#define BQ40Z50_CMD_CYCLE_COUNT                      0x17
#define BQ40Z50_CMD_DESIGN_CAPACITY                  0x18
#define BQ40Z50_CMD_DESIGN_VOLTAGE                   0x19
#define BQ40Z50_CMD_SPECIFICATION_INFO               0x1A
#define BQ40Z50_CMD_MANUFACTURER_DATE                0x1B
#define BQ40Z50_CMD_SERIAL_NUMBER                    0x1C
#define BQ40Z50_CMD_MANUFACTURER_NAME                0x20
#define BQ40Z50_CMD_DEVICE_NAME                      0x21
#define BQ40Z50_CMD_DEVICE_CHEMISTRY                 0x22
#define BQ40Z50_CMD_MANUFACTURER_DATA                0x23
#define BQ40Z50_CMD_AUTHENTICATE                     0x2F
#define BQ40Z50_CMD_CELL_VOLTAGE_4                   0x3C
#define BQ40Z50_CMD_CELL_VOLTAGE_3                   0x3D
#define BQ40Z50_CMD_CELL_VOLTAGE_2                   0x3E
#define BQ40Z50_CMD_CELL_VOLTAGE_1                   0x3F
#define BQ40Z50_CMD_BTP_DISCHARGE_SET                0x4A
#define BQ40Z50_CMD_BTP_CHARGE_SET                   0x4B
#define BQ40Z50_CMD_STATE_OF_HEALTH                  0x4F
#define BQ40Z50_CMD_SAFETY_ALERT                     0x50
#define BQ40Z50_CMD_SAFETY_STATUS                    0x51
#define BQ40Z50_CMD_PF_ALERT                         0x52
#define BQ40Z50_CMD_PF_STATUS                        0x53
#define BQ40Z50_CMD_OPERATION_STATUS                 0x54
#define BQ40Z50_CMD_CHARGING_STATUS                  0x55
#define BQ40Z50_CMD_GAUGING_STATUS                   0x56
#define BQ40Z50_CMD_MANUFACTURING_STATUS             0x57
#define BQ40Z50_CMD_AFE_REGISTER                     0x58
#define BQ40Z50_CMD_TURBO_POWER                      0x59
#define BQ40Z50_CMD_TURBO_FINAL                      0x5A
#define BQ40Z50_CMD_TURBO_PACK_R                     0x5B
#define BQ40Z50_CMD_TURBO_SYS_R                      0x5C
#define BQ40Z50_CMD_TURBO_EDV                        0x5D
#define BQ40Z50_CMD_TURBO_CURRENT                    0x5E
#define BQ40Z50_CMD_NO_LOAD_REM_CAP                  0x5F
#define BQ40Z50_CMD_LIFETIME_DATA_BLOCK_1            0x60
#define BQ40Z50_CMD_LIFETIME_DATA_BLOCK_2            0x61
#define BQ40Z50_CMD_LIFETIME_DATA_BLOCK_3            0x62
#define BQ40Z50_CMD_LIFETIME_DATA_BLOCK_4            0x63
#define BQ40Z50_CMD_LIFETIME_DATA_BLOCK_5            0x64
#define BQ40Z50_CMD_MANUFACTURER_INFO                0x70
#define BQ40Z50_CMD_DA_STATUS_1                      0x71
#define BQ40Z50_CMD_DA_STATUS_2                      0x72
#define BQ40Z50_CMD_GAUGE_STATUS_1                   0x73
#define BQ40Z50_CMD_GAUGE_STATUS_2                   0x74
#define BQ40Z50_CMD_GAUGE_STATUS_3                   0x75
#define BQ40Z50_CMD_CB_STATUS                        0x76
#define BQ40Z50_CMD_STATE_OF_HEALTH_ALTERNATE        0x77
#define BQ40Z50_CMD_FILTERED_CAPACITY                0x78

/* ManufacturerAccess() sub-commands */
#define BQ40Z50_MANU_ACCESS_DEVICE_TYPE              0x0001
#define BQ40Z50_MANU_ACCESS_FIRMWARE_VERSION         0x0002
#define BQ40Z50_MANU_ACCESS_HARDWARE_VERSION         0x0003
#define BQ40Z50_MANU_ACCESS_INSTR_FLASH_SIGNATURE    0x0004
#define BQ40Z50_MANU_ACCESS_STATIC_DF_SIGNATURE      0x0005
#define BQ40Z50_MANU_ACCESS_CHEMICAL_ID              0x0006
#define BQ40Z50_MANU_ACCESS_STATIC_CHEM_DF_SIG       0x0008
#define BQ40Z50_MANU_ACCESS_ALL_DF_SIGNATURE         0x0009
#define BQ40Z50_MANU_ACCESS_SHUTDOWN_MODE            0x0010
#define BQ40Z50_MANU_ACCESS_SLEEP_MODE               0x0011
#define BQ40Z50_MANU_ACCESS_AUTO_CC_OFFSET           0x0013
#define BQ40Z50_MANU_ACCESS_FUSE_TOGGLE              0x001D
#define BQ40Z50_MANU_ACCESS_PCHG_FET_TOGGLE          0x001E
#define BQ40Z50_MANU_ACCESS_CHG_FET_TOGGLE           0x001F
#define BQ40Z50_MANU_ACCESS_DSG_FET_TOGGLE           0x0020
#define BQ40Z50_MANU_ACCESS_GAUGING                  0x0021
#define BQ40Z50_MANU_ACCESS_FET_CONTROL              0x0022
#define BQ40Z50_MANU_ACCESS_LIFETIME_DATA_COLLECTION 0x0023
#define BQ40Z50_MANU_ACCESS_PERMANENT_FAILURE        0x0024
#define BQ40Z50_MANU_ACCESS_BLACK_BOX_RECORDER       0x0025
#define BQ40Z50_MANU_ACCESS_FUSE                     0x0026
#define BQ40Z50_MANU_ACCESS_LED_DISPLAY_ENABLE       0x0027
#define BQ40Z50_MANU_ACCESS_LIFETIME_DATA_RESET      0x0028
#define BQ40Z50_MANU_ACCESS_PERM_FAIL_DATA_RESET     0x0029
#define BQ40Z50_MANU_ACCESS_BLACK_BOX_RECORDER_RESET 0x002A
#define BQ40Z50_MANU_ACCESS_LED_TOGGLE               0x002B
#define BQ40Z50_MANU_ACCESS_LED_DISPLAY_PRESS        0x002C
#define BQ40Z50_MANU_ACCESS_CALIBRATION_MODE         0x002D
#define BQ40Z50_MANU_ACCESS_LIFETIME_DATA_FLUSH      0x002E
#define BQ40Z50_MANU_ACCESS_LIFETIME_DATA_SPEEDUP    0x002F
#define BQ40Z50_MANU_ACCESS_SEAL_DEVICE              0x0030
#define BQ40Z50_MANU_ACCESS_SECURITY_KEYS            0x0035
#define BQ40Z50_MANU_ACCESS_AUTHENTICATION_KEY       0x0037
#define BQ40Z50_MANU_ACCESS_DEVICE_RESET             0x0041
#define BQ40Z50_MANU_ACCESS_SAFETY_ALERT             0x0050
#define BQ40Z50_MANU_ACCESS_SAFETY_STATUS            0x0051
#define BQ40Z50_MANU_ACCESS_PF_ALERT                 0x0052
#define BQ40Z50_MANU_ACCESS_PF_STATUS                0x0053
#define BQ40Z50_MANU_ACCESS_OPERATION_STATUS         0x0054
#define BQ40Z50_MANU_ACCESS_CHARGING_STATUS          0x0055
#define BQ40Z50_MANU_ACCESS_GAUGING_STATUS           0x0056
#define BQ40Z50_MANU_ACCESS_MANUFACTURING_STATUS     0x0057
#define BQ40Z50_MANU_ACCESS_AFE_REGISTER             0x0058
#define BQ40Z50_MANU_ACCESS_NO_LOAD_REM_CAP          0x005F
#define BQ40Z50_MANU_ACCESS_ROM_MODE                 0x0F00
#define BQ40Z50_MANU_ACCESS_EXIT_CAL_OUTPUT          0xF080
#define BQ40Z50_MANU_ACCESS_OUTPUT_CC_ADC_CAL        0xF081
#define BQ40Z50_MANU_ACCESS_OUTPUT_SHORTED_CC_ADC    0xF082

/* Data Flash Access (0x4000-0x5FFF) */
#define BQ40Z50_DF_ACCESS_START                      0x4000
#define BQ40Z50_DF_ACCESS_END                        0x5FFF

#define BQ40Z50_BATT_STATUS_FULLY_DISCHARGED         BIT(4)  /* Fully discharged */
#define BQ40Z50_BATT_STATUS_FULLY_CHARGED            BIT(5)  /* Fully charged */
#define BQ40Z50_BATT_STATUS_DSG                      BIT(6)   /* Battery is discharging */
#define BQ40Z50_BATT_STATUS_INIT                     BIT(7)   /* Gauge initialization is complete */
#define BQ40Z50_BATT_STATUS_RTA                      BIT(8)   /* Remaining time alarm */
#define BQ40Z50_BATT_STATUS_RCA                      BIT(9)  /* Remaining capacity alarm */
#define BQ40Z50_BATT_STATUS_TDA                      BIT(11)  /* Terminate Discharge Alarm */
#define BQ40Z50_BATT_STATUS_OTA                      BIT(12)  /* Overtemperature Alarm */
#define BQ40Z50_BATT_STATUS_TCA                      BIT(14)  /* Terminate Charge Alarm */
#define BQ40Z50_BATT_STATUS_OCA                      BIT(15)  /* Overcharged Alarm */

#define BQ40Z50_BATT_STATUS_EC_MASK                  0x0F     /* Mask for error code bits (Bits 3 - 0) */
#define BQ40Z50_BATT_STATUS_EC_OK                    0x0      /* OK */
#define BQ40Z50_BATT_STATUS_EC_BUSY                  0x1      /* Busy */
#define BQ40Z50_BATT_STATUS_EC_RESERVED_COMMAND      0x2      /* Reserved Command */
#define BQ40Z50_BATT_STATUS_EC_UNSUPPORTED_COMMAND   0x3      /* Unsupported Command */
#define BQ40Z50_BATT_STATUS_EC_ACCESS_DENIED         0x4      /* AccessDenied */
#define BQ40Z50_BATT_STATUS_EC_OVERFLOW_UNDERFLOW    0x5      /* Overflow/Underflow */   

#endif /* BQ40Z50 */