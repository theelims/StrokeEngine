#ifndef MOTOR_LINMOT_H
#define MOTOR_LINMOT_H

#include <time.h>

#include "CANopen.h"
#include "motor.hpp"

#define LINMOT_CONTROL_SWITCH_ON ((uint16_t)(1 << 0))
#define LINMOT_CONTROL_ERROR_ACKNOWLEDGE ((uint16_t)(1 << 7))
#define LINMOT_CONTROL_HOME ((uint16_t)(1 << 11))

#define LINMOT_STATUS_OPERATION_ENABLED (1 << 0)
#define LINMOT_STATUS_SWITCH_ON (1 << 1)
#define LINMOT_STATUS_ERROR (1 << 3)
#define LINMOT_STATUS_VOLTAGE_ENABLE (1 << 4)
#define LINMOT_STATUS_WARNING (1 << 7)
#define LINMOT_STATUS_AT_TARGET (1 << 10)
#define LINMOT_STATUS_HOMED (1 << 11)
#define LINMOT_STATUS_FATAL_ERROR (1 << 12)
#define LINMOT_STATUS_MOTION_ACTIVE (1 << 13)

#define LINMOT_WARN_MOTOR_HOT_SENSOR (1 << 0)
#define LINMOT_WARN_MOTOR_SHORT_TIME_OVERLOAD (1 << 1)
#define LINMOT_WARN_MOTOR_SUPPLY_VOLTAGE_LOW (1 << 2)
#define LINMOT_WARN_MOTOR_SUPPLY_VOLTAGE_HIGH (1 << 3)
#define LINMOT_WARN_MOTOR_DRIVE_HOT (1 << 6)
#define LINMOT_WARN_MOTOR_NOT_HOMED (1 << 7)

#define LINMOT_STATE_ERROR (0x04)
#define LINMOT_STATE_OPERATIONAL (0x08)
#define LINMOT_STATE_HOMING (0x09)

#define LINMOT_ERROR_MOTION_CMD_WRONG_STATE (0x86)
#define LINMOT_ERROR_CANBUS_GUARD_TIMEOUT (0xCD)

// Accepts four 16bit parameters 
// TODO - Define parameters better
#define LINMOT_CMD_VAI_16BIT_GO_TO_POS (0x0900)

class LinmotMotor: public MotorInterface {
  public:
    // Motion
    //void goToHome();
    void goToPos(float position, float speed, float acceleration);
    void stopMotion();

    // General
    void registerTasks();

    // CANOpen
    void CO_setNodeId(uint8_t nodeId);
    void CO_setControl(OD_entry_t *entry);
    void CO_setCmdHeader(OD_entry_t *entry);
    void CO_setCmdParameters(OD_entry_t *entry);
    void CO_setStatus(OD_entry_t *entry);
    void CO_setMonitor(OD_entry_t *entry);

    void CO_control_addFlag(uint16_t flag);
    void CO_control_removeFlag(uint16_t flag);

    void CO_sendCmd(uint16_t cmd, uint16_t parameter_a, uint16_t parameter_b, uint16_t parameter_c, uint16_t parameter_d);

    void CO_run_rpdo_received();
    void CO_motion_rpdo_received();
    void CO_monitor_rpdo_received();

    // Tasks
    void task_motion();
    void task_heartbeat();

  private:
    uint8_t CO_nodeId;
    
    OD_entry_t *CO_statusWord_entry;
    OD_extension_t CO_statusWord_extension;
    
    OD_entry_t *CO_monitorWord_entry;
    OD_extension_t CO_monitorWord_extension;

    OD_entry_t *CO_controlWord_entry;
    OD_extension_t CO_controlWord_extension;
    uint8_t *CO_controlWord_flags;

    OD_entry_t *CO_cmdHeader_entry;

    OD_entry_t *CO_cmdParameters_entry;
    OD_extension_t CO_cmdParameters_extension;
    uint8_t *CO_cmdParameters_flags;

    uint16_t CO_controlWord = 0x003E;

    // TPDO 1
    uint16_t CO_statusWord;
    uint16_t CO_runWord;
    uint16_t CO_errorWord;
    uint16_t CO_warnWord;

    // TPDO 2
    uint16_t CO_cmdWord;
    int16_t CO_demandPositionWord; // UPID 0xE9A4 - SInt16 - 0.0001 mm Scale
    int16_t CO_actualPositionWord; // UPID 0xE9A5 - SInt16 - 0.0001 mm Scale
    int16_t CO_demandCurrentWord; // UPID 0xE9E7 - SInt16 - 0.001 A Scale

    // TPDO 3
    int16_t CO_modelTempWord; // UPID 0xEB10 - SInt16 - 0.1 C Scale
    uint16_t CO_realTempWord; // UPID 0x6990 - UInt16 - 0.980392156862745 C Scale, -50 C Offset
    int16_t CO_motorVoltageWord; // UPID 0xE96D - SInt16 - 0.01 V Scale
    int16_t CO_powerLossWord; // UPID 0xEADB - SInt16 - 0.1 W Scale

    uint8_t CO_cmdCount = 0;
    uint16_t position = 0;
    bool hasInitialized = false;
    time_t lastRPDOUpdate;
};

#endif