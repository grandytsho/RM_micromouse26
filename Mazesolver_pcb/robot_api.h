#ifndef ROBOT_API_H
#define ROBOT_API_H

/* =====================================================================
 *  MODULE 5 : THE INTERFACE  (robot_api.h)
 * =====================================================================
 *  Single header the algorithm / maze-solver team includes to get the
 *  full Hardware Abstraction Layer. Include this and nothing else:
 *
 *      #include "robot_api.h"
 *
 *  ------------------------------------------------------------------
 *  WHAT YOU GET
 *  ------------------------------------------------------------------
 *  Motion / odometry (motors.h):
 *      setMotorSpeeds(int left, int right)   // -255..+255, sign=direction
 *      disableMotors()
 *      getLeftTicks() / getRightTicks()      // 4x-decoded quadrature ticks
 *      resetEncoders()
 *
 *  Sensing (sensors.h):
 *      getQuaternion()                       // fused BNO085 orientation
 *      readIRDifferential(IRSensor s)        // ambient-cancelled IR
 *      readAllIR(int *out)                   // fills out[IR_COUNT]
 *      IR channels: IR_FRONT1, IR_FRONT2, IR_FRONT_LEFT,
 *                   IR_FRONT_RIGHT, IR_LEFT, IR_RIGHT
 *
 *  Live PID gains (telemetry.h):
 *      Kp, Ki, Kd   // global floats, tweakable over BT as "P..","I..","D.."
 *
 *  Power / status (system_state.h):
 *      readBatteryVoltage()                  // true pack voltage (V)
 *      (the kill-switch + status LED run automatically from loop())
 *
 *  ------------------------------------------------------------------
 *  CONTRACT
 *  ------------------------------------------------------------------
 *  - All HAL services are non-blocking and millis()-driven; call your
 *    control logic from loop() and it will co-exist cleanly.
 *  - If pack voltage falls below the cutoff, the HAL latches motors OFF
 *    and never returns. Treat motor commands as best-effort, not assured.
 * ===================================================================== */

#include "system_state.h"
#include "telemetry.h"
#include "sensors.h"
#include "motors.h"

#endif // ROBOT_API_H
