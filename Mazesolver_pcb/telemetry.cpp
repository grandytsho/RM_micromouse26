#include "telemetry.h"

/* =====================================================================
 *  Global PID gains (single source of truth, live-tunable over BT)
 *  Sensible starting values; overwrite on the fly with "P..","I..","D.."
 * ===================================================================== */
float Kp = 1.0f;
float Ki = 0.0f;
float Kd = 0.0f;

/* ---- Receive accumulator --------------------------------------------
 * We assemble one command line here as bytes trickle in. The parser is
 * NON-BLOCKING: each call to serviceBluetoothParser() consumes only the
 * bytes already sitting in the UART FIFO and then returns immediately.
 * A command is considered complete when we see '\n' or '\r'.
 * --------------------------------------------------------------------- */
static char    rxBuffer[32];
static uint8_t rxIndex = 0;

void telemetryInit() {
    pinMode(BT_STATE_PIN, INPUT);   // HC-05 STATE pin drives this HIGH/LOW
    BT_SERIAL.begin(BT_BAUD);
    rxIndex = 0;
}

bool btConnected() {
    return digitalRead(BT_STATE_PIN) == HIGH;
}

void btPrint(const String &s) {
    if (btConnected()) BT_SERIAL.print(s);
}

void btPrintln(const String &s) {
    if (btConnected()) BT_SERIAL.println(s);
}

/* ---- Apply a single parsed command ----------------------------------
 * cmd  : 'P' / 'I' / 'D'  (case-insensitive)
 * value: the float that followed it, e.g. "P1.5" -> 'P', 1.5
 * --------------------------------------------------------------------- */
static void applyCommand(char cmd, float value) {
    switch (cmd) {
        case 'P': case 'p': Kp = value; break;
        case 'I': case 'i': Ki = value; break;
        case 'D': case 'd': Kd = value; break;
        default: return;            // unknown letter -> ignore silently
    }
    // Echo an acknowledgement so the operator sees the gain "took".
    btPrint("ACK ");
    btPrint(String(cmd));
    btPrint("=");
    btPrintln(String(value, 4));
}

/* ---- The non-blocking parser ----------------------------------------
 * Protocol (one command per line, newline-terminated):
 *     P1.5\n     -> Kp = 1.5
 *     I0.10\n    -> Ki = 0.10
 *     D0.5\n     -> Kd = 0.5
 *
 * IMPORTANT: send a trailing newline from your BT terminal app. We treat
 * the first character as the command letter and atof() the remainder.
 * --------------------------------------------------------------------- */
void serviceBluetoothParser() {
    // Process only what is already buffered; never wait/spin for more.
    while (BT_SERIAL.available() > 0) {
        char c = (char)BT_SERIAL.read();

        if (c == '\n' || c == '\r') {           // end of a command line
            if (rxIndex > 0) {
                rxBuffer[rxIndex] = '\0';        // null-terminate
                char  cmd = rxBuffer[0];
                float val = atof(&rxBuffer[1]);  // parse "1.5", "0.10", ...
                applyCommand(cmd, val);
                rxIndex = 0;                     // reset for next line
            }
        } else {
            if (rxIndex < sizeof(rxBuffer) - 1) {
                rxBuffer[rxIndex++] = c;         // accumulate
            } else {
                rxIndex = 0;                     // overflow guard -> drop line
            }
        }
    }
}
