/*
 *  Definitions for ESCPID.ino
 *
 */

#ifndef __ESCPID_H
#define __ESCPID_H

// Defines
#define ESCPID_NB_ESC             1                 // Number of ESCs
#define ESCPID_MAX_ESC            6                 // Max number of ESCs

#define ESCPID_USB_UART_SPEED     115200            // Baudrate of the teeensy USB serial link

#define ESCPID_PID_P              400               // Default PID proportional gain
#define ESCPID_PID_I              1050              // Default PID integral gain
#define ESCPID_PID_D              1000              // Default PID derivative gain
#define ESCPID_PID_F              9900              // Default PID derivative filtering pole
#define ESCPID_PID_MIN            200                 // Default PID min control input value
#define ESCPID_PID_MAX            1999               // Default PID max control input value
#define ESCPID_PID_ADAPT_GAIN     0.0001            // Range adaptation gain for PID coefficient

#define ESCPID_COMM_MAGIC         0x43305735        // Magic number: "teensy35" in leet speech
#define ESCPID_RESET_GAIN         0xffff            // PIDf gain value that triggers teensy reset
#define ESCPID_RESET_DELAY        1500              // Delay between reception of reset cmd and effective reset (ms)
#define ESCPID_COMM_WD_LEVEL      20                // Maximum number of periods without reference refresh
                                                    //   20 x 2 ms = 40 ms; then ESCCMD's throttle watchdog
                                                    //   (another 40 ms) sends DShot MOTOR_STOP -> rotor COASTS.

// Reference shaping (AFC decision 2026-09-22, Q4): the reference is RATE-LIMITED
// toward the host target, not stepped. Spin-up 0 -> 30k rpm takes ~5 s. Stop is
// NOT ramped: link silence (disarm/terminate/link loss) -> MOTOR_STOP -> coast.
// Units: RPM_r / reference / measurement are all in 10 rpm (teensyshot units).
#define ESCPID_REF_RATE_RPM_S     6000              // max reference slew, rpm/s (30k / 5 s)
#define ESCPID_REF_MAX            6000              // reference clamp, 10 rpm units (= 60k rpm)
#define ESCPID_USB_DEBUG          1                 // 1 = 10 Hz USB line: tgt ref rpm cmd wd err deg

#define ESCPID_ERROR_MAGIC        -1                // Magic number error code

// Teensy->host communication data structure
// sizeof(ESCPID_comm)=64 to match USB 1.0 buffer size
typedef struct {
  uint32_t      magic;                        // Magic number
  int8_t        err[ESCPID_MAX_ESC];          // Last error number
  uint8_t       deg[ESCPID_MAX_ESC];          // ESC temperature (°C)
  uint16_t      cmd[ESCPID_MAX_ESC];          // Current ESC command value
  uint16_t      volt[ESCPID_MAX_ESC];         // Voltage of the ESC power supply (0.01V)
  uint16_t      amp[ESCPID_MAX_ESC];          // ESC current (0.01A)
  int16_t       rpm[ESCPID_MAX_ESC];          // Motor rpm (10 rpm)
} ESCPIDcomm_struct_t;

// Host->teensy communication data structure
// sizeof(Host_comm)=64 to match USB 1.0 buffer size
typedef struct {
  uint32_t      magic;                        // Magic number
  int16_t       RPM_r[ESCPID_MAX_ESC];        // Velocity reference (10 rpm)
  uint16_t      PID_P[ESCPID_MAX_ESC];        // PID proportional gain
  uint16_t      PID_I[ESCPID_MAX_ESC];        // PID integral gain
  uint16_t      PID_D[ESCPID_MAX_ESC];        // PID derivative gain
  uint16_t      PID_f[ESCPID_MAX_ESC];        // PID filtering pole
} Hostcomm_struct_t;

#endif
