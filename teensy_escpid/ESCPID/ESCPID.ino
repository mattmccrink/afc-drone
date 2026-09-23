/*
 *  ESCPID:   PID control of up to 6 ESCs using teensy 3.5 MCU
 *
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Arda Yiğit and Jacques Gangloff
 *  Date:     May 2019
 */

// Includes
#include <Arduino.h>
#include "DSHOT.h"
#include "ESCCMD.h"
#include "AWPID.h"
#include "ESCPID.h"

// Forward declarations (defined below loop helpers)
static void ESCPID_restart_bookkeeping( int i );

// Globals
float     ESCPID_Target[ESCPID_NB_ESC] = {};      // host target, clamped [0, ESCPID_REF_MAX]
float     ESCPID_Reference[ESCPID_NB_ESC] = {};   // rate-limited reference fed to the PID
bool      ESCPID_Running[ESCPID_NB_ESC] = {};     // start sequence begun since the last MOTOR_STOP
bool      ESCPID_Fresh[ESCPID_NB_ESC] = {};       // a post-start telemetry sample has seeded the loop
float     ESCPID_Measurement[ESCPID_NB_ESC] = {};
float     ESCPID_Control[ESCPID_NB_ESC] = {};
uint16_t  ESCPID_comm_wd = 0;

float     ESCPID_Kp[ESCPID_NB_ESC];
float     ESCPID_Ki[ESCPID_NB_ESC];
float     ESCPID_Kd[ESCPID_NB_ESC];
float     ESCPID_f[ESCPID_NB_ESC];
float     ESCPID_Min[ESCPID_NB_ESC];
float     ESCPID_Max[ESCPID_NB_ESC];

//Add memory for serial transmission
static uint8_t serial3_rx_buf[512];

ESCPIDcomm_struct_t ESCPID_comm = {
                                  ESCPID_COMM_MAGIC,
                                  {},
                                  {},
                                  {},
                                  {},
                                  {},
                                  {}
                                  };
Hostcomm_struct_t   Host_comm =   {
                                  ESCPID_COMM_MAGIC,
                                  {},
                                  {},
                                  {},
                                  {},
                                  {}
                                  };

//
// Manage communication with the host
//
int ESCPID_comm_update( void ) {
  static int          i;
  static uint8_t      *ptin   = (uint8_t*)(&Host_comm),
                      *ptout  = (uint8_t*)(&ESCPID_comm);
  static int          ret;
  static int          in_cnt = 0;
  
  ret = 0;

  // Read all incoming bytes available until incoming structure is complete
  while(  ( Serial3.available( ) > 0 ) && 
          ( in_cnt < (int)sizeof( Host_comm ) ) )
    ptin[in_cnt++] = Serial3.read( );
  
  // Check if a complete incoming packet is available
  if ( in_cnt == (int)sizeof( Host_comm ) ) {
  
    // Clear incoming bytes counter
    in_cnt = 0;
    
    // Look for a reset command
    // If first ESC has 0xffff for PID and f gains, reset teensy
    if (  ( Host_comm.PID_P[0] == ESCPID_RESET_GAIN ) &&
          ( Host_comm.PID_I[0] == ESCPID_RESET_GAIN ) &&
          ( Host_comm.PID_D[0] == ESCPID_RESET_GAIN ) &&
          ( Host_comm.PID_f[0] == ESCPID_RESET_GAIN ) ) {
      
      // Give time to host to close serial port
      delay( ESCPID_RESET_DELAY );

      // Reset command
      SCB_AIRCR = 0x05FA0004;
    }
    
    // Check the validity of the magic number
    if ( Host_comm.magic != ESCPID_COMM_MAGIC ) {
    
      // Flush input buffer
      while ( Serial3.available( ) )
        Serial3.read( );
    
      ret = ESCPID_ERROR_MAGIC;
    }
    else {
    
      // Reset the communication watchdog
      ESCPID_comm_wd = 0;
      
      // Update the TARGET (the reference is rate-limited toward it in loop()).
      // Unidirectional compressor: clamp to [0, ESCPID_REF_MAX]. A negative or
      // corrupt value can no longer reach the PID's reverse branch (removed).
      for ( i = 0; i < ESCPID_NB_ESC; i++ ) {
        float t = Host_comm.RPM_r[i];
        if ( t < 0.0f )           t = 0.0f;
        if ( t > ESCPID_REF_MAX ) t = ESCPID_REF_MAX;
        ESCPID_Target[i] = t;
      }
      
      // Update PID tuning parameters
      for ( i = 0; i < ESCPID_NB_ESC; i++ ) {
        
        // Gain conversion from int to float
        ESCPID_Kp[i] =  ESCPID_PID_ADAPT_GAIN * Host_comm.PID_P[i];
        ESCPID_Ki[i] =  ESCPID_PID_ADAPT_GAIN * Host_comm.PID_I[i];
        ESCPID_Kd[i] =  ESCPID_PID_ADAPT_GAIN * Host_comm.PID_D[i];
        ESCPID_f[i] =   ESCPID_PID_ADAPT_GAIN * Host_comm.PID_f[i];
        
        // Update PID tuning
        AWPID_tune(     i,
                        ESCPID_Kp[i],
                        ESCPID_Ki[i],
                        ESCPID_Kd[i],
                        ESCPID_f[i],
                        ESCPID_Min[i],
                        ESCPID_Max[i]
                       );
      }
      
      // Update output data structure values
      // If telemetry is invalid, data structure remains unmodified
      for ( i = 0; i < ESCPID_NB_ESC; i++ ) {
        ESCCMD_read_err( i, &ESCPID_comm.err[i] );
        ESCCMD_read_cmd( i, &ESCPID_comm.cmd[i] );
        ESCCMD_read_deg( i, &ESCPID_comm.deg[i] );
        ESCCMD_read_volt( i, &ESCPID_comm.volt[i] );
        ESCCMD_read_amp( i, &ESCPID_comm.amp[i] );
        ESCCMD_read_rpm( i, &ESCPID_comm.rpm[i] );
      }
      
      // Send data structure to host
      Serial3.write( ptout, sizeof( ESCPID_comm ) );
      
    }
  }

  return ret;
}

//
//  Arduino setup function
//
void setup() {
  int i;

  // Initialize USB serial link
  Serial.begin( ESCPID_USB_UART_SPEED );
  Serial3.addMemoryForRead(serial3_rx_buf, sizeof(serial3_rx_buf));
  Serial3.begin(921600);
  

  // Initialize PID gains
  for ( i = 0; i < ESCPID_NB_ESC; i++ ) {
    ESCPID_Kp[i] =  ESCPID_PID_ADAPT_GAIN * ESCPID_PID_P;
    ESCPID_Ki[i] =  ESCPID_PID_ADAPT_GAIN * ESCPID_PID_I;
    ESCPID_Kd[i] =  ESCPID_PID_ADAPT_GAIN * ESCPID_PID_D;
    ESCPID_f[i] =   ESCPID_PID_ADAPT_GAIN * ESCPID_PID_F;
    ESCPID_Min[i] = ESCPID_PID_MIN;
    ESCPID_Max[i] = ESCPID_PID_MAX;
  }

  // Initialize PID subsystem
  AWPID_init( ESCPID_NB_ESC, 
              ESCPID_Kp, 
              ESCPID_Ki, 
              ESCPID_Kd, 
              ESCPID_f, 
              ESCPID_Min, 
              ESCPID_Max );

  // Initialize the CMD subsystem
  ESCCMD_init( ESCPID_NB_ESC );

  // Arming ESCs
  ESCCMD_arm_all( );
  
  // 3D mode is NOT used: the compressor is unidirectional. (ESCCMD_3D_on() also
  // wrote ESC EEPROM on every boot.) Normal-mode throttle range 0..1999.
  //ESCCMD_3D_on( );

  // Arming ESCs
  ESCCMD_arm_all( );
  
  // Start periodic loop
  ESCCMD_start_timer( );
  
  // Stop all motors
  for ( i = 0; i < ESCPID_NB_ESC; i++ ) {
    ESCCMD_stop( i );
  }

  // Reference watchdog is initially triggered
  ESCPID_comm_wd = ESCPID_COMM_WD_LEVEL;
  for ( i = 0; i < ESCPID_NB_ESC; i++ )
    ESCPID_restart_bookkeeping( i );        // clean stopped state (throttle floor, no seed)
}

//
//  Start / hold / stop sequencing and reference shaping (AFC, decision 2026-09-22).
//
//  * Brief link lapse (comm watchdog tripped but ESCCMD has NOT yet sent
//    MOTOR_STOP): HOLD everything -- reference, PID state, last throttle (ESCCMD
//    keeps re-sending it). A single late/dropped frame must not cost airflow.
//  * Stop (ESCCMD's throttle watchdog has sent MOTOR_STOP): rotor coasts. Reset
//    the PID and mark the loop not running; the next start is a fresh start.
//  * Start: hold throttle at PID_MIN only until a FRESH telemetry sample arrives
//    (ESCCMD invalidates telemetry on MOTOR_STOP), then seed the reference from
//    the MEASURED rpm (<= target) so a still-coasting rotor is picked up where it
//    is instead of being braked down to a ramp from 0. The PID's first call
//    initialises its derivative history from that same fresh sample (no kick).
//  * Running: reference rate-limited toward the target at ESCPID_REF_RATE_RPM_S
//    in both directions (0 -> 30k in ~5 s).
//
static void ESCPID_restart_bookkeeping( int i ) {
  AWPID_reset( );
  ESCPID_Running[i]   = false;
  ESCPID_Fresh[i]     = false;
  ESCPID_Reference[i] = 0.0f;
  ESCPID_Control[i]   = ESCPID_Min[i];   // never re-send a stale high throttle
}

// Returns true once the loop has a fresh measurement and may run the PID.
static bool ESCPID_shape_reference( int i ) {
  const float step = ( ESCPID_REF_RATE_RPM_S / 10.0f ) * ( ESCCMD_TIMER_PERIOD * 1e-6f );

  if ( !ESCPID_Running[i] ) {             // first live tick after a stop / boot
    ESCPID_Running[i] = true;
    ESCPID_Fresh[i]   = false;
    ESCPID_Control[i] = ESCPID_Min[i];
  }

  if ( !ESCPID_Fresh[i] ) {
    int16_t rpm;
    if ( ESCCMD_read_tlm_status( i ) == 0 && ESCCMD_read_rpm( i, &rpm ) == 0 ) {
      float seed = ( rpm > 0 ) ? (float)rpm : 0.0f;          // 10 rpm units
      if ( seed > ESCPID_Target[i] ) seed = ESCPID_Target[i];
      ESCPID_Reference[i] = seed;
      ESCPID_comm.rpm[i]  = rpm;           // PID measurement = this fresh sample
      ESCPID_Fresh[i]     = true;
    } else {
      return false;                        // still acquiring: throttle stays at PID_MIN
    }
  }

  float d = ESCPID_Target[i] - ESCPID_Reference[i];
  if ( d >  step ) d =  step;
  if ( d < -step ) d = -step;
  ESCPID_Reference[i] += d;
  return true;
}

#if ESCPID_USB_DEBUG
// 10 Hz human-readable line on USB (bench verification of ramp / stop).
static void ESCPID_debug_print( void ) {
  static uint32_t last = 0;
  uint32_t now = millis( );
  if ( ( now - last ) < 100 ) return;
  last = now;
  // Never block the control loop on USB: skip the line if no terminal is open or
  // the USB TX buffer can't take it right now.
  // (64 = one USB FS packet; Teensy 3.x never reports more than that.)
  if ( !Serial.dtr( ) || Serial.availableForWrite( ) < 64 ) return;
  // NB: never call ESCCMD_read_err() here -- it CLEARS the error, which would
  // hide it from the host reply. Print the last value already reported instead.
  uint16_t cmd = 0; int16_t rpm = 0; uint8_t deg = 0;
  int8_t   err = ESCPID_comm.err[0];
  ESCCMD_read_cmd( 0, &cmd );
  ESCCMD_read_rpm( 0, &rpm );
  ESCCMD_read_deg( 0, &deg );
  Serial.printf( "tgt=%ld ref=%ld rpm=%ld cmd=%u wd=%s err=%d deg=%u\n",
                 (long)( ESCPID_Target[0] * 10 ), (long)( ESCPID_Reference[0] * 10 ),
                 (long)rpm * 10, cmd,
                 ( ESCPID_comm_wd < ESCPID_COMM_WD_LEVEL ) ? "live" : "STALE",
                 err, deg );
}
#endif

//
//  Arduino main loop
//
void loop( ) {
  static int    i, ret;

  // Check for next timer event
  ret = ESCCMD_tic( );

  // Bidirectional serial exchange with host
  ESCPID_comm_update(  );

  if ( ret == ESCCMD_TIC_OCCURED )  {

    // Process timer event
    bool live = ( ESCPID_comm_wd < ESCPID_COMM_WD_LEVEL );

    for ( i = 0; i < ESCPID_NB_ESC; i++ ) {

      if ( live ) {
        // Sequence the start and advance the rate-limited reference
        bool seeded = ESCPID_shape_reference( i );

        // Compute control signal only with a fresh-since-start, valid sample.
        // In case of invalid telemetry, last control signal is sent.
        if ( seeded && !ESCCMD_read_tlm_status( i ) ) {
          ESCPID_Measurement[i] = ESCPID_comm.rpm[i];
          AWPID_control(  i,
                          ESCPID_Reference[i],
                          ESCPID_Measurement[i],
                          &ESCPID_Control[i] );
        }

        // Send control signal (forward only; ESCPID_PID_MIN..MAX)
        ret = ESCCMD_throttle( i, (int16_t)ESCPID_Control[i] );
      }
      else {
        // Link silent (host disarmed/terminated, or link lost). Stop feeding
        // throttle; ESCCMD's throttle watchdog sends MOTOR_STOP ~40 ms later and
        // the rotor COASTS. Until that has actually happened, HOLD all state so a
        // brief lapse resumes seamlessly.
        uint16_t c = DSHOT_CMD_MOTOR_STOP;
        ESCCMD_read_cmd( i, &c );
        if ( c == DSHOT_CMD_MOTOR_STOP && ESCPID_Running[i] )
          ESCPID_restart_bookkeeping( i );
      }
    }
    
    // Update watchdog
    if ( ESCPID_comm_wd < ESCPID_COMM_WD_LEVEL )  {
      ESCPID_comm_wd++;
    }
  }

#if ESCPID_USB_DEBUG
  ESCPID_debug_print( );
#endif
}
