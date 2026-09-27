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
// S17 telemetry-loss fallback (see ESCPID.h)
bool      ESCPID_OpenLoop[ESCPID_NB_ESC] = {};    // open-loop fallback active
float     ESCPID_OLTarget[ESCPID_NB_ESC] = {};    // throttle the open-loop ramp heads for
uint16_t  ESCPID_AcqTicks[ESCPID_NB_ESC] = {};    // live ticks spent waiting for a first fresh sample
uint16_t  ESCPID_StaleTicks[ESCPID_NB_ESC] = {};  // closed-loop ticks since the last NEW packet
uint32_t  ESCPID_LastRx[ESCPID_NB_ESC] = {};      // telemetry packet count at the previous tick
uint16_t  ESCPID_GoodRun[ESCPID_NB_ESC] = {};     // consecutive good packets (open-loop hand-back)
uint16_t  ESCPID_GapTicks[ESCPID_NB_ESC] = {};    // ticks since the last good packet
uint32_t  ESCPID_PktCount[ESCPID_NB_ESC] = {};    // good packets, for the pkt/s debug field
float     ESCPID_CtrlFilt[ESCPID_NB_ESC] = {};    // ~100 ms average of the closed-loop throttle
float     ESCPID_OLHold[ESCPID_NB_ESC] = {};      // open-loop floor captured at entry
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

        // S17 / emulation: STATE codes, repeated on every reply while true (the
        // ESCCMD error above is only "last event since the previous reply").
        // A DShot output error (-1) or ESC over-temperature (-8) still wins.
        int8_t e = ESCPID_comm.err[i];
        bool urgent = ( e == ESCCMD_ERROR_DSHOT || e == ESCCMD_ERROR_TLM_TEMP );
        if ( ESCPID_OpenLoop[i] && !urgent )
          ESCPID_comm.err[i] = ESCPID_ERROR_NO_TLM;
        else if ( ESCCMD_is_emulated( ) && !urgent )   // emulated packet loss is not news
          ESCPID_comm.err[i] = ESCPID_ERROR_EMULATION;
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
  if ( ESCCMD_is_emulated( ) )
    Serial.println( "*** ESC EMULATION BUILD: telemetry is FAKE -- never connect a real ESC ***" );
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
  ESCPID_OpenLoop[i]  = false;
  ESCPID_AcqTicks[i]  = 0;
  ESCPID_StaleTicks[i]= 0;
  ESCPID_LastRx[i]    = ESCCMD_read_tlm_rx_cnt( i );
  ESCPID_GoodRun[i]   = 0;
  ESCPID_GapTicks[i]  = 0;
  ESCPID_CtrlFilt[i]  = ESCPID_Min[i];
  ESCPID_OLHold[i]    = ESCPID_Min[i];
}

// A telemetry sample is usable only if it is valid AND physically plausible
// (CRC8 lets ~1 in 256 garbage packets through; rpm is read as int16).
static bool ESCPID_sample_ok( int i, int16_t *rpm ) {
  if ( ESCCMD_read_tlm_status( i ) != 0 ) return false;
  if ( ESCCMD_read_rpm( i, rpm ) != 0 )   return false;
  return ( *rpm >= 0 && *rpm <= ESCPID_RPM_PLAUS_MAX );
}

// Advance the rate-limited reference. Returns true once the loop is seeded.
// Seeding (from a measured rpm) is done by the caller via ESCPID_seed().
static bool ESCPID_shape_reference( int i ) {
  const float step = ( ESCPID_REF_RATE_RPM_S / 10.0f ) * ( ESCCMD_TIMER_PERIOD * 1e-6f );

  if ( !ESCPID_Running[i] ) {             // first live tick after a stop / boot
    ESCPID_Running[i] = true;
    ESCPID_Fresh[i]   = false;
    ESCPID_Control[i] = ESCPID_Min[i];
  }
  if ( !ESCPID_Fresh[i] ) return false;   // still acquiring: throttle stays put

  float d = ESCPID_Target[i] - ESCPID_Reference[i];
  if ( d >  step ) d =  step;
  if ( d < -step ) d = -step;
  ESCPID_Reference[i] += d;
  return true;
}

// Seed the reference from a measured rpm (10 rpm units). NOT clamped to the
// target: a rotor already above target (coasting, or open loop on a light load)
// is walked down by the rate limit instead of stepped -- no throttle dip.
static void ESCPID_seed( int i, int16_t rpm ) {
  float seed = ( rpm > 0 ) ? (float)rpm : 0.0f;
  if ( seed > ESCPID_REF_MAX ) seed = ESCPID_REF_MAX;
  ESCPID_Reference[i] = seed;
  ESCPID_comm.rpm[i]  = rpm;               // PID measurement = this fresh sample
  ESCPID_Fresh[i]     = true;
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
  static uint32_t last_pkts = 0;
  uint32_t pps = ( ESCPID_PktCount[0] - last_pkts ) * 10;   // 10 Hz line -> packets/s
  last_pkts = ESCPID_PktCount[0];
  Serial.printf( "tgt=%ld ref=%ld rpm=%ld cmd=%u wd=%s err=%d deg=%u pkt/s=%lu%s%s\n",
                 (long)( ESCPID_Target[0] * 10 ), (long)( ESCPID_Reference[0] * 10 ),
                 (long)rpm * 10, cmd,
                 ( ESCPID_comm_wd < ESCPID_COMM_WD_LEVEL ) ? "live" : "STALE",
                 err, deg, (unsigned long)pps,
                 ESCPID_OpenLoop[0] ? " OPENLOOP" : "",
                 ESCCMD_is_emulated( ) ? " EMU" : "" );
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
        const uint16_t acq_ticks   = (uint16_t)( ESCPID_ACQ_TIMEOUT_MS  * 1000UL / ESCCMD_TIMER_PERIOD );
        const uint16_t stale_ticks = (uint16_t)( ESCPID_TLM_STALE_MS    * 1000UL / ESCCMD_TIMER_PERIOD );
        const uint16_t gap_ticks   = (uint16_t)( ESCPID_HANDBACK_GAP_MS * 1000UL / ESCCMD_TIMER_PERIOD );

        // Sequence the start and advance the rate-limited reference
        bool seeded = ESCPID_shape_reference( i );

        // ---- S17: telemetry freshness ----
        // A GOOD packet = arrived since the last tick (the packet counter moved;
        // ESCCMD's 'valid' flag alone stays set after packets stop), CRC-valid
        // and plausible. Only good packets refresh anything.
        uint32_t rx  = ESCCMD_read_tlm_rx_cnt( i );
        int16_t  rpm = 0;
        bool good = ( rx != ESCPID_LastRx[i] ) && ESCPID_sample_ok( i, &rpm );
        bool bad  = ( rx != ESCPID_LastRx[i] ) && !good;   // arrived but unusable
        ESCPID_LastRx[i] = rx;
        if ( good ) { ESCPID_GapTicks[i] = 0; ESCPID_PktCount[i]++; }
        else if ( ESCPID_GapTicks[i] < 0xFFFF ) ESCPID_GapTicks[i]++;
        if ( bad || ESCPID_GapTicks[i] > gap_ticks ) ESCPID_GoodRun[i] = 0;
        else if ( good && ESCPID_GoodRun[i] < 0xFFFF ) ESCPID_GoodRun[i]++;

        // Open-loop point for THIS target: the characterized throttle scaled by
        // target / OL_RPM (<= 1), floored at PID_MIN. Unset -> 0 here.
        float ol_pt = 0.0f;
        if ( ESCPID_OL_THROTTLE > ESCPID_PID_MIN ) {
          float frac = ( ESCPID_Target[i] * 10.0f ) / (float)ESCPID_OL_RPM;
          if ( frac > 1.0f ) frac = 1.0f;
          if ( frac < 0.0f ) frac = 0.0f;
          ol_pt = ESCPID_Min[i] + frac * ( (float)ESCPID_OL_THROTTLE - ESCPID_Min[i] );
        }

        if ( ESCPID_OpenLoop[i] ) {
          // Head for the (scaled) OL point, but never below what the closed loop
          // was averaging when it lost telemetry (fail toward airflow: a heavy
          // load or a >30k target needed more than the calibration point).
          ESCPID_OLTarget[i] = ( ol_pt > ESCPID_OLHold[i] ) ? ol_pt : ESCPID_OLHold[i];
          // Hand back only after a run of consecutive good, plausible packets.
          if ( good && ESCPID_GoodRun[i] >= ESCPID_HANDBACK_N ) {
            ESCPID_seed( i, rpm );                       // from the latest good sample
            AWPID_preset( i, ESCPID_Control[i] );        // bumpless
            ESCPID_CtrlFilt[i] = ESCPID_Control[i];
            ESCPID_OpenLoop[i]   = false;
            ESCPID_AcqTicks[i]   = 0;
            ESCPID_StaleTicks[i] = 0;
            seeded = true;
          }
        }
        else if ( !seeded ) {
          // Acquiring after a (re)start: the first good packet seeds the loop.
          if ( good ) {
            ESCPID_seed( i, rpm );
            ESCPID_AcqTicks[i] = 0;
            seeded = true;
          } else {
            if ( ESCPID_AcqTicks[i] < 0xFFFF ) ESCPID_AcqTicks[i]++;
            if ( ESCPID_AcqTicks[i] >= acq_ticks ) {
              ESCPID_OpenLoop[i] = true;
              ESCPID_OLHold[i]   = ESCPID_Control[i];            // no closed-loop history: PID_MIN
              ESCPID_OLTarget[i] = ( ol_pt > ESCPID_OLHold[i] ) ? ol_pt : ESCPID_OLHold[i];
              ESCPID_GoodRun[i]  = 0;
            }
          }
        }
        else {
          // Closed loop: trip to open loop after STALE_MS without a good packet.
          if ( good )                              ESCPID_StaleTicks[i] = 0;
          else if ( ESCPID_StaleTicks[i] < 0xFFFF ) ESCPID_StaleTicks[i]++;
          if ( ESCPID_StaleTicks[i] >= stale_ticks ) {
            ESCPID_OpenLoop[i] = true;
            // Floor = the ~100 ms AVERAGE throttle, not the last value: a single
            // in-range garbage packet just before the wire died can't set it.
            ESCPID_OLHold[i]   = ESCPID_CtrlFilt[i];
            ESCPID_OLTarget[i] = ( ol_pt > ESCPID_OLHold[i] ) ? ol_pt : ESCPID_OLHold[i];
            ESCPID_Fresh[i]    = false;
            ESCPID_GoodRun[i]  = 0;
            seeded = false;
          }
        }

        if ( ESCPID_OpenLoop[i] ) {
          // Rate-limited ramp toward the open-loop point (PID_MIN -> OL in ~OL_RAMP_S).
          float span = (float)ESCPID_OL_THROTTLE - ESCPID_Min[i];
          float step = ( span > 1.0f ? span : 1.0f ) * ( ESCCMD_TIMER_PERIOD * 1e-6f ) / ESCPID_OL_RAMP_S;
          float d = ESCPID_OLTarget[i] - ESCPID_Control[i];
          if ( d >  step ) d =  step;
          if ( d < -step ) d = -step;
          ESCPID_Control[i] += d;
          if ( ESCPID_Control[i] < ESCPID_Min[i] ) ESCPID_Control[i] = ESCPID_Min[i];
          if ( ESCPID_Control[i] > ESCPID_Max[i] ) ESCPID_Control[i] = ESCPID_Max[i];
        }
        // Closed loop: run the PID ONLY on a tick with a good new sample, so a
        // frozen reading is never integrated. Between samples the last control
        // signal is sent.
        else if ( seeded && good ) {
          ESCPID_comm.rpm[i]    = rpm;
          ESCPID_Measurement[i] = rpm;
          AWPID_control(  i,
                          ESCPID_Reference[i],
                          ESCPID_Measurement[i],
                          &ESCPID_Control[i] );
        }
        // ~100 ms average of the closed-loop throttle, sampled only on ticks with
        // a good packet: a held spike between packets (or after the wire dies)
        // counts once, not for the whole stale window.
        if ( !ESCPID_OpenLoop[i] && seeded && good ) {
          const float a = ( ESCCMD_TIMER_PERIOD * 1e-6f ) / 0.1f;
          ESCPID_CtrlFilt[i] += a * ( ESCPID_Control[i] - ESCPID_CtrlFilt[i] );
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
