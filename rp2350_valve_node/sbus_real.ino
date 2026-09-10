// =============================================================================
//  sbus_real.ino  --  Real SBUS receiver (PIO + DMA), core 0
//
//  Replaces the sbus_sim path when USE_REAL_SBUS == 1. Populates the SAME
//  globals the sim did (g_sbus_in, g_sbus_failsafe, g_sbus_framelost), so
//  arbitration/allocation downstream are unchanged.
//
//  Architecture:
//    * PIO SM (sbus_rx.pio) decodes inverted-uart bytes on PIN_SBUS (GP6).
//    * DMA continuously drains the PIO RX FIFO into a ring buffer, PACED BY THE
//      PIO RX DREQ -- this is mandatory: the 4/8-deep FIFO fills in <1 ms at
//      100 kbaud, far faster than a 100 Hz loop could drain it. DMA means the
//      PIO never stalls and core 0 scans the ring at its leisure.
//    * A byte-fed state machine finds 0x0F headers, collects 25-byte frames,
//      validates the footer, unpacks 16x11-bit channels, maps failsafe /
//      frame-lost, and normalizes the 4 flight axes into g_sbus_in.
//
//  ---------------------------------------------------------------------------
//  !!! NOT BENCH-VALIDATED. This was written from the SBUS spec and the RP2350
//  !!! datasheet; it has NOT been compiled or run. Before trusting it in
//  !!! flight, scope/logic-analyze against YOUR receiver and verify each item
//  !!! flagged "VALIDATE:" below.
//  ---------------------------------------------------------------------------
// =============================================================================
#include "config.h"
#include "types.h"

#if USE_REAL_SBUS
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "sbus_rx.pio.h"          // generated from sbus_rx.pio (see integration notes)

// ---- tunables ----
#define SBUS_BAUD            100000.0f
#define SBUS_RING_WORDS      256                  // uint32 ring; MUST be power of 2
#define SBUS_RING_BITS       10                   // log2(SBUS_RING_WORDS * 4 bytes) = log2(1024)
#define SBUS_FRAME_LEN       25
#define SBUS_HEADER          0x0F
#define SBUS_LOST_TIMEOUT_MS 100                  // no good frame this long -> framelost

// ---- CHANNEL ASSIGNMENT: EDIT to match YOUR transmitter (0-based SBUS ch) ----
// VALIDATE: print raw ch[] first and move each stick to confirm these indices.
#define SBUS_CH_ROLL         0
#define SBUS_CH_PITCH        1
#define SBUS_CH_THROTTLE     2
#define SBUS_CH_YAW          3

// SBUS raw endpoints (standard Futaba scaling).
#define SBUS_RAW_MIN  172
#define SBUS_RAW_MID  992
#define SBUS_RAW_MAX  1811

// ---- module state ----
static PIO      s_pio = pio0;                     // VALIDATE: pio0 free on this node?
static uint     s_sm;
static int      s_dma;
static uint32_t s_ring[SBUS_RING_WORDS] __attribute__((aligned(SBUS_RING_WORDS * 4)));
static uint32_t s_tail;                           // consumer index (word units)
static uint32_t s_last_good_ms;

static uint8_t  s_frame[SBUS_FRAME_LEN];          // frame assembly buffer
static int      s_idx;                            // 0 = hunting for header

static uint16_t s_ch[16];        // last decoded raw channels (172..1811)
static uint32_t s_frames_ok;     // footer-valid frames decoded
static uint32_t s_frames_bad;    // footer-rejected frames

static inline float norm_sym(uint16_t v) {        // -> [-1, 1]
    float f = ((float)v - SBUS_RAW_MID) / 819.0f;
    return f < -1.0f ? -1.0f : (f > 1.0f ? 1.0f : f);
}
static inline float norm_uni(uint16_t v) {        // -> [0, 1]
    float f = ((float)v - SBUS_RAW_MIN) / (float)(SBUS_RAW_MAX - SBUS_RAW_MIN);
    return f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);
}

// Unpack a validated 25-byte frame and publish to the shared globals.
static void sbus_decode(const uint8_t* f) {
    const uint8_t* d = f + 1;                      // 22 payload bytes (f[1..22])
    uint16_t ch[16];
    ch[0]  = (uint16_t)((d[0]      | d[1]  << 8)               & 0x07FF);
    ch[1]  = (uint16_t)((d[1]  >> 3 | d[2]  << 5)               & 0x07FF);
    ch[2]  = (uint16_t)((d[2]  >> 6 | d[3]  << 2 | d[4]  << 10) & 0x07FF);
    ch[3]  = (uint16_t)((d[4]  >> 1 | d[5]  << 7)               & 0x07FF);
    ch[4]  = (uint16_t)((d[5]  >> 4 | d[6]  << 4)               & 0x07FF);
    ch[5]  = (uint16_t)((d[6]  >> 7 | d[7]  << 1 | d[8]  << 9)  & 0x07FF);
    ch[6]  = (uint16_t)((d[8]  >> 2 | d[9]  << 6)               & 0x07FF);
    ch[7]  = (uint16_t)((d[9]  >> 5 | d[10] << 3)               & 0x07FF);
    ch[8]  = (uint16_t)((d[11]     | d[12] << 8)               & 0x07FF);
    ch[9]  = (uint16_t)((d[12] >> 3 | d[13] << 5)               & 0x07FF);
    ch[10] = (uint16_t)((d[13] >> 6 | d[14] << 2 | d[15] << 10) & 0x07FF);
    ch[11] = (uint16_t)((d[15] >> 1 | d[16] << 7)               & 0x07FF);
    ch[12] = (uint16_t)((d[16] >> 4 | d[17] << 4)               & 0x07FF);
    ch[13] = (uint16_t)((d[17] >> 7 | d[18] << 1 | d[19] << 9)  & 0x07FF);
    ch[14] = (uint16_t)((d[19] >> 2 | d[20] << 6)               & 0x07FF);
    ch[15] = (uint16_t)((d[20] >> 5 | d[21] << 3)               & 0x07FF);
    
    for (int i = 0; i < 16; ++i) s_ch[i] = ch[i];
    s_frames_ok++;

    uint8_t flags = f[23];
    bool failsafe  = (flags & 0x08) != 0;         // bit3
    bool framelost = (flags & 0x04) != 0;         // bit2

    // Publish the flags every valid frame (arbitration trusts these directly).
    g_sbus_failsafe  = failsafe;
    g_sbus_framelost = framelost;

    // Only refresh sticks + stamp on a clean frame; never feed stale/failsafe
    // values into allocation.
    if (!failsafe && !framelost) {
        g_sbus_in.roll     = norm_sym(ch[SBUS_CH_ROLL]);
        g_sbus_in.pitch    = norm_sym(ch[SBUS_CH_PITCH]);
        g_sbus_in.yaw      = norm_sym(ch[SBUS_CH_YAW]);
        g_sbus_in.throttle = norm_uni(ch[SBUS_CH_THROTTLE]);
        g_sbus_in.valid    = true;
        g_sbus_in.stamp_ms = millis();
        s_last_good_ms     = millis();
    }
}

// Feed one received byte through the frame state machine.
static void sbus_feed(uint8_t b) {
    if (s_idx == 0) {                             // hunting for header
        if (b == SBUS_HEADER) s_frame[s_idx++] = b;
        return;
    }
    s_frame[s_idx++] = b;
    if (s_idx >= SBUS_FRAME_LEN) {
        s_idx = 0;
        // Footer: standard SBUS = 0x00; SBUS2 receivers send 0x04/0x14/0x24/0x34
        // (low nibble 0x0 or 0x4). VALIDATE against your receiver's actual footer.
        uint8_t foot = s_frame[24];
        uint8_t lo   = foot & 0x0F;
        if (lo == 0x00 || lo == 0x04) sbus_decode(s_frame);
        // else: bad frame -> drop, re-hunt for header (self-resyncs)
    }
}

void sbus_real_print() {
    Serial.printf("[sbus] ok=%lu bad=%lu | fs=%d lost=%d valid=%d age=%lums\n",
        (unsigned long)s_frames_ok, (unsigned long)s_frames_bad,
        g_sbus_failsafe, g_sbus_framelost, g_sbus_in.valid,
        (unsigned long)(millis() - g_sbus_in.stamp_ms));
    Serial.printf("[sbus] raw:");
    for (int i = 0; i < 16; ++i) Serial.printf(" %4u", s_ch[i]);
    Serial.println();
    Serial.printf("[sbus] map: R=%.3f P=%.3f Y=%.3f T=%.3f (ch R%d P%d Y%d T%d)\n",
        g_sbus_in.roll, g_sbus_in.pitch, g_sbus_in.yaw, g_sbus_in.throttle,
        SBUS_CH_ROLL, SBUS_CH_PITCH, SBUS_CH_YAW, SBUS_CH_THROTTLE);
}

void sbus_real_setup() {
    // ---- PIO ----
    uint off = pio_add_program(s_pio, &sbus_rx_program);
    s_sm = pio_claim_unused_sm(s_pio, true);
    sbus_rx_program_init(s_pio, s_sm, off, PIN_SBUS, SBUS_BAUD);

    // ---- DMA: PIO RX FIFO -> ring buffer, paced by RX DREQ ----
    s_dma = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(s_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);   // full FIFO word
    channel_config_set_read_increment(&c, false);             // fixed FIFO addr
    channel_config_set_write_increment(&c, true);             // advance in ring
    channel_config_set_ring(&c, true, SBUS_RING_BITS);        // wrap the write addr
    channel_config_set_dreq(&c, pio_get_dreq(s_pio, s_sm, false));
    dma_channel_configure(s_dma, &c,
                          s_ring,                 // write: ring buffer
                          &s_pio->rxf[s_sm],      // read: RX FIFO
                          0xFFFFFFFF,             // count: ~143 h @100kbaud, re-armed per boot
                          true);                  // start now

    s_tail         = 0;
    s_idx          = 0;
    s_last_good_ms = millis();
    g_sbus_failsafe  = true;      // start untrusted until a good frame arrives
    g_sbus_framelost = true;
    g_sbus_in.valid  = false;
}

// Call from the CORE 0 loop, where sbus_sim_update() used to be called.
void sbus_real_update(uint32_t now) {
    // Where has DMA written to? (ring-wrapped, so this stays in-buffer.)
    uint32_t waddr = dma_hw->ch[s_dma].write_addr;
    uint32_t head  = ((waddr - (uint32_t)s_ring) >> 2) & (SBUS_RING_WORDS - 1);

    while (s_tail != head) {
        uint32_t w = s_ring[s_tail];
        sbus_feed((uint8_t)(w >> 24));            // our byte lives in bits [31:24]
        s_tail = (s_tail + 1) & (SBUS_RING_WORDS - 1);
    }

    // Receiver-loss watchdog: no clean frame recently -> declare frame-lost so
    // arbitration reverts away from SBUS (fail toward the safe source).
    if ((now - s_last_good_ms) > SBUS_LOST_TIMEOUT_MS) {
        g_sbus_framelost = true;
        g_sbus_in.valid  = false;
    }
}
#endif // USE_REAL_SBUS
