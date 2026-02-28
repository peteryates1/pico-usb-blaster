#include "blaster.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#include "hardware/pio.h"
#include "blaster_jtag.pio.h"

// Pin assignments — match pico-fpga / pico-dirtyJtag wiring
#define TCK_DCLK_PIN        2
#define TMS_nCONFIG_PIN     3
#define nCE_PIN             6   // unused for JTAG, keep out of the way
#define nCS_PIN             7   // unused for JTAG, keep out of the way
#define TDI_ASDI_PIN        4
#define TDO_CONF_DONE_PIN   5
#define DATAOUT_nSTATUS_PIN 8   // unused for JTAG, keep out of the way

#define ACTIVE_LED_PIN PICO_DEFAULT_LED_PIN

// Pin masks for direct SIO register access
#define TCK_MASK  (1u << TCK_DCLK_PIN)
#define TDI_MASK  (1u << TDI_ASDI_PIN)
#define TDO_MASK  (1u << TDO_CONF_DONE_PIN)

// Output pin mask (TCK, TMS, nCE, nCS, TDI)
#define OUT_PIN_MASK ((1u << TCK_DCLK_PIN) | (1u << TMS_nCONFIG_PIN) | \
                      (1u << nCE_PIN) | (1u << nCS_PIN) | (1u << TDI_ASDI_PIN))

#ifdef ACTIVE_LED_WS2812_PIN
    #include "ws2812.h"
    #if !defined(ACTIVE_LED_WS2812_COLOR_OFF) || !defined(ACTIVE_LED_WS2812_COLOR_ON)
        #error ws2812 colors not defined
    #endif
#endif

#define SHIFT_MODE_FLAG(b)  (!!((b) & 0b10000000))
#define READ_FLAG(b)        (!!((b) & 0b01000000))
#define OE_FLAG(b)          (!!((b) & 0b00100000))
#define PAYLOAD(b)             ((b) & 0b00111111)

static bool initialized = false;
static bool output_enabled = false;
static int shift_bytes_left = 0;
static bool shift_read_set;

// PIO JTAG shift
static PIO jtag_pio;
static uint jtag_sm;
static uint jtag_offset;
static bool jtag_pio_ok = false;

// rx/tx byte to signal relations in bitbang mode
//
// rx 
// bitmask | name            | JTAG | AS        | PS
// --------+-----------------+------+-----------+-----------
// 0x01    | TCK_DCLK        | TCK  | DCLK      | DCLK
// 0x02    | TMS_nCONFIG     | TMS  | nCONFIG   | nCONFIG
// 0x04    | nCE             | -    | nCE       | -
// 0x08    | nCS             | -    | nCS       | -
// 0x10    | TDI_ASDI        | TDI  | ASDI      | DATA0 
// 0x20    |            Output Enable/LED active
//
// tx
// 0x01    | TDO_CONF_DONE   | TDO  | CONF_DONE | CONF_DONE
// 0x02    | DATAOUT_nSTATUS | -    | DATAOUT   | nSTATUS

static void blaster_init(void)
{
#ifdef ACTIVE_LED_PIN
    gpio_init(ACTIVE_LED_PIN);
    gpio_set_dir(ACTIVE_LED_PIN, true);
#endif

    // Init all 7 pins as GPIO inputs
    gpio_init(TCK_DCLK_PIN);
    gpio_init(TMS_nCONFIG_PIN);
    gpio_init(nCE_PIN);
    gpio_init(nCS_PIN);
    gpio_init(TDI_ASDI_PIN);
    gpio_init(TDO_CONF_DONE_PIN);
    gpio_init(DATAOUT_nSTATUS_PIN);

    // Init PIO JTAG shift engine
    jtag_pio = pio0;
    if (pio_can_add_program(jtag_pio, &blaster_jtag_program)) {
        jtag_offset = pio_add_program(jtag_pio, &blaster_jtag_program);
        int sm = pio_claim_unused_sm(jtag_pio, false);
        if (sm >= 0) {
            jtag_sm = (uint)sm;
            blaster_jtag_program_init(jtag_pio, jtag_sm, jtag_offset,
                                      TDI_ASDI_PIN, TDO_CONF_DONE_PIN, TCK_DCLK_PIN);
            // SM is running but stalls immediately on empty TX FIFO (TCK low)
            // Pins still under SIO control until we switch them
            gpio_set_function(TCK_DCLK_PIN, GPIO_FUNC_SIO);
            gpio_set_function(TDI_ASDI_PIN, GPIO_FUNC_SIO);
            jtag_pio_ok = true;
        }
    }

    initialized = true;
}

// :)
static inline void delay_5_cycles(void) 
{
    __asm__ volatile 
    (
        "nop\n\t" // 1
        "nop\n\t" // 2
        "nop\n\t" // 3
        "nop\n\t" // 4
        "nop"     // 5
    );
}

static inline void output_enable(bool enable)
{
    if (output_enabled == enable)
        return;

    output_enabled = enable;

#ifdef ACTIVE_LED_PIN
    gpio_put(ACTIVE_LED_PIN, enable);
#endif

    // Set 5 output pins (TCK, TMS, nCE, nCS, TDI) as output or high-Z
    uint32_t dir = enable ? 0xFFFFFFFF : 0;
    gpio_set_dir_masked(OUT_PIN_MASK, dir);
}

// Map protocol bitfield to GPIO pin positions atomically
static inline uint32_t protocol_to_gpio(uint8_t data)
{
    uint32_t gpio_val = 0;
    if (data & 0x01) gpio_val |= (1u << TCK_DCLK_PIN);        // bit 0 -> TCK
    if (data & 0x02) gpio_val |= (1u << TMS_nCONFIG_PIN);     // bit 1 -> TMS
    if (data & 0x04) gpio_val |= (1u << nCE_PIN);             // bit 2 -> nCE
    if (data & 0x08) gpio_val |= (1u << nCS_PIN);             // bit 3 -> nCS
    if (data & 0x10) gpio_val |= (1u << TDI_ASDI_PIN);        // bit 4 -> TDI
    return gpio_val;
}

static inline uint8_t bitbang(uint8_t data)
{
    uint8_t ret = (!!gpio_get(TDO_CONF_DONE_PIN)) | ((!!gpio_get(DATAOUT_nSTATUS_PIN)) << 1);
    delay_5_cycles();
    // Set all output pins atomically to avoid glitches
    gpio_put_masked(OUT_PIN_MASK, protocol_to_gpio(data));
    return ret;
}

static inline uint8_t shift_bitbang(uint8_t data)
{
    uint8_t ret = 0;

    for (int i = 0; i < 8; ++i)
    {
        // Set TDI via direct SIO — branchless (clr then conditionally set)
        sio_hw->gpio_clr = TDI_MASK;
        sio_hw->gpio_set = ((uint32_t)(data & 1)) << TDI_ASDI_PIN;

        ret >>= 1;

        // Read TDO via direct SIO
        if (sio_hw->gpio_in & TDO_MASK)
            ret |= 0x80;

        // TCK high — minimal delay (just the register write latency)
        sio_hw->gpio_set = TCK_MASK;
        // TCK low
        sio_hw->gpio_clr = TCK_MASK;

        data >>= 1;
    }

    return ret;
}

static inline void shift_enter_pio(void)
{
    pio_gpio_init(jtag_pio, TCK_DCLK_PIN);
    pio_gpio_init(jtag_pio, TDI_ASDI_PIN);
}

static inline void shift_exit_pio(void)
{
    // Drain RX FIFO in case of leftover data
    while (!pio_sm_is_rx_fifo_empty(jtag_pio, jtag_sm))
        (void)jtag_pio->rxf[jtag_sm];

    gpio_set_function(TCK_DCLK_PIN, GPIO_FUNC_SIO);
    gpio_set_function(TDI_ASDI_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(TCK_DCLK_PIN, true);
    gpio_set_dir(TDI_ASDI_PIN, true);
}

static inline uint8_t shift_pio(uint8_t data)
{
    *(io_rw_8 *)&jtag_pio->txf[jtag_sm] = data;
    while (pio_sm_is_rx_fifo_empty(jtag_pio, jtag_sm))
        tight_loop_contents();
    // Right-shift IN places 8 bits at [31:24]; read full word and extract top byte
    return (uint8_t)(jtag_pio->rxf[jtag_sm] >> 24);
}

void blaster_reset(void)
{
    if (!initialized)
        blaster_init();

    shift_bytes_left = 0;
    output_enable(false);
    // Default nCS=1 so shift mode reads TDO (JTAG) not DATAOUT (AS)
    gpio_put_masked(OUT_PIN_MASK, 1u << nCS_PIN);
}

int blaster_process(uint8_t rxBuf[], int rxCount, uint8_t txBuf[])
{
    int txCount = 0;

    for (int i = 0; i < rxCount; ++i)
    {
        uint8_t b = rxBuf[i];

        if (shift_bytes_left > 0) // shift mode active
        {
            uint8_t input = jtag_pio_ok ? shift_pio(b) : shift_bitbang(b);

            if (shift_read_set)
            {
                txBuf[txCount] = input;
                ++txCount;
            }

            --shift_bytes_left;

            if (shift_bytes_left == 0 && jtag_pio_ok)
                shift_exit_pio();
        }
        else if (SHIFT_MODE_FLAG(b)) // shift mode activated
        {
            shift_read_set = READ_FLAG(b);
            shift_bytes_left = PAYLOAD(b);
            gpio_put(TCK_DCLK_PIN, false);

            if (jtag_pio_ok)
                shift_enter_pio();
        }
        else // bitbang mode
        {
            output_enable(OE_FLAG(b));
            uint8_t input = bitbang(b);

            if (READ_FLAG(b))
            {
                txBuf[txCount] = input;
                ++txCount;
            }
        }
    }

    return txCount;
}
