/**
 * @brief Adaption of the pca301serial library to the RFM69 transmitter
 *
 * This library was created by extensively reading source code and forum
 * comments contributed by other authors. Thank you.
 *
 * It is currently only tested on the Arduino Nano with the RFM69 transmitter
 * connected by SPI. Thanks to this guide (german)
 * https://steigerbalett.wordpress.com/2015/05/23/jeelink-clone-loten-und-mit-einer-firmware-flashen-fur-lacrosse-sensoren-in-fhem/
 * I was able to wire them together.
 *
 * Copyright (c) 2017, Sven Bachmann <dev@mcbachmann.de>
 *
 * Licensed under the MIT license, see LICENSE for details.

 * For files directly taken and modified from the pca301serial project please
 * read their headers for information and license details.
 */
#include <SPI.h>
#include "funky_rfm69.h"


/*****************************************************************************/
/* Local defines */
/*****************************************************************************/
#define RFM69_IS_HW                 true
#define PCA301_SERIAL_SPEED_BPS     57600
#define PCA301_FREQ_CARRIER_KHZ     868950
#define PCA301_BITRATE_BS           6631
#define PCA301_PIN_SPI_SS           10
#define PCA301_PIN_INT              2


/*****************************************************************************/
/* Local variables */
/*****************************************************************************/
static uint8_t pca301_sync_values[] = { 0x2d, 0xd4 }; /**< sync word values */


/*****************************************************************************/
/* Local prototypes */
/*****************************************************************************/
static void pca301_board_init(
    void
);

static void pca301_rfm69_init(
    void
);


/*****************************************************************************/
/* External prototypes */
/*****************************************************************************/
void pca301serial_loop(
    void
);

void pca301serial_setup(
    void
);


/*****************************************************************************/
/** Arduino Setup Function
 */
void setup(
    void
)
{
    pca301_board_init();

    pca301serial_setup();
}


/*****************************************************************************/
/** Arduino Loop Function
 */
void loop(
    void
)
{
    rfm69_timer_loop();
    pca301serial_loop();
}


/*****************************************************************************/
/** Board Initialization
 */
static void pca301_board_init(
    void
)
{
    /* initialize Serial communication */
    Serial.begin(PCA301_SERIAL_SPEED_BPS);
    while (!Serial);

    /* initialize RFM69 for PCA301 */
    pca301_rfm69_init();

    /* configure IRQ for RFM69 */
    pinMode(PCA301_PIN_INT, INPUT);

    /* enable interrupts */
    rfm69_int_enable();
}


/*****************************************************************************/
/** RFM69 Initialization
 */
static void pca301_rfm69_init(
    void
)
{
    /* configure RFM69 SPI */
    rfm69_init(PCA301_PIN_SPI_SS, RFM69_IS_HW);

    /* put transceiver in standby mode */
    rfm69_opmode_set(RFM69_OPMODE_STANDBY);

    /* frequency: 868.950 MHz */
    rfm69_freq_carrier_khz(PCA301_FREQ_CARRIER_KHZ);

    /* bitrate: 6.631 kb/s */
    rfm69_bitrate_bs(PCA301_BITRATE_BS);

    /* configure RX and TX interrupt generators */
    rfm69_dio_mapping_rx(0, RFM69_DIO0_RX_PAYLOADREADY_TX_TXREADY);
    rfm69_dio_mapping_tx(0, RFM69_DIO0_RX_CRCOK_TX_PACKETSENT);

    /* disable CLKOUT to save power */
    rfm69_clkout(RFM69_CLKOUT_OFF);

    /* configure CRC */
    rfm69_crc_on(false);
    rfm69_crc_auto_clear_off(true);

    /* set payload length */
    rfm69_payload_length(12);

    /* configure sync word */
    rfm69_sync_word(2, pca301_sync_values);
    rfm69_sync_on(true);

    /* RX bandwidth exponent */
    rfm69_rx_bw_exp(2);

    /* RSSI threshold */
    rfm69_rssi_threshold(0xdc);

    /* variable length packet format */
    rfm69_packet_format_var_len(false);

    /* TX start condition */
    rfm69_tx_start_cond(RFM69_FIFO_NOT_EMPTY);

    /* set frequency deviation in Hz */
    rfm69_fdev_hz(45000);

    /* enable receiver mode */
    rfm69_opmode_set(RFM69_OPMODE_RX);

    /* clear fifo */
    rfm69_fifo_clear();
}


/*****************************************************************************/
/** RFM69 Interrupt Enable
 */
void rfm69_int_enable(
    void
)
{
    attachInterrupt(digitalPinToInterrupt(PCA301_PIN_INT), rfm69_isr, RISING);
}


/*****************************************************************************/
/** RFM69 Interrupt Disable
 */
void rfm69_int_disable(
    void
)
{
    detachInterrupt(digitalPinToInterrupt(PCA301_PIN_INT));
}
