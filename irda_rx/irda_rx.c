/**
 * Copyright (c) 2022 Kaz Kojima <kkojima@rr.iij4u.or.jp>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "irda_rx.pio.h"

// This program
// - Uses a PIO state machine to receive low speed IrDA samples
// - Prints out the received text to the default console

#define SERIAL_BAUD PICO_DEFAULT_UART_BAUD_RATE

#define PIO_RX_PIN 15

// one 16-bit mark + 9 16-bit samples packed in 5 32-bit words
#define WORD_COUNT 5

#define IRDA_BUFFER_COUNT 2

static int dma_channel;
static uint32_t raw_buffer[IRDA_BUFFER_COUNT][WORD_COUNT];
static volatile int raw_buffer_write_index;
static volatile int raw_buffer_read_index;
static uint dma_irq;

static void samples_ready_handler();

static void irda_dma_handler() {
    // clear IRQ
    dma_hw->ints0 = (1u << dma_channel);

    // get the current buffer index
    raw_buffer_read_index = raw_buffer_write_index;

    // get the next capture index to send the dma to start
    raw_buffer_write_index = (raw_buffer_write_index + 1) % IRDA_BUFFER_COUNT;

    // give the channel a new buffer to write to and re-trigger it
    dma_channel_transfer_to_buffer_now(
        dma_channel,
        raw_buffer[raw_buffer_write_index],
        WORD_COUNT
    );

    samples_ready_handler();
}

#define RX_FIFO_SIZE 128
static uint8_t char_fifo[RX_FIFO_SIZE];
static volatile int char_fifo_write_index;
static volatile int char_fifo_read_index;
static volatile size_t char_fifo_len;

static void char_fifo_write(uint8_t c) {
    if (char_fifo_len == RX_FIFO_SIZE)
        return;
    char_fifo[char_fifo_write_index] = c;
    char_fifo_write_index = (char_fifo_write_index + 1) % RX_FIFO_SIZE;
    ++char_fifo_len;
}

static int char_fifo_read(void) {
    if (char_fifo_len == 0)
        return -1;
    uint8_t c = char_fifo[char_fifo_read_index];
    char_fifo_read_index = (char_fifo_read_index + 1) % RX_FIFO_SIZE;
    --char_fifo_len;
    return c;
}

static bool char_fifo_empty(void) {
  return (char_fifo_len == 0);
}

static volatile uint16_t error_sample;
static volatile int sample_error_count;

static void samples_ready_handler(void) {
    static bool decode = false;
    static int bidx;
    static uint8_t ch;
    static uint uu;
    uint16_t u;

    for (int i = 0; i < 2*WORD_COUNT; i++) {
      if (!(i & 1)) {
	    uu = raw_buffer[raw_buffer_read_index][i>>1];
	    u = uu >> 16;
	} else {
	    u = (uint16_t)uu;
	}

	if (u == 0x0) {
	    bidx = 0;
	    ch = 0;
	    decode = true;
	} else if (decode) {
	    if (bidx > 7) {
		if (u == 0xffff)
		    char_fifo_write(ch);
	        decode = false;
		continue;
	    } 
	    if (u == 0xffff) {
		ch |= 1 << bidx;
		bidx++;
	    } else if (u == 0xff1f || u == 0xff9f || u == 0xff8f) {
	        bidx++;
	    } else {
	        // error
	        sample_error_count++;
		error_sample = u;
		decode = false;
		continue;
	    }
	}
    }
}

#if defined(TEST)
void print_capture_u16(const uint16_t bits) {
    // Display the capture buffer in text form, like this:
    // __--__--__--__--__--__--
    for (int index = 0; index < 16; ++index) {
	uint word_mask = 1u << index;
	printf((bits & word_mask) ? "-" : "_");
    }
    printf("\n");
}
#endif

int main() {
    // Console output
    stdio_init_all();
    printf("Starting PIO IrDA RX example\n");

    // Set up the state machine we're going to use to receive them.
    PIO pio = pio0;
    uint sm = 0;

    dma_channel = dma_claim_unused_channel(true);
    if (dma_channel < 0) {
        return -1;
    }

    uint offset = pio_add_program(pio, &irda_rx_program);
    irda_rx_program_init(pio, sm, offset, PIO_RX_PIN, SERIAL_BAUD);

    dma_channel_config dma_channel_cfg = dma_channel_get_default_config(dma_channel);

    channel_config_set_transfer_data_size(&dma_channel_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_channel_cfg, false);
    channel_config_set_write_increment(&dma_channel_cfg, true);
    channel_config_set_dreq(&dma_channel_cfg, pio_get_dreq(pio, sm, false));
    dma_irq = DMA_IRQ_0;
    dma_channel_configure(
        dma_channel,
        &dma_channel_cfg,
        raw_buffer[0],
        &pio->rxf[sm],
        WORD_COUNT,
        false
    );
    irq_set_enabled(dma_irq, true);
    irq_set_exclusive_handler(dma_irq, irda_dma_handler);
    dma_channel_set_irq0_enabled(dma_channel, true);

    pio_sm_set_enabled(pio, sm, true);

    raw_buffer_write_index = 0;
    raw_buffer_read_index = 0;

    dma_channel_transfer_to_buffer_now(
        dma_channel,
        raw_buffer[0],
        WORD_COUNT
    );

    pio_sm_set_enabled(pio, sm, true);

    while (true) {
        __asm__ __volatile__("wfi");
	if (sample_error_count) {
	    sample_error_count = 0;
	    printf("\n[%04x]\n", error_sample);
	}
        if (!char_fifo_empty()) {
            printf("%c", char_fifo_read());
        }
    }
}
