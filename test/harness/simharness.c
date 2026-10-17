/*
  Thin C API over libsimavr for the Python test harness (sim.py)

  One sim_t is one powered ATmega328P running firmware.elf at 16 MHz. The harness drives input
  pins and the ADC, records every level change of the watched output pins with its time, collects
  UART output and feeds UART input at 9600 baud.

  The IR LED (D3, OC2B) carries IRremote's 38 kHz carrier while Timer2 drives OC2B, that is while
  COM2B1 in TCCR2A is set. simavr does not toggle OC2B in the phase-correct PWM mode IRremote uses,
  so the harness records that bit after every instruction as the pseudo pin CARRIER_PIN.
*/

#include <stdlib.h>
#include <string.h>

#include "sim_avr.h"
#include "sim_elf.h"
#include "sim_io.h"
#include "sim_time.h"
#include "sim_cycle_timers.h"
#include "avr_ioport.h"
#include "avr_adc.h"
#include "avr_uart.h"
#include "avr_eeprom.h"

#define PORTS 3 /* B, C, D */
#define EEPROM_SIZE 1024
#define UART_BYTE_USEC 1042 /* 10 bits at 9600 baud */
#define TCCR2A 0xB0
#define COM2B1 5
#define CARRIER_PIN (PORTS * 8)

typedef struct {
  uint64_t t_us;
  uint8_t pin; /* port index * 8 + bit */
  uint8_t level;
} sim_edge_t;

typedef struct sim sim_t;

typedef struct {
  sim_t *sim;
  uint8_t pin;
} watch_t;

struct sim {
  avr_t *avr;
  watch_t watch[PORTS * 8];
  uint8_t level[PORTS * 8 + 1];
  uint8_t ext_mask[PORTS], ext_value[PORTS];
  sim_edge_t *edges;
  size_t n_edges, cap_edges;
  char *out;
  size_t n_out, cap_out;
  char *in;
  size_t in_head, n_in;
  int feeding;
};

static int port_index(char port) { return port == 'B' ? 0 : port == 'C' ? 1 : 2; }
static char port_name(int index) { return "BCD"[index]; }

static uint64_t now_us(sim_t *s) { return avr_cycles_to_usec(s->avr, s->avr->cycle); }

static void record(sim_t *s, uint8_t pin, uint8_t level) {
  if (s->level[pin] == level) return;
  s->level[pin] = level;
  if (s->n_edges == s->cap_edges) {
    s->cap_edges = s->cap_edges ? s->cap_edges * 2 : 4096;
    s->edges = realloc(s->edges, s->cap_edges * sizeof(sim_edge_t));
  }
  s->edges[s->n_edges++] = (sim_edge_t){now_us(s), pin, level};
}

static void on_pin(struct avr_irq_t *irq, uint32_t value, void *param) {
  watch_t *w = param;
  record(w->sim, w->pin, value & 1);
}

static void on_uart(struct avr_irq_t *irq, uint32_t value, void *param) {
  sim_t *s = param;
  if (s->n_out == s->cap_out) {
    s->cap_out = s->cap_out ? s->cap_out * 2 : 4096;
    s->out = realloc(s->out, s->cap_out);
  }
  s->out[s->n_out++] = (char)value;
}

static avr_cycle_count_t feed_uart(avr_t *avr, avr_cycle_count_t when, void *param) {
  sim_t *s = param;
  if (s->in_head == s->n_in) {
    s->feeding = 0;
    return 0;
  }
  avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT), (uint8_t)s->in[s->in_head++]);
  return when + avr_usec_to_cycles(avr, UART_BYTE_USEC);
}

/* eeprom: EEPROM_SIZE bytes loaded before the first instruction, or NULL for an erased EEPROM (0xFF) */
sim_t *sim_new(const char *elf_path, const uint8_t *eeprom) {
  elf_firmware_t fw;
  memset(&fw, 0, sizeof(fw));
  if (elf_read_firmware(elf_path, &fw)) return NULL;
  avr_t *avr = avr_make_mcu_by_name("atmega328p");
  if (!avr) return NULL;
  avr_init(avr);
  avr->log = LOG_ERROR;
  avr_load_firmware(avr, &fw);
  avr->frequency = 16000000;
  avr->vcc = avr->avcc = avr->aref = 5000;

  sim_t *s = calloc(1, sizeof(sim_t));
  s->avr = avr;

  uint8_t image[EEPROM_SIZE];
  memset(image, 0xFF, sizeof(image));
  if (eeprom) memcpy(image, eeprom, sizeof(image));
  avr_eeprom_desc_t desc = {.ee = image, .offset = 0, .size = EEPROM_SIZE};
  avr_ioctl(avr, AVR_IOCTL_EEPROM_SET, &desc);

  uint32_t flags = 0;
  avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &flags);
  flags &= ~AVR_UART_FLAG_STDIO;
  avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &flags);
  avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), on_uart, s);

  for (int p = 0; p < PORTS; p++)
    for (int b = 0; b < 8; b++) {
      watch_t *w = &s->watch[p * 8 + b];
      w->sim = s;
      w->pin = p * 8 + b;
      avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(port_name(p)), b), on_pin, w);
    }
  return s;
}

void sim_free(sim_t *s) {
  avr_terminate(s->avr);
  free(s->edges);
  free(s->out);
  free(s->in);
  free(s);
}

/* runs for usec of simulated time; returns the simavr cpu state (cpu_Done = 6 or cpu_Crashed = 7 when the firmware stops) */
int sim_run_us(sim_t *s, uint64_t usec) {
  avr_cycle_count_t target = s->avr->cycle + (avr_cycle_count_t)usec * (s->avr->frequency / 1000000);
  int state = s->avr->state;
  while (s->avr->cycle < target) {
    state = avr_run(s->avr);
    record(s, CARRIER_PIN, s->avr->data[TCCR2A] >> COM2B1 & 1);
    if (state == cpu_Done || state == cpu_Crashed) break;
  }
  return state;
}

uint64_t sim_now_us(sim_t *s) { return now_us(s); }

/* drives an input pin from outside; the level also holds against the firmware's internal pull-up */
void sim_set_input(sim_t *s, char port, int bit, int level) {
  int p = port_index(port);
  s->ext_mask[p] |= 1 << bit;
  if (level) s->ext_value[p] |= 1 << bit;
  else s->ext_value[p] &= ~(1 << bit);
  avr_ioport_external_t ext = {.name = port, .mask = s->ext_mask[p], .value = s->ext_value[p]};
  avr_ioctl(s->avr, AVR_IOCTL_IOPORT_SET_EXTERNAL(port), &ext);
  avr_raise_irq(avr_io_getirq(s->avr, AVR_IOCTL_IOPORT_GETIRQ(port), bit), level ? 1 : 0);
}

void sim_set_adc_mv(sim_t *s, int channel, int millivolts) {
  avr_raise_irq(avr_io_getirq(s->avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC0 + channel), millivolts);
}

int sim_level(sim_t *s, char port, int bit) { return s->level[port_index(port) * 8 + bit]; }

int sim_carrier(sim_t *s) { return s->level[CARRIER_PIN]; }

/* copies up to max recorded edges into out and removes them; returns the count copied */
size_t sim_take_edges(sim_t *s, sim_edge_t *out, size_t max) {
  size_t n = s->n_edges < max ? s->n_edges : max;
  memcpy(out, s->edges, n * sizeof(sim_edge_t));
  memmove(s->edges, s->edges + n, (s->n_edges - n) * sizeof(sim_edge_t));
  s->n_edges -= n;
  return n;
}

size_t sim_take_uart(sim_t *s, char *out, size_t max) {
  size_t n = s->n_out < max ? s->n_out : max;
  memcpy(out, s->out, n);
  memmove(s->out, s->out + n, s->n_out - n);
  s->n_out -= n;
  return n;
}

/* queues bytes for the firmware's UART receiver, one byte per UART_BYTE_USEC */
void sim_send_uart(sim_t *s, const char *data, size_t len) {
  size_t pending = s->n_in - s->in_head;
  char *in = malloc(pending + len);
  memcpy(in, s->in + s->in_head, pending);
  memcpy(in + pending, data, len);
  free(s->in);
  s->in = in;
  s->in_head = 0;
  s->n_in = pending + len;
  if (!s->feeding) {
    s->feeding = 1;
    avr_cycle_timer_register_usec(s->avr, UART_BYTE_USEC, feed_uart, s);
  }
}

void sim_read_eeprom(sim_t *s, uint8_t *out) {
  avr_eeprom_desc_t desc = {.ee = NULL, .offset = 0, .size = EEPROM_SIZE};
  avr_ioctl(s->avr, AVR_IOCTL_EEPROM_GET, &desc);
  memcpy(out, desc.ee, EEPROM_SIZE);
}

/* 32-bit little-endian firmware variables at a data address, such as timer0_millis; writes wait
   until interrupts are enabled, so a change never lands inside millis() copying it with interrupts off */
uint32_t sim_read_u32(sim_t *s, uint16_t addr) {
  uint8_t *d = s->avr->data + addr;
  return d[0] | d[1] << 8 | (uint32_t)d[2] << 16 | (uint32_t)d[3] << 24;
}

void sim_write_u32(sim_t *s, uint16_t addr, uint32_t value) {
  while (!s->avr->sreg[S_I]) avr_run(s->avr);
  uint8_t *d = s->avr->data + addr;
  d[0] = value;
  d[1] = value >> 8;
  d[2] = value >> 16;
  d[3] = value >> 24;
}

void sim_add_u32(sim_t *s, uint16_t addr, uint32_t delta) {
  while (!s->avr->sreg[S_I]) avr_run(s->avr);
  sim_write_u32(s, addr, sim_read_u32(s, addr) + delta);
}
