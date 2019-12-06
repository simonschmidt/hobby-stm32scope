// A0 and A1,   used to meassure signal
// USART used for data, pins, A9 TX, A10 RX
// TIM1 (timer) that tells ADC to meassure
// ADC (analog 2 digital converter)  gives us 12bit measurment of voltage
// DMA (direct memory access)  shuffles them to RAM
// DMA shuffles the RAM out over serial data port to computer

#include <Arduino.h>
#include <STM32ADC.h>
#include <libmaple/timer.h>
#include <libmaple/usart.h>

static const uint32_t baud_rate = 1200000;
static const uint8_t error_pin = PB12;

static const uint8_t adc_clock_divider = 2;
static const adc_smp_rate adc_sampletime = ADC_SMPR_7_5; // Also change in parameter_sanity_check

static const uint32_t adc_timer_prescaler = 40;
static const uint32_t adc_timer_arr = 64;
static const uint32_t adc_samplerate = CLOCK_SPEED_HZ / adc_timer_prescaler / adc_timer_arr;
// Time to *perform* one sample
// 11.6:
// Tconv  = sampling time + 12.5
// with 7.5 cycles sampling time, TConv = 19.5 adc cycles
// with 72MHz cpu clock, and 8 divider on adc
// total time = (19.5 cycles) / (72 MHz / 8) = 2.17µs
// so can go up to 460ksps with current settings, but transmission is bottleneck
//
// With adc div 8,  sample time 77.5 cycles, 72MHz clock
// 1 adc cycle = 8/72MHz
// total_time = 77.5 + 12.5 adc cycles = 90 adc cycles = 10µs
// up to 100ksps
//
// With adc div 8, sample time 239.5 cycles
// total time = 239.5 + 12.5 adc cycles = 252 adc cycles = 28µs
// up to 35ksps



enum commands
{
  CMD_STOP = 1,
  CMD_START = 2,
  CMD_GET_SAMPLERATE =  3,
  CMD_GET_ERROR = 4,
  CMD_SET_TRIGGER = 5,
};
enum cmd_state_type
{
  CMD_STATE_IDLE = 0,
  CMD_STATE_READ_ARG = 1,
};

volatile auto cmd_state = CMD_STATE_IDLE;
volatile uint8_t cmd_buf[6];
volatile size_t cmd_buf_ix = 0;

enum error_type
{
  NO_ERROR = 0,
  USART_BUSY_ERROR = 1,
  ADC_DMA_ERROR = 1 << 2,
  USART_DMA_ERROR = 1 << 3,
  INVALID_ADC_CLK_DIV = 1 << 4,
  BAD_PARAMETERS_TIMER_FASTER_THAN_ADC = 1 << 5,
  BAD_PARAMETERS_SERIAL_TOO_SLOW = 1 << 6,
};
volatile uint32_t error = NO_ERROR;

STM32ADC myADC(ADC1);
//Channels to be acquired.
uint8 pins[] = {PA0, PA1};

const size_t nPins = sizeof(pins) / sizeof(uint8);

// Array for the ADC data
const size_t adc_buffer_len = 200 * nPins;
uint16_t adc_buffer[adc_buffer_len];

uint32 nirqs_o = 0;

volatile bool usart_busy = false;


// AWD
// Set when AWD fired.
// TODO figure out how to tag the exact event that triggered the interrupt, possible?
volatile bool adc_awd_did_interrupt = false;


/* Dump half ADC buffer on usart */
void trigger_adc_buffer_to_usart(bool second_half)
{
  if (usart_busy)
  {
    error |= USART_BUSY_ERROR;
    return;
  }
  usart_busy = true;

  // Tag interrupt happened
  // This sucks as it will wiggle around and not be exactly when the AWD fired.
  if(adc_awd_did_interrupt) {
    if (second_half) {
      adc_buffer[adc_buffer_len / 2] |= 1<<14;
    } else {
      adc_buffer[0] |= 1<<14;
    }
    adc_awd_did_interrupt = false;
  }

  dma_setup_transfer(
      DMA1,
      DMA_CH4,
      &USART1->regs->DR,
      DMA_SIZE_8BITS,
      adc_buffer + (second_half ? adc_buffer_len / 2: 0),
      DMA_SIZE_8BITS,
      (DMA_MINC_MODE | DMA_FROM_MEM | DMA_TRNS_CMPLT));
  // Note; /2 because we're transfering half of it, *2 because 16bit data sent as 8bit
  dma_set_num_transfers(DMA1, DMA_CH4, (adc_buffer_len) * 2 / 2);
  dma_enable(DMA1, DMA_CH4);
}

void on_adc_dma_complete()
{
  const auto irq_cause = dma_get_irq_cause(DMA1, DMA_CH1);
  switch (irq_cause)
  {
  case DMA_TRANSFER_HALF_COMPLETE:
    trigger_adc_buffer_to_usart(false);
    break;

  case DMA_TRANSFER_COMPLETE:
    trigger_adc_buffer_to_usart(true);
    break;

  case DMA_TRANSFER_ERROR:
    error |= ADC_DMA_ERROR;
    break;

  default:
    break;
  }
  return;
}

void on_adc_awd_interrupt()
{
  // Disable watchdog (interrupt bit already cleared)
  ADC1->regs->CR1 &= ~ADC_CR1_AWDEN;

  DMA1->regs->CMAR1;
  dma_channel_reg_map *chan_regs;


  // TODO come back to this, this is too error prone
  // chan_regs = dma_channel_regs(DMA1, DMA_CH1);
  // uint16_t * value = reinterpret_cast<uint16_t*>(chan_regs->CMAR);
  adc_awd_did_interrupt = true;
}

volatile bool got_usart_irq = false;

/* Reset usart busy when data has been transferred */
void on_dma_to_usart_done()
{
  const dma_irq_cause usart_irq_cause = dma_get_irq_cause(DMA1, DMA_CH4);
  switch (usart_irq_cause)
  {
  case DMA_TRANSFER_ERROR:
    error |= USART_DMA_ERROR;
  default:
    break;
  }
  usart_busy = false;
  got_usart_irq = true;
}

void setup_timer()
{
  // TODO: WTF, timer_init makes it not work, while just rcc_clk_enable is fine...
  // timer_init(TIMER1);
  rcc_clk_enable(TIMER1->clk_id);
  timer_set_prescaler(TIMER1, adc_timer_prescaler);
  timer_set_reload(TIMER1, adc_timer_arr);
  timer_set_compare(TIMER1, TIMER_CH1, adc_timer_arr / 2);
  timer_oc_set_mode(TIMER1, TIMER_CH1, TIMER_OC_MODE_PWM_1, 0);
  timer_cc_enable(TIMER1, TIMER_CH1);
  timer_cc_set_pol(TIMER1, TIMER_CH1, 1);

  timer_resume(TIMER1);
}

void setup_adc()
{
  // Set up our analog pin(s)
  for (size_t j = 0; j < nPins; j++)
    pinMode(pins[j], INPUT_ANALOG);

  // Slow ADC the fuck down, this sets the ADCPRE (hopefully)
  // 00: PCLK2/2
  // 01: PCLK2/4
  // 10: PCLK2/6
  // 11: PCLK2/8
  uint32_t div = RCC_ADCPRE_PCLK_DIV_8;
  switch (adc_clock_divider) {
    case 2:
      div = RCC_ADCPRE_PCLK_DIV_2;
      break;
    case 4:
      div = RCC_ADCPRE_PCLK_DIV_4;
      break;
    case 6:
      div = RCC_ADCPRE_PCLK_DIV_6;
      break;
    case 8:
      div = RCC_ADCPRE_PCLK_DIV_8;
      break;
    default:
      error |= INVALID_ADC_CLK_DIV;    
  };
  RCC_BASE->CFGR = (RCC_BASE->CFGR & ~RCC_CFGR_ADCPRE) | div;

  delay_us(1000); // Maybe not needed, whatever

  // ADC calibration should be done at startup
  myADC.calibrate();

  // myADC.setSampleRate(ADC_SMPR_1_5); //set the Sample Rate
  // myADC.setSampleRate(ADC_SMPR_7_5);
  // myADC.setSampleRate(ADC_SMPR_239_5);
  myADC.setSampleRate(adc_sampletime);
  myADC.setScanMode();        //set the ADC in Scan mode.
  myADC.setPins(pins, nPins); //set how many and which pins to convert.
  myADC.resetContinuous();
  // myADC.setContinuous();
  myADC.setTrigger(ADC_EXT_EV_TIM1_CC1);

  //set the DMA transfer for the ADC.
  //in this case we want to increment the memory side and run it in circular mode
  dma_set_priority(DMA1, DMA_CH1, DMA_PRIORITY_HIGH);

  myADC.setDMA(
      adc_buffer,
      adc_buffer_len,
      (DMA_MINC_MODE | DMA_CIRC_MODE | DMA_HALF_TRNS | DMA_TRNS_CMPLT),
      on_adc_dma_complete);
  
  //start the conversion.
  myADC.startConversion();

  // enable_awd_irq(ADC1);
  adc_attach_interrupt(ADC1, ADC_AWD, on_adc_awd_interrupt);
}

void stop()
{
  timer_pause(TIMER1);
}

void start()
{
  timer_resume(TIMER1);
  digitalWrite(error_pin, LOW);
}


/**
 * Trigger mode
 * - Analog watchog is set up for the specified threshold
 * - When watchdog triggers, the measured value will be tagged
 * - User can then detect this tag
 * - TODO tag should go in the high unused bits, but how to target it correctly in time?
 */
void handle_trigger_cmd() {
  uint8_t cmd = cmd_buf[0];
  if (cmd != CMD_SET_TRIGGER) {
    // TODO set error
    return;
  }
  if (cmd_buf_ix < 5) {
    // Wait for more arguments
    cmd_state = CMD_STATE_READ_ARG;
    return;
  }

  uint8 channel = cmd_buf[1];
  uint16_t value_low = (((uint16_t)cmd_buf[2]) << 8) + cmd_buf[3]; // TOD low and high, maybe multi-byte
  uint16_t value_high = (((uint16_t)cmd_buf[4]) << 8) + cmd_buf[5];

  // TODO check that arguments make sense


  // set_awd_channel(ADC1, channel);
  ADC1->regs->CR1 |= (channel & ADC_CR1_AWDCH);

  // set_awd_low_limit(ADC1, (uint32)value_low);
  // set_awd_high_limit(ADC1, (uint32)value_high);
  ADC1->regs->LTR = value_low;
  ADC1->regs->HTR = value_high;

  //   enable_awd(ADC1);
  ADC1->regs->CR1 |= ADC_CR1_AWDEN;

  cmd_state = CMD_STATE_IDLE;
}

void handle_cmd()
{
  uint8_t cmd = cmd_buf[0];
  switch (cmd)
  {
  case CMD_STOP:
    stop();
    cmd_state = CMD_STATE_IDLE;
    break;
  case CMD_START:
    start();
    cmd_state = CMD_STATE_IDLE;
    break;
  case CMD_GET_SAMPLERATE:
    Serial1.write((uint8_t*) &adc_samplerate, 4);
    // Serial1.println(adc_samplerate);
    cmd_state = CMD_STATE_IDLE;
    break;
  case CMD_GET_ERROR:
    Serial1.println(error);
    cmd_state = CMD_STATE_IDLE;
    break;
  case CMD_SET_TRIGGER:
    handle_trigger_cmd();
    break;
  }
}


extern "C"
{
  void __irq_usart1(void)
  {
    // Reading clears the thing
    while ((USART1_BASE->SR & USART_SR_RXNE) != 0) {
      switch (cmd_state) {
        case CMD_STATE_IDLE:
          cmd_buf_ix = 0;
          // fallthrough
        case CMD_STATE_READ_ARG: {
          cmd_buf[cmd_buf_ix] = USART1_BASE->DR & 0xff;
          if (cmd_buf[0] != 0) {
            handle_cmd();
            cmd_buf_ix += 1;
          }
          break;
        }
      }
    }

    // This just ties in to existing Serial stuff so Serial1.print keeps working
    /* TXE signifies readiness to send a byte to DR. */
    if ((USART1_BASE->CR1 & USART_CR1_TXEIE) && (USART1_BASE->SR & USART_SR_TXE))
    {
      if (!rb_is_empty(USART1->wb))
        USART1_BASE->DR = rb_remove(USART1->wb);
      else
        USART1_BASE->CR1 &= ~((uint32)USART_CR1_TXEIE); // disable TXEIE
    }
  }
}


void parameter_sanity_check() {
  uint32_t channels = nPins;

  // This is 10(adc_smpr + 12.5)
  uint32_t sampletime_cycles_x10 = 75 + 125;

  // Time it takes for adc to sample all channels, in PCLK cycles
  uint32_t sampletime = sampletime_cycles_x10 * adc_clock_divider * channels / 10;

  // Time between timer triggers
  uint32_t timer_interval = adc_timer_prescaler * adc_timer_arr;


  if (timer_interval <= sampletime) {
    error |= BAD_PARAMETERS_TIMER_FASTER_THAN_ADC;
  }

  uint32_t send_capacity_bytes_per_second = baud_rate / 10;  // 10 bits per byte (2 for star/stop bits)
  uint32_t requested_bytes_per_second = 2 * channels * (CLOCK_SPEED_HZ / timer_interval);
  if (send_capacity_bytes_per_second <= requested_bytes_per_second) {
    error |= BAD_PARAMETERS_SERIAL_TOO_SLOW;
  }
}

void setup()
{
  Serial1.begin(baud_rate);
  pinMode(error_pin, OUTPUT);
  parameter_sanity_check();

  for (size_t i = 0; i < adc_buffer_len; i++)
  {
    adc_buffer[i] = 0;
  }

  // Prepare usart DMA
  USART1->regs->CR3 |= USART_CR3_DMAT;

  // Interrupt on incoming
  USART1->regs->CR1 |= USART_CR1_RXNEIE;

  dma_attach_interrupt(DMA1, DMA_CH4, on_dma_to_usart_done);

  setup_timer();
  setup_adc();

  stop(); // TODO no!
}

void loop()
{
  if (error != NO_ERROR)
  {
    digitalWrite(error_pin, HIGH);
  }
  // Everything relies on interrupts, do nothing
  asm("wfi \n");
};
