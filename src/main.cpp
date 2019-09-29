// A0 and A1,   used to meassure signal
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
};

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

/* Dump half ADC buffer on usart */
void trigger_adc_buffer_to_usart(bool second_half)
{
  if (usart_busy)
  {
    error |= USART_BUSY_ERROR;
    return;
  }
  usart_busy = true;
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

void adc_dma_irq_handler()
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

volatile bool got_usart_irq = false;

/* Reset usart busy when data has been transferred */
void usart_irq_handler()
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
  //By doing this, we can read the last value sampled from the channels by reading the dataPoints array
  dma_set_priority(DMA1, DMA_CH1, DMA_PRIORITY_HIGH);

  myADC.setDMA(
      adc_buffer,
      adc_buffer_len,
      (DMA_MINC_MODE | DMA_CIRC_MODE | DMA_HALF_TRNS | DMA_TRNS_CMPLT),
      adc_dma_irq_handler);
  
  //start the conversion.
  //because the ADC is set as continuous mode and in circular fashion, this can be done
  //on setup().
  myADC.startConversion();
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

void handle_cmd(uint8_t cmd)
{
  switch (cmd)
  {
  case CMD_STOP:
    stop();
    break;
  case CMD_START:
    start();
    break;
  case CMD_GET_SAMPLERATE:
    Serial1.println(adc_samplerate);
    break;
  case CMD_GET_ERROR:
    Serial1.println(error);
    break;
  }
}

extern "C"
{
  void __irq_usart1(void)
  {
    // Reading clears the thing
    uint8_t cmd = 0;
    while ((USART1_BASE->SR & USART_SR_RXNE) != 0)
    {
      cmd = USART1_BASE->DR & 0xff;
    };
    if (cmd != 0)
    {
      handle_cmd(cmd);
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

  dma_attach_interrupt(DMA1, DMA_CH4, usart_irq_handler);

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

  // TODO, if error, then set LED or somesuch
};
