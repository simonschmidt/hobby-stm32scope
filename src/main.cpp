// TODO
// Control input channel
#include <Arduino.h>
#include <STM32ADC.h>
#include <libmaple/timer.h>
#include <libmaple/usart.h>

static const uint32_t adc_timer_prescaler = 32;
static const uint32_t adc_timer_arr = 64;
static const uint32_t adc_samplerate = CLOCK_SPEED_HZ / adc_timer_prescaler / adc_timer_arr;

// A0 and A1,   used to meassure signal
// TIM1 (timer) that tells ADC to meassure
// ADC (analog 2 digital converter)  gives us 12bit measurment of voltage
// DMA (direct memory access)  shuffles them to RAM
// DMA shuffles the RAM out over serial data port to computer

enum commands
{
  CMD_STOP = 1,
  CMD_START = 2,
  CMD_GET_SAMPLERATE = 3,
};

enum error_type
{
  NO_ERROR = 0,
  USART_BUSY_ERROR = 1,
  ADC_DMA_ERROR = 1 << 2,
  USART_DMA_ERROR = 1 << 3,
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
      adc_buffer + (second_half ? adc_buffer_len / 2 : 0),
      DMA_SIZE_8BITS,
      (DMA_MINC_MODE | DMA_FROM_MEM | DMA_TRNS_CMPLT));
  dma_set_num_transfers(DMA1, DMA_CH4, adc_buffer_len / 2);
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
  RCC_BASE->CFGR |= RCC_ADCPRE_PCLK_DIV_8;

  delay_us(1000); // Maybe not needed, whatever

  // ADC calibration should be done at startup
  myADC.calibrate();

  // myADC.setSampleRate(ADC_SMPR_1_5); //set the Sample Rate
  myADC.setSampleRate(ADC_SMPR_239_5); // Slowed down for now, TODO
  myADC.setScanMode();                 //set the ADC in Scan mode.
  myADC.setPins(pins, nPins);          //set how many and which pins to convert.
  myADC.resetContinuous();
  // myADC.setContinuous();
  myADC.setTrigger(ADC_EXT_EV_TIM1_CC1);

  //set the DMA transfer for the ADC.
  //in this case we want to increment the memory side and run it in circular mode
  //By doing this, we can read the last value sampled from the channels by reading the dataPoints array
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

void setup()
{
  Serial1.begin(1000000);

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
  // Everything relies on interrupts, do nothing
  asm("wfi \n");
  // TODO, if error, then set LED or somesuch
};
