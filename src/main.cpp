// TODO
// * Set up ADC
//     - self-calibration
// * Set up DMA
//     - alloc buffer, how big?
// * Figure out where set ADC<->DMA speed
// * Output on serial

#include <Arduino.h>
#include <STM32ADC.h>
#include <libmaple/timer.h>
#include <libmaple/usart.h>

STM32ADC myADC(ADC1);
//Channels to be acquired.
uint8 pins[] = {PA0, PA1};

const size_t nPins = sizeof(pins) / sizeof(uint8);

// Array for the ADC data
const size_t adc_buffer_len = 20 * nPins;
uint16_t adc_buffer[adc_buffer_len];

uint32 nirqs = 0;
volatile bool did_irq = false;
volatile bool handle_half_complete = false;
volatile bool handle_complete = false;
volatile bool dma_error = false;
uint32 nirqs_e = 0;
uint32 nirqs_o = 0;


void dma_irq_handler()
{
  const auto irq_cause = dma_get_irq_cause(DMA1, DMA_CH1);
  nirqs++;
  did_irq = true;
  switch (irq_cause)
  {
  case DMA_TRANSFER_HALF_COMPLETE:
    handle_half_complete = true;
    break;

  case DMA_TRANSFER_COMPLETE:
    handle_complete = true;
    break;

  case DMA_TRANSFER_ERROR:
    dma_error = true;
    break;

  default:
    nirqs_o++;
    break;
  }
  return;
}

volatile bool got_usart_irq = false;
volatile bool usart_busy = false;
volatile dma_irq_cause usart_irq_cause;

void usart_irq_handler()
{
  usart_irq_cause = dma_get_irq_cause(DMA1, DMA_CH4);
  usart_busy = false;
  got_usart_irq = true; 
}


void _dump_half(bool second_half) {
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
void dump_first_half()
{
  _dump_half(false);
} 

void dump_second_half()
{
  _dump_half(true);
} 

void setup_timer()
{
  // TODO: WTF, timer_init makes it not work, while just rcc_clk_enable is fine...
  // timer_init(TIMER1);
  rcc_clk_enable(TIMER1->clk_id);
  timer_set_prescaler(TIMER1, 1024);
  timer_set_reload(TIMER1, 1024);
  timer_set_compare(TIMER1, TIMER_CH1, 32);
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

  //calibrate ADC
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
      dma_irq_handler);

  //start the conversion.
  //because the ADC is set as continuous mode and in circular fashion, this can be done
  //on setup().
  myADC.startConversion();
}

void setup()
{
  Serial1.begin(614400);
  Serial1.println("Serial1");
  Serial1.println("Serial1 agin");

  for (size_t i = 0; i < adc_buffer_len; i++) {
    adc_buffer[i] = 0;
  }

  // Prepare usart DMA
  USART1->regs->CR3 |= USART_CR3_DMAT;
  dma_attach_interrupt(DMA1, DMA_CH4, usart_irq_handler);

  setup_timer();
  setup_adc();
}


bool too_slow = false;
void dump_buffer() {
  did_irq = false;
  if (usart_busy) {
    Serial1.println("USART BUSY!!");
    too_slow = true;
    return;
  }
  if (handle_half_complete) {
    handle_half_complete = false;
    dump_first_half();
  }

  if (handle_complete) {
    handle_complete = false;
    dump_second_half();
  }

  if (did_irq) {
    too_slow = true;
  }
}


uint64 step = 0;
uint64 nirq_last = 0;
uint32 t_last = 0;
bool run = true;

void loop()
{
  //send the latest data acquired when the button is pushed.

  // TODO critical region
  // if (!did_irq) {
  //   return;
  // }
  if (did_irq) {
    dump_buffer();
  }

  step++;
  if (step % 500000 == 0)
  {
    uint32 t_now = millis();
    uint32 t_elapsed = t_now - t_last;
    // One IRQ per half transfer
    // 16 bytes per point, but only half of them
    auto n_points = adc_buffer_len * (nirqs - nirq_last);
    auto kbps = (16 / 2 * n_points) / t_elapsed;
    nirq_last = nirqs;
    t_last = t_now;

    uint16 counter = timer_get_count(TIMER1);
    uint32 reading = myADC.getData();
    Serial1.println("----------");
    Serial1.print("TIM1.SR: ");
    Serial1.println(timer_get_status(TIMER1));
    Serial1.print("ADC1.CR2: ");
    Serial1.println(ADC1->regs->CR2);
    Serial1.print("First meassures: ");
    Serial1.print(adc_buffer[0]);
    Serial1.print(", ");
    Serial1.println(adc_buffer[1]);
    Serial1.print("counter: ");
    Serial1.println(counter);
    Serial1.print("reading: ");
    Serial1.println(reading);
    Serial1.print("kbps: ");
    Serial1.println(kbps);
    Serial1.print("nirqs: ");
    Serial1.println(nirqs); 
    Serial1.print("nirqs_e: ");
    Serial1.println(nirqs_e);
    Serial1.print("err: ");
    Serial1.println(dma_error);
    Serial1.print("too_slow: ");
    Serial1.println(too_slow);
    Serial1.print("got_usart_irq: ");
    Serial1.println(got_usart_irq);
    Serial1.print("usart_irq_cause: ");
    Serial1.println(usart_irq_cause);
    run = true;
    got_usart_irq = false;
  }

}; //end loop
