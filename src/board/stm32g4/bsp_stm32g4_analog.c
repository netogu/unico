#include "bsp.h"
#include "hal.h"
#include "hal_stm32_adc.h"
#include "log.h"
#include "stm32g4xx.h"

static adc_t adc1 = {.regs = ADC1};
static adc_t adc2 = {.regs = ADC2};
static adc_t adc3 = {.regs = ADC3};
static adc_t adc4 = {.regs = ADC4};
static adc_t adc5 = {.regs = ADC5};

#define ADC_SAMPLE_1_5_CYCLES 0x0   // 1.5 ADC clock cycles
#define ADC_SAMPLE_2_5_CYCLES 0x1   // 2.5 ADC clock cycles
#define ADC_SAMPLE_6_5_CYCLES 0x2   // 6.5 ADC clock cycles
#define ADC_SAMPLE_12_5_CYCLES 0x3  // 12.5 ADC clock cycles
#define ADC_SAMPLE_24_5_CYCLES 0x4  // 24.5 ADC clock cycles
#define ADC_SAMPLE_47_5_CYCLES 0x5  // 47.5 ADC clock cycles
#define ADC_SAMPLE_92_5_CYCLES 0x6  // 92.5 ADC clock cycles
#define ADC_SAMPLE_247_5_CYCLES 0x7 // 247.5 ADC clock cycles
#define ADC_SAMPLE_640_5_CYCLES 0x8 // 640.5 ADC clock cycles

#define ADC_SCALE_12BIT 3.3f / 4096.0f

//------------------------------------------------------
// OPAMP Config
//------------------------------------------------------
void board_opamp_setup(void) {

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  // VM_FB
  OPAMP1->CSR = 0 << OPAMP_CSR_VPSEL_Pos | // VINP0
                3 << OPAMP_CSR_VMSEL_Pos | // Follower
                OPAMP_CSR_HIGHSPEEDEN |
                OPAMP_CSR_OPAMPINTEN | // Internal connection
                OPAMP_CSR_OPAMPxEN;

  // IA_FB
  OPAMP3->CSR = 1 << OPAMP_CSR_VPSEL_Pos | // VINP1
                3 << OPAMP_CSR_VMSEL_Pos | // Follower, output on PB12
                OPAMP_CSR_HIGHSPEEDEN | OPAMP_CSR_OPAMPxEN;

  // IC_FB
  OPAMP4->CSR = 2 << OPAMP_CSR_VPSEL_Pos | // VINP2
                3 << OPAMP_CSR_VMSEL_Pos | // Follower, output on PB12
                OPAMP_CSR_HIGHSPEEDEN | OPAMP_CSR_OPAMPxEN;

  // IB_FB
  OPAMP5->CSR = 0 << OPAMP_CSR_VPSEL_Pos | // VINP0
                3 << OPAMP_CSR_VMSEL_Pos | OPAMP_CSR_HIGHSPEEDEN |
                OPAMP_CSR_OPAMPINTEN | // Internal connection
                OPAMP_CSR_OPAMPxEN;
}

//------------------------------------------------------
// ADC Config
//------------------------------------------------------
void board_adc_setup(void) {
  board_t *brd = board_get_handle();

  const hal_gpio_t analog_pins[] = {
      {.port = GPIOA, .pin = 0, .mode = GPIO_MODE_ANALOG}, // Vbatt - ADC12_1

      {.port = GPIOA,
       .pin = 1,
       .mode = GPIO_MODE_ANALOG}, // VM_FB - ADC1_13 + OPAMP1/COMP1

      {.port = GPIOA, .pin = 2, .mode = GPIO_MODE_ANALOG}, // VGD_MON - ADC1_3

      {.port = GPIOA,
       .pin = 3,
       .mode = GPIO_MODE_ANALOG}, // IM_FB (OCP) - COMP2

      {.port = GPIOB,
       .pin = 1,
       .mode = GPIO_MODE_ANALOG}, // IA_ADC_NC (OPAMP3_VOUT) - ADC3_1

      {.port = GPIOB,
       .pin = 11,
       .mode = GPIO_MODE_ANALOG}, // IC_FB (OPAMP4_VINP + COMP6)

      {.port = GPIOB,
       .pin = 12,
       .mode = GPIO_MODE_ANALOG}, // IC_ADC_NC (OPAMP4_VOUT) - ADC4_3

      {.port = GPIOB,
       .pin = 13,
       .mode = GPIO_MODE_ANALOG}, // IA_FB (OPAMP3_VINP + COMP5)

      {.port = GPIOB,
       .pin = 14,
       .mode = GPIO_MODE_ANALOG}, // IB_FB - ADC5_3 + (OPAMP5_VINP + COMP7)

      {.port = GPIOC, .pin = 0, .mode = GPIO_MODE_ANALOG}, // TEMP_A - ADC2_6

      {.port = GPIOC, .pin = 1, .mode = GPIO_MODE_ANALOG}, // TEMP_B - ADC2_7

      {.port = GPIOC, .pin = 2, .mode = GPIO_MODE_ANALOG}, // TEMP_C - ADC2_8

      {.port = GPIOC, .pin = 3, .mode = GPIO_MODE_ANALOG}, // IM_FB - ADC1_9

      {.port = GPIOC, .pin = 4, .mode = GPIO_MODE_ANALOG}, // TEMP_M - ADC2_5

      {.port = GPIOC, .pin = 5, .mode = GPIO_MODE_ANALOG}, // VL_MON - ADC2_11

      {.port = GPIOE,
       .pin = 7,
       .mode = GPIO_MODE_ANALOG}, // VM_FB (OVLO) - COMP4_VINP

      {.port = GPIOE, .pin = 8, .mode = GPIO_MODE_ANALOG}, // VC_FB - ADC5_6

      {.port = GPIOE, .pin = 9, .mode = GPIO_MODE_ANALOG}, // VA_FB - ADC3_IN2

      {.port = GPIOE, .pin = 11, .mode = GPIO_MODE_ANALOG}, // VB_FB - ADC345_15

  };

  brd->ai = (struct board_ai_s){

      //--- Adc1 ---

      .vbatt_mon =
          (hal_analog_input_t){
              .name = "vbat",
              .channel = 1,
              .scale = ADC_SCALE_12BIT * 23.3881f,
              .offset = 9.45841036e-3f,
              .units = "V",
          },

      .vm_fb =
          (hal_analog_input_t){
              .name = "vm",
              .channel = 13,
              .scale = ADC_SCALE_12BIT * 23.3881f,
              .offset = 9.45841036e-3f,
              .units = "V",
          },

      .vgd_mon =
          (hal_analog_input_t){
              .name = "vgd",
              .channel = 3,
              .scale = ADC_SCALE_12BIT * 1.945525292f,
              .offset = -0.2237354086f,
              .units = "V",
          },

      .vl_mon =
          (hal_analog_input_t){
              .name = "vl",
              .channel = 11,
              .scale = ADC_SCALE_12BIT * 15.31399397f,
              .offset = -14.75467388,
              .units = "V",
          },

      .im_fb =
          (hal_analog_input_t){
              .name = "im",
              .channel = 4,
              .scale = ADC_SCALE_12BIT,
              .offset = 0.0,
              .units = "A",
          },

      //--- Adc2 ---

      .temp_a =
          (hal_analog_input_t){
              .name = "ta",
              .channel = 6,
              .scale = ADC_SCALE_12BIT,
              .offset = 0.0,
              .units = "C",
          },

      .temp_b =
          (hal_analog_input_t){
              .name = "tb",
              .channel = 7,
              .scale = ADC_SCALE_12BIT,
              .offset = 0.0,
              .units = "C",
          },
      .temp_c =
          (hal_analog_input_t){
              .name = "tc",
              .channel = 8,
              .scale = ADC_SCALE_12BIT,
              .offset = 0.0,
              .units = "C",
          },

      .temp_m =
          (hal_analog_input_t){
              .name = "tm",
              .channel = 5,
              .scale = ADC_SCALE_12BIT,
              .offset = 0.0,
              .units = "C",
          },

      //--- Adc3 ---

      .ia_fb =
          (hal_analog_input_t){
              .name = "ia",
              .channel = 1,
              .scale = ADC_SCALE_12BIT / (0.001f * 20.0f),
              .offset = 0.0f,
              .units = "A",
          },

      .va_fb =
          (hal_analog_input_t){
              .name = "va",
              .channel = 2,
              .scale = 1.0,
              .offset = 0.0,
              .units = "V",
          },

      //--- Adc4 ---

      .ic_fb =
          (hal_analog_input_t){
              .name = "ic",
              .channel = 3,
              .scale = ADC_SCALE_12BIT / (0.001f * 20.0f),
              .offset = 0.0f,
              .units = "A",
          },

      .vb_fb =
          (hal_analog_input_t){
              .name = "vb",
              .channel = 15,
              .scale = ADC_SCALE_12BIT,
              .offset = 0.0,
              .units = "V",
          },

      //--- Adc5 ---

      .ib_fb =
          (hal_analog_input_t){
              .name = "ib",
              .channel = 3,
              .scale = ADC_SCALE_12BIT / (0.001f * 20.0f),
              .offset = 0.0f,
              .units = "A",
          },

      .vc_fb =
          (hal_analog_input_t){
              .name = "vc",
              .channel = 6,
              .scale = 1.0,
              .offset = 0.0,
              .units = "V",
          },
  };

  // Configure Analog Pins
  for (size_t i = 0; i < sizeof(analog_pins) / sizeof(analog_pins[0]); i++) {
    hal_gpio_init(&analog_pins[i]);
  }

  // Init Analog Analog Inputs
  hal_analog_input_t *ain_ptr = (hal_analog_input_t *)&brd->ai;
  for (size_t i = 0; i < sizeof(brd->ai) / sizeof(hal_analog_input_t); i++) {
    hal_analog_input_init(&ain_ptr[i]);
  }

  board_opamp_setup();

  // TODO: Configure comparators

  // adc_register_input(&adc1, &brd->ai.vbatt_mon, 'r',
  // ADC_SAMPLE_247_5_CYCLES);
  adc_register_input(&adc1, &brd->ai.vgd_mon, 'i', ADC_SAMPLE_92_5_CYCLES);
  adc_register_input(&adc1, &brd->ai.vm_fb, 'i', ADC_SAMPLE_92_5_CYCLES);
  adc_register_input(&adc1, &brd->ai.im_fb, 'i', ADC_SAMPLE_2_5_CYCLES);
  adc_register_input(&adc1, &brd->ai.vbatt_mon, 'i', ADC_SAMPLE_2_5_CYCLES);

  // adc_register_input(&adc2, &brd->ai.vl_mon, 'r', ADC_SAMPLE_247_5_CYCLES);
  adc_register_input(&adc2, &brd->ai.temp_b, 'i', ADC_SAMPLE_247_5_CYCLES);
  adc_register_input(&adc2, &brd->ai.temp_a, 'i', ADC_SAMPLE_247_5_CYCLES);
  adc_register_input(&adc2, &brd->ai.temp_c, 'i', ADC_SAMPLE_247_5_CYCLES);
  // adc_register_input(&adc2, &brd->ai.vl_mon, 'i', ADC_SAMPLE_247_5_CYCLES);
  adc_register_input(&adc2, &brd->ai.temp_m, 'i', ADC_SAMPLE_247_5_CYCLES);

  adc_register_input(&adc3, &brd->ai.ia_fb, 'i', ADC_SAMPLE_2_5_CYCLES);
  adc_register_input(&adc3, &brd->ai.va_fb, 'i', ADC_SAMPLE_2_5_CYCLES);

  adc_register_input(&adc4, &brd->ai.ic_fb, 'i', ADC_SAMPLE_2_5_CYCLES);
  adc_register_input(&adc4, &brd->ai.vb_fb, 'i', ADC_SAMPLE_2_5_CYCLES);

  adc_register_input(&adc5, &brd->ai.ib_fb, 'i', ADC_SAMPLE_2_5_CYCLES);
  adc_register_input(&adc5, &brd->ai.vc_fb, 'i', ADC_SAMPLE_2_5_CYCLES);

  printf("%s", timestamp());
  if (adc_init(&adc1) == 0) {
    LOG_OK("ADC1");
  } else {
    LOG_FAIL("ADC1");
  }

  printf("%s", timestamp());
  if (adc_init(&adc2) == 0) {
    LOG_OK("ADC2");
  } else {
    LOG_FAIL("ADC2");
  }

  printf("%s", timestamp());
  if (adc_init(&adc3) == 0) {
    LOG_OK("ADC3");
  } else {
    LOG_FAIL("ADC3");
  }

  printf("%s", timestamp());
  if (adc_init(&adc4) == 0) {
    LOG_OK("ADC4");
  } else {
    LOG_FAIL("ADC4");
  }

  printf("%s", timestamp());
  if (adc_init(&adc5) == 0) {
    LOG_OK("ADC5");
  } else {
    LOG_FAIL("ADC5");
  }
  // Enable EOC interrupt
  {
    ADC_TypeDef *adc_regs = (ADC_TypeDef *)adc3.regs;
    adc_regs->IER |= ADC_IER_JEOSIE;
    NVIC_EnableIRQ(ADC3_IRQn);
  }

  // adc_start_regular_sampling(&adc1);
  // adc_start_regular_sampling(&adc2);
  adc_start_injected_sampling(&adc1);
  adc_start_injected_sampling(&adc2);
  adc_start_injected_sampling(&adc3);
  adc_start_injected_sampling(&adc4);
  adc_start_injected_sampling(&adc5);
}

void ADC1_IRQHandler(void) {
  if (ADC1->ISR & ADC_ISR_JEOS) {
    board_t *brd = board_get_handle();
    hal_gpio_set(&brd->dio.test_pin0);
    hal_gpio_clear(&brd->dio.test_pin0);
    ADC1->ISR |= ADC_ISR_JEOS;
  }
}

void ADC3_IRQHandler(void) {
  if (ADC3->ISR & ADC_ISR_JEOS) {
    board_t *brd = board_get_handle();
    hal_gpio_set(&brd->dio.test_pin0);
    hal_gpio_clear(&brd->dio.test_pin0);
    ADC3->ISR |= ADC_ISR_JEOS;
  }
}
