#include "bsp.h"
#include "log.h"
#include "stm32g4.h"

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

#define ADC_SCALE_12BIT 3.3 / 4096.0

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

  const gpio_t analog_pins[] = {
      {.port = GPIO_PORT_A,
       .pin = GPIO_PIN_0,
       .mode = GPIO_MODE_ANALOG}, // Vbatt - ADC12_1

      {.port = GPIO_PORT_A,
       .pin = GPIO_PIN_1,
       .mode = GPIO_MODE_ANALOG}, // VM_FB - ADC1_13 + OPAMP1/COMP1

      {.port = GPIO_PORT_A,
       .pin = GPIO_PIN_2,
       .mode = GPIO_MODE_ANALOG}, // VGD_MON - ADC1_3

      {.port = GPIO_PORT_A,
       .pin = GPIO_PIN_3,
       .mode = GPIO_MODE_ANALOG}, // IM_FB (OCP) - COMP2

      {.port = GPIO_PORT_B,
       .pin = GPIO_PIN_1,
       .mode = GPIO_MODE_ANALOG}, // IA_ADC_NC (OPAMP3_VOUT) - ADC3_1

      {.port = GPIO_PORT_B,
       .pin = GPIO_PIN_11,
       .mode = GPIO_MODE_ANALOG}, // IC_FB (OPAMP4_VINP + COMP6)

      {.port = GPIO_PORT_B,
       .pin = GPIO_PIN_12,
       .mode = GPIO_MODE_ANALOG}, // IC_ADC_NC (OPAMP4_VOUT) - ADC4_3

      {.port = GPIO_PORT_B,
       .pin = GPIO_PIN_13,
       .mode = GPIO_MODE_ANALOG}, // IA_FB (OPAMP3_VINP + COMP5)

      {.port = GPIO_PORT_B,
       .pin = GPIO_PIN_14,
       .mode = GPIO_MODE_ANALOG}, // IB_FB - ADC5_3 + (OPAMP5_VINP + COMP7)

      {.port = GPIO_PORT_C,
       .pin = GPIO_PIN_0,
       .mode = GPIO_MODE_ANALOG}, // TEMP_A - ADC2_6

      {.port = GPIO_PORT_C,
       .pin = GPIO_PIN_1,
       .mode = GPIO_MODE_ANALOG}, // TEMP_B - ADC2_7

      {.port = GPIO_PORT_C,
       .pin = GPIO_PIN_2,
       .mode = GPIO_MODE_ANALOG}, // TEMP_C - ADC2_8

      {.port = GPIO_PORT_C,
       .pin = GPIO_PIN_3,
       .mode = GPIO_MODE_ANALOG}, // IM_FB - ADC1_9

      {.port = GPIO_PORT_C,
       .pin = GPIO_PIN_4,
       .mode = GPIO_MODE_ANALOG}, // TEMP_M - ADC2_5

      {.port = GPIO_PORT_E,
       .pin = GPIO_PIN_7,
       .mode = GPIO_MODE_ANALOG}, // VM_FB (OVLO) - COMP4_VINP

      {.port = GPIO_PORT_E,
       .pin = GPIO_PIN_8,
       .mode = GPIO_MODE_ANALOG}, // VC_FB - ADC5_6

      {.port = GPIO_PORT_E,
       .pin = GPIO_PIN_9,
       .mode = GPIO_MODE_ANALOG}, // VA_FB - ADC3_IN2

      {.port = GPIO_PORT_E,
       .pin = GPIO_PIN_11,
       .mode = GPIO_MODE_ANALOG}, // VB_FB - ADC345_15

  };

  brd->ai = (struct board_ai_s){

      //--- Adc1 ---

      .vbatt_mon =
          (adc_input_t){
              .name = "vbatt_mon",
              .channel = 1,
              .scale = 1.0,
              .offset = 0.0,
              .units = "V",
          },

      .vm_fb =
          (adc_input_t){
              .name = "vm_fb",
              .channel = 1,
              .scale = ADC_SCALE_12BIT * (1 / 43.2e-3),
              .offset = 0.0,
              .units = "V",
          },

      .vgd_mon =
          (adc_input_t){
              .name = "vgd_mon",
              .channel = 1,
              .scale = 1.0,
              .offset = 0.0,
              .units = "V",
          },

      .im_fb =
          (adc_input_t){
              .name = "im_fb",
              .channel = 9,
              .scale = 1.0,
              .offset = 0.0,
              .units = "A",
          },

      //--- Adc2 ---

      .temp_a =
          (adc_input_t){
              .name = "temp_a",
              .channel = 6,
              .scale = 1.0,
              .offset = 0.0,
              .units = "C",
          },

      .temp_b =
          (adc_input_t){
              .name = "temp_b",
              .channel = 7,
              .scale = 1.0,
              .offset = 0.0,
              .units = "C",
          },
      .temp_c =
          (adc_input_t){
              .name = "temp_c",
              .channel = 8,
              .scale = 1.0,
              .offset = 0.0,
              .units = "C",
          },

      .temp_m =
          (adc_input_t){
              .name = "temp_a",
              .channel = 5,
              .scale = 1.0,
              .offset = 0.0,
              .units = "C",
          },

      //--- Adc3 ---

      .ia_fb =
          (adc_input_t){
              .name = "ia_fb",
              .channel = 1,
              .scale = ADC_SCALE_12BIT,
              .offset = 0.0,
              .units = "A",
          },

      .va_fb =
          (adc_input_t){
              .name = "va_fb",
              .channel = 2,
              .scale = 1.0,
              .offset = 0.0,
              .units = "V",
          },

      //--- Adc4 ---

      .ic_fb =
          (adc_input_t){
              .name = "ic_fb",
              .channel = 3,
              .scale = ADC_SCALE_12BIT,
              .offset = 0.0,
              .units = "A",
          },

      .vb_fb =
          (adc_input_t){
              .name = "vb_fb",
              .channel = 15,
              .scale = 1.0,
              .offset = 0.0,
              .units = "V",
          },

      //--- Adc5 ---

      .ib_fb =
          (adc_input_t){
              .name = "ib_fb",
              .channel = 3,
              .scale = ADC_SCALE_12BIT,
              .offset = 0.0,
              .units = "A",
          },

      .vc_fb =
          (adc_input_t){
              .name = "vc_fb",
              .channel = 6,
              .scale = 1.0,
              .offset = 0.0,
              .units = "V",
          },
  };

  // Configure Analog Pins
  for (size_t i = 0; i < sizeof(analog_pins) / sizeof(analog_pins[0]); i++) {
    gpio_pin_init(&analog_pins[i]);
  }

  board_opamp_setup();

  // TODO: Configure comparators

  adc_register_input(&adc1, &brd->ai.vbatt_mon, 'r', ADC_SAMPLE_247_5_CYCLES);
  adc_register_input(&adc1, &brd->ai.vm_fb, 'i', ADC_SAMPLE_247_5_CYCLES);
  adc_register_input(&adc1, &brd->ai.vgd_mon, 'i', ADC_SAMPLE_247_5_CYCLES);
  adc_register_input(&adc1, &brd->ai.im_fb, 'i', ADC_SAMPLE_247_5_CYCLES);

  adc_register_input(&adc2, &brd->ai.temp_a, 'i', ADC_SAMPLE_247_5_CYCLES);
  adc_register_input(&adc2, &brd->ai.temp_b, 'i', ADC_SAMPLE_247_5_CYCLES);
  adc_register_input(&adc2, &brd->ai.temp_c, 'i', ADC_SAMPLE_247_5_CYCLES);
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

  adc_start_regular_sampling(&adc1);
  adc_start_regular_sampling(&adc2);
  adc_start_injected_sampling(&adc1);
  adc_start_injected_sampling(&adc2);
  adc_start_injected_sampling(&adc3);
  adc_start_injected_sampling(&adc4);
  adc_start_injected_sampling(&adc5);
}

void ADC3_IRQHandler(void) {
  if (ADC3->ISR & ADC_ISR_JEOS) {
    board_t *brd = board_get_handle();
    gpio_pin_set(&brd->dio.test_pin0);
    gpio_pin_clear(&brd->dio.test_pin0);
    ADC3->ISR |= ADC_ISR_JEOS;
  }
}
