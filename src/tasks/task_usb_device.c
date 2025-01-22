
#include "bsp.h"
#include "hal.h"
#include "rtos.h"
#include "shell.h"
#include "tasklist.h"
#include "tiny_printf.h"
#include "tusb.h"

/*-----------------------------------------------------------*/
// Timers
/*-----------------------------------------------------------*/
/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */

enum {
  BLINK_NOT_MOUNTED = 1000,
  BLINK_MOUNTED = 200,
  BLINK_SUSPENDED = 2500,
};

static TaskHandle_t task_usb_device_handle;
static StaticTask_t task_usb_device_tcb;
static StackType_t task_usb_stack[TASK_STACK_SIZE_USB];
static void task_usb_device_app(void *parameters);

TaskHandle_t task_usb_device_init(void) {
  // Create the task

  task_usb_device_handle = xTaskCreateStatic(
      task_usb_device_app, xstr(TASK_NAME_USB), TASK_STACK_SIZE_USB, NULL,
      TASK_PRIORITY_USB, task_usb_stack, &task_usb_device_tcb);

  if (task_usb_device_handle != NULL) {
    tusb_init();
    return task_usb_device_handle;
  } else {
    return NULL;
  }
}

void task_usb_device_app(void *parameters) {
  /* Unused parameters. */
  (void)parameters;

  while (1) {
    tud_task();
    tud_cdc_write_flush();
    vTaskDelay(TASK_DELAY_USB);
  }
}

//--------------------------------------------------------------------+
// USB CDC Device Callbacks
//--------------------------------------------------------------------+

void tud_mount_cb(void) {
  // Invoked when device is mounted

  //   xTimerChangePeriod(led_blink_timer, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
}

// Invoked when device is un-mounted
void tud_umount_cb(void) {

  //   xTimerChangePeriod(led_blink_timer, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), 0);
}
//
// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
  (void)remote_wakeup_en;
  //   xTimerChangePeriod(led_blink_timer, pdMS_TO_TICKS(BLINK_SUSPENDED), 0);
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
  if (tud_mounted()) {
    // xTimerChangePeriod(led_blink_timer, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
  } else {
  }
  // xTimerChangePeriod(led_blink_timer, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), 0);
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
  (void)itf;
  (void)rts;

  // TODO set some indicator
  if (dtr) {
  } else {
    // Terminal disconnected
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf) { (void)itf; }
