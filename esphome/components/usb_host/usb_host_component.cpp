// Should not be needed, but it's required to pass CI clang-tidy checks
#if defined(USE_ESP32_VARIANT_ESP32P4) || defined(USE_ESP32_VARIANT_ESP32S2) || defined(USE_ESP32_VARIANT_ESP32S3) || \
    defined(USE_ESP32_VARIANT_ESP32S31) || defined(USE_ESP32_VARIANT_ESP32H4)
#include "usb_host.h"
#include <cinttypes>
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome::usb_host {

// CALLBACK CONTEXT: USB task (called from usb_host_client_handle_events in USB task)
static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *ptr) {
  auto *host = static_cast<USBHost *>(ptr);

  // Allocate event from pool
  UsbEvent *event = host->event_pool.allocate();
  if (event == nullptr) {
    // No events available - increment counter for periodic logging
    host->event_queue.increment_dropped_count();
    return;
  }

  // Queue events to be processed in main loop
  switch (event_msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV: {
      ESP_LOGD(TAG, "New device %d", event_msg->new_dev.address);
      event->type = EVENT_DEVICE_NEW;
      event->data.device_new.address = event_msg->new_dev.address;
      break;
    }
    case USB_HOST_CLIENT_EVENT_DEV_GONE: {
      ESP_LOGD(TAG, "Device gone");
      event->type = EVENT_DEVICE_GONE;
      event->data.device_gone.handle = event_msg->dev_gone.dev_hdl;
      break;
    }
    default:
      ESP_LOGD(TAG, "Unknown event %d", event_msg->event);
      host->event_pool.release(event);
      return;
  }

  // Push always succeeds: pool is sized to queue capacity (SIZE-1), so if
  // allocate() returned non-null, the queue cannot be full.
  host->event_queue.push(event);

  // Wake main loop immediately to process USB event
  App.wake_loop_threadsafe();
}

void USBHost::setup() {
  usb_host_config_t config{};

  if (usb_host_install(&config) != ESP_OK) {
    this->status_set_error(LOG_STR("usb_host_install failed"));
    this->mark_failed();
    return;
  }

  usb_host_client_config_t client_config{.is_synchronous = false,
                                         .max_num_event_msg = 5,
                                         .async = {.client_event_callback = client_event_cb, .callback_arg = this}};
  auto err = usb_host_client_register(&client_config, &this->handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "client register failed: %s", esp_err_to_name(err));
    this->status_set_error(LOG_STR("Client register failed"));
    this->mark_failed();
    return;
  }

  // Create and start USB task
  xTaskCreate(usb_task_fn, "usb_task",
              USB_TASK_STACK_SIZE,  // Stack size
              this,                 // Task parameter
              USB_TASK_PRIORITY,    // Priority (higher than main loop)
              &this->usb_task_handle_);

  if (this->usb_task_handle_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create USB task");
    this->mark_failed();
  }
}

void USBHost::usb_task_fn(void *arg) {
  auto *host = static_cast<USBHost *>(arg);
  host->usb_task_loop_();
}
void USBHost::usb_task_loop_() const {
  while (true) {
    usb_host_client_handle_events(this->handle_, portMAX_DELAY);
  }
}

void USBHost::loop() {
  int err;
  uint32_t event_flags;
  err = usb_host_lib_handle_events(0, &event_flags);
  if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
    ESP_LOGD(TAG, "lib_handle_events failed failed: %s", esp_err_to_name(err));
  }
  if (event_flags != 0) {
    ESP_LOGD(TAG, "Event flags %" PRIu32 "X", event_flags);
  }

  // Process any events from the USB task
  UsbEvent *event;
  while ((event = this->event_queue.pop()) != nullptr) {
    switch (event->type) {
      case EVENT_DEVICE_NEW:
        this->on_device_new_(event->data.device_new.address);
        break;
      case EVENT_DEVICE_GONE:
        this->on_device_gone_(event->data.device_gone.handle);
        break;
    }
    // Return event to pool for reuse
    this->event_pool.release(event);
  }

  // Log dropped events periodically
  uint16_t dropped = this->event_queue.get_and_reset_dropped_count();
  if (dropped > 0) {
    ESP_LOGW(TAG, "Dropped %u USB events due to queue overflow", dropped);
  }
}

void USBHost::on_device_new_(uint8_t address) {
  usb_device_handle_t device_handle;
  auto err = usb_host_device_open(this->handle_, address, &device_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Device %u open failed: %s", address, esp_err_to_name(err));
    return;
  }
  const usb_device_desc_t *desc;
  usb_device_info_t info;
  if ((err = usb_host_get_device_descriptor(device_handle, &desc)) != ESP_OK ||
      (err = usb_host_device_info(device_handle, &info)) != ESP_OK) {
    ESP_LOGW(TAG, "Device %u descriptor query failed: %s", address, esp_err_to_name(err));
    usb_host_device_close(this->handle_, device_handle);
    return;
  }
  char buf_manuf[DESC_STRING_BUF_SIZE];
  char buf_product[DESC_STRING_BUF_SIZE];
  char buf_serial[DESC_STRING_BUF_SIZE];
  ESP_LOGD(TAG, "Device %u: %04X:%04X; Manuf: %s; Prod: %s; Serial: %s", address, desc->idVendor, desc->idProduct,
           get_descriptor_string(info.str_desc_manufacturer, buf_manuf),
           get_descriptor_string(info.str_desc_product, buf_product),
           get_descriptor_string(info.str_desc_serial_num, buf_serial));

#ifdef USB_HOST_MATCHER_COUNT
  for (auto *matcher : this->matchers_) {
    auto *client = matcher->match(*desc, info);
    if (client == nullptr) {
      continue;
    }
    // The client owns the open handle from here on and closes it when the device goes away
    client->attach_(device_handle);
    return;
  }
#endif
  ESP_LOGW(TAG, "No driver for device %04X:%04X, ignoring it", desc->idVendor, desc->idProduct);
  usb_host_device_close(this->handle_, device_handle);
}

void USBHost::on_device_gone_(usb_device_handle_t device_handle) {
#ifdef USB_HOST_CLIENT_COUNT
  for (auto *client : this->clients_) {
    if (client->device_handle_ == device_handle) {
      client->disconnect();
      return;
    }
  }
#endif
}

}  // namespace esphome::usb_host

#endif  // USE_ESP32_VARIANT_ESP32P4 || USE_ESP32_VARIANT_ESP32S2 || USE_ESP32_VARIANT_ESP32S3 ||
        // USE_ESP32_VARIANT_ESP32S31 || USE_ESP32_VARIANT_ESP32H4
