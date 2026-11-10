/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020, Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if CFG_TUH_ENABLED

#include "tusb.h"
#include "usbh_classdriver.h"
#include <stdlib.h>

enum
{
  STAGE_SETUP,
  STAGE_DATA,
  STAGE_ACK
};

typedef struct
{
  tusb_control_request_t request TU_ATTR_ALIGNED(4);

  uint8_t stage;
  uint8_t* buffer;
  tuh_control_complete_cb_t complete_cb;
  bool done;
  bool bypass;
  uint8_t retry;
  xfer_result_t result;
} usbh_control_xfer_t;

static usbh_control_xfer_t _ctrl_xfer;

//CFG_TUSB_MEM_SECTION CFG_TUSB_MEM_ALIGN
//static uint8_t _tuh_ctrl_buf[CFG_TUH_ENUMERATION_BUFSIZE];

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

static bool ctrlSync (uint8_t daddr, tusb_control_request_t const * request, xfer_result_t result)
{
  _ctrl_xfer.result = result;
  _ctrl_xfer.done = true;
  return true;
}

static void tuh_control_event_cb(hcd_event_t *event, bool start) {
  if (start) {
    TU_LOG2("Control Setup (addr = %u)\n", event->dev_addr);
    TU_LOG2_MEM(event->request, sizeof(tusb_control_request_t), 2);

    _ctrl_xfer.request     = *((tusb_control_request_t*)event->request);
    _ctrl_xfer.buffer      = event->buffer;
    _ctrl_xfer.stage       = STAGE_SETUP;
    _ctrl_xfer.bypass      = false;
    _ctrl_xfer.complete_cb = event->complete_cb;
  } else {
    if (event->request != NULL) {
      free(event->request);
      event->request = NULL;
    }
  }

}
bool tuh_control_xfer_bypass (uint8_t dev_addr, tusb_control_request_t const* request, void* buffer, tuh_control_complete_cb_t complete_cb) {
  const uint8_t rhport = usbh_get_rhport(dev_addr);
  TU_LOG2("HCD_EVENT_CTRL_BYPASS_COMMAND\r\n");
  TU_LOG2("Control Setup (addr = %u)\n", dev_addr);

  _ctrl_xfer.request     = *request;
  _ctrl_xfer.buffer      = buffer;
  _ctrl_xfer.stage       = STAGE_SETUP;
  _ctrl_xfer.bypass      = true;
  _ctrl_xfer.complete_cb = complete_cb;

  // Send setup packet
  TU_ASSERT( hcd_setup_send(rhport, dev_addr, (uint8_t const*) request));

  TU_LOG2("HCD_EVENT_CTRL_BYPASS_COMMAND done\r\n");
}
bool tuh_control_xfer (uint8_t dev_addr, tusb_control_request_t const* request, void* buffer, tuh_control_complete_cb_t complete_cb)
{
  // TODO need to claim the endpoint first
  const uint8_t rhport = usbh_get_rhport(dev_addr);
  tusb_control_request_t *new_request = (tusb_control_request_t*)malloc(sizeof(tusb_control_request_t));
  memcpy(new_request, request, sizeof(tusb_control_request_t));

  hcd_event_t event =
  {
    .rhport = rhport,
    .dev_addr  = dev_addr,
    .id = getCmdId(),
    .event_id = HCD_EVENT_CTRL_COMMAND,
    .request = (void *)new_request,
    .complete_cb = (void *)complete_cb,
    .buffer = buffer,
    .event_cb = tuh_control_event_cb,
  };
  TU_LOG2("Add handler HCD_EVENT_CTRL_COMMAND 0x%x req 0x%x\n", event.id, request->bRequest);
  hcd_event_handler(&event, false);
  return true;
}

static void _xfer_complete(uint8_t dev_addr, xfer_result_t result)
{
  TU_LOG2("\r\n");
  if (_ctrl_xfer.complete_cb) _ctrl_xfer.complete_cb(dev_addr, &_ctrl_xfer.request, result);
}

static bool tuh_control_clear_feature(uint8_t dev_addr, uint8_t ep_addr, uint8_t feature) {
  TU_LOG3("tuh_control_clear_feature %x %x %x\n", dev_addr, ep_addr, feature);
  tusb_control_request_t const request =
  {
    .bmRequestType_bit =
    {
      .recipient = TUSB_REQ_RCPT_ENDPOINT,
      .type      = TUSB_REQ_TYPE_STANDARD,
      .direction = TUSB_DIR_OUT
    },
    .bRequest = HOST_REQUEST_CLEAR_FEATURE,
    .wValue   = feature,
    .wIndex   = ep_addr, //genre 129
    .wLength  = 0
  };

  TU_ASSERT( tuh_control_xfer(dev_addr, &request, NULL, NULL) );
  return true;
}

bool usbh_control_xfer_cb (uint8_t dev_addr, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
  (void) ep_addr;
  (void) xferred_bytes;

  const uint8_t rhport = usbh_get_rhport(dev_addr);

  tusb_control_request_t const * request = &_ctrl_xfer.request;

  if (XFER_RESULT_SUCCESS != result)
  {
    TU_LOG2("Control failed: result = %d\r\n", result);
    if (!_ctrl_xfer.bypass) usbh_can_accept_cmd();
    _xfer_complete(dev_addr, result);
    // terminate transfer if any stage failed
    return true;
  }

  switch(_ctrl_xfer.stage)
  {
    case STAGE_SETUP:
      _ctrl_xfer.stage = STAGE_DATA;
      if (request->wLength)
      {
        // DATA stage: initial data toggle is always 1
        hcd_edpt_xfer(rhport, dev_addr, tu_edpt_addr(0, request->bmRequestType_bit.direction), _ctrl_xfer.buffer, request->wLength);
        return false;
      }
      __attribute__((fallthrough));

    case STAGE_DATA:
      _ctrl_xfer.stage = STAGE_ACK;

      if (request->wLength)
      {
        TU_LOG2("Control data (addr = %u):\r\n", dev_addr);
        TU_LOG2_MEM(_ctrl_xfer.buffer, request->wLength, 2);
      }

      // ACK stage: toggle is always 1
      hcd_edpt_xfer(rhport, dev_addr, tu_edpt_addr(0, 1-request->bmRequestType_bit.direction), NULL, 0);
      return false;
    break;

    case STAGE_ACK:
      if (!_ctrl_xfer.bypass) usbh_can_accept_cmd();
      _xfer_complete(dev_addr, result);
      return true;
    break;

    default: return false;
  }

  return false;
}

#endif
