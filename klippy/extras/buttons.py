# Support for button detection and callbacks
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

QUERY_TIME = .002
RETRANSMIT_COUNT = 50
ADC_REPORT_TIME = 0.015
ADC_DEBOUNCE_TIME = 0.025
ADC_SAMPLE_TIME = 0.001
ADC_SAMPLE_COUNT = 6

# Rotary encoder handler https://github.com/brianlow/Rotary
# Copyright 2011 Ben Buxton (bb@cactii.net).
# Licenced under the GNU GPL Version 3.
R_START     = 0x0
R_CW_FINAL  = 0x1
R_CW_BEGIN  = 0x2
R_CW_NEXT   = 0x3
R_CCW_BEGIN = 0x4
R_CCW_FINAL = 0x5
R_CCW_NEXT  = 0x6
R_DIR_CW    = 0x10
R_DIR_CCW   = 0x20
R_DIR_MSK   = 0x30
# Use the full-step state table (emits a code at 00 only)
ENCODER_STATES = (
  # R_START
  (R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START),
  # R_CW_FINAL
  (R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | R_DIR_CW),
  # R_CW_BEGIN
  (R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START),
  # R_CW_NEXT
  (R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START),
  # R_CCW_BEGIN
  (R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START),
  # R_CCW_FINAL
  (R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | R_DIR_CCW),
  # R_CCW_NEXT
  (R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START)
)

######################################################################
# Button state tracking
######################################################################

class MCU_buttons:
    def __init__(self, printer, mcu):
        self.reactor = printer.get_reactor()
        self.mcu = mcu
        self.mcu.register_config_callback(self.build_config)
        self.pin_list = []
        self.callbacks = []
        self.invert = self.last_button = 0
        self.ack_cmd = None
        self.ack_count = 0
    def setup_buttons(self, pins, callback):
        mask = 0
        shift = len(self.pin_list)
        for pin_params in pins:
            if pin_params['invert']:
                self.invert |= 1 << len(self.pin_list)
            mask |= 1 << len(self.pin_list)
            self.pin_list.append((pin_params['pin'], pin_params['pullup']))
        self.callbacks.append((mask, shift, callback))
    def build_config(self):
        if not self.pin_list:
            return
        self.oid = self.mcu.create_oid()
        self.mcu.add_config_cmd("config_buttons oid=%d button_count=%d" % (
            self.oid, len(self.pin_list)))
        for i, (pin, pull_up) in enumerate(self.pin_list):
            self.mcu.add_config_cmd(
                "buttons_add oid=%d pos=%d pin=%s pull_up=%d" % (
                    self.oid, i, pin, pull_up), is_init=True)
        cmd_queue = self.mcu.alloc_command_queue()
        self.ack_cmd = self.mcu.lookup_command(
            "buttons_ack oid=%c count=%c", cq=cmd_queue)
        clock = self.mcu.get_query_slot(self.oid)
        rest_ticks = self.mcu.seconds_to_clock(QUERY_TIME)
        self.mcu.add_config_cmd(
            "buttons_query oid=%d clock=%d"
            " rest_ticks=%d retransmit_count=%d" % (
                self.oid, clock, rest_ticks, RETRANSMIT_COUNT), is_init=True)
        self.mcu.register_msg(
            self.handle_buttons_state, "buttons_state", self.oid)
    def handle_buttons_state(self, params):
        # Expand the message ack_count from 8-bit
        ack_count = self.ack_count
        ack_diff = (ack_count - params['ack_count']) & 0xff
        if ack_diff & 0x80:
            ack_diff -= 0x100
        msg_ack_count = ack_count - ack_diff
        # Determine new buttons
        buttons = params['state']
        new_count = msg_ack_count + len(buttons) - self.ack_count
        if new_count <= 0:
            return
        new_buttons = buttons[-new_count:]
        # Send ack to MCU
        self.ack_cmd.send([self.oid, new_count])
        self.ack_count += new_count
        # Call self.handle_button() with this event in main thread
        for b in new_buttons:
            self.reactor.register_async_callback(
                (lambda e, s=self, b=ord(b): s.handle_button(e, b)))
    def handle_button(self, eventtime, button):
        button ^= self.invert
        changed = button ^ self.last_button
        for mask, shift, callback in self.callbacks:
            if changed & mask:
                callback(eventtime, (button & mask) >> shift)
        self.last_button = button


class MCU_ADC_buttons:
    def __init__(self, printer, pin, pullup, debug=False):
        self.reactor = printer.get_reactor()
        self.buttons = []
        self.last_button = None
        self.last_pressed = None
        self.last_debouncetime = 0
        self.pullup = pullup
        self.debug = debug
        self.pin = pin
        self.min_value = self.max_value = None
        ppins = printer.lookup_object('pins')
        self.mcu_adc = ppins.setup_pin('adc', self.pin)
        self.mcu_adc.setup_minmax(ADC_SAMPLE_TIME, ADC_SAMPLE_COUNT)
        self.mcu_adc.setup_adc_callback(ADC_REPORT_TIME, self.adc_callback)

    def setup_button(self, min_value, max_value, callback):
        if self.min_value is None:
            self.min_value = min_value
        else:
            self.min_value = min(self.min_value, min_value)

        if self.max_value is None:
            self.max_value = max_value
        else:
            self.max_value = max(self.max_value, max_value)

        self.buttons.append((min_value, max_value, callback))

    def adc_callback(self, read_time, read_value):
        adc = max(.00001, min(.99999, read_value))
        r = self.pullup * adc / (1.0 - adc)
        self.reactor.register_async_callback(
            (lambda e, s=self, v=r: s.handle_button(e, v)))

    def get_button(self, value):
        if (self.min_value is not None and self.max_value is not None
                and self.min_value <= value <= self.max_value):
            for i, (min_value, max_value, cb) in enumerate(self.buttons):
                if min_value < value < max_value:
                    return i
        return None

    def handle_button(self, eventtime, value):
        btn = self.get_button(int(value))

        # If the button changed, due to noise or pressing:
        if btn != self.last_button:
            # reset the debouncing timer
            self.last_debouncetime = eventtime

        # button debounce check & new button pressed
        if ((eventtime - self.last_debouncetime) >= ADC_DEBOUNCE_TIME
                and self.last_button == btn and self.last_pressed != btn):
                # release last_pressed
                if self.last_pressed is not None:
                    self.call_button(eventtime, self.last_pressed, False)
                    self.last_pressed = None
                if btn is not None:
                    self.call_button(eventtime, btn, True)
                    self.last_pressed = btn

        self.last_button = btn
        if self.debug is True:
            logging.info(
                "analog pin: %s value: %d" % (self.pin, int(value)))

    def call_button(self, eventtime, button, state):
        if button < len(self.buttons):
            minval, maxval, callback = self.buttons[button]
            callback(eventtime, state)


######################################################################
# Rotary Encoders
######################################################################

class RotaryEncoder:
    def __init__(self, cw_callback, ccw_callback):
        self.cw_callback = cw_callback
        self.ccw_callback = ccw_callback
        self.encoder_state = R_START
    def encoder_callback(self, eventtime, state):
        es = ENCODER_STATES[self.encoder_state & 0xf][state & 0x3]
        self.encoder_state = es
        if es & R_DIR_MSK == R_DIR_CW:
            self.cw_callback(eventtime)
        elif es & R_DIR_MSK == R_DIR_CCW:
            self.ccw_callback(eventtime)


######################################################################
# Button registration code
######################################################################

class PrinterButtons:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.mcu_buttons = {}
        self.adc_buttons = {}
    def register_adc_button(
            self, pin, min_val, max_val, pullup, callback, debug=False):
        adc_buttons = self.adc_buttons.get(pin)
        if adc_buttons is None:
            self.adc_buttons[pin] = adc_buttons = MCU_ADC_buttons(
                self.printer, pin, pullup, debug)
        adc_buttons.setup_button(min_val, max_val, callback)
    def register_adc_button_push(
            self, pin, min_val, max_val, pullup, callback, debug=False):
        def helper(eventtime, state, callback=callback):
            if state:
                callback(eventtime)
        self.register_adc_button(pin, min_val, max_val, pullup, helper, debug)
    def register_buttons(self, pins, callback):
        # Parse pins
        ppins = self.printer.lookup_object('pins')
        mcu = mcu_name = None
        pin_params_list = []
        for pin in pins:
            pin_params = ppins.lookup_pin(pin, can_invert=True, can_pullup=True)
            if mcu is not None and pin_params['chip'] != mcu:
                raise ppins.error("button pins must be on same mcu")
            mcu = pin_params['chip']
            mcu_name = pin_params['chip_name']
            pin_params_list.append(pin_params)
        # Register pins and callback with the appropriate MCU
        mcu_buttons = self.mcu_buttons.get(mcu_name)
        if (mcu_buttons is None
            or len(mcu_buttons.pin_list) + len(pin_params_list) > 8):
            self.mcu_buttons[mcu_name] = mcu_buttons = MCU_buttons(
                self.printer, mcu)
        mcu_buttons.setup_buttons(pin_params_list, callback)
    def register_rotary_encoder(self, pin1, pin2, cw_callback, ccw_callback):
        re = RotaryEncoder(cw_callback, ccw_callback)
        self.register_buttons([pin1, pin2], re.encoder_callback)
    def register_button_push(self, pin, callback):
        def helper(eventtime, state, callback=callback):
            if state:
                callback(eventtime)
        self.register_buttons([pin], helper)

def load_config(config):
    return PrinterButtons(config)
