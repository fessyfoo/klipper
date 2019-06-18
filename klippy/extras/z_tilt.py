# Mechanical bed tilt calibration with multiple Z steppers
#
# Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import probe, mathutil

class ZAdjustHelper:
    def __init__(self, config, z_count):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.z_count = z_count
        self.z_steppers = []
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
    def handle_connect(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        z_steppers = kin.get_steppers('Z')
        if len(z_steppers) != self.z_count:
            raise self.printer.config_error(
                "%s z_positions needs exactly %d items" % (
                    self.name, len(z_steppers)))
        if len(z_steppers) < 2:
            raise config.error("%s requires multiple z steppers" % (self.name,))
        self.z_steppers = z_steppers
    def adjust_steppers(self, adjustments, speed):
        toolhead = self.printer.lookup_object('toolhead')
        gcode = self.printer.lookup_object('gcode')
        curpos = toolhead.get_position()
        # Report on movements
        stepstrs = ["%s = %.6f" % (s.get_name(), a)
                    for s, a in zip(self.z_steppers, adjustments)]
        msg = "Making the following Z adjustments:\n%s" % ("\n".join(stepstrs),)
        gcode.respond_info(msg)
        # Disable Z stepper movements
        for s in self.z_steppers:
            s.set_ignore_move(True)
        # Move each z stepper (sorted from lowest to highest) until they match
        positions = [(-a, s) for a, s in zip(adjustments, self.z_steppers)]
        positions.sort()
        first_stepper_offset, first_stepper = positions[0]
        z_low = curpos[2] - first_stepper_offset
        for i in range(len(positions)-1):
            stepper_offset, stepper = positions[i]
            next_stepper_offset, next_stepper = positions[i+1]
            stepper.set_ignore_move(False)
            curpos[2] = z_low + next_stepper_offset
            try:
                toolhead.move(curpos, speed)
                toolhead.set_position(curpos)
            except:
                logging.exception("ZAdjustHelper adjust_steppers")
                for s in self.z_steppers:
                    s.set_ignore_move(False)
                raise
        # Z should now be level - do final cleanup
        last_stepper_offset, last_stepper = positions[-1]
        last_stepper.set_ignore_move(False)
        curpos[2] += first_stepper_offset
        toolhead.set_position(curpos)
        gcode.reset_last_position()

class ZTilt:
    def __init__(self, config):
        self.printer = config.get_printer()
        z_positions = config.get('z_positions').split('\n')
        try:
            z_positions = [line.split(',', 1)
                           for line in z_positions if line.strip()]
            self.z_positions = [(float(zp[0].strip()), float(zp[1].strip()))
                                for zp in z_positions]
        except:
            raise config.error("Unable to parse z_positions in %s" % (
                config.get_name()))

        self.retry_helper = RetryHelper(
            config = config,
            value_label = "Probed points range")

        self.probe_helper = probe.ProbePointsHelper(config, self.probe_finalize)
        self.probe_helper.minimum_points(2)
        self.z_helper = ZAdjustHelper(config, len(self.z_positions))
        # Register Z_TILT_ADJUST command
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('Z_TILT_ADJUST', self.cmd_Z_TILT_ADJUST,
                                    desc=self.cmd_Z_TILT_ADJUST_help)
    cmd_Z_TILT_ADJUST_help = "Adjust the Z tilt"
    def cmd_Z_TILT_ADJUST(self, params):
        self.retries = self.retry_helper.retry(params)
        self.probe_helper.start_probe(params)
    def probe_finalize(self, offsets, positions):
        # Setup for coordinate descent analysis
        z_offset = offsets[2]
        logging.info("Calculating bed tilt with: %s", positions)
        params = { 'x_adjust': 0., 'y_adjust': 0., 'z_adjust': z_offset }
        # Perform coordinate descent
        def adjusted_height(pos, params):
            x, y, z = pos
            return (z - x*params['x_adjust'] - y*params['y_adjust']
                    - params['z_adjust'])
        def errorfunc(params):
            total_error = 0.
            for pos in positions:
                total_error += adjusted_height(pos, params)**2
            return total_error
        new_params = mathutil.coordinate_descent(
            params.keys(), params, errorfunc)
        # Apply results
        speed = self.probe_helper.get_lift_speed()
        logging.info("Calculated bed tilt parameters: %s", new_params)
        x_adjust = new_params['x_adjust']
        y_adjust = new_params['y_adjust']
        z_adjust = (new_params['z_adjust'] - z_offset
                    - x_adjust * offsets[0] - y_adjust * offsets[1])

        adjustments = [x*x_adjust + y*y_adjust + z_adjust
                       for x, y in self.z_positions]
        self.z_helper.adjust_steppers(adjustments, speed)

        z_positions = [ p[2] for p in positions]
        return self.retries.check(max(z_positions) - min(z_positions))

######################################################################
# Retry Helper Class
######################################################################

# this along with the following RetryState class is originally designed to
# share retry logic between QuadGantryLevel cmd_QUAD_GANTRY_LEVEL and ZTilt
# cmd_Z_TILT_ADJUST

# RetryHelper __init__() gathers default options retries and retry_tolerance
# from the passed in config object
#
# RetryHelper retry() parses the retries and retry_tolerance params for the
# calling gcode command and then instantiates, configures and returns the
# RetryState object to keep state involved with retrying and watching for
# errors.

class RetryHelper:

    def __init__(self, config, error_msg_extra = "", value_label = "value"):

        self.value_label     = value_label
        self.error_msg_extra = error_msg_extra

        self.default_retries         = config.getint("retries", 0)
        self.default_retry_tolerance = config.getfloat("retry_tolerance", 0)

        self.gcode = config.get_printer().lookup_object('gcode')

    def retry(self, params):
        retries = self.gcode.get_int(
            'RETRIES',
            params,
            default=self.default_retries,
            minval=0,
            maxval=30)

        tolerance = self.gcode.get_float(
            'RETRY_TOLERANCE',
            params,
            default=self.default_retry_tolerance,
            minval=0,
            maxval=1.0)

        return RetryState(
            retries,
            tolerance,
            self.gcode,
            self.error_msg_extra,
            self.value_label)

######################################################################
# RetryState Class
######################################################################

# Instantiated by RetryHeper retry() method. Keeps state for retries

# provides a .check() function that must be called with a value that is the
# result of the current run. If this value is larger than the tolerance and we
# are below the configured number of retries then return "retry" to trigger the
# probe support for retry.

# It further watches for the value  getting worse consecutively and aborts

# Last it issues an error messgage if retries were configured but they don't
# converge less than retry_tolerance in the specified number of retries
class RetryState(object):

    def __init__(self, retries, tolerance, gcode, error_msg_extra, value_label):

        self.gcode             = gcode
        self.retry_tolerance   = tolerance
        self.retries_remaining = retries
        self.retries_total     = retries
        self.enabled           = retries > 0
        self.cnt               = 0
        self.previous          = None
        self.errors            = 0
        self.history           = []
        self.error_msg_extra   = error_msg_extra
        self.value_label       = value_label

    @property
    def errors(self):
        return self.__errors
    @errors.setter
    def errors(self,errors):
        self.__errors = 0 if errors < 0 else errors

    def check_for_error(self,value):
        if self.previous and value > self.previous + 0.0000001:
            self.errors += 1
        else:
            self.errors -= 1

        self.history.append(value)
        self.previous = value
        return self.errors > 1

    def good(self,value):
        return value <= self.retry_tolerance

    def check(self,value):
        if not self.enabled:
            return

        if self.check_for_error(value):
            history = [ round(v,6) for v in self.history ]
            self.gcode.respond_error(
                "Retries aborting: %s is increasing. " % (self.value_label) +
                "%s %s" % (history, self.error_msg_extra))
            return

        self.gcode.respond_info(
            "Retries: %d/%d " % (self.cnt, self.retries_total) +
            "%s: " % (self.value_label) +
            "%0.6f tolerance: %0.6f" % (value, self.retry_tolerance))
        self.cnt += 1

        if self.good(value):
            return

        if self.retries_remaining > 0:
            self.retries_remaining -= 1
            return "retry"
        else:
            self.gcode.respond_error(
                "Retries: %s failed to converge to tolerance" %
                (self.value_label))

def load_config(config):
    return ZTilt(config)
