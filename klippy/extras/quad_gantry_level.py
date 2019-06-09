# Mechanicaly conforms a moving gantry to the bed with 4 Z steppers
#
# Copyright (C) 2018  Maks Zolin <mzolin@vorondesign.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import probe


class QuadGantryLevel:
    def __init__(self, config):
        self.printer = config.get_printer()

        self.retry_helper = RetryHelper(
            config = config,
            value_label = "Probed points range",
            error_msg_extra = "Possibly Z motor numbering is wrong")

        self.max_adjust = config.getfloat("max_adjust", 4, above=0)
        self.horizontal_move_z = config.getfloat("horizontal_move_z", 5.0)
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.probe_helper = probe.ProbePointsHelper(config, self.probe_finalize)
        if len(self.probe_helper.probe_points) != 4:
            raise config.error(
                "Need exactly 4 probe points for quad_gantry_level")
        gantry_corners = config.get('gantry_corners').split('\n')
        try:
            gantry_corners = [line.split(',', 1)
                           for line in gantry_corners if line.strip()]
            self.gantry_corners = [(float(zp[0].strip()), float(zp[1].strip()))
                                for zp in gantry_corners]
        except:
            raise config.error("Unable to parse gantry_corners in %s" % (
                config.get_name()))
        if len(self.gantry_corners) < 2:
            raise config.error(
                "quad_gantry_level requires at least two gantry_corners")
        self.z_steppers = []
        # Register QUAD_GANTRY_LEVEL command
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'QUAD_GANTRY_LEVEL', self.cmd_QUAD_GANTRY_LEVEL,
            desc=self.cmd_QUAD_GANTRY_LEVEL_help)
    def handle_connect(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        z_steppers = kin.get_steppers('Z')
        if len(z_steppers) != 4:
            raise self.printer.config_error(
                "quad_gantry_level needs exactly 4 z steppers")
        self.z_steppers = z_steppers
    cmd_QUAD_GANTRY_LEVEL_help = (
        "Conform a moving, twistable gantry to the shape of a stationary bed")
    def cmd_QUAD_GANTRY_LEVEL(self, params):
        self.retries = self.retry_helper.retry(
            params,
            lambda: self.probe_helper.start_probe(params))
        self.retries.start()

    def probe_finalize(self, offsets, positions):
        # Mirror our perspective so the adjustments make sense
        # from the perspective of the gantry
        z_positions = [self.horizontal_move_z - p[2] for p in positions]
        points_message = "Gantry-relative probe points:\n%s\n" % (
            " ".join(["%s: %.6f" % (z_id, z_positions[z_id])
                for z_id in range(len(z_positions))]))
        self.gcode.respond_info(points_message)
        p1 = [positions[0][0] + offsets[0],z_positions[0]]
        p2 = [positions[1][0] + offsets[0],z_positions[1]]
        p3 = [positions[2][0] + offsets[0],z_positions[2]]
        p4 = [positions[3][0] + offsets[0],z_positions[3]]
        f1 = self.linefit(p1,p4)
        f2 = self.linefit(p2,p3)
        logging.info("quad_gantry_level f1: %s, f2: %s" % (f1,f2))
        a1 = [positions[0][1] + offsets[1],
              self.plot(f1,self.gantry_corners[0][0])]
        a2 = [positions[1][1] + offsets[1],
              self.plot(f2,self.gantry_corners[0][0])]
        b1 = [positions[0][1] + offsets[1],
              self.plot(f1,self.gantry_corners[1][0])]
        b2 = [positions[1][1] + offsets[1],
              self.plot(f2,self.gantry_corners[1][0])]
        af = self.linefit(a1,a2)
        bf = self.linefit(b1,b2)
        logging.info("quad_gantry_level af: %s, bf: %s" % (af,bf))
        z_height = [0,0,0,0]
        z_height[0] = self.plot(af,self.gantry_corners[0][1])
        z_height[1] = self.plot(af,self.gantry_corners[1][1])
        z_height[2] = self.plot(bf,self.gantry_corners[1][1])
        z_height[3] = self.plot(bf,self.gantry_corners[0][1])

        ainfo = zip(["z","z1","z2","z3"], z_height[0:4])
        apos = " ".join(["%s: %06f" % (x) for x in ainfo])
        self.gcode.respond_info("Actuator Positions:\n" + apos)

        z_ave = sum(z_height) / len(z_height)
        self.gcode.respond_info("Average: %0.6f" % z_ave)
        z_adjust = []
        for z in z_height:
            z_adjust.append(z_ave - z)

        adjust_max = max(z_adjust)
        if adjust_max > self.max_adjust:
            self.gcode.respond_error(
                "Aborting quad_gantry_level " +
                "required adjustment %0.6f " % ( adjust_max ) +
                "is greater than max_adjust %0.6f" % (self.max_adjust))
            return

        try:
            self.adjust_steppers(z_adjust)
        except:
            logging.exception("quad_gantry_level adjust_steppers")
            for s in self.z_steppers:
                s.set_ignore_move(False)
            raise

        return self.retries.check(max(z_positions) - min(z_positions))


    def linefit(self,p1,p2):
        if p1[1] == p2[1]:
            # Straight line
            return 0,p1[1]
        m = (p2[1] - p1[1])/(p2[0] - p1[0])
        b = p1[1] - m * p1[0]
        return m,b
    def plot(self,f,x):
        return f[0]*x + f[1]
    def adjust_steppers(self, z_adjust):
        msg = "Making the following gantry adjustments:\n%s\n" % (
            "\n".join(["%s = %.6f" % (
                self.z_steppers[z_id].get_name(), z_adjust[z_id]
                ) for z_id in range(4)]))
        self.gcode.respond_info(msg)
        toolhead = self.printer.lookup_object('toolhead')
        cur_pos = toolhead.get_position()
        speed = self.probe_helper.get_lift_speed()
        # Disable moves on all Z steppers
        for s in self.z_steppers:
            s.set_ignore_move(True)
        for z_id in range(len(z_adjust)):
            stepper = self.z_steppers[z_id]
            stepper.set_ignore_move(False)
            cur_pos[2] = cur_pos[2] + z_adjust[z_id]
            toolhead.move(cur_pos, speed)
            toolhead.set_position(cur_pos)
            stepper.set_ignore_move(True)
        # Re-enable moves on all Z steppers
        for s in self.z_steppers:
            s.set_ignore_move(False)
        self.gcode.reset_last_position()

######################################################################
# Retry Helper Class
######################################################################

# this along with the following RetryRun class is originally designed to share
# retry logic between QuadGantryLevel cmd_QUAD_GANTRY_LEVEL and ZTilt
# cmd_Z_TILT_ADJUST

# RetryHelper __init__() gathers default options retries and retry_tolerance
# from the passed in config object
#
# RetryHelper retry() parses the retries and retry_tolerance params for the
# calling gcode command and then instantiates, configures and returns the
# RetryRun object to keep state involved with retrying and watching for errors.

class RetryHelper:

    def __init__(self, config, error_msg_extra = "", value_label = "value"):

        self.value_label     = value_label
        self.error_msg_extra = error_msg_extra

        self.default_retries         = config.getint("retries", 0)
        self.default_retry_tolerance = config.getfloat("retry_tolerance", 0)

        self.gcode = config.get_printer().lookup_object('gcode')

    def retry(self, params, retry_function):
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

        retry_run = RetryRun(retry_function, retries, tolerance, self.gcode)

        retry_run.value_label     = self.value_label
        retry_run.error_msg_extra = self.error_msg_extra

        return retry_run

######################################################################
# RetryRun Class
######################################################################

# Instantiated by RetryHeper retry() method. Keeps state for retries of the
# supplied retry_function.

# retry_run.check() must be called with a value that is the result of the
# current run.  if this value is larger than the tolerance retry the provided
# retry_function again up to the configured number of retries.

# it further watches for the value  getting worse consecutively and aborts

# last it issues an error messgage if retries were configured but they don't
# converge less than retry_tolerance in the specified number of retries
class RetryRun(object):

    def __init__(self, retry_function, retries, tolerance, gcode):
        self.gcode             = gcode
        self.retry_tolerance   = tolerance
        self.retries_remaining = retries
        self.retries_total     = retries
        self.enabled           = retries > 0
        self.cnt               = 0
        self.retry_function    = retry_function
        self.previous          = None
        self.errors            = 0
        self.history           = []

    def start(self):
        self.retry_function()

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
    return QuadGantryLevel(config)
