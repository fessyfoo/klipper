
class RetryHelper:

    def __init__(self, config):

        self.value_label     = "value"
        self.error_msg_extra = ""

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


class RetryRun:

    def __init__(self, retry_function, retries, tolerance, gcode):
        self.gcode             = gcode
        self.retry_tolerance   = tolerance
        self.retries_remaining = retries
        self.retries_total     = retries
        self.enabled           = retries > 0
        self.cnt               = 0
        self.retry_function    = retry_function
        self.previous          = None

    def retry(self):
        self.retry_function()

    def start(self):
        self.retry()

    def error(self,value):
        if self.previous and value > self.previous + 0.0000001:
            return True

        self.previous = value
        return False

    def good(self,value):
        return value <= self.retry_tolerance

    def check(self,value):
        if not self.enabled:
            return

        if self.error(value):
            self.gcode.respond_error(
                "Error: %s of " % (self.value_label) +
                "%0.6f is worse than previous %0.6f " % (value, self.previous) + 
                self.error_msg_extra)
            return

        self.cnt += 1
        self.gcode.respond_info(
            "Try: %d/%d " % (self.cnt, self.retries_total + 1) +
            "%s: " % (self.value_label) +
            "%0.6f tolerance: %0.6f" % (value, self.retry_tolerance))

        if self.good(value):
            return

        if self.retries_remaining > 0:
            self.retries_remaining -= 1
            self.retry()
        else:
            self.gcode.respond_error(
                "Retries: %s failed to converge to tolerance" %
                (self.value_label))

