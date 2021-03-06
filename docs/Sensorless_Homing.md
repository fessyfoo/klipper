# Sensorless Homing
Sensorless homing allows to home an axis without the need for a physical limit switch. Instead, the carriage on the axis is moved into the mechanical limit making the stepper motor lose steps. The stepper driver senses the lost steps and indicates this to the controlling MCU (Klipper) by toggling a pin. This information can be used by Klipper as end stop for the axis.

This guide covers the setup of sensorless homing for the X axis of your (cartesian) printer. However, it works the same with all other axes (that require an end stop). You should configure and tune it for one axis at a time.

## Prerequisites
A few prerequisites are needed to use sensorless homing:

1. TMC2130 stepper driver
2. SPI interface of the TMC2130 wired to MCU (stand-alone mode does not work)
3. DIAG1 pin of TMC2130 connected to the MCU


## Limitations
Be sure that your mechanical components are able to handle the load of the carriage bumping into the limit of the axis repeatedly. Especially spindles (on the Z axis) might generate a lot of force. Homing a Z axis by bumping the nozzle into the printing surface might not be a good idea.

Further, sensorless homing might not be accurate enough for you printer. While homing X and Y axes on a cartesian machine can work well, homing the Z axis is generally not accurate enough and results in inconsistent first layer height. Homing a delta printer sensorless is not advisable due to missing accuracy.

Further, the stall detection of the stepper driver is dependant on the mechanical load on the motor, the motor current and the motor temperature (coil resistance).

Sensorless homing works best at medium motor speeds. For very slow speeds (less than 10 RPM) the motor does not generate significant back EMF and the TMC2130 cannot reliably detect motor stalls. Further, at very high speeds, the back EMF of the motor approaches the supply voltage of the motor, so the TMC2130 cannot detect stalls anymore. For more details on limitations refer to section 14 (stallGuard2
Load Measurement) in the TMC2130 datasheet.

## Configuration
To enable sensorless homing add a section to configure the TMC2130 stepper driver to your `printer.cfg`:

```
[tmc2130 stepper_x]
cs_pin:        # chip select pin of the SPI interface
microsteps:    # number of microsteps per full step of the motor
run_current:   # value in amps
diag1_pin: !   # pin on the MCU where DIAG1 is connected (active low)
driver_SGT: 0  # tuning value for sensorless homing, set to 0 as a start
```

The above snippet configures a TMC2130 for the stepper on the X axis. Make sure to fill in the missing values based on your configuration.

If you have a CoreXY machine, you can configure one stepper driver for X and the other for Y homing as you would on a cartesian printer. Be aware that Klipper needs both `DIAG1` pins connected to the MCU. It is not sufficient to use only one signal from one of the stepper drivers (as it is possible on e.g. Marlin).

The `diag1_pin` of the TMC2130 is configured as open-collector pin. This means, the stepper driver pulls the pin low to indicate a stalled motor (active low) and the pin must be inverted by adding a `!` in front of the pin name. Further, you need a pull-up resistor on the connection. If your PCB has no external pull-up, you can enable the internal pull-up of your MCU by adding a `^` in front of the pin name. The resulting line might look like this:

```
diag1_pin: ^!PA1  # DIAG1 connected to PA1, internal pull-up is enabled, signal is active low
```

By configuring the `diag1_pin`, Klipper allows you to use a special virtual end stop for the axis. You can use this instead of a physical end stop pin by changing the `endstop_pin` of the corresponding axis:

```
[stepper_x]
...
endstop_pin: tmc2130_stepper_x:virtual_endstop  # use the virtual end stop generated by the [tmc2130 stepper_x] section of this config file
...
homing_retract_dist: 0
...
```

The name of the virtual end stop pin is derived from the name of the TMC2130 section. The `homing_retract_dist` setting should be set to zero to disable the second homing move as a second pass is not needed, and attempts to do so are error prone.

ATTENTION: This guide only mentions the mandatory parameters and the ones needed to set up sensorless homing. There are many other options to configure on a TMC2130, make sure to take a look at `config/example-extras.cfg` for all the available options.

## Testing of SPI communication
Now that the stepper driver is configured, let's make sure that Klipper can communicate with the TMC2130 by sending the following extended G-Code command to the printer:

```
DUMP_TMC stepper=stepper_x
```

This command tells Klipper to read a few registers via SPI from the TMC2130. If everything works correctly, the output should look similar to this (in OctoPrint terminal tab):

```
Send: DUMP_TMC stepper=stepper_x
Recv: // GCONF:          00000004
Recv: // GSTAT:          00000001
Recv: // IOIN:           11000078
Recv: // TSTEP:          000fffff
Recv: // XDIRECT:        00000000
Recv: // MSCNT:          00000010
Recv: // MSCURACT:       00f60018
Recv: // CHOPCONF:       15008384
Recv: // DRV_STATUS:     800d0000
Recv: // PWM_SCALE:      00000000
Recv: // LOST_STEPS:     00000000
```

The actual register values might differ based the configuration of your TMC2130. If the register values are all `ffffffff` or look otherwise bogus (for example, `LOST_STEPS` should be always `00000000` here) make sure that the SPI is wired and configured correctly.

## Homing and Tuning

Let's try the first sensorless homing now. It will likely not work as intended. There are three possible outcomes of this experiment:

1. The axis stops moving before hitting the mechanical limit or does not move at all
2. The axis homes correctly (which is unlikely at this point)
3. The axis bumps into the mechanical limit and keeps moving while making horrible noise

If the third outcome happens to you, disable the stepper (by cutting the power or issuing a `M112` emergency stop).

Ok, now that you know what can happen, let's try it out. Put the carriage somewhere in the middle of the X axis. Home the X axis by sending the following G-Code command to Klipper and observe the outcome:

```
G28 X
```

If the axis stopped early (first outcome), the stepper driver detected a motor stall even though there was none. To trigger stall detection at a higher load, increase the value of `driver_SGT` (for example from 0 to 5). The values can be any interger between `-64` and `63`. The higher the value, the later it triggers stall detection.

If your axis did not stop (third outcome), the stepper driver was not able to detect the stall, because the load on the motor still seemed reasonable to the driver. To trigger stall detection at a lighter load, decrease the value of `driver_SGT`.

Even if your axis homed correctly, it might be worth to try a few different values for `driver_SGT`. If you think that it bumps too hard into the mechanical limit, try to decrease the value by 1 or 2.

At this point, your axis should be able to home based on the stall detection of the TMC2130. Congratulations! You can now proceed with the next axis of your printer.
