# Mechanical gantry tilting, for crooked crow
#
# Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
# Modified from z_tilt by Rob Mink - @printcepts

import math
from . import z_tilt


class quad_gantry_tilt:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.frontBH = config.getfloat('front_block_height',0.0)
        self.rearBH = config.getfloat('rear_block_height',0.0)
        self.max_angle = config.getfloat('maximum_gantry_angle', 45.0)
        self.min_angle = config.getfloat('minimum_gantry_angle',0.0)
        self.bearing_space = config.getfloat('bearing_y_distance',100.0)
        self.gangle = float(math.radians(config.getfloat('initial_angle', 0.0)))
        self.tilt_speed = config.getfloat('tilt_speed',20.0)
        self.z_steppers = []
        self.current_offset = float(0.0 - (self.rearBH + self.frontBH))
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.z_helper = z_tilt.ZAdjustHelper(config, 4)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SET_GANTRY_TILT', self.cmd_SET_GANTRY_TILT, desc=self.cmd_SET_GANTRY_TILT_help)
        gcode.register_command('GET_ANGLE', self.cmd_GET_GANTRY_ANGLE, desc=self.cmd_GET_ANGLE_help)

    cmd_SET_GANTRY_TILT_help = "Adjust the angle of the gantry"
    cmd_GET_ANGLE_help = "Returns the current gantry angle"
    
    def handle_connect(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        z_steppers = [s for s in kin.get_steppers() if s.is_active_axis('z')]
        if len(z_steppers) != 4:
            raise self.printer.config_error(
                "%s requires 4 z steppers" % (self.name,))
        self.z_steppers = z_steppers
        self.gangle = 0.0
        self.current_offset = float(0.0 - (self.rearBH + self.frontBH))

    def cmd_GET_GANTRY_ANGLE(self, gcmd):
        degrees = math.degrees(self.gangle)
        gcmd.respond_info("Current Gantry Angle: %.2f degrees (%.4f radians)" % (degrees, self.gangle))
        return float(self.gangle)

    def cmd_SET_GANTRY_TILT(self, gcmd):
        angle = float(gcmd.get_float('ANGLE'))
        if self.min_angle <= angle <= self.max_angle:
            self.gangle = float(math.radians(angle))
        else:
            return
        s = float(math.sin(self.gangle))
        c = float(math.cos(self.gangle))
        t = float(math.tan(self.gangle))
        axle_offset = float((s * self.frontBH) + (s * self.rearBH))
        adjusted_axle = float(self.bearing_space - axle_offset)
        final_mm_offset = float((t * adjusted_axle) - ((c * self.frontBH) + (c * self.rearBH)))
        how_far = float(final_mm_offset - self.current_offset)
        self.current_offset = float(final_mm_offset)
        
        z_adjust = [0.0, float(how_far), float(how_far), 0.0]
        speed = float(self.tilt_speed)
        self.z_helper.adjust_steppers(z_adjust, speed)

    def adjust_steppers(self, adjustments, speed):
        toolhead = self.printer.lookup_object('toolhead')
        gcode = self.printer.lookup_object('gcode')
        curpos = list(toolhead.get_position())
        stepstrs = ["%s = %.6f" % (s.get_name(), float(a))
                    for s, a in zip(self.z_steppers, adjustments)]
        msg = "Making the following Z adjustments:\n%s" % ("\n".join(stepstrs),)
        gcode.respond_info(msg)
        toolhead.flush_step_generation()
        for s in self.z_steppers:
            s.set_trapq(None)
        positions = [(-float(a), s) for a, s in zip(adjustments, self.z_steppers)]
        z_move = float(curpos[2])
        for i in range(len(positions)):
            stepper_offset, stepper = positions[i]
            toolhead.flush_step_generation()
            stepper.set_trapq(toolhead.get_trapq())
            curpos[2] = float(z_move + stepper_offset)
            toolhead.move(curpos, float(speed))
            toolhead.set_position(curpos)

def load_config(config):
    return quad_gantry_tilt(config)