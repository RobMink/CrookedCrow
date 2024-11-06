# Mechanical gantry tilting, for crooked crow
#
# Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
# Modified from z_tilt by Rob Mink - @printcepts

import numpy
from . import z_tilt


class quad_gantry_tilt:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.frontBH = config.getfloat('front_block_height',0.0)
        self.rearBH = config.getfloat('rear_block_height',0.0)
        self.max_angle = config.getfloat('maximum_gantry_angle', 45.0)
        self.min_angle = config.getfloat('minimum_gantry_angle',0.0)
        self.bearing_space = config.getfloat('bearing_y_distance',100.0)
        self.gangle = numpy.radians(config.getfloat('initial_angle', 0.0))
        self.tilt_speed = config.getfloat('tilt_speed',20.0)
        self.z_steppers = []
        self.current_offset=(0.0-(self.rearBH + self.frontBH))
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.z_helper = z_tilt.ZAdjustHelper(config, 4)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SET_GANTRY_TILT', self.cmd_SET_GANTRY_TILT, desc=self.cmd_SET_GANTRY_TILT_help)
        
    cmd_SET_GANTRY_TILT_help = "Adjust the angle of the gantry"
    
    def handle_connect(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        z_steppers = [s for s in kin.get_steppers() if s.is_active_axis('z')]
        if len(z_steppers) != 4:
            raise self.printer.config_error(
                "%s requires 4 z steppers" % (self.name,))
        self.z_steppers = z_steppers
        self.gangle=0
        self.current_offset=(0.0-(self.rearBH + self.frontBH))
    def cmd_GET_ANGLE(self):
        return self.gangle
    def cmd_SET_GANTRY_TILT(self, gcmd):
        angle = gcmd.get_float('ANGLE')
        if self.min_angle <= angle and angle <= self.max_angle:
            self.gangle=numpy.radians(angle)
        else:
            #raise self.gcode.error("angle out of range for gantry tilt")
            return
        axle_offset=((numpy.sin(self.gangle)*self.frontBH)+(numpy.sin(self.gangle)*self.rearBH))  ##figure out how much the offset of the axle bearings offset from gantry has - how far are the 608s from the gantry
        adjusted_axle=(self.bearing_space-axle_offset)       ##figure out what distance we are working with now
        final_mm_offset = ((numpy.tan(self.gangle)*adjusted_axle)-((numpy.cos(self.gangle)*self.frontBH) +(numpy.cos(self.gangle)*self.rearBH)))  ##take that distance and figure out height based on triangles (opposet/adjacent) then use more trangle math for interferance due to block offsets
        how_far=final_mm_offset-self.current_offset  ##have we made and ajustment before?  How far to move now?
        self.current_offset=final_mm_offset  #save for next time.
        
        
        z_adjust = []
        
        z_adjust.append(0)
        z_adjust.append(how_far)
        z_adjust.append(how_far)
        z_adjust.append(0)

        
        speed = self.tilt_speed
        self.z_helper.adjust_steppers(z_adjust, speed)
     









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
        toolhead.flush_step_generation()
        for s in self.z_steppers:
            s.set_trapq(None)
        # Move each z stepper (sorted from lowest to highest) until they match
        positions = [(-a, s) for a, s in zip(adjustments, self.z_steppers)]
        #positions.sort(key=(lambda k: k[0]))
        #first_stepper_offset, first_stepper = positions[0]
        #z_low = curpos[2] - first_stepper_offset
        z_move=curpos[2]
        
        for i in range(len(positions)):
            stepper_offset, stepper = positions[i]
            toolhead.flush_step_generation()
            stepper.set_trapq(toolhead.get_trapq())
            curpos[2] = z_move + stepper_offset
            #try:
            toolhead.move(curpos, speed)
            toolhead.set_position(curpos)
            #except:
            #    logging.exception("ZAdjustHelper adjust_steppers")
            #    toolhead.flush_step_generation()
            #    for s in self.z_steppers:
            ##        s.set_trapq(toolhead.get_trapq())
             #   raise
        # Z should now be level - do final cleanup
       # last_stepper_offset, last_stepper = positions[-1]
        #toolhead.flush_step_generation()
        #last_stepper.set_trapq(toolhead.get_trapq())
        #curpos[2] += first_stepper_offset
        #toolhead.set_position(curpos)





       
def load_config(config):
    return quad_gantry_tilt(config)