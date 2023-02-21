"""
These configurations are based off of the hoverboard tutorial found on the 
Odrive Robotics website located here:
https://docs.odriverobotics.com/hoverboard

"""

import sys
import time
import odrive
from odrive.enums import *
#from fibre.protocol import ChannelBrokenException

class MotorConfig:
    """
    Class for configuring an Odrive axis for a Hoverboard motor. 
    Only works with one Odrive at a time.
    """
    
    # Motor Kv (6374 TURNIGY 149)
    MOTOR_KV = 149.0
        
    # Min/Max phase inductance of motor
    MIN_PHASE_INDUCTANCE = 0
    MAX_PHASE_INDUCTANCE = 0.001
    
    # Min/Max phase resistance of motor
    MIN_PHASE_RESISTANCE = 0
    MAX_PHASE_RESISTANCE = 0.5
    
    # Tolerance for encoder offset float
    ENCODER_OFFSET_FLOAT_TOLERANCE = 0.05

    def __init__(self, axis_num):
        """
        Initalizes MotorConfig class by finding odrive, erase its 
        configuration, and grabbing specified axis object.
        
        :param axis_num: Which channel/motor on the odrive your referring to.
        :type axis_num: int (0 or 1)
        """
        
        self.axis_num = axis_num
    
        # Connect to Odrive
        print("Looking for ODrive...")
        self._find_odrive()
        print("Found ODrive Axis ", self.axis_num)
        
    def _find_odrive(self):
        # connect to Odrive
        self.odrv = odrive.find_any()
        self.odrv_axis = getattr(self.odrv, "axis{}".format(self.axis_num))
        
    def _print_odrive_config(self):
        print("ODrive HW Ver =", self.odrv.hw_version_major, ".", self.odrv.hw_version_minor)
        print("ODrive FW Rev =", self.odrv.fw_version_major, ".", self.odrv.fw_version_minor,".", self.odrv.fw_version_revision)
        print("Serial# ", self.odrv.serial_number)
        print("VBUS Voltage =", self.odrv.vbus_voltage)
        print("Brake Resistance =", self.odrv.config.brake_resistance)
        
    
    def _print_odrive_axis_config(self): 
        print("Motor Type", self.odrv_axis.motor.config.motor_type)       
        print("Current Limit ", self.odrv_axis.motor.config.current_lim)
        print("Calibration Current", self.odrv_axis.motor.config.calibration_current)
        print("Torque Constant", self.odrv_axis.motor.config.torque_constant)
        
        print("Encoder Mode", self.odrv_axis.encoder.config.mode)
        print("Encoder CPR", self.odrv_axis.encoder.config.cpr)
        print("Encoder CS Pin", self.odrv_axis.encoder.config.abs_spi_cs_gpio_pin)      
        print("Encoder Count", self.odrv_axis.encoder.shadow_count)       
  
        print("Error", self.odrv_axis.error)
    
    def _save_cofiguration(self):
        print("Saving calibration configuration and rebooting...")
        self.odrv.save_configuration()
        print("Calibration configuration saved.")
    
    def configure(self):
        """
        Configures the odrive device for a hoverboard motor.
        """

        
        # Erase pre-exsisting configuration
        #print("Erasing pre-exsisting configuration...")
        # try:
        #     self.odrv.erase_configuration()
        # except ChannelBrokenException:
        #     pass
        
        self._find_odrive()
        self._print_odrive_config()

        print("\nConfiguring 6374 TURNIGY 149 Motors...")
        print("------------------------")
        
        self.odrv_axis.motor.config.pole_pairs = 7
        
        # self.odrv_axis.motor.config.resistance_calib_max_voltage = 4
        # self.odrv_axis.motor.config.requested_current_range      = 25
        # self.odrv_axis.motor.config.current_control_bandwidth    = 100
        self.odrv_axis.motor.config.torque_constant = 8.27 / self.MOTOR_KV
        self.odrv_axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        self.odrv_axis.encoder.config.abs_spi_cs_gpio_pin = self.axis_num + 7
        self.odrv_axis.encoder.config.mode = 257
        self.odrv_axis.encoder.config.cpr = 16384
        
        self._print_odrive_axis_config()
        

        # # Since hall sensors are low resolution feedback, we also bump up the 
        # #offset calibration displacement to get better calibration accuracy.
        # # self.odrv_axis.encoder.config.calib_scan_distance = 150

        # # Since the hall feedback only has 90 counts per revolution, we want to 
        # # reduce the velocity tracking bandwidth to get smoother velocity 
        # # estimates. We can also set these fairly modest gains that will be a
        # # bit sloppy but shouldn’t shake your rig apart if it’s built poorly. 
        # # Make sure to tune the gains up when you have everything else working 
        # # to a stiffness that is applicable to your application.
        # self.odrv_axis.encoder.config.bandwidth = 100
        # self.odrv_axis.controller.config.pos_gain = 1
        # self.odrv_axis.controller.config.vel_gain = 0.02 * self.odrv_axis.motor.config.torque_constant * self.odrv_axis.encoder.config.cpr
        # self.odrv_axis.controller.config.vel_integrator_gain = 0.1 * self.odrv_axis.motor.config.torque_constant * self.odrv_axis.encoder.config.cpr
        # self.odrv_axis.controller.config.vel_limit = 10

        # # Set in position control mode so we can control the position of the 
        # # wheel
        # self.odrv_axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

        # # In the next step we are going to start powering the motor and so we 
        # # want to make sure that some of the above settings that require a 
        # # reboot are applied first.
        # print("Saving manual configuration and rebooting...")
        # self.odrv.save_configuration()
        # print("Manual configuration saved.")
        # try:
        #     self.odrv.reboot()
        # except ChannelBrokenException:
        #     pass
            
        # self._find_odrive()

        # input("Make sure the motor is free to move, then press enter...")
        
        # print("Calibrating Odrive for robot motor (you should hear a "
        # "beep)...")
        
        # self.odrv_axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        
        # # Wait for calibration to take place
        # time.sleep(10)

        # if self.odrv_axis.motor.error != 0:
        #     print("Error: Odrive reported an error of {} while in the state " 
        #     "AXIS_STATE_MOTOR_CALIBRATION. Printing out Odrive motor data for "
        #     "debug:\n{}".format(self.odrv_axis.motor.error, 
        #                         self.odrv_axis.motor))
            
        #     sys.exit(1)

        # if self.odrv_axis.motor.config.phase_inductance <= self.MIN_PHASE_INDUCTANCE or \
        # self.odrv_axis.motor.config.phase_inductance >= self.MAX_PHASE_INDUCTANCE:
        #     print("Error: After odrive motor calibration, the phase inductance "
        #     "is at {}, which is outside of the expected range. Either widen the "
        #     "boundaries of MIN_PHASE_INDUCTANCE and MAX_PHASE_INDUCTANCE (which "
        #     "is between {} and {} respectively) or debug/fix your setup. Printing "
        #     "out Odrive motor data for debug:\n{}".format(self.odrv_axis.motor.config.phase_inductance, 
        #                                                   self.MIN_PHASE_INDUCTANCE,
        #                                                   self.MAX_PHASE_INDUCTANCE, 
        #                                                   self.odrv_axis.motor))
            
        #     sys.exit(1)

        # if self.odrv_axis.motor.config.phase_resistance <= self.MIN_PHASE_RESISTANCE or \
        # self.odrv_axis.motor.config.phase_resistance >= self.MAX_PHASE_RESISTANCE:
        #     print("Error: After odrive motor calibration, the phase resistance "
        #     "is at {}, which is outside of the expected range. Either raise the "
        #     "MAX_PHASE_RESISTANCE (which is between {} and {} respectively) or "
        #     "debug/fix your setup. Printing out Odrive motor data for " 
        #     "debug:\n{}".format(self.odrv_axis.motor.config.phase_resistance, 
        #                         self.MIN_PHASE_RESISTANCE,
        #                         self.MAX_PHASE_RESISTANCE, 
        #                         self.odrv_axis.motor))
            
        #     sys.exit(1)

        # # If all looks good, then lets tell ODrive that saving this calibration 
        # # to persistent memory is OK
        # self.odrv_axis.motor.config.pre_calibrated = True

        # # Check the alignment between the motor and the hall sensor. Because of 
        # # this step you are allowed to plug the motor phases in random order and
        # # also the hall signals can be random. Just don’t change it after 
        # # calibration.
        # print("Calibrating Odrive for encoder...")
        # self.odrv_axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        
        # # Wait for calibration to take place
        # time.sleep(30)
            
        # if self.odrv_axis.encoder.error != 0:
        #     print("Error: Odrive reported an error of {} while in the state "
        #     "AXIS_STATE_ENCODER_OFFSET_CALIBRATION. Printing out Odrive encoder "
        #     "data for debug:\n{}".format(self.odrv_axis.encoder.error, 
        #                                  self.odrv_axis.encoder))
            
        #     sys.exit(1)
        
        # # If offset_float isn't 0.5 within some tolerance, or its not 1.5 within
        # # some tolerance, raise an error
        # if not ((self.odrv_axis.encoder.config.offset_float > 0.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        # self.odrv_axis.encoder.config.offset_float < 0.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE) or \
        # (self.odrv_axis.encoder.config.offset_float > 1.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        # self.odrv_axis.encoder.config.offset_float < 1.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE)):
        #     print("Error: After odrive encoder calibration, the 'offset_float' "
        #     "is at {}, which is outside of the expected range. 'offset_float' "
        #     "should be close to 0.5 or 1.5 with a tolerance of {}. Either "
        #     "increase the tolerance or debug/fix your setup. Printing out "
        #     "Odrive encoder data for debug:\n{}".format(self.odrv_axis.encoder.config.offset_float, 
        #                                                 self.ENCODER_OFFSET_FLOAT_TOLERANCE, 
        #                                                 self.odrv_axis.encoder))
                       
        #     sys.exit(1)
        
        # # If all looks good, then lets tell ODrive that saving this calibration 
        # # to persistent memory is OK
        # self.odrv_axis.encoder.config.pre_calibrated = True
        

            
        self._find_odrive()
        
        print("Odrive configuration finished.\n")
    
    def mode_idle(self):
        """
        Puts the motor in idle (i.e. can move freely).
        """
        
        self.odrv_axis.requested_state = AXIS_STATE_IDLE
    
    def mode_close_loop_control(self):
        """
        Puts the motor in closed loop control.
        """
        
        self.odrv_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        
    def move_input_pos(self, angle):
        """
        Puts the motor at a certain angle.
        
        :param angle: Angle you want the motor to move.
        :type angle: int or float
        """
        
        self.odrv_axis.controller.input_pos = angle/360.0

if __name__ == "__main__":
    motor_axis0 = MotorConfig(axis_num = 0)
    motor_axis0.configure()
    #motor_axis0._save_cofiguration()
    
    time.sleep(3)
    
    motor_axis1 = MotorConfig(axis_num = 1)
    motor_axis1.configure()
    motor_axis1._save_cofiguration()
    # print("CONDUCTING MOTOR TEST")
    # print("Placing motor in close loop control. If you move motor, motor will "
    #       "resist you.")
    # motor_config.mode_close_loop_control()
    
    # # Go from 0 to 360 degrees in increments of 30 degrees
    # for angle in range(0, 390, 30):
    #     print("Setting motor to {} degrees.".format(angle))
    #     motor_config.move_input_pos(angle)
    #     time.sleep(5)
    
    # print("Placing motor in idle. If you move motor, motor will "
    #       "move freely")
    motor_axis0.mode_idle()
    motor_axis1.mode_idle()
    