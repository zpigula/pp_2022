"""
Z PP2022 Robot Drive
"""

import sys
import time
import odrive
from odrive.enums import *
#from fibre.protocol import ChannelBrokenException

class AxisConfig:
    """
    Class for configuring an Odrive axis.
    Only works with one Odrive at a time.
    """
    
    # Motor Kv (6374 TURNIGY 149)
    MOTOR_KV = 149.0

    def __init__(self, axis_num, axisObj):
        
        """      
        :param axis_num: Which channel/motor on the odrive your referring to.
        :type axis_num: int (0 or 1)
        
        :param axisObj: axis object.
        
        """
        
        self.axis_num = axis_num
        self.odrv_axis = axisObj
    
    def configure_6374_Turnigy_149KV_motor(self):
        """
        Configures the 6374 TURNIGY 149 Motor
        """
        print("\nConfiguring Axis", self.axis_num, "...")
        print("\nSetting 6374 TURNIGY 149 Motor")
        print("------------------------")
        
        # motor config
        self.odrv_axis.motor.config.pole_pairs = 7
        #self.odrv_axis.motor.config.calibration_current = 5
        #self.odrv_axis.motor.config.resistance_calib_max_voltage = 4
        self.odrv_axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        #self.odrv_axis.motor.config.current_lim = 15
        #self.odrv_axis.motor.config.requested_current_range = 20       
        self.odrv_axis.motor.config.torque_constant = 8.27 / self.MOTOR_KV
        
        print("6374 TURNIGY 149 Motor configuration finished.\n")
        
    def configure_AS5047P_encoder(self):
        """
        Configures the AMS (AS5047P) encoder
        """
        
        print("\nConfiguring Axis", self.axis_num, " Encoder")
        print("\nSetting AMS (AS5047P) encoder type")
        print("------------------------")
    

        # encoder config
        self.odrv_axis.encoder.config.abs_spi_cs_gpio_pin = self.axis_num + 7
        self.odrv_axis.encoder.config.mode = 257
        self.odrv_axis.encoder.config.cpr = 16384
        self.odrv_axis.encoder.config.bandwidth = 3000
        
        print("AS5047P encoder configuration finished.\n")
            
    def configure(self):
        # """
        # Configures the AMS (AS5047P) encoder
        # """
        
        # print("\nConfiguring Axis", self.axis_num, " Encoder")
        # print("\nSetting AMS (AS5047P) encoder type")
        # print("------------------------")
    

        # # encoder config
        # self.odrv_axis.encoder.config.abs_spi_cs_gpio_pin = self.axis_num + 7
        # self.odrv_axis.encoder.config.mode = 257
        # self.odrv_axis.encoder.config.cpr = 16384
        # self.odrv_axis.encoder.config.bandwidth = 3000
        
        #self.print_odrive_axis_config()
        

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
        

            
        #self._find_odrive()
        
        print("Odrive Axis configuration finished.\n")
              
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
        
    def set_ramped_velocity_control(self, ramp):
        """
        Puts the motor in Ramped Velocity Control
        """
        print("Placing motor in Ramped Velocity Control Mode")
        # Set the control mode
        self.odrv_axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        
        #Set the velocity ramp rate (acceleration in turn/s^2)
        self.odrv_axis.controller.config.vel_ramp_rate = ramp
        
        # Activate the ramped velocity mode
        self.odrv_axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        
        # Set default velocity
        self.odrv_axis.controller.input_vel = 0
    
    def set_trajectory_control(self, pos, acc_dec=5.0):

        """
        Puts the motor in Trajectory Control
        """
        print("Placing motor in Trajectory Control Mode")
        print("Encoder Count", self.odrv_axis.encoder.shadow_count) 
        
        self.odrv_axis.trap_traj.config.vel_limit = 6.0
        self.odrv_axis.trap_traj.config.accel_limit = acc_dec
        self.odrv_axis.trap_traj.config.decel_limit = acc_dec
        
        # Activate the trajectory module
        self.odrv_axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        
        # Set default velocity
        #self.odrv_axis.controller.input_pos = pos
        self.odrv_axis.controller.move_incremental(pos, True)
        
    def set_position_control_mode(self):

        """
        Puts the motor in Position Control Mode
        """
        print("Placing motor in Position Control Mode")
        print("Encoder Count", self.odrv_axis.encoder.shadow_count) 
        
        self.odrv_axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.odrv_axis.controller.config.vel_limit = 50
        self.odrv_axis.controller.config.pos_gain = 30
        self.odrv_axis.controller.config.vel_gain = 0.02  				
        self.odrv_axis.controller.config.vel_integrator_gain = 0.2
        
        # Activate the trajectory module
        self.odrv_axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        self.odrv_axis.trap_traj.config.vel_limit = 30
        self.odrv_axis.trap_traj.config.accel_limit = 5.0
        self.odrv_axis.trap_traj.config.decel_limit = 5.0
        
        
    def set_velocity(self, velocity):
        """
        Sets motor velocity 
        """
        print("Setting Motor Velocity to", velocity)
        # Set default velocity
        self.odrv_axis.controller.input_vel = velocity
        
    def clear_errors(self):
        """
        Clear Errors 
        """
        if self.odrv_axis.error != 0:
            print("Error: Odrive reported an error of {}".format(self.odrv_axis.error))
            self.odrv_axis.clear_errors()
            print("Error", self.odrv_axis.error)
            
    def test_drive(self):
        """
        Test motor
        """
        print("\nMOTOR TEST")    
        # set ramped velocity mode
        self.odrv_axis.clear_errors()
        time.sleep(1)
        
        self.print_odrive_axis_config()
        self.set_ramped_velocity_control(2)
        
        print("Placing motor in close loop control")
        self.mode_close_loop_control()
        time.sleep(1)
        
        self.set_velocity(3)
        time.sleep(5)
        self.set_velocity(7)
        time.sleep(2)
        self.set_velocity(2)
        time.sleep(2)
        self.set_velocity(0)
        time.sleep(1)
        self.set_velocity(-3)
        time.sleep(5)
        self.set_velocity(-7)
        time.sleep(2)
        self.set_velocity(-4)
        time.sleep(4)
        self.set_velocity(0)
        time.sleep(2)
        
        self.mode_idle()
        time.sleep(2)
        self.mode_close_loop_control()
        #motor_axis0.mode_close_loop_control()
        
        print("Placing motor in trajectory control")
        self.set_trajectory_control(0)
        #motor_axis0.set_trajectory_control(0)
        time.sleep(3)
        
        
        # Go from 0 to 360 degrees in increments of 90 degrees
        for angle in range(0, 360, 90):
            print("Setting motor to {} degrees.".format(angle))
            # motor_config.move_input_pos(angle)
            # time.sleep(5)

            self.set_trajectory_control(0.25)
            #motor_axis0.set_trajectory_control(0.25)
            time.sleep(1)

        self.mode_idle() 
        time.sleep(1)    
            
    def print_odrive_axis_config(self):
         
        print("\n------- Axis", self.axis_num , "Configuration --------" )
            
        print("Pole Pairs", self.odrv_axis.motor.config.pole_pairs)
        print("Calibration Current", self.odrv_axis.motor.config.calibration_current)
        print("Resistance Calibration Max Voltage", self.odrv_axis.motor.config.resistance_calib_max_voltage)  
        print("Motor Type", self.odrv_axis.motor.config.motor_type)       
        print("Current Limit ", self.odrv_axis.motor.config.current_lim)
        print("Requested Current Range ", self.odrv_axis.motor.config.requested_current_range)
        
        print("Torque Constant", self.odrv_axis.motor.config.torque_constant)
        
        print("Encoder Mode", self.odrv_axis.encoder.config.mode)
        print("Encoder CPR", self.odrv_axis.encoder.config.cpr)
        print("Encoder Bandwidth", self.odrv_axis.encoder.config.bandwidth)
        print("Calibration Lockin Current", self.odrv_axis.config.calibration_lockin.current)
        print("Calibration Lockin Ramp Time", self.odrv_axis.config.calibration_lockin.ramp_time)
        print("Calibration Lockin Ramp Distance", self.odrv_axis.config.calibration_lockin.ramp_distance)
        print("Calibration Lockin Accel", self.odrv_axis.config.calibration_lockin.accel)
        print("Calibration Lockin Vel", self.odrv_axis.config.calibration_lockin.vel)
        
        print("Encoder CS Pin", self.odrv_axis.encoder.config.abs_spi_cs_gpio_pin)      
        print("Encoder Count", self.odrv_axis.encoder.shadow_count)
        
        print("Control Mode", self.odrv_axis.controller.config.control_mode)
        print("Vel Limit", self.odrv_axis.controller.config.vel_limit)
        print("Pos Gain", self.odrv_axis.controller.config.pos_gain)
        print("Vel Gain", self.odrv_axis.controller.config.vel_gain) 
        print("Vel Integrator Gain", self.odrv_axis.controller.config.vel_integrator_gain)
        print("Input Mode", self.odrv_axis.controller.config.input_mode)
        
        print("Trap Traj Vel Limit", self.odrv_axis.trap_traj.config.vel_limit) 
        print("Trap Traj Accel Limit", self.odrv_axis.trap_traj.config.accel_limit)
        print("Trap Traj Decel Limit", self.odrv_axis.trap_traj.config.decel_limit)         

  
        print("Error", self.odrv_axis.error)
    