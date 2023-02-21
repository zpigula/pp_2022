"""
Z PP2022 Robot Drive
"""

import sys
import time
import odrive
import odrive_axis_config
from odrive.enums import *
#from fibre.protocol import ChannelBrokenException


class MotorConfig:
    """
    Class for configuring an Odrive axis.
    Only works with one Odrive at a time.
    """
    
    # Motor Kv (6374 TURNIGY 149)
    MOTOR_KV = 149.0

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
        self.odrv_axis0 = getattr(self.odrv, "axis{}".format(self.axis_num))
        self.odrv_axis1 = getattr(self.odrv, "axis{}".format(self.axis_num+1))
        self.axis0 = odrive_axis_config.AxisConfig(0,self.odrv_axis0)
        self.axis1 = odrive_axis_config.AxisConfig(1,self.odrv_axis1)

        
    def print_odrive_config(self):
        print("\n------- ODrive Configuration --------" )
        print("ODrive HW Ver =", self.odrv.hw_version_major, ".", self.odrv.hw_version_minor)
        print("ODrive FW Rev =", self.odrv.fw_version_major, ".", self.odrv.fw_version_minor,".", self.odrv.fw_version_revision)
        print("Serial# ", self.odrv.serial_number)
        print("VBUS Voltage =", self.odrv.vbus_voltage)
        
        print("\nBrake Resistance =", self.odrv.config.brake_resistance)
        print("dc_bus_undervoltage_trip_level =", self.odrv.config.dc_bus_undervoltage_trip_level)
        print("dc_bus_overvoltage_trip_level =", self.odrv.config.dc_bus_overvoltage_trip_level)
        print("dc_max_positive_current =", self.odrv.config.dc_max_positive_current)
        print("dc_max_negative_current =", self.odrv.config.dc_max_negative_current)
        print("max_regen_current =", self.odrv.config.max_regen_current)       

    
    def _save_cofiguration(self):
        
        print("Saving odrive configuration...")
        try: 
            self.odrv.save_configuration()
            print("Configuration saved.")
        except ChannelBrokenException:
            print("ChannelBrokenException on saving configuration")
            pass       

    
    def configure(self):
        """
        Configures the odrive device for a hoverboard motor.
        """
        
        # Erase pre-exsisting configuration
        print("Erasing pre-exsisting configuration...")
        try:
            self.odrv.erase_configuration()
        except ChannelBrokenException:
            pass
        
        self._find_odrive()
        self._print_odrive_config()

        print("\nConfiguring 6374 TURNIGY 149 Motors...")
        print("------------------------")
        
        self.odrv.config.brake_resistance = 2.0
        self.odrv.config.dc_bus_undervoltage_trip_level = 8.0
        #self.odrv.config.dc_bus_overvoltage_trip_level = 56.0
        self.odrv.config.dc_max_positive_current = 4.0
        #self.odrv.config.dc_max_negative_current = -3.0
        self.odrv.config.max_regen_current = 0
        
        self._print_odrive_config()
        self._save_configuration()
        


                
        # self.odrv_axis.motor.config.pole_pairs = 7
        
        # # self.odrv_axis.motor.config.resistance_calib_max_voltage = 4
        # # self.odrv_axis.motor.config.requested_current_range      = 25
        # # self.odrv_axis.motor.config.current_control_bandwidth    = 100
        # self.odrv_axis.motor.config.torque_constant = 8.27 / self.MOTOR_KV
        # self.odrv_axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        # self.odrv_axis.encoder.config.abs_spi_cs_gpio_pin = self.axis_num + 7
        # self.odrv_axis.encoder.config.mode = 257
        # self.odrv_axis.encoder.config.cpr = 16384
        
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
        

            
        self._find_odrive()
        
        print("Odrive configuration finished.\n")

    def test_drive2(self):
        """
        Test motor
        """
        print("\nMOTOR TEST")
            

        self.axis0.odrv_axis.clear_errors()
        self.axis1.odrv_axis.clear_errors()
        time.sleep(1)
        
        # set ramped velocity mode
        self.axis0.set_ramped_velocity_control(2)  
        self.axis1.set_ramped_velocity_control(2)
        
        print("Placing motor in close loop control")
        self.axis0.mode_close_loop_control()
        self.axis1.mode_close_loop_control()
        time.sleep(1)
        
        self.axis0.set_velocity(-9)
        self.axis1.set_velocity(9)
        time.sleep(5)
        self.axis0.set_velocity(-2)
        self.axis1.set_velocity(2)
        time.sleep(2)
        self.axis0.set_velocity(0)
        self.axis1.set_velocity(0)
        time.sleep(2)
        self.axis0.set_velocity(9)
        self.axis1.set_velocity(-9)
        time.sleep(5)
        self.axis0.set_velocity(2)
        self.axis1.set_velocity(-2)
        time.sleep(2)
        self.axis0.set_velocity(0)
        self.axis1.set_velocity(0)
        time.sleep(2)
        
        self.axis0.mode_idle()
        self.axis1.mode_idle() 
        time.sleep(1)
        

    
    #--------------------------------------------------------
    def test_drive1(self):
        """
        Test motor
        """
        print("\nMOTOR TEST")
            

        self.axis0.odrv_axis.clear_errors()
        self.axis1.odrv_axis.clear_errors()
        time.sleep(1)
        
        # set ramped velocity mode
        self.axis0.set_ramped_velocity_control(2)  
        self.axis1.set_ramped_velocity_control(2)
        
        print("Placing motor in close loop control")
        self.axis0.mode_close_loop_control()
        self.axis1.mode_close_loop_control()
        time.sleep(1)
        
        self.axis0.set_velocity(-3)
        self.axis1.set_velocity(3)
        time.sleep(1)
        self.axis0.set_velocity(-2)
        self.axis1.set_velocity(2)
        time.sleep(1)
        self.axis0.set_velocity(0)
        self.axis1.set_velocity(0)
        time.sleep(1)
        self.axis0.set_velocity(3)
        self.axis1.set_velocity(-3)
        time.sleep(1)
        self.axis0.set_velocity(2)
        self.axis1.set_velocity(-2)
        time.sleep(1)
        self.axis0.set_velocity(0)
        self.axis1.set_velocity(0)
        time.sleep(1)
        
        self.axis0.mode_idle()
        self.axis1.mode_idle() 
        time.sleep(1)
        


        
        print("Placing motor in trajectory control")
        self.axis0.set_trajectory_control(0)
        self.axis1.set_trajectory_control(0)
        time.sleep(1)
        
        print("Placing motor in close loop control")
        self.axis0.mode_close_loop_control()
        self.axis1.mode_close_loop_control()
        time.sleep(1)
                
        # Go from 0 to 360 degrees in increments of 90 degrees
        for angle in range(0, 360, 90):
            print("Setting motor to {} degrees.".format(angle))
            self.axis0.set_trajectory_control(1.5)
            self.axis1.set_trajectory_control(1.5)
            time.sleep(1.2)
        
        # Go from 0 to 360 degrees in increments of 90 degrees
        #for angle in range(0, 360, 90):
        #    print("Setting motor to {} degrees.".format(angle))
        self.axis0.set_trajectory_control(-6)
        self.axis1.set_trajectory_control(-6)
        time.sleep(4)
            
        # Go from 0 to 360 degrees in increments of 90 degrees
        #for angle in range(0, 360, 90):
        #    print("Setting motor to {} degrees.".format(angle))
        self.axis0.set_trajectory_control(4)
        self.axis1.set_trajectory_control(-4)
        time.sleep(3)
            
        # Go from 0 to 360 degrees in increments of 90 degrees
        #for angle in range(0, 360, 90):
        #    print("Setting motor to {} degrees.".format(angle))
        self.axis0.set_trajectory_control(-4)
        self.axis1.set_trajectory_control(4)
        time.sleep(3)  

        self.axis0.set_trajectory_control(12)
        self.axis1.set_trajectory_control(12)
        time.sleep(4)

        self.axis0.set_trajectory_control(-24)
        self.axis1.set_trajectory_control(-24)
        time.sleep(6)  
        
        
                # set ramped velocity mode
        self.axis0.set_ramped_velocity_control(2)  
        self.axis1.set_ramped_velocity_control(2)
        
        print("Placing motor in close loop control")
        self.axis0.mode_close_loop_control()
        self.axis1.mode_close_loop_control()
        time.sleep(1)
        
        self.axis0.set_velocity(-3)
        self.axis1.set_velocity(3)
        time.sleep(1)
        self.axis0.set_velocity(-2)
        self.axis1.set_velocity(2)
        time.sleep(1)
        self.axis0.set_velocity(0)
        self.axis1.set_velocity(0)
        time.sleep(1)
        self.axis0.set_velocity(3)
        self.axis1.set_velocity(-3)
        time.sleep(1)
        self.axis0.set_velocity(2)
        self.axis1.set_velocity(-2)
        time.sleep(1)
        self.axis0.set_velocity(0)
        self.axis1.set_velocity(0)
        time.sleep(1)
        
        self.axis0.mode_idle()
        self.axis1.mode_idle() 
        time.sleep(1)
        

        self.axis0.mode_idle()
        self.axis1.mode_idle()  
        time.sleep(1)      
        
    #--------------------------------------------------------------
    def test_drive(self):
        """
        Test motor
        """
        print("\nMOTOR TEST")
            

        self.axis0.odrv_axis.clear_errors()
        self.axis1.odrv_axis.clear_errors()
        time.sleep(1)
        
        # set ramped velocity mode
        self.axis0.set_ramped_velocity_control(2)  
        self.axis1.set_ramped_velocity_control(2)
        
        print("Placing motor in close loop control")
        self.axis0.mode_close_loop_control()
        self.axis1.mode_close_loop_control()
        time.sleep(1)
        
        self.axis0.set_velocity(-3)
        self.axis1.set_velocity(3)
        time.sleep(7)
        self.axis0.set_velocity(-5)
        self.axis1.set_velocity(5)
        time.sleep(2)
        self.axis0.set_velocity(-2)
        self.axis1.set_velocity(2)
        time.sleep(2)
        self.axis0.set_velocity(0)
        self.axis1.set_velocity(0)
        time.sleep(1)
        self.axis0.set_velocity(3)
        self.axis1.set_velocity(-3)
        time.sleep(7)
        self.axis0.set_velocity(5)
        self.axis1.set_velocity(-5)
        time.sleep(2)
        self.axis0.set_velocity(2)
        self.axis1.set_velocity(-2)
        time.sleep(2)
        self.axis0.set_velocity(0)
        self.axis1.set_velocity(0)
        time.sleep(1)
        
        self.axis0.mode_idle()
        self.axis1.mode_idle() 
        time.sleep(2)
        


        
        print("Placing motor in trajectory control")
        self.axis0.set_trajectory_control(0)
        self.axis1.set_trajectory_control(0)
        time.sleep(1)
        
        print("Placing motor in close loop control")
        self.axis0.mode_close_loop_control()
        self.axis1.mode_close_loop_control()
        time.sleep(1)
                
        # Go from 0 to 360 degrees in increments of 90 degrees
        for angle in range(0, 360, 90):
            print("Setting motor to {} degrees.".format(angle))
            self.axis0.set_trajectory_control(1.5)
            self.axis1.set_trajectory_control(1.5)
            time.sleep(2)
        
        # Go from 0 to 360 degrees in increments of 90 degrees
        for angle in range(0, 360, 90):
            print("Setting motor to {} degrees.".format(angle))
            self.axis0.set_trajectory_control(-1.5)
            self.axis1.set_trajectory_control(-1.5)
            time.sleep(2)
            
        # Go from 0 to 360 degrees in increments of 90 degrees
        for angle in range(0, 360, 90):
            print("Setting motor to {} degrees.".format(angle))
            self.axis0.set_trajectory_control(1)
            self.axis1.set_trajectory_control(-1)
            time.sleep(1)
            
        # Go from 0 to 360 degrees in increments of 90 degrees
        for angle in range(0, 360, 90):
            print("Setting motor to {} degrees.".format(angle))
            self.axis0.set_trajectory_control(-1)
            self.axis1.set_trajectory_control(1)
            time.sleep(1)    

        self.axis0.mode_idle()
        self.axis1.mode_idle()  
        time.sleep(1)    
        

if __name__ == "__main__":
    motor = MotorConfig(0)
    motor.print_odrive_config()
    motor.axis0.print_odrive_axis_config()
    motor.axis1.print_odrive_axis_config()
    
    
    # motor.axis0.set_position_control_mode()
    # motor.axis0.mode_close_loop_control()
    # motor.axis0.move_input_pos(180)
    # time.sleep(1)
    # motor.axis0.move_input_pos(360)
    # time.sleep(1)
    # motor.axis0.move_input_pos(120)
    # time.sleep(1)
    # motor.axis0.move_input_pos(0)
    # time.sleep(1)
    # motor.axis0.mode_idle() 
    
    
    motor.test_drive2()
    #motor.axis0.test_drive()
    #motor.axis1.test_drive()
    #motor_axis1 = MotorConfig(1)
    
    #motor_axis0.test_drive()
    
    #print("MOTOR 0 TEST")
    #motor_axis0.print_odrive_axis_config()
    #motor_axis0.print_odrive_axis_config(1)
 
    
    #print("\nMOTOR 1 TEST")    

    #motor_axis1.test_drive()

    #motor_axis1._print_odrive_axis_config()

    