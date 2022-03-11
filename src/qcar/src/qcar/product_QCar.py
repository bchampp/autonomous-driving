import numpy as np
import pygame
import time

from quanser.hardware import HIL, HILError, PWMMode
# from quanser.multimedia import Video3D, VideoCapture, Video3DStreamType, MediaError, ImageFormat, ImageDataType
# from quanser.devices import RPLIDAR, RangingMeasurement, RangingMeasurementMode, DeviceError, RangingDistance
from .q_misc import Utilities
saturate = Utilities.saturate

class QCar():
    #region: Buffers
    # Throttle Write Only - PWM channel 0 is mtr cmd
    write_pwm_channel_throttle = np.array([0], dtype=np.int32)
    write_pwm_buffer_throttle = np.array([0], dtype=np.float64)

    # Steering Write Only - Other channel 1000 is steering cmd
    write_other_channel_steering = np.array([1000], dtype=np.int32)
    write_other_buffer_steering = np.array([0], dtype=np.float64)

    # LEDs Write Only - Other channel 11000:11003 + 11008:11011 are LEDs
    write_other_channels_LEDs = np.array([11008, 11009, 11010, 11011, 11000, 11001, 11002, 11003], dtype=np.int32)
    write_other_buffer_LEDs = np.zeros(8, dtype=np.float64)
    
    # User LEDs Write Only - Other channel 11004:11007 are User LEDs
    write_other_channels_usr_LEDs = np.array([11004, 11005, 11006, 11007], dtype=np.int32)
    write_other_buffer_usr_LEDs = np.zeros(4, dtype=np.float64)

    # Steering and LEDs Write - Other channel 1000 is steering cmd and 11000:11003 + 11008:11011 are LEDs
    write_other_channels_str_LEDs = np.array([1000, 11008, 11009, 11010, 11011, 11000, 11001, 11002, 11003], dtype=np.int32)
    write_other_buffer_str_LEDs = np.append(np.array([0], dtype=np.float64), np.zeros(8, dtype=np.float64))

    # Initialize return arrays
    mtr_current, bat_voltage = np.zeros(2, dtype=np.float64)
    mtr_encoder = np.zeros(1, dtype=np.int32)
    accelerometer = np.zeros(3, dtype=np.float64)
    gyroscope = np.zeros(3, dtype=np.float64)

    # Battery Read Only - Analog channel 6 is battery voltage
    read_analog_channels_battery = np.array([6], dtype=np.int32)
    read_analog_buffer_battery = np.zeros(1, dtype=np.float64)

    # Encoder Read Only - Encoder channel 0 is throttle motor encoder
    read_encoder_channels_throttle = np.array([0], dtype=np.int32)
    read_encoder_buffer_throttle = np.zeros(1, dtype=np.int32)

    # Gyroscope Read Only - Other channels 3000:3002 are for gyroscope 
    read_other_channels_gyroscope = np.array([3000, 3001, 3002], dtype=np.int32)
    read_other_buffer_gyroscope = np.zeros(3, dtype=np.float64)

    # Accelerometer Read Only - Other channels 4000:4002 are for accelerometer 
    read_other_channels_accelerometer = np.array([4000, 4001, 4002], dtype=np.int32)
    read_other_buffer_accelerometer = np.zeros(3, dtype=np.float64)

    # IMU Read - Other channels 3000:3002 + 4000:4002 are for IMU 
    read_other_channels_IMU = np.array([3000, 3001, 3002, 4000, 4001, 4002], dtype=np.int32)
    read_other_buffer_IMU = np.zeros(6, dtype=np.float64)

    # Power Read - Analog channels 5, 6 are motor current and battery voltage
    read_analog_channels_power = np.array([5, 6], dtype=np.int32)
    read_analog_buffer_power = np.zeros(2, dtype=np.float64)
    #endregion

    def __init__(self):
        ''' This function configures the QCar and returns a handle to the QCar card. Use the handle for other methods such as qcar_io or terminate_qcar.'''

        self.card = HIL()
        try:
            # Open the Card
            self.card.open("qcar", "0")
            if self.card.is_valid():
                # Set PWM mode (duty cycle) and frequency
                self.card.set_pwm_mode(np.array([0], dtype=np.uint32), len(np.array([0], dtype=np.uint32)), np.array([PWMMode.DUTY_CYCLE], dtype=np.int32))
                self.card.set_pwm_frequency(np.array([0], dtype=np.uint32), len(np.array([0], dtype=np.uint32)), np.array([60e6/4096], dtype=np.float64))

                # Set Motor coast to 0            
                self.card.write_digital(np.array([40], dtype=np.uint32), len(np.array([40], dtype=np.uint32)), np.zeros(len(np.array([0], dtype=np.uint32)), dtype=np.float64))
            
                # Set Encoder Quadrature 
                encoder_channels = np.array([0], dtype=np.uint32)
                num_encoder_channels = len(encoder_channels)                     
                self.card.set_encoder_quadrature_mode(encoder_channels, num_encoder_channels, np.array([4], dtype=np.uint32))
                self.card.set_encoder_filter_frequency(encoder_channels, num_encoder_channels, np.array([60e6/1], dtype=np.uint32))         
                self.card.set_encoder_counts(encoder_channels, num_encoder_channels, np.zeros(1, dtype=np.int32))
                    
                print('QCar configured successfully.')
        
        except HILError as h:
            print(h.get_error_message())  

    def terminate(self):
        ''' This function terminates the QCar card after setting final values for throttle, steering and LEDs.'''
        
        # PWM channel 0 is mtr cmd
        pwm_channels = np.array([0], dtype=np.int32)
        pwm_buffer = np.zeros(1, dtype=np.float64)

        # Other channel 1000 is steering, 11008:11011 are 4x indicators, and 11000:11003 are 4x lamps  
        other_channels = np.array([1000, 11000, 11001, 11002, 11003, 11008, 11009, 11010, 11011], dtype=np.int32)
        other_buffer = np.zeros(9, dtype=np.float64)

        try:    
            self.card.write(None, 0, pwm_channels, len(pwm_channels), None, 0, other_channels, len(other_channels), None, pwm_buffer, None, other_buffer)    
            self.card.close()
            
        except HILError as h:
            print(h.get_error_message())

    def read_encoder(self):
        '''Use this to read encoder counts \n

        OUTPUTS:
        mtr_encoder - throttle motor encoder measurement'''
    
        try:
            if True:
                #Reads: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.read(None, 0, self.read_encoder_channels_throttle, 1, None, 0, None, 0, 
                                None, self.read_encoder_buffer_throttle, None, None)

        except HILError as h:
            print(h.get_error_message())

        finally:
            return self.read_encoder_buffer_throttle[0]

    def read_gyroscope(self):
        '''Use this to read the gyroscope \n
        
        OUTPUTS:
        gyroscope - gyroscopic measurement'''
    
        try:
            if True:
                #Reads: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.read(None, 0, None, 0, None, 0, self.read_other_channels_gyroscope, 3, 
                                None, None, None, self.read_other_buffer_gyroscope)

        except HILError as h:
            print(h.get_error_message())

        finally:
            self.read_other_buffer_IMU[0:3] = self.read_other_buffer_gyroscope
            return self.read_other_buffer_gyroscope

    def read_accelerometer(self):
        '''Use this to read the accelerometer \n

        OUTPUTS:
        accelerometer - accelerometer measurement'''
    
        try:
            if True:
                #Reads: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.read(None, 0, None, 0, None, 0, self.read_other_channels_accelerometer, 3, 
                                None, None, None, self.read_other_buffer_accelerometer)

        except HILError as h:
            print(h.get_error_message())

        finally:
            self.read_other_buffer_IMU[3:6] = self.read_other_buffer_accelerometer
            return self.read_other_buffer_accelerometer

    def read_IMU(self):
        '''Use this to read the IMU (gyroscope and accelerometer) \n

        OUTPUTS:
        gyroscope - gyroscopic measurement
        accelerometer - accelerometer measurement'''
    
        try:
            if True:
                #Reads: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.read(None, 0, None, 0, None, 0, self.read_other_channels_IMU, 6, 
                                None, None, None, self.read_other_buffer_IMU)

        except HILError as h:
            print(h.get_error_message())

        finally:
            self.read_other_buffer_gyroscope = self.read_other_buffer_IMU[0:3]
            self.read_other_buffer_accelerometer = self.read_other_buffer_IMU[3:6]
            return self.read_other_buffer_gyroscope, self.read_other_buffer_gyroscope 

    def read_power(self):
        '''Use this to read the motor current and battery voltage \n

        OUTPUTS:
        mtr_current - throttle motor current measurement
        bat_voltage - battery voltage measurement'''
    
        try:
            if True:
                #Reads: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.read(self.read_analog_channels_power, 2, None, 0, None, 0, None, 0, 
                                self.read_analog_buffer_power, None, None, None)

        except HILError as h:
            print(h.get_error_message())

        finally:
            self.read_analog_buffer_battery = self.read_analog_buffer_power[1]
            return self.read_analog_buffer_power[0], self.read_analog_buffer_battery

    def read_std(self):
        '''Use this to read the motor current, battery voltage and encoder counts \n
        
        OUTPUTS:
        mtr_current - throttle motor current measurement
        bat_voltage - battery voltage measurement
        mtr_encoder - throttle motor encoder measurement'''
        
        # IO
        try:
            if True:
                #Reads: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.read(self.read_analog_channels_power, 2, self.read_encoder_channels_throttle, 1, None, 0, None, 0, 
                                self.read_analog_buffer_power, self.read_encoder_buffer_throttle, None, None)

        except HILError as h:
            print(h.get_error_message())

        finally:
            self.read_analog_buffer_battery = self.read_analog_buffer_power[1]
            return self.read_analog_buffer_power[0], self.read_analog_buffer_power[1], self.read_encoder_buffer_throttle[0]

    def write_mtrs(self, mtr_cmd):
        '''Use this to write motor commands\n
        INPUTS:
        mtr_cmd - numpy 1x2 array of throttle (%) and steering (rad) motor commands. '''
    
        self.write_pwm_buffer_throttle[0] = -saturate(mtr_cmd[0], 0.2, -0.2)
        self.write_other_buffer_steering[0] = -saturate(mtr_cmd[1], 0.5, -0.5)
        
        try:
            if True:
                #Writes: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer           
                self.card.write(None, 0, self.write_pwm_channel_throttle, 1, None, 0, self.write_other_channel_steering, 1, 
                                None, self.write_pwm_buffer_throttle, None, self.write_other_buffer_steering)

        except HILError as h:
            print(h.get_error_message())

    def write_LEDs(self, LEDs):
        '''Use this to write LED commands\n
        INPUTS:
        LEDs - numpy 1x8 array of 4x indicators (0, 1) and 4x lamps (0, 1)'''

        self.write_other_buffer_LEDs = LEDs

        try:
            if True:
                #Writes: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer           
                self.card.write(None, 0, None, 0, None, 0, self.write_other_channels_LEDs, 8, 
                                None, None, None, self.write_other_buffer_LEDs)
                
        except HILError as h:
            print(h.get_error_message())

    def write_usr_LEDs(self, LEDs):
        '''Use this to write user LED commands\n
        INPUTS:
        LEDs - numpy 1x4 array of 4x LEDs'''

        self.write_other_buffer_usr_LEDs = LEDs

        try:
            if True:
                #Writes: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer           
                self.card.write(None, 0, None, 0, None, 0, self.write_other_channels_usr_LEDs, 4, 
                                None, None, None, self.write_other_buffer_usr_LEDs)
                
        except HILError as h:
            print(h.get_error_message())

    def write_std(self, mtr_cmd, LEDs):
        '''Use this to write motor and LED commands\n
        INPUTS:
        mtr_cmd - numpy 1x2 array of throttle (%) and steering (rad) motor commands. 
        LEDs - numpy 1x8 array of 4x indicators (0, 1) and 4x lamps (0, 1)'''
    
        self.write_pwm_buffer_throttle[0] = -saturate(mtr_cmd[0], 0.2, -0.2)
        self.write_other_buffer_str_LEDs[0] = -saturate(mtr_cmd[1], 0.5, -0.5)
        self.write_other_buffer_str_LEDs[1:9] = LEDs
        
        try:
            if True:
                #Writes: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer           
                self.card.write(None, 0, self.write_pwm_channel_throttle, 1, None, 0, self.write_other_channels_str_LEDs, 9, 
                                None, self.write_pwm_buffer_throttle, None, self.write_other_buffer_str_LEDs)

        except HILError as h:
            print(h.get_error_message())

    def read_write_std(self, mtr_cmd, LEDs):
        '''Use this to write motor and LED commands, and read the battery voltage, motor current and encoder counts \n

        INPUTS:
        mtr_cmd - numpy 1x2 array of throttle (%) and steering (rad) motor commands. 
        LEDs - numpy 1x8 array of 4x indicators (0, 1) and 4x lamps (0, 1)

        OUTPUTS:
        mtr_current - throttle motor current measurement
        bat_voltage - battery voltage measurement
        mtr_encoder - throttle motor encoder measurement'''
    
        self.write_pwm_buffer_throttle[0] = -saturate(mtr_cmd[0], 0.2, -0.2)
        self.write_other_buffer_str_LEDs[0] = -saturate(mtr_cmd[1], 0.5, -0.5)
        self.write_other_buffer_str_LEDs[1:9] = LEDs

        # IO
        try:
            if True:
                #Writes: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer           
                self.card.write(None, 0, self.write_pwm_channel_throttle, 1, None, 0, self.write_other_channels_str_LEDs, 9, 
                                None, self.write_pwm_buffer_throttle, None, self.write_other_buffer_str_LEDs)

                #Reads: Analog Channel, Num Analog Channel, PWM Channel, Num PWM Channel, Digital Channel, Num Digital Channel, Other Channel, Num Other Channel, Analog Buffer, PWM Buffer, Digital Buffer, Other Buffer
                self.card.read(self.read_analog_channels_power, 2, self.read_encoder_channels_throttle, 1, None, 0, None, 0, 
                                self.read_analog_buffer_power, self.read_encoder_buffer_throttle, None, None)

        except HILError as h:
            print(h.get_error_message())

        finally:
            self.read_analog_buffer_battery = self.read_analog_buffer_power[1]
            return self.read_analog_buffer_power[0], self.read_analog_buffer_battery, self.read_encoder_buffer_throttle[0]
