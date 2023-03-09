from queue import Queue
from threading import Thread, Event, Lock
import csv
import os
import time
import random
import numpy as np
import pickle
import math
from simple_pid import PID

class DHS1100_controller:

    def change_power(self, target_power, duration = 10):
        """
        Changes the power from {self.current power} to {target_power} in {duration} seconds
        Power is changed linearly every {gap} seconds.
        """

        # Seconds between each change
        gap = 0.4
        sleep_time = 0.01

        steps = int(duration / gap) + 1
        approaching_power = np.linspace(self.current_power, target_power, steps)[1:]
        approaching_time = np.linspace(0, duration, steps)[1:]


        index = 0
        initial_time = time.time()
        
        print("Changing power from {} to {} in {} seconds".format(self.current_power, target_power, duration))
        self.logger.info("Changing power from {} to {} in {} seconds".format(self.current_power, target_power, duration))

        while 1:
            if time.time() > (initial_time + approaching_time[index]):

                self.lock_communication_furnace.acquire()

                try:
                    self.dhs1100.write_attribute("target_power", approaching_power[index])
                except:
                    print("Exeption when sending power")
                    
                if self.debug_power:
                    print(approaching_time[index], approaching_power[index], time.time() - initial_time)
                index += 1

                self.lock_communication_furnace.release()

                time.sleep(sleep_time)

                if index == steps - 1:
                    break

            else:
                time.sleep(0.01)
                
        print("Power changed from {} to {} in {} seconds".format(self.current_power, target_power, time.time() - initial_time))
        self.logger.info("Power changed from {} to {} in {} seconds".format(self.current_power, target_power, time.time() - initial_time))

        try:
        
            self.lock_communication_furnace.acquire()
            
            time.sleep(0.005)
            
            self.current_power = self.dhs1100.read_attribute("target_power").value

            self.lock_communication_furnace.release()
            
        except:
            
            self.current_power = target_power

    def do_ramp(self):
        """
        Custom hardcoded ramp. Changes the power.
        """

        self.change_power(1670, 17)

        self.change_power(5900, 143)

        self.change_power(6320, 50)

        self.change_power(5880, 115)

    def change_setpoint(self, setpoint, delta, ramp = 300):
        """
        Change setpoint using "dynamic setpoint".
        1 - start with setpoint 1100.
        2 - when temperature > ({setpoint} - {delta}) set setpoint to {setpoint}
        """
        self.dhs1100.write_attribute("ramp_time", ramp)

        current_temperature = self.dhs1100.read_attribute("temperature").value

        if setpoint > (current_temperature + delta):
            self.dhs1100.write_attribute("setpoint", 1100)

            while current_temperature < (setpoint - delta) and not self.new_setpoint.is_set():
                time.sleep(1)
                self.lock_communication_furnace.acquire()
                current_temperature = self.dhs1100.read_attribute("temperature").value
                self.lock_communication_furnace.release()

        # If dynamic setpoint has not been interrupted
        if not self.new_setpoint.is_set():
            self.dhs1100.write_attribute("setpoint", setpoint)

    def modify_pid(self, kp = None, ki = None, kd = None):
        
        if kp is not None:
            self.kp = float(kp)
        if ki is not None:
            self.ki = float(ki)
        if kd is not None:
            self.kd = float(kd)

        self.pid.tunings = (self.kp, self.ki, self.kd)
        
        print(self.pid.Kp, self.pid.Ki, self.pid.Kd)


    def start_pid(self, setpoint, integral_term = None):

        self.kp = 1.5
        self.ki = 0.006
        self.kd = 0.5
   
        self.pid = PID(
            Kp = self.kp, 
            Ki = self.ki, 
            Kd = self.kd, 
            setpoint=setpoint
            )
        
        if integral_term is not None:
            self.pid._integral = integral_term
            
        print("PID started!")

        while not self.new_pid.is_set():
            self.lock_communication_furnace.acquire()
            time.sleep(0.005)
            # Read current_temperature
            # try:
            temp = self.dhs1100.read_attribute("temperature").value
            # Calculate the change in power
            control = self.pid(temp)

            # Calculate new power
            new_power = self.current_power + control
            
            if self.debug_pid:
                print(new_power)
                print(self.pid._proportional, self.pid._derivative, self.pid._integral)
            # Change current power to new power
            self.dhs1100.write_attribute("target_power", new_power)

            # Update current power
            if new_power > 0:
                self.current_power = new_power
            else:
                self.current_power = 0
            # except Exception as e:
                # print(e)

            self.lock_communication_furnace.release()
            
            if integral_term is not None:
                self.pid._integral = self.pid._integral * 0.9
            
            time.sleep(1)
            
        print("PID stopped!")
