####################################################
# CLI program for the control of a TLAG experiment #
####################################################

from queue import Queue
from threading import Thread, Event, Lock
import csv
import os
import time
import random
import numpy as np
import pickle
import math
import logging

from furnace_controller import DHS1100_controller
from logger_instruments import Instruments_logger

import sys
import colorama
from colorama import colorama_text, Fore, Style

import tango

# sys.path.insert(1, 'C:\\Users\\User\\Documents\\TLAG-tool_SAFA')
# sys.path.insert(1, "/homelocal/opbl11/Documents/TLAG/TLAG-tool_SAFA")
# sys.path.insert(1, "/home/jalarruy/Documents/ICMAB/TLAG-tool_SAFA/TLAG-tool_SAFA/")
# from instruments import DHS1100_rtk, GraphixTwo, MOVE1250, eth002Relay, Keithley2450


KEITHLEY_PARAMETERS = {
    "current": 0.0001,
    "buffer_name": "buf_res_acq"    
}


class CLI_experiment_manager(DHS1100_controller, Instruments_logger):


    def start_reading(self, filepath_pressure, filepath_temperature): # filepath_pressure42

        # Run the reader and writer processes in the background...
        self.dhs1100_reader_process = Thread(target=self.read_furnace_data)
        self.dhs1100_writer_process = Thread(target=self.write_from_buffer, kwargs={'file_path': filepath_temperature, 'queue': self.dhs1100_data_queue})
        self.graphix2_reader_process = Thread(target=self.read_graphix2_data)
        self.graphix2_writer_process = Thread(target=self.write_from_buffer, kwargs={'file_path': filepath_pressure, 'queue': self.graphix2_data_queue})
        # Temporary
        # self.graphix242_reader_process = Thread(target=self.read_graphix242_data)
        # self.graphix242_writer_process = Thread(target=self.write_from_buffer, kwargs={'file_path': filepath_pressure42, 'queue': self.graphix242_data_queue})

        self.dhs1100_reader_process.daemon = True
        self.dhs1100_writer_process.daemon = True
        self.graphix2_reader_process.daemon = True
        self.graphix2_writer_process.daemon = True
        # self.graphix242_reader_process.daemon = True
        # self.graphix242_writer_process.daemon = True
        
        self.dhs1100_reader_process.start()
        self.dhs1100_writer_process.start()
        self.graphix2_reader_process.start()
        self.graphix2_writer_process.start()
        # self.graphix242_reader_process.start()
        # self.graphix242_writer_process.start()

    def update_status_relay(self):
        # TODO: Add command relay_states in ETH002 DS
        # self.relay_status = self.eth002Relay.relay_states
        self.relay_status = [0, 0]

        for i, status in enumerate(self.relay_status):
            state = "open" if status else "closed"
            door = 'relay_{}'.format(i+1)
            self.logger.info(self.relayDoor2pump[door] + " is " + state)

    def log_move_position(self, move_position):
        self.logger.info("MOVE position is " + str(move_position))

    def print_status(self):
        self.lock_communication_furnace.acquire()
        print("Current status is: ")
        temp = self.dhs1100.read_attribute("temperature").value
        setpoint = self.dhs1100.read_attribute("setpoint").value
        output = self.dhs1100.read_attribute("output").value
        ramp = self.dhs1100.read_attribute("ramp_time").value
        target_power = self.dhs1100.read_attribute("target_power").value
        # temp_control = self.dhs1100.read_attribute("temp_control").value
        self.lock_communication_furnace.release()

        print("Temperature: ", temp, "ºC")
        print("Setpoint: ", setpoint, "ºC")
        print("Output: ", output, "%")
        print("Ramp: ", ramp, "ºC/s")
        print("Target power: ", target_power)
        
        
        # GRAPHIX TWO status
        self.lock_communication_graphix.acquire()
        pressure1 = self.GraphixTwo.read_attribute("P1").value
        time.sleep(0.005)
        pressure2 = self.GraphixTwo.read_attribute("P2").value
        self.lock_communication_graphix.release()

        print("Pressure 1: ", pressure1)
        print("Pressure 2: ", pressure2)
        
        # Relay status
        self.update_status_relay()

        for i, status in enumerate(self.relay_status):
            state = "open" if status else "closed"
            door = 'relay_{}'.format(i+1)
            print(self.relayDoor2pump[door] + " is " + state)
           
        # MOVE status
        move_position = self.move1250.read_attribute("position").value
        self.log_move_position(move_position)
        
        print("Move position: ", round(move_position, 1), "%")
        
        # Routines status
        
        if self.new_pid.is_set():
            print("PID is OFF")
        else:
            print("PID is ON!")
            
        if self.stop_reading.is_set():
            print("Reading is OFF")
        else:
            print("Reading is ON!")
 
    def print_keithley_status(self):
        print("Keithley2450 measurement", self.keithley2450.make_measurement())
        
    def command_start_resistance(self):
        if not self.stop_reading_keithley.is_set():
            self.stop_reading_keithley.set()
            time.sleep(2)
            
        self.stop_reading_keithley.clear()

        # Reset keithley
        self.keithley2450.reset_device()
        time.sleep(1)
        
        input_parameters = (
            [KEITHLEY_PARAMETERS["current"]], 
            [KEITHLEY_PARAMETERS["buffer_name"]])
        self.t0_resistance = self.keithley2450.start_resistance(input_parameters)
        
        # Start fetching acquisition (low rate reading)
        self.keithley2450_reader_process = Thread(target=self.read_keithley2450_data, args=(KEITHLEY_PARAMETERS["buffer_name"],))
        self.keithley2450_writer_process = Thread(target=self.write_from_buffer, kwargs={'file_path': self.filepath_resistance, 'queue': self.keithley2450_data_queue})

        self.keithley2450_reader_process.daemon = True
        self.keithley2450_writer_process.daemon = True
        
        self.keithley2450_reader_process.start()
        self.keithley2450_writer_process.start()

    def command_stop_resistance(self):
        # Stop fetching acquisition
        self.stop_reading_keithley.set()
        time.sleep(0.5)
        
        # Stop buffer acquisition and read and store buffer data
        self.keithley2450.stop_resistance()
        time.sleep(0.5)
        
        buf = self.keithley2450.read_buffer(KEITHLEY_PARAMETERS["buffer_name"])
        buf_df = self.parse_buffer(buf, self.header_resistance, self.t0_resistance)
        self.save_buf(buf_df, self.filepath_resistance_buffer)

    def command_prepare_resistance(self):
        # Deprecated
        self.keithley2450.prepare_resistance_measurement(0.0001)

    def jump_function(self, x, initial_power, final_power, ramp = 0.5, peak = 3.5):
        initial_power_mod = initial_power - (final_power - initial_power)
        A = final_power - initial_power_mod
        
        soft = 0.75
        factorials = np.array([math.gamma((x_value - 10)) if x_value > 10 else 1 for x_value in x])
        
        return (initial_power_mod + (A/(1+np.exp(-ramp*x))) + ((peak**(x-10))*np.exp(1)/factorials) - np.exp(1-x))

    def jump_function_manual(self, pressure, temperature):
        r"""
        This linear function has been fitted in a jupyter notebook in 'C:\Users\Jordi\Documents\ICMAB\TLAG-tool\tests\Pressure_furnace\fit_power_pressure_temp.ipynb'
        We use a linear function w.r.t the logarithm of the pressure and the logarithm of the temperature
        The function looks like this: power = -385.21014679421853 + 2.98008146*log(pressure) + 65.17184746*log(temperature)]
        """

        power_percentage = -385.21014679421853 + 2.98008146*np.log(pressure) + 65.17184746*np.log(temperature)

        power_miliamperes = power_percentage*11500/100

        return power_miliamperes

    def change_valve_state(self, action, pump):
        # Open or close valves (rotary or turbo) using the Relay.
        self.logger.info("Changing relay {} to {}".format(pump, action))
        
        channel = self.pump2relayDoor[pump]
        if action == "open":
            self.eth002Relay.write_attribute(channel, True)
            # relay_status = self.eth002Relay.relay_states
            # if not relay_status[-channel + 2]: # Check that the other channel is closed before # oppening another valve
            #     self.eth002Relay.single_open(channel)
            # else:
            #     print("BE CAREFUL! Attempted to open {} relay but the other electrovalve was still open. First, close the other electrovalve.".format(pump))
            
        elif action == "close":
            self.eth002Relay.write_attribute(channel, False)

        self.update_status_relay()

    def pressure_jump(self, delay = 2):
        # Closes the low pressure circuit. Waits {delay} seconds. Opens the high pressure circuit.

        initial_time = time.time()

        channel_rotary = self.pump2relayDoor["rotary"]
        channel_turbo = self.pump2relayDoor["turbo"]

        self.eth002Relay.write_attribute(channel_turbo, False)

        self.update_status_relay()

        time.sleep(delay)

        print("Jump at:", time.time())
        if not self.eth002Relay.read_attribute(channel_turbo).value:
            self.eth002Relay.write_attribute(channel_rotary, True)
        else:
            raise Exception("Attempted to open rotary but turbo was still open")
        break

        self.update_status_relay()

    def pressure_jump_move(self, percentage, delay = 2):
        # Closes the low pressure circuit. Waits {delay} seconds. Opens the high pressure circuit.

        initial_time = time.time()

        channel_rotary = self.pump2relayDoor["rotary"]
        channel_turbo = self.pump2relayDoor["turbo"]

        self.eth002Relay.write_attribute(channel_turbo, False)

        self.update_status_relay()

        time.sleep(delay)
        
        print("Jump at:", time.time())
        if not self.eth002Relay.read_attribute(channel_rotary).value:
            
            self.control_move(percentage)
            ################################
            ### Sleep after control move ###
            ################################
            time.sleep(2.3) # HARDCODE, change to 2.3
        else:
            raise Exception("Attempted to open rotary but turbo was still open")
        break

        self.update_status_relay()
        
    def control_move(self, percentage):
        self.logger.info("Changing MOVE position to {}".format(percentage))
        self.move1250.write_attribute("position", percentage)
        self.log_move_position(percentage)

    def pressure2power(self, target_pressure):
        file = r"C:\Users\Jordi\Documents\ICMAB\TLAG-tool\tests\Pressure_furnace\interpolation_pressure_power.pickle"

        with open(file, "rb") as f:
            interpolation_pressure_power = pickle.load(f)

        power = interpolation_pressure_power(target_pressure) /100 * 11500

        return power

    def change_power_jump(self, target_pressure, percentage):

        self.lock_communication_furnace.acquire()

        current_temperature = self.dhs1100.read_attribute("temperature").value
    
        self.lock_communication_furnace.release()

        target_power_stabilization = self.jump_function_manual(target_pressure, current_temperature)

        print("Interpolated power: ", target_power_stabilization)


        ###########
        #   TPS   #
        ###########
        target_power_stabilization = 7750
        # 7935 # Stabilization at 880 degrees and 35 mbar (69%) (only tested, not sure if it is optimal).
        # 7820 # Stabilization at 870 degrees and 35 mbar (68%) (only tested, not sure if it is optimal).
        # 7590 # Stabilization at 850 (and 860) degrees and 35 mbar (66%). Outdated
        # 7590 # Stabilization at 850 (and 860) degrees and 35 mbar (66%). Outdated
        # 7590 # Stabilization at 870 (and 860) degrees and 35 mbar (66%).
        # 7302 # Stabilization at 840 degrees and 35 mbar (63.53%). 
        # 7200 # Stabilization at 810 degrees and 35 mbar (63.53%). 
        # 7383 # Stabilization at 850 degrees and 35 mbar. Not used anymore because it was too low.

        # target_power_peak_overheatting = self.current_power + (target_power_stabilization - self.current_power)*2.3
        
        ############
        #   TPPO   #
        ############
        target_power_peak_overheatting = 9750
        # 9718 # Peak at 880 degrees and 35mbar (only tested, not sure if it is optimal).
        # 9660 # Peak at 870? degrees and jump from 0.005 to 35 mbar 84%
        # 9545 # Peak at 850? degrees and jump from 0.005 to 35 mbar 83%
        # 9500 # Peak at 870 degrees and jump from 0.005 to 35 mbar without previous ramp
        # 9430 # Peak at 840 degrees and jump from 0.005 to 35 mbar (82%) Not really, too much +15
        # 9380 # Peak at 840 degrees and jump from 0.005 to 35 mbar (81.6%) +10
        # 9200 # Peak at 810 degrees and jump from 0.005 to 35 mbar (81%)
        # 9200 # Peak at 840 degrees and jump from 0.005 to 35 mbar (81%) at 250 ml/min
        # 9200 # Peak at 870 degrees and jump from 0.005 to 35 mbar (81%) at 100 ml/min
        # 9050 # Peak at 810 degrees and jump from 0.005 to 35 mbar (81%) at 250 ml/min
        # 9050 # Peak at 840 degrees and jump from 0.005 to 35 mbar (81%) at 100 ml/min or 73%
        
        # power_elbow = self.current_power + (target_power_stabilization - self.current_power)*1.6
        power_elbow = target_power_peak_overheatting - (target_power_peak_overheatting - target_power_stabilization)*0.55
        

        print("Target power overheating:", target_power_peak_overheatting)
        print("Target power stabilization:", target_power_stabilization)

        if current_temperature < 790 or target_pressure < 0.1 or target_power_peak_overheatting > 10000:
            print("Current temperature < 810 or target pressure < 0.1 or target_power > 10000!!")
            return 0
        
        # Do jump
        self.jumping = True
        # self.pressure_jump()
        self.pressure_jump_move(percentage=percentage)

        # time.sleep(0.1)


        ###############
        # D peak ramp #
        ###############
        self.change_power(target_power_peak_overheatting, duration = 1.6) # Time for 500ml and 100% -> 1.6 # Time for 250ml and 100% -> 28
        
        
        #############
        # D plateau #
        #############
        time.sleep(3)

        print("changing power elbow....")
        
        ###########
        # D elbow #
        ###########
        self.change_power(power_elbow, duration = 8)
        
        self.lock_communication_furnace.acquire()
        setpoint_pid = str(self.dhs1100.read_attribute("setpoint").value)
        self.lock_communication_furnace.release()
        
        
        #################
        # Integral term #
        #################
        integral_term=-60
        self.command_start_pid(setpoint=setpoint_pid, integral_term=integral_term)
        self.logger.info("Integral term {}".format(integral_term))
        
        self.jumping = False
        # print("changing power stabilization....")
        # self.change_power(target_power_stabilization, duration = 20)

    def command_change_power(self):
        # Get curent power
        current_power = self.dhs1100.read_attribute("target_power").value

        print("Current power is {}".format(current_power))
        new_power = input("Change power to: ")
        duration = input("Duration of the change: ")
        if new_power.isnumeric() and duration.isnumeric():
            self.logger.info("New power introduced: {}, New duration introduced: {}".format(new_power, duration))
            new_power = float(new_power)
            duration = float(duration)
            power_process = Thread(target=self.change_power, args=(new_power, duration))
            power_process.daemon = True
            power_process.start()
            
    def command_debug_power(self):
        self.debug_power = not self.debug_power

    def command_manual_ramp(self):
        ramp_process = Thread(target=self.do_ramp)
        ramp_process.daemon = True
        ramp_process.start()

    def command_temp_control(self):
        current_temp_control = self.dhs1100.read_attribute("temp_control").value
        print("Current temp_control is {}".format(current_temp_control))
        new_temp_control = input("Change temp_control to (0 - Automatic, 1 - Manual): ")
        new_temp_control = int(new_temp_control)
        
        self.lock_communication_furnace.acquire()
        
        # If we are changing to manual mode, we have to update the target power to the current power. Otherwise the current power will be the old target power.
        if new_temp_control == 1:
            self.current_power = self.dhs1100.read_attribute("output").value/100 * 11500

        self.dhs1100.write_attribute("temp_control", new_temp_control)
        
        self.lock_communication_furnace.release()
        
        if new_temp_control == 1 and current_temp_control == 0:

            # Insist on changing the power for {duration} seconds. Otherwise the power goes down to 0
            
            duration = 12
            t0 = time.time()
            print("Changing temperature control to 'manual'", end = "")
            sys.stdout.flush() # Needed when print(..., end = "") for some terminals
            while time.time() - t0 < duration:
                self.lock_communication_furnace.acquire()
                print(".", end = "")
                sys.stdout.flush() # Needed when print(..., end = "") for some terminals
                try:
                    time.sleep(0.05)
                    self.dhs1100.write_attribute("target_power", self.current_power)
                    time.sleep(0.05)
                except:
                    pass
                self.lock_communication_furnace.release()
                time.sleep(0.005)
                
            print("") # Print a new line
                
            self.lock_communication_furnace.acquire()
            setpoint_pid = str(self.dhs1100.read_attribute("setpoint").value)
            self.lock_communication_furnace.release()
            self.command_start_pid(setpoint=setpoint_pid)

    def command_dynamic_setpoint(self):
        setpoint = input("Introduce setpoint:")
        delta = input("Introduce delta:")
        ramp_speed = input("Introduce ramp speed:")
        if setpoint.isnumeric() and delta.isnumeric():
            self.logger.info("New setpoint introduced: {}, New delta introduced: {}, New ramp introduced: {}".format(setpoint, delta, ramp_speed))
            if not self.new_setpoint.is_set():
                self.new_setpoint.set()
                time.sleep(2)
                self.new_setpoint.clear()
            setpoint_process = Thread(target=self.change_setpoint, args=(int(setpoint), int(delta), int(ramp_speed)))
            setpoint_process.daemon = True
            setpoint_process.start()

    def command_start_pid(self, setpoint=None, integral_term = None):
        
        if setpoint is None:
            setpoint = input("Introduce setpoint: ")
        if setpoint.isnumeric():
            if not self.new_pid.is_set():
                self.new_pid.set()
                time.sleep(1.5)
            self.new_pid.clear()
            pid_process = Thread(target=self.start_pid, args=(int(setpoint), integral_term))
            pid_process.daemon = True
            pid_process.start()

    def command_stop_pid(self):
        self.new_pid.set()
        
    def command_debug_pid(self):
        self.debug_pid = not self.debug_pid

    def command_modify_pid(self):
        kp = input("Current Kp is {}, introduce new Kp: ".format(self.kp))
        ki = input("Current Ki is {}, introduce new Ki: ".format(self.ki))
        kd = input("Current Kd is {}, introduce new Kd: ".format(self.kd))

        if kp == '':
            kp = None
        if ki == '':
            ki = None
        if kd == '':
            kd = None

        self.modify_pid(kp, ki, kd)
        
    def command_mv_move(self):
        percentage = float(input("Introduce new percentage: "))
        
        self.control_move(percentage)

    def command_mvr_move(self):
        # MOVE status
        move_position = self.move1250.read_attribute("position").value
        
        print("Current move position: ", round(move_position, 1), "%")
        rel_mov = float(input("Introduce relative movement: "))
        
        new_percentage = move_position + rel_mov
        
        self.control_move(new_percentage)

    def command_do_jump(self):
        # jump_mbar = input("Introduce the target pressure (in mbar): ")
        jump_mbar = 35
        self.logger.info("Ready to change pressure to {} mbar".format(jump_mbar))
        
        move_percentage = input("Introduce the target percentage of the MOVE (in %): ")
        self.logger.info("Ready to open MOVE to {} %".format(move_percentage))
        
        # Stop_pid
        self.command_stop_pid()

        jump_process = Thread(target=self.change_power_jump, args=(float(jump_mbar), float(move_percentage)))
        jump_process.daemon = True
        jump_process.start()

    def command_cooling(self):
        self.lock_communication_furnace.acquire()
        current_temp_control = self.dhs1100.read_attribute("temp_control").value
        self.lock_communication_furnace.release()
        
        # Stop_pid
        self.command_stop_pid()
        
        self.change_setpoint(setpoint=25, delta=25)
        
        if current_temp_control == 1:
            self.change_power(target_power=0, duration=100)

    def command_start_reading(self):
        with open(self.communication_file, 'a') as f:
            f.write("start_acquisition" + "\n")

    def command_stop_reading(self):
        with open(self.communication_file, 'a') as f:
            f.write("stop_acquisition" + "\n")

    def read_last_line(self):
        with open(self.communication_file, 'r') as f:
            for line in f.readlines():
                pass
            last_line = line.strip()

        return last_line

    def read_last_prefix(self):

        last_prefix = None

        with open(self.communication_file, 'r') as f:
            for line in f.readlines():
                if line.split()[0] == 'new_prefix':
                    last_prefix = line.split()[1]

        return last_prefix

    def get_index(self, file):
        return file.split('.')[0].split('_')[-1]

    def find_last_index(self):

        current_pressure_files = os.listdir(self.path_pressure)
        current_temperature_files = os.listdir(self.path_temperature)

        max_index_pressure = max([int(self.get_index(file)) if self.get_index(file).isnumeric() else 0 for file in current_pressure_files])
        max_index_temperature = max([int(self.get_index(file)) if self.get_index(file).isnumeric() else 0 for file in current_temperature_files])

        current_max_index = max(max_index_pressure, max_index_temperature)

        return current_max_index

    def create_and_write_header(self, file_path, header):
        with open(file_path, "w", newline='') as f:
            csvwriter = csv.writer(f)

            csvwriter.writerow(header)

    def load_logger(self, current_prefix):
        # Create logger
        self.logger = logging.getLogger(__name__)
        self.logger_path = self.path_logger + "/log_" + str(current_prefix) + ".log"
        # Remove all handlers associated with the root logger object.
        for handler in logging.root.handlers[:]:
            logging.root.removeHandler(handler)
        logging.basicConfig(
            filename=self.logger_path,
            level=logging.INFO,
            format='%(levelname)s-%(asctime)s-%(created)s-%(message)s',
            datefmt='%Y/%m/%d %H:%M:%S'
        )
        # self.logger.addHandler(file_handler)

    def server_reader(self):

        self.prepared_to_start = True

        while 1:
            time.sleep(1)
            # read_last_line of communication file
            last_line = self.read_last_line()
            # print(last_line)
            if self.prepared_to_start and last_line == "start_acquisition":
                self.prepared_to_start = False

                self.logger.info("'start_acquisition' command from sardana received")

                # Read last prefix:
                last_prefix = self.read_last_prefix()

                # Check if prefix exists
                filename_pressure = "pressure_{}.csv"
                # filename_pressure42 = "pressure42_{}.csv"
                filename_temperature = "temperature_{}.csv"
                filename_resistance = "resistance_{}.csv"
                filename_resistance_buffer = "buffer_resistance_{}.csv"
                existing_file_pressure = filename_pressure.format(last_prefix) in os.listdir(self.path_pressure)
                existing_file_temperature = filename_temperature.format(last_prefix) in os.listdir(self.path_temperature)
                if last_prefix is not None and not (existing_file_pressure or existing_file_temperature):
                    current_prefix = last_prefix
                else:
                    current_prefix = self.find_last_index() + 1

                print("Experiment name = ", current_prefix)
                self.logger.info("New prefix found: {}".format(current_prefix))

                # create files with headers
                # Double check if they exist
                filepath_pressure = self.path_pressure + "/" + filename_pressure.format(current_prefix)
                # temporary
                # filepath_pressure42 = self.path_pressure42 + "/" + filename_pressure42.format(current_prefix)
                filepath_temperature = self.path_temperature + "/" + filename_temperature.format(current_prefix)
                self.filepath_resistance = self.path_resistance + "/" + filename_resistance.format(current_prefix)
                self.filepath_resistance_buffer = self.path_resistance_buffer + "/" + filename_resistance_buffer.format(current_prefix)

                existing_file_pressure = filename_pressure.format(current_prefix) in os.listdir(self.path_pressure)
                existing_file_temperature = filename_temperature.format(current_prefix) in os.listdir(self.path_temperature)
                existing_file_resistance = filename_resistance.format(current_prefix) in os.listdir(self.path_resistance)

                if not (existing_file_pressure or existing_file_temperature):
                    self.create_and_write_header(filepath_pressure, self.header_pressure)
                    self.create_and_write_header(filepath_temperature, self.header_temperature)
                    self.create_and_write_header(self.filepath_resistance, self.header_resistance)
                    # self.create_and_write_header(filepath_pressure42, self.header_pressure)
                    
                else:
                    raise Exception("Sorry, file for temperature and pressure already exists")

                # Create logger
                self.load_logger(current_prefix)

                self.stop_reading.clear()
                time.sleep(1)
                self.start_reading(filepath_pressure, filepath_temperature) # , filepath_pressure42

            elif not self.prepared_to_start and last_line == "stop_acquisition":
                self.logger.info("'stop_acquisition' command from sardana received")
                self.stop_reading.set()

                self.prepared_to_start = True

    def __init__(self):
        self.dhs1100 = tango.DeviceProxy("tlag/ex/dhs1100-01") # DHS1100_rtk()
        self.GraphixTwo = tango.DeviceProxy("tlag/vc/vgct-01") #GraphixTwo()
        # self.GraphixTwo42 = tango.DeviceProxy("sys/tg_test/1") #GraphixTwo("rfc2217://192.168.127.254:4006")
        self.eth002Relay = tango.DeviceProxy("tlag/ex/eth002-01") #eth002Relay()
        self.move1250 = tango.DeviceProxy("tlag/ex/move1250-01") #MOVE1250()
        try:
            self.keithley2450 = tango.DeviceProxy("tlag/ex/keithley2450-01") #Keithley2450()
        except:
            print("Keithley2450 not connected")

        self.dhs1100_data_queue = Queue()
        self.graphix2_data_queue = Queue()
        # temporary
        # self.graphix242_data_queue = Queue()
        self.keithley2450_data_queue = Queue()
        self.stop_reading = Event()
        self.stop_reading.set()
        self.stop_reading_keithley = Event()
        self.new_setpoint = Event()
        self.new_pid = Event()
        self.new_pid.set()
        self.lock_communication_furnace = Lock()
        self.lock_communication_graphix = Lock()
        self.debug_pid = False
        self.debug_power = False
        self.jumping = False

        # self.path_pressure = "/homelocal/opbl11/Documents/TLAG/TLAG-tool_SAFA/experiment_manager/results/pressure"
        # self.path_pressure42 = "/homelocal/opbl11/Documents/TLAG/TLAG-tool_SAFA/experiment_manager/results/pressure42" # temporary
        # self.path_temperature = "/homelocal/opbl11/Documents/TLAG/TLAG-tool_SAFA/experiment_manager/results/temperature"
        # self.path_resistance = "/homelocal/opbl11/Documents/TLAG/TLAG-tool_SAFA/experiment_manager/results/resistance"
        # self.path_resistance_buffer = "/homelocal/opbl11/Documents/TLAG/TLAG-tool_SAFA/experiment_manager/results/buffer_resistance"
        # self.path_logger = "/homelocal/opbl11/Documents/TLAG/TLAG-tool_SAFA/experiment_manager/results/logs"
        # self.communication_file = "/homelocal/opbl11/Documents/TLAG/TLAG-tool_SAFA/experiment_manager/sardana_communication.txt"

        cwd = os.path.dirname(__file__)
        print(len(os.path.dirname(__file__)))
        self.path_results = "/homelocal/opbl11/Documents/TLAG/tlag_manager/results/"

        self.path_pressure = os.path.join(self.path_results, "pressure")
        self.path_pressure42 = os.path.join(self.path_results, "pressure42") # temporary
        self.path_temperature = os.path.join(self.path_results, "temperature")
        self.path_resistance = os.path.join(self.path_results, "resistance")
        self.path_resistance_buffer = os.path.join(self.path_results, "buffer_resistance")
        self.path_logger = os.path.join(self.path_results, "logs")
        self.communication_file = "/homelocal/opbl11/Documents/TLAG/tlag_manager/sardana_communication.txt"

        #Create generig logger
        self.load_logger("global")

        self.header_pressure = ["Time", "Pressure1", "Pressure2"]
        self.header_temperature = ["Time", "Temperature", "Setpoint", "Output", "Ramp", "Temp_control"] # target_power
        self.header_resistance = ["Time", "Measurement"]

        # Get current power of furnace in case we have to do ramps
        self.current_power = self.dhs1100.read_attribute("target_power").value

        # Define which electrovalve is controlled by each channel of the Relay
        self.pump2relayDoor = {
            'rotary' : 'relay_2',
            'turbo' : 'relay_1'
        }

        self.relayDoor2pump = {value : key for key, value in self.pump2relayDoor.items()}

        self.get_started()

    def get_started(self):
        # start_server_reader
        server_reader_process = Thread(target=self.server_reader)
        server_reader_process.daemon = True
        server_reader_process.start()
        
        print("WARNING: This script needs to be updated with the new content of the scrip that does not use tango: /tlag_manager/scripts/cli_experiment.py")


        while 1:
            with colorama_text():
                command = input(Style.DIM + "Introduce new command \n>>" + Style.RESET_ALL + Fore.CYAN)
            self.logger.info("Command introduced: {}".format(command))
            
            print(Style.RESET_ALL, end = "")
            sys.stdout.flush()

            if command == "status":
                self.print_status()

            elif command == "change power":
                self.command_change_power()
                
            elif command == "debug power":
                self.command_debug_power()

            elif command == "manual ramp":
                self.command_manual_ramp()
                
            elif command == "temp control":
                self.command_temp_control()

            elif command == "dynamic setpoint":
                self.command_dynamic_setpoint()

            elif command == "cooling":
                self.command_cooling()

            elif command == "start pid":
                self.command_start_pid()

            elif command == "stop pid":
                self.command_stop_pid()
                
            elif command == "debug pid":
                self.command_debug_pid()

            elif command == "modify pid":
                self.command_modify_pid()
                
            elif command == "mv move":
                self.command_mv_move()
                
            elif command == "mvr move":
                self.command_mvr_move()

            elif command == "jump":
                self.command_do_jump()
                
            elif command == "single jump":
                self.pressure_jump()
                
            elif command.split(' ')[0] in ["open", "close"] and \
                 command.split(' ')[1] in ["rotary", "turbo"] and \
                 len(command.split(' ')) == 2:
                self.change_valve_state(*command.split(' '))
                
            elif command == "keithley status":
                self.print_keithley_status()
                
            elif command == "start resistance":
                self.command_start_resistance()
                
            elif command == "stop resistance":
                self.command_stop_resistance()
                
            elif command == "prepare resistance":
                self.command_prepare_resistance()
                
            elif command == "start reading":
                self.command_start_reading()

            elif command == "stop reading":
                self.command_stop_reading()

            elif command == "exit":
                self.command_stop_reading()
                self.stop_reading.set()
                break
            
            elif command == "":
                pass
            
            else:
                print(Fore.YELLOW + "WARNING: Command not found!")
                
            print(Style.RESET_ALL, end="")

if __name__ == "__main__":
    a = CLI_experiment_manager()
