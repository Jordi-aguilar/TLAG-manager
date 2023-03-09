from queue import Queue
from threading import Thread, Event, Lock
import csv
import os
import time
import random
import pandas as pd
import numpy as np
import pickle
import math

class Instruments_logger:

    def get_attribute_time(self, attr):
        return time.time()
        # return attr.time.tv_sec # + attr.time.tv_usec/100000

    def write_line(self, file, values):
        # write line in terminal
        # header = ["Temperature", "Setpoint", "Output", "Ramp"]
        # write line in file
        with open(file, "a", newline='') as f:
            csvwriter = csv.writer(f)

            csvwriter.writerow(values)

    def write_from_buffer(self, file_path, queue):
        # global sensor_data_queue
        while 1:
            data = queue.get(block=True)
            if data is None:
                break
            else:
                self.write_line(file_path, data)

    def read_furnace_data(self):

        sleep_time = 0.007

        while not self.stop_reading.is_set():

            self.lock_communication_furnace.acquire()
            
            try:
                if not self.jumping:
                    temp_attribute = self.dhs1100.read_attribute("temperature")
                    temp = temp_attribute.value
                    current_time = self.get_attribute_time(temp_attribute)
                    temp = self.dhs1100.read_attribute("temperature").value
                    time.sleep(sleep_time)
                    output = self.dhs1100.read_attribute("output").value
                    time.sleep(sleep_time)
                    setpoint = self.dhs1100.read_attribute("setpoint").value
                    time.sleep(sleep_time)
                    ramp = self.dhs1100.read_attribute("ramp_time").value
                    time.sleep(sleep_time)
                    # target_power = self.dhs1100.read_attribute("target_power").value
                    # time.sleep(sleep_time)
                    temp_control = self.dhs1100.read_attribute("temp_control").value
                    time.sleep(sleep_time)
                else:
                    temp_attribute = self.dhs1100.read_attribute("temperature")
                    temp = temp_attribute.value
                    current_time = self.get_attribute_time(temp_attribute)
                    time.sleep(sleep_time)
                    output = self.dhs1100.read_attribute("output").value
                    time.sleep(sleep_time)
                    setpoint = -1
                    ramp = -1
                    temp_control = -1
            except:
                current_time = time.time()
                ramp = -2
                print("EXCEPTION HAS OCURRED WHEN READING DHS1100")
                time.sleep(sleep_time)

            self.lock_communication_furnace.release()

            # data = [3, 2, 1]
            data = [current_time, temp, setpoint, output, ramp, temp_control] # target_power

            self.dhs1100_data_queue.put(data)

            time.sleep(0.15)

        self.dhs1100_data_queue.put(None)  # None means we're done.

    def read_graphix2_data(self):
        sleep_time = 0.05

        while not self.stop_reading.is_set():

            self.lock_communication_graphix.acquire()
            # try:
            p1_attribute = self.GraphixTwo.read_attribute("P1")
            pressure1 = p1_attribute.value
            current_time = self.get_attribute_time(p1_attribute)
            time.sleep(sleep_time)
            pressure2 = self.GraphixTwo.read_attribute("P2").value
            time.sleep(sleep_time)
            # except:
            #     pressure1=-1
            #     print("EXCEPTION HAS OCURRED WHEN READING GRAPHIXTWO")
            #     time.sleep(sleep_time)
                
            self.lock_communication_graphix.release()

            # data = [1, 2, 3]
            data = [current_time, pressure1, pressure2]

            self.graphix2_data_queue.put(data)

            time.sleep(sleep_time)

        self.graphix2_data_queue.put(None)  # None means we're done.
        
        
    # Temporary
    def read_graphix242_data(self):
        sleep_time = 0.05

        while not self.stop_reading.is_set():

            self.lock_communication_graphix.acquire()
            current_time = time.time()
            try:
                p1_attribute = self.GraphixTwo.read_attribute("P1")
                pressure1 = p1_attribute.value
                current_time = self.get_attribute_time(p1_attribute)
                time.sleep(sleep_time)
                pressure2 = self.GraphixTwo.read_attribute("P2").value
                time.sleep(sleep_time)
            except Exception as e:
                print(e)
                pressure1=-1
                pressure2=-1
                print("EXCEPTION HAS OCURRED WHEN READING GRAPHIXTWO")
                time.sleep(sleep_time)
                
            self.lock_communication_graphix.release()

            # data = [1, 2, 3]
            data = [current_time, pressure1, pressure2]

            self.graphix242_data_queue.put(data)

            time.sleep(sleep_time)

        self.graphix242_data_queue.put(None)  # None means we're done.
        
        
    def read_keithley2450_data(self, buffer_name):
        sleep_time = 0.05

        while not self.stop_reading_keithley.is_set():
            
            current_time = time.time()
            # try:
            resistance_measurement = self.keithley2450.fetch_buffer(buffer_name)
            
            resistance_measurement = float(resistance_measurement)
                # time.sleep(sleep_time)
            
            # except:
                # print("EXCEPTION HAS OCURRED WHEN READING KEITHLEY2450")
                # time.sleep(sleep_time)
            
            t = time.time()
            # print("read in:, ", t-current_time)

            # data = [1, 2, 3]
            data = [current_time, resistance_measurement]

            self.keithley2450_data_queue.put(data)
            
            t1 = time.time()
            # print("queued in:, ", t1-t)

            time.sleep(sleep_time)

        self.graphix2_data_queue.put(None)  # None means we're done.
        
    
    def parse_buffer(self, buf, header, t0_resistance = 0):
        buf = buf.split(',')
        print("buf l: ", len(buf))
        mes = buf[0::2]
        rel_time = buf[1::2]
        
        mes = np.array(mes).astype(np.float)
        time = np.array(rel_time).astype(np.float) + t0_resistance
        
        print("time l: ", len(time))
        print("mes l: ", len(mes))

        df = pd.DataFrame({
            header[0] : time,
            header[1] : mes
        })
        
        return df
    
    def save_buf(self, buf, path):
        buf.to_csv(path, index = False)
        



    

    
