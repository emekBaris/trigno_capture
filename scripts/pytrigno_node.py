#!/usr/bin/env python3
import rospy
from pytrignos.pytrignos import TrignoAdapter
import time
import numpy
import datetime
import pandas as pd

class TrignoCapture:
    def __init__(self, sensors):
        # self.trigno_sensors = TrignoAdapter()

        self.emg_sensors = TrignoAdapter()
        self.imu_sensors = TrignoAdapter()

        self.nSensor = len(sensors)

        emgIDs = []
        emgLabels = []
        imuIDs = []
        imuLabels = []

        for label, mode in sensors.items():
            if(mode == 'both'):
                emgIDs.append(int(label))
                imuIDs.append(int(label))
                emgLabels.append(label)
                imuLabels.append(label)
            elif(mode == 'emg'):
                emgIDs.append(int(label))
                emgLabels.append(label)
            elif(mode == 'imu'):
                imuIDs.append(int(label))
                imuLabels.append(label)

        self.emg_sensors.add_sensors(sensors_mode='EMG', sensors_ids = tuple(emgIDs),
                                        sensors_labels = tuple(emgLabels))
        self.imu_sensors.add_sensors(sensors_mode='ORIENTATION', sensors_ids = tuple(imuIDs),
                                        sensors_labels = tuple(imuLabels))

        self.empty_counter_imu = 0
        self.empty_counter_emg = 0
        self.call_counter = 0
    def advance(self):

        emgData = self.emg_sensors.sensors_reading('', False)
        imuData = self.imu_sensors.sensors_reading('', False)

        #instEMG = emgData[emgData['Sensor_id'] == 1]['EMG']

        self.call_counter = self.call_counter + 1
        if len(imuData) == 0:
            self.empty_counter_imu = self.empty_counter_imu + 1
            print("Empty IMU", self.empty_counter_imu, "in ", self.call_counter)
        if len(emgData) == 0:
            self.empty_counter_emg = self.empty_counter_emg + 1
            print("Empty EMG", self.empty_counter_emg, "in ", self.call_counter)

        # if len(imuData) != 0:
        #     #a = 1
        #     print("Time datetime: ", datetime.datetime.now())
        #     print(imuData)
        #     print("---------------")
            #print(imuData.tail(1))

        #print(emgData)
        #print(emgData.loc[0])
        # print(imuData)

    def start(self):
        self.emg_sensors.start_acquisition()
        self.imu_sensors.start_acquisition()
        time.sleep(2)
    def stop(self):
        self.emg_sensors.stop_acquisition()
        self.imu_sensors.stop_acquisition()

if __name__ == "__main__":
    rospy.init_node("trigno_capture")

    # Dictionary of the sensor with sensor label and mode
    # sensors = {'1': 'both', '2': 'both', '3': 'both', '4': 'both', '5': 'both', '6': 'both', '7': 'both', '8': 'both'}

    # sensors = {'1': 'both', '2': 'both', '3': 'both', '4': 'both'}

    sensors = {'1': 'both', '2': 'both'}

    trigno_capture = TrignoCapture(sensors)
    trigno_capture.start()
    rospy.on_shutdown(trigno_capture.stop)
    rospy.loginfo("Sensor node is ready.")

    rate = 100
    while not rospy.is_shutdown():
        prevTime = time.perf_counter()
        trigno_capture.advance()
        # print("advance takes: ", (time.perf_counter() - prevTime)*1000, "ms.")

        while(time.perf_counter() - prevTime < 1.0/rate):
            pass

        timePassed = time.perf_counter() - prevTime
        prevTime = time.perf_counter()
        # print("Time diff: ", timePassed*1000)


