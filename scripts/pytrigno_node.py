#!/usr/bin/env python3
import rospy
from pytrignos.pytrignos import TrignoAdapter
import time
import numpy
import datetime
import pandas as pd
from trigno_capture.msg import trignoIMU, trignoMultiIMU, trignoEMG, trignoMultiEMG
from geometry_msgs.msg import Quaternion

class TrignoCapture:
    def __init__(self, sensors):
        # self.trigno_sensors = TrignoAdapter()

        self.emg_sensors = TrignoAdapter()
        self.imu_sensors = TrignoAdapter()

        self.nSensor = len(sensors)

        self.emgIDs = []
        self.emgLabels = []
        self.imuIDs = []
        self.imuLabels = []

        for label, mode in sensors.items():
            if(mode == 'both'):
                self.emgIDs.append(int(label))
                self.imuIDs.append(int(label))
                self.emgLabels.append(label)
                self.imuLabels.append(label)
            elif(mode == 'emg'):
                self.emgIDs.append(int(label))
                self.emgLabels.append(label)
            elif(mode == 'imu'):
                self.imuIDs.append(int(label))
                self.imuLabels.append(label)

        self.emg_sensors.add_sensors(sensors_mode='EMG', sensors_ids = tuple(self.emgIDs),
                                        sensors_labels = tuple(self.emgLabels))
        self.imu_sensors.add_sensors(sensors_mode='ORIENTATION', sensors_ids = tuple(self.imuIDs),
                                        sensors_labels = tuple(self.imuLabels))

        self.imuPublisher = rospy.Publisher('trigno_imu', trignoMultiIMU, queue_size = 10)
        self.emgPublisher = rospy.Publisher('trigno_emg', trignoMultiEMG, queue_size = 10)

        self.empty_counter_imu = 0
        self.empty_counter_emg = 0
        self.call_counter = 0
    def advance(self):
        self.emgData = self.emg_sensors.sensors_reading('', False)
        self.imuData = self.imu_sensors.sensors_reading('', False)

        # print(self.emgData)
        # print(self.imuData)

        #instEMG = emgData[emgData['Sensor_id'] == 1]['EMG']

        self.call_counter = self.call_counter + 1
        if len(self.imuData) == 0:
            self.empty_counter_imu = self.empty_counter_imu + 1
            print("Empty IMU", self.empty_counter_imu, "in ", self.call_counter)
        if len(self.emgData) == 0:
            self.empty_counter_emg = self.empty_counter_emg + 1
            print("Empty EMG", self.empty_counter_emg, "in ", self.call_counter)

        if len(self.imuData) != 0:
            # pass
            self.publish()

        #print(emgData)
        #print(emgData.loc[0])
        # print(imuData)
    def publish(self):

        multiIMUMsg = trignoMultiIMU()
        multiEMGMsg = trignoMultiEMG()



        for sensorId in list(set(self.emgIDs) | set(self.imuIDs)):
            imuMsg = trignoIMU()
            emgMsg = trignoEMG()

            imuMsg.imu_id = sensorId
            emgMsg.imu_id = sensorId
            for dataIndex, data in enumerate([self.imuData, self.emgData]):
                data['Sensor_id'] = data['Sensor_id'].astype(int)

                if(dataIndex == 0): # imu
                    imuMsg.start_time = data.index[0]
                elif(dataIndex == 1): # emg
                    emgMsg.start_time = data.index[0]
                for _, dataPoint in data[data['Sensor_id'] == sensorId].iterrows():
                    if (dataIndex == 0):  # imu
                        quaternion = Quaternion()
                        quaternion.x = dataPoint['qx']
                        quaternion.y = dataPoint['qy']
                        quaternion.z = dataPoint['qz']
                        quaternion.w = dataPoint['qw']
                        imuMsg.q.append(quaternion)
                    elif (dataIndex == 1):  # emg
                        emg = dataPoint['EMG']
                        emgMsg.emg.append(emg)

            multiIMUMsg.trigno_imu.append(imuMsg)
            multiEMGMsg.trigno_emg.append(emgMsg)

        self.imuPublisher.publish(multiIMUMsg)
        self.emgPublisher.publish(multiEMGMsg)

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

    rate = 50
    while not rospy.is_shutdown():
        prevTime = time.perf_counter()
        trigno_capture.advance()
        # print("advance takes: ", (time.perf_counter() - prevTime)*1000, "ms.")

        while(time.perf_counter() - prevTime < 1.0/rate):
            pass

        timePassed = time.perf_counter() - prevTime
        prevTime = time.perf_counter()
        # print("Time diff: ", timePassed*1000)


