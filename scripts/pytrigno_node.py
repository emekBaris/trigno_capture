#!/usr/bin/env python3
import rospy
from pytrignos.pytrignos import TrignoAdapter
import time
import numpy
import datetime
import pandas as pd
from trigno_capture.msg import trignoIMU, trignoMultiIMU, trignoEMG, trignoMultiEMG
from geometry_msgs.msg import Quaternion
from trigno_msgs.msg import trignoEMG, trignoIMU, trignoMultiIMU, trignoMultiEMG
from PyQt5.QtWidgets import QApplication, QPushButton, QVBoxLayout, QWidget
from PyQt5.QtCore import QCoreApplication
import sys # We need sys so that we can pass argv to QApplication
from scipy import signal

# class GuiApp(QWidget):
#     def __init__(self, trigno_capture):
#         super().__init__()
#         self.trigno_capture = trigno_capture
#         self.initUI()
#
#
#     def initUI(self):
#         self.setWindowTitle('Simple Button GUI')
#         layout = QVBoxLayout(self)
#
#         self.button = QPushButton('Calibrate', self)
#         self.button.clicked.connect(self.onButtonClicked)
#
#         self.button.setFixedSize(300, 200)  # Set to 200px wide by 50px tall
#         self.button.setStyleSheet("font-size: 40px;")
#
#         layout.addWidget(self.button)
#         self.setLayout(layout)
#
#     def onButtonClicked(self):
#         self.trigno_capture.calibrate()
#

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

        for id, [pos, mode] in sensors.items():
            if(mode == 'both'):
                self.emgIDs.append(id)
                self.imuIDs.append(id)
                self.emgLabels.append(pos)
                self.imuLabels.append(pos)
            elif(mode == 'emg'):
                self.emgIDs.append(id)
                self.emgLabels.append(pos)
            elif(mode == 'imu'):
                self.imuIDs.append(id)
                self.imuLabels.append(pos)

        self.emg_sensors.add_sensors(sensors_mode='EMG', sensors_ids = tuple(self.emgIDs),
                                        sensors_labels = tuple(str(self.emgIDs)))
        self.imu_sensors.add_sensors(sensors_mode='ORIENTATION', sensors_ids = tuple(self.imuIDs),
                                        sensors_labels = tuple(str(self.imuIDs)))

        self.imuPublisher = rospy.Publisher('/m1_x/trigno_imu', trignoMultiIMU, queue_size = 10)
        self.emgPublisher = rospy.Publisher('/m1_x/trigno_emg', trignoMultiEMG, queue_size = 10)

        self.empty_counter_imu = 0
        self.empty_counter_emg = 0
        self.call_counter = 0

        self.q0List = [] # initializing list of Quat0
        for imuId in self.imuIDs:
            q = Quaternion()
            q.w = 1
            self.q0List.append(q)


    def advance(self):
        self.emgData = self.emg_sensors.sensors_reading('', False)
        self.imuData = self.imu_sensors.sensors_reading('', False)

        # print(self.emgData)
        # print(self.imuData)

        #instEMG = emgData[emgData['Sensor_id'] == 1]['EMG']

        self.call_counter = self.call_counter + 1
        if len(self.imuData) == 0 and len(self.imuIDs) > 0:
            self.empty_counter_imu = self.empty_counter_imu + 1
            print("Empty IMU", self.empty_counter_imu, "in ", self.call_counter)
        if len(self.emgData) == 0 and len(self.emgIDs) > 0:
            self.empty_counter_emg = self.empty_counter_emg + 1
            print("Empty EMG", self.empty_counter_emg, "in ", self.call_counter)

        if len(self.imuData) != 0 or len(self.emgData) != 0:
            pass
            self.publish()

        #print(emgData)
        #print(emgData.loc[0])
        # print(imuData)

    def downSample(self, data, original_fs, desired_fs):

        # Downsampling
        downsample_factor = int(original_fs / desired_fs)
        downsampled_signal = data[::downsample_factor]

        # print(numpy.size(downsampled_signal))
        return downsampled_signal

    def publish(self):

        multiIMUMsg = trignoMultiIMU()
        multiEMGMsg = trignoMultiEMG()

        if len(self.imuData) > 0:
            for imuId, imuLabel, q0 in zip(self.imuIDs, self.imuLabels, self.q0List):
                imuMsg = trignoIMU()
                imuMsg.imu_id = imuId
                imuMsg.imu_pos = imuLabel
                data = self.imuData
                data['Sensor_id'] = data['Sensor_id'].astype(int)
                imuMsg.start_time = data.index[0]
                imuMsg.q0 = q0

                for _, dataPoint in data[data['Sensor_id'] == imuId].iterrows():
                    quaternion = Quaternion()
                    quaternion.x = dataPoint['qx']
                    quaternion.y = dataPoint['qy']
                    quaternion.z = dataPoint['qz']
                    quaternion.w = dataPoint['qw']
                    imuMsg.q.append(quaternion)

                multiIMUMsg.trigno_imu.append(imuMsg)

            self.imuPublisher.publish(multiIMUMsg)

        if len(self.emgData) > 0:
            for emgId, emgLabel in zip(self.emgIDs, self.emgLabels):
                emgMsg = trignoEMG()
                emgMsg.emg_id = emgId
                emgMsg.emg_pos = emgLabel
                data = self.emgData
                data['Sensor_id'] = data['Sensor_id'].astype(int)
                emgMsg.start_time = data.index[0]
                data = data[data['Sensor_id'] == emgId]
                data = data['EMG'].to_numpy()
                # print("ID: ", emgId)
                # print(numpy.size(data))
                data = self.downSample(data, 2000, 1000);
                for dataPoint in data:
                    emgMsg.emg.append(dataPoint)
                multiEMGMsg.trigno_emg.append(emgMsg)

            self.emgPublisher.publish(multiEMGMsg)

    # def calibrate(self):
    #     self.q0List = []
    #     for imuId in self.imuIDs:
    #         data = self.imuData
    #         #print(data)
    #         data['Sensor_id'] = data['Sensor_id'].astype(int)
    #         data = data[data['Sensor_id'] == imuId]
    #         q = Quaternion()
    #         q.x = data.iloc[0]['qx']
    #         q.y = data.iloc[0]['qy']
    #         q.z = data.iloc[0]['qz']
    #         q.w = data.iloc[0]['qw']
    #         print("sensor ", imuId, "calibration q: ")
    #         # print(numpy.linalg.norm([q.x, q.y, q.z, q.w]))
    #         print(q)
    #         self.q0List.append(q)

    def start(self):
        self.emg_sensors.start_acquisition()
        self.imu_sensors.start_acquisition()
        time.sleep(2)

    def stop(self):
        self.emg_sensors.stop_acquisition()
        self.imu_sensors.stop_acquisition()

if __name__ == "__main__":
    rospy.init_node("trigno_capture")

    right_param = rospy.get_param('/right_side')

    # Dictionary of the sensor with sensor label and mode
    if right_param:
        sensors = {5: ['right_TA', 'emg'], 6: ['right_MG', 'emg'], 7: ['right_SOL', 'emg'], 8: ['right_LG', 'emg']}
    else:
        sensors = {1: ['left_TA', 'emg'], 2: ['left_MG', 'emg'], 3: ['left_SOL', 'emg'], 4: ['left_LG', 'emg']}

    trigno_capture = TrignoCapture(sensors)
    trigno_capture.start()
    rospy.on_shutdown(trigno_capture.stop)
    rospy.loginfo("Sensor node is ready.")
    rospy.loginfo("-- Only publishing raw q values - Calibrate on CORC --")


    # Initialize PyQt application
    # app = QApplication(sys.argv)
    # gui = GuiApp(trigno_capture)
    # gui.show()

    rate = 50
    while not rospy.is_shutdown():
        prevTime = time.perf_counter()
        trigno_capture.advance()
        # print("advance takes: ", (time.perf_counter() - prevTime)*1000, "ms.")

        # Process PyQt events to keep GUI responsive
        # QCoreApplication.processEvents()

        while(time.perf_counter() - prevTime < 1.0/rate):
            pass

        #timePassed = time.perf_counter() - prevTime
        prevTime = time.perf_counter()
        # print("Time diff: ", timePassed*1000)