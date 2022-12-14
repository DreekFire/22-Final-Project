import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox
import matplotlib
from tkinter import *
from tkinter import ttk
from tkinter import font
import time
from ttkthemes import ThemedTk
from PyQt5 import QtWidgets, QtCore
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
import os
from random import randint




class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)
        self.graphWidget.setLabel("bottom", "Time (s)")
        

        self.t = []
        self.error = []  # 100 data points
        self.integral = []
        self.derivitive = []
        self.u1 = []


        self.graphWidget.setBackground('w')

        redPen = pg.mkPen(color=(255, 0, 0), width=2)
        greenPen = pg.mkPen(color=(0, 255 , 0) , width=2)
        bluePen = pg.mkPen(color=(0, 0, 255), width=2)
        orangePen = pg.mkPen(color=(255,163,0), width=2)
        self.data_line1 =  self.graphWidget.plot(self.t, self.error, pen=redPen)
        self.data_line2 =  self.graphWidget.plot(self.t, self.integral, pen=greenPen)
        self.data_line3 =  self.graphWidget.plot(self.t, self.derivitive, pen=bluePen)
        self.data_line4 =  self.graphWidget.plot(self.t, self.u1, pen=orangePen)
        


    def update_plot_data(self, inputArray, printFlag):

        # self.x = self.x[1:]  # Remove the first y element.
        if printFlag:
            if len(self.t) > 500:
                self.t = self.t[1:]
            self.t.append(float(inputArray[0])/1000000)  # Add a new value 1 higher than the last.

            # self.y = self.y[1:]  # Remove the first
            if len(self.error) > 500:
                self.error = self.error[1:]
            self.error.append(float(inputArray[1])) 
            if len(self.integral) > 500:
                self.integral = self.integral[1:] 
            self.integral.append(float(inputArray[2])) 
            if len(self.derivitive) > 500:
                self.derivitive = self.derivitive[1:] 
            self.derivitive.append(float(inputArray[3])) 

            if len(self.u1) > 500:
                self.u1 = self.u1[1:] 
            self.u1.append(float(inputArray[7])*10) 

            self.data_line1.setData(self.t, self.error)  # Update the data.
            self.data_line2.setData(self.t, self.integral)
            self.data_line3.setData(self.t, self.derivitive)
            self.data_line4.setData(self.t, self.u1)

serialPort = serial.Serial(port='COM9', baudrate=115200, timeout=0, parity=serial.PARITY_EVEN, stopbits=1)
size = 1024
count = 0


# Create the root window and set its title
root = ThemedTk(theme='yaru')
root.title("Robot Control")
root.geometry("400x700+0+0")
s = ttk.Style()
s.configure('.', font=('Helvetica', 16))
app = QtWidgets.QApplication(sys.argv)
imuGraph = MainWindow()
imuGraph.setWindowTitle("IMU Graph")

imuGraph.show()

velGraph = MainWindow()
velGraph.setWindowTitle("Velocity Graph")
velGraph.show()



# Create 3 empty variables to hold the input values
KpInput = None
KIInput = None
KDInput = None
offset1 = None
offset2 = None
printFlag = False


KpInputYaw = None
KIInputYaw = None
KDInputYaw = None

startButton = None
stopButton = None



# This function will be called when the user clicks the "Save Inputs" button
def save_inputs():
    # Set the global variables to the values in the text boxes
    global KpInput
    global KIInput
    global KDInput
    


    KpInput = Kp_textbox.get()
    KIInput = KI_textbox.get()
    KDInput = KD_textbox.get()
    print(f"PID Inputs saved: {KpInput}, {KIInput}, {KDInput}")
    # f.write(f"PID Inputs Sent at {time.time()}: {KpInput}, {KIInput}, {KDInput}")
    outString = "PID," + KpInput + "," + KIInput + "," + KDInput + "|"
    serialPort.write(bytes(outString,'utf-8'))
    time.sleep(0.3)
    with open('PID_Log.txt', 'a') as f:
        f.writelines(f"PID Inputs Sent at {time.ctime(time.time())}: {KpInput}, {KIInput}, {KDInput}\n")


def save_yaw():
    # Set the global variables to the values in the text boxes
    global KpInput
    global KIInput
    global KDInput
    
    KpInputYaw = Kp_textboxYaw.get()
    KIInputYaw = KI_textboxYaw.get()
    KDInputYaw = KD_textboxYaw.get()
    print(f"VEL Inputs saved: {KpInputYaw}, {KIInputYaw}, {KDInputYaw}")
    outStringYaw = "VEL," + KpInputYaw + "," + KIInputYaw + "," + KDInputYaw + "|"
    serialPort.write(bytes(outStringYaw,'utf-8'))
    time.sleep(0.3)


def save_offsets():
    global offset1
    global offset2
    offset1 = offset1Textbox.get()
    offset2 = offset2Textbox.get()
    print(f"Offset Inputs saved: {offset1}, {offset2}")
    offsetOut = "OFFSET," + offset1 + "," + offset2 + "|"
    
    serialPort.write(bytes(offsetOut,'utf-8'))
    time.sleep(0.3)

def sendStart():
    global printFlag
    printFlag = True
    print("Sending 0 PID")
    outString = "PID," + "0" + "," + "0" + "," + "0" + "|"
    serialPort.write(bytes(outString,'utf-8'))
    time.sleep(0.3)
    print("Sending 0 VEL")
    outString = "VEL," + "0" + "," + "0" + "," + "0" + "|"
    serialPort.write(bytes(outString,'utf-8'))
    time.sleep(0.3)
    print("Sending Start")
    startOut = "START"
    serialPort.write(bytes(startOut,'utf-8'))
    time.sleep(0.3)

def sendStop():
    global printFlag
    print("Sending Stop")
    printFlag = False
    stopOut = "STOP"
    serialPort.write(bytes(stopOut,'utf-8'))
    time.sleep(0.3)




# Create the text boxes and a button to save the inputs
largerText = font.Font(size=20)
PIDLabel = ttk.Label(text="PID Values")
PIDLabel.pack(pady=5)

kpDefault = StringVar()
kpDefault.set("0.095")

Kp_textbox = ttk.Entry(root, textvariable=kpDefault)
Kp_textbox.pack(pady=5)

kIDefault = StringVar()
kIDefault.set("0.00000485")
KI_textbox = ttk.Entry(root, textvariable=kIDefault)
KI_textbox.pack(pady=5)


kDDefault = StringVar()
kDDefault.set("0.0001")
KD_textbox = ttk.Entry(root, textvariable=kDDefault)
KD_textbox.pack(pady=5)

button = ttk.Button(root, text="Update PID", command=save_inputs)
button.pack(pady=5)


YawPIDLabel = ttk.Label(text="Velocity PID Values")
YawPIDLabel.pack(pady=5)

kpVDefault = StringVar()
kpVDefault.set("4.5")
Kp_textboxYaw = ttk.Entry(root, textvariable=kpVDefault)
Kp_textboxYaw.pack(pady=5)

kIVDefault = StringVar()
kIVDefault.set("0")
KI_textboxYaw = ttk.Entry(root, textvariable=kIVDefault)
KI_textboxYaw.pack(pady=5)


kDVDefault = StringVar()
kDVDefault.set("0.5")
KD_textboxYaw = ttk.Entry(root, textvariable=kDVDefault)
KD_textboxYaw.pack(pady=5)

buttonYaw = ttk.Button(root, text="Update Velocity", command=save_yaw)
buttonYaw.pack(pady=5)


OffsetLabel = ttk.Label(text="Offset Values")
OffsetLabel.pack(pady=5)
offset1Textbox = ttk.Entry(root)
offset1Textbox.pack(pady=5)

offset2Textbox = ttk.Entry(root)
offset2Textbox.pack(pady=5)



offsetButton = ttk.Button(root, text="Update Offsets", command=save_offsets)
offsetButton.pack(pady=5)

startButton = ttk.Button(root, text="Enable", command=sendStart)
startButton.pack(pady=5)

stopButton = ttk.Button(root, text="Disable", command=sendStop)
stopButton.pack(pady=5)


while 1:
    
    data = serialPort.readline(size)

    if data:
        outdata = data.strip()
        remChar = ['b', '\'', ' ']
        for char in remChar:
            outdata = str(outdata).replace(char,'')
        # print(outdata)
        outArray = outdata.split(',')
        print(outArray)
        imuArray = outArray[0:9]
        velArray = outArray[9:17]
        if len(outArray) > 1:
            imuGraph.update_plot_data(imuArray, printFlag)
            velGraph.update_plot_data(velArray, printFlag)
    root.update()
# First 10 are x hat, next 9 are sensor reading gyro, speed, roll pitch yaw.



