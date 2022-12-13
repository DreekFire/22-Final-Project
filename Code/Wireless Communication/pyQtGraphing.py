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

        self.graphWidget.setBackground('w')

        redPen = pg.mkPen(color=(255, 0, 0))
        greenPen = pg.mkPen(color=(0, 255 , 0))
        bluePen = pg.mkPen(color=(0, 0, 255))
        self.data_line1 =  self.graphWidget.plot(self.t, self.error, pen=redPen)
        self.data_line2 =  self.graphWidget.plot(self.t, self.integral, pen=greenPen)
        self.data_line3 =  self.graphWidget.plot(self.t, self.derivitive, pen=bluePen)

    def update_plot_data(self, inputArray):

        # self.x = self.x[1:]  # Remove the first y element.
        if len(self.t) > 100:
            self.t = self.t[1:]
        self.t.append(float(inputArray[0])/1000000)  # Add a new value 1 higher than the last.

        # self.y = self.y[1:]  # Remove the first
        if len(self.error) > 100:
            self.error = self.error[1:]
        self.error.append(float(inputArray[1])) 
        if len(self.integral) > 100:
            self.integral = self.integral[1:] 
        self.integral.append(float(inputArray[2])) 
        if len(self.derivitive) > 100:
            self.derivitive = self.derivitive[1:] 
        
        self.derivitive.append(float(inputArray[3])) 

        self.data_line1.setData(self.t, self.error)  # Update the data.
        self.data_line2.setData(self.t, self.integral)
        self.data_line3.setData(self.t, self.derivitive)

serialPort = serial.Serial(port='COM13', baudrate=115200, timeout=0, parity=serial.PARITY_EVEN, stopbits=1)
size = 1024
count = 0


# Create the root window and set its title
root = ThemedTk(theme='yaru')
root.title("Robot Control")
root.geometry("400x700+0+0")
s = ttk.Style()
s.configure('.', font=('Helvetica', 12))
app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
w.show()


# Create the figure and set its size
# fig = plt.figure(figsize=(6, 4))

# Create the axes for the plot
# ax = fig.add_subplot(1, 1, 1)

#     # axi.xaxis.set_major_locator(plt.MaxNLocator(3))


# # Set the axes labels and title
# ax.set_xlabel("Time")
# ax.set_ylabel("Value")
# ax.set_title("Live Graph of Random Data")

# # Create 3 empty lists to hold the random data
# datalist1 = []
# datalist2 = []
# datalist3 = []

# Create 3 empty variables to hold the input values
KpInput = None
KIInput = None
KDInput = None
offset1 = None
offset2 = None


KpInputYaw = None
KIInputYaw = None
KDInputYaw = None

startButton = None
stopButton = None

# This function will be called repeatedly to update the graph with new data
# def update_graph(inputData):
    # Generate some random data and append it to the lists
    # timeArray.append(inputData[0])
    # data1.append(inputData[1])
    # data2.append(inputData[2])
    # data3.append(inputData[3])

    # # Clear the axes and redraw the plot with the new data
    
    # ax.clear()
    # ax.plot(timeArray, data1, label="Error")
    # ax.plot(timeArray, data2, label="Derivitive")
    # ax.plot(timeArray, data3, label="Integral")
    # # ax.set_xlim(len(timeArray) - 100, len(timeArray))
 
    # # ax.legend()
    # # every_nth = round(len(ax.yaxis.get_ticklabels())/4)
    # # for n, label in enumerate(ax.yaxis.get_ticklabels()):
    # #     if n % every_nth != 0:
    # #         label.set_visible(False)

 

    # # Update the canvas to show the new plot
    # fig.canvas.draw()


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
    outString = "PID," + KpInput + "," + KIInput + "," + KDInput + "|"
    serialPort.write(bytes(outString,'utf-8'))


def save_yaw():
    # Set the global variables to the values in the text boxes
    global KpInput
    global KIInput
    global KDInput
    
    KpInputYaw = Kp_textboxYaw.get()
    KIInputYaw = KI_textboxYaw.get()
    KDInputYaw = KD_textboxYaw.get()
    print(f"PID Inputs saved: {KpInputYaw}, {KIInputYaw}, {KDInputYaw}")
    outStringYaw = "YAW," + KpInputYaw + "," + KIInputYaw + "," + KDInputYaw + "|"
    serialPort.write(bytes(outStringYaw,'utf-8'))


def save_offsets():
    global offset1
    global offset2
    offset1 = offset1Textbox.get()
    offset2 = offset2Textbox.get()
    print(f"Offset Inputs saved: {offset1}, {offset2}")
    offsetOut = "OFFSET," + offset1 + "," + offset2 + "|"
    print
    serialPort.write(bytes(offsetOut,'utf-8'))

def sendStart():
    print("Sending 0 PID")
    outString = "PID," + "0" + "," + "0" + "," + "0" + "|"
    serialPort.write(bytes(outString,'utf-8'))

    print("Sending 0 YAW")
    outString = "YAW," + "0" + "," + "0" + "," + "0" + "|"
    serialPort.write(bytes(outString,'utf-8'))

    print("Sending Start")
    startOut = "START"
    serialPort.write(bytes(startOut,'utf-8'))

def sendStop():
    print("Sending Stop")
    stopOut = "STOP"
    serialPort.write(bytes(stopOut,'utf-8'))




# Create the text boxes and a button to save the inputs
largerText = font.Font(size=20)
PIDLabel = ttk.Label(text="PID Values")
PIDLabel.pack(pady=5)
Kp_textbox = ttk.Entry(root)
Kp_textbox.pack(pady=5)

KI_textbox = ttk.Entry(root)
KI_textbox.pack(pady=5)

KD_textbox = ttk.Entry(root)
KD_textbox.pack(pady=5)

button = ttk.Button(root, text="Update PID", command=save_inputs)
button.pack(pady=5)


YawPIDLabel = ttk.Label(text="YAW PID Values")
YawPIDLabel.pack(pady=5)
Kp_textboxYaw = ttk.Entry(root)
Kp_textboxYaw.pack(pady=5)

KI_textboxYaw = ttk.Entry(root)
KI_textboxYaw.pack(pady=5)

KD_textboxYaw = ttk.Entry(root)
KD_textboxYaw.pack(pady=5)

buttonYaw = ttk.Button(root, text="Update Yaw", command=save_yaw)
buttonYaw.pack(pady=5)


OffsetLabel = ttk.Label(text="Offset Values")
OffsetLabel.pack(pady=5)
offset1Textbox = ttk.Entry(root)
offset1Textbox.pack(pady=5)

offset2Textbox = ttk.Entry(root)
offset2Textbox.pack(pady=5)



offsetButton = ttk.Button(root, text="Update Offsets", command=save_offsets)
offsetButton.pack(pady=5)

startButton = ttk.Button(root, text="Start", command=sendStart)
startButton.pack(pady=5)

stopButton = ttk.Button(root, text="Stop", command=sendStop)
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
        if len(outArray) > 1:
            w.update_plot_data(outArray)
    root.update()
# First 10 are x hat, next 9 are sensor reading gyro, speed, roll pitch yaw.



