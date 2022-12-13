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
serialPort = serial.Serial(port='COM8', baudrate=115200, timeout=0, parity=serial.PARITY_EVEN, stopbits=1)
size = 1024
count = 0



# Create the root window and set its title
root = ThemedTk(theme='yaru')
root.title("Robot Control")
root.geometry("400x400+0+0")



# Create 3 empty variables to hold the input values
KpInput = None
KIInput = None
KDInput = None
offset1 = None
offset2 = None

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

    print("Sending Start")
    startOut = "START"
    serialPort.write(bytes(startOut,'utf-8'))

def sendStop():
    print("Sending Stop")
    stopOut = "STOP"
    serialPort.write(bytes(stopOut,'utf-8'))




# Create the text boxes and a button to save the inputs
largerText = font.Font(size=20)
Kp_textbox = ttk.Entry(root)
Kp_textbox.pack(pady=5)

KI_textbox = ttk.Entry(root)
KI_textbox.pack(pady=5)

KD_textbox = ttk.Entry(root)
KD_textbox.pack(pady=5)

button = ttk.Button(root, text="Update PID", command=save_inputs)
button.pack(pady=5)

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
        remChar = ['b', '\'']
        for char in remChar:
            outdata = str(outdata).replace(char,'')
        print(outdata)
        outArray = outdata.split(',')
    root.update()
# First 10 are x hat, next 9 are sensor reading gyro, speed, roll pitch yaw.



