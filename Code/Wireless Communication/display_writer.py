import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox
import matplotlib
from tkinter import *
from tkinter import font
import time
serialPort = serial.Serial(port='COM8', baudrate=115200, timeout=0, parity=serial.PARITY_EVEN, stopbits=1)
size = 1024
count = 0
# use ggplot style for more sophisticated visuals
matplotlib.use("Qt5Agg")
plt.style.use('ggplot')

data1 = []
data2 = []
data3 = []
timeArray = []
lineArg = [data1, data2, data3]

dataMax = [0,0,0]
dataMin = [0,0,0]


# Create the root window and set its title
root = Tk()
root.title("Live Graph")
root.geometry("600x600+0+0")

# Create the figure and set its size
fig = plt.figure(figsize=(6, 4))

# Create the axes for the plot
ax = fig.add_subplot(1, 1, 1)

    # axi.xaxis.set_major_locator(plt.MaxNLocator(3))


# Set the axes labels and title
ax.set_xlabel("Time")
ax.set_ylabel("Value")
ax.set_title("Live Graph of Random Data")

# Create 3 empty lists to hold the random data
datalist1 = []
datalist2 = []
datalist3 = []

# Create 3 empty variables to hold the input values
input1 = None
input2 = None
input3 = None

# This function will be called repeatedly to update the graph with new data
def update_graph(inputData):
    # Generate some random data and append it to the lists
    timeArray.append(inputData[0])
    data1.append(inputData[1])
    data2.append(inputData[2])
    data3.append(inputData[3])

    # Clear the axes and redraw the plot with the new data
    
    ax.clear()
    ax.plot(timeArray, data1, label="Error")
    ax.plot(timeArray, data2, label="Derivitive")
    ax.plot(timeArray, data3, label="Integral")
    # ax.set_xlim(len(timeArray) - 100, len(timeArray))
 
    # ax.legend()
    # every_nth = round(len(ax.yaxis.get_ticklabels())/4)
    # for n, label in enumerate(ax.yaxis.get_ticklabels()):
    #     if n % every_nth != 0:
    #         label.set_visible(False)

 

    # Update the canvas to show the new plot
    fig.canvas.draw()


# This function will be called when the user clicks the "Save Inputs" button
def save_inputs():
    # Set the global variables to the values in the text boxes
    global input1
    global input2
    global input3
    
    input1 = textbox1.get()
    input2 = textbox2.get()
    input3 = textbox3.get()
    print(f"Inputs saved: {input1}, {input2}, {input3}")
    outString = input1 + "," + input2 + "," + input3 + "|"
    serialPort.write(bytes(outString,'utf-8'))


# Create the text boxes and a button to save the inputs
largerText = font.Font(size=20)
textbox1 = Entry(root, font = largerText)
textbox1.pack()
textbox2 = Entry(root, font = largerText)
textbox2.pack()
textbox3 = Entry(root, font = largerText)
textbox3.pack()
button = Button(root, text="Update Values", command=save_inputs, font=largerText)
button.pack()



# Show the plot
plt.ion()
plt.show()


while 1:
    data = serialPort.readline(size)

    if data:
        outdata = data.strip()
        remChar = ['b', '\'']
        for char in remChar:
            outdata = str(outdata).replace(char,'')
        print(outdata)
        outArray = outdata.split(',')
        if (len(outArray) > 2):
            update_graph(outArray)
            #outString = input1 + "," + input2 + "," + input3 + "|"
        # TODO change to outString for writing
        root.update()
# First 10 are x hat, next 9 are sensor reading gyro, speed, roll pitch yaw.



