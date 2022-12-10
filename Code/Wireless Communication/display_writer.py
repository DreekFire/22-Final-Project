import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox
import matplotlib
serialPort = serial.Serial(port='COM5', baudrate=115200, timeout=0, parity=serial.PARITY_EVEN, stopbits=1)
size = 1024
count = 0
# use ggplot style for more sophisticated visuals
matplotlib.use("Qt5Agg")
plt.style.use('ggplot')

data1 = []
data2 = []
data3 = []
lineArg = [data1, data2, data3]
ax = ""
fig = ""
dataMax = [0,0,0]
dataMin = [0,0,0]
sizeArg = 100
x_vec = np.linspace(0,1,sizeArg+1)[0:-1]
y_vec = np.zeros(sizeArg)
y_vec2 = np.zeros(sizeArg)
y_vec3 = np.zeros(sizeArg)
# Secondary plot
plt.ion()
fig2 = plt.figure(figsize=(13,6))
ax2 = fig2.add_subplot(111)
plt.show()

def live_plotter(x_vec,y1_data,line1,id,identifier='',pause_time=0.1):
    global ax
    global fig
    if line1[0]==[]:
        plt.ion()
        fig = plt.figure(figsize=(13,6))
        ax = fig.add_subplot(111)
        plt.ylabel('Output Values')
        plt.title('Sensor Vals: {}'.format(identifier))
        plt.legend(loc="upper left")

        plt.show()


    if line1[id]==[]:
        # this is the call to matplotlib that allows dynamic plotting
        # create a variable for the line so we can later update it
        line1[id], = ax.plot(x_vec,y1_data,'-o',alpha=0.8)        
        #update plot label/title

    
    # after the figure, axis, and line are created, we only need to update the y-data
    line1[id].set_ydata(y1_data)
    # adjust limits if new data goes beyond bounds
    if(id == 2):
        j = 0 
        for i in range(len(line1)):
            dataMin[j] = line1[i].axes.get_ylim()[0]
            dataMax[j] = line1[i].axes.get_ylim()[1]
            j = j+1
        maxLim = np.max(dataMax)
        minLim = np.min(dataMin)

        if np.min(y1_data)<=maxLim or np.max(y1_data)>=minLim:
            plt.ylim([np.min(y1_data)-np.std(y1_data),np.max(y1_data)+np.std(y1_data)])
    # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
    plt.pause(pause_time)
    
    # return line so we can update it again in the next iteration
    return line1[id]

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
            #print(outArray[1])
            count = count + 1
            y_vec[-1] = outArray[0]
            y_vec2[-1] = outArray[1]
            #y_vec3[-1] = outArray[2]
            lineArg[0] = live_plotter(x_vec, y_vec, lineArg, 0)
            lineArg[1] = live_plotter(x_vec, y_vec2, lineArg, 1)
            #lineArg[2] = live_plotter(x_vec,y_vec3, lineArg, 2)
            #lineArg2 = live_plotter(x_vec, y_vec, lineArg2)
            y_vec = np.append(y_vec[1:],0.0)
            y_vec2 = np.append(y_vec2[1:],0.0)
            #y_vec3 = np.append(y_vec3[1:],0.0)
        serialPort.write(bytes("-0.035,0,-0.001|",'utf-8'))
# First 10 are x hat, next 9 are sensor reading gyro, speed, roll pitch yaw.



