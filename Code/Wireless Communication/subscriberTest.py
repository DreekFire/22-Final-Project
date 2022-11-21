import time
import paho.mqtt.client as paho
import matplotlib.pyplot as plt

broker="192.168.4.168"
PORT = 1883
#define callback
rec_data = []
def on_message(client, userdata, message):
    time.sleep(1)
    rec_data.append(float(message.payload.decode("utf-8")))
    print("received data =",str(message.payload.decode("utf-8")))
    if float(message.payload.decode("utf-8")) == -350.00:
        plt.plot(rec_data)
        plt.show()
        data = []
        

client= paho.Client("client-001") #create client object client1.on_publish = on_publish #assign function to callback client1.connect(broker,port) #establish connection client1.publish("house/bulb1","on")
######Bind function to callback
client.on_message=on_message
#####
print("connecting to broker ",broker)
client.connect(broker, PORT)#connect
print("subscribing ")
client.subscribe("/swa/temperature")#subscribe
client.loop_forever() #start loop to process received messages


# time.sleep(2)
# print("publishing ")
# client.publish("house/bulb1","on")#publish
# time.sleep(4)
# client.disconnect() #disconnect
# client.loop_stop() #stop loop
