import cv2
import numpy as np
import urllib.request
import serial
import time
from xbee import XBee

url = 'http://192.168.4.1/cam-hi.jpg'
port = '/dev/cu.usbserial-1130'  # Modify this with the appropriate serial port
baud_rate = 115200  # Modify this with the appropriate baud rate

cap = cv2.VideoCapture(url)
whT = 320
confThreshold = 0.5
nmsThreshold = 0.3
classesfile = 'coco.names'
classNames = []

with open(classesfile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

carClassId = classNames.index('cell phone')  # Get the class ID for car

modelConfig = 'yolov3.cfg'
modelWeights = 'yolov3.weights'
net = cv2.dnn.readNetFromDarknet(modelConfig, modelWeights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

up = False
down = False
x1, y1 = 0, 500
x2, y2 = 800, 500
xbee_port = serial.Serial(port, baud_rate)
xbee = XBee(xbee_port)

# def read(num_char = 1):
#     string = xbee_port.read(num_char)
#     return string.decode('ascii')

# def send(angle):
#     command = str.encode(angle+"\n")
#     #command = f"{angle}\n"  # Append newline character to the command
#     xbee_port.write(command)

# def xbeeSend(message):
#     destination_address = b'\x13\xA2\x00\x40\xD4\xF0\xA1'
#     try:
#         xbee.tx(dest_addr=destination_address, data=message)
#         print("Message sent successfully.")
#     except:
#         print("Error sending message")


def findObject(outputs, img):
    global up, down
    hT, wT, cT = img.shape
    bbox = []
    classIds = []
    confs = []

    # misc
    message = "1000"
    destination_address = b'\x13\xA2\x00\x40\xD4\xF0\xA1'
    #destination_address = b'\x13\xA2\x00\x40\xD4\xF0\x60'
    try:
        xbee.tx(dest_addr=destination_address, data=message)
        print("Message sent successfully.")
    except:
        print("Error sending message")
    for output in outputs:
        for det in output:
            scores = det[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if classId == carClassId and confidence > confThreshold:
                w, h = int(det[2] * wT), int(det[3] * hT)
                x, y = int((det[0] * wT) - w / 2), int((det[1] * hT) - h / 2)
                bbox.append([x, y, w, h])
                classIds.append(classId)
                confs.append(float(confidence))
            else:
                #xbeeSend("1000")
                message = "1000"
                destination_address = b'\x13\xA2\x00\x40\xD4\xF0\xA1'
                #destination_address = b'\x13\xA2\x00\x40\xD4\xF0\x60'
                try:
                    xbee.tx(dest_addr=destination_address, data=message)
                    print("Message sent successfully.")
                except:
                    print("Error sending message")
                #time.sleep(1)
                # try:
                #     printer = xbee_port.readline().decode('ascii')
                # except:
                #     printer = "unable to decode"
                # print(printer) 
                
                #print("Off")
    indices = cv2.dnn.NMSBoxes(bbox, confs, confThreshold, nmsThreshold)
    for i in indices:
        i = i
        box = bbox[i]
        x, y, w, h = box[0], box[1], box[2], box[3]
        center_x = x + w // 2
        center_y = y + h // 2
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
        cv2.putText(img, f'{classNames[classIds[i]].upper()} {int(confs[i] * 100)}%', (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        cv2.circle(img, (center_x, center_y), 10, (0, 255, 0), -1)  # Draw a dot at the center of the bounding box
        if center_y > y1:
            print("Up")
            #xbeeSend("1025") 
            #send("1010")  #send or left or right
            # try:
            #     printer = xbee_port.readline().decode('ascii')
            # except:
            #     printer = "unable to decode"
            # print(printer) 
        elif center_y < y1-400:
            print("Down")
            #xbeeSend("1000")  
            # try:
            #     printer = xbee_port.readline().decode('ascii')
            # except:
            #     printer = "unable to decode"
            # print(printer)
        else:
            #xbeeSend("1000")  
            print("Center")
        

    # Draw line 1
    cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # Draw line 2
    cv2.line(img, (x1, y1-400), (x2, y2-400), (0, 0, 255), 2)


while True:
    img_resp = urllib.request.urlopen(url)
    imgnp = np.array(bytearray(img_resp.read()), dtype=np.uint8)
    im = cv2.imdecode(imgnp, -1)
    success, img = cap.read()
    blob = cv2.dnn.blobFromImage(im, 1 / 255, (whT, whT), [0, 0, 0], 1, crop=False)
    net.setInput(blob)
    layerNames = net.getLayerNames()
    outputNames = [layerNames[i - 1] for i in net.getUnconnectedOutLayers()]
    outputs = net.forward(outputNames)
    findObject(outputs, im)
    cv2.imshow('Image', im)
    if up and down:
        up = False
        down = False
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()