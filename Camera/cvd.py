import cv2
import numpy as np


#set path& defaults
configPath = "/home/pi/Desktop/EC544/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "/home/pi/Desktop/EC544/frozen_inference_graph.pb"
classFile = "/home/pi/Desktop/EC544/model2/coco.names"
classNames = open(classFile).read().strip().split('\n')
np.random.seed(42)
colors = np.random.randint(0, 255, size=(len(classNames), 3), dtype='uint8')

net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(224,224)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

def getObjects(img, thres, nms, draw=True, objects=[]):
    classIds, confs, bbox = net.detect(img,thres,nms)
    if len(objects) == 0:
        objects = classNames
    objectInfo =[]
    if len(classIds) != 0:
        for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
            className = classNames[classId - 1]
            #print(classId)
            colo = colors[classId]
            #colo = int(colo)
            #print(int(colo[0]))
            if className in objects: 
                objectInfo.append([box,className])
                if (draw):
                    cv2.rectangle(img,box,color=[int(colo[0]),int(colo[1]),int(colo[2])],thickness=2)
                    cv2.putText(img,classNames[classId-1].upper(),(box[0]+10,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(int(colo[0]),int(colo[1]),int(colo[2])),2)
                    cv2.putText(img,str(round(confidence*100,2)),(box[0]+200,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(int(colo[0]),int(colo[1]),int(colo[2])),2)
    
    return img,objectInfo

cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)
#cap.set(10,70)

def capture():
    while True:
        success, img = cap.read()
                
        result, objectInfo = getObjects(img,0.45,0.2)
        print(len(objectInfo))    
        cv2.imshow("Output",img)
        cv2.waitKey(1)
