# import the necessary packages
import numpy as np
import time, cv2, os
from telegrambot import telegramBot

class detect():
    def __init__(self,ConfidenceLevel = 0.4,Alert_TimeInterval=30):
        self.confidence_level = ConfidenceLevel
        self.Alert_TimeInterval = Alert_TimeInterval
        self.threshold_level = 0.3
        #Pathes
        self.labelsPath = 'yolov3/coco.names'
        self.weightsPath = 'yolov3/yolov3.weights'
        self.configPath =  'yolov3/yolov3.cfg'
        self.AlertPath = 'Alerts_history'
        # Get number of images in Alerts folder
        self.AlertIndex = self.Alerts_count()
        # Telegram bot
        self.TelegramBot = telegramBot()
        # load the COCO class labels our YOLO model was trained on
        label_file = open(self.labelsPath, 'r')
        self.LABELS = label_file.read().strip().split("\n")
        label_file.close()
        print(self.LABELS)
        # initialize a list of colors to represent each possible class label
        np.random.seed(42)
        self.COLORS = np.random.randint(0, 255, size=(len(self.LABELS), 3), dtype="uint8")
        # derive the paths to the YOLO weights and model configuration
        print("[INFO] loading YOLO from disk...")
        self.net = cv2.dnn.readNetFromDarknet(self.configPath, self.weightsPath)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        # Set time interval
        self.LastAlertTime = 0
    def doDetection(self,image_path):
        #print(image_path)
        image = cv2.imread(image_path)
        (H, W) = image.shape[:2]
        # determine only the *output* layer names that we need from YOLO
        ln = self.net.getLayerNames()
        ln = [ln[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        #blob from the input image and then perform a forward pass of the YOLO object detector
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        start = time.time()
        layerOutputs = self.net.forward(ln)
        end = time.time()
        # show timing information on YOLO
        print(f'[INFO] YOLO took {end - start} seconds')
        boxes = []
        confidences = []
        classIDs = []
        #print(layerOutputs)
        # loop over each of the layer outputs
        for output in layerOutputs:
            # loop over each of the detections
            for detection in output:
                # extract the class ID and confidence (i.e., probability) of
                # the current object detection
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]
                # filter out weak predictions by ensuring the detected
                # probability is greater than the minimum probability
                if confidence > self.confidence_level:
                    # scale the bounding box coordinates back relative to the
                    # size of the image, keeping in mind that YOLO actually
                    # returns the center (x, y)-coordinates of the bounding
                    # box followed by the boxes' width and height
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")
                    # use the center (x, y)-coordinates to derive the top and
                    # and left corner of the bounding box
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))
                    # update our list of bounding box coordinates, confidences,
                    # and class IDs
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)
        print(classIDs)
        if self.Check_Threat(classIDs):
            alert_Path = self.Annotate_Img(image,classIDs, boxes,confidences)
            self.send_telegram_alert(alert_Path)
        else:
            print('NO THREAT')


    def Check_Threat(self,classIDs):
        # Check if the frame has both object (car and person)
        if 0 in classIDs:
            if 1 in classIDs:
                print('THREAT!!!')
                return True
        return False
                
    def Annotate_Img(self,image,classIDs, boxes,confidences):
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_level,
            self.threshold_level)
        # ensure at least one detection exists
        if len(idxs) > 0:
            # loop over the indexes we are keeping
            for i in idxs.flatten():
                # extract the bounding box coordinates
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
                # draw a bounding box rectangle and label on the image
                color = [int(c) for c in self.COLORS[classIDs[i]]]
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(self.LABELS[classIDs[i]], confidences[i])
                cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, color, 2)
            #Get the name of current img and Store it in 'Alerts' Folder.
            self.AlertIndex = self.AlertIndex + 1
            NewAlert_Path = self.AlertPath + '/' +str(self.AlertIndex) + '.jpg'
            cv2.imwrite(NewAlert_Path, image)
            return NewAlert_Path
    def send_telegram_alert(self,imgPath):
        currentTime = time.time()
        #print(currentTime)
        #print(self.LastAlertTime)
        #print(currentTime - self.LastAlertTime )
        if currentTime - self.LastAlertTime >= self.Alert_TimeInterval:
            self.TelegramBot.send_alert(imgPath)
            self.LastAlertTime = currentTime
            return True
        else:
            return False

    def Alerts_count(self):
        count = 0
        for path in os.listdir(self.AlertPath):
            if os.path.isfile(os.path.join(self.AlertPath, path)):
                count += 1
        return count



