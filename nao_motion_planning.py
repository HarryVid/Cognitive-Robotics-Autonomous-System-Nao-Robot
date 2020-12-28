import naoqi
from naoqi import ALProxy
from time import sleep
import almath as m
import sys
import cv2
import numpy as np
import vision_definitions
import argparse
import imutils
import math
import almath
import manipulation
import movement



# ------------------------------------------------------------------------------------------------------
# Global variable declerations
# ------------------------------------------------------------------------------------------------------

set_sleep=2 # sec for nao sleep between the process
nao_ip="localhost"
nao_port=9559
max_nao_move=30 # final destination Nao to reach
nao_turn_value=1.5
tmp_nao_move=10
nao_pos=[]
bottle_pos=[5,0.00045095,-1.69977]# Bottle position
bottle_x_pos=bottle_pos[0]
bottle_y_pos=bottle_pos[1]
bottle_z_pos= bottle_pos[2]
nao_calib_x=4.80
nao_calib_z=-1.7

class DistanceToCamera(object):
    def __init__(self):
        # camera params
        self.alpha = 8.0 * math.pi / 180
        self.v0 = 119.865631204
        self.ay = 332.262498472

    def calculate(self, v, h,x_shift,y_shift, image,nao_motion,nao_posture,nao_pos):
        # compute and return the distance from the target point to the camera
        d = h*5 / math.tan(self.alpha + math.atan((v - self.v0) / self.ay))
        k_flag=0
        if d > 0:
            cv2.putText(image, "%.1fcm" % d,
                ( x_shift,y_shift), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            print "Distance",d
            if float(d)<=27.25:
                nao_motion.killAll()
                sleep(1)
                nao_posture.goToPosture("StandInit", 0.5)
                if len(nao_pos) == 3:
                    nao_x_pos=nao_pos[0]
                    nao_z_pos=nao_pos[2]
                    if nao_x_pos != bottle_x_pos:
                        manipulation.manipulation_pick(nao_motion,nao_posture) # pick the object
                        nao_motion.moveTo(1,0,-1) # move to near by place
                        manipulation.manipulation_place(nao_motion)
                    if (round(nao_x_pos,2) == round(nao_calib_x,2)) and (round(nao_z_pos,2) == round(bottle_z_pos,2)):
                        manipulation.manipulation_pick(nao_motion,nao_posture) # pick the object
                        nao_motion.moveTo(1,0,-1) # move to near by place
                        manipulation.manipulation_place(nao_motion,nao_posture) # place the object
                    elif (round(nao_x_pos,2) == round(nao_calib_x,2)):
                        #tmp_g=bottle_pos[2]-current_position[2]
                        #nao_motion.moveTo(0,0,tmp_g+current_position)
                        nao_motion.moveTo(0,0,nao_calib_z)
                k_flag=1
                nao_motion.killAll()
                print "Nao motion killed"
            else:
                k_flag=0
            
        return d,k_flag

    
def speak():
    global nao_speak
    nao_speak= ALProxy("ALTextToSpeech",nao_ip,nao_port)
    nao_speak.setLanguage("English")
    nao_speak.say("Hello All I'm nao robot")
    print "speak process done"

def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

    
    
def motion():
    global nao_speak
    robot_name=""
    d_h=15.5-10 #(cm) height defintion
    dist_cam=20 # camera distance to stop
    nao_pos=[]
    try:
        nao_motion=naoqi.ALProxy("ALMotion",nao_ip,nao_port)
        nao_memory=naoqi.ALProxy("ALMemory",nao_ip,nao_port)
        nao_sonar_config=naoqi.ALProxy("ALSonar",nao_ip,nao_port)
        nao_posture=naoqi.ALProxy("ALRobotPosture", nao_ip,nao_port)
        videoDevice=naoqi.ALProxy("ALVideoDevice",nao_ip,nao_port)
    except Exception, e:
        print "Library initiation failed"
        print "Fix the error",e

    # ----------------------------------------------------------------------------    
    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--prototxt", required=True,
            help="path to Caffe 'deploy' prototxt file")
    ap.add_argument("-m", "--model", required=True,
            help="path to Caffe pre-trained model")
    ap.add_argument("-c", "--confidence", type=float, default=0.2,
            help="minimum probability to filter weak detections")
    args = vars(ap.parse_args())

    # initialize the list of class labels MobileNet SSD was trained to
    # detect, then generate a set of bounding box colors for each class
    CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
            "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
            "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
            "sofa", "train", "tvmonitor"]
    COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

    # load our serialized model from disk
    print("[INFO] loading model...")
    net = cv2.dnn.readNetFromCaffe(args["prototxt"], args["model"])
    # initialize the video stream, allow the cammera sensor to warmup,
    # and initialize the FPS counter
    print("[INFO] starting video stream...")

    # get NAOqi module proxy
    resolution = vision_definitions.kQVGA
    colorSpace = vision_definitions.kBGRColorSpace
    fps = 30

    captureDevice= videoDevice.subscribe("python_GVM", resolution, colorSpace, fps)
    videoDevice.setParam(vision_definitions.kCameraSelectID,1)

    # create image
    width = 320
    height = 240
    image = np.zeros((height, width, 3), np.uint8)
    d_to_camera = DistanceToCamera()

    # --------------------------------------------------------------------------------    
    # get robot details    
    robot_config=nao_motion.getRobotConfig()
    print "robot configuration ---------- "
    print "robot name:",robot_config[1][0]

    # prepare Nao
    StiffnessOn(nao_motion)
    #nao_motion.setStiffnesses("Body",1.0)
    nao_motion.wakeUp()
    killflag = nao_posture.goToPosture("StandInit", 0.5)
    print "kill_flag",killflag
    initial_position=nao_motion.getRobotPosition(False)
    print "Initial postion of nao in 2D",m.Pose2D(nao_motion.getRobotPosition(False))
    print "Initial postion of nao:",initial_position
    sleep(set_sleep)
    nao_motion.moveInit()
    movement.bot_move_forward_backend(nao_motion,max_nao_move)
    while True:
        current_position=nao_motion.getRobotPosition(False)
        result = videoDevice.getImageRemote(captureDevice);
        pkill1=0
        pkill2=0

        # ---------------------------------------------------------------------
        for i in range (0,len(current_position)):
            nao_pos.append(current_position[i]-initial_position[i])
            
        print "Nao current position in space:",nao_pos
        
        # ----------------------------------------------------------------------
        # get the ultrasonic sensor value
        nao_sonar_config.subscribe("Application")
        l_ult_val=nao_memory.getData("Device/SubDeviceList/US/Left/Sensor/Value")
        r_ult_val=nao_memory.getData("Device/SubDeviceList/US/Right/Sensor/Value")
        print "Left ult sensor value", l_ult_val
        print "Right ult sensor value", r_ult_val
        sleep(0.5)
        #####################################
        condition1 = l_ult_val >=0.40 and l_ult_val <=0.65
        condition2 = r_ult_val >=0.40 and r_ult_val <=0.65
        if condition1:
            pkill1=1
            print "moving right"
            nao_motion.killWalk()
            movement.bot_move_right(nao_motion,nao_turn_value)
        elif condition2:
            pkill2=1
            nao_motion.killWalk()
            print "moving left"
            movement.bot_move_left(nao_motion,nao_turn_value)
            #bot_move_forward(nao_motion,5)

        if pkill1 == 1 or pkill2==1:
            movement.bot_move_forward_backend(nao_motion,tmp_nao_move)

        if (l_ult_val <=0.30 or r_ult_val<=0.30):
            movement.bot_move_backward(nao_motion,1)
            movement.bot_move_forward_backend(nao_motion,tmp_nao_move)
            
        if (killflag==False) or (l_ult_val <=0.10 or r_ult_val<=0.10):
            nao_sonar_config.unsubscribe("Application")
            print "ultrasonic sensor removed"
            nao_motion.killMove()
            nao_motion.rest()
            print ("Nao motion reseted")
            #nao_motion.killAll()
            #print "All process are killed"
            break
        # ---------------------------------------------------------------------
        # obeject detection and representation part
        if result == None:
            print 'cannot capture.'
        elif result[6] == None:
            print 'no image data string.'
        else:
            # translate value to mat
            values = map(ord, list(result[6]))
            i = 0
            for y in range(0, height):
                for x in range(0, width):
                    image.itemset((y, x, 0), values[i + 0])
                    image.itemset((y, x, 1), values[i + 1])
                    image.itemset((y, x, 2), values[i + 2])
                    i += 3
            # show image
            image = imutils.resize(image, width=400)
            (h, w) = image.shape[:2]
            blob = cv2.dnn.blobFromImage(cv2.resize(image, (400, 400)),0.007843, (300, 300), 127.5)
            net.setInput(blob)
            detections = net.forward()
            # loop over the detections
            for i in np.arange(0, detections.shape[2]):
                # extract the confidence (i.e., probability) associated with
                # the prediction
                confidence = detections[0, 0, i, 2]
                # filter out weak detections by ensuring the `confidence` is
                # greater than the minimum confidenc
                if confidence > args["confidence"]:
                    idx = int(detections[0, 0, i, 1])
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")
                    d_w = startY + endY - 5 # calculate width
                    if CLASSES[idx] != "bottle":
                        continue
                    label = "{}: {:.2f}%".format(CLASSES[idx],confidence * 100)
                    cv2.rectangle(image, (startX, startY), (endX, endY),COLORS[idx], 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(image, label, (startX, y),cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
                    d,k = d_to_camera.calculate(d_w,d_h,startX+100,startY-10,image,nao_motion,nao_posture,nao_pos)
                
                    if k==1:
                        break
                    
        cv2.imshow("videofeed",image)
        nao_pos=[]    
        if cv2.waitKey(1)==ord('q'):
            nao_motion.killAll()
            break
    
    final_position= nao_motion.getRobotPosition(False)
    print "Final positon of nao in 2D",m.Pose2D(nao_motion.getRobotPosition(False))
    print "Final postion of nao:",final_position
    cv2.destroyAllWindows()
        

if __name__=="__main__":
    #speak()
    motion()
    
