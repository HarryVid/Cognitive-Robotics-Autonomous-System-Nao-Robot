# This module consist of Manipulation part of the Nao robot.
from time import sleep
import almath

def manipulation_pick(motionProxy,postureProxy):

    # Define The Initial Posture for the Nao
    #postureProxy.goToPosture("StandInit", 0.5)
    # -------------------------------------------------------------------------------------
    # Define The Initial Position for the upper body
    # Head
    HeadYawAngle       = + 0.0
    HeadPitchAngle     =  0.0
    # Shoulder
    ShoulderPitchAngle = 20.0
    ShoulderRollAngle  = +10.0
    ElbowYawAngle      = -20.0
    ElbowRollAngle     = -70.0
    WristYawAngle      = + 0.0
    HandAngle          = + 100.0
    # Legs and Torso
    kneeAngle    = +100.0
    torsoAngle   = + 100.0 # bend the torso
    spreadAngle  = + 0.0 # spread the legs
    # Get the Robot Configuration
    robotConfig = motionProxy.getRobotConfig()
    robotName = ""
    for i in range(len(robotConfig[0])):
        if (robotConfig[0][i] == "Model Type"):
            robotName = robotConfig[1][i]
        if robotName == "naoH25":
            Head     = [HeadYawAngle, HeadPitchAngle]
            LeftArm  = [ShoulderPitchAngle, +ShoulderRollAngle, +ElbowYawAngle, +ElbowRollAngle, WristYawAngle, HandAngle]
            RightArm = [ShoulderPitchAngle, -ShoulderRollAngle, -ElbowYawAngle, -ElbowRollAngle, WristYawAngle, HandAngle]
            LeftLeg  = [0.0,                      #hipYawPitch
                        spreadAngle,              #hipRoll
                        -kneeAngle/2-torsoAngle,  #hipPitch
                        kneeAngle,                #kneePitch
                        -kneeAngle/2,             #anklePitch
                        -spreadAngle]             #ankleRoll
            RightLeg = [0.0, -spreadAngle, -kneeAngle/2-torsoAngle, kneeAngle, -kneeAngle/2,  spreadAngle]
        
        else:
            print "System error"
            print "---------------------"
            exit(1)
            
        # Gather the joints together
        pTargetAngles = Head + LeftArm + LeftLeg + RightLeg + RightArm
        # Convert to radians
        pTargetAngles = [ x * almath.TO_RAD for x in pTargetAngles]
        #------------------------------ send stiffness -----------------------------
        motionProxy.stiffnessInterpolation("Body", 1.0, 0.5)
        #------------------------------ send the commands -----------------------------

        # We use the "Body" name to signify the collection of all joints
        pNames = "Body"
        # We set the fraction of max speed
        pMaxSpeedFraction = 0.1
        # Ask motion to do this with a blocking call
        motionProxy.angleInterpolationWithSpeed(pNames, pTargetAngles, pMaxSpeedFraction)
        # Define the grasping of hands and fingers
        names      = ["LHand", "RHand"]
        angleLists = [ [45.0*almath.TO_RAD, 45.0*almath.TO_RAD], [45.0*almath.TO_RAD, 45.0*almath.TO_RAD]]
        timeLists  = [ [1, 2], [1, 2]]
        isAbsolute = True
        motionProxy.angleInterpolation(names, angleLists, timeLists, isAbsolute)
        # Define The New Position for the head
        HeadYawAngle       = + 0.0
        HeadPitchAngle     =  0.0

        # ---------------------------------------------------------------------------------------
        # Define The New Position for the upper body
        # Shoulder
        ShoulderPitchAngle = 20.0
        ShoulderRollAngle  = +10.0
        ElbowYawAngle      = -20.0
        ElbowRollAngle     = -70.0
        WristYawAngle      = + 0.0
        HandAngle          = + 45.0
        # Legs and Torso
        kneeAngle    = +10.0
        torsoAngle   = 10.0 # bend the torso
        spreadAngle  = + 0.0 # spread the legs
        # Get the Robot Configuration
        robotConfig = motionProxy.getRobotConfig()
        robotName = ""
        for i in range(len(robotConfig[0])):
            if (robotConfig[0][i] == "Model Type"):
                robotName = robotConfig[1][i]
            if robotName == "naoH25":
                Head     = [HeadYawAngle, HeadPitchAngle]
                LeftArm  = [ShoulderPitchAngle, +ShoulderRollAngle, +ElbowYawAngle, +ElbowRollAngle, WristYawAngle, HandAngle]
                RightArm = [ShoulderPitchAngle, -ShoulderRollAngle, -ElbowYawAngle, -ElbowRollAngle, WristYawAngle, HandAngle]
                LeftLeg  = [0.0,                      #hipYawPitch
                            spreadAngle,              #hipRoll
                            -kneeAngle/2-torsoAngle,  #hipPitch
                            kneeAngle,                #kneePitch
                            -kneeAngle/2,             #anklePitch
                            -spreadAngle]             #ankleRoll
                RightLeg = [0.0, -spreadAngle, -kneeAngle/2-torsoAngle, kneeAngle, -kneeAngle/2,  spreadAngle]
            else:
                print "system error"
                print "---------------------"
                exit(1)

            # Gather the joints together
            pTargetAngles = Head + LeftArm + LeftLeg + RightLeg + RightArm
            # Convert to radians
            pTargetAngles = [ x * almath.TO_RAD for x in pTargetAngles]
            #------------------------------ send stiffness -----------------------------
            motionProxy.stiffnessInterpolation("Body", 1.0, 0.5)
            #------------------------------ send the commands -----------------------------
            # We use the "Body" name to signify the collection of all joints
            pNames = "Body"
            # We set the fraction of max speed
            pMaxSpeedFraction = 0.1
            # Ask motion to do this with a blocking call
            motionProxy.angleInterpolationWithSpeed(pNames, pTargetAngles, pMaxSpeedFraction)
            sleep(2)
            #motionProxy.killAll()

            
def manipulation_place(motionProxy):
    HeadYawAngle       = + 0.0
    HeadPitchAngle     =  0.0
    # Shoulder
    ShoulderPitchAngle = 20.0
    ShoulderRollAngle  = +10.0
    ElbowYawAngle      = -20.0
    ElbowRollAngle     = -70.0
    WristYawAngle      = + 0.0
    HandAngle          = +45.0
    # Legs and Torso
    kneeAngle    = +100.0
    torsoAngle   = + 100.0 # bend the torso
    spreadAngle  = + 0.0 # spread the legs
    # Get the Robot Configuration
    robotConfig = motionProxy.getRobotConfig()
    robotName = ""
    for i in range(len(robotConfig[0])):
        if (robotConfig[0][i] == "Model Type"):
            robotName = robotConfig[1][i]
        if robotName == "naoH25":
            Head     = [HeadYawAngle, HeadPitchAngle]
            LeftArm  = [ShoulderPitchAngle, +ShoulderRollAngle, +ElbowYawAngle, +ElbowRollAngle, WristYawAngle, HandAngle]
            RightArm = [ShoulderPitchAngle, -ShoulderRollAngle, -ElbowYawAngle, -ElbowRollAngle, WristYawAngle, HandAngle]
            LeftLeg  = [0.0,                      #hipYawPitch
                        spreadAngle,              #hipRoll
                        -kneeAngle/2-torsoAngle,  #hipPitch
                        kneeAngle,                #kneePitch
                        -kneeAngle/2,             #anklePitch
                        -spreadAngle]             #ankleRoll
            RightLeg = [0.0, -spreadAngle, -kneeAngle/2-torsoAngle, kneeAngle, -kneeAngle/2,  spreadAngle]
        
        else:
            print "ERROR : Your robot is unknown"
            print "This test is not available for your Robot"
            print "---------------------"
            exit(1)
        # Gather the joints together
        pTargetAngles = Head + LeftArm + LeftLeg + RightLeg + RightArm
        # Convert to radians
        pTargetAngles = [ x * almath.TO_RAD for x in pTargetAngles]
        #------------------------------ send stiffness -----------------------------
        motionProxy.stiffnessInterpolation("Body", 1.0, 0.5)
        #------------------------------ send the commands -----------------------------
        # We use the "Body" name to signify the collection of all joints
        pNames = "Body"
        # We set the fraction of max speed
        pMaxSpeedFraction = 0.1
        # Ask motion to do this with a blocking call
        motionProxy.angleInterpolationWithSpeed(pNames, pTargetAngles, pMaxSpeedFraction)
        # Define the grasping of hands and fingers
        motionProxy.openHand('LHand')
        motionProxy.openHand('RHand')
        # Define The New Position for the head
        HeadYawAngle       = + 0.0
        HeadPitchAngle     =  0.0
        # Define The New Position for the upper body
        # Shoulder
        ShoulderPitchAngle = 20.0
        ShoulderRollAngle  = +50.0
        ElbowYawAngle      = -20.0
        ElbowRollAngle     = -70.0
        WristYawAngle      = + 0.0
        HandAngle          = + 100.0
        # Legs and Torso
        kneeAngle    = +10.0
        torsoAngle   = 10.0 # bend the torso
        spreadAngle  = + 0.0 # spread the legs
        # Get the Robot Configuration
        robotConfig = motionProxy.getRobotConfig()
        robotName = ""
        for i in range(len(robotConfig[0])):
            if (robotConfig[0][i] == "Model Type"):
                robotName = robotConfig[1][i]
            if robotName == "naoH25":
                Head     = [HeadYawAngle, HeadPitchAngle]
                LeftArm  = [ShoulderPitchAngle, +ShoulderRollAngle, +ElbowYawAngle, +ElbowRollAngle, WristYawAngle, HandAngle]
                RightArm = [ShoulderPitchAngle, -ShoulderRollAngle, -ElbowYawAngle, -ElbowRollAngle, WristYawAngle, HandAngle]
                LeftLeg  = [0.0,                      #hipYawPitch
                            spreadAngle,              #hipRoll
                            -kneeAngle/2-torsoAngle,  #hipPitch
                            kneeAngle,                #kneePitch
                            -kneeAngle/2,             #anklePitch
                            -spreadAngle]             #ankleRoll
                RightLeg = [0.0, -spreadAngle, -kneeAngle/2-torsoAngle, kneeAngle, -kneeAngle/2,  spreadAngle]
            
            else:
                print "ERROR : Your robot is unknown"
                print "This test is not available for your Robot"
                print "---------------------"
                exit(1)
            # Gather the joints together
            pTargetAngles = Head + LeftArm + LeftLeg + RightLeg + RightArm
            # Convert to radians
            pTargetAngles = [ x * almath.TO_RAD for x in pTargetAngles]
            #------------------------------ send stiffness -----------------------------
            motionProxy.stiffnessInterpolation("Body", 1.0, 0.5)
            #------------------------------ send the commands -----------------------------
            # We use the "Body" name to signify the collection of all joints
            pNames = "Body"
            # We set the fraction of max speed
            pMaxSpeedFraction = 0.1
            # Ask motion to do this with a blocking call
            motionProxy.angleInterpolationWithSpeed(pNames, pTargetAngles, pMaxSpeedFraction)
            sleep(2) # 2 seconds delay to hold the current position 
            moitonProxy.killAll() # kill all the NAO motion operation 
