#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import sys, select, termios, tty
import sys
sys.path.append('/path/to/opencv/module')
import cv2
import mediapipe as mp
import time
import math

#making publishers for publishing joint angles to each joint
pub_index_tip = rospy.Publisher('/hand_bot/index_tip_controller/command', Float64, queue_size=10) 
pub_index_middle = rospy.Publisher('/hand_bot/index_middle_controller/command', Float64, queue_size=10)
pub_index_lower = rospy.Publisher('/hand_bot/index_lower_controller/command', Float64, queue_size=10)
pub_middle_tip = rospy.Publisher('/hand_bot/middle_tip_controller/command', Float64, queue_size=10)
pub_middle_middle = rospy.Publisher('/hand_bot/middle_middle_controller/command', Float64, queue_size=10)
pub_middle_lower = rospy.Publisher('/hand_bot/middle_lower_controller/command', Float64, queue_size=10)
pub_ring_tip = rospy.Publisher('/hand_bot/ring_tip_controller/command', Float64, queue_size=10)
pub_ring_middle= rospy.Publisher('/hand_bot/ring_middle_controller/command', Float64, queue_size=10)
pub_ring_lower = rospy.Publisher('/hand_bot/ring_lower_controller/command', Float64, queue_size=10)
pub_little_tip = rospy.Publisher('/hand_bot/little_tip_controller/command', Float64, queue_size=10)
pub_little_middle = rospy.Publisher('/hand_bot/little_middle_controller/command', Float64, queue_size=10)
pub_little_lower = rospy.Publisher('/hand_bot/little_lower_controller/command', Float64, queue_size=10)
pub_thumb_tip = rospy.Publisher('/hand_bot/thumb_tip_controller/command', Float64, queue_size=10)
pub_thumb_lower = rospy.Publisher('/hand_bot/thumb_lower_controller/command', Float64, queue_size=10)
rospy.init_node('hand_bot')
#setting flag for closing once gazebo is shut down
terminate= False
while not rospy.is_shutdown() and not terminate :


    def calculate_angle(point_a, point_b, point_c):
        '''Function to create joint angles based on the 3d coordintes provided
         Args: 3 lists containing x,y,z 
         return: angle in degrees'''
        # Calculate vectors AB and BC
        vector_ab = [point_b[0] - point_a[0], point_b[1] - point_a[1], point_b[2] - point_a[2]]
        vector_bc = [point_c[0] - point_b[0], point_c[1] - point_b[1], point_c[2] - point_b[2]]

        # Calculate dot product
        dot_product = vector_ab[0] * vector_bc[0] + vector_ab[1] * vector_bc[1] + vector_ab[2] * vector_bc[2]

        # Calculate magnitudes
        magnitude_ab = math.sqrt(vector_ab[0] ** 2 + vector_ab[1] ** 2 + vector_ab[2] ** 2)
        magnitude_bc = math.sqrt(vector_bc[0] ** 2 + vector_bc[1] ** 2 + vector_bc[2] ** 2)

        # Calculate angle in radians
        angle_radians = math.acos(dot_product / (magnitude_ab * magnitude_bc))

        # Convert angle to degrees
        angle_degrees = math.degrees(angle_radians)

        return angle_degrees
    #capture the video
    cap = cv2.VideoCapture(0)
    # variable to initialize mediapipe pipeline to track hands
    mpHands = mp.solutions.hands
    hands = mpHands.Hands(static_image_mode=True,
                        max_num_hands=2,
                        min_detection_confidence=0.60,
                        min_tracking_confidence=0.60)
    mpDraw = mp.solutions.drawing_utils
    # initializing times to get frame rate
    pTime = 0
    cTime = 0
    #initializing some rough joint coordinates 
    one = [0.0001, 0.0001, 0.0001]
    two = [0.002,0.002,0.002]
    three = [0.001,0.001 , 0.001]
    four = [0.01,0.01,0.01]
    eight = [0.001,0.001 , 0.001]
    six = [0.0001, 0.0001, 0.0001]
    seven = [0.002,0.002,0.002]
    five = [0.002,0.002,0.002]
    twelve = [0.001,0.001 , 0.001]
    ten = [0.0001, 0.0001, 0.0001]
    eleven = [0.002,0.002,0.002]
    thirteen = [0.001,0.001 , 0.001]
    fourteen = [0.0001, 0.0001, 0.0001]
    fifteen = [0.002,0.002,0.002]
    sixteen = [0.001,0.001,0.001]
    seventeen = [0.001,0.001 , 0.001]
    eighteen = [0.0001, 0.0001, 0.0001]
    nineteen = [0.002,0.002,0.002]
    twenty = [0.001,0.001,0.001]
    nine = [0.002,0.002,0.002]
    zero = [0, 0, 0]

    
    while True:
        #reading images dynamically and converting each frame to RGB
        success, img = cap.read()
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = hands.process(imgRGB)
        #if a hand is detected and a landmark is detected
        if results.multi_hand_landmarks:
            for handLms in results.multi_hand_landmarks:
                #enumerate tracking id and keypoints on each joints extracted
                for id, lm in enumerate(handLms.landmark):
                    #representing keypoints as circles 
                    h, w, c = img.shape
                    cx, cy = int(lm.x *w), int(lm.y*h)
                    #tracking coordinates based on ID
                    if id ==1:
                        one = [lm.x,lm.y,lm.z]
                        
                    if id ==2:
                        two = [lm.x,lm.y,lm.z]
                    
                    if id == 3:
                        three = [lm.x,lm.y,lm.z]
                    if id == 4:
                        four = [lm.x,lm.y,lm.z]                        
                    if id ==8:
                        eight = [lm.x,lm.y,lm.z]
                        
                    if id ==6:
                        six = [lm.x,lm.y,lm.z]
                    
                    if id == 7:
                        seven = [lm.x,lm.y,lm.z]
                    if id == 5:
                        five = [lm.x,lm.y,lm.z]
                    if id == 0:
                        zero = [lm.x, lm.y, lm.z]
                    if id == 9:
                        nine = [lm.x, lm.y, lm.z]
                    if id == 10:
                        ten = [lm.x, lm.y, lm.z]
                    if id == 11:
                        eleven = [lm.x, lm.y, lm.z]
                    if id == 12:
                        twelve = [lm.x, lm.y, lm.z]
                    if id == 13:
                        thirteen = [lm.x, lm.y, lm.z]
                    if id == 14:
                        fourteen = [lm.x, lm.y, lm.z]
                    if id == 15:
                        fifteen = [lm.x, lm.y, lm.z]
                    if id == 16:
                        sixteen = [lm.x, lm.y, lm.z]
                    if id == 17:
                        seventeen = [lm.x, lm.y, lm.z]
                    if id == 18:
                        eighteen = [lm.x, lm.y, lm.z]
                    if id == 19:
                        nineteen = [lm.x, lm.y, lm.z]
                    if id == 20:
                        twenty = [lm.x, lm.y, lm.z]                    

                    #calculating joint angles based on the points calculated above                    
                    angle2 = calculate_angle(five,six,seven)
                    angle1 = calculate_angle(six,seven,eight)
                    angle3 = calculate_angle(zero, five, eight)
                    angle4 = calculate_angle(ten,eleven,twelve)
                    angle5 = calculate_angle(nine,ten,eleven)
                    angle6 = calculate_angle(zero,nine,ten)
                    angle7 = calculate_angle(fourteen,fifteen,sixteen)
                    angle8 = calculate_angle(thirteen,fourteen,fifteen)
                    angle9 = calculate_angle(zero,thirteen,fourteen)
                    angle10 = calculate_angle(eighteen,nineteen,twenty)
                    angle11 = calculate_angle(seventeen,eighteen,nineteen)
                    angle12 = calculate_angle(zero,seventeen,eighteen)
                    angle14= calculate_angle(one,two,three)
                    angle13= calculate_angle(two,three,four)
                    
                    #setting threshold for index finger upper joint
                    if angle1 < 20.0:
                        Indexj1 = 0

                    elif angle1 >= 20.0:
                        Indexj1 = 90
                    else:
                        continue

                    print("Index Joint 1", Indexj1)
                    pub_index_tip.publish(Indexj1* 0.01744)

                    #setting threshold for index finger middle joint
                    if angle2 < 30.0:
                        Indexj2 = 0
                    elif angle2 > 20.0 and angle2 < 45.0:
                        Indexj2 = 45
                    elif angle2 >= 45.0:
                        Indexj2 = 90
                    else:
                        continue
                    
                    print("Index Joint 2", Indexj2)
                    pub_index_middle.publish(Indexj2* 0.01744)

                    #setting threshold for index finger lower joint
                    if angle3 < 30.0:
                        Indexj3 = 0
                    elif angle3 > 30.0 and angle3 < 60.0:
                        Indexj3 = 45
                    elif angle3 > 60.0:
                        Indexj3 = 90
                    else:
                        continue
                
                    print("Index Joint 3", Indexj3)
                    pub_index_lower.publish(Indexj3* 0.01744)

                    #setting threshold for Middle finger upper joint
                    if angle4 < 50.0:
                        Middlej1 = 0
                    elif angle4 > 50.0:
                        Middlej1 = 90
                    else:
                        continue

                    print("Middle Joint 1", Middlej1)
                    pub_middle_tip.publish(Middlej1* 0.01744)

                    #setting threshold for Middle finger middle joint
                    if angle5<15.0:
                        Middlej2 = 0
                    elif angle5>15.0 and angle5<60.0:
                        Middlej2 = 45
                    elif angle5>60.0:
                        Middlej2 = 90
                    else:
                        continue

                    print("Middle Joint 2", Middlej2)
                    pub_middle_middle.publish(Middlej2* 0.01744)

                    #setting threshold for Middle finger lower joint

                    if angle6>20.0:
                        Middlej3 = 90
                    elif angle6<15.0:
                        Middlej3 = 0
                    elif angle6<20.0 and angle6>15.0:
                        Middlej3 = 45
                    print("Middle Joint 3", Middlej3)
                    pub_middle_lower.publish(Middlej3* 0.01744)
                    #setting threshold for Ring finger upper joint
                    if angle7 < 10.0:
                        Ringj1 = 0
                    elif angle7 >= 10.0:
                        Ringj1 = 90
                    else:
                        continue
                
                    print("Ring Joint 1", Ringj1)
                    pub_ring_tip.publish(Ringj1* 0.01744)

                    #setting threshold for Ring finger middle joint

                    if angle8 < 10.0:
                        Ringj2 = 0
                    elif angle8 > 10.0 and angle8 < 50.0:
                        Ringj2 = 45
                    elif angle8 >= 50.0:
                        Ringj2 = 90
                    else:
                        continue
                    
                    print("Ring Joint 2", Ringj2)
                    pub_ring_middle.publish(Ringj2* 0.01744)
                    #setting threshold for Ring finger lower joint
                    if angle9 > 25.0:
                        Ringj3 = 90
                    elif angle9 > 10.0 and angle9 < 25.0:
                        Ringj3 = 45
                    elif angle9 < 7.0:
                        Ringj3 = 0
                    else:
                        continue
                    
                    print("Ring Joint 3", Ringj3)
                    pub_ring_lower.publish(Ringj3* 0.01744)

                    #setting threshold for little finger upper joint
                    if angle10 < 14.0:
                        littlej1 = 0
                    elif angle10 >= 14.0:
                        littlej1 = 90
                    else:
                        continue
                    
                    print("little joint 1", littlej1)
                    pub_little_tip.publish(littlej1* 0.01744)

                    #setting threshold for little finger middle joint
                    if angle11 <= 18.0:
                        littlej2 = 0
                    elif angle11 > 18.0 and angle11 < 30.0:
                        littlej2 = 45
                    elif angle11 >= 30.0:
                        littlej2 = 90
                    else:
                        continue
                   
                    print("little joint 2", littlej2)
                    pub_little_middle.publish(littlej2* 0.01744)

                    #setting threshold for little finger lower joint
                    if angle12 < 30.0:
                        littlej3 = 0
                    elif angle12 > 20.0 and angle12 < 45.0:
                        littlej3 = 45
                    elif angle12 >= 45.0:
                        littlej3 = 90
                    else:
                        continue
                    
                    print("little joint 3", littlej3)
                    pub_little_lower.publish(-littlej3* 0.01744)

                    #setting threshold for thumb finger upper joint
                    if angle13 < 30.0:
                        thumbj2 = 0
                    elif angle13 >= 30.0:
                        thumbj2 = 90
                    else:
                        continue
                    
                    print("thumb joint 2", thumbj2)
                    pub_thumb_lower.publish(thumbj2* 0.01744)

                    #setting threshold for thumb finger lower joint
                    if angle14 < 11.0:
                        thumbj1 = 0
                    elif angle14 >= 11.0:
                        thumbj1 = 90
                    else:
                        continue
                    
                    print("thumb joint 1", thumbj1)
                    pub_thumb_tip.publish(-thumbj1* 0.01744)
                    #drawing circle on keypoints detected
                    cv2.circle(img, (cx,cy), 3, (255,0,255), cv2.FILLED)

                mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)
        #showing fps
        cTime = time.time()
        fps = 1/(cTime-pTime)
        pTime = cTime
        #printing FPS
        #cv2.putText(img,str(int(fps)), (10,70), cv2.FONT_HERSHEY_PLAIN, 3, (255,0,255), 3)

        cv2.imshow("Image", img)
        cv2.waitKey(5)
        if cv2.waitKey(5) == ord('q'):  # Press 'q' to end the code
            terminate = True

 


    

