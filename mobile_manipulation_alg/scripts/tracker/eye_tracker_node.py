#!/usr/bin/env python3
#opncv-python version = 4.5.4.60
#mediapipe version = 0.8.9.1
import rospy
import cv2
from mpFaceSimplified import mpFace
import math
import numpy
import numpy as np
from numpy import *
import time
from mobile_manipulation_alg.msg import vision_command
red = (0,0,255)
green = (0,255,0)
blue = (255,0,0)
yellow = (0,255,255)
#For connecting range of dots
def connectPoints(indx1,indx2,frame,face):
    for i in range(indx1,indx2):
        if i==(indx2-1):
            cv2.line(frame,face[i],face[indx1],green,1)
            break
        cv2.line(frame,face[i],face[i+1],green,1)
def intersection_point(x1_,y1_,x2_,y2_):
    x1 = x1_[0]
    x2 = x1_[1]
    x3 = x2_[0]
    x4 = x2_[1]
    y1 = y1_[0]
    y2 = y1_[1]
    y3 = y2_[0]
    y4 = y2_[1]
    D = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)
    Px = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4))/D
    Py = ((x1*y2-y1*x2)*(y3-x4)-(y1-y2)*(x3*y4-y3*x4))/D
    return Px, Py
#Finding length between two points
def findRadius(pt1,pt2):
    x1,y1 = pt1
    x2,y2 = pt2
    radius = math.sqrt(((y2-y1)*(y2-y1))+((x2-x1)*(x2-x1)))
    return radius
def transformRange(value, r1,r2):
    scale = 0
    if (r1[1]-r1[0]) == 0:
        scale = 0
    else:
        scale = (r2[1]-r2[0])/(r1[1]-r1[0])
    return((value-r1[0])*scale+ r2[0])
def distance_between_two_point(p1, p2):
    result= ((((p2[0] - p1[0] )**2) + ((p2[1] - p1[1])**2) )**0.5)
    return result
def openorclose(flag):
    if flag == True:
        str = "Open"
    else:
        str = "Close"
    return str
def trueoFalse(flag):
    if flag == True:
        str = "True"
    else:
        str = "False"
    return str

def find_line_equation(p1, p2):
    m = (p1[1]-p2[1])/(p1[0]-p2[0])
    n = p1[1]-m*p1[0]
    return [m,n]
def find_intersection_point(p1,p2,p3,p4):
    x1 = [p1[0], p2[0]]
    y1 = [p1[1], p2[1]]
    x2 = [p3[0], p4[0]]
    y2 = [p3[1], p4[1]]
    k1 = find_line_equation(p1, p2)
    k2 = find_line_equation(p3, p4)

    x_inter = (k2[1]-k1[1])/(k1[0]-k2[1])
    y_inter = (k1[0]*x_inter)+k1[1]
    return x_inter,y_inter

def perp( a ) :
    b = empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

# line segment a given by endpoints a1, a2
# line segment b given by endpoints b1, b2
# return
def seg_intersect(a1,a2, b1,b2) :
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = dot( dap, db)
    num = dot( dap, dp )
    return (num / denom.astype(float))*db + b1


def vision_computation():
    rospy.init_node('vision_node', anonymous=True)
    vision_command_pub = rospy.Publisher('/vision/command', vision_command,queue_size=1)  # rostopic
    cam = cv2.VideoCapture(0)
    # cam = cv2.VideoCapture("http://192.168.1.3:4747/video")# connecting to ip cam
    width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
    #print(width,height)

    cam.set(cv2.CAP_PROP_FPS, 30)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    radius = 2


    faceLm = mpFace()

    ModeString = "Mode 1"
    mode = 1
    previous_head_roll = 0
    previous_head_yaw = 0
    previous_head_pitch = 0
    emergency_stop = False
    start_timer = False
    start = 0
    end = 0
    previous_face_detected = False
    _, frame = cam.read()
    size = frame.shape
    x_cursor = size[1]/2
    y_cursor = size[0]/2
    vision_command_msg = vision_command()
    vision_command_msg.x_screen = 600
    vision_command_msg.y_screen = 480
    vision_command_msg.x_cursor = 600/2
    vision_command_msg.y_cursor = 480/2
    vision_command_msg.mode = mode
    vision_command_msg.open_mouth = False
    vision_command_msg.emergency_stop = False
    vision_command_msg.roll = 0
    vision_command_msg.pitch = 0
    vision_command_msg.yaw = 0
    timer_emergency_stop = ""
    previous_both_eye_close = False
    start_e = 0
    while not rospy.is_shutdown():
        a, frame = cam.read()
        size = frame.shape

        faces = faceLm.faceLandmarksSimplified(frame)
        # for face in faces:
        #     for lm in face:
        #         cv2.circle(frame,lm,radius,green,-1)
        # emergency_stop stop no facce detected
        if len(faces) == 0:
            if start_timer == False or previous_face_detected:
                start = time.time()
                start_timer = True
            else:
                if not (previous_face_detected):
                    end = time.time()
                    timer_emergency_stop = "Time Face not detected " +  str(math.floor(end-start))
                    if((end-start) > 15):
                        emergency_stop = True
                        start_timer = False
        else:
            timer_emergency_stop = "Time Face not detected " +  str(0)

        previous_face_detected = not len(faces) == 0




        emergency_stop_string = "Emergency Stop " + trueoFalse(emergency_stop)
        cv2.putText(frame, emergency_stop_string, (400, 40), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(frame, timer_emergency_stop, (400, 80), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255), 2)

        for face in faces:
            cv2.putText(frame, ModeString, (20, 40), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255), 2)
            for indx in range(0,138):# Total Landmarks = 138
                cv2.circle(frame,face[indx],radius,red,-1)
            # cv2.putText(frame, str(21), face[21], cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)
            connectPoints(0,10,frame,face)#Left Eyebrow (0->9)
            connectPoints(10,20,frame,face)#right Eyebrow (10->19)
            connectPoints(20,36,frame,face)#Left Eye (20->35)
            connectPoints(36,52,frame,face)#Right Eye (36->51)
            connectPoints(52,72,frame,face)#iner Lip (52->71)
            connectPoints(72,92,frame,face)#outer Lip (72->91)
            connectPoints(92,128,frame,face)#face boundary (92->127)

            cv2.circle(frame,face[128],3,yellow,-1)#left pupil (centre->128,adjacent->129)
            rl=findRadius(face[128],face[129])#left iris radius
            cv2.circle(frame,face[128],int(rl),blue,1)

            cv2.circle(frame,face[133],3,yellow,-1)#right pupil (centre->133,adjacent->134)
            rr=findRadius(face[133],face[134])#right iris radius
            cv2.circle(frame,face[133],int(rr),blue,1)

            #face axis
            face_L = (face[119][0], face[119][1])
            face_R = (face[101][0], face[101][1])
            face_B = (face[110][0], face[110][1])
            face_T = (face[92][0], face[92][1])
            cv2.line(frame, face_L, face_R, (100, 100, 100), 2)
            cv2.line(frame, face_T, face_B, (100, 100, 100), 2)
            center_point_v = [int((face_B[0]+face_T[0])/2) ,int((face_B[1]+face_T[1])/2)]
            center_point_h = [int((face_L[0]+face_R[0])/2) ,int((face_L[1]+face_R[1])/2)]
            # cv2.putText(frame, "vc", (center_point_v[0],center_point_v[1]), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)
            # cv2.putText(frame, "hc", (center_point_h[0],center_point_h[1]), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)
            #

            # head roll
            # cv2.line(frame, face[92], face[110], green,1)
            # cv2.line(frame, face[119], face[101], green,1)
            # #find the line coefficients
            # x1 = [face[110][0], face[92][0]]
            # y1 = [face[110][1], face[92][1]]
            # x2 = [face[101][0], face[119][0]]
            # y2 = [face[101][1], face[119][1]]
            # px,py = intersection_point(x1,y1,x2,y2)
            # cv2.putText(frame, "c", (int(face[129][0]),int(face[129][1])), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)
            # cv2.putText(frame, "b", (x1[0],y1[0]), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)
            # cv2.putText(frame, "t", (x1[1],y1[1]), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)
            # cv2.putText(frame, "r", (x2[0],y2[0]), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)
            # cv2.putText(frame, "l", (x2[1],y2[1]), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)
            #

            p1 = np.array([float(face_T[0]),float(face_T[1])] )
            p2 = np.array([float(face_B[0]),float(face_B[1])] )
            p3 = np.array([float(face_L[0]),float(face_L[1])] )
            p4 = np.array([float(face_R[0]),float(face_R[1])] )

            inter_point = seg_intersect(p1, p2, p3, p4)


            # head roll
            var_x_rool = (face[110][0]-face[92][0])
            var_y_rool = (face[110][1]-face[92][1])
            if not var_y_rool == 0:
                head_roll = math.atan(var_x_rool/var_y_rool)
            else:
                head_roll = 0
            head_roll = numpy.rad2deg(head_roll)
            head_roll = math.floor(head_roll)
            if abs(head_roll-previous_head_roll) < 1.5:
                head_roll = previous_head_roll
            previous_head_roll = head_roll
            # cv2.putText(frame, str(head_roll), face[92], cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)

            # head yaw
            # left_length_vc = abs(face_L[0]-center_point_v[0])
            # right_length_vc = abs(face_R[0]-center_point_v[0])
            # max_horizontal_distance = abs(face_L[0]-face_R[0])
            right_length_vc = distance_between_two_point(face_R,inter_point)
            max_horizontal_distance = distance_between_two_point(face_L,face_R)
            r1 = [0,max_horizontal_distance ]
            r2 = [-80, 80]
            head_yaw = transformRange(right_length_vc, r1,r2)
            head_yaw = math.floor(head_yaw)
            if abs(head_yaw-previous_head_yaw) < 2:
                head_yaw = previous_head_yaw
            previous_head_yaw= head_yaw
            # cv2.putText(frame, str(head_yaw), face_R, cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)

            #head pitch
            # top_lenght_hc = abs(face_T[0]-center_point_h[1])
            # max_vertical_distance = abs(face_T[1]-face_B[0])

            cv2.putText(frame, "o", (int(inter_point[0]), int(inter_point[1])), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)

            top_lenght_hc = distance_between_two_point(face_T,inter_point)
            max_vertical_distance = distance_between_two_point(face_T, face_B)
            r1_ = [max_vertical_distance,0 ]
            r2_ = [-80, 80]
            head_pitch = transformRange(top_lenght_hc, r1_,r2_)
            head_pitch = math.floor(head_pitch)
            if abs(head_pitch-previous_head_pitch) < 2:
                head_pitch = previous_head_pitch
            previous_head_pitch= head_pitch
            # head_pitch+=18
            rpy = "rpy head: [ " + str(head_roll) + "," + str(head_pitch) + "," +str(head_yaw) + " ]"
            cv2.putText(frame, rpy, face_B, cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255), 2)

            # eyes
            cv2.line(frame,face[24],face[32],green,1)
            cv2.line(frame,face[40],face[48],blue,1)

            cv2.line(frame,face[57],face[67],blue,1)

            mouth_distance = distance_between_two_point(face[57], face[67])
            l_eye_distance = distance_between_two_point(face[40], face[48])
            r_eye_distance = distance_between_two_point(face[24], face[32])


            mouth_open = True
            left_eye_open = True
            right_eye_open = True

            if mouth_distance < 8:
                mouth_open = False
            if l_eye_distance < 5:
                left_eye_open = False
            if r_eye_distance < 5:
                right_eye_open = False
            m_string = "Mouth " + openorclose(mouth_open)
            le_string = "Left eye " + openorclose(left_eye_open)
            re_string = "Right eye " + openorclose(right_eye_open)

            cv2.putText(frame, m_string, (20, 60), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(frame, le_string, (20, 80), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(frame, re_string, (20, 100), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255), 2)

            #mode 1 to 2
            if abs(head_roll) > 25 and ( not (right_eye_open)):
                if mode ==1:
                    ModeString = "Mode 2"
                    mode = 2
        #mode 2 to 1
            if abs(head_roll) > 25 and ( not (left_eye_open)):
                if mode ==2:
                    ModeString = "Mode 1"
                    mode = 1
                    x_cursor = size[1]/2
                    y_cursor = size[0]/2
            # control cursor
            if mode == 2:
                if not mouth_open :
                # x cursor
                    disr = distance_between_two_point(face[44], face[36])
                    disl = distance_between_two_point(face[20], face[28])
                    l_pupil_dist_extrem = distance_between_two_point(face[128], face[28])
                    r_pupil_dist_extrem = distance_between_two_point(face[133], face[36])
                    ratiol = l_pupil_dist_extrem/(disl/2)
                    ratior = r_pupil_dist_extrem/(disr/2)
                    main_ratio = ratiol
                    if main_ratio < ratior:
                        main_ratio = ratior
                    main_ratio = main_ratio *10

                    x_inc =10
                    main_ratio = math.ceil(main_ratio)
                    if main_ratio < 8:
                            x_cursor = x_cursor - x_inc
                    if main_ratio > 13:
                            x_cursor = x_cursor + x_inc
                    if x_cursor < 5:
                        x_cursor = 5
                    if x_cursor > (600):
                        x_cursor = (600)
                    # print(x_cursor)
                    # print((size[0]-5))
                    # y cursor
                    disr_up = distance_between_two_point(face[40], face[48])
                    disl_up = distance_between_two_point(face[24], face[31])
                    l_pupil_dist_extrem_up = distance_between_two_point(face[128], face[39])
                    r_pupil_dist_extrem_up = distance_between_two_point(face[133], face[48])
                    if(disl_up > 0):
                        ratiol_up = l_pupil_dist_extrem_up/(disl_up/2)
                    else:
                        ratiol_up = 0
                    if(disr_up > 0):
                        ratior_up = r_pupil_dist_extrem_up/(disr_up/2)
                    else:
                        ratiol_up = 0
                    main_ratio_up = ratiol_up
                    if main_ratio_up < ratior_up:
                        main_ratio_up = ratior_up
                    main_ratio_up = main_ratio_up
                    main_ratio_up = math.ceil(main_ratio_up)
                    # cv2.putText(frame, str(main_ratio_up), (int(255), int(255)), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 255, 255), 2)

                    #
                    y_inc =5
                    if main_ratio_up < 12:
                            y_cursor = y_cursor - 10
                    if main_ratio_up > 18:
                            y_cursor = y_cursor + 10
                    if y_cursor < 5:
                        y_cursor = 5
                    if y_cursor > (480):
                        y_cursor = (480)
                cv2.putText(frame, "+", (int(x_cursor), int(y_cursor)), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 255, 255), 2)

            # emergency_stop intentionally done by the patient
            if abs(head_roll) > 45:
                if mouth_open:
                    emergency_stop = True
            both_eye_closed = not (left_eye_open and right_eye_open)
            timer_eyes_close_counter = 0
            if both_eye_closed:
                if not previous_both_eye_close:
                    start_e = time.time()
                else:
                    end_e = time.time()
                    timer_eyes_close_counter = end_e-start_e
                    if(end_e-start_e) > 3:
                        emergency_stop = True


            previous_both_eye_close = both_eye_closed
            string_emer_eye_closed = "Eyes closed for:  " + str(math.floor(timer_eyes_close_counter))
            cv2.putText(frame, string_emer_eye_closed, (400, 120), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255), 2)



            vision_command_msg.x_cursor = int(x_cursor)
            vision_command_msg.y_cursor = int(y_cursor)
            vision_command_msg.mode = mode
            vision_command_msg.open_mouth = mouth_open
            vision_command_msg.roll = head_roll
            vision_command_msg.pitch = head_pitch
            vision_command_msg.yaw = head_yaw
        vision_command_msg.emergency_stop = emergency_stop
        vision_command_pub.publish(vision_command_msg)
        cv2.imshow('Webcam', frame)
        if cv2.waitKey(1) & 0xff == ord('q'): # to quit the camera press 'q'
            print('end')
            break
    cam.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        vision_computation()
    except rospy.ROSInterruptException:
        pass
