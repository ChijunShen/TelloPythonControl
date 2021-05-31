import logging
import time
import cv2
import numpy as np
from djitellopy import tello
import KeyPressModule as kp
from time import sleep

def findFace(img):
    faceCascade = cv2.CascadeClassifier("Resources/haarcascade_frontalface_default.xml")
    faceCascade.load("Resources/haarcascade_frontalface_default.xml")
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(imgGray, 1.3, 8)
    myFaceListC = []
    myFaceListArea = []

    for (x, y, w, h) in faces:
        cv2.rectangle(img,(x,y),(x+w, y+h),(0, 0, 255),2)
        cx = x + w//2
        cy = y + h//2
        area = w*h
        cv2.circle(img, (cx, cy), 5, (255, 0, 0), cv2.FILLED)
        myFaceListC.append([cx,cy])
        myFaceListArea.append(area)
    if len(myFaceListArea) != 0:
        i = myFaceListArea.index(max(myFaceListArea))
        InfoText = "Area:{0} X:{1} Y:{2}".format(area , cx, cy)
        cv2.putText(img, InfoText, (cx+20, cy), font, fontScale, fontColor, lineThickness)
        return img, [myFaceListC[i], myFaceListArea[i]]
    else:
        return img, [[0, 0], 0]

def trackFace(me, img, info, w, h, pid, DetectRange, pErrorRotate, pErrorUp):
    area = info[1]
    x, y = info[0][0], info[0][1]
    fb = 0
    ErrorRotate = x - w/2
    ErrorUp = h/2 - y
    cv2.circle(img, (int(w/2), int(h/2)), 5, (0, 255, 0), cv2.FILLED)
    if x >10 or y >10:
        cv2.line(img, (int(w/2), int(h/2)), (x,y), (255, 0, 0), lineThickness)
    rotatespeed = pid[0]*ErrorRotate + pid[1]*(ErrorRotate - pErrorRotate)
    updownspeed = pid[0]*ErrorUp + pid[1]*(ErrorUp - pErrorUp)
    rotatespeed = int(np.clip(rotatespeed, -40, 40))
    updownspeed = int(np.clip(updownspeed, -60, 60))

    area = info[1]
    if area > DetectRange[0] and area < DetectRange[1]:
        fb = 0
        # updownspeed = 0
        # rotatespeed = 0
        InfoText = "Hold Speed:{0} Rotate:{1} Up:{2}".format(fb, rotatespeed, updownspeed)
        cv2.putText(img, InfoText, (10, 60), font, fontScale, fontColor, lineThickness)
        me.send_rc_control(0, fb, updownspeed, rotatespeed)
    elif area > DetectRange[1]:
        fb = -20
        InfoText = "Backward Speed:{0} Rotate:{1} Up:{2}".format(fb, rotatespeed, updownspeed)
        cv2.putText(img, InfoText, (10, 60), font, fontScale, fontColor, lineThickness)
        me.send_rc_control(0, fb, updownspeed, rotatespeed)
    elif area < DetectRange[0] and area > 1000:
        fb = 20
        InfoText = "Forward Speed:{0} Rotate:{1} Up:{2}".format(fb, rotatespeed, updownspeed)
        cv2.putText(img, InfoText, (10, 60), font, fontScale, fontColor, lineThickness)
        me.send_rc_control(0, fb, updownspeed, rotatespeed)
    else:
        me.send_rc_control(0, 0, 0, 0)

    if x == 0:
        speed = 0
        error = 0
    return ErrorRotate, ErrorUp


def getKeyboardInput(drone, speed, image):
    lr, fb, ud, yv = 0, 0, 0, 0
    key_pressed = 0
    if kp.getKey("e"):
        cv2.imwrite('D:/snap-{}.jpg'.format(time.strftime("%H%M%S", time.localtime())), image)
    if kp.getKey("UP"):
        Drone.takeoff()
    elif kp.getKey("DOWN"):
        Drone.land()

    if kp.getKey("j"):
        key_pressed = 1
        lr = -speed
    elif kp.getKey("l"):
        key_pressed = 1
        lr = speed

    if kp.getKey("i"):
        key_pressed = 1
        fb = speed
    elif kp.getKey("k"):
        key_pressed = 1
        fb = -speed

    if kp.getKey("w"):
        key_pressed = 1
        ud = speed
    elif kp.getKey("s"):
        key_pressed = 1
        ud = -speed

    if kp.getKey("a"):
        key_pressed = 1
        yv = -speed
    elif kp.getKey("d"):
        key_pressed = 1
        yv = speed
    InfoText = "battery : {0}% height: {1}cm   time: {2}".format(drone.get_battery(), drone.get_height(), time.strftime("%H:%M:%S",time.localtime()))
    cv2.putText(image, InfoText, (10, 20), font, fontScale, (0, 0, 255), lineThickness)
    if key_pressed == 1:
        InfoText = "Command : lr:{0}% fb:{1} ud:{2} yv:{3}".format(lr, fb, ud, yv)
        cv2.putText(image, InfoText, (10, 40), font, fontScale, (0, 0, 255), lineThickness)

    drone.send_rc_control(lr, fb, ud, yv)




# Main Program

# Camera Setting
Camera_Width = 720
Camera_Height = 480
DetectRange = [6000, 11000]  # DetectRange[0] 是保持静止的检测人脸面积阈值下限，DetectRange[0] 是保持静止的检测人脸面积阈值上限
PID_Parameter = [0.5, 0.0004, 0.4]
pErrorRotate, pErrorUp = 0, 0

# Font Settings
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 0.5
fontColor = (255, 0, 0)
lineThickness = 1

# Tello Init
Drone = tello.Tello()  # 创建飞行器对象
Drone.connect()  # 连接到飞行器
Drone.streamon()  # 开启视频传输
Drone.LOGGER.setLevel(logging.ERROR)  # 只显示错误信息
sleep(5)  #  等待视频初始化
kp.init()  # 初始化按键处理模块


while True:
    OriginalImage = Drone.get_frame_read().frame
    Image = cv2.resize(OriginalImage, (Camera_Width, Camera_Height))
    getKeyboardInput(drone=Drone, speed=70, image=Image)  # 按键控制
    #  img, info = findFace(Image)
    # pErrorRotate, pErrorUp = trackFace(Drone, img, info, Camera_Width, Camera_Height, PID_Parameter, DetectRange, pErrorRotate, pErrorUp)
    cv2.imshow("Drone Control Centre", Image)
    cv2.waitKey(1)


