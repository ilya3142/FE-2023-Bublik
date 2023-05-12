import cv2
import RobotAPI as rapi
import numpy as np
import serial
import time
from array import *

tim_s = time.time()
tim_l = time.time()
tp = time.time()
ts = time.time()
tz = time.time()
tr = time.time()
tg = time.time()

fst = False

low_black=np.array([0,0,0]) # HSV чёрного
up_black=np.array([70,255,27])

low_red=np.array([0,130,0]) # HSV красного
up_red=np.array([5,220,255])

low_blue=np.array([93,108,56]) # HSV синего
up_blue=np.array([128,255,139])

low_orange=np.array([0,110,100]) # HSV оранжеаого
up_orange=np.array([20,255,182])

low_green=np.array([74, 168, 67]) # HSV зелёного
up_green=np.array([99, 255, 145])

port = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)
robot = rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)

direction = "None" # направление движения

vrm= [0, 0, 0, 0]
vi = 0
message = ""
fps = 0
fps1 = 0
fps_time = 0

art = 0

x_zn = 0
y_zn = 0
w_zn = 0
h_zn = 0

r_zn = 0
g_zn = 0

zn_old = 0
zn_t = 0
wzn_old = 0


e_old = 0
e_old_zn = 0

kp = 0.15
kd = 0.15

d1 = 0
d2 = 0
d1old = 0
d2old = 0
d1t = 0
d2t = 0

color = 'Green'
line1 = 'Orange'
lap = 0

speed = 0 # скорость
serv = 0 # угол поворота сервомотора
rgb = "111" # сообщение для светодиода
inn = ""


xleft = 160
xright = 470

def znak(frame): # распознавание знаков
    global x_zn, y_zn, h_zn, w_zn,color, tz, xleft, xright, zn_t, zn_old, wzn_old
    x_zn, y_zn, h_zn, w_zn = 0, 0, 0, 0
    x1 = 80 # область интереса
    x2 = 560
    y1 = 210
    y2 = 400
    dat = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(dat, cv2.COLOR_BGR2HSV) # преобразование BGR в HSV


    mask = cv2.inRange(hsv, low_red, up_red) # создание маски
    imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # находим на маске контуры
    xx, yy, ww, hh = 0,0,0,0
    for contor in contours:
        x, y, w, h = cv2.boundingRect(contor) # обводим контуры
        if h*w > 100 and h*w > hh*ww:
            xx, yy, ww, hh = x,y,w,h




    mask1 = cv2.inRange(hsv, low_green, up_green) # создание маски
    imd, contours, hod = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # находим контуры на маске
    xq, yq, wq, hq = 0, 0, 0, 0
    for contor in contours:
        x, y, w, h = cv2.boundingRect(contor) # обводим контуры
        if h*w > 100 and h*w > hq*wq:
            xq, yq, wq, hq = x,y,w,h


    color = 'None'

    if hh > 0 or hq > 0: # изменение областей интересов для распознавания стен

        if yy + hh > yq + hq: # объезд знаков
            cv2.rectangle(dat, (xx, yy), (xx + ww, yy + hh), (0, 255, 255), 2) # обводим найденные когтуры
            x_zn, y_zn, w_zn, h_zn = xx, yy, ww, hh
            color = 'Red'
            if direction == "blue": # направление движения
                xleft = 30

        else:
            cv2.rectangle(dat, (xq, yq), (xq + wq, yq + hq), (255, 0, 255), 2) # обводим найденные контуры
            x_zn, y_zn, w_zn, h_zn = xq, yq, wq, hq
            color = 'Green'
            if direction == "orange": # направление движения
                xright = 610

        tz = time.time()
        zn_old = x_zn
        wzn_old = w_zn

    else:
        if zn_t + 0.5 > time.time():
            x_zn = zn_old
            w_zn = wzn_old

        if tz + 0.5 < time.time():
            xleft = 160
            xright = 470


def line(frame): # распознаваие стен
    global d1, d2, d1old, d2old, d1t, d2t, xleft, xright
    x3 = 0
    x4 = xleft
    y3 = 225
    y4 = 255
    dat = frame[y3:y4, x3:x4]
    hsv = cv2.cvtColor(dat, cv2.COLOR_BGR2HSV)
    mask = cv2.blur(cv2.inRange(hsv, low_black, up_black), (3, 3))
    imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    xe, ye, we, he = 0, 0, 0, 0
    d1 = 0
    for contor in contours:
        x, y, w, h = cv2.boundingRect(contor)
        a = cv2.contourArea(contor)
        if a > 200 and x + w > xe + we:
            xe, ye, we, he = x, y, w, h
    d1 = xe + we
    cv2.rectangle(dat, (xe, ye), (xe + we, ye + he), (255, 255, 0), 2)
    cv2.rectangle(frame, (x3, y3), (x4, y4), (255, 0, 0), 2)
    if d1 == 0:
        if time.time() - d1t <= 0.05:
            d1 = d1old
    else:
        d1old = d1
        d1t = time.time()

    x5 = xright
    x6 = 640
    y5 = 225
    y6 = 255

    dat = frame[y5:y6, x5:x6]
    hsv = cv2.cvtColor(dat, cv2.COLOR_BGR2HSV)
    mask = cv2.blur(cv2.inRange(hsv, low_black, up_black), (3, 3))
    imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    xr, yr, wr, hr = 150, 0, 0, 0
    d2 = 0
    for contor in contours:
        x, y, w, h = cv2.boundingRect(contor)
        a = cv2.contourArea(contor)
        if a > 200 and 150 - x> 150 - xr:
            xr, yr, wr, hr = x, y, w, h
    d2 = 150 - xr
    cv2.rectangle(dat, (xr, yr), (xr + wr, yr + hr), (255, 255, 0), 2)

    cv2.rectangle(frame, (x5, y5), (x6, y6), (255, 0, 0), 2)
    if d2 == 0:
        if time.time() - d2t <= 0.05:
            d2 = d2old
    else:
        d2old = d2
        d2t = time.time()

def povorot(frame): # распознавание оранжевых и синих линий на трассе
    global d1,d2,lap,tp, direction, vi, vrm, art, rgb
    x7 = 280
    x8 = 340
    y7 = 440
    y8 = 480
    dat = frame[y7:y8, x7:x8]
    hsv = cv2.cvtColor(dat, cv2.COLOR_BGR2HSV)

    if direction == "None":
        mask = cv2.inRange(hsv, low_orange, up_orange)
        imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        xx1, yy1, ww1, hh1 = 0, 0, 0, 0
        for contor in contours:
            x, y, w, h = cv2.boundingRect(contor)
            if h * w > 50 and h * w > hh1 * ww1:
                cv2.rectangle(dat, (x, y), (x + w, y + h), (0, 255, 0), 2)
                xx1, yy1, ww1, hh1 = x, y, w, h
        cv2.rectangle(dat, (xx1, yy1), (xx1 + ww1, yy1 + hh1), (0, 255, 255), 2)

        mask = cv2.inRange(hsv, low_blue, up_blue)
        imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        xx2, yy2, ww2, hh2 = 0, 0, 0, 0
        for contor in contours:
            x, y, w, h = cv2.boundingRect(contor)
            if h * w > 50 and h * w > hh2 * ww2:
                cv2.rectangle(dat, (x, y), (x + w, y + h), (0, 255, 0), 2)
                xx2, yy2, ww2, hh2 = x, y, w, h
        cv2.rectangle(dat, (xx2, yy2), (xx2 + ww2, yy2 + hh2), (0, 255, 255), 2)

        if hh1>0 or hh2 > 0: # счёт кругов пройденных роботом
            if tp + 0.5 < time.time():

                lap += 1
                if hh1 > 0: # распознавание направления движения
                    direction = "orange"
                    rgb = "110"
                if hh2 > 0:
                    direction = "blue"
                    rgb = "001"
                vrm[vi] = round(time.time() - tp, 2)
                vi += 1
                tp = time.time()
                art = time.time()


    elif direction == "orange":
        mask = cv2.inRange(hsv, low_orange, up_orange)
        imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        xx1, yy1, ww1, hh1 = 0, 0, 0, 0
        for contor in contours:
            x, y, w, h = cv2.boundingRect(contor)
            if h * w > 50 and h * w > hh1 * ww1:
                cv2.rectangle(dat, (x, y), (x + w, y + h), (0, 255, 0), 2)
                xx1, yy1, ww1, hh1 = x, y, w, h
        cv2.rectangle(dat, (xx1, yy1), (xx1 + ww1, yy1 + hh1), (0, 255, 255), 2)
        if hh1 > 0:
            if tp + 0.5 < time.time():
                rgb = "110"
                art = time.time()

                lap += 1
                vrm[vi] = round(time.time() - tp, 2)
                vi += 1
                if vi > 3:
                    vi = 0
                tp = time.time()

    else:
        mask = cv2.inRange(hsv, low_blue, up_blue)
        imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        xx2, yy2, ww2, hh2 = 0, 0, 0, 0
        for contor in contours:
            x, y, w, h = cv2.boundingRect(contor)
            if h * w > 50 and h * w > hh2 * ww2:
                cv2.rectangle(dat, (x, y), (x + w, y + h), (0, 255, 0), 2)
                xx2, yy2, ww2, hh2 = x, y, w, h
        cv2.rectangle(dat, (xx2, yy2), (xx2 + ww2, yy2 + hh2), (0, 255, 255), 2)

        if hh2 > 0:
            if tp + 0.5 < time.time():
                rgb = "001"
                art = time.time()
                lap += 1
                vrm[vi] = round(time.time() - tp, 2)
                vi += 1
                if vi > 3:
                    vi = 0
                tp = time.time()


    cv2.rectangle(frame, (x7, y7), (x8, y8), (255, 0, 0), 2)

def PD(): # ПД регулятор для движения по трассе
    global e_old, d1, d2
    kp = 0.15
    kd = 0.15
    e = d2 - d1
    Up = e * kp
    Ud = (e - e_old) * kd
    e_old = e
    U = Up + Ud
    return U

def PD_zn(): # ПД регулятор для движения по трассе с определением знаков
    global e_old_zn, x_zn, y_zn, w_zn, h_zn, color, tg, tr
    kp = 0.15
    kd = 0.15
    prib = (y_zn+h_zn)*1.1
    if color == "Red":
        e = (220 - prib) - (x_zn + w_zn)
    if color == "Green":
        e = (260 + prib) - x_zn
    Up = e * kp
    Ud = (e - e_old_zn) * kd
    e_old_zn = e
    U = Up + Ud
    return U


while 1: # цикл
    key = robot.get_key()
    frame = robot.get_frame(wait_new_frame=1)
    # запуск функций
    znak(frame)
    line(frame)
    povorot(frame)
    if art + 0.5 <= time.time() and fst:
        rgb = "000"

    serv = 0
    if speed > 0:
        if h_zn > 0:
            serv = PD_zn()
        else:
            serv = PD()
            if direction == "blue" and d1 == 0:
                serv = 40
            if direction == "orange" and d2 == 0:
                serv = -40
            if d1 == 0 and d2 == 0 and direction == "blue":
                serv = 40
            if d1 == 0 and d2 == 0 and direction == "orange":
                serv = -40

    if lap == 12: # остановка на финише
        if ts + vrm[0]*0.6 < time.time():
            speed = 0
    else:
        ts=time.time()
    # отправка сообщений RaspBerry
    message = str(int(speed) + 200) + str(int(serv) + 200) + rgb+'$'
    port.write(message.encode("utf-8"))


    if port.in_waiting > 0:
        inn = ""
        t = time.time()
        while 1:
            a = str(port.read(), "utf-8")
            if a != "$":
                inn += a
            else:
                break
            if t + 0.02 < time.time():
                break
        port.reset_input_buffer()

    if inn == "0" and tim_s + 1 < time.time():
        tim_s = time.time()
        if speed == 0:
            fst = True
            speed = 80
        else:
            speed = 0

    if lap >= 12:
        if tim_l + 5 < time.time():
            speed = 0
    else:
        tim_l = time.time()

    fps1 += 1
    if time.time() > fps_time + 1:
        fps_time = time.time()
        fps = fps1
        fps1 = 0
    x1 = 80
    x2 = 560
    y1 = 210
    y2 = 400
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
    cv2.rectangle(frame, (0, 0), (0 + 160, 0 + 105), (0, 0, 0), -1)
    cv2.rectangle(frame, (440, 0), (440 + 200, 0 + 100), (0, 0, 0), -1)
    robot.text_to_frame(frame, message, 20, 20)
    robot.text_to_frame(frame, inn, 20, 40)
    robot.text_to_frame(frame, 'fps = ' + str(fps), 500, 20)
    robot.text_to_frame(frame, 'd1 = ' + str(d1), 20, 60)
    robot.text_to_frame(frame, 'd2 = ' + str(d2), 20, 80)
    robot.text_to_frame(frame, 'color = ' + str(color), 450, 40)
    robot.text_to_frame(frame, 'line = ' + str(direction), 450, 60)
    robot.text_to_frame(frame, 'lap = ' + str(lap), 500, 80)

    cv2.rectangle(frame, (200, 0), (400, 120), (0, 0, 0), -1)
    robot.text_to_frame(frame, 'lap1 = ' + str(vrm[0]), 220, 20)
    robot.text_to_frame(frame, 'lap2 = ' + str(vrm[1]), 220, 40)
    robot.text_to_frame(frame, 'lap3 = ' + str(vrm[2]), 220, 60)
    robot.text_to_frame(frame, 'lap4 = ' + str(vrm[3]), 220, 80)
    robot.text_to_frame(frame, 'xw = ' + str(x_zn+w_zn), 220, 100)

    robot.set_frame(frame, 40)
