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
fst = False
# hsv цветов
low_black=np.array([0,0,0]) # HSV чёрного
up_black=np.array([70,255,27])

low_red=np.array([0,168,105]) # HSV красного
up_red=np.array([13,255,131])

low_blue=np.array([93,108,56]) # HSV синего
up_blue=np.array([128,255,139])

low_orange=np.array([0,110,100]) # HSV оранжевого
up_orange=np.array([20,255,182])

low_green=np.array([74, 168, 67]) # HSV зелёного
up_green=np.array([99, 255, 145])

port = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)
robot = rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)

direction = "None" # направление движения

vrm = [0, 0, 0, 0]
vi = 0
message = ""
fps = 0 # показатель fps
fps1 = 0
fps_time = 0

art = 0


e_old = 0 # ошибка старая
kp = 0.15 # коэфицент 
kd = 0.15 # коэфицент

d1 = 0 
d2 = 0
d1old = 0
d2old = 0
d1t = 0
d2t = 0

color = 'Green' # цвет обнаруженного знака
line1 = 'Orange' # цвет обнаруженной линии на трассе
lap = 0 # количество пройденных кругов

speed = 0 # скорость
serv = 0 # угол поворота серво мотора
rgb = "111" # сообщение передающееся на Raspberry
inn = ""

def line(frame):# распознаваие стен
    global d1, d2, d1old, d2old, d1t, d2t
    x3 = 0 # размеры области интереса на камере
    x4 = 170
    y3 = 200
    y4 = 280
    dat = frame[y3:y4, x3:x4] # создание области интереса
    hsv = cv2.cvtColor(dat, cv2.COLOR_BGR2HSV) # преобразует BGR в HSV
    mask = cv2.blur(cv2.inRange(hsv, low_black, up_black), (3, 3)) # создание маски
    imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # находим на маске контуры
    xe, ye, we, he = 0, 0, 0, 0
    d1 = 0
    for contor in contours:
        x, y, w, h = cv2.boundingRect(contor) # выделение найденых стен
        a = cv2.contourArea(contor)
        if a > 200 and x + w > xe + we:
            xe, ye, we, he = x, y, w, h
    d1 = xe + we
    cv2.rectangle(dat, (xe, ye), (xe + we, ye + he), (255, 255, 0), 2) # обводка для найденых стен
    cv2.rectangle(frame, (x3, y3), (x4, y4), (255, 0, 0), 2) # отображение области интереса
    if d1 == 0:
        if time.time() - d1t <= 0.05:
            d1 = d1old # запоминаем значения
    else:
        d1old = d1 # запоминаем значение
        d1t = time.time()



    x5 = 470 # размеры области интереса на камере
    x6 = 640
    y5 = 200
    y6 = 280

    dat = frame[y5:y6, x5:x6] # создание области интереса
    hsv = cv2.cvtColor(dat, cv2.COLOR_BGR2HSV) # преобразует BGR в HSV
    mask = cv2.blur(cv2.inRange(hsv, low_black, up_black), (3, 3))# создание маски
    imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # находим на маске контуры
    xr, yr, wr, hr = 150, 0, 0, 0
    d2 = 0
    for contor in contours:
        x, y, w, h = cv2.boundingRect(contor) # выделение найденых стен
        a = cv2.contourArea(contor)
        if a > 200 and 150 - x> 150 - xr:
            xr, yr, wr, hr = x, y, w, h
    d2 = 150 - xr
    cv2.rectangle(dat, (xr, yr), (xr + wr, yr + hr), (255, 255, 0), 2) # обводка для найденых стен
    cv2.rectangle(frame, (x5, y5), (x6, y6), (255, 0, 0), 2) # отображение области интереса
    if d2 == 0:
        if time.time() - d2t <= 0.05:
            d2 = d2old # запоминаем значения
    else:
        d2old = d2 # запоминаем значения
        d2t = time.time()

def povorot(frame):# распознавание оранжевых и синих линий на трассе
    global d1,d2,lap,tp, direction, vi, vrm, art, rgb
    x7 = 280 # размеры области интереса 
    x8 = 340
    y7 = 440
    y8 = 480
    dat = frame[y7:y8, x7:x8]
    hsv = cv2.cvtColor(dat, cv2.COLOR_BGR2HSV) # преобразует BGR в HSV

    if direction == "None":
        mask = cv2.inRange(hsv, low_orange, up_orange) # создание маски для оранжевого
        imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # находим на маске контуры
        xx1, yy1, ww1, hh1 = 0, 0, 0, 0
        for contor in contours:
            x, y, w, h = cv2.boundingRect(contor) # выделение найденных линий на трассе
            if h * w > 50 and h * w > hh1 * ww1:
                cv2.rectangle(dat, (x, y), (x + w, y + h), (0, 255, 0), 2) # обводка найденных линий
                xx1, yy1, ww1, hh1 = x, y, w, h
        cv2.rectangle(dat, (xx1, yy1), (xx1 + ww1, yy1 + hh1), (0, 255, 255), 2) # рисуем область интереса

        mask = cv2.inRange(hsv, low_blue, up_blue) # создание маски для синего
        imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # находим на маске контуры
        xx2, yy2, ww2, hh2 = 0, 0, 0, 0
        for contor in contours:
            x, y, w, h = cv2.boundingRect(contor) # выделениенайденных линий на трассе
            if h * w > 50 and h * w > hh2 * ww2:
                cv2.rectangle(dat, (x, y), (x + w, y + h), (0, 255, 0), 2) # обводка найденных линий
                xx2, yy2, ww2, hh2 = x, y, w, h
        cv2.rectangle(dat, (xx2, yy2), (xx2 + ww2, yy2 + hh2), (0, 255, 255), 2) # рисуем область интереса

        if hh1>0 or hh2 > 0:# счёт кругов пройденных роботом
            if tp + 0.5 < time.time():

                lap += 1
                if hh1 > 0:# распознавание направления движения
                    direction = "orange" # направление движения
                    rgb = "110" # сообщение для светодиода
                if hh2 > 0:
                    direction = "blue" # направление движения
                    rgb = "001" # сообщение для светодиода
                vrm[vi] = round(time.time() - tp, 2)
                vi += 1
                tp = time.time()
                art = time.time()


    elif direction == "orange":
        mask = cv2.inRange(hsv, low_orange, up_orange) # создание маски
        imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # находим на маске контуры
        xx1, yy1, ww1, hh1 = 0, 0, 0, 0
        for contor in contours:
            x, y, w, h = cv2.boundingRect(contor) # выделение контуров
            if h * w > 50 and h * w > hh1 * ww1:
                cv2.rectangle(dat, (x, y), (x + w, y + h), (0, 255, 0), 2) # обводка найденных контуров прямоугольником
                xx1, yy1, ww1, hh1 = x, y, w, h
        cv2.rectangle(dat, (xx1, yy1), (xx1 + ww1, yy1 + hh1), (0, 255, 255), 2)
        if hh1 > 0: # защита от двойного счёта кругов на 1 линии
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
        mask = cv2.inRange(hsv, low_blue, up_blue) # создание маски
        imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # находим контуры на маске
        xx2, yy2, ww2, hh2 = 0, 0, 0, 0
        for contor in contours:
            x, y, w, h = cv2.boundingRect(contor) # выделение контуров
            if h * w > 50 and h * w > hh2 * ww2:
                cv2.rectangle(dat, (x, y), (x + w, y + h), (0, 255, 0), 2) # обводим найденные контуры прямоугольником
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


    cv2.rectangle(frame, (x7, y7), (x8, y8), (255, 0, 0), 2) # рисуем область интереса

def PD():# ПД регулятор для движения по трассе
    global e_old, d1, d2
    e = d2 - d1
    Up = e * kp
    Ud = (e - e_old) * kd
    e_old = e
    U = Up + Ud
    return U


while 1:# цикл
    key = robot.get_key()

    # запуск функций
    frame = robot.get_frame(wait_new_frame=1)
    line(frame)
    povorot(frame)
    if art + 0.5 <= time.time() and fst:
        rgb = "000"

    serv = 0 # осуществление поворотов
    if speed > 0:
        serv = PD() # едем по ПД регулятору
        if direction == "blue" and d1 == 0: # поварачиваем 
            serv = 20
        if direction == "orange" and d2 == 0:
            serv = -20
        if d1 == 0 and d2 == 0 and direction == "blue":
            serv = 20
        if d1 == 0 and d2 == 0 and direction == "orange":
            serv = -20

    if lap == 12:# остановка на финише
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
            speed = 100
        else:
            speed = 0

    fps1 += 1 # считаем fps
    if time.time() > fps_time + 1:
        fps_time = time.time()
        fps = fps1
        fps1 = 0
    cv2.rectangle(frame, (0, 0), (0 + 160, 0 + 105), (0, 0, 0), -1) # рисуем прямоугольники на камере
    cv2.rectangle(frame, (440, 0), (440 + 200, 0 + 100), (0, 0, 0), -1)
    robot.text_to_frame(frame, message, 20, 20) # выводим на камеру информацию
    robot.text_to_frame(frame, inn, 20, 40)
    robot.text_to_frame(frame, 'fps = ' + str(fps), 500, 20)
    robot.text_to_frame(frame, 'd1 = ' + str(d1), 20, 60)
    robot.text_to_frame(frame, 'd2 = ' + str(d2), 20, 80)
    robot.text_to_frame(frame, 'color = ' + str(color), 450, 40)
    robot.text_to_frame(frame, 'line = ' + str(direction), 450, 60)
    robot.text_to_frame(frame, 'lap = ' + str(lap), 500, 80)

    cv2.rectangle(frame, (200, 0), (400, 100), (0, 0, 0), -1)
    robot.text_to_frame(frame, 'lap1 = ' + str(vrm[0]), 220, 20)
    robot.text_to_frame(frame, 'lap2 = ' + str(vrm[1]), 220, 40)
    robot.text_to_frame(frame, 'lap3 = ' + str(vrm[2]), 220, 60)
    robot.text_to_frame(frame, 'lap4 = ' + str(vrm[3]), 220, 80)

    robot.set_frame(frame, 40)
