from pyb import delay, Pin, ADC, Timer, UART, LED
from machine import UART
uart = UART(6, 115200, stop=1)
inn = ""
speed = 0
serv = 0
r = 0
g = 0
b = 0
sv_r = Pin('Y8', Pin.OUT_PP)
sv_b = Pin('Y6', Pin.OUT_PP)
sv_g = Pin('Y7', Pin.OUT_PP)
sv_r.high()
sv_b.high()
sv_g.high()

kn = Pin('X5', Pin.IN, Pin.PULL_UP)

servo1 = pyb.Servo(1)
servo1.angle(serv)

p = Pin('X9')
prv = Pin('X10', Pin.OUT_PP)
lev = Pin('X11', Pin.OUT_PP)
prv.low()
lev.high()



tim = Timer(4, freq=1000)
motor = tim.channel(1, Timer.PWM, pin=p)
motor.pulse_width_percent(speed)



def sv(r1, g1, b1):# настройка сервомотора
    if r1 == 0:
        sv_r.high()
    else:
        sv_r.low()
    if g1 == 0:
        sv_g.high()
    else:
        sv_g.low()
    if b1 == 0:
        sv_b.high()
    else:
        sv_b.low()




# sv(0, 0, 0)
# delay(1000)
# sv(1, 0, 0)
# delay(1000)
# sv(0, 1, 0)
# delay(1000)
# sv(0, 0, 1)
# delay(1000)
# sv(1, 1, 0)
# delay(1000)

while True:# цикл

    # print(kn.value())
    # сообщения RaspBerry
    if uart.any():
        a = chr(uart.readchar())
        if a != "$":
            inn += a
            if len(inn) > 10:
                inn = ""
        else:

            try:
                if len(inn) == 9:
                    speed = int(inn[0:3])-200
                    serv = int(inn[3:6])-200
                    r = int(inn[6:7])
                    g = int(inn[7:8])
                    b = int(inn[8:9])
                    print(speed,serv,r,g,b)
            except ValueError:
                print("err")
            inn = ""
            uart.write(str(kn.value())+"$")





# import pyb
# uart = UART(6, 115200, stop=1)
# message = "123$"
# inn = ''
# serv_deg = ADC("X4")
#
# pic = Pin('Y10', Pin.OUT_PP)
# # p_out.high()
# # p_out.low()
#
# button = Pin('Y9', Pin.IN, Pin.PULL_UP)
# # p_in.value()
#
#
# M1 = Pin('X5', Pin.OUT_PP)
# M2 = Pin('X6', Pin.OUT_PP)
# MS = Pin('X10')
# tim = Timer(4, freq = 10000)
# ch = tim.channel(2, Timer.PWM, pin = MS)
#
#
# S1 = Pin('X7', Pin.OUT_PP)
# S2 = Pin('X8', Pin.OUT_PP)
# SS = Pin('X9')
# tim = Timer(4, freq = 10000)
# chS = tim.channel(1, Timer.PWM, pin = SS)
#
# def Motor(sp):
#     global ch, M1, M2
#     if sp < 0:
#         M1.low()
#         M2.high()
#         sp = -sp
#     else:
#         M2.low()
#         M1.high()
#     ch.pulse_width_percent(sp)
#
# eold = 0
# def Serv(deg):
#     global eold, S1, S2, chS
#     if deg<700:
#         deg = 700
#     if deg > 3000:
#         deg = 3000
#     e = deg - serv_deg.read()
#     u = e * 0.1 + (e - eold)*0.3
#
#     if u < 0:
#         S1.low()
#         S2.high()
#         u = -u
#     else:
#         S2.low()
#         S1.high()
#
#
#     if u > 100:
#         u = 100
#
#     if u < 15:
#         u = 30
#
#     if -50 < e < 50:
#         u = 0
#         S1.low()
#         S2.low()
#
#     chS.pulse_width_percent(u)
#     eold = e
#
#
# flag_start = True
# speed = 0
# rul = 0
#
#
#
# while True:
#     if uart.any():
#         a = chr(uart.readchar())
#         if a != '$':
#             inn += a
#             if len(inn) > 9:
#                 inn = ""
#         else:
#             if flag_start:
#                 if inn == "9999999":
#                     flag_start = False
#                     pic.high()
#                     delay(500)
#                     pic.low()
#             else:
#                 message = 'B=' + str(button.value()) + '$'
#                 try:
#                     if len(inn) == 7 and inn != '9999999':
#                         speed = int(inn[:3])-200
#                         rul = int(inn[3:])-1000
#                         # print(speed, rul)
#                         Motor(speed)
#                         Serv(rul)
#                 except ValueError:
#                     print("err")
#
#             # print(inn)
#             inn = ""
#             uart.write(message)




