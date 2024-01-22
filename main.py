from djitellopy import Tello
import cv2

from func import *

#########################################
width = 680
height = 420
#########################################



# Конфиг к телло

me = Tello()
me.connect()
me.forward_back_velocity = 0
me.left_right_velocity = 0
me.up_down_velocity = 0
me.yaw_velocity = 0
me.speed = 0

print(f"Заряд батареи: {me.get_battery()}")

me.streamoff()
me.streamon()

frameWidth = width
frameHeight = height
deadZone = 8
stab_speed = 20

def nothing(x):
    pass


def color_setup():
    cv2.namedWindow("settings")  # создаем окно настроек
    cv2.namedWindow("res")  # создаем окно настроек

    cv2.createTrackbar('h1', 'settings', 0, 255, nothing)
    cv2.createTrackbar('s1', 'settings', 0, 255, nothing)
    cv2.createTrackbar('v1', 'settings', 0, 255, nothing)
    cv2.createTrackbar('h2', 'settings', 255, 255, nothing)
    cv2.createTrackbar('s2', 'settings', 255, 255, nothing)
    cv2.createTrackbar('v2', 'settings', 255, 255, nothing)

    while True:
        DroneImg = me.get_frame_read().frame

        img = cv2.resize(DroneImg, (width, height))

        kernel = np.zeros(img.shape[:2], np.uint8)

        img = cv2.bilateralFilter(img, 9, 75, 75)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # считываем значения бегунков
        h1 = cv2.getTrackbarPos('h1', 'settings')
        s1 = cv2.getTrackbarPos('s1', 'settings')
        v1 = cv2.getTrackbarPos('v1', 'settings')
        h2 = cv2.getTrackbarPos('h2', 'settings')
        s2 = cv2.getTrackbarPos('s2', 'settings')
        v2 = cv2.getTrackbarPos('v2', 'settings')

        # формируем начальный и конечный цвет фильтра
        h_min = np.array((h1, s1, v1), np.uint8)
        h_max = np.array((h2, s2, v2), np.uint8)

        # накладываем фильтр на кадр в модели HSV
        thresh = cv2.inRange(hsv, h_min, h_max)

        res = cv2.bitwise_and(img, img, mask=thresh)

        contors, h = cv2.findContours(res, )
        cv2.imshow('res', res)

        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            print(h_min, h_max)
            break

    cv2.destroyAllWindows()


def display(img):
    cv2.line(img,(int(frameWidth/2) - deadZone, 0), (int(frameWidth/2) - deadZone, frameHeight),(255,255,0),3)
    cv2. line(img, (int(frameWidth/2) + deadZone, 0), (int(frameWidth/2) + deadZone, frameHeight),(255,255,0),3)
    cv2. circle(img, (int(frameWidth/2), int (frameHeight/2)),5,(0,0,255),5)
    cv2. line (img, (0,int(frameHeight / 2) - deadZone), (frameWidth, int (frameHeight / 2) - deadZone), (255, 255, 0), 3)
    cv2. line(img, (0, int(frameHeight / 2) + deadZone), (frameWidth, int (frameHeight / 2) + deadZone), (255, 255, 0), 3)


def rotation():
    while True:
        DroneImg = me.get_frame_read().frame

        img = cv2.resize(DroneImg, (width, height))

        ##################################################################
        # Поиск по цвету
        ##################################################################
        blf = cv2.bilateralFilter(img, 9, 75, 75)
        hsv = cv2.cvtColor(blf, cv2.COLOR_BGR2HSV)

        ########################################################
        # Установить настройки цветового поиска тут:
        hsv_min = (0, 71, 175)
        hsv_max = (14, 255, 255)
        ########################################################

        cl_s = cv2.inRange(hsv, hsv_min, hsv_max)

        kernel = np.ones((3, 3), np.uint8)

        cl_s = cv2.dilate(cl_s, kernel, iterations=7)
        edge = cv2.Canny(cl_s, 120, 180)

        contours, h = cv2.findContours(edge, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        ##################################################################
        # Обработка и отрисовка
        ##################################################################
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        img = display(img)

        if len(contours)==0:
            continue

        contour = contours[0]

        if cv2.contourArea(contour)<4000:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        center = (int(x + w // 2), int(y + h // 2))
        img = cv2.circle(img, center, center[0] - x, (0, 255, 0), 5)

        if center[0]<width/2-deadZone:
            dirction = "left"
        elif center[0]>width/2+deadZone:
            dirction = "right"
        elif center[1]>height/2-deadZone:
            dirction = "up"
        elif center[1]>height/2+deadZone:
            dirction = "down"
        else:
            dirction =""


        if dirction=="left":
            me.left_right_velocity = stab_speed
        elif dirction=="right":
            me.left_right_velocity = -stab_speed

        elif dirction=="up":
            me.up_down_velocity = stab_speed

        elif dirction=="down":
            me.up_down_velocity = -stab_speed
        else:
            print("Стабилизация закончена")
            me.left_right_velocity = 0
            me.up_down_velocity = 0
            cv2.destroyAllWindows()
            me.streamoff()

        if me.send_rc_control:
            me.send_rc_control(me.left_right_velocity, me.forward_back_velocity, me.up_down_velocity, me.yaw_velocity)

        cv2.imshow("drone_view", img)

        # Wait for the "q" ket to stop
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break



###############################################################
# Здесь будет программа полета
###############################################################