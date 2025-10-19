from djitellopy import Tello
import cv2, math, time
from visionMano import HandTrackingDynamic
from control import controlDrone
import datetime

now = datetime.datetime.now()
file_name = "Salidas/" + now.strftime("%Y-%m-%d_%H-%M-%S") + ".txt"

tello = Tello()
tello.connect()
speed = 20
max_speed = 10
tello.set_speed(max_speed)

tello.streamoff()
tello.streamon()
frame_read = tello.get_frame_read()

# Initialize velocities and speed
for_back_velocity = 0
left_right_velocity = 0
up_down_velocity = 0
yaw_velocity = 0

detector = HandTrackingDynamic()

obj_x = 150
obj_z = 150
obj_y = 320
obj_yaw = 0.0

tb_nm11 = 'K_px'
tb_nm12 = 'K_dx'

tb_nm21 = 'K_py'
tb_nm22 = 'K_dy'

tb_nm31 = 'K_pz'
tb_nm32 = 'K_dz'

max_slider = 1.0
min_slider = 0.0

#cv2.createTrackbar(tb_nm11, "Tello" , min_slider, max_slider, on_trackbar)
#cv2.createTrackbar(tb_nm12, "Tello" , min_slider, max_slider, on_trackbar)
#cv2.createTrackbar(tb_nm21, "Tello" , min_slider, max_slider, on_trackbar)
#cv2.createTrackbar(tb_nm22, "Tello" , min_slider, max_slider, on_trackbar)
#cv2.createTrackbar(tb_nm31, "Tello" , min_slider, max_slider, on_trackbar)
#cv2.createTrackbar(tb_nm32, "Tello" , min_slider, max_slider, on_trackbar)


vx = controlDrone(obj_x, [0.5, 0.5, 0], 7)
vy = controlDrone(obj_y, [0.1, 0.5, 0], 7)
vz = controlDrone(obj_z, [0.4, 0.5, 0], 5)
vyaw = controlDrone(obj_yaw, [30, 30, 0], 0.2)

prueba_flag = False
control_flag = False
manual_flag = False
key_press = False
while True:
    
    img = frame_read.frame
    img_resize = cv2.resize(img, (640, 480))
    img_rgb = cv2.cvtColor(img_resize, cv2.COLOR_BGR2RGB)

    bandera_vuelo = tello.is_flying
    bateria = tello.get_battery()
    altura = tello.get_height()

    #Inicializa el procesamiento de imagen 
    if prueba_flag:
        img_procesada = detector.findFingers(img_rgb)
        lmsList = detector.findPosition(img_procesada)
        if(lmsList[1] != []):
            xmin, ymin, xmax, ymax = lmsList[1][0], lmsList[1][1], lmsList[1][2], lmsList[1][3]
            centroide = detector.calcularCentro(ymin, ymax, xmin, xmax)
            h, b = detector.calcularTam(ymin, ymax, xmin, xmax)
            xcc = centroide[0]
            ycc = centroide[1]
            if lmsList[2] is not None:
                normal_vector = lmsList[2]
                yaw = math.atan2(normal_vector[0], normal_vector[2])
                yaw_d = math.degrees(yaw)
            else:
                yaw = 0
                yaw_d = 0

            mano = True
        else:
            mano = False
            
            #Inicializa el lazo de control
        if control_flag:
            if mano:
                vx_sat, ex_ant = vx.estimacion_vel_zm(h, ex_ant)
                vy_sat, ey_ant = vy.estimacion_vel_zm(xcc, ey_ant)
                vy_sat = -1 * vy_sat
                vz_sat, ez_ant = vz.estimacion_vel_zm(ycc, ez_ant)
                vyaw_sat, eyaw_ant = vyaw.estimacion_vel_zm(yaw, eyaw_ant)
                vyaw_sat = -1 * vyaw_sat
            else:
                vx_sat = 0
                vy_sat = 0
                vz_sat = 0
                vyaw_sat = 0

            tello.send_rc_control(vy_sat, vx_sat, vz_sat, vyaw_sat)
            #tello.send_rc_control(0, 0, vz_sat, 0)
            
            with open(file_name, "a") as f:
                valores_x = str(obj_x) + " "+str(h)+ " " +str(vx_sat)+ " " + str(ex_ant) 
                valores_y = str(obj_y) + " "+str(xcc)+ " " +str(vy_sat)+ " " + str(ey_ant) 
                valores_z = str(obj_z) + " "+str(ycc)+ " " + str(vz_sat) + " " + str(ez_ant) 
                valores_yaw = str(obj_yaw) + " "+str(yaw)+ " " + str(vyaw_sat) + " " + str(eyaw_ant) 
                valores = valores_x + " "+ valores_y + " "+ valores_z + " "+valores_yaw 
                f.write(valores+"\n")
            
            vel = "Vx : {} Vy : {} Vz {} Vyaw {}".format(vx_sat, vy_sat, vz_sat, vyaw_sat)
            cv2.putText(img_procesada, vel, (10, 60), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,0,255), 2)
            
            cv2.circle(img_procesada,(320,240), 1, (255,0,0))

            #v2.imshow("Tello", img_procesada)
        else:
            #Se reinician los errores anteriores
            ex_ant = vx.e_ant
            ey_ant = vy.e_ant
            ez_ant = vz.e_ant
            eyaw_ant = vyaw.e_ant
        mano_detectada = "Mano detectada: {}".format(mano)
        cv2.putText(img_procesada, mano_detectada, (10, 20), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,0,0), 2)
        if mano:
            datos = "h: {}px xc: {}px yc: {}px  Orientacion {:.2f} grados {:3f} rad".format(h, xcc, ycc, yaw_d, yaw)
            cv2.putText(img_procesada, datos, (10, 40), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,0,0), 2)
        info = "Volando: {} Bateria: {}% Altura: {} cm".format(bandera_vuelo ,bateria, altura)
        controles = "M: {} PI: {} CA: {}".format(manual_flag, prueba_flag, control_flag)
        cv2.putText(img_procesada, info, (10, 440), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,0,0), 2)
        cv2.putText(img_procesada, controles, (10, 460), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,0,0), 2)
        cv2.imshow("Tello", img_procesada)
    else:
        info = "Volando: {} Bateria: {}% Altura: {} cm".format(bandera_vuelo ,bateria, altura)
        controles = "M: {} PI: {} CA: {}".format(manual_flag, prueba_flag, control_flag)
        cv2.putText(img_rgb, info, (10, 440), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,0,0), 2)
        cv2.putText(img_rgb, controles, (10, 460), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,0,0), 2)
        cv2.imshow("Tello", img_rgb)


    '''
    Q | W | E | R        | O | P |
    -------------- ... H |
    A | S | D | F        |   
    --------------
    
    W = forward
    S = backward
    A = left
    D = right
    Q = yaw left
    E = yaw right
    R = up
    F = down

    H = Hover
    i = manual control
    o = autonomous control
    p = vision processing
    '''
    key = cv2.waitKey(1) & 0xff

    if key == 27:  # ESC
        break
    elif key == ord('t'):
        tello.takeoff()
    elif key == ord('l'):
        tello.land()
    elif key == ord('h'):
        # Hover: set all velocities to 0
        for_back_velocity = 0
        left_right_velocity = 0
        up_down_velocity = 0
        yaw_velocity = 0
    elif key == ord('w'):
        for_back_velocity = speed
    elif key == ord('s'):
        for_back_velocity = -speed
    elif key == ord('a'):
        left_right_velocity = -speed
    elif key == ord('d'):
        left_right_velocity = speed
    elif key == ord('e'):
        yaw_velocity = speed
    elif key == ord('q'):
        yaw_velocity = -speed
    elif key == ord('r'):
        up_down_velocity = speed
    elif key == ord('f'):
        up_down_velocity = -speed
    elif key == ord('i'):
        manual_flag = not manual_flag
    elif key == ord('p'):
        prueba_flag = not prueba_flag
    elif key == ord('o'):
        control_flag = not control_flag

    #print("M: {} PI: {} CA: {} Volando {} Bateria: {:.2f} Altura {}".format(manual_flag, prueba_flag, control_flag, bandera_vuelo, bateria, altura))

    if tello.is_flying and control_flag == False and manual_flag == True:   # Send the velocities to the drone
        tello.send_rc_control(left_right_velocity, for_back_velocity, up_down_velocity, yaw_velocity)

tello.end()
