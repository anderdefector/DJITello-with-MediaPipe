from djitellopy import Tello
import cv2, math
from visionMano import HandTrackingDynamic
from control import controlDrone
import datetime
import pygame
import numpy as np

now = datetime.datetime.now()
file_name = "Salidas/" + now.strftime("%Y-%m-%d_%H-%M-%S") + ".txt"

# --- Pygame Initialization ---
pygame.init()
screen = pygame.display.set_mode((640, 480))
pygame.display.set_caption("Tello Control")
font = pygame.font.SysFont("Arial", 20)

tello = Tello()
tello.connect()
speed = 20
max_speed = 10
tello.set_speed(max_speed)

tello.streamoff()
tello.streamon()
frame_read = tello.get_frame_read()

def draw_text(text, x, y, color=(0, 0, 255)):
    """Helper function to draw text on the pygame screen."""
    surface = font.render(text, True, color)
    screen.blit(surface, (x, y))


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


vx = controlDrone(obj_x, [0.5, 0.5, 0], 7)
vy = controlDrone(obj_y, [0.1, 0.5, 0], 7)
vz = controlDrone(obj_z, [0.4, 0.5, 0], 5)
vyaw = controlDrone(obj_yaw, [30, 30, 0], 0.2)

prueba_flag = False
control_flag = False
manual_flag = False
key_press = False
running = True
while running:
    
    img = frame_read.frame
    img_resize = cv2.resize(img, (640, 480))
    #img_rgb = cv2.cvtColor(img_resize, cv2.COLOR_BGR2RGB)

    bandera_vuelo = tello.is_flying
    bateria = tello.get_battery()
    altura = tello.get_height()

    #Inicializa el procesamiento de imagen 
    if prueba_flag:
    
        img_rgb = detector.findFingers(img_resize)
        #display_img = img_procesada # Use processed image for display
        lmsList = detector.findPosition(img_rgb)
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
            cv2.circle(img_rgb,(320,240), 5, (0,255,0))
        else:
            #Se reinician los errores anteriores
            ex_ant = vx.e_ant
            ey_ant = vy.e_ant
            ez_ant = vz.e_ant
            eyaw_ant = vyaw.e_ant
    else:
        img_rgb = img_resize

    # --- Pygame Display and Event Handling ---
    # Convert image for pygame
    img_rgb = np.rot90(img_rgb)
    frame = pygame.surfarray.make_surface(img_rgb)
    frame = pygame.transform.flip(frame, True, False)
    screen.blit(frame, (0, 0))

    # Draw status text on top of the video feed
    info = "Volando: {} Bateria: {}% Altura: {} cm".format(bandera_vuelo ,bateria, altura)
    controles = "M: {} PI: {} CA: {}".format(manual_flag, prueba_flag, control_flag)
    draw_text(info, 10, 430)
    draw_text(controles, 10, 450)
    if prueba_flag:
        mano_detectada = "Mano detectada: {}".format(mano)
        draw_text(mano_detectada, 10, 20)
        if control_flag:
            vel = "Vx : {} Vy : {} Vz {} Vyaw {}".format(vx_sat, vy_sat, vz_sat, vyaw_sat)
            draw_text(vel, 10, 60, color=(255, 0, 255))
            #cv2.circle(img_rgb,(320,240), 1, (255,0,0))
        if mano:
            datos = "h: {}px xc: {}px yc: {}px  Orientacion {:.1f} grados ".format(h, xcc, ycc, yaw_d)
            draw_text(datos, 10, 40)

    pygame.display.update()

    # Reset velocities to 0 after each manual command
    if manual_flag:
        for_back_velocity = left_right_velocity = up_down_velocity = yaw_velocity = 0

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
            elif event.key == pygame.K_t:
                tello.takeoff()
            elif event.key == pygame.K_l:
                tello.land()
            # Toggle flags
            elif event.key == pygame.K_i:
                manual_flag = not manual_flag
            elif event.key == pygame.K_p:
                prueba_flag = not prueba_flag
            elif event.key == pygame.K_o:
                control_flag = not control_flag

    # Manual control key presses (movement)
    if manual_flag:
        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            for_back_velocity = speed
        if keys[pygame.K_s]:
            for_back_velocity = -speed
        if keys[pygame.K_a]:
            left_right_velocity = -speed
        if keys[pygame.K_d]:
            left_right_velocity = speed
        if keys[pygame.K_e]:
            yaw_velocity = speed
        if keys[pygame.K_q]:
            yaw_velocity = -speed
        if keys[pygame.K_r]:
            up_down_velocity = speed
        if keys[pygame.K_f]:
            up_down_velocity = -speed
        if keys[pygame.K_h]: # Hover
            for_back_velocity = left_right_velocity = up_down_velocity = yaw_velocity = 0

    #print("M: {} PI: {} CA: {} Volando {} Bateria: {:.2f} Altura {}".format(manual_flag, prueba_flag, control_flag, bandera_vuelo, bateria, altura))

    if tello.is_flying and control_flag == False and manual_flag == True:   # Send the velocities to the drone
        tello.send_rc_control(left_right_velocity, for_back_velocity, up_down_velocity, yaw_velocity)

pygame.quit()
tello.end()
