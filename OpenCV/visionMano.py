import cv2
import mediapipe as mp
import time
import math as math
import numpy as np

class HandTrackingDynamic:
    def __init__(self, mode=False, maxHands=1, detectionCon=0.5, trackCon=0.5):
        self.__mode__   =  mode
        self.__maxHands__   =  maxHands
        self.__detectionCon__   =   detectionCon
        self.__trackCon__   =   trackCon
        self.handsMp = mp.solutions.hands
        self.hands = self.handsMp.Hands()
        self.mpDraw= mp.solutions.drawing_utils
        self.tipIds = [4, 8, 12, 16, 20]
    
    def calcularCentro(self, ymin, ymax, xmin, xmax):
        return (int( (xmin + xmax) / 2 ), int( (ymin + ymax) / 2 ))
    
    def calcularTam(self, ymin, ymax, xmin, xmax):
        altura = ymax - ymin
        base = xmax - xmin
        return altura, base

    def calculate_hand_rotation(self, landmarks):
        if not landmarks:
            return None

        # Seleccionar landmarks para definir el plano de la palma
        # Wrist (0), Index finger MCP (5), Pinky finger MCP (17)
        try:
            wrist = np.array([landmarks.landmark[0].x, landmarks.landmark[0].y, landmarks.landmark[0].z])
            index_mcp = np.array([landmarks.landmark[5].x, landmarks.landmark[5].y, landmarks.landmark[5].z])
            pinky_mcp = np.array([landmarks.landmark[17].x, landmarks.landmark[17].y, landmarks.landmark[17].z])

            # Crear dos vectores en el plano de la palma
            vec1 = index_mcp - wrist
            vec2 = pinky_mcp - wrist
            if landmarks.landmark[5].x > landmarks.landmark[17].x:
                vec1, vec2 = vec2, vec1
            # Calcular el producto cruz para obtener el vector normal al plano de la palma
            normal_vector = np.cross(vec1, vec2)

            # Normalizar el vector para que su magnitud sea 1
            if np.linalg.norm(normal_vector) != 0:
                normal_vector /= np.linalg.norm(normal_vector)
            else:
                return None # Evitar división por cero

            return normal_vector
        except IndexError:
            return None

    def findFingers(self, frame, draw=True):
        self.results = self.hands.process(frame) 
        #print(self.results.multi_handedness)
        if self.results.multi_hand_landmarks: 
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(frame, handLms,self.handsMp.HAND_CONNECTIONS)

        return frame
  
    def findPosition( self, frame, handNo=0, draw=True):
        xList =[]
        yList =[]
        bbox = []
        self.lmsList=[]
        normal_vector = None
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = frame.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                xList.append(cx)
                yList.append(cy)
                self.lmsList.append([id, cx, cy])
                
                if draw:
                    cv2.circle(frame,  (cx, cy), 5, (255, 0, 255), cv2.FILLED)
            for hand_landmarks in self.results.multi_hand_landmarks:
            
                normal_vector = self.calculate_hand_rotation(hand_landmarks)
            
            xmin, xmax = min(xList), max(xList)
            ymin, ymax = min(yList), max(yList)
            bbox = xmin, ymin, xmax, ymax
            
            centro = self.calcularCentro(ymin, ymax, xmin, xmax)

            cv2.circle(frame,(centro[0],centro[1]),5,(255,0,0),cv2.FILLED)
            if draw:
                cv2.rectangle(frame, (xmin - 20, ymin - 20),(xmax + 20, ymax + 20),
                               (0, 255 , 0) , 2)

        return self.lmsList, bbox, normal_vector

    def findFingerUp(self):
         fingers=[]

         if self.lmsList[self.tipIds[0]][1] > self.lmsList[self.tipIds[0]-1][1]:
              fingers.append(1)
         else:
              fingers.append(0)

         for id in range(1, 5):            
              if self.lmsList[self.tipIds[id]][2] < self.lmsList[self.tipIds[id]-2][2]:
                   fingers.append(1)
              else:
                   fingers.append(0)
        
         return fingers
