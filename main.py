import cv2
import numpy as np
import serial
import threading
import time

ser = serial.Serial('/dev/ttyACM0',115200,bytesize=8,parity="N",stopbits=1,timeout=None)

speed_l = 0
speed_r = 0
speed_base = 200
speed_top = 235
dir_l = 0
dir_r = 0
Kp = 2
Kd = 0

time.sleep(3)
data = np.zeros(6, dtype=np.uint8)


data[0] = 0x73
data[1] = 1
data[2] = 190
data[3] = 1
data[4] = 190
data[5] = 0x64

def move_motor(dir1,speed1,dir2,speed2):
    global data
    data[1] = dir1
    data[2] = speed1
    data[3] = dir2
    data[4] = speed2



def writing():
    global data
    while True:
        ser.write(data)
        time.sleep(0.1)

cap = cv2.VideoCapture(0)
cap.set(3, 160)
cap.set(4, 120)
in1 = 4
in2 = 17
in3 = 27
in4 = 22

en1 = 23
en2 = 24
qcd = cv2.QRCodeDetector()

def detect():    
    global speed_l
    global speed_r
    global dir_l
    global dir_r
    
    while True:
        _, frame = cap.read()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define red color range
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        mask = cv2.inRange(hsv_frame, lower_red1, upper_red1) + cv2.inRange(hsv_frame, lower_red2, upper_red2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours by area (optional)
        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 100]


        if valid_contours:
            largest_contour = max(valid_contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)  # Mark the center
            # print("CX : "+str(cx)+"  CY : "+str(cy))
            print("cx value = " + str(cx))
            
            if cx > 70 and cx < 90:
                print ("lurus")
                move_motor(2,220,2,220)
                # single_L = 90 
                # single_R = 90
            elif cx > 90 and cx < 125:
                print ("belok kanan")
                move_motor(2,230,2,100)
                # single_L = 120
                # single_R = 40
            elif cx < 70 and cx < 35:
                print ("belok kiri")
                move_motor(2,100,2,230)
                # single_L = 40
                # single_R = 120
            elif cx > 125 and cx < 160:
                print("patah kanan")
                move_motor(2,250,1,230)
                # single_L = 190
                # single_R = 40
            elif cx > 0 and cx < 35:
                print("patah kiri")
                move_motor(1,230,2,250)
            #     single_L = 40
            #     single_R = 150
                

        
        cv2.imshow("Red Line Detection", frame)
        if cv2.waitKey(1) & 0xff == ord('q'):   # 1 is the time in ms
            break
    cap.release()
    cv2.destroyAllWindows()

    


detect_thread = threading.Thread(target=detect)
ser_thread = threading.Thread(target=writing)

detect_thread.start()
ser_thread.start()

detect_thread.join()
ser_thread.join()
