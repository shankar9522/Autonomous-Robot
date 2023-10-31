import RPi.GPIO as gpio
import time
import numpy as np
import serial
import cv2
import imutils

trig = 16
echo = 18
ser = serial.Serial('/dev/ttyUSB0',9600)
count = 0

def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31,gpio.OUT) #IN1
    gpio.setup(33,gpio.OUT) #IN2
    gpio.setup(35,gpio.OUT) #IN3
    gpio.setup(37,gpio.OUT) #IN4
    gpio.setup(36,gpio.OUT)
    gpio.setup(12,gpio.IN,pull_up_down = gpio.PUD_UP)   #right back wheel encoder
    gpio.setup(7,gpio.IN,pull_up_down = gpio.PUD_UP)    #left front wheel encoder
    gpio.setup(trig, gpio.OUT)
    gpio.setup(echo, gpio.IN)

def limit_straight_pwm(pwm):
    if pwm >= 70:
        pwm = 70
    if pwm <= 50:
        pwm = 50
    return pwm

def pivotleft(pwm):
    if pwm >= 50:
        pwm = 50
    if pwm <= 40:
        pwm = 40
    pwm1.start(pwm)
    pwm2.start(pwm)
    
def pivotright(pwm):
    if pwm >= 50:
        pwm = 50
    if pwm <= 40:
        pwm = 40
    pwm3.start(pwm)
    pwm4.start(pwm)

def forward(pwm_left,pwm_right):
    pwm_left = limit_straight_pwm(pwm_left)
    pwm_right = limit_straight_pwm(pwm_right)
    pwm3.start(pwm_left)
    pwm2.start(pwm_right)

def reverse(pwm_left,pwm_right):
    pwm_left = limit_straight_pwm(pwm_left)
    pwm_right = limit_straight_pwm(pwm_right)
    pwm1.start(pwm_left)
    pwm4.start(pwm_right)

def stop():
    pwm1.stop()
    pwm2.stop()
    pwm3.stop()
    pwm4.stop()

def gripper_open():
    pwm5.ChangeDutyCycle(8)
    time.sleep(1)

def gripper_close():
    pwm5.ChangeDutyCycle(2)
    time.sleep(1)

def imu_readings(imu_val,count):
    if(ser.in_waiting > 0):
        count +=1
        line = ser.readline()
        if(count>10):
            line = line.rstrip().lstrip()
            line = str(line)
            line = line.strip("'")
            line = line.strip("b'")
            imu_val = float(line[2:7])
    return imu_val, count

def detect_block(block_height,target_found,low_hsv,upper_hsv,block_height_readings,roi_coords,center_block_x_pixel):
    _, frame = vid.read()
    frame = frame[120:, :]
    frame = cv2.flip(frame,-1)
    frame = frame[130:, :]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low_hsv, upper_hsv)   

    if target_found:
        cv2.polylines(frame, [np.array(roi_coords)], True, (0, 255, 0), thickness=2)
        mask_roi = np.zeros_like(mask)
        cv2.fillPoly(mask_roi, [np.array(roi_coords)], 255)
        mask = cv2.bitwise_and(mask, mask_roi)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        target_found = False
    else:
        max_area_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(max_area_contour)
        if area < 5:
            target_found = False
        else:
            target_found = True
            x, y, w, h = cv2.boundingRect(max_area_contour)
            block_height_readings.append(h)
            block_height = sum(block_height_readings)/len(block_height_readings) # calculate block height
            block_height_readings.pop(0)                  # calculate block height

            mask_x_min = int(x-(w/2) - 20)
            mask_x_max = int(x+w+(w/2) + 20)
            mask_y_min = int(y-(h/2) - 20)
            mask_y_max = int(y+h+(h/2) +20)

            roi_coords = [(mask_x_min, mask_y_min), (mask_x_max, mask_y_min), (mask_x_max, mask_y_max), (mask_x_min, mask_y_max)]
            center_block_x_pixel = (x+x+w)/2
    return block_height,center_block_x_pixel,target_found,roi_coords

def turn_direction(IV,TA):      # IV imu value, TA targer angle
    if (180<= TA <=360) and (180 <=IV<= 360):
        diff_angle = abs(TA-IV)
        if TA > IV:
            return 2, diff_angle
        else:
            return 1, diff_angle
    elif (0<= TA <= 180) and (0<= IV <= 180):
        diff_angle = abs(TA-IV)
        if TA > IV:
            return 2, diff_angle
        else:
            return 1, diff_angle
    elif (180<= TA <=360) and (0<= IV <= 180):
        diff_angle = abs(IV + (360-TA))
        return 1, diff_angle
    elif (180 <=IV<= 360) and (0<= TA <= 180):
        diff_angle = abs(TA + (360-IV))
        return 2, diff_angle
    else:
        return 0, 0

def turn(target_angle,gain_turn_angle,imu_val,turn_control_orientation):
    prev_turn_control_orientation = turn_control_orientation
    turn_control_orientation, diff_turn_angle = turn_direction(imu_val,target_angle)
    if not turn_control_orientation ==  prev_turn_control_orientation:
        stop()
    pwm_turn = gain_turn_angle * diff_turn_angle
    if turn_control_orientation == 1:
        pivotleft(pwm_turn)
    elif turn_control_orientation == 2:
        pivotright(pwm_turn)
    return diff_turn_angle,turn_control_orientation

def straight(pwm_left,pwm_right,imu_val,counter1,counter2,button1,button2,heading_angle,fwd_bwk_dir):
    if int (gpio.input(12)) != int(button1):
        button1 = int(gpio.input(12))
        counter1+= 1
    if int (gpio.input(7)) != int(button2):
        button2 = int(gpio.input(7))
        counter2+=1
    straight_line_control_orientation, diff_straight_angle = straight_line_heading(imu_val,heading_angle)    # 1:left, 2:right, 0:aligned
    if diff_straight_angle>1:
        if straight_line_control_orientation == 1:
            pwm_left -=2
            pwm_right +=2
            if fwd_bwk_dir == 1:
                forward(pwm_left,pwm_right)
            if fwd_bwk_dir == -1:
                reverse(pwm_left,pwm_right)

        elif straight_line_control_orientation == 2:
            pwm_left +=2
            pwm_right -=2
            if fwd_bwk_dir == 1:
                forward(pwm_left,pwm_right)
            if fwd_bwk_dir == -1:
                reverse(pwm_left,pwm_right)
    else:
        pwm_left = 60
        pwm_right = 60
        if fwd_bwk_dir == 1:
            forward(pwm_left,pwm_right)
        if fwd_bwk_dir == -1:
            reverse(pwm_left,pwm_right)
    wheel_counter = (counter1+counter2)/2
    x = np.cos(np.radians(imu_val))*(wheel_counter*(1/30))
    y = np.sin(np.radians(360-imu_val))*(wheel_counter*(1/30))
    current_distance = np.sqrt((x)**2 + (y)**2)
    return current_distance,counter1,counter2,button1,button2,x,y,pwm_left,pwm_right

def straight_line_heading(IV,HA):
    if (0<= int(IV) <=180) and (0<= int(HA) <=180):
        diff_angle = abs(HA-IV)
        if int(HA)>int(IV):
            return 2, diff_angle
        else:
            return 1, diff_angle
    elif (180<= int(IV) <=360) and (180<= int(HA) <=360):
        diff_angle = abs(HA-IV)
        if int(HA)>int(IV):
            return 2, diff_angle
        else:
            return 1, diff_angle
    elif (0<= int(HA) <= 180) and (180<= int(IV) <=360):
        diff_angle = abs(HA+(360-IV))
        return 2, diff_angle
    elif (180<= int(HA) <=360) and (0<= int(IV) <= 180):
        diff_angle = abs(IV+(360-HA))
        return 1, diff_angle  
    else:
        return 0, 0 
    
def go_to_loc(x_curr,y_curr,x_goal,y_goal):
    distance = np.sqrt((x_curr-x_goal)**2 + (y_curr-y_goal)**2)
    orientation = np.degrees(np.arctan2(y_goal-y_curr,x_goal-x_curr))
    if orientation<0:
        orientation = 360 + orientation
    orientation = 360 - orientation
    return distance,orientation

def ultrasonic_distance():
    gpio.output(trig, False)
    time.sleep(0.01)
    gpio.output(trig, True)
    time.sleep(0.01)
    gpio.output(trig, False)
    while gpio.input(echo) == 0:
        pulse_start = time.time()
    while gpio.input(echo) == 1:
        pulse_end = time.time()      
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration*17150
    distance = round(distance, 2)
    return distance
############## initializations #######################
init()
pin1 = 33
pin2 = 37
pwm1 = gpio.PWM(pin1,50)
pwm2 = gpio.PWM(pin2,50)
pin3 = 31
pin4 = 35
pin5 = 36
pwm3 = gpio.PWM(pin3,50)
pwm4 = gpio.PWM(pin4,50)
pwm5 = gpio.PWM(pin5,50)
counter1 = np.uint64(0)
counter2 = np.uint64(0) 
button1 = int(0)
button2 = int(0)
pwm5.start(0)

target_found = False        # initialization   
roi_coords = []
block_height = 0        
block_height_readings = [0,0,0,0,0]
center_block_x_pixel = 0
gain_turn_angle = 4
imu_val = 0
turn_control_orientation = 0
x_global = 1
y_global = 1

pwm_left = 60       # variable
pwm_right = 60
block_height_threshold = 115     
diff_turn_angle_threshold = 3
distance_retrieve = 1
place_x_location = 1.5
place_y_location = 8.5

color_sequence = 1
sequence = 5

vid = cv2.VideoCapture(0)
vid.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
time.sleep(5)
while True:
    imu_val, count = imu_readings(imu_val,count)
    ###################################################################################################################

    if color_sequence ==1 or color_sequence ==4 or color_sequence ==7:       
        low_hsv = (144, 114, 85)           # red
        upper_hsv = (255, 255, 255)
    
    if color_sequence ==2 or color_sequence ==5 or color_sequence ==8:
        low_hsv = (33, 43, 12)            # green 
        upper_hsv = (83, 255, 255)
    
    if color_sequence ==3 or color_sequence ==6 or color_sequence ==9:
        low_hsv = (89, 44, 23)            # blue
        upper_hsv = (117, 255, 255)

######################## detect block ##########################
    block_height,center_block_x_pixel,target_found,roi_coords = detect_block(block_height,target_found,low_hsv,upper_hsv,block_height_readings,roi_coords,center_block_x_pixel)

    if sequence == 5:
        pivotleft(45)
        if target_found:
            stop()
            sequence= 6
            start_time = time.time()

    if sequence == 6:
        end_time = time.time()
        delta_time = end_time - start_time
        if delta_time >0.1:
            sequence = 10
            delta_time = 0
    
    if sequence == 10:
        theta = (center_block_x_pixel-320) * 0.061
        theta_abs = abs(theta)
        if theta_abs <4:
            theta_abs +=4
        if theta>0:
            theta_dir = 1
        else:
            theta_dir = -1
        if 0<= imu_val <=180 and theta_dir == 1:
            target_angle = imu_val + theta_abs

        elif 180<= imu_val <=360 and theta_dir == -1:
            target_angle = imu_val - theta_abs

        elif 180<= imu_val<=360 and theta_dir == 1:
            target_angle = imu_val + theta_abs
            if target_angle<=360:
                target_angle = target_angle
            else:
                target_angle = target_angle - 360
        elif 0 <= imu_val <= 180 and theta_dir == -1:
            target_angle = imu_val - theta_abs
            if target_angle>0:
                target_angle=target_angle
            else: 
                target_angle = 360 + target_angle
        else:
            None
        sequence = 20
            
    if sequence == 20:
        diff_turn_angle,turn_control_orientation = turn(target_angle,gain_turn_angle,imu_val,turn_control_orientation)
        if diff_turn_angle<diff_turn_angle_threshold:
            stop()
            sequence = 21
            start_time = time.time()

    if sequence == 21:
        end_time = time.time()
        delta_time = end_time - start_time
        if delta_time >0.1:
            sequence = 30
            target_angle = imu_val
            delta_time = 0

    if sequence == 30:
        fwd_bwk_dir = 1
        distance,counter1,counter2,button1,button2,x,y,pwm_left,pwm_right = straight(pwm_left,pwm_right,imu_val,counter1,counter2,button1,button2,target_angle,fwd_bwk_dir)
        if not (270 <= center_block_x_pixel <= 370):
            stop()
            counter1 = 0
            counter2 = 0
            distance = 0
            x_global +=x            
            y_global +=y
            sequence = 5

        if block_height>block_height_threshold:
            stop()
            counter1 = 0
            counter2 = 0
            distance = 0
            x_global +=x            
            y_global +=y
            sequence = 40

    if sequence == 40:
        gripper_open()
        sequence = 50

    if sequence == 50:
        fwd_bwk_dir = 1
        distance,counter1,counter2,button1,button2,x,y,pwm_left,pwm_right = straight(pwm_left,pwm_right,imu_val,counter1,counter2,button1,button2,target_angle,fwd_bwk_dir)
        if distance>0.8:
            stop()
            counter1 = 0
            counter2 = 0
            distance = 0
            x_global +=x            
            y_global +=y
            sequence = 60

    if sequence == 60:
        gripper_close()
        sequence = 70
        
    if sequence == 70:
        distance_to_go, orientation = go_to_loc(x_global,y_global,place_x_location,place_y_location)
        diff_turn_angle,turn_control_orientation = turn(orientation,gain_turn_angle,imu_val,turn_control_orientation)
        if diff_turn_angle<diff_turn_angle_threshold:
            stop()
            sequence = 80

    if sequence == 80:
        fwd_bwk_dir = 1
        distance,counter1,counter2,button1,button2,x,y,pwm_left,pwm_right = straight(pwm_left,pwm_right,imu_val,counter1,counter2,button1,button2,orientation,fwd_bwk_dir)
        if distance > distance_to_go:
            stop()
            print()
            counter1 = 0
            counter2 = 0
            distance = 0
            x_global +=x            
            y_global +=y
            sequence = 90

    if sequence == 90:
        gripper_open()
        sequence = 100

    if sequence == 100:
        fwd_bwk_dir = -1
        distance,counter1,counter2,button1,button2,x,y,pwm_left,pwm_right = straight(pwm_left,pwm_right,imu_val,counter1,counter2,button1,button2,orientation,fwd_bwk_dir)
        if distance > distance_retrieve:
            stop()
            counter1 = 0
            counter2 = 0
            distance = 0
            x_global -=x            
            y_global -=y
            sequence = 110

    if sequence == 110:
        gripper_close()
        sequence = 120

    if sequence == 120:
        diff_turn_angle,turn_control_orientation = turn(270,gain_turn_angle,imu_val,turn_control_orientation)
        if diff_turn_angle<diff_turn_angle_threshold:
            stop()
            sequence = 121

    if sequence == 121:
        ultrasonic_dist = 0
        ultrasonic_dist += ultrasonic_distance()
        ultrasonic_dist += ultrasonic_distance()
        ultrasonic_dist += ultrasonic_distance()
        ultrasonic_dist += ultrasonic_distance()
        ultrasonic_dist += ultrasonic_distance()
        ultrasonic_dist = ultrasonic_dist/5

        ultrasonic_dist = ultrasonic_dist*0.0328084
        y_global = 10 - ultrasonic_dist
        time.sleep(0.5)
        sequence = 122

    if sequence == 122:
        diff_turn_angle,turn_control_orientation = turn(180,gain_turn_angle,imu_val,turn_control_orientation)
        if diff_turn_angle<diff_turn_angle_threshold:
            stop()
            sequence = 123

    if sequence == 123:
        ultrasonic_dist = 0
        ultrasonic_dist += ultrasonic_distance()
        ultrasonic_dist += ultrasonic_distance()
        ultrasonic_dist += ultrasonic_distance()
        ultrasonic_dist += ultrasonic_distance()
        ultrasonic_dist += ultrasonic_distance()
        ultrasonic_dist = ultrasonic_dist/5
    
        ultrasonic_dist = ultrasonic_dist*0.0328084
        x_global = ultrasonic_dist
        time.sleep(0.5)
        sequence = 124

    if sequence == 124:
        diff_turn_angle,turn_control_orientation = turn(80,gain_turn_angle,imu_val,turn_control_orientation)
        if diff_turn_angle<diff_turn_angle_threshold:
            stop()
            color_sequence +=1
            sequence = 5

            if color_sequence >9:       # change according to number of blocks to pick
                break
   



