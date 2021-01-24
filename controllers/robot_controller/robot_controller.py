"""robot_controller controller."""
from controller import Robot

import math
import cv2
import numpy as np

# constants
FORWARD_SPEED = 200
FORWARD_K_SPEED = 0.4

TIME_STEP = 32

OAM_OBST_THRESHOLD = 100
OAM_FORWARD_SPEED = 150
OAM_K_PS_90 = 0.2
OAM_K_PS_45 = 0.9
OAM_K_PS_00 = 1.2
OAM_K_MAX_DELTAS = 600

LLM_THRESHOLD = 800

OFM_DELTA_SPEED = 150

LEM_FORWARD_SPEED = 100
LEM_K_GS_SPEED = 0.5
LEM_THRESHOLD = 500
LEM_STATE_STANDBY = 0
LEM_STATE_LOOKING_FOR_LINE = 1
LEM_STATE_LINE_DETECTED = 2
LEM_STATE_ON_LINE = 3

# robot instance
ticks = 0
robot = Robot()

# ground sensors
gs_left = robot.getDevice('gs0')
gs_left.enable(TIME_STEP)

gs_right = robot.getDevice('gs2')
gs_right.enable(TIME_STEP)

gs_center = robot.getDevice('gs1')
gs_center.enable(TIME_STEP)

# proximity sensors
ps0 = robot.getDevice('ps0')
ps0.enable(TIME_STEP)

ps1 = robot.getDevice('ps1')
ps1.enable(TIME_STEP)

ps2 = robot.getDevice('ps2')
ps2.enable(TIME_STEP)

ps3 = robot.getDevice('ps3')
ps3.enable(TIME_STEP)

ps4 = robot.getDevice('ps4')
ps4.enable(TIME_STEP)

ps5 = robot.getDevice('ps5')
ps5.enable(TIME_STEP)

ps6 = robot.getDevice('ps6')
ps6.enable(TIME_STEP)

ps7 = robot.getDevice('ps7')
ps7.enable(TIME_STEP)

ps_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
gs_value = [0.0, 0.0, 0.0]

# start opencv
cv2.startWindowThread()
cv2.namedWindow("preview")

# camera sensor
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

# motor instances
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

motors_speed = [0.0, 0.0]

# line following parameters
lfm_speed = [0.0, 0.0]

# obstacle avoidance parameters
oam_active = False
oam_reset = True
oam_speed = [0.0, 0.0]
oam_side = -1

# line leaving parameters
llm_active = False
lem_reset = False
llm_inibit_ofm_speed = False
llm_past_side = -1

# obstacle following parameters
ofm_active = False
ofm_speed = [0.0, 0.0]

# line entering parameters
lem_active = False
lem_speed = [0.0, 0.0]
lem_state = LEM_STATE_STANDBY
lem_black_counter = 0
cur_op_gs_value = 0
prev_op_gs_value = 0

# semaphore parameters
smp_stop = False
smp_ticks = 0

def line_following():
    global lfm_speed

    # delta between ground sensors (left and right)
    delta = gs_right.getValue() - gs_left.getValue()

    lfm_speed[0] = FORWARD_SPEED - (FORWARD_K_SPEED * delta)
    lfm_speed[1] = FORWARD_SPEED + (FORWARD_K_SPEED * delta)

def avoid_obstacle():
    global oam_reset
    global oam_active
    global oam_side
    global oam_speed

    activation = [0, 0]

    # Reset
    if oam_reset:
        oam_active = False
        oam_side = -1

    oam_reset = False

    # check if there is an obstacle and which is the side of obstacle
    max_ds_value = 0

    # iterate through sensors 0,1
    for i in range(0,2):
        if max_ds_value < ps_value[i]:
            max_ds_value = ps_value[i]
        activation[1] += ps_value[i]

    # iterate through sensors 6,7
    for i in range(6,8):
        if max_ds_value < ps_value[i]:
            max_ds_value = ps_value[i]
        activation[0] += ps_value[i]

    if max_ds_value > OAM_OBST_THRESHOLD:
        oam_active = True

    if oam_active and oam_side == -1:
        if activation[1] > activation[0]:
            oam_side = 1
        else:
            oam_side = 0

    # forward speed
    oam_speed[0] = OAM_FORWARD_SPEED
    oam_speed[1] = OAM_FORWARD_SPEED

    # go away from obstacle
    if oam_active:
        delta = 0

        if oam_side == 0:
            delta -= int(OAM_K_PS_90 * ps_value[5])
            delta -= int(OAM_K_PS_45 * ps_value[6])
            delta -= int(OAM_K_PS_00 * ps_value[7])
        else:
            delta += int(OAM_K_PS_90 * ps_value[2])
            delta += int(OAM_K_PS_45 * ps_value[1])
            delta += int(OAM_K_PS_00 * ps_value[0])

        if delta > OAM_K_MAX_DELTAS:
            delta = OAM_K_MAX_DELTAS

        if delta < -OAM_K_MAX_DELTAS:
            delta = -OAM_K_MAX_DELTAS

        oam_speed[0] -= delta
        oam_speed[1] += delta


def line_leaving(side):
    global llm_active
    global llm_past_side
    global llm_inibit_ofm_speed
    global lem_reset

    # check if line leaving is active
    if llm_active == False and side != -1 and llm_past_side == -1:
        llm_active = True

    # save previous side
    llm_past_side = side

    if llm_active:
        # left side
        if side == 0:
            if (gs_center.getValue() + gs_left.getValue()) / 2 > LLM_THRESHOLD:
                llm_active = False
                llm_inibit_ofm_speed = False
                lem_reset = True
            else:
                llm_inibit_ofm_speed = True
        # right side
        else:
            if (gs_center.getValue() + gs_right.getValue()) / 2 > LLM_THRESHOLD:
                llm_active = False
                llm_inibit_ofm_speed = False
                lem_reset = True
            else:
                llm_inibit_ofm_speed = True

def obstacle_following(side):
    global ofm_active
    global ofm_speed

    if side != -1:
        ofm_active = True
        # left side
        if side == 0:
            ofm_speed[0] = -OFM_DELTA_SPEED
            ofm_speed[1] = OFM_DELTA_SPEED
        else:
            ofm_speed[0] = OFM_DELTA_SPEED
            ofm_speed[1] = -OFM_DELTA_SPEED
    else:
        ofm_active = False
        ofm_speed[0] = 0.0
        ofm_speed[1] = 0.0

def line_entering(side):
    global lem_reset
    global lem_state
    global lem_speed
    global lem_active
    global lem_black_counter
    global oam_reset
    global cur_op_gs_value
    global prev_op_gs_value

    # reset line entering
    if lem_reset:
        lem_state = LEM_STATE_LOOKING_FOR_LINE

    side_direction = -1
    op_side = -1
    gs_side = -1
    gs_opside = -1
    lem_reset = False

    # initialization
    lem_speed[0] = LEM_FORWARD_SPEED
    lem_speed[1] = LEM_FORWARD_SPEED

    # left side
    if side == 0:
        side_direction = 1
        op_side = 0
        gs_side = 1
        gs_opside = 0
    else:
        side_direction = 0
        op_side = 1
        gs_side = 0
        gs_opside = 1

    # stand by
    if lem_state == LEM_STATE_STANDBY:
        lem_active = False
    # looking for line
    elif lem_state == LEM_STATE_LOOKING_FOR_LINE:
        if gs_value[gs_side] < LEM_THRESHOLD:
            lem_active = True
            lem_speed[op_side] = LEM_FORWARD_SPEED
            lem_speed[side_direction] = LEM_FORWARD_SPEED
            lem_state = LEM_STATE_LINE_DETECTED

            if gs_value[gs_opside] < LEM_THRESHOLD:
                cur_op_gs_value = 1
                lem_black_counter = 1
            else:
                cur_op_gs_value = 0
                lem_black_counter = 0

            prev_op_gs_value = cur_op_gs_value
    # line detected
    elif lem_state == LEM_STATE_LINE_DETECTED:
        if gs_value[gs_opside] < LEM_THRESHOLD:
            cur_op_gs_value = 1
            lem_black_counter += 1
        else:
            cur_op_gs_value = 0

        if prev_op_gs_value == 1 and cur_op_gs_value == 0:
            lem_state = LEM_STATE_ON_LINE
            lem_speed[op_side] = 0
            lem_speed[side_direction] = 0
        else:
            prev_op_gs_value = cur_op_gs_value
            lem_speed[op_side] = LEM_FORWARD_SPEED + LEM_K_GS_SPEED * (900 - gs_value[gs_side])
            lem_speed[side_direction] = LEM_FORWARD_SPEED - LEM_K_GS_SPEED * (900 - gs_value[gs_side])
    # on line
    elif lem_state == LEM_STATE_ON_LINE:
        oam_reset = True
        lem_active = False
        lem_state = LEM_STATE_STANDBY

def detect_color_and_draw_rectangle(type, contours, image):
    ret = False

    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        if(area > 300): 
            x, y, w, h = cv2.boundingRect(contour) 
            image = cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2) 
              
            label = "Pare" if type == 0 else "Prossiga"
            color = (0, 0, 255) if type == 0 else (0, 255, 0)

            cv2.putText(image, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color)

            # rectangle is entirely detected by camera?
            if x > 0 and y > 0 and x + w < 364 and y + h < 224:
                ret = True

    return ret

def process_image(image):
    global smp_stop
    global smp_ticks
    global ticks

    # convert image color to HSV
    hsvFrame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
  
    # define red color range mask
    red_lower = np.array([136, 87, 111], np.uint8) 
    red_upper = np.array([180, 255, 255], np.uint8) 
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 
  
     # define green color range mask
    green_lower = np.array([25, 52, 72], np.uint8) 
    green_upper = np.array([102, 255, 255], np.uint8) 
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 
  
    # morphological transform to detect color
    kernal = np.ones((5, 5), "uint8") 
      
    # red color mask
    red_mask = cv2.dilate(red_mask, kernal) 
    res_red = cv2.bitwise_and(image, image, mask = red_mask) 
      
    # green color mask
    green_mask = cv2.dilate(green_mask, kernal) 
    res_green = cv2.bitwise_and(image, image, mask = green_mask) 
      
    # detect red color mask
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    if detect_color_and_draw_rectangle(0, contours, image):
        if smp_stop == False:
            smp_stop = True
            smp_ticks = ticks
            print("Pare!")
    elif smp_stop:
        smp_stop = False
  
    # detect green color mask
    contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    detect_color_and_draw_rectangle(1, contours, image)

    cv2.imshow("green mask", green_mask)
    cv2.imshow("red mask", red_mask)
    cv2.imshow("preview", image)
    cv2.waitKey(TIME_STEP)

# set motor position (undefined for now)
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# set velocity motor
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

oam_ofm_speed = [0, 0]

# run simulation
while robot.step(TIME_STEP) != -1:
    ticks += 1

    cameraData = camera.getImage()
    image = np.frombuffer(cameraData, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))

    # process camera sensor output
    process_image(image)
    
    # update distance sensors values
    ps_value[0] = ps0.getValue() - 300 if ps0.getValue() - 300 > 0 else 0
    ps_value[1] = ps1.getValue() - 300 if ps1.getValue() - 300 > 0 else 0
    ps_value[2] = ps2.getValue() - 300 if ps2.getValue() - 300 > 0 else 0
    ps_value[3] = ps3.getValue() - 300 if ps3.getValue() - 300 > 0 else 0
    ps_value[4] = ps4.getValue() - 300 if ps4.getValue() - 300 > 0 else 0
    ps_value[5] = ps5.getValue() - 300 if ps5.getValue() - 300 > 0 else 0
    ps_value[6] = ps6.getValue() - 300 if ps6.getValue() - 300 > 0 else 0
    ps_value[7] = ps7.getValue() - 300 if ps7.getValue() - 300 > 0 else 0

    # update ground sensors values
    gs_value[0] = gs_left.getValue()
    gs_value[1] = gs_right.getValue()
    gs_value[2] = gs_center.getValue()

    # we must stop
    if smp_stop and ticks - smp_ticks < 100:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        continue

    # motors speed (left and right)
    speed = [0.0, 0.0]

    # line following
    line_following()

    # update speed
    speed[0] = lfm_speed[0]
    speed[1] = lfm_speed[1]

    # avoid obstacle
    avoid_obstacle()

    # line leaving and obstacle following handle
    line_leaving(oam_side)
    obstacle_following(oam_side)

    # line leaving inhibited the obstacle following?
    if llm_inibit_ofm_speed:
        ofm_speed[0] = 0.0
        ofm_speed[1] = 0.0

    oam_ofm_speed[0] = oam_speed[0] + ofm_speed[0]
    oam_ofm_speed[1] = oam_speed[1] + ofm_speed[1]

    # is avoiding obstacle and following obstacle? update motor speed
    if oam_active or ofm_active:
      speed[0] = oam_ofm_speed[0]
      speed[1] = oam_ofm_speed[1]
    
    # entering to line
    line_entering(oam_side)

    # line entering is active? update the motor speed
    if lem_active:
      speed[0] = lem_speed[0]
      speed[1] = lem_speed[1]

    # set motors speed
    left_motor.setVelocity(0.00628 * speed[0])
    right_motor.setVelocity(0.00628 * speed[1])