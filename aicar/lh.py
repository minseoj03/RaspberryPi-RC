#############################################################################################
# Light House                                                            <lh.py>
#
# 신호등을 설정합니다. 
#
# 빨간색, 녹색, 노란색 신호에 대하여 
#    1. 크기(원 지름), 
#    2. 형태(진원/타원), 
#    3. 타원의 각도 
#    4. 색상값(Value), 
#    5. 위치
#    6. 점등 시간을 설정할 수 있습니다.
#
# 프로그램 기동시 클리어 화면 모드로 시작합니다.
# 설정 모드로 동작하려면 마우스 오른 쪽 버튼을 클릭합니다.
# 
# 객체 기준점을 마우스 왼쪽 버튼으로 선택하면 객체가 선택되고 객체 색상이 힌색으로 변합니다.
# 마우스를 움직여 적정한 값을 설정합니다.
# 마우스 왼쪽 버튼 또는 오른쪽 버튼을 클릭하면 선택한 객체가 해지 됩니다.
#
# 프리런 모드와 센서 동작 모드가 있습니다.
# 프리런 모드: Green -> Red -> Yellow 순서로 지정된 Time 값 만큼 표시됩니다.
# 특정 색상을 표시 하지 않으려면 Value 를 0 으로 Time을 0.1 로 설정합니다.
#
# 센서 동작 모드:
# Green 신호는 거리센서의 입력값이 설정된 값보다 크면 무한 대기합니다. 
# 작으면 (차가 신호등 근접) Red -> Yellow 순서로 동작합니다.
# 센서는 설정 값을 4CM 보다 크게 하여야 동작하며 신호등과 차량의 거리를 수평 바 그래프로 실시간 표시합니다.
#
# 객체 아이콘:
# Range: 거리 측정 센서의 동작을 선택합니다. 4CM 이상 값으로 설정하면 센서 모드로 동작합니다.
# Green Value: Green 색상 값(HSV) 입니다.
# Green Time: Green 신호 표시 시간(초단위) 입니다.
# Red Value: Red 색상 값(HSV) 입니다.
# Red Time: Red 신호 표시 시간(초단위) 입니다.
# Yellow Value: Yellow 색상 값(HSV) 입니다.
# Yellow Time: Yellow 신호 표시 시간(초단위) 입니다.
# Circle(Ellipse) Rigid: 반지름 입니다.
# Circle(Ellipse) Center: 중심 좌표(X, Y) 입니다.
# Circle(Ellipse) Aspect: 가로 세로 비율 입니다.
# Circle(Ellipse) Angle: 각도 입니다.
# [X] : 파라메터를 저장하고 Shut Down(Power Off) 합니다.
# [S] : 파리메터를 저장하고 System OS 로 돌아갑니다.
# [C] : 화면을 클리어 합니다. 마우스 오른쪽 버튼을 누르면 설정 모드로 전환됩니다.
#
# 마우스로 [X] 객체를 선택하면 Raspberry Power Off (Sut Down) 됩니다.
# 시스템 OS 로 돌아가기 원하면 'X', 또는 'ESC' 를 입력합니다.
#
#
#
#
# 2024-01-01
#
# SAMPLE Electronics co.                                        
# http://www.ArduinoPLUS.com
#                                                               
import os                   # File 처리 라이브러리
import numpy as np          # 이미지 배열 생성(OpenCV 에서 사용) 라이브러리
import cv2                  # 영상처리(OpenCV) 라이브러리 
import time                 # 시간(time)라이브러리
import pickle               # python 객채 압축/복원 라이브러리
import subprocess
import RPi.GPIO as GPIO     # Raspberry Pi GPIO 라이브러리
from operator import itemgetter
# Constant ----------------------------------------------------------------------------------
RED     =   0,  0,255       # Red
GREEN   =   0,255,  0       # Green
BLUE    = 255,  0,  0       # Blue
MAGENTA = 255,  0,255       # Magenta(Pink)
CYAN    = 255,255,  0       # Cyan(Sky Blue)
YELLOW  =   0,255,255       # Yellow
WHITE   = 255,255,255       # White
BLACK   =   0,  0,  0       # Black
GRAY    =  96, 96, 96       # Gray
MELON   =   0,180,255       # Melon
#--------------------------------------------------------------------------------------------
mouseX = 0                  # 마우스 X좌표
mouseY = 0                  # 마우스 Y좌표
mouseButtenLeft = False     # 마우스 왼쪽 버튼
mouseButtenToggle = False   # 마우스 오른쪽 버튼
selectedObject = None       # 객체 선택
SET_MODE = False            # True: 설정 모드 / False: 신호등 동작 모드
POWER_OFF = False           # 프로그램 실행 정지하고 Raspberry Pi 의 동작을 멈춥니다.
#--------------------------------------------------------------------------------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO_TRIGGER = 14           # Trig 핀
GPIO_ECHO = 15              # Echo 핀
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#--------------------------------------------------------------------------------------------
viewWin = np.zeros((480,800,3),np.uint8)  # 표시되는 윈도우 가로, 세로, 컬러층, 8비트
#--------------------------------------------------------------------------------------------
cv2.namedWindow('Out',cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty('Out', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
# Mouse Callback Function -------------------------------------------------------------------
def controlMain(event,x,y,flags,param):
    '''
    마우스 이벤트 발생시 처리되는 함수 입니다.
    3 개의 마우스 버튼 Up/Down 이벤트와 X,Y 이동 이벤트를 처리합니다.
    '''
    global mouseX, mouseY, mouseButtenLeft, mouseButtenToggle, selectedObject, SET_MODE
    
    mouseX = x; mouseY = y                # 마우스 좌표값 X, Y를 전역변수 mouseX, mouseY에 저장

    if event == cv2.EVENT_LBUTTONDOWN:    # print('L-Down') # 마우스 왼쪽 버튼 이벤트 발생
        if SET_MODE:
            mouseButtenLeft = True

    if event == cv2.EVENT_RBUTTONDOWN:    # print('R-Down') # 마우스 오른쪽 버튼 이벤트 발생
        mouseButtenToggle = False
        selectedObject = None
        SET_MODE = True

#--------------------------------------------------------------------------------------------
def sonicDistance():
    #
    #
    #         +-+
    #         | |
    #         | |
    #         | |
    #---------+ +------------------------------------------------- Triger
    #
    #                           +------------------+
    #                           |                  |
    #                           |<---------------->|
    #                           |   Elapsed Time   |
    #---------------------------+                  +--------------- Echo
    #                           Start             Stop
    #

    elapsedTime = 0

    GPIO.output(GPIO_TRIGGER, True)               # Triger Pulse HIGH
    time.sleep(0.00001)                   
    GPIO.output(GPIO_TRIGGER, False)              # Triger Pulse Off
 
    watchdog = 0
    flag = True
    while GPIO.input(GPIO_ECHO) == 0:
        time.sleep(0.000001)
        watchdog += 1
        if watchdog > 1000:
            flag = False
            print('Low Over')
            break
    if not flag:                                  # ECHO 가 0 으로 고정되어 있으면 9999 에러
        return 8888

    StartTime = time.time()
    watchdog = 0
    while GPIO.input(GPIO_ECHO) == 1:             # ECHO 가 High 에서 Low 로 되면 빠져 나간다.
        time.sleep(0.000001)
        watchdog += 1
        if watchdog > 2000:
            flag = False                          # Error
            print('High Over')
            break

    if flag:                                      # EHCO 펄스가 정상일 때 
        elapsedTime =  time.time() - StartTime    # ECHO High 시간을 구한다.
        distance = (elapsedTime * 343000) / 2     # 거리로 환산한다.
    else:
        distance = 9999                           # High 에러
    return int(distance)                          # Sec(초) 단위로 측정값 반환

#--------------------------------------------------------------------------------------------
def constrain(value, minNum, maxNum):
    '''
    Arduino 의 constrain() 함수와 같습니다.
    value 값이 minNum 보다 작으면 minMum 을 반환하고 maxNum 보다 크면 maxNum 을 반환합니다.
    value 값이 minNum 보다 크고 maxNum 보다 작으면 value 를 그대로 반환합니다.
    '''
    if value < minNum: 
        r = minNum
    elif value > maxNum:
        r = maxNum
    else:
        r = value
    return int(r)
#--------------------------------------------------------------------------------------------
def logScale(v):
    '''
    Semi Log 스케일로 반환 합니다.
    입력 범위: 0 ~ 255 
    출력 범위: 0.1 ~ 100.0
    '''
    return(pow(1.01826,v) -1 + 0.1)

# Main Loop =================================================================================
def main():

    global mouseButtenLeft, mouseButtenToggle, selectedObject
    global SET_MODE, POWER_OFF

    BOXStart = 20       # 박스 시작 Absolute X
    BOXDOY = 30         # 객체 원의 Increment Y
    BOXDX = 256         # 박스의 가로 길이
    BOXDY = 60          # 박스의 세로 길이
    BOXDVTX = 80        # Value 문자 Inc X
    BOXDVTY = 40        # Value 문자 Inc Y
    BOXDVTNX = 275      # Value, Time 값 Inc X
    BOXDVTNY = 35       # Value, Time 값 Inc Y

    BLUEAVY = 25

    GREENAVY = 80
    GREENATY = 145

    REDAVY = 215
    REDATY = 280

    YELLOWAVY = 350
    YELLOWATY = 415

    circleRadius = 160                          # Pickle[0]   신호 반지름
    blueValueX = 0                              # Pickle[1]
    greenValueX = 255                           # Pickle[2]
    greenTimeX = 50                             # Pickle[3]
    redValueX = 255                             # Pickle[4]   빨간 신호 Value
    redTimeX = 50                               # Pickle[5]
    yellowValueX = 255                          # Pickle[6]
    yellowTimeX = 20                            # Pickle[7]
    circleCenterX = 560                         # Pickle[8] 신호 디스프레이 X
    circleCenterY = 240                         # Pickle[9] 신호 디스프레이 Y
    cirWidth = circleRadius                     # Pickle[10] 신호 가로 길이
    cirHeight = circleRadius                    # Pickle[11] 신호 세로 길이
    cirAngle = 0                                # Pickle[12] 신호 각도

    pfile = 'lh.pickle'                         # 가져올 pickle 파일 이름
    if os.path.exists(pfile):                   # pickle 파일 있으면 읽어들임
        with open(pfile, 'rb') as f:
            d = pickle.load(f)           

            circleRadius=d[0]; 
            blueValueX=d[1];
            greenValueX=d[2]; 
            greenTimeX=d[3]; 
            redValueX=d[4]; 
            redTimeX=d[5]; 
            yellowValueX=d[6]; 
            yellowTimeX=d[7]; 
            circleCenterX=d[8]; 
            circleCenterY=d[9]; 
            cirWidth=d[10]; 
            cirHeight=d[11]; 
            cirAngle=d[12]
        
    greenTimeXL = logScale(greenTimeX)          # Log Scale Green Time
    redTimeXL = logScale(redTimeX)              # Log Scale Red Time
    yellowTimeXL = logScale(yellowTimeX)        # Log Scale Yellow Time     

    circleRigidX = circleCenterX
    circleRigidY = circleCenterY-circleRadius

    exitX = 765; exitY = 30                     # Power Off 아이콘 위치
    saveX = 765; saveY = 160                    # Save Exit 아이콘 위치
    clearWinX = 765; clearWinY = 450            # Clear Screen 아이콘 위치 

    # Object 리스트에 11 개의 객체 이름과 객체 기준점과 마우스와의 거리 정보
    objectList = [
                  ['BLUE_VALUE',0],          # 거리 측정기 Value 값
                  ['GREEN_VALUE',0],         # 녹색 신호등 Value 값
                  ['GREEN_TIME',0],          # 녹색 신호등 지연시간
                  ['RED_VALUE',0],           # 빨간 신호등 Value 값
                  ['RED_TIME',0],            # 빨간 신호등 지연시간
                  ['YELLOW_VALUE',0],        # 노란 신호등 Value 값
                  ['YELLOW_TIME',0],         # 노란 신호등 지연시간 
                  ['CIRCLE_CENTER',0],       # 원의 중심 좌표
                  ['CIRCLE_RIGID',0],        # Rigir Circle 가로 세로 비가 1:1 을 유지 
                  ['CIRCLE_DIAGONAL',0],     # Ellipsed Circle 가로 세로 비가 X:Y 비율
                  ['CIRCLE_ANGLE',0],        # Ellipsed Circle 가로 세로 비가 X:Y 비율
                  ['SHUT_DOWN',0],           # Shut Down (Power Off)
                  ['SYSTEM',0],              # Save Parameter
                  ['CLEAR',0]                # Exit(Without Save Parameter)
                 ]                           

    GREEN_SEQ = True
    RED_SEQ = False
    YELLOW_SEQ = False

    RANGE_MIN = 40                           # 거리계 최소거리

    green_time_start = time.time()
    red_time_start = 0
    yellow_time_start = 0

    sandGlass = 0.0

    rangeData = 0
    #----------------------------------------------------------------------------------------
    while True:

        viewWin[:,:] = BLACK                 # 스크린 클리어

        if blueValueX > RANGE_MIN:           # 센서 모드에서 거리 측정
            rangeData = sonicDistance()

        if GREEN_SEQ:
            cv2.ellipse(viewWin,(circleCenterX,circleCenterY),(cirWidth, cirHeight),cirAngle, 0, 360,(0,greenValueX,0),-1,cv2.LINE_AA)
            sandGlass = greenTimeXL+green_time_start-time.time()
            if not (RANGE_MIN < blueValueX < rangeData): 
                if ((green_time_start+greenTimeXL ) < time.time()): 
                    GREEN_SEQ = False
                    RED_SEQ = True
                    red_time_start = time.time()

        if RED_SEQ:
            cv2.ellipse(viewWin,(circleCenterX,circleCenterY),(cirWidth, cirHeight),cirAngle, 0, 360,(0,0,redValueX),-1,cv2.LINE_AA)
            sandGlass = redTimeXL+red_time_start-time.time() 
            if (red_time_start + redTimeXL ) < time.time():
                RED_SEQ = False
                YELLOW_SEQ = True
                yellow_time_start = time.time()
        if YELLOW_SEQ:
            cv2.ellipse(viewWin,(circleCenterX,circleCenterY),(cirWidth, cirHeight),cirAngle, 0, 360,(0,yellowValueX,yellowValueX),-1,cv2.LINE_AA)
            sandGlass = yellowTimeXL+yellow_time_start-time.time()
            if (yellow_time_start + yellowTimeXL ) < time.time():
                YELLOW_SEQ = False
                GREEN_SEQ = True
                green_time_start = time.time()

        # 설정 모드 --------------------------------------------------------------------------
        if SET_MODE:
            # Blue Value --------------------------------------------------------------------
            if blueValueX > RANGE_MIN:
                cv2.rectangle(viewWin,(BOXStart+0, 5),(BOXStart+rangeData, 10),CYAN,-1)     # ULTRA Sonic Distance Bar
                cv2.putText(viewWin,f'{rangeData/10:.1f} CM',(620,BLUEAVY+30),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,CYAN)
                cv2.putText(viewWin,f'{blueValueX/10:.1f} CM',(BOXStart+blueValueX-18,BLUEAVY+30),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,CYAN)
            if selectedObject == 'BLUE_VALUE': C = WHITE
            else: C = CYAN
            cv2.circle(viewWin,(BOXStart+blueValueX,BLUEAVY),10,C,-1,cv2.LINE_AA)
            # Sand Glass --------------------------------------------------------------------
            if not (GREEN_SEQ and blueValueX >= RANGE_MIN):
                cv2.putText(viewWin,f'{sandGlass:.2f}',(circleCenterX-70,circleCenterY+5),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,WHITE)
            # Green Value -------------------------------------------------------------------
            cv2.rectangle(viewWin,(BOXStart+0, GREENAVY+0),(BOXStart+BOXDX,  GREENAVY+BOXDY),GREEN,2)     # Value Box 외곽선 45
            cv2.putText(viewWin,'Value',(BOXStart+BOXDVTX,GREENAVY+BOXDVTY),cv2.FONT_HERSHEY_COMPLEX_SMALL,1.5,GREEN)
            cv2.putText(viewWin,f'{greenValueX}',(BOXStart+BOXDVTNX,GREENAVY+BOXDVTNY),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,GREEN)
            if selectedObject == 'GREEN_VALUE': C = WHITE
            else: C = GREEN
            cv2.circle(viewWin,(BOXStart+greenValueX,GREENAVY+BOXDOY),10,C,-1,cv2.LINE_AA)
            # Green Time --------------------------------------------------------------------
            cv2.rectangle(viewWin,(BOXStart+0, GREENATY+0),(BOXStart+BOXDX, GREENATY+BOXDY),GREEN,2)     # Value Box 외곽선 115
            cv2.putText(viewWin,'Time',(BOXStart+BOXDVTX,GREENATY+BOXDVTY),cv2.FONT_HERSHEY_COMPLEX_SMALL,1.5,GREEN)
            cv2.putText(viewWin,f'{greenTimeXL:.1f}',(BOXStart+BOXDVTNX,GREENATY+BOXDVTNY),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,GREEN)
            if selectedObject == 'GREEN_TIME': C = WHITE
            else: C = GREEN
            cv2.circle(viewWin,(BOXStart+greenTimeX,GREENATY+BOXDOY),10,C,-1,cv2.LINE_AA)
            # Red Value ---------------------------------------------------------------------
            cv2.rectangle(viewWin,(BOXStart+0, REDAVY+0),(BOXStart+BOXDX,  REDAVY+BOXDY),RED,2)     # Value Box 외곽선 45
            cv2.putText(viewWin,'Value',(BOXStart+BOXDVTX,REDAVY+BOXDVTY),cv2.FONT_HERSHEY_COMPLEX_SMALL,1.5,RED)
            cv2.putText(viewWin,f'{redValueX}',(BOXStart+BOXDVTNX,REDAVY+BOXDVTNY),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,RED)
            if selectedObject == 'RED_VALUE': C = WHITE
            else: C = RED
            cv2.circle(viewWin,(BOXStart+redValueX,REDAVY+BOXDOY),10,C,-1,cv2.LINE_AA)
            # Red Time ----------------------------------------------------------------------
            cv2.rectangle(viewWin,(BOXStart+0, REDATY+0),(BOXStart+BOXDX, REDATY+BOXDY),RED,2)     # Value Box 외곽선 115
            cv2.putText(viewWin,'Time',(BOXStart+BOXDVTX,REDATY+BOXDVTY),cv2.FONT_HERSHEY_COMPLEX_SMALL,1.5,RED)
            cv2.putText(viewWin,f'{redTimeXL:.1f}',(BOXStart+BOXDVTNX,REDATY+BOXDVTNY),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,RED)
            if selectedObject == 'RED_TIME': C = WHITE
            else: C = RED
            cv2.circle(viewWin,(BOXStart+redTimeX,REDATY+BOXDOY),10,C,-1,cv2.LINE_AA)
            # Yellow Value ------------------------------------------------------------------
            cv2.rectangle(viewWin,(BOXStart+0, YELLOWAVY+0),(BOXStart+BOXDX,  YELLOWAVY+BOXDY),YELLOW,2)     # Value Box 외곽선 45
            cv2.putText(viewWin,'Value',(BOXStart+BOXDVTX,YELLOWAVY+BOXDVTY),cv2.FONT_HERSHEY_COMPLEX_SMALL,1.5,YELLOW)
            cv2.putText(viewWin,f'{yellowValueX}',(BOXStart+BOXDVTNX,YELLOWAVY+BOXDVTNY),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,YELLOW)
            if selectedObject == 'YELLOW_VALUE': C = WHITE
            else: C = YELLOW
            cv2.circle(viewWin,(BOXStart+yellowValueX,YELLOWAVY+BOXDOY),10,C,-1,cv2.LINE_AA)
            # Yellow Time -------------------------------------------------------------------
            cv2.rectangle(viewWin,(BOXStart+0, YELLOWATY+0),(BOXStart+BOXDX, YELLOWATY+BOXDY),YELLOW,2)     # Value Box 외곽선 115
            cv2.putText(viewWin,'Time',(BOXStart+BOXDVTX,YELLOWATY+BOXDVTY),cv2.FONT_HERSHEY_COMPLEX_SMALL,1.5,YELLOW)
            cv2.putText(viewWin,f'{yellowTimeXL:.1f}',(BOXStart+BOXDVTNX,YELLOWATY+BOXDVTNY),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,YELLOW)
            if selectedObject == 'YELLOW_TIME': C = WHITE
            else: C = YELLOW
            cv2.circle(viewWin,(BOXStart+yellowTimeX,YELLOWATY+BOXDOY),10,C,-1,cv2.LINE_AA)
            # Exit --------------------------------------------------------------------------
            cv2.rectangle(viewWin,(exitX-15,exitY-15),(exitX+15,exitY+15),GRAY,-1)            # Exit  (765,30)
            cv2.putText(viewWin,'X',(exitX-12, exitY+10),cv2.FONT_HERSHEY_COMPLEX_SMALL,1.5,WHITE)
            # Save Parameter ----------------------------------------------------------------
            cv2.rectangle(viewWin,(saveX-15,saveY-15),(saveX+15,saveY+15),GRAY,-1)            # Save  (765,85)
            cv2.putText(viewWin,'S',(saveX-12,saveY+10),cv2.FONT_HERSHEY_COMPLEX_SMALL,1.5,WHITE)
            # Clear -------------------------------------------------------------------------
            cv2.rectangle(viewWin,(clearWinX-15,clearWinY-15),(clearWinX+15,clearWinY+15),GRAY,-1)          # Clear (765,450)
            cv2.putText(viewWin,'C',(clearWinX-12,clearWinY+10),cv2.FONT_HERSHEY_COMPLEX_SMALL,1.5,WHITE)
            # Circle(Ellipse) Center --------------------------------------------------------
            if selectedObject == 'CIRCLE_CENTER': C = WHITE
            else: C = GRAY                                     # Circle(Ellipse) Center
            cv2.circle(viewWin,(circleCenterX,circleCenterY),10,C,-1,cv2.LINE_AA) # 선택된 객체 기준원을 힌색으로 표시
            cv2.putText(viewWin,f'X:{circleCenterX}',(circleCenterX+15,circleCenterY-5),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,WHITE)
            cv2.putText(viewWin,f'Y:{circleCenterY}',(circleCenterX+15,circleCenterY+15),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,WHITE)
            # Circle (Rigid) ----------------------------------------------------------------
            if selectedObject == 'CIRCLE_RIGID': C = WHITE
            else: C = BLUE                                     # Circle
            cv2.circle(viewWin,(circleCenterX,circleCenterY-circleRadius),10,C,-1,cv2.LINE_AA)
            cv2.putText(viewWin,f'R:{circleRadius}',(circleCenterX+15,circleCenterY-circleRadius+7),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,WHITE)
            # Ellipse Aspect ----------------------------------------------------------------
            if selectedObject == 'CIRCLE_DIAGONAL': C = WHITE
            else: C = MELON                                    # Ellipse Aspect
            cv2.circle(viewWin,(circleCenterX-cirWidth,circleCenterY+cirHeight),10,C,-1,cv2.LINE_AA)
            cv2.putText(viewWin,f'W:{cirWidth} H:{cirHeight}',(circleCenterX-cirWidth+15,circleCenterY+cirHeight+7),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,WHITE)
            # Ellipse Angle -----------------------------------------------------------------
            if selectedObject == 'CIRCLE_ANGLE': C = WHITE
            else: C = MAGENTA                                  # Ellipse Angle
            x = int(circleRadius*np.cos(np.deg2rad(cirAngle))); y=int(circleRadius*np.sin(np.deg2rad(cirAngle))) 
            cv2.circle(viewWin,(circleCenterX+x,circleCenterY+y),10,C,-1,cv2.LINE_AA)
            cv2.putText(viewWin,f'A:{cirAngle}',(circleCenterX+x+15,circleCenterY+y+7),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,WHITE)
            # 객체 기준점과 마우스와의 거리 계산 (거리의 상대적인 크기 비교 sqart 연산하지 않음) --------
            objectList[0][1]  = (BOXStart+blueValueX-mouseX)**2+(BLUEAVY+BOXDOY-mouseY)**2
            objectList[1][1]  = (BOXStart+greenValueX-mouseX)**2+(GREENAVY+BOXDOY-mouseY)**2
            objectList[2][1]  = (BOXStart+greenTimeX-mouseX)**2+(GREENATY+BOXDOY-mouseY)**2
            objectList[3][1]  = (BOXStart+redValueX-mouseX)**2+(REDAVY+BOXDOY-mouseY)**2
            objectList[4][1]  = (BOXStart+redTimeX-mouseX)**2+(REDATY+BOXDOY-mouseY)**2
            objectList[5][1]  = (BOXStart+yellowValueX-mouseX)**2+(YELLOWAVY+BOXDOY-mouseY)**2
            objectList[6][1]  = (BOXStart+yellowTimeX-mouseX)**2+(YELLOWATY+BOXDOY-mouseY)**2
            objectList[7][1]  = (circleCenterX-mouseX)**2+(circleCenterY-mouseY)**2
            objectList[8][1]  = (circleCenterX-mouseX)**2+(circleCenterY-circleRadius-mouseY)**2
            objectList[9][1]  = (circleCenterX-cirWidth-mouseX)**2+(circleCenterY+cirHeight-mouseY)**2
            objectList[10][1] = int((circleCenterX+circleRadius*np.cos(np.deg2rad(cirAngle))-mouseX)**2+(circleCenterY+circleRadius*np.sin(np.deg2rad(cirAngle))-mouseY)**2)
            objectList[11][1] = (exitX-mouseX)**2+(exitY-mouseY)**2
            objectList[12][1] = (saveX-mouseX)**2+(saveY-mouseY)**2
            objectList[13][1] = (clearWinX-mouseX)**2+(clearWinY-mouseY)**2
            # 객체 선택 ----------------------------------------------------------------------
            if mouseButtenLeft:
                selectedObject = sorted(objectList,key=itemgetter(1))[0][0] # 마우스와 최 단거리 객체
                mouseButtenToggle = not mouseButtenToggle
                mouseButtenLeft = False                                     # 원 슈트(One Shoot)
            # 선택된 객체 처리 ----------------------------------------------------------------
            if mouseButtenToggle:
                if selectedObject == 'BLUE_VALUE':
                    if 15 <= mouseX <(286+256):
                        blueValueX = constrain(mouseX-BOXStart, 0, 511)
                if selectedObject == 'GREEN_VALUE':
                    if 15 <= mouseX <286:
                        greenValueX = constrain(mouseX-BOXStart, 0, 255)
                if selectedObject == 'GREEN_TIME':
                    if 15 <= mouseX <286:
                        greenTimeX = constrain(mouseX-BOXStart, 0, 255)
                        greenTimeXL = logScale(greenTimeX)
                if selectedObject == 'RED_VALUE':
                    if 15 <= mouseX <286:
                        redValueX = constrain(mouseX-BOXStart, 0, 255)
                if selectedObject == 'RED_TIME':
                    if 15 <= mouseX <286:
                        redTimeX = constrain(mouseX-BOXStart, 0, 255)
                        redTimeXL = logScale(redTimeX)
                if selectedObject == 'YELLOW_VALUE':
                    if 15 <= mouseX <286:
                        yellowValueX = constrain(mouseX-BOXStart, 0, 255)
                if selectedObject == 'YELLOW_TIME':
                    if 15 <= mouseX <286:
                        yellowTimeX = constrain(mouseX-BOXStart, 0, 255)
                        yellowTimeXL = logScale(yellowTimeX)
                if selectedObject == 'CIRCLE_CENTER':
                    if 50 <= mouseX < 730:
                        circleCenterX = mouseX
                    if 20 <= mouseY < 460:
                        circleCenterY = mouseY
                if selectedObject == 'CIRCLE_RIGID':
                    circleRadius = abs(circleCenterY - mouseY)
                    cirWidth = circleRadius              # 신호등 가로 길이
                    cirHeight = circleRadius             # 신호등 세로 길이
                if selectedObject == 'CIRCLE_DIAGONAL':
                    cirWidth =abs(circleCenterX - mouseX)
                    cirHeight = abs(circleCenterY - mouseY)
                if selectedObject == 'CIRCLE_ANGLE':
                    cirAngle = int(np.rad2deg(np.arctan2((mouseY-circleCenterY ),(mouseX -circleCenterX ))))
                if selectedObject == 'SHUT_DOWN':
                    POWER_OFF = True
                    break
                if selectedObject == 'SYSTEM':
                    break
                if selectedObject == 'CLEAR':
                    mouseButtenToggle = False
                    SET_MODE = False
            else:
                selectedObject = None
        #------------------------------------------------------------------------------------
        cv2.imshow('Out', viewWin)           # 이미지를 LCD에 표시
        #------------------------------------------------------------------------------------
        keyBoard = cv2.waitKey(1) & 0xFF     # 키보드 입력
        if keyBoard == 0x1B or keyBoard == 0x09 or keyBoard == ord('x') or keyBoard == ord('X'):
            break                            # ESC / TAB / 'x' 프로그램 종료
        if keyBoard == ord('c') or keyBoard == ord('C'):
            SET_MODE = not SET_MODE
    #----------------------------------------------------------------------------------------
    d = [circleRadius, 
         blueValueX,
         greenValueX, 
         greenTimeX, 
         redValueX, 
         redTimeX, 
         yellowValueX, 
         yellowTimeX,
         circleCenterX, 
         circleCenterY, 
         cirWidth, 
         cirHeight, 
         cirAngle]
    with open(pfile, 'wb') as f:
        pickle.dump(d, f)                            #
    cv2.destroyAllWindows()                          # 열려 있는 모든 윈도우를 닫기
# Mouse Event -------------------------------------------------------------------------------
cv2.namedWindow('Out')                               # 윈도우 창을 생성
cv2.setMouseCallback('Out', controlMain, viewWin)    # 마우스 제어 설정
#--------------------------------------------------------------------------------------------
main()                                               # 신호등 메인 프로그램 실행
#--------------------------------------------------------------------------------------------
if POWER_OFF:
    subprocess.call(['sudo', 'poweroff'])            # Raspberry Pi 를 Power Off 
#############################################################################################


