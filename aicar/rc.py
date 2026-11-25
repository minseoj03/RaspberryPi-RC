#############################################################################################
#                                                               
# Auto Drive Car                                                    <rc.py>
#
# 선형 회귀 직선(Linear Regression Line)으로 동작하는 자율주행 자동차
#
# AUG 8 2024
#
# SAMPLE Electronics co.                                        
# http://www.ArduinoPLUS.cc                                     
#                                                               
#############################################################################################
pixthr = 128                # Pixel Threshold '-': 감소 '+','=': 증가
LANE_WHITE = True           # 선 색상 True: 힌색 차선 검정 바탕 False: 검정 차선 힌색 바탕
Dcode = 'X'                 # 회귀 직선의 기준 축 
#
WIN_GAP_X = 300             # 제어 윈도우 X 최소 폭
WIN_GAP_Y = 100             # 제어 윈도우 Y 최소 폭
xOFFSET = 200
WIN_XL_ORG = 400 - xOFFSET  # 제어 윈도우 왼쪽 X 값
WIN_XR_ORG = 400 + xOFFSET  # 제어 윈도우 오른쪽 X 값
WIN_YU_ORG = 300            # 제어 윈도우 위쪽 Y 값
WIN_YD_ORG = 459            # 제어 윈도우 아래쪽 Y값

WIN_XL = WIN_XL_ORG         # [0] 제어 윈도우 왼쪽 X 값
WIN_XR = WIN_XR_ORG         # [1] 제어 윈도우 오른쪽 X 값
WIN_YU = WIN_YU_ORG         # [2] 제어 윈도우 위쪽 Y 값
WIN_YD = WIN_YD_ORG         # [3] 제어 윈도우 아래쪽 Y값
#--------------------------------------------------------------------------------------------
LINE_SAPCE_X = 50           # X Window 등분 수
LINE_SPACE_Y = 20           # Y Window 등분 수
# Library Import ============================================================================
import os                   # 파일, 디렉토리 존재 유무 확인, 디렉토리 생성 라이브러리
import sys                  # 커맨드 라인 인수(이미지 저장 디렉토리, 레코딩 시작번호)처리 라이브러리 
import numpy as np          # 이미지 배열 생성(OpenCV 에서 사용) 라이브러리
import cv2 as cv            # 영상처리(OpenCV) 라이브러리 
import pickle               # python 객체 저장/읽기 라이브러리
import RPi.GPIO as GPIO     # Raspberry Pi 헤터핀 제어 라이브러리
import NEOPIXEL as nx       # NeoPixel 제어 라이브러리
# Constant ----------------------------------------------------------------------------------
RED     =   0,  0,255       # Red
GREEN   =   0,255,  0       # Green
BLUE    = 255,  0,  0       # Blue
MAGENTA = 255,  0,255       # Magenta(Pink)
CYAN    = 255,255,  0       # Cyan(Sky Blue)
YELLOW  =   0,255,255       # Yellow
WHITE   = 255,255,255       # White
GRAY    =  32, 32, 32       # Gray
BLACK   =   0,  0,  0       # Black
#--------------------------------------------------------------------------------------------
VIDEO_X = 640               # Video X Size (960x240 / 960x800 / 1600x240)
VIDEO_Y = 480               # Video Y Size
mouseX = 400                # 마우스 X 초기 위치
mouseY = 420                # 마우스 Y 초기 위치
mL = 0                      # 왼쪽 모터의 PWM 값이며 절대 값으로 0 부터 100까지(99 아님)
mR = 0                      # 오른쪽 모터의 PWM 값이며 절대 값으로 0 부터 100까지(99 아님)
# GPIO --------------------------------------------------------------------------------------
MOTOR_L_PWM = 12            # GPIO.12  왼쪽 모터 펄스 푹 변조(Pulse Width Modulation)
MOTOR_L_DIR = 5             # GPIO.5   원쪽 모터 방향 (Motor Rotation Direction)
MOTOR_R_PWM = 13            # GPIO.13  오른쪽 모터 펄스 폭 변조(Pulse Width Modulation)
MOTOR_R_DIR = 6             # GPIO.6   오른쪽 모터 방향 (Motor Rotation Direction)
# Video Window ------------------------------------------------------------------------------
cam = cv.VideoCapture(0,cv.CAP_V4L)   # 카메라 객체 생성
cam.set(3,VIDEO_X)                    # 카메라 입력 화면 X 크기
cam.set(4,VIDEO_Y)                    # 카메라 입력 화면 Y 크기
# Set up ------------------------------------------------------------------------------------
GPIO.setmode(GPIO.BCM)                # GPIO 핀 번호를 BCM 방식으로 설정 
GPIO.setwarnings(False)               # 부팅시 경고 메시지 출력(터미널 창에서)안보기로 설정
GPIO.setup(MOTOR_L_PWM,GPIO.OUT)      # 왼쪽 모터 PWM 신호 출력 핀 
GPIO.setup(MOTOR_L_DIR,GPIO.OUT)      # 왼쪽 모터 방향 신호 출력 핀
GPIO.setup(MOTOR_R_PWM,GPIO.OUT)      # 오른쪽 모터 PWM 신호 출력 핀 
GPIO.setup(MOTOR_R_DIR,GPIO.OUT)      # 오른쪽 모터 방향 신호 출력 핀
MOTOR_L = GPIO.PWM(MOTOR_L_PWM,500)   # 왼쪽 모터 PWM 주파수 500Hz
MOTOR_R = GPIO.PWM(MOTOR_R_PWM,500)   # 오른쪽 모터 PWM 주파수 500Hz
MOTOR_L.start(0)                      # 왼쪽 모터 펄스폭 변조(PWM)값 0 으로 시작
MOTOR_R.start(0)                      # 오른쪽 모터 펄스폭 변조(PWM)값 0 으로 시작
#--------------------------------------------------------------------------------------------
viewWin = np.zeros((480,800,3),np.uint8)     # 표시되는 윈도우 가로, 세로, 컬러층, 8비트
msgBoxL = cv.imread('./_IMAGE/rcL1.png',cv.IMREAD_COLOR)
msgBoxR = cv.imread('./_IMAGE/rcR1.png',cv.IMREAD_COLOR)
#--------------------------------------------------------------------------------------------
cv.namedWindow('Out',cv.WND_PROP_FULLSCREEN)
cv.setWindowProperty('Out', cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
# Motor Run Function ------------------------------------------------------------------------
def motorRun(leftMotor, rightMotor):
    #return
    if leftMotor>100: leftMotor = 100        # 왼쪽 모터 전진 최대 PWM 값 (100: 최고속도)
    if leftMotor<-100: leftMotor = -100      # 왼쪽 모터 후진 최대 PWM 값 (100: 최고속도)

    if (leftMotor>=0): GPIO.output(MOTOR_L_DIR,GPIO.HIGH)   # 왼쪽 모터 전진
    else: GPIO.output(MOTOR_L_DIR,GPIO.LOW)  # 왼쪽 모터 후진
    #----------------------------------------------------------------------------------------
    if rightMotor>100: rightMotor = 100      # 오른쪽 모터 전진 최대 PWM 값 (100: 최고속도)
    if rightMotor<-100: rightMotor = -100    # 오른쪽 모터 후진 최대 PWM 값 (100: 최고속도)

    if  rightMotor >= 0: GPIO.output(MOTOR_R_DIR,GPIO.HIGH)   # 오른쪽 모터 전진
    else: GPIO.output(MOTOR_R_DIR,GPIO.LOW)  # 오른쪽 모터 후진
    #----------------------------------------------------------------------------------------
    MOTOR_L.ChangeDutyCycle(abs(leftMotor))  # 왼쪽 모터 PWM 값 지정
    MOTOR_R.ChangeDutyCycle(abs(rightMotor)) # 오른쪽 모터 PWM 값 지정

# Main Loop =================================================================================
def main():

    global WIN_XL, WIN_XR, WIN_YU, WIN_YD, pixthr, Dcode, LANE_WHITE, mL, mR
    global viewWin

    nx.lamp(255,255,255)               # 전조등 On

    preKey = ord(' ')                  # 초기 키 값으로 space 를 지정합니다.
    dataX = []
    dataY = []
    autoRun = False

    #----------------------------------------------------------------------------------------
    while(cam.isOpened()):             # 카메라 활성화 되어 있므면 무한 반복

        sample_x = np.linspace(WIN_XL+5, WIN_XR-5, LINE_SAPCE_X)
        sample_y = np.linspace(WIN_YU+5, WIN_YD-5, LINE_SPACE_Y)

        ret, frame = cam.read()        # 카메라로부터 한장의 정지화면 데이터를 가져 옵니다.
        viewWin[0:480,0:80] = msgBoxL 
        viewWin[0:480,720:800] = msgBoxR 
        #------------------------------------------------------------------------------------
        viewWin[0:480,80:720] = frame[0:480,0:640]  # 카메라 전체 이미지 
        #------------------------------------------------------------------------------------
        roadImg = viewWin[WIN_YU:WIN_YD,80+WIN_XL:80+WIN_XR]  # frame 화면이므로 +80
        #------------------------------------------------------------------------------------
        cv.rectangle(viewWin,(WIN_XL,WIN_YU),(WIN_XR,WIN_YD),CYAN,1) # 데이터 입력 영역
        #------------------------------------------------------------------------------------
        dataX.clear(); dataY.clear()

        for y in sample_y:
            y = int(y)
            for x in sample_x:
                x = int(x)
                p = viewWin[y,x]
                g = int(0.299*p[2]+0.587*p[1]+0.114*p[0])  # Color 를 Gray로 변환
                
                c = RED
                if LANE_WHITE:
                    if g>pixthr:   # 검은 바탕에 힌색 라인 검출
                        c = GREEN
                        dataX.append(x); dataY.append(y)       #
                else:
                    if g<pixthr:   # 검은 바탕에 힌색 라인 검출
                        c = GREEN
                        dataX.append(x); dataY.append(y)       #

                cv.circle(viewWin,(x,y),3,c,1)

        x = np.array(dataX); y = np.array(dataY)
        x = x - 400; y = int((WIN_YD+WIN_YU)/2) - y  # LCD 좌표계 값을 수학 좌표계로 변환한다.
   
        #x = np.array([-30, -20, -10, 0, 10]); y = np.array([-20, -10,  0, 10, 200]) # 예제 데이터
        #-----------------------------------------------------------------------------------

        xs = WIN_XL-400; ys = -int((WIN_YD-WIN_YU)/2)
        xe = WIN_XR-400; ye = int((WIN_YD-WIN_YU)/2)
        w = 0; b = 0

        try:
            #A = np.vstack([x, np.ones(len(x))]).T
            #w, b = np.linalg.lstsq(A, y, rcond=None)[0]

            n = len(x)                  # 입력 데이터 x 의 개수(or y)
            sx = np.sum(x)              # xi 의 합
            sy = np.sum(y)              # yi 의 합
            sxx = np.sum(x*x)           # xi * xi 의 합
            syy = np.sum(y*y)           # yi * yi 의 합
            sxy = np.sum(x*y)           # xi * yi 의 합 

            Dx = n*sxx - sx*sx          # X 좌표계 Determinant
            Dy = n*syy - sy*sy          # Y 좌표계 Determinant
        
            x1 = 0; y1 = ys             # 초기 값
            x2 = 0; y2 = ye             # 초기 값
    
            if abs(Dx) > abs(Dy):       # X 좌표계  (증명이 필요함)

                wx = (n*sxy - sx*sy)/Dx     # X 좌표계 기울기
                bx = (sxx*sy - sxy*sx)/Dx   # X 좌표계 절편

                cv.putText(viewWin,f'{wx:>6.2f}',(730, 200),cv.FONT_HERSHEY_PLAIN,1,WHITE) 
                cv.putText(viewWin,f'{bx:>6.2f}',(730, 240),cv.FONT_HERSHEY_PLAIN,1,WHITE) 

                w = wx; b = bx; Dcode = 'X'
            
                x1 = xs; y1 = int(w * x1 + b)
                if y1 < ys:
                    y1 = ys; x1 = int((y1-b)/w)
                elif y1 > ye:
                    y1 = ye; x1 = int((y1-b)/w)

                x2 = xe; y2 = int(w * x2 + b);
                if y2 > ye:
                    y2 = ye; x2 = int((y2-b)/w)
                elif y2 < ys:
                    y2 = ys; x2 = int((y2-b)/w)

            elif abs(Dx) < abs(Dy):        # Y 좌표계
                wy = (n*sxy - sx*sy)/Dy     # Y 좌표계 기울기
                by = (syy*sx - sxy*sy)/Dy   # Y 좌표계 절편

                cv.putText(viewWin,f'{wy:>6.2f}',(730, 280),cv.FONT_HERSHEY_PLAIN,1,WHITE) 
                cv.putText(viewWin,f'{by:>6.2f}',(730, 320),cv.FONT_HERSHEY_PLAIN,1,WHITE) 

                w = wy; b = by; Dcode = 'Y'

                y1 = ys; x1 = int(w * y1 + b)
                if x1 < xs:
                    x1 = xs; y1 = int((x1-b)/w)
                elif x1 > xe:
                    x1 = xe; y1 = int((x1-b)/w)

                y2 = ye; x2 = int(w * y2 + b);
                if x2 > xe:
                    x2 = xe; y2 = int((x2-b)/w)
                elif x2 < xs:
                    x2 = xs; y2 = int((x2-b)/w)

            if y1 > y2:                          # Y 좌표값이 작은쪽을 시작 좌표로 한다.
                x1, x2 = x2, x1; y1, y2 = y2, y1

        except:

            print('Error 발생')

        # X=0, Y=0 중심선 -------------------------------------------------------------------
        cv.line(viewWin,((int((WIN_XL+WIN_XR)/2)),(WIN_YU)),((int((WIN_XL+WIN_XR)/2)),(WIN_YD)),GREEN,1)
        cv.line(viewWin,(WIN_XL,int((WIN_YU+WIN_YD)/2)),(WIN_XR,int((WIN_YU+WIN_YD)/2)),GREEN,1)

        # 회귀 직선 --------------------------------------------------------------------------
        cv.arrowedLine(viewWin,(x1+400,int((WIN_YU+WIN_YD)/2)-y1),(x2+400,int((WIN_YU+WIN_YD)/2)-y2),BLACK,3,cv.LINE_AA)
        cv.arrowedLine(viewWin,(x1+400,int((WIN_YU+WIN_YD)/2)-y1),(x2+400,int((WIN_YU+WIN_YD)/2)-y2),WHITE,1,cv.LINE_AA)
        # -----------------------------------------------------------------------------------
        if Dx > Dy:
            cv.putText(viewWin,f'DX:{Dx} > DY:{Dy}',(85,30),cv.FONT_HERSHEY_PLAIN,2,WHITE) 
        elif Dx < Dy:
            cv.putText(viewWin,f'DX:{Dx} < DY:{Dy}',(85,30),cv.FONT_HERSHEY_PLAIN,2,WHITE) 
        else:
            cv.putText(viewWin,f'DX:{Dx} == DY:{Dy}',(85,30),cv.FONT_HERSHEY_PLAIN,2,WHITE) 

        cv.putText(viewWin,f'X1:{x1} Y1:{y1}',(330, WIN_YD+15),cv.FONT_HERSHEY_PLAIN,1,WHITE) 
        cv.putText(viewWin,f'X2:{x2} Y2:{y2}',(330, WIN_YU-5),cv.FONT_HERSHEY_PLAIN,1,WHITE) 


        cv.putText(viewWin,f'{xs:>4}',(WIN_XL-50, WIN_YU+10),cv.FONT_HERSHEY_PLAIN,1,WHITE) 
        cv.putText(viewWin,f'{ye:>4}',(WIN_XL-50, WIN_YU+25),cv.FONT_HERSHEY_PLAIN,1,WHITE) 
        cv.putText(viewWin,f'{xe:>4}',(WIN_XR+5, WIN_YU+10),cv.FONT_HERSHEY_PLAIN,1,WHITE) 
        cv.putText(viewWin,f'{ye:>4}',(WIN_XR+5, WIN_YU+25),cv.FONT_HERSHEY_PLAIN,1,WHITE) 

        cv.putText(viewWin,f'{xs:>4}',(WIN_XL-50, WIN_YD-15),cv.FONT_HERSHEY_PLAIN,1,WHITE) 
        cv.putText(viewWin,f'{ys:>4}',(WIN_XL-50, WIN_YD),cv.FONT_HERSHEY_PLAIN,1,WHITE) 
        cv.putText(viewWin,f'{xe:>4}',(WIN_XR+5, WIN_YD-15),cv.FONT_HERSHEY_PLAIN,1,WHITE) 
        cv.putText(viewWin,f'{ys:>4}',(WIN_XR+5, WIN_YD),cv.FONT_HERSHEY_PLAIN,1,WHITE) 

        t = 'B'
        if LANE_WHITE: t = 'W'
        cv.putText(viewWin,f'{t}',(755, 35),cv.FONT_HERSHEY_PLAIN,2,WHITE) 
        cv.putText(viewWin,f'{pixthr}',(760, 65),cv.FONT_HERSHEY_PLAIN,1,WHITE) 
        cv.putText(viewWin,f'{Dcode}',(755, 100),cv.FONT_HERSHEY_PLAIN,2,WHITE) 
        cv.putText(viewWin,f'{n}',(755,130),cv.FONT_HERSHEY_PLAIN,1,WHITE) 

        #------------------------------------------------------------------------------------
        if autoRun:
            #////////////////////////////////////////////////////////////////////////////////
            if (x1<0 and x2<0 and w>0) or (x1>0 and x2>0 and w<0):   # 직진
                mL = 90; mR = 90
            elif x2<0 and w<0:                                       # 좌회전
                mL = 30; mR = 90
            elif x2>0 and w>0:                                       # 우회전
                mL = 90; mR = 30
            else:                                                    # ?
                mL = 30; mR = 30
            #////////////////////////////////////////////////////////////////////////////////
            motorRun(mL, mR)
        #------------------------------------------------------------------------------------
        cv.imshow('Out', viewWin)           # 이미지를 LCD에 표시합니다.
        #------------------------------------------------------------------------------------
        keyBoard = cv.waitKey(1) & 0xFF     #print(hex(keyBoard)) # 키보드의 키 값 확인  
        if keyBoard == 0x1B or keyBoard == 0x09 or keyBoard == ord('x') or keyBoard == ord('X'):
            break                           # ESC / TAB / 'x' 프로그램 종료
        #------------------------------------------------------------------------------------
        if keyBoard == ord('g') or keyBoard == ord('G'):     # Auto Run Start
            autoRun = True
        #------------------------------------------------------------------------------------
        if keyBoard == ord('-') or keyBoard == ord('_'):     # Thresh 값 감소
           if pixthr>0: pixthr -= 1
        #------------------------------------------------------------------------------------
        if keyBoard == ord('+') or keyBoard == ord('='):     # Thresh 값 증가
           if pixthr<255: pixthr += 1
        # 카메라 입력 영역 조정 -----------------------------------------------------------------
        if keyBoard == ord('!'):                             # X(가로)방향 줄임
            if (WIN_XL <= WIN_GAP_X): WIN_XL += 2; WIN_XR -= 2
        if keyBoard == ord('@'):                             # X(가로)방향 늘림
            if (WIN_XL >= 2): WIN_XL -= 2; WIN_XR += 2
        if keyBoard == ord('#'):                             # Y-Up 내리기
            if ((WIN_YD-WIN_YU) > (WIN_GAP_Y+4)): WIN_YU += 2
        if keyBoard == ord('$'):                             # Y-Up 올리기
            if (WIN_YU > 152): WIN_YU -= 2
        if keyBoard == ord('%'):                             # Y-Down 올리기
            if ((WIN_YD-WIN_YU) > (WIN_GAP_Y+4)): WIN_YD -= 2
        if keyBoard == ord('^'):                             # Y-Down 내리기
            if (WIN_YD <= 477): WIN_YD += 2
        if keyBoard == ord(')'):                             # X, Y 원위치
            WIN_XL = WIN_XL_ORG                              # 제어 윈도우 왼쪽 X 값
            WIN_XR = WIN_XR_ORG                              # 제어 윈도우 오른쪽 X 값
            WIN_YU = WIN_YU_ORG                              # 제어 윈도우 위쪽 Y 값
            WIN_YD = WIN_YD_ORG                              # 제어 윈도우 아래쪽 Y값
        if keyBoard == ord('l') or keyBoard == ord('L'):   
            LANE_WHITE = not LANE_WHITE                      # 차선 색상 
        # 수동 차량 조작 ----------------------------------------------------------------------
        if keyBoard == 82:                                   # 전진 Arrow Up
            autoRun = False
            if preKey == ord(' '): preKey = keyBoard; mL = 80; mR = 80
            else: preKey = ord(' '); mL = 0; mR = 0
            motorRun(mL, mR)
        #------------------------------------------------------------------------------------
        if keyBoard == 84:                                   # 후진 Arrow Down
            autoRun = False
            if preKey == ord(' '): preKey = keyBoard; mL = -70; mR = -70
            else: preKey = ord(' '); mL = 0; mR = 0
            motorRun(mL, mR)
        #------------------------------------------------------------------------------------
        if keyBoard == 81:                                   # 좌 회전 Arrow Left
            autoRun = False
            if preKey == ord(' '): preKey = keyBoard; mL = -70; mR = 70
            else: preKey = ord(' '); mL = 0; mR = 0
            motorRun(mL, mR)
        #------------------------------------------------------------------------------------
        if keyBoard == 83:                                   # 우 회전 Arrow Right
            autoRun = False
            if preKey == ord(' '): preKey = keyBoard; mL = 70; mR = -70
            else: preKey = ord(' '); mL = 0; mR = 0
            motorRun(mL, mR)
        #------------------------------------------------------------------------------------
        if keyBoard == ord(' '):                             # Space 정지
            autoRun = False
            preKey = ord(' '); mL = 0; mR = 0
            motorRun(mL, mR)
    #----------------------------------------------------------------------------------------
    nx.lamp(0,0,0)                          # 전조등 Off
    MOTOR_L.stop()                          # 왼쪽 모터 PWM(펄스 폭 변조) 정지
    MOTOR_R.stop()                          # 오른쪽 모터 PWM(펄스 폭 변조) 정지
    GPIO.cleanup()                          # GPIO 초기화
    cam.release()                           # 카메라 자원을 반납
    cv.destroyAllWindows()                  # 열려 있는 모든 윈도우를 닫기

#============================================================================================
if __name__ == '__main__':
    # OpenCV 라이브러리 버젼 표시
    print('\n')
    print('OpenCV Version: ', cv.__version__)
    print('\n')

    # Mouse Event ---------------------------------------------------------------------------
    #cv.namedWindow('Out')                              # 윈도우 창을 생성합니다.
    #cv.setMouseCallback('Out', controlMain, viewWin)   # 마우스 제어 설정
    #----------------------------------------------------------------------------------------
    pfile = 'rc.pickle'                # 가져올 pickle 파일 이름

    if os.path.exists(pfile):          # pickle 파일 있으면 읽어들임
        with open(pfile, 'rb') as f:
            d = pickle.load(f)           

            WIN_XL = d[0]
            WIN_XR = d[1]
            WIN_YU = d[2]
            WIN_YD = d[3]
            pixthr = d[4]              # Pixel Threshold 
            LANE_WHITE = d[5]          # 차선 색상 True: 힌색 차선 검정 바탕 False: 검정 차선 힌색 바탕
    #----------------------------------------------------------------------------------------
    main()                             # 프로그램 실행
    #----------------------------------------------------------------------------------------
    d = [WIN_XL, WIN_XR, WIN_YU, WIN_YD, pixthr, LANE_WHITE]
    with open(pfile, 'wb') as f:
        pickle.dump(d, f)              #

    print('WIN_XL:', WIN_XL, 'WIN_XR:', WIN_XR, 'WIN_YU:', WIN_YU, 'WIN_YD:', WIN_YD)

#############################################################################################

