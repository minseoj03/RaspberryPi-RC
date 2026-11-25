# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Simple test for NeoPixels on Raspberry Pi
import time
import board
import neopixel
import sys

# Choose an open pin connected to the Data In of the NeoPixel strip, i.e. board.D18
# NeoPixels must be connected to D10, D12, D18 or D21 to work.
pixel_pin = board.D18
# The number of NeoPixels
num_pixels = 8
# The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
ORDER = neopixel.GRB
pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=1, auto_write=False, pixel_order=ORDER)

#---------------------------------------------------------------------------------------------
NEO_SUDO = True
frameCount = 0; neoIndex = 0
#---------------------------------------------------------------------------------------------
# NEOPIXEL을 사용하려면 관리자 모드로 프로그램 실행하여야 합니다. sudo 모드로 실행중인지 확인합니다.
try:
    pixels[0]=255, 0, 0; pixels[1]=182, 73, 0; pixels[2]=109, 146, 0; pixels[3]=36, 219, 0  
    pixels[4]=0, 219, 36; pixels[5]=0, 146, 109; pixels[6]=0, 73, 182; pixels[7]=0, 0, 255
    pixels.show()
except:
    print('\n\n\n\n')
    print('NEOPIXEL 을 사용하려면 sudo 를 사용하여 관리자 모드로 실행 하여야 합니다.\n')
    print('Example:\n$sudo python 프로그램.py\n\n')
    print('NEOPIXEL 사용하지 않고 실행하려면 "y"+Enter')
    print('시스템으로 돌아가서 sudo 모드로 다시 실핼하려면 Enter 를 입력하세요.\n\n\n\n\n\n\n\n\n')

    key = input('< Y: Yes>  <Any Key: No>')  # input()은 Key 가 입력될 때까지 무한 대기
    if key == 'y' or key == 'Y': 
        NEO_SUDO = False                     # NEOPIXEL(전조등 Lamp) 사용하지 않고 계속 진행
    else: 
        sys.exit()                           # 프로그램 종료

#---------------------------------------------------------------------------------------------
def lamp(r, g, b):
    global NEO_SUDO
    if NEO_SUDO:
        try:
            pixels.fill((r, g, b))
            pixels.show()
        except:
            NEO_SUDO = False
            print('\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')
            print('NEOPIXEL 을 사용하려면 sudo 를 사용하여 관리자 모드로 실행 하여야 합니다.\n')
            print('Example:\n$sudo python 프로그램')
            print('\n\n\n\n\n\n\n\n\n\n\n')
#---------------------------------------------------------------------------------------------
def policeCar1():
    global frameCount, neoIndex

    if NEO_SUDO and frameCount == 0:

        R = 9; r = 1; B = 9; b =1    # 자율주행시 카메라에 영향을 주지 않도록 색상 값을 낮게 하여야 합니다.
        if neoIndex>7: neoIndex=0
        if (neoIndex==0):
            pixels[0]=R,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,B
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==1):
            pixels[0]=r,0,0; pixels[1]=R,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,B; pixels[7]=0,0,b
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==2):
            pixels[0]=0,0,0; pixels[1]=r,0,0; pixels[2]=R,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,B; pixels[6]=0,0,b; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==3):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=r,0,0; pixels[3]=R,0,0  
            pixels[4]=0,0,B; pixels[5]=0,0,b; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==4):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,B  
            pixels[4]=R,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==5):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,B; pixels[3]=0,0,b  
            pixels[4]=r,0,0; pixels[5]=R,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==6):
            pixels[0]=0,0,0; pixels[1]=0,0,B; pixels[2]=0,0,b; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=r,0,0; pixels[6]=R,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다. 
        elif(neoIndex==7):
            pixels[0]=0,0,B; pixels[1]=0,0,b; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=r,0,0; pixels[7]=R,0,0
            neoIndex += 1                 # 처음으로 돌아간다

        pixels.show()

    frameCount += 1
    if frameCount > 5: frameCount = 0
#---------------------------------------------------------------------------------------------
def policeCar2():
    global frameCount, neoIndex

    if NEO_SUDO and frameCount == 0:

        R = 9; B = 9    # 자율주행시 카메라에 영향을 주지 않도록 색상 값을 낮게 하여야 합니다.
        if neoIndex>15: neoIndex = 0
        if (neoIndex==0):
            pixels[0]=R,0,0; pixels[1]=R,0,0; pixels[2]=R,0,0; pixels[3]=R,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==1):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==2):
            pixels[0]=R,0,0; pixels[1]=R,0,0; pixels[2]=R,0,0; pixels[3]=R,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==3):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==4):
            pixels[0]=R,0,0; pixels[1]=R,0,0; pixels[2]=R,0,0; pixels[3]=R,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==5):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==6):
            pixels[0]=R,0,0; pixels[1]=R,0,0; pixels[2]=R,0,0; pixels[3]=R,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==7):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==8):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,B; pixels[5]=0,0,B; pixels[6]=0,0,B; pixels[7]=0,0,B
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==9):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==10):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,B; pixels[5]=0,0,B; pixels[6]=0,0,B; pixels[7]=0,0,B
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==11):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==12):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,B; pixels[5]=0,0,B; pixels[6]=0,0,B; pixels[7]=0,0,B
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==13):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==14):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,B; pixels[5]=0,0,B; pixels[6]=0,0,B; pixels[7]=0,0,B
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==15):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.

        pixels.show()

    frameCount += 1
    if frameCount > 5: frameCount = 0
#---------------------------------------------------------------------------------------------
def policeCar3():
    global frameCount, neoIndex

    if NEO_SUDO and frameCount == 0:

        R = 9; B = 9    # 자율주행시 카메라에 영향을 주지 않도록 색상 값을 낮게 하여야 합니다.
        if neoIndex>17: neoIndex = 0
        if (neoIndex==0):
            pixels[0]=R,0,0; pixels[1]=R,0,0; pixels[2]=R,0,0; pixels[3]=R,0,0  
            pixels[4]=R,0,0; pixels[5]=R,0,0; pixels[6]=R,0,0; pixels[7]=R,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==1):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif (neoIndex==2):
            pixels[0]=R,0,0; pixels[1]=R,0,0; pixels[2]=R,0,0; pixels[3]=R,0,0  
            pixels[4]=R,0,0; pixels[5]=R,0,0; pixels[6]=R,0,0; pixels[7]=R,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==3):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif (neoIndex==4):
            pixels[0]=R,0,0; pixels[1]=R,0,0; pixels[2]=R,0,0; pixels[3]=R,0,0  
            pixels[4]=R,0,0; pixels[5]=R,0,0; pixels[6]=R,0,0; pixels[7]=R,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==5):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif (neoIndex==6):
            pixels[0]=R,0,0; pixels[1]=R,0,0; pixels[2]=R,0,0; pixels[3]=R,0,0  
            pixels[4]=R,0,0; pixels[5]=R,0,0; pixels[6]=R,0,0; pixels[7]=R,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==7):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==8):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==9):
            pixels[0]=0,0,B; pixels[1]=0,0,B; pixels[2]=0,0,B; pixels[3]=0,0,B  
            pixels[4]=0,0,B; pixels[5]=0,0,B; pixels[6]=0,0,B; pixels[7]=0,0,B
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==10):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==11):
            pixels[0]=0,0,B; pixels[1]=0,0,B; pixels[2]=0,0,B; pixels[3]=0,0,B  
            pixels[4]=0,0,B; pixels[5]=0,0,B; pixels[6]=0,0,B; pixels[7]=0,0,B
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==12):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==13):
            pixels[0]=0,0,B; pixels[1]=0,0,B; pixels[2]=0,0,B; pixels[3]=0,0,B  
            pixels[4]=0,0,B; pixels[5]=0,0,B; pixels[6]=0,0,B; pixels[7]=0,0,B
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==14):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==15):
            pixels[0]=0,0,B; pixels[1]=0,0,B; pixels[2]=0,0,B; pixels[3]=0,0,B  
            pixels[4]=0,0,B; pixels[5]=0,0,B; pixels[6]=0,0,B; pixels[7]=0,0,B
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==16):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==17):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.

        pixels.show()

    frameCount += 1
    if frameCount > 5: frameCount = 0
#---------------------------------------------------------------------------------------------
def fireTruck():
    global frameCount, neoIndex

    if NEO_SUDO and frameCount == 0:

        R = 9; B = 9    # 자율주행시 카메라에 영향을 주지 않도록 색상 값을 낮게 하여야 합니다.
        if neoIndex>8: neoIndex = 0
        if (neoIndex==0):
            pixels[0]=R,0,0; pixels[1]=R,0,0; pixels[2]=R,0,0; pixels[3]=R,0,0  
            pixels[4]=R,0,0; pixels[5]=R,0,0; pixels[6]=R,0,0; pixels[7]=R,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==1):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==2):
            pixels[0]=R,0,0; pixels[1]=R,0,0; pixels[2]=R,0,0; pixels[3]=R,0,0  
            pixels[4]=R,0,0; pixels[5]=R,0,0; pixels[6]=R,0,0; pixels[7]=R,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==3):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==4):
            pixels[0]=R,0,0; pixels[1]=R,0,0; pixels[2]=R,0,0; pixels[3]=R,0,0  
            pixels[4]=R,0,0; pixels[5]=R,0,0; pixels[6]=R,0,0; pixels[7]=R,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==5):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==6):
            pixels[0]=R,0,0; pixels[1]=R,0,0; pixels[2]=R,0,0; pixels[3]=R,0,0  
            pixels[4]=R,0,0; pixels[5]=R,0,0; pixels[6]=R,0,0; pixels[7]=R,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==7):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==8):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.

        pixels.show()

    frameCount += 1
    if frameCount > 5: frameCount = 0
#---------------------------------------------------------------------------------------------
def ambulance():
    global frameCount, neoIndex

    if NEO_SUDO and frameCount == 0:

        G = 9       # 자율주행시 카메라에 영향을 주지 않도록 색상 값을 낮게 하여야 합니다.
        if neoIndex>8: neoIndex = 0
        if (neoIndex==0):
            pixels[0]=0,G,0; pixels[1]=0,G,0; pixels[2]=0,G,0; pixels[3]=0,G,0  
            pixels[4]=0,G,0; pixels[5]=0,G,0; pixels[6]=0,G,0; pixels[7]=0,G,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==1):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==2):
            pixels[0]=0,G,0; pixels[1]=0,G,0; pixels[2]=0,G,0; pixels[3]=0,G,0  
            pixels[4]=0,G,0; pixels[5]=0,G,0; pixels[6]=0,G,0; pixels[7]=0,G,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==3):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==4):
            pixels[0]=0,G,0; pixels[1]=0,G,0; pixels[2]=0,G,0; pixels[3]=0,G,0  
            pixels[4]=0,G,0; pixels[5]=0,G,0; pixels[6]=0,G,0; pixels[7]=0,G,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==5):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==6):
            pixels[0]=0,G,0; pixels[1]=0,G,0; pixels[2]=0,G,0; pixels[3]=0,G,0  
            pixels[4]=0,G,0; pixels[5]=0,G,0; pixels[6]=0,G,0; pixels[7]=0,G,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==7):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==8):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0  
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.

        pixels.show()

    frameCount += 1
    if frameCount > 5: frameCount = 0

#---------------------------------------------------------------------------------------------
def patrol():
    global frameCount, neoIndex

    if NEO_SUDO and frameCount == 0:

        G = 9; R = 9      # 자율주행시 카메라에 영향을 주지 않도록 색상 값을 낮게 하여야 합니다.
        if neoIndex>10: neoIndex = 0
        if  (neoIndex==0):
            pixels[0]=R,G,0; pixels[1]=R,G,0; pixels[2]=R,G,0; pixels[3]=0,0,0
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==1):
            pixels[0]=0,0,0; pixels[1]=R,G,0; pixels[2]=R,G,0; pixels[3]=R,G,0
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==2):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=R,G,0; pixels[3]=R,G,0
            pixels[4]=R,G,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==3):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=R,G,0
            pixels[4]=R,G,0; pixels[5]=R,G,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==4):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0
            pixels[4]=R,G,0; pixels[5]=R,G,0; pixels[6]=R,G,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==5):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0
            pixels[4]=0,0,0; pixels[5]=R,G,0; pixels[6]=R,G,0; pixels[7]=R,G,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==6):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=0,0,0
            pixels[4]=R,G,0; pixels[5]=R,G,0; pixels[6]=R,G,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==7):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=0,0,0; pixels[3]=R,G,0
            pixels[4]=R,G,0; pixels[5]=R,G,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==8):
            pixels[0]=0,0,0; pixels[1]=0,0,0; pixels[2]=R,G,0; pixels[3]=R,G,0
            pixels[4]=R,G,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==9):
            pixels[0]=0,0,0; pixels[1]=R,G,0; pixels[2]=R,G,0; pixels[3]=R,G,0
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.
        elif(neoIndex==10):
            pixels[0]=R,G,0; pixels[1]=R,G,0; pixels[2]=R,G,0; pixels[3]=0,0,0
            pixels[4]=0,0,0; pixels[5]=0,0,0; pixels[6]=0,0,0; pixels[7]=0,0,0
            neoIndex += 1                 # 다음 번호를 지정한다.

        pixels.show()

    frameCount += 1
    if frameCount > 5: frameCount = 0
#---------------------------------------------------------------------------------------------
if __name__ == '__main__':

    import select
 
    print('실행 정지 하려면 "q + [Enter]" 입력 하세요.')

    while True:

        pixels.fill((255, 0, 0))
        pixels.show(); time.sleep(1)

        pixels.fill((0, 255, 0))
        pixels.show(); time.sleep(1)

        pixels.fill((0, 0, 255))
        pixels.show(); time.sleep(1)
    
        pixels[0] = 1, 1, 1
        pixels[1] = 0, 0, 255
        pixels[2] = 0, 255, 0
        pixels[3] = 0, 255, 255
        pixels[4] = 255, 0, 0
        pixels[5] = 255, 0, 255
        pixels[6] = 255, 255, 0
        pixels[7] = 255, 255, 255
    
        pixels.show(); time.sleep(1)

        # 표준 입력 장치로 부터 Key 가 눌려졌는지 확인하고 있으면 Key 값 반환한다. 
        if sys.stdin in select.select([sys.stdin],[],[],0)[0]:
            key = sys.stdin.readline().strip()
            if key == 'q':
                break

#=============================================================================================
