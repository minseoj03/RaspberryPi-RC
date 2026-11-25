# View Image                                         <vi.py>
# 
# 수집한 이미지를 연속 디스프레이 합니다.
# 디스프레이 시작 번호는 0 부터 입니다. 
#
# 
# 
#
#
#
import os                         # File 읽기와 쓰기에 관련된 라이브러리
import sys                        # 커맨드 라인 처리를 위한 라이브러리
import time                       # 시간 정보 라이브러리
import glob                       # 파일 찾기 라이브러리
import numpy as np                # OpenCV 에서 이미지 데이터 저장 배열 변수
import cv2 as cv

title = '[S]:Speed    [C]:Clear    [Space Bar]:Pause     [X]:Exit' 
imageDir = ''
FAST = 100                        # 이미지 재생 간격 0.1초(고속) 
SLOW = 500                        # 이미지 재생 간격 0.5초(저속)
delayTime = FAST                  # 디폴트 재생 속도
seqNum = 0                        # 이미지 시작 번호
HIGH_SPEED = True                 # 이미지 재생 속도
VIEW = True                       # 커서 바, 각도 표시
KEY = ''                          # 키보드 코드 
textOffset = 10                   # 각도 표시 문자 위치 오프셋
#=============================================================================================

if len(sys.argv) >= 2:            # 대상 디렉터리를 정하여야 한다.
    imageDir = sys.argv[1]        # 이미지 디렉토리
    #----------------------------------------------------------------------
    dataDir = './'+imageDir

    if  os.path.exists(dataDir):     # 커맨드 라인에 주어진 디렉터리가 있는지 검사
        # 규정에 맞는 파일을 리스트로 만들고 AUG와 기타 파일은 무시한다.
        s = glob.glob(f'{dataDir}/*__?[0-9][0-9].png')  # *P1234__+56.png 
        u = s[0]; v =u[:-13]         # 첫번째 파일을 가져와서 번호 앞부분만 가져온다
        for i in range(len(s)):
            w = v+f'{i:04d}__?[0-9][0-9].png' 
            q = glob.glob(w)
            if len(q):               # 조건을 만족하는 파일 개수를 확인한다.
                k = q[0]             # 여러개 파일이 있을때 첫번째만 사용한다.
                print(f'{seqNum:04d}:',k)
                img = cv.imread(k, cv.IMREAD_COLOR)         # 이미지 파일 읽기
                img = cv.cvtColor(img, cv.COLOR_YUV2BGR)    # RGB 컬러 이미지로 변환
                width, height  = img.shape[:2]              # 이미지의 크기를 구한다.
                m_big = np.float32([[3, 0, 0], [0, 3, 0]])  # 확대 비율
                Limg = cv.warpAffine(img, m_big, 
                       (int(height*3), int(width*3)), None, cv.INTER_CUBIC)
                #--------------------------------------------------------------------------
                if VIEW:
                    a = k[-7:-4]; n = 300+4*int(a) 
                    cv.line(Limg,(n,0),(n,197),(0,255,255),3)
                    if n < 200: textOffset = 10
                    if n > 400: textOffset = -140
                    cv.putText(Limg,a,(n+textOffset,120),cv.FONT_HERSHEY_COMPLEX_SMALL,3,(0, 255, 0))                
                #--------------------------------------------------------------------------

                cv.imshow(title, Limg)                      # 확대한 이미지를 디스프레이
                KEY = cv.waitKey(delayTime)                 # 시간 지연하고 Key값 가져온다
                if KEY == ord('x') or KEY == ord('X'):      # 재생 정지
                    break
                elif KEY == ord(' '):                       # 일시 정지(pause)
                    cv.waitKey(0)
                elif KEY == ord('s') or KEY == ord('S'):    # 재생 속도 설정
                    HIGH_SPEED = not HIGH_SPEED             # 토글 모드로 동작
                    if HIGH_SPEED:
                        delayTime = FAST                    # 고속 재생
                    else:
                        delayTime = SLOW                    # 저속 재생
                elif KEY == ord('c') or KEY == ord('C'):    # 재생 속도 설정
                    VIEW = not VIEW
                seqNum += 1
            else:
                print('-----')                              # 존재하지 않은 파일 표시
        KEY = cv.waitKey(0)
        cv.destroyAllWindows()                              # 모든 창 닫기
    else:
        print('작업 대상 디렉터리를 지정하여야 합니다.')

