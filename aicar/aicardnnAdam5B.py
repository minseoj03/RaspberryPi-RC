# Deep Neural Network with Adam B                          <aicardnnAdam5B.py> 

epochs = 5000                     # 학습회수
earlyStopCount = 500              # 조기종료
learning_rate = 0.001                                                            # 

import os                         # File 일기 쓰기와 관련된 라이브러리
import sys                        # 커맨드 라인 처리를 위한 라이브러리
import pickle
import numpy as np
import cv2
import matplotlib.pyplot as plt

np.random.seed(12)  # Set a random seed for reproducibility

# 학습 데이터 생성 -----------------------------------------------------------
def batchGen(imageList, angleList, batchSize):

  imageBatch = []
  angleBatch = np.zeros((batchSize, 50), dtype=float)                           # Specify dtype=float for 0.0

  for i in range(batchSize):

    t = f'{project}/'+ imageList[i]
    image = cv2.imread(t)

    y_layer = image[:, :, 0]                                                    # Y 레이어만 추출한다.
    resized_y = cv2.resize(y_layer, (25, 8))                                    # 이미지를 8 x 25 Pixle 로 축소, width, height 순서로 입력

    # Ensure resized_y has the correct data type for interpolation
    resized_y = resized_y.astype(np.uint8)

    normalized_y = resized_y / 255.0                                            # 정규화
    image = normalized_y.flatten()                                              # 1 차원으로 직열화 한다.
    imageBatch.append(image)

    angle = angleList[i] #; print('angle=', angle )

    if angle < -90:
      a = -90
    elif angle > 89:
      a = 89
    else:
      a = angle
    b = a + 90
    #c = int(b/7.2)                     # 25
    c = int(b/3.6)                     # 50

    angleBatch[i][c] = 1.0

  #print(*angleBatch)
  return (np.asarray(imageBatch), np.asarray(angleBatch))

def sigmoid(x):
  return 1 / (1 + np.exp(-x))

def sigmoid_derivative(x):
  return sigmoid(x) * (1 - sigmoid(x))


class NeuralNetwork:
  def __init__(self, input_size, hidden_size1, hidden_size2, hidden_size3, output_size):
    self.input_size = input_size
    self.hidden_size1 = hidden_size1
    self.hidden_size2 = hidden_size2
    self.hidden_size3 = hidden_size3
    self.output_size = output_size

    # 가중치 및 편향 초기화 (He 초기화 사용)
    self.W1 = np.random.randn(self.input_size, self.hidden_size1) * np.sqrt(2 / self.input_size)
    self.B1 = np.zeros((1, self.hidden_size1))
    self.W2 = np.random.randn(self.hidden_size1, self.hidden_size2) * np.sqrt(2 / self.hidden_size1)
    self.B2 = np.zeros((1, self.hidden_size2))
    self.W3 = np.random.randn(self.hidden_size2, self.hidden_size3) * np.sqrt(2 / self.hidden_size2)
    self.B3 = np.zeros((1, self.hidden_size3))
    self.W4 = np.random.randn(self.hidden_size3, self.output_size) * np.sqrt(2 / self.hidden_size3)
    self.B4 = np.zeros((1, self.output_size))

    self.sW1 = np.zeros((self.input_size, self.hidden_size1))
    self.sB1 = np.zeros((1, self.hidden_size1))
    self.sW2 = np.zeros((self.hidden_size1, self.hidden_size2))
    self.sB2 = np.zeros((1, self.hidden_size2))
    self.sW3 = np.zeros((self.hidden_size2, self.hidden_size3))
    self.sB3 = np.zeros((1, self.hidden_size3))
    self.sW4 = np.zeros((self.hidden_size3, self.output_size))
    self.sB4 = np.zeros((1, self.output_size))

    # Adam optimizer parameters
    self.beta1 = 0.9
    self.beta2 = 0.999
    self.epsilon = 1e-8
    self.t = 0

    self.mW1 = np.zeros_like(self.W1)
    self.vW1 = np.zeros_like(self.W1)
    self.mB1 = np.zeros_like(self.B1)
    self.vB1 = np.zeros_like(self.B1)

    self.mW2 = np.zeros_like(self.W2)
    self.vW2 = np.zeros_like(self.W2)
    self.mB2 = np.zeros_like(self.B2)
    self.vB2 = np.zeros_like(self.B2)

    self.mW3 = np.zeros_like(self.W3)
    self.vW3 = np.zeros_like(self.W3)
    self.mB3 = np.zeros_like(self.B3)
    self.vB3 = np.zeros_like(self.B3)

    self.mW4 = np.zeros_like(self.W4)
    self.vW4 = np.zeros_like(self.W4)
    self.mB4 = np.zeros_like(self.B4)
    self.vB4 = np.zeros_like(self.B4)

  def forward(self, X, Y=None):
    # 순전파 계산
    self.Z1 = np.dot(X, self.W1) + self.B1
    self.A1 = sigmoid(self.Z1)                                                  #
    self.Z2 = np.dot(self.A1, self.W2) + self.B2
    self.A2 = sigmoid(self.Z2)                                                  #
    self.Z3 = np.dot(self.A2, self.W3) + self.B3
    self.A3 = sigmoid(self.Z3)                                                  #
    self.Z4 = np.dot(self.A3, self.W4) + self.B4
    self.Y_hat = sigmoid(self.Z4)                                               #

    # Calculate error if Y is provided
    if Y is not None:
      error = np.mean((self.Y_hat - Y)**2)                                      # Mean Squared Error
    else:
      error = None

    return self.Y_hat, error                                                    # Return both Y_hat and error

  def backward(self, X, Y, learning_rate):
    # 역전파 계산
    # M x N 행열과 N x P 행열의 곱셈은 M x P 행열이 되며 앞의 행열의 열의(N) 개수와 뒤 행열의 행(N)의 개수가 같아야 한다.
    self.t += 1                                                               # Increment time step

    D4 = (self.Y_hat - Y) * sigmoid_derivative(self.Z4)                         # D4 shape (1, 25), Y_hat shape (1, 25), Z4 shape (1, 25)
    dW4 = np.dot(self.A3.T, D4)                                                 # A3 shape (1, 50), dW4 shape (50, 25) = A3.T(50, 1) dot D4(1, 25)
    dB4 = np.sum(D4, axis=0, keepdims=True)                                     # dB4 shape (1, 25)

    D3 = np.dot(D4, self.W4.T) * sigmoid_derivative(self.Z3)                     # D3 shape (1, 50), D4 shape (1, 25), W4.T shape (25, 50), Z3 shape (1, 50)
    dW3 = np.dot(self.A2.T, D3)                                                 # A2 shape (1, 100), dW3 shape (100, 50) = A2.T(100, 1) dot D3(1, 50)
    dB3 = np.sum(D3, axis=0, keepdims=True)                                     # dB3 shape (1, 50)

    D2 = np.dot(D3, self.W3.T) * sigmoid_derivative(self.Z2)                     # D2 shape (1, 100), D3 shape (1, 50), W3.T shape (50, 100), Z2 shape (1, 100)
    dW2 = np.dot(self.A1.T, D2)                                                 # A1 shape (1, 200), dW2 shape (200, 100) = A1.T(200, 1) dot D2(1, 100)
    dB2 = np.sum(D2, axis=0, keepdims=True)                                     # dB2 shape (1, 100)

    D1 = np.dot(D2, self.W2.T) * sigmoid_derivative(self.Z1)                     # D1 shape (1, 200), D2 shape (1, 100), W2.T shape (100, 200), Z1 shape (1, 200)
    dW1 = np.dot(X.T, D1)                                                       # X shape (1, 200), dW1 shape (200, 200) = X.T(200, 1) dot D1(1, 200)
    dB1 = np.sum(D1, axis=0, keepdims=True)                                     # dB1 shape (1, 200)

    # Adam update
    # W1
    self.mW1 = self.beta1 * self.mW1 + (1 - self.beta1) * dW1
    self.vW1 = self.beta2 * self.vW1 + (1 - self.beta2) * (dW1**2)
    mW1_hat = self.mW1 / (1 - self.beta1**self.t)
    vW1_hat = self.vW1 / (1 - self.beta2**self.t)
    self.W1 -= learning_rate * mW1_hat / (np.sqrt(vW1_hat) + self.epsilon)

    self.mB1 = self.beta1 * self.mB1 + (1 - self.beta1) * dB1
    self.vB1 = self.beta2 * self.vB1 + (1 - self.beta2) * (dB1**2)
    mB1_hat = self.mB1 / (1 - self.beta1**self.t)
    vB1_hat = self.vB1 / (1 - self.beta2**self.t)
    self.B1 -= learning_rate * mB1_hat / (np.sqrt(vB1_hat) + self.epsilon)

    # W2
    self.mW2 = self.beta1 * self.mW2 + (1 - self.beta1) * dW2
    self.vW2 = self.beta2 * self.vW2 + (1 - self.beta2) * (dW2**2)
    mW2_hat = self.mW2 / (1 - self.beta1**self.t)
    vW2_hat = self.vW2 / (1 - self.beta2**self.t)
    self.W2 -= learning_rate * mW2_hat / (np.sqrt(vW2_hat) + self.epsilon)

    self.mB2 = self.beta1 * self.mB2 + (1 - self.beta1) * dB2
    self.vB2 = self.beta2 * self.vB2 + (1 - self.beta2) * (dB2**2)
    mB2_hat = self.mB2 / (1 - self.beta1**self.t)
    vB2_hat = self.vB2 / (1 - self.beta2**self.t)
    self.B2 -= learning_rate * mB2_hat / (np.sqrt(vB2_hat) + self.epsilon)

    # W3
    self.mW3 = self.beta1 * self.mW3 + (1 - self.beta1) * dW3
    self.vW3 = self.beta2 * self.vW3 + (1 - self.beta2) * (dW3**2)
    mW3_hat = self.mW3 / (1 - self.beta1**self.t)
    vW3_hat = self.vW3 / (1 - self.beta2**self.t)
    self.W3 -= learning_rate * mW3_hat / (np.sqrt(vW3_hat) + self.epsilon)

    self.mB3 = self.beta1 * self.mB3 + (1 - self.beta1) * dB3
    self.vB3 = self.beta2 * self.vB3 + (1 - self.beta2) * (dB3**2)
    mB3_hat = self.mB3 / (1 - self.beta1**self.t)
    vB3_hat = self.vB3 / (1 - self.beta2**self.t)
    self.B3 -= learning_rate * mB3_hat / (np.sqrt(vB3_hat) + self.epsilon)

    # W4
    self.mW4 = self.beta1 * self.mW4 + (1 - self.beta1) * dW4
    self.vW4 = self.beta2 * self.vW4 + (1 - self.beta2) * (dW4**2)
    mW4_hat = self.mW4 / (1 - self.beta1**self.t)
    vW4_hat = self.vW4 / (1 - self.beta2**self.t)
    self.W4 -= learning_rate * mW4_hat / (np.sqrt(vW4_hat) + self.epsilon)

    self.mB4 = self.beta1 * self.mB4 + (1 - self.beta1) * dB4
    self.vB4 = self.beta2 * self.vB4 + (1 - self.beta2) * (dB4**2)
    mB4_hat = self.mB4 / (1 - self.beta1**self.t)
    vB4_hat = self.vB4 / (1 - self.beta2**self.t)
    self.B4 -= learning_rate * mB4_hat / (np.sqrt(vB4_hat) + self.epsilon)

  def train(self, Xt, Yt, Xv, Yv, epochs, learning_rate):
    Terrors = []
    Verrors = []
    earlyStop = 0
    final_epoch = 0
    previousTError = float('inf')
    previousVError = float('inf')

    for i in range(epochs):
      Y_hat, tError = self.forward(Xt, Yt)                                      # Pass both X and y to forward
      Terrors.append(tError)
      self.backward(Xt, Yt, learning_rate)                                      # 가중치 W, 바이어스 b 업데이트

      _, vError = self.forward(Xv, Yv)                                          # 검증용 데이터 에러 계산
      Verrors.append(vError)
      print(f"Epoch {i+1}/{epochs}  Training Loss: {tError:.8f}  Validation Loss: {vError:.8f}")
      if vError < previousVError:
        final_epoch = i
        earlyStop = 0
        previousVError = vError
        previousTError = tError
        # 학습 결과가 개선 되었으므로 W, b 값 보관
        self.sW1 = self.W1; self.sB1 = self.B1
        self.sW2 = self.W2; self.sB2 = self.B2
        self.sW3 = self.W3; self.sB3 = self.B3
        self.sW4 = self.W4; self.sB4 = self.B4
      else:
        earlyStop += 1
        print(f"----- Early stopping {earlyStop}/{earlyStopCount}")
        if earlyStop >= earlyStopCount:
          print("Early stopping triggered")
          break

    # 보관된 학습 결과 복원
    self.W1 = self.sW1; self.B1 = self.sB1
    self.W2 = self.sW2; self.B2 = self.sB2
    self.W3 = self.sW3; self.B3 = self.sB3
    self.W4 = self.sW4; self.B4 = self.sB4

    return Terrors, Verrors, previousTError, previousVError, final_epoch

  def saveWeightBias(self, fileName):
    with open(fileName, 'wb') as f:
      pickle.dump(self.W1, f)
      pickle.dump(self.B1, f)
      pickle.dump(self.W2, f)
      pickle.dump(self.B2, f)
      pickle.dump(self.W3, f)
      pickle.dump(self.B3, f)
      pickle.dump(self.W4, f)
      pickle.dump(self.B4, f)

#==========================================================================================

if len(sys.argv) >= 2:
  project = sys.argv[1]              # Video 저장 디렉토리
  #---------------------------------------------------------------------------------------
  dataDir = './'+project

  if  os.path.exists(dataDir):            # 커맨드 라인에 주어진 디렉터리가 있는지 검사
    print('프로젝트:',project)

    # 이미지 데이터 조향각 추출
    with open(f'{project}/_{project}.pickle', 'rb') as f:                           # pickle 파일
      x_test_Image = pickle.load(f)                                               # 시험(Test) 이미지 파일 이름 리스트
      x_valid_Image = pickle.load(f)                                              # 검증(Valid) 이미지 파일 이름 리스트
      x_train_Image = pickle.load(f)                                              # 훈련(Train) 이미지 파일 이름 리스트
    #-------------------------------------------------------------------------------
    y_test_Angle=[]
    y_valid_Angle=[]
    y_train_Angle = []
    for f in x_test_Image:                                                          # 시험용 이미지 파일 이름에서 각도값
        y_test_Angle.append(int(f[-7:-4]))
    for f in x_valid_Image:                                                         # 검증용 이미지 파일 이름에서 각도값
        y_valid_Angle.append(int(f[-7:-4]))
    for f in x_train_Image:                                                         # 훈련용 이미지 파일 이름에서 각도값
        y_train_Angle.append(int(f[-7:-4]))

    print('시험 이미지 개수:',len(x_test_Image))
    print('검증 이미지 개수:',len(x_valid_Image))
    print('훈련 이미지 개수:',len(x_train_Image))

    # 신경망 객체 생성 ---------------------------------------------------------------------------
    nn = NeuralNetwork(200, 200, 150, 100, 50)

    Xt, Yt = batchGen(x_train_Image, y_train_Angle, batchSize=len(x_train_Image))   # Train 데이터 준비
    Xv, Yv = batchGen(x_valid_Image, y_valid_Angle, batchSize=len(x_valid_Image))   # Valid 데이터 준비

    print('Xt', Xt.shape); print('Yt', Yt.shape); print('Xv', Xv.shape); print('Yv', Yv.shape)

    Terrors, Verrors, terror, verror, final_epoch = nn.train(Xt, Yt, Xv, Yv, epochs, learning_rate)              # 학습

    fn = f'{project}/_{project}_model.dnn'                                                    # 가중치 및 편향 파일 이름
    nn.saveWeightBias(fn)                                                           # 가중치 및 편향 저장

    plt.plot(Terrors[10:], label = f'Training Loss: {terror:.5f}', color = 'b')
    plt.plot(Verrors[10:], label = f'Validation Loss: {verror:.5f}', color = 'r')
    plt.legend()
    plt.xlabel(f"Training and Validation Loss")
    plt.ylabel("Error (MSE)")
    plt.title(f"{project}     Epochs: {final_epoch}     Learn rate: {learning_rate:}")
    plt.show()
    #------------------------------------------------------------------------------------------
    X, Y = batchGen(x_test_Image, y_test_Angle, batchSize=len(x_test_Image))
    predictions, _ = nn.forward(X)
    rounded_predictions = np.round(predictions, 2)                                  # Round to 2 decimal places
    print(rounded_predictions)
    max_positions = np.argmax(predictions, axis=1)                                  # Find positions of maximum values along each row
    print("최대값 위치:", max_positions)
    predAngle = []
    maxValues = np.max(predictions, axis=1)
    print("최대값:", maxValues)
    for i in max_positions:
      #predAngle.append(int(i*7.2+3.6-90))                    # 25  # int 로 변경(numpy -> python)후 리스트에 추가
      predAngle.append(int(i*3.6+1.8-90))                        # 50  # int 로 변경(numpy -> python)후 리스트에 추가
    print("예측 조향각:",predAngle)

    # 테스트 이미지 라벨 조향각/예측 조향각 ------------------------------------
    fig, axes = plt.subplots(2, len(x_test_Image))#, figsize=(30, 2+3))

    for x, c in enumerate(x_test_Image):
      yuv_image = cv2.imread(f'{project}/{c}', cv2.IMREAD_UNCHANGED)                # 테스트 이미지 읽기
      bgr_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR)                        # YUV 이미지를 BGR 형식으로 변환
      rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)                        # MatPlot Lib 에서 표시되는 RGB 형식으로 변환
      axes[0,x].imshow(rgb_image)

      y_layer = yuv_image[:, :, 0]                                                  # Y 레이어만 추출
      resized_y = cv2.resize(y_layer, (25, 8))                                      # 이미지를 8 x 25 Pixle 로 축소, width, height 순서로 입력

      # Ensure resized_y has the correct data type for interpolation
      resized_y = resized_y.astype(np.uint8)

      axes[1,x].imshow(resized_y)

      t = x_test_Image[x][-7:-4] + '/' + str(predAngle[x])
      n = x_test_Image[x][-14:-9]

      axes[0,x].set_title(n)#, fontsize=50)
      axes[0,x].xaxis.set_ticks([])
      axes[0,x].yaxis.set_ticks([])
      axes[1,x].set_title(t)#, fontsize=50)
      axes[1,x].xaxis.set_ticks([])
      axes[1,x].yaxis.set_ticks([])
    plt.show()

  else:
    print(f'Project 디렉토리 "{project}" 가 없습니다.')
else:
  print('Project 디렉토리 이름을 지정하세요.')

#===============================================================================

