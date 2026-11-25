# Raspberry Pi 기반 자율주행 RC카 및 재난 대응 시스템

## 1. Project Overview

Raspberry Pi를 기반으로 주변 환경을 인식하고 자율적으로 주행하는 RC카를 개발하는 프로젝트입니다.

카메라와 다양한 센서를 활용하여 주변 환경 데이터를 수집하고, 이를 바탕으로 주행 및 재난 상황 대응 기능을 구현하는 것을 목표로 합니다.

특히 딥러닝 기반 환경 인식 기술을 적용하여 재난 상황을 감지하고, 상황에 따라 차량의 주행을 제어하는 시스템을 구상하고 있습니다.

## 2. Project Objectives

- 카메라와 센서를 활용한 주변 환경 데이터 수집
- 센서 데이터를 활용한 환경 정보 파악
- 모터 및 조향 제어를 통한 자율주행 구현
- 딥러닝 기반 재난 상황 인식 기능 개발
- 재난 상황에 따른 차량의 자율적인 주행 제어

## 3. Hardware

| Component | Description |
|---|---|
| Raspberry Pi 4 Model B | 메인 제어 장치 |
| CSI Camera (OV5647) | 영상 데이터 수집 |
| MPU6050 | 가속도 및 자이로 센서 |
| VL53L0X | 거리 측정 센서 |
| GY-906 | 적외선 온도 센서 |
| MAX30105 | 광학 센서 |
| Servo Motors | 차량 구동 및 조향 제어 |

## 4. Technologies

- Python
- Raspberry Pi OS (Linux)
- I2C Communication
- Computer Vision
- Deep Learning (Development Goal)

## 5. Implementation

### Implemented
- Raspberry Pi 기반 차량 하드웨어 구성
- 카메라 연결 및 영상 촬영 테스트
- 모터 구동 및 조향 제어 코드 작성
- 센서 연결 및 데이터 수집 실험

### In Progress
- 센서 데이터 수집 및 통합
- 딥러닝 기반 재난 상황 인식 기능
- 인식 결과에 따른 자율주행 제어

## 6. Future Development

1. 센서 데이터 수집 및 전처리 체계 구축
2. 딥러닝 기반 재난 상황 인식 모델 학습
3. 환경 인식 결과와 차량 제어 시스템 연동
4. 재난 상황 발생 시 차량 정지 및 주변 환경 기록 기능 구현
5. 재난 상황 종료 판단에 따른 자율주행 재개 기능 개발

## 7. Project Status

현재 하드웨어 구성과 모터·조향 제어를 중심으로 개발을 진행하고 있으며, 센서 데이터 수집과 딥러닝 기반 환경 인식 기능을 단계적으로 구현할 예정입니다.

---

**Project:** Raspberry Pi Autonomous RC Car for Disaster Detection and Response

**Platform:** Raspberry Pi 4 Model B

**Language:** Python
