---
title: "Paper Review"
date: 2024-07-29 12:00:00 +0900
categories: [Reviews, Robotics]
tags: [control, simulation, robotics, dynamics]
math: true
toc: true
author: dareumHJ
img_path: /assets/img/post_img
pin: false
comments: true
description: Review on a multilayer control for multirotor UAVs equipped with a servo robot arm
---

## **A multilayer control for multirotor UAVs equipped with a servo robot arm**

**Authors**: F. Ruggiero, M.A. Trujillo, R. Cano, H. Ascorbe, A. Viguria, C. Per´ez, V. Lippiello, A. Ollero, and B. Siciliano

### **1. Abstract**
Moving manipulator의 존재를 고려하여 aerial platform을 제어하는 것이 주요 목적
세 계층...
첫 번째 → robot arm의 정적 균형을 맞추기 위한 움직이틑 배터리가 고안됨..
두 번째 → 첫 번째 계층의 mechanical limitation을 극복하기 위해 UAV에 미치는 arm static effects의 잔여량을 계산하고, 제어 추력과 토크를 이용해 보상.
세 번째 → UAV에 작용하는 외력 및 모먼트를 추정하고 이 추정값을 컨트롤러에 피드백하여 무시된 공기역학적 효과와 팔의 동역학을 보상.
이렇게 제안된 architecture의 성능 또한 실험적으로 평가되었다.

> UAV = Unmanned Aerial Vehicle
> UAV = AV = multirotor / robotic arm = arm = manipulator

### **2. Introduction**
AV와 manipulator 사이의 coupling effects 때문에 다양한 설계 및 제어 문제가 발생한다. 일반적으로 aerial manipulator를 제어하기 위해 두 가지 접근 방식이 고려되는데,
첫 번째는 multirotor와 robotic arm을 하나의 unique entity로 보고, 완전한 동역학 모델을 기반으로 제어기를 디자인 하는 것.
또 하나는 UAV와 arm을 두 개의 별개의 독립된 시스템으로 고려하는 것이다.

multirotor에 가해지는 robotic arm의 영향은 외란(external disturbances)으로 볼 수 있고, 반대도 똑같다.

-> 이러한 접근은 UAV의 위치 에러를 보상할 만큼 arm의 반응성이 충분하지 않거나, arm이 torque control을 쓰지 않을 때 유용할 수 있다. arm이 서보라 position control을 쓴다는 거겠죠?

" 본 논문에서는, multirotor의 CoG에 위치한 robotic arm의 움직임에 의한 영향을 줄이는 데 집중한 multilayer 제어 시스템을 제안한다! (complete system이 아니라 별개의 시스템으로 보고, 주고받는 영향을 분석)"

" 또한, moving battery라는 CoG의 움직임을 보상하는 새로운 방법을 제시하고,"

" UAV에 작용하는 일반 외력의 추정기에서, 각속도의 측정을 rotation의 표현에 덜 의존적이도록 하였다. "

이 multilayer 제어 시스템이란 게 abstract에서 언급한 세 개의 계층이고 여기에 항상 활성화되어 있는 multirotor에 대한 PID 제어기가 기본 layer로 들어간다.

### **Modelling**

world-fixed inertial frame:  $$ \{O, X, Y, Z\} $$ <br/>
body-fixed frame: $$ \{O_b, X_b, Y_b, Z_b\}$$ <br/>
inertial frame 기준 multirotor position: $$ \mathbf{p}_b \in \R^3 $$ <br/>
RPY Euler angle: $$ \mathbf{\eta}_b \in \R^3 $$ <br/>
inertial frame 기준 body frame의 attitude: $$ \mathbf{R}_b(\mathbf{\eta}_b) \in SO(3)$$ <br/>
<br/>

이렇게 두면, multirotor의 동역학 모델은 다음과 같다.
$$ m\ddot{\mathbf{p}}_b + m\mathbf{g}_b = \mathbf{R}_b(\mathbf{\eta}_b)(\mathbf{f}_b^b + \mathbf{f}_v^b), $$

$$\mathbf{I}_b\dot{\mathbf{\omega}}_b^b + \mathbf{S}(\mathbf{\omega}_b^b)\mathbf{I}_b\mathbf{\omega}_b^b = \mathbf{\tau}_b^b + \mathbf{\tau}_v^b$$

여기서 $$S(\cdot)$$은 skew matrix, $$\mathbf{f}_b^b, \mathbf{\tau}_b^b$$는 force 및 torque input vector이고, $$\mathbf{f}_v^b, \mathbf{\tau}_v^b$$는 외력 및 외부 토크를 나타낸다.

multirotor의 특정 configuration에 따라 $$\mathbf{f}_b^b, \mathbf{\tau}_b^b $$의 표현이 달라지지만, 이번 paper에서는 quadrotor를 가정하여, 다음과 같이 표현한다.
여기서 u는 프로펠러 평면에 수직하는 total thrust를 의미한다.

$$\mathbf{f}_b^b = [0 \,\, 0 \,\, u]^T$$

$$\mathbf{\tau}_b^b = [\tau_{\phi} \,\, \tau_{\theta} \,\, \tau_{\psi}]^T$$

### **4. Multilayer Control System**


Introduction에서 설명된 계층들에 의한 multirotor의 제어 방식은 다음과 같다.

$$u = f_u(\mathbf{g}_b, \mathbf{f}_0, \mathbf{f}_2, \mathbf{f}_3), $$

$$\mathbf{\tau}_b^b = \mathbf{f_{\tau}}(\mathbf{\tau}_0, \mathbf{\tau}_2, \mathbf{\tau}_3), $$

이때, $$\mathbf{f}_0$$와 $$\mathbf{\tau}_0$$는 각각 quadrotor의 position과 attitude에 대한 기본적인 PID 제어이다.

또한, $$\mathbf{f}_i$$와 $$\mathbf{\tau}_i$$는 각각 i번째 레이어에 의한 힘과 토크 제어이다.

각 함수 $$f_u$$와 $$\mathbf{f}_{\tau}$$에 대해, f_u는 rotation의 표현에 따라 그 표현이 달라질 수 있겠지만은, f_tau는 단순 선형 결합으로 생각할 수 있을 것이다....

#### L1: Battery Movement Compensation

첫 번째 layer에서는 앞서 설명한 것처럼 arm의 움직임에 의한 CoG 변위를 보상해주기 위한 시스템을 구현한다. 이를 Displacement Compensation System, 이하 DCS라고 하자.

이 DCS는 linear slider 위를 움직이는 counterweight 장치로 구성되어 있는데, 이 장치는 (UAV + 매니퓰레이터 + load, counterweight)으로 구성된 전체 시스템의 CoG가 multirotor의 기하학적 중심에 최대한 가깝게 위치하도록 제어된다.

(이 기하학적 중심이라는 건 robotic arm이 compact configuration에 있을 대의 CoG를 의미한다.)

여기서 쓰이는 counterweight은 앞서 설명한 대로 on-board 배터리이고, 작은 움직임으로도 CoG 변위를 보상할 수 있을 만큼 무겁다.

매니퓰레이터의 각 Link i에 대해, 어떤 순간의 CoG 위치를 다음과 같이 두자. 이때, 기준 좌표계는 base에 고정된 arm의 좌표계$$\{O_0,\, X_0,\, Y_0,\, Z_0\}$$이다. (A는 CoG를 의미하는 듯)

$$ [x_{Ai}^0 \,\, y_{Ai}^0 \,\, z_{Ai}^0 \,\, 1]^T = \mathbf{T}_i^0 [x_{Ai}^i \,\, y_{Ai}^i \,\, z_{Ai}^i \,\, 1]^T$$

i = 1, ..., 7이고, i=7에 해당하는 link는 end effector에 쥐어진 load이다. $$\mathbf{T}_i^0 \in \R^{4 \times 4}$$ 는 servo feedback에 의해 update된 i번째 link에 해당하는 homogeneous transformation matrix(이하 HTM)이다.

또한, robotic arm의 CoG 위치 벡터인 $$\mathbf{p}_A^b \in \R^3$$는 다음과 같다.

$$ p_A^b = \frac{1}{m_A}\mathbf{E}_3\mathbf{T}_0^b\left(\sum_{i=1}^7{m_i[x_{Ai}^0 \; y_{Ai}^0 \; z_{Ai}^0 \; 1]^T}\right)$$

$$m_A = \sum_{i=1}^7{m_i}$$

이때, $$\mathbf{T}_0^b \in \R^{4\times4}$$는 arm → body frame으로의 constant HTM이고()


#### L2: Arm Static Compensation

#### L3: External Generalized Forces Estimation & Compensation

### **4. Experimental Validation**

~ 대충 결과 ~