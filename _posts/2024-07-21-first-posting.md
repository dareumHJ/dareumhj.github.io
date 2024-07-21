---
title: "Paper Review"
date: 2024-07-21 12:00:00 +0900
categories: [Reviews, Robotics]
tags: [control, simulation, robotics, dynamics]
math: true
toc: true
author: dareumHJ
img_path: /assets/img/post_img
pin: false
comments: true
description: Review on Optimal Ankle Compliance Regulation
---

## **Optimal Ankle Compliance Regulation for Humanoid Balancing Control**
**Authors**: Mohamad Mosadeghzad, Zhibin Li, Nikos G. Tsagarakis, Gustavo A. Medrano-Cerda, Houman Dallali, and Darwin G. Caldwell

### 1. Intro
최적의 제어 전략을 사용하여 휴머노이드를 밸런싱하기 위한 유효 발목 임피던스 프로파일을 생성하는 방법에 대하여 다룬다.

#### COP란?
COP는 Center of pressure로, 접지 surface에 작용하는 모든 힘의 **평균적인** 작용점을 의미한다.
&nbsp;
이 개념을 쓰는 이유는, 로봇의 발 또는 다른 접촉 부위가 지면에 힘을 가할 때, 이 힘의 분포가 일정하지 않고,
따라서 이 힘들의 평균적인 작용점이 로봇의 균형 또는 안정성을 평가하는 데 중요하게 작용하기 때문이다.
&nbsp;
일반적으로 COM이 COP 위에 있도록 제어하면 로봇이 안정적으로 서있을 수 있다...라고 하는데 이건 아마도 조금 보수적인 기준인 듯 하다.
&nbsp;
그러나 practical하게, 접촉면의 모든 부위에서의 가해지는 힘을 알 수는 없다. 따라서, 실제로 COP를 측정하기 위해서는 특정 몇몇 지점에서 가해지는 힘 만을 가지고 계산해야 한다. (특정 지점 힘 센서 설치 -> 모델 기반 COP 계산 및 자세 제어 -> 피드백)
&nbsp;
예를 들어보자.
<p align="center"> <img src="../assets/img/post_img/20240721-0.png" width="120px" height="150px" title="foot"/> </p>

위와 같은 세모난 발바닥에 세 개의 force sensor가 달려 있다고 생각하면,
$$ F_{1,x}+F_{2,x}+F_{3,x}=GRF_x $$
$$ F_{1,x}+F_{2,x}+F_{3,x}=GRF_x $$
$$ F_{1,x}+F_{2,x}+F_{3,x}=GRF_x $$


#### ZMP란?