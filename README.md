# RoboticCane

This repository contains the code relative to the development of an robotic cane, to serve as an lightweight locomotion assistant for people with reduced mobility.

It is devided in two major parts, the Matlab Code and the Arduino Code.

The Matlab Code folder contains all the models created in Matlab and Simulink to simulate and analyse mathmatical models of the concept, to verify it's viability.
It contains 5 models: 
- The first model is an mathematical model of a pendulum on a cart, the first system that was studied. It is controlled by a feedback gain obtained with LQR;
- The second model is an mathematical model of a motor with rod, representing the first half of the second system that was studied. It is controlled by a feedback gain obtained with LQR;
- The third model is an mathematical model of a motor with rod and wheel, the second system that was studied. It is controlled by a feedback gain obtained with LQR;
- The fourth model is an mathematical model of a motor with rod and wheel, the second system that was studied. It is controlled by Polinomial Pole Placement;
- The fifth model is an mathematical model of a motor with rod and wheel, the second system that was studied. It is controlled by Polinomial Pole Placement with adaptive control.

The Arduino Code folder contains the code used to controll the prototypes:
- The Lego prototype;
- The Aluminium prototype.
