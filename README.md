# Data-Driven Controller for a Two-Wheeled Self-Balancing Robot

This repository contains the MATLAB code and associated files for implementing a data-driven controller on a two-wheeled self-balancing robot. The project uses a model-free approach to control an inherently unstable, nonlinear robotic system, with controller gains calculated based on recorded input-output data alone. This design is inspired by recent advancements in data-driven control, particularly the work presented in [De Persis and Tesi, IEEE Transactions on Automatic Control, 2019](https://doi.org/10.1109/TAC.2019.2959924).

## Project Overview

The two-wheeled self-balancing robot is controlled similarly to an inverted pendulum, requiring real-time adjustments to maintain stability. Traditional controllers, such as PID or LQR, depend on a known model of the system. In this project, however, we use a data-driven method, which enables the design of an effective controller without explicit system modeling. By leveraging Linear Matrix Inequalities (LMI) within the CVX toolbox, this approach stabilizes the robot based on input-output data alone, building on techniques established by De Persis and Tesi (2019).

The main reference for this work, *“Formulas for Data-Driven Control: Stabilization, Optimality, and Robustness”* by De Persis and Tesi, demonstrates a robust methodology for stabilizing control of nonlinear systems using data-driven techniques without explicit model identification. The paper shows that sufficiently rich input-output data can substitute traditional model-based approaches, addressing both optimality and robustness concerns. This project applies these principles to control a two-wheeled self-balancing robot, testing and extending the theoretical framework provided by De Persis and Tesi in a real-world robotic system.

### Key Features

- **Data-Driven Control**: Uses recorded input-output data from prior experiments to calculate the controller gains directly.
- **LMI Optimization**: Implements the CVX toolbox in MATLAB to solve an LMI problem, ensuring stability through calculated gains.
- **Empirical Validation**: Compares the data-driven approach to a conventional PID controller, with favorable results in terms of stability, settling time, and disturbance rejection.

## Hardware Specifications

The self-balancing robot consists of:
- **Arduino UNO** for real-time control processing.
- **NEMA 17 Stepper Motors** with **A4988 Motor Drivers**.
- **MPU-6050 IMU** to measure the robot’s tilt and angular speed.
- A **custom-built wooden frame** and wheels for stability.

<img src="/20211120_150846.jpg" width="400" alt="Robot Image">


## Repository Structure

- `DataDriven_Controller_n7.ino`: Arduino code uploaded on the Uno Arduino board on the robot for controlling the motors and recording the data.
- `controller_design.m`: MATLAB script to set up the LMI and compute the controller gain `Kcvx`.
- `Book1S2.csv`: Recorded input-output data.
- `README.md`: This README file.
- Additional hardware specifications and component details are included in the accompanying project paper.

## Getting Started

### Prerequisites

- **MATLAB** with **CVX Toolbox** installed.
- **Recorded data**: Loading the input-output data in the correct format (`y` for outputs and `u` for control inputs).

### Running the Code

1. **Load Data**: to the MATLAB file provided.
2. **Run `controller_design.m`**: This script will compute the controller gain using LMI optimization.
3. **Deploy**: Update the Arduino code with the generated controller gains, upload the computed gain to the robot’s microcontroller, and test its performance.

## Results and Evaluation

Experimental results confirm that the data-driven controller has superior performance over a PID controller, especially in handling disturbances and reducing oscillations. [See the project paper for detailed comparisons and performance graphs](https://doi.org/10.1109/ICEE55646.2022.9827321).

## PID Controller
You can explore the implementation of the PID controller and the MATLAB GUI for remote control of this robot in this [GitHub repository](https://github.com/MDKAM/Self-Balancing-Robot_GUI_MATLAB.git). Additionally, the repository provides detailed information about the hardware structure.

## Authors

- Mohammad Akhavan, Faculty of Electrical Engineering, Shahid Beheshti University
- Haniye Parvahan, Faculty of Electrical Engineering, Shahid Beheshti University
- Mojtaba Nouri Manzar, Faculty of Electrical Engineering, Shahid Beheshti University
