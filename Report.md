# Control and Trajectory Tracking for Autonomous Vehicle
## Step-by-Step Instructions

### Step 1: Set Up the PID Controller

1. **Build the PID Controller Object**  
   - Complete the TODO tasks in `pid_controller.h`, `pid_controller.cpp` and `main.cpp`.
   - After implementing the code, run the CARLA simulator in desktop mode. Take a screenshot of the car in the simulation to include in your report. At this stage, the car should remain stationary.

### Step 2: Implement Throttle Control

1. **Compute Throttle Error**  
   - In `main.cpp`, follow the instructions for step 2 to calculate the error for the throttle PID controller. The error is defined as the difference between the actual speed and the target speed.
   
   **Useful Variables:**  
   - `v_points`: The last element contains the velocity determined by the path planner.  
   - `velocity`: Represents the current speed of the car.  
   
   - The output from the PID controller should be within the range [-1, 1].
   
2. **Tune the PID Parameters**  
   - Adjust the PID parameters to achieve satisfactory performance. While a perfectly smooth trajectory isn't required, aim for a stable and responsive throttle control.

3. **Document Your Work**  
   - Comment the code to explain your approach for computing the error.

### Step 3: Implement Steering Control

1. **Compute Steering Error**  
   - In `main.cpp`, follow the instructions for step 3 to compute the error for the steering PID controller. This error is based on the difference between the current steering angle and the desired steering angle to stay on the planned path.
   
   **Useful Variables:**  
   - `y_points` and `x_points`: These arrays represent the planned trajectory coordinates from the path planner.  
   - `yaw`: The current orientation of the car.  
   
   - The controller output should be in the range [-1.2, 1.2].

2. **Tune the PID Parameters**  
   - Continue tuning the parameters until the steering control performs well. Focus on reducing oscillations and ensuring the car tracks the desired path.

3. **Document Your Work**  
   - Comment on the code to clarify why you chose a particular approach for computing the error.

### Step 4: Analyze and Evaluate the Controller's Performance

1. **Plot the Results**  
   - The error values and control outputs for the throttle and steering are saved in `throttle_data.txt` and `steer_data.txt`, respectively.  
   - Use the following command to plot these values:  
     ```bash
     python3 plot_pid.py
      ```
![Result sample 1](project/pid_controller/screenshot/Screenshot%202024-10-13%20071859.png)
![Result sample 2](project/pid_controller/screenshot/Screenshot%202024-10-13%20072254.png)
![Result sample 3](project/pid_controller/screenshot/Screenshot%202024-10-13%20072934.png)

## Anser question
# PID Controller Analysis

## 1. Add the plots to your report and explain them

In the plots provided, we observe the system's response to a setpoint change over time. The first plot illustrates the output of the PID controller versus time, showing how quickly and accurately the system reaches the desired setpoint.

- **Steady-State Response**: The output stabilizes at the setpoint, indicating that the PID controller effectively maintains the desired value.
- **Overshoot**: There may be instances where the output exceeds the setpoint before settling. This is typically a result of the proportional component being too high.
- **Settling Time**: The time taken for the output to settle within a specific error band around the setpoint reflects the system's responsiveness and stability.

## 2. What is the effect of the PID according to the plots, how each part of the PID affects the control command?

The PID controller consists of three components, each contributing uniquely to the control command:

- **Proportional (P)**: This component reacts to the current error. A higher proportional gain results in a larger control output for a given error, which can lead to faster responses but may cause overshoot and oscillations.

- **Integral (I)**: This term accumulates the error over time, correcting any steady-state offset. If the error persists, the integral term increases, helping eliminate the residual steady-state error. However, excessive integral gain can lead to overshoot and instability.

- **Derivative (D)**: This component predicts future errors based on the current rate of change, providing a damping effect. It helps reduce overshoot and stabilize the system but can amplify noise in the measurement signal.

## 3. How would you design a way to automatically tune the PID parameters?

To design an automatic PID tuning method, one could implement a few strategies:

- **Ziegler-Nichols Method**: This classical method involves setting the I and D gains to zero and increasing the P gain until the system oscillates. The ultimate gain and period are then used to calculate the PID parameters.

- **Cohen-Coon Method**: This is another empirical method, which provides more accurate tuning by requiring the process reaction curve to determine the optimal PID parameters.

- **Model-Free Tuning Algorithms**: Utilize algorithms such as Genetic Algorithms or Particle Swarm Optimization to iteratively adjust PID parameters based on the system's performance.

- **Software Tools**: Implement software that analyzes the system's response to disturbances or setpoint changes, adjusting the PID parameters dynamically to optimize performance.

## 4. PID controller is a model-free controller; could you explain the pros and cons of this type of controller?

### Pros:

- **Simplicity**: PID controllers are straightforward to implement and understand, requiring minimal system knowledge.
- **Robustness**: They can handle a variety of systems effectively, even when the system dynamics are not precisely known.
- **Flexibility**: Easy to tune and adjust for different operating conditions.

### Cons:

- **Lack of Adaptability**: Without a model, PID controllers may struggle with nonlinear systems or those that change over time, potentially leading to suboptimal performance.
- **Performance Limitations**: They may not perform well in systems requiring precise control or complex dynamics, leading to issues like oscillation or slow response.
- **No Predictive Capability**: PID controllers cannot anticipate future behavior; they react to errors after they occur, which may result in slower responses in dynamic systems.

## 5. (Optional) What would you do to improve the PID controller?

To improve the performance of a PID controller, consider the following strategies:

- **Advanced Control Strategies**: Implement adaptive control, fuzzy logic controllers, or model predictive control for complex systems that need more than traditional PID tuning.
  
- **Feedforward Control**: Combine PID with feedforward control to anticipate and counteract disturbances before they affect the system.

- **Anti-Windup Mechanisms**: Implement anti-windup techniques to prevent the integral term from accumulating excessively during saturation.

- **Multi-Loop Control**: Use a multi-loop PID controller for systems with multiple interacting variables to manage complex interactions more effectively.

- **Continuous Monitoring and Tuning**: Integrate machine learning algorithms that continuously analyze system performance and adjust PID parameters in real-time for optimal control.


 