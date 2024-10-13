# Control and Trajectory Tracking for Autonomous Vehicle

In this project, you will apply the skills you have acquired in this course to design a Proportional-Integral-Derivative (PID) controller to perform vehicle trajectory tracking. Given a trajectory as an array of locations, and a simulation environment, you will design and code a PID controller and test its efficiency on the CARLA simulator used in the industry.


# Run the scripts Guideline

## Clone the Project Github Repository
Run code in WM Workspace Udacity
Clone https://github.com/udacity/nd013-c6-control-starter.git
cd cd nd013-c6-control-starter

Get the code to replace the files named main.cpp, pid_controller.cpp, and pid_controller.h located in the project/pid_controller directory.

## Start the Carla Server
Start the Carla server by executing the following shell script.
```bash
cd /opt/carla-simulator/
chmod +x CarlaUE4.sh  //grant execution rights
/opt/carla-simulator/CarlaUE4.sh
```


## Build and run code
Open new Terminal

```bash
cd nd013-c6-control-starter/project
chmod +x install-ubuntu.sh  //grant execution rights
./install-ubuntu.sh

cd pid_controller/
rm -rf rpclib //Delete rpclib

git clone https://github.com/rpclib/rpclib.git
cmake .
make //Compiler code C++


```
Open new Terminal
```bash
cd nd013-c6-control-starter/project
./run_main_pid.sh //ctrl + C when break console, If Carla error please ctrl +c and run this again

```

# summary of the project

In this project, you will enhance the autonomous vehicle's capabilities by implementing steering and throttle control, allowing the car to follow a planned path. This project builds on the previous one, where a path planner was developed.

You will design and tune a PID (Proportional-Integral-Derivative) controller, as discussed in the earlier course, to achieve this goal.

## Getting Started

In the `/pid_controller` directory, you will find the following files:

- `pid_controller.cpp`
- `pid_controller.h`

These are the files where you will implement the PID controller logic. The `pid` function is called in the main program located in `main.cpp`.

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


   **Note:** If needed, install additional Python modules:  
   ```bash
   pip3 install pandas
   pip3 install matplotlib
   ```







# Reference 
 - https://github.com/udacity/nd013-c6-control-starter
 - https://github.com/RuchitJathania/Control-and-Trajectory-Tracking-for-Autonomous-Vehicles

# video Reference
 - https://www.youtube.com/playlist?list=PL05umP7R6ij321zzKXK6XCQXAaaYjQbzr
 - https://www.youtube.com/watch?v=kVYy2kjZjhA
 