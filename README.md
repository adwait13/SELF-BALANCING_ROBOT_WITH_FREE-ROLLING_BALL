PROJECT: SELF-BALANCING ROBOT WITH FREE-ROLLING BALL
================================================================================

Course:      ME4010 - Control Systems
Institution: IIT Madras
ProfessorL   Dr. Manish Anand
Author:      Adwait Thosare (ME23B123)


1. PROJECT OVERVIEW
-------------------
This project involves the modeling, simulation, and control system design for 
a self-balancing robot balancing a free-rolling ball on its head. The work 
progresses from first-principles modeling (Lagrangian mechanics) to advanced 
state-space control (LQR) and observer design (FSO and ROO).

The project compares Classical Control (PID) against Modern Control (LQR) and 
evaluates observer performance under realistic sensor noise conditions.


2. DEPENDENCIES
---------------
- MATLAB
- Simulink
- Symbolic Math Toolbox (Required for derive_model.m)
- Control System Toolbox


3. ORDER OF EXECUTION (HOW TO RUN)
----------------------------------
IMPORTANT: Please run the scripts in this specific order to ensure all 
parameters and matrices are loaded correctly into the MATLAB workspace.

STEP 1: SYSTEM MODELING
   Run file: "derive_model.m"
   - Defines physical parameters (mass, inertia, dimensions).
   - Derives Equations of Motion (EoM) and linearizes the system.
   - Calculates State-Space matrices (A, B, C, D).
   - Output: Generates "model_data.mat".

STEP 2: CONTROLLER DESIGN
   
   Option A: LQR (Recommended)
   Run file: "run_lqr.m"
   - Loads "model_data.mat".
   - Defines Weighting Matrices Q and R.
   - Computes the optimal Gain Matrix K.
   - Output: Generates "lqr_data.mat" and plots system response.

   Option B: PID Control
   Run file: "pid_controller.m"
   - Implements a cascaded PID architecture.
   - Plots impulse/step response for comparison with LQR.

STEP 3: OBSERVER SIMULATION (SIMULINK)
   (Note: Requires "lqr_data.mat" to be in the workspace)

   - Full State Observer (FSO):
     1. Run "FSO_code.m" (Initializes observer gains and parameters).
     2. Open and run "FSO.slx".
     
   - Reduced Order Observer (ROO):
     1. Run "ROO_code.m" (Initializes partitioning and suppression logic).
     2. Open and run "ROO.slx".


4. FILE MANIFEST AND DESCRIPTIONS
---------------------------------

derive_model.m
    Core script. Derives symbolic EoM, linearizes the plant, and checks 
    observability/controllability.

robot_dynamics.m
    Non-linear dynamics definition. Encodes the full, non-linear equations of 
    motion used to simulate the true physical behavior of the robot.

run_lqr.m
    LQR design script. Tunes Q/R matrices and plots closed-loop performance.

pid_controller.m
    Classical control baseline. Implements cascaded PID loops.

FSO_code.m
    Initialization script for the Full State Observer. Calculates observer 
    gains (Ke) via pole placement and sets filter parameters.

ROO_code.m
    Initialization script for the Reduced Order Observer. Performs system 
    partitioning, gain calculation, and handles transient suppression logic.

noise.m
    Generates FFT plots to characterize sensor noise and justifies the design 
    of the Low Pass Filter (15Hz cutoff).

model_data.mat
    Data file storing the Plant matrices (A, B, C, D) and physical parameters.

lqr_data.mat
    Data file storing the computed LQR Gain vector (K).

FSO.slx
    Simulink model for the Full State Observer with LQR feedback.

ROO.slx
    Simulink model for the Reduced Order Observer with LQR feedback.

================================================================================

PS: If you're unable to view the report please click on this link:
https://drive.google.com/file/d/1IbOQ8TQvOPd9NX32A_KbWb5Ku5mqDTPK/view?usp=sharing
