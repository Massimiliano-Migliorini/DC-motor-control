# DC-motor-control
 DC Motor Control for Tramway Applications - Project for the course "Dynamics of Electrical Machines" at Politecnico di Milano

 ## 🚀 Project Summary
Individual project carried out as part of the “Dynamics of Electrical Machines and Drives” course at Politecnico di Milano.

• Developed a complete control scheme for a real-world tram system powered by four DC motors, modeled as a single equivalent separately-excited DC machine.

• Derived missing mechanical and electrical parameters based on technical data and constructed a realistic simulation model of the motor-tram system.

• Designed and implemented a cascade PI control architecture to ensure accurate tracking of a 10 km reference speed profile, accounting for varying track slopes and vehicle loading conditions.

• Built a detailed Simulink model including torque and speed control loops, excitation control, anti-windup strategies, and actuator saturation.

• Tuned PI controllers in MATLAB based on the cascade control principle, ensuring decoupled and stable system response.

## 📁 Contents
- `report.pdf`: detailed technical report (EN)
- `DC_motor_script.m`: MATLAB implementation
- `DC_motor_simulink.slx`: Simulink simulation

## ▶️ How to Run the Simulation
To execute the full simulation workflow:

1. **Run the MATLAB script**  
   Open `DC_motor_script.m` in MATLAB and run it. This script initializes all necessary parameters and populates the workspace.  
   You can freely modify the parameters in the script to explore different system behaviors.
2. **Open and run the Simulink model**  
   Launch `DC_motor_simulink.slx` and click **Run**.  
   The system response will be displayed directly in the embedded **Scope** block.

NB: Make sure to run the MATLAB script **before** the Simulink model, so that all variables are correctly defined in the workspace.

## 📷 Results
see `report.pdf`

## 🛠️ Technologies Used
- MATLAB / Simulink
- Control Systems Theory
