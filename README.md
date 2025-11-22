About
Designed as a demonstration tool for (simplified) aerospace dynamics. Contributions and scenario expansions are welcome.

3D Missile–Aircraft Pursuit Simulation
This repository contains a Python simulation and visualization of a missile pursuing an aircraft in 3D space. Aircraft movement includes distinct straight-line and curved segments, and the missile uses proportional pursuit to intercept the target. Results are visualized in an animated 3D plot with real-time position and distance tracking.
Features
	•	Multi-segment target trajectory: Straight flight, 3D turn, and final straight segment simulation.
	•	Adaptive missile guidance: Missile steers towards aircraft’s instantaneous position.
	•	Configurable scenario: Set speeds, launch positions, kill distance, turn geometry, and timings.
	•	3D animated visualization using Matplotlib.
	•	Real-time intercept distance and marker display.
	•	Customizable for aerospace research or education purposes.
Requirements
	•	Python 3.7+
	•	Numpy
	•	matplotlib
	•	scipy

The program will:
	•	Simulate both missile and aircraft trajectories.
	•	Print key simulation events (missile launch, intercept).
	•	Visualize the pursuit in a 3D Matplotlib animation.
Parameters
Key variables exposed for scenario customization:
	•	 Straight_time ,  curve_time ,  Straight_time2 : Segment durations.
	•	 targ_vel ,  miss_vel : Aircraft and missile speeds.
	•	 turn_angle ,  kill_dist ,  climb_rate_curve , etc.
Modify these in the script to change dynamics.
Output
On completion, the simulation will display:
	•	Aircraft and missile flight paths in 3D.
	•	Instantaneous positions, distances, and intercept (if achieved).
	•	Start and intercept points are marked for clarity.
