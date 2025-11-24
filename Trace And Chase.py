import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import time

# ============================================================================
# Variables
# ============================================================================
Straight_time = 25      # Duration of first straight segment (s)
curve_time = 25         # Duration of turn (s)
Straight_time2 = 25     # Duration of second straight segment (s)
targ_vel = 750          # Target velocity (m/s)
miss_vel = 800          # missile velocity (m/s)
turn_angle = -np.pi*4/3   # Turn angle in radians (np.pi = 180°, np.pi/2 = 90°, etc.)
tmax = 75
dt = 0.001
animation_interval = 5  # milliseconds
yz_angle = -np.pi/12
missile_start_loc = np.array([13000, 12000, 0])
aircraft_start_loc = np.array([0, 0, 12000])  # Aircraft starting position
missile_launch_time = 0  # Time when missile launches (s)
kill_dist=2
climb_rate_curve= -0.001
# ============================================================================
# Global variables for segment start positions
# ============================================================================
curve_start_x = None
curve_start_y = None
curve_start_z = None
curve_initialized = False

straight2_start_x = None
straight2_start_y = None
straight2_start_z = None
straight2_initialized = False

radius = (targ_vel * curve_time) / turn_angle  # Calculate radius once
center_x = None
center_y = None
center_z = None

# ============================================================================
# TARGET LOCATION
# ============================================================================
def target_location(t, target_states):
    """
    Calculate target position at time t.
    
    Parameters:
    - t: time (s)
    - target_states: array of previous target states for initialization
    
    Returns:
    - np.array([x, y, z]): position in meters
    """
    global curve_start_x, curve_start_y, curve_start_z, curve_initialized
    global straight2_start_x, straight2_start_y, straight2_start_z, straight2_initialized
    global center_x, center_y, center_z
    
    if 0 <= t <= Straight_time:
        # Straight flight in +X direction from starting position
        x = aircraft_start_loc[0] + targ_vel * t
        y = aircraft_start_loc[1]
        z = aircraft_start_loc[2]
        
    elif Straight_time < t <= Straight_time + curve_time:
        # Curved turn with proper center
        tc = t - Straight_time
        # First time entering curve segment - store start position
        if not curve_initialized:
            if len(target_states) > 0:
                curve_start_x = target_states[-1, 0]
                curve_start_y = target_states[-1, 1]
                curve_start_z = target_states[-1, 2]
            else:
                # Fallback for edge case
                curve_start_x = aircraft_start_loc[0] + targ_vel * Straight_time
                curve_start_y = aircraft_start_loc[1]
                curve_start_z = aircraft_start_loc[2]
            
            # Define center of circular arc
            center_x = curve_start_x
            center_y = curve_start_y + radius * np.cos(yz_angle)
            center_z = curve_start_z + radius * np.sin(yz_angle)
            
            curve_initialized = True
        
        angle = tc * turn_angle / curve_time  # 0 to turn_angle over curve_time
        
        # Position on circular arc
        # Start at angle = -pi/2 (pointing in +X direction from center)
        arc_angle = -np.pi/2 + angle
        
        x = center_x + radius * np.cos(arc_angle)
        y = center_y + radius * np.sin(arc_angle) * np.cos(yz_angle) + np.cos(yz_angle+np.pi/2) * targ_vel**2 * (1-np.cos(np.pi*tc/curve_time)) * climb_rate_curve
        z = center_z + radius * np.sin(arc_angle) * np.sin(yz_angle) + np.sin(yz_angle+np.pi/2) * targ_vel**2 * (1-np.cos(np.pi*tc/curve_time)) * climb_rate_curve
        
    elif Straight_time + curve_time < t <= Straight_time + curve_time + Straight_time2:
        # Straight flight after turn
        
        # First time entering second straight segment - store start position
        if not straight2_initialized:
            if len(target_states) > 0:
                straight2_start_x = target_states[-1, 0]
                straight2_start_y = target_states[-1, 1]
                straight2_start_z = target_states[-1, 2]
            else:
                # Fallback for edge case
                straight2_start_x = curve_start_x
                straight2_start_y = curve_start_y
                straight2_start_z = curve_start_z
            
            straight2_initialized = True
        
        ts = t - (Straight_time + curve_time)
        
        # Direction after turn (rotated by turn_angle from initial +X direction)
        dx = np.cos(turn_angle)
        dy = np.sin(turn_angle) * np.cos(yz_angle)
        dz = np.sin(turn_angle) * np.sin(yz_angle)
        
        # Continue in new direction
        x = straight2_start_x + targ_vel * ts * dx
        y = straight2_start_y + targ_vel * ts * dy
        z = straight2_start_z + targ_vel * ts * dz
    
    else:
        # Out of bounds - return last known position
        if straight2_initialized:
            ts_max = Straight_time2
            dx = np.cos(turn_angle)
            dy = np.sin(turn_angle) * np.cos(yz_angle)
            dz = np.sin(turn_angle) * np.sin(yz_angle)
            x = straight2_start_x + targ_vel * ts_max * dx
            y = straight2_start_y + targ_vel * ts_max * dy
            z = straight2_start_z + targ_vel * ts_max * dz
        else:
            # Fallback if never initialized
            x = aircraft_start_loc[0] + targ_vel * Straight_time
            y = aircraft_start_loc[1]
            z = aircraft_start_loc[2]
    
    return np.array([x, y, z])

# ============================================================================
# GENERATE TARGET TRAJECTORY Array
# ============================================================================
# Reset initialization flags before generating trajectory
curve_initialized = False
straight2_initialized = False

# Time array
times = np.arange(0, tmax, dt)
n_points = len(times)

# Generate target states using regular for loop
target_states = np.zeros((n_points, 3))
for i in range(n_points):
    t = times[i]
    target_states[i] = target_location(t, target_states[:i])

print(f"Generated {n_points} trajectory points over {tmax:.1f} seconds")
print(f"Aircraft start: ({aircraft_start_loc[0]:.1f}, {aircraft_start_loc[1]:.1f}, {aircraft_start_loc[2]:.1f})")
print(f"Curve start: ({curve_start_x:.1f}, {curve_start_y:.1f}, {curve_start_z:.1f})")
print(f"Straight2 start: ({straight2_start_x:.1f}, {straight2_start_y:.1f}, {straight2_start_z:.1f})")

# ============================================================================
# GENERATE MISSILE TRAJECTORY
# ============================================================================
missile_states = np.zeros((n_points, 3))
missile_states[0] = missile_start_loc
missile_launched = False
intercept_time = None
intercept_index = None
intercepted = False  # <-- Add this line

for i in range(1, n_points):
    t = times[i]
    
    # Check if missile should launch
    if t >= missile_launch_time and not missile_launched:
        missile_launched = True
        print(f"Missile launched at t = {t:.2f}s")
    
    if missile_launched:
        # Missile holds intercept point after intercept occurs
        if intercepted:
            missile_states[i] = missile_states[i-1]
            continue

        # Calculate direction to target's current position
        direction = target_states[i] - missile_states[i-1]
        distance = np.linalg.norm(direction)
        
        # Check for intercept
        if distance < kill_dist and intercept_time is None:
            intercept_time = t
            intercept_index = i
            intercepted = True  # <-- Set as soon as interception occurs
            print(f"Intercept at t = {t:.2f}s, distance = {distance:.1f}m")
            missile_states[i] = missile_states[i-1]  # Stop missile at intercept point
            continue
        
        # Update missile position if not intercepted
        if distance > 0:
            unitvec = direction / distance
            dr = unitvec * miss_vel * dt
            missile_states[i] = missile_states[i-1] + dr
        else:
            # Target reached, stop moving
            missile_states[i] = missile_states[i-1]
    else:
        # Missile hasn't launched yet, stays at starting position
        missile_states[i] = missile_start_loc

# Calculate final miss distance
final_distance = np.linalg.norm(target_states[-1] - missile_states[-1])
print(f"Final miss distance: {final_distance:.1f}m")

# ============================================================================
# CREATE 3D PLOT
# ============================================================================
fig = plt.figure(figsize=(14, 10))
ax = fig.add_subplot(111, projection='3d')

# Set axis limits based on both trajectories
all_points = np.vstack([target_states, missile_states])
padding = 0.1
x_range = np.ptp(all_points[:, 0])
y_range = np.ptp(all_points[:, 1])
z_range = np.ptp(all_points[:, 2])
max_range = max(x_range, y_range, z_range)

x_center = (np.max(all_points[:, 0]) + np.min(all_points[:, 0])) / 2
y_center = (np.max(all_points[:, 1]) + np.min(all_points[:, 1])) / 2
z_center = (np.max(all_points[:, 2]) + np.min(all_points[:, 2])) / 2

plot_radius = max_range / 2 * (1 + padding)

ax.set_xlim(x_center - plot_radius, x_center + plot_radius)
ax.set_ylim(y_center - plot_radius, y_center + plot_radius)
ax.set_zlim(z_center - plot_radius, z_center + plot_radius)
ax.set_box_aspect([1, 1, 1])

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('3D Missile-Aircraft Pursuit Simulation - Trace And Chase')
ax.grid(True)
ax.view_init(elev=20, azim=45)

# Create animation artists
target_point, = ax.plot([], [], [], 'bo', markersize=10, label='Aircraft')
target_trail, = ax.plot([], [], [], 'b-', linewidth=2, alpha=0.5, label='Aircraft Trail')
missile_point, = ax.plot([], [], [], 'ro', markersize=8, label='Missile')
missile_trail, = ax.plot([], [], [], 'r-', linewidth=1.5, alpha=0.5, label='Missile Trail')
time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=12)
speed_text = ax.text2D(0.02, 0.90, '', transform=ax.transAxes, fontsize=10)
distance_text = ax.text2D(0.02, 0.85, '', transform=ax.transAxes, fontsize=10)

# Starting position markers
ax.scatter(target_states[0, 0], target_states[0, 1], target_states[0, 2], 
           c='green', s=100, marker='s', label='Aircraft Start')
ax.scatter(missile_start_loc[0], missile_start_loc[1], missile_start_loc[2],
           c='orange', s=100, marker='^', label='Missile Start')

# Mark intercept point if it exists
if intercept_index is not None:
    ax.scatter(target_states[intercept_index, 0], target_states[intercept_index, 1], 
               target_states[intercept_index, 2], 
               c='red', s=200, marker='*', label='Intercept')

ax.legend()

# ============================================================================
# ANIMATION FUNCTIONS
# ============================================================================
def init():
    """Initialize animation."""
    target_point.set_data([], [])
    target_point.set_3d_properties([])
    target_trail.set_data([], [])
    target_trail.set_3d_properties([])
    missile_point.set_data([], [])
    missile_point.set_3d_properties([])
    missile_trail.set_data([], [])
    missile_trail.set_3d_properties([])
    time_text.set_text('')
    speed_text.set_text('')
    distance_text.set_text('')
    return target_point, target_trail, missile_point, missile_trail, time_text, speed_text, distance_text


def update(frame):
    """Update animation for given frame."""
    # Update aircraft position
    target_point.set_data([target_states[frame, 0]], [target_states[frame, 1]])
    target_point.set_3d_properties([target_states[frame, 2]])
    
    # Update aircraft trail
    target_trail.set_data(target_states[:frame+1, 0], target_states[:frame+1, 1])
    target_trail.set_3d_properties(target_states[:frame+1, 2])
    
    # Update missile position
    missile_point.set_data([missile_states[frame, 0]], [missile_states[frame, 1]])
    missile_point.set_3d_properties([missile_states[frame, 2]])
    
    # Update missile trail
    missile_trail.set_data(missile_states[:frame+1, 0], missile_states[:frame+1, 1])
    missile_trail.set_3d_properties(missile_states[:frame+1, 2])
    
    # Calculate current speed (for display)
    if frame > 0:
        dx = target_states[frame, 0] - target_states[frame-1, 0]
        dy = target_states[frame, 1] - target_states[frame-1, 1]
        dz = target_states[frame, 2] - target_states[frame-1, 2]
        speed = np.sqrt(dx**2 + dy**2 + dz**2) / dt
    else:
        speed = targ_vel
    
    # Calculate distance between missile and target
    distance = np.linalg.norm(target_states[frame] - missile_states[frame])
    
    # Update text
    time_text.set_text(f'Time = {times[frame]:.2f} s')
    speed_text.set_text(f'Target Speed = {speed:.1f} m/s')
    distance_text.set_text(f'Distance = {distance:.1f} m')
    
    return target_point, target_trail, missile_point, missile_trail, time_text, speed_text, distance_text


# ============================================================================
# CREATE AND SHOW ANIMATION
# ============================================================================
# Skip frames for smoother playback (reduce to 500 frames max)
frame_skip = max(1, len(times) // 500)
frames = range(0, len(times), frame_skip)

print(f"Animation will show {len(frames)} frames")

anim = FuncAnimation(fig, update, frames=frames, init_func=init, 
                     blit=False, interval=animation_interval, repeat=True)

print("Showing animation...")
plt.show()
