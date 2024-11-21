import numpy as np
from kinematics_key_hw06 import forward_kinematics, jacobian

# Constants for potential field
ALPHA = 1.0  # Attractive potential gain
BETA = 1.0   # Repulsive potential gain
DIST_THRESHOLD = 1.5  # Distance at which repulsive forces start acting
STEP_SIZE = 0.1  # Step size for each iteration

def attractive_force(current_position, goal):
    """Calculate the attractive force towards the goal."""
    return ALPHA * (np.array(goal) - np.array(current_position))

def repulsive_force(current_position, obst_location, obst_radius):
    """Calculate the repulsive force from the obstacle."""
    dist_to_obstacle = np.linalg.norm(current_position - np.array(obst_location))
    if dist_to_obstacle > obst_radius + DIST_THRESHOLD:
        return np.zeros(3)  # No repulsive force if far enough from obstacle
    else:
        # Repulsive force decreases as robot gets further from obstacle
        return BETA * (1.0 / dist_to_obstacle - 1.0 / (obst_radius + DIST_THRESHOLD)) * \
               (current_position - np.array(obst_location)) / (dist_to_obstacle**3)

def compute_robot_path(q_init, goal, obst_location, obst_radius, max_iterations=1000):
    """Compute a collision-free path to the goal."""
    q_current = np.array(q_init)
    q_ik_slns = [q_current.copy()]  # List to store joint configurations over time
    
    for i in range(max_iterations):
        # Compute current end-effector position using forward kinematics
        current_position = forward_kinematics(q_current)

        # Calculate forces (attractive and repulsive)
        f_attr = attractive_force(current_position, goal)
        f_rep = repulsive_force(current_position, obst_location, obst_radius)
        f_total = f_attr + f_rep

        # If the end-effector is close to the goal, stop
        if np.linalg.norm(f_attr) < 1e-2:  # Threshold for convergence
            break

        # Compute joint velocities using the Jacobian
        J = jacobian(q_current)  # Get the Jacobian matrix for the current configuration
        dq = np.linalg.pinv(J).dot(f_total)  # Compute joint velocities

        # Update joint angles
        q_current += STEP_SIZE * dq
        q_ik_slns.append(q_current.copy())

    return np.array(q_ik_slns)

# Example usage
q_init = [0.0, -np.pi/4, np.pi/2, 0.0]  # Initial configuration (example)
goal = [0, 2, 4]  # Goal position
obst_location = [0, 3, 2]  # Obstacle location
obst_radius = 1.0  # Obstacle radius

q_ik_slns = compute_robot_path(q_init, goal, obst_location, obst_radius)
print(q_ik_slns)