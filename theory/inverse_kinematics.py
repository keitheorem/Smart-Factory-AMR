import numpy as np

def forward_kinematics(joint_angles, link_lengths):
    # Define symbolic variables for joint angles and link lengths
    theta1, theta2, theta3, theta4, theta5 = joint_angles
    l1, l2, l3, l4, l5 = link_lengths

    # T1: fixed link to servo No.1 
    T1 = np.array([
        [np.cos(theta1), -np.sin(theta1), 0, 0],
        [np.sin(theta1),  np.cos(theta1), 0, 0],
        [0,              0,               1, 0],
        [0,              0,               0, 1]
    ])

    # T2: Rotation along y-axis, translation along z-axis (servo No.1 to No.2)
    T2 = np.array([
        [np.cos(theta2),  0, np.sin(theta2), 0],
        [0,               1, 0,               0],
        [-np.sin(theta2), 0, np.cos(theta2), l1],
        [0,               0, 0,               1]
    ])

    # T3: Rotation along y-axis, translation along z-axis (servo No.2 to No.3)
    T3 = np.array([
        [np.cos(theta3),  0, np.sin(theta3), 0],
        [0,               1, 0,               0],
        [-np.sin(theta3), 0, np.cos(theta3), l2],
        [0,               0, 0,               1]
    ])

    # T4: Rotation along y-axis, translation along z-axis (servo No.3 to No.4)
    T4 = np.array([
        [np.cos(theta4),  0, np.sin(theta4), 0],
        [0,               1, 0,               0],
        [-np.sin(theta4), 0, np.cos(theta4), l3],
        [0,               0, 0,               1]
    ])

    # T5: Rotation about z-axis, translation along z-axis
    T5 = np.array([
        [np.cos(theta5), -np.sin(theta5), 0, 0],
        [np.sin(theta5),  np.cos(theta5), 0, 0],
        [0,              0,               1, l4],
        [0,              0,               0, 1]
    ])


    # T6: Fixed link, translation along z-axis
    T6 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, l5],
        [0, 0, 0, 1]
    ])

    # Overall transformation matrix from base to end-effector
    T = T1 @ T2 @ T3 @ T4 @ T5 @ T6

    return T


def jacobian_matrix(joint_angles, link_lengths):
    delta = 1e-4
    n_joints = len(joint_angles)
    jacobian = np.zeros((6, n_joints))  # 6xN Jacobian: 3 rows for position, 3 for orientation

    T = forward_kinematics(joint_angles, link_lengths)
    position = T[:3, 3]  # Extract end-effector position
    orientation = T[:3, :3]  # Extract end-effector orientation (rotation matrix)

    for i in range(n_joints):
        perturbed_angles = joint_angles.copy()
        perturbed_angles[i] += delta  # Perturb the i-th joint angle

        T_perturbed = forward_kinematics(perturbed_angles, link_lengths)
        position_perturbed = T_perturbed[:3, 3]
        orientation_perturbed = T_perturbed[:3, :3]

        # position portion of jacobian
        jacobian[:3, i] = (position_perturbed - position) / delta

        # orientation portion of jacobian
        orientation_difference = (orientation_perturbed - orientation) / delta
        jacobian[3:, i] = np.array([
            orientation_difference[2, 1] - orientation_difference[1, 2],  # wx
            orientation_difference[0, 2] - orientation_difference[2, 0],  # wy
            orientation_difference[1, 0] - orientation_difference[0, 1],  # wz
        ])

    return jacobian

def normalize_angle(angle):
    # Normalize angle to the range [-pi, pi]
    return (angle + np.pi) % (2 * np.pi) - np.pi

def compute_orientation_error(current_orientation, target_orientation):
    # Compute orientation error using the skew-symmetric matrix 
    R_error = target_orientation @ current_orientation.T
    orientation_error = 0.5 * np.array([
        R_error[2, 1] - R_error[1, 2],
        R_error[0, 2] - R_error[2, 0],
        R_error[1, 0] - R_error[0, 1],
    ])
    return orientation_error

def inverse_kinematics_gradient_descent(
    target_pose, 
    initial_angles,
    link_lengths,
    alpha=0.3, 
    max_iter=5000,
    tolerance=1e-4
):
    joint_angles = np.array(initial_angles)

    for iteration in range(max_iter):
        current_transform = forward_kinematics(joint_angles, link_lengths)
        current_position = current_transform[:3, 3] # Translation column matrix 
        current_orientation = current_transform[:3, :3]  # Rotation matrix

        target_position = target_pose[:3]

        # note that this approach assumes that rotation is only about z-axis
        target_orientation = np.array([
            [np.cos(target_pose[5]), -np.sin(target_pose[5]), 0],
            [np.sin(target_pose[5]),  np.cos(target_pose[5]), 0],
            [0,                      0,                      1],
        ])

        # Compute position error
        position_error = target_position - current_position

        # Compute orientation error
        orientation_error = compute_orientation_error(current_orientation, target_orientation)

        # Combine errors and normalize
        position_error_scaled = position_error  # Scale if necessary (e.g., divide by max link length)
        orientation_error_scaled = orientation_error  # Scale based on rotation limits

        error = np.concatenate((position_error_scaled, orientation_error_scaled))
        error_norm = np.linalg.norm(error)

        print(f"Iteration: {iteration}, Error norm: {error_norm}")

        # Check for convergence
        if error_norm < tolerance:
            print(f"Converged in {iteration} iterations.")
            return joint_angles
        

        # Compute Jacobian
        jacobian = jacobian_matrix(joint_angles, link_lengths)

        # Compute pseudo-inverse of the Jacobian
        jacobian_pseudo_inv = np.linalg.pinv(jacobian)

        # Update joint angles using gradient descent
        joint_angles -= alpha * jacobian_pseudo_inv @ error

        # Normalize joint angles (optional, for revolute joints)
        joint_angles = np.array([normalize_angle(angle) for angle in joint_angles])

    print("Did not converge within the maximum number of iterations.")
    return joint_angles


link_lengths = [0.037, 0.13, 0.13, 0.054, 0.07]
target_pose = np.array([0.1, 0.01, 0.35, 0.0, 0.0, np.pi])  # Target position and orientation
initial_angles = [0.0, 0.0, 0.0, 0.0, 0.0]  # Initial joint angles

final_angles = inverse_kinematics_gradient_descent(target_pose, initial_angles, link_lengths)
final_angles = np.rad2deg(final_angles)
print("Final Joint Angles:", final_angles)

