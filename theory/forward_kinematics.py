import sympy as sp

def forward_kinematics():
    # Define symbolic variables for joint angles and link lengths
    theta1, theta2, theta3, theta4, theta5 = sp.symbols('theta1 theta2 theta3 theta4 theta5')
    l1, l2, l3, l4, l5 = sp.symbols('l1 l2 l3 l4 l5')

    # T1: fixed link to servo No.1 
    T1 = sp.Matrix([
        [sp.cos(theta1), -sp.sin(theta1), 0, 0],
        [sp.sin(theta1),  sp.cos(theta1), 0, 0],
        [0,              0,               1, 0],
        [0,              0,               0, 1]
    ])

    # T2: Rotation along y-axis, translation along z-axis (servo No.1 to No.2)
    T2 = sp.Matrix([
        [sp.cos(theta2),  0, sp.sin(theta2), 0],
        [0,               1, 0,               0],
        [-sp.sin(theta2), 0, sp.cos(theta2), l1],
        [0,               0, 0,               1]
    ])

    # T3: Rotation along y-axis, translation along z-axis (servo No.2 to No.3)
    T3 = sp.Matrix([
        [sp.cos(theta3),  0, sp.sin(theta3), 0],
        [0,               1, 0,               0],
        [-sp.sin(theta3), 0, sp.cos(theta3), l2],
        [0,               0, 0,               1]
    ])

    # T4: Rotation along y-axis, translation along z-axis (servo No.3 to No.4)
    T4 = sp.Matrix([
        [sp.cos(theta4),  0, sp.sin(theta4), 0],
        [0,               1, 0,               0],
        [-sp.sin(theta4), 0, sp.cos(theta4), l3],
        [0,               0, 0,               1]
    ])

    # T5: Rotation about z-axis, translation along z-axis
    T5 = sp.Matrix([
        [sp.cos(theta5), -sp.sin(theta5), 0, 0],
        [sp.sin(theta5),  sp.cos(theta5), 0, 0],
        [0,              0,               1, l4],
        [0,              0,               0, 1]
    ])


    # T6: Fixed link, translation along z-axis
    T6 = sp.Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, l5],
        [0, 0, 0, 1]
    ])

    # Overall transformation matrix from base to end-effector
    T = T1 * T2 * T3 * T4 * T5 * T6
    return T

# Example: Get the transformation matrix
T = forward_kinematics()
sp.pprint(T)

values = {
        sp.symbols('theta1'): sp.rad(-174.273), 
        sp.symbols('theta2'): sp.rad(-65.5794),  
        sp.symbols('theta3'): sp.rad(69.1789), 
        sp.symbols('theta4'): sp.rad(-6.59),  
        sp.symbols('theta5'): sp.rad(174.273), 
        sp.symbols('l1'): 0.037,            
        sp.symbols('l2'): 0.13,             
        sp.symbols('l3'): 0.13,            
        sp.symbols('l4'): 0.054,             
        sp.symbols('l5'): 0.07,             
    }

T_numeric = T.subs(values)
# Evaluate the matrix to get numerical values
T_numeric_eval = T_numeric.evalf()

print("Numerical Homogeneous Transformation Matrix:")
sp.pprint(T_numeric_eval)


