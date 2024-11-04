import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt
from ros_robot_controller.ros_robot_controller_sdk import Board

board = Board(device="/dev/rrc", baudrate=1000000, timeout=5)
board.enable_reception()
# print(board.pwm_servo_read_position(1))

my_chain = ikpy.chain.Chain.from_urdf_file("/home/keitheorem/torch/robot_arm.urdf", active_links_mask = [False, True, True, True, True, True, False])
max_angle = 2.0944 #i.e. 120 degrees 

target_position = [0.113, 0.001, 0.101] # Any of the target input along any axis shouldn't be zero, it will cause issues 
target_orientation = [0,0,-1.5708]

servo_angle = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode='Z')

print("The angles of each joints are : ", servo_angle)
real_frame = my_chain.forward_kinematics(servo_angle)
print("Computed position vector : %s, original position vector : %s" % (real_frame[:3, 3], target_position))
print(servo_angle[1])

servo_command = [0,0,0,0,0]
servo = [1,2,3,4,5]
for i in range(0,5):
    servo_command[i] = int(500 - ((servo_angle[servo[i]]*500)/max_angle))

board.bus_servo_set_position(5, [[1,servo_angle[1], [2,servo_angle[2]], [3,servo_angle[3]], [4,servo_angle[4]], [5,servo_angle[5]]])

# # To visualise virtually
# fig, ax = plot_utils.init_3d_figure()
# my_chain.plot(servo_angle, ax, target=target_position)
# plt.xlim(-0.5, 0.5)
# plt.ylim(-0.5, 0.5)
# plt.show()

