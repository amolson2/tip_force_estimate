import numpy as np
from basic_robotics.general import tm, fsr, Wrench
from basic_robotics.kinematics import Arm
from basic_robotics.plotting.vis_matplotlib import plt, DrawArm
from matplotlib.pyplot import hist


file = np.load('tip_forces_12_12_3.npz')

force_list = file['arr_0']
tau_list = file['arr_1']
theta_list = file['arr_2']

tau_list = np.reshape(tau_list, (-1, 3))
theta_list = np.reshape(theta_list, (-1, 3))

print(len(force_list))

# print(tau_list[np.argmax(force_list)])
# print(theta_list[np.argmax(force_list)])

hist(force_list, bins=10)
plt.show()

# fig = plt.figure()
# ax = plt.axes(projection = '3d')
# ax.set_xlim3d(-0.5, 0.5)
# ax.set_ylim3d(-0.5, 0.5)
# ax.set_zlim3d(-0.5, 0.5)
#
# Base_T = tm() # Set a transformation for the base
#
# in_2_m = 0.0254
# link_length_m = in_2_m*10
# L1 = link_length_m
# L2 = link_length_m
# L3 = in_2_m*3
# W = 0.01
# # Define the transformations of each link
# basic_arm_end_effector_home = fsr.TAAtoTM(np.array([[L1+L2+L3],[0],[0],[0],[0],[0]]))
#
# basic_arm_joint_axes = np.array([[0, 0, 1], [0, 0, 1], [0, 0, 1]]).conj().T
# basic_arm_joint_homes = np.array([[0, 0, 0], [L1, 0, 0],[L1+L2, 0, 0]]).conj().T
# basic_arm_screw_list = np.zeros((6,3))
#
# #Create the screw list
# for i in range(0,3):
#     basic_arm_screw_list[0:6,i] = np.hstack((basic_arm_joint_axes[0:3,i],np.cross(basic_arm_joint_homes[0:3,i],basic_arm_joint_axes[0:3,i])))
#
#
# # Create the arm from the above paramters
# arm = Arm(Base_T, basic_arm_screw_list, basic_arm_end_effector_home, basic_arm_joint_homes, basic_arm_joint_axes)
#
# # Input some basic dimensions
# basic_arm_link_box_dims = np.array([[W, W, L1], [W, W, L2], [W, W, L3]]).conj().T
#
# #ALTERNATIVELY, JUST LOAD A URDF USING THE 'loadArmFromURDF' function in basic_robotics.kinematics
# arm.setJointProperties(
#         np.array([np.pi, np.pi, np.pi])* -2,
#         np.array([np.pi, np.pi, np.pi]) * 2)
#
# arm.setVisColProperties(link_dimensions = basic_arm_link_box_dims)
#
# theta = theta_list[np.argmax(force_list)]
# tau = tau_list[np.argmax(force_list)]
# print(arm.staticForcesInv(tau, theta))
# arm.FK(theta)
# DrawArm(arm, ax, jheight=0.01, jdia=0.01, axes_lens=0.1)
# plt.show()


