import numpy as np
from basic_robotics.general import tm, fsr, Wrench
from basic_robotics.kinematics import Arm
from basic_robotics.plotting.vis_matplotlib import plt, DrawArm

fig = plt.figure()
ax = plt.axes(projection = '3d')
ax.set_xlim3d(-0.5, 0.5)
ax.set_ylim3d(-0.5, 0.5)
ax.set_zlim3d(-0.5, 0.5)

Base_T = tm() # Set a transformation for the base

# Define some link lengths

in_2_m = 0.0254
link_length_m = in_2_m*12
L1 = link_length_m
L2 = link_length_m
L3 = in_2_m*3
W = 0.01
# Define the transformations of each link
basic_arm_end_effector_home = fsr.TAAtoTM(np.array([[L1+L2+L3],[0],[0],[0],[0],[0]]))

basic_arm_joint_axes = np.array([[0, 0, 1], [0, 0, 1], [0, 0, 1]]).conj().T
basic_arm_joint_homes = np.array([[0, 0, 0], [L1, 0, 0], [L1+L2, 0, 0]]).conj().T
basic_arm_screw_list = np.zeros((6,3))

# Create the screw list
for i in range(0,3):
    basic_arm_screw_list[0:6,i] = np.hstack((basic_arm_joint_axes[0:3, i], np.cross(basic_arm_joint_homes[0:3, i], basic_arm_joint_axes[0:3, i])))

# Create the arm from the above parameters
arm = Arm(Base_T, basic_arm_screw_list, basic_arm_end_effector_home, basic_arm_joint_homes, basic_arm_joint_axes)

num_intervals = 10
theta_range = np.linspace(-np.pi, np.pi, num_intervals)
tau_range = np.linspace(-2.4, 2.4, num_intervals)

theta = -np.pi*np.ones(3)
tau = -2.4*np.ones(3)
force_list = np.array([])
tau_list = np.array([])
theta_list = np.array([])
for i in range(num_intervals):
    theta[0] = theta_range[i]
    for j in range(num_intervals):
        theta[1] = theta_range[j]
        for k in range(num_intervals):
            theta[2] = theta_range[k]
            for l in range(num_intervals):
                tau[0] = tau_range[l]
                for m in range(num_intervals):
                    tau[1] = tau_range[m]
                    for n in range(num_intervals):
                        tau[2] = tau_range[n]
                        tip_force = np.linalg.norm(arm.staticForcesInv(tau, theta).getForce())
                        if tip_force > 40:
                            force_list = np.append(force_list, tip_force)
                            tau_list = np.append(tau_list, tau)
                            theta_list = np.append(theta_list, theta)

np.savez('tip_forces_12_12_3', force_list, tau_list, theta_list)






