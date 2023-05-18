#! /usr/bin/env python3
import ikpy.urdf.utils
import ikpy.chain
import pathlib
import stretch_body.hello_utils as hu
from IPython import display
import ipywidgets as widgets
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt

class IKSolver():
    def __init__(self):
        iktuturdf_path = "/tmp/iktutorial/stretch.urdf"
        self.chain = ikpy.chain.Chain.from_urdf_file(iktuturdf_path)
        self.tool = 'tool_stretch_gripper'
    
    #Executes Stretch to [target]
    def goTo(self, target_point = [0.01, -0.3, 1]):
        target_orientation = ikpy.utils.geometry.rpy_matrix(0.0, 0.0, 0.0)
        pretarget_orientation = ikpy.utils.geometry.rpy_matrix(0.0, 0.0, 0.0)
        #Solve for poses
        q_init = self.get_current_configuration()
        q_mid = self.chain.inverse_kinematics(target_point, initial_position=q_init)
        q_soln = self.chain.inverse_kinematics(target_point,  initial_position=q_init)

        fig, ax = plot_utils.init_3d_figure()
        plt.xlim(-0.75, 0.2)
        plt.ylim(-0.75, 0.2)
        ax.set_zlim(0.0, 1.0)
        self.chain.plot(q_soln, ax, target=target_point, show=True)
        
        if self.isValidMove(target_point, q_soln):
            self.move_to_configuration(q_soln)
        else:
            #self.move_to_configuration(q_soln)
            #print(q_soln)
            #print(self.get_current_configuration())
            print("NO")
        
    def forwardKinematics(self):
        q = self.get_current_configuration()
        return self.chain.forward_kinematics(q)[:3, 3]
    

    def isValidMove(self, target_point, q_soln):
        return np.linalg.norm(self.chain.forward_kinematics(q_soln)[:3, 3] - target_point) < 0.01



    #Pushes commands to change the pose of the robot
    def move_to_configuration(self, q):
        if self.tool == 'tool_stretch_gripper':
          
            q_lift = q[2]
            q_arm = q[4] + q[5] + q[6] + q[7]
            q_yaw = q[8]-(np.pi/2)
            print(q_yaw)

            robot.lift.move_to(q_lift)
            robot.arm.move_to(q_arm)
            robot.end_of_arm.move_to('wrist_yaw', q_yaw)
            robot.push_command()
        elif self.tool == 'tool_stretch_dex_wrist':
            q_lift = q[2]
            q_arm = q[4] + q[5] + q[6] + q[7]
            q_yaw = q[8]
            q_pitch = q[10]
            q_roll = q[11]
            robot.lift.move_to(q_lift)
            robot.arm.move_to(q_arm)
            robot.end_of_arm.move_to('wrist_yaw', q_yaw)
            robot.end_of_arm.move_to('wrist_pitch', q_pitch)
            robot.end_of_arm.move_to('wrist_roll', q_roll)
            robot.push_command()


    #Gets the current pose of the robot
    def get_current_configuration(self):
        def bound_range(name, value):
            names = [l.name for l in self.chain.links]
            index = names.index(name)
            bounds = self.chain.links[index].bounds
            if name == "joint_wrist_yaw":
                print(value, bounds[0], bounds[1])
            return min(max(value, bounds[0]), bounds[1])

        if self.tool == 'tool_stretch_gripper':
            q_lift = bound_range('joint_lift', robot.lift.status['pos'])
            q_arml = bound_range('joint_arm_l0', robot.arm.status['pos'] / 4.0)
            q_yaw = bound_range('joint_wrist_yaw', robot.end_of_arm.status['wrist_yaw']['pos'])
            return [0.0, 0.0, q_lift, 0.0, q_arml, q_arml, q_arml, q_arml, q_yaw, 0.0, 0.0]
        elif self.tool == 'tool_stretch_dex_wrist':
            q_lift = bound_range('joint_lift', robot.lift.status['pos'])
            q_arml = bound_range('joint_arm_l0', robot.arm.status['pos'] / 4.0)
            q_yaw = bound_range('joint_wrist_yaw', robot.end_of_arm.status['wrist_yaw']['pos'])
            q_pitch = bound_range('joint_wrist_pitch', robot.end_of_arm.status['wrist_pitch']['pos'])
            q_roll = bound_range('joint_wrist_roll', robot.end_of_arm.status['wrist_roll']['pos'])
        return [0.0, 0.0, q_lift, 0.0, q_arml, q_arml, q_arml, q_arml, q_yaw, 0.0, q_pitch, q_roll, 0.0, 0.0]

ik_test = IKSolver()
print(ik_test.forwardKinematics())
robot.stow()
ik_test.goTo(target_point=[-0.01, -0.3,  1.2])
ik_test.goTo(target_point=[0.01, -0.7,  1.2])
ik_test.goTo(target_point=[-0.01, -0.3,  1.2])