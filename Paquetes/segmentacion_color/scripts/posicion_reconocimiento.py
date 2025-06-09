#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point  # Tipo de mensaje para coordenadas
from sensor_msgs.msg import JointState, Image
from open_manipulator_msgs.srv import (
    SetActuatorState, SetActuatorStateRequest, 
    SetJointPosition, SetJointPositionRequest, 
    SetKinematicsPose, SetKinematicsPoseRequest)
from open_manipulator_msgs.msg import JointPosition, KinematicsPose, OpenManipulatorState

class movimiento_pos_reconocimiento:
    def setJointSpacePath(self, joint_name, joint_angle, path_time):
        try:
            srv = SetJointPositionRequest()
            srv.joint_position.joint_name = joint_name
            srv.joint_position.position = joint_angle
            srv.path_time = path_time
            response = self.set_joint_position_client(srv)
            return response.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
#######################################################################################
    def pos_reco(self):
        kinematics_pose = [0.2300, 0.0000, 0.0793]
        print(f"{kinematics_pose}")
        #print(f"{kinematics_pose}")
        #l gripper angle siempre abierto
        path_time=2.0
        if not self.setTaskSpacePath(kinematics_pose, path_time):
            print("[ERR!!] Failed to send joint angles")
            return
        print("Send task pose + gripper position")

    def setToolControl(self, joint_angle):
        try:
            srv = SetJointPositionRequest()
            srv.joint_position.joint_name.append("gripper")
            srv.joint_position.position = joint_angle
            response = self.set_tool_control_client(srv)
            return response.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def pos_inicial(self):
        joint_angle = 0.01
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle = [0.000, 0.000 , 0.000, 1.456]  # Ángulos de la posición inicial
        #joint_angle = [0.000, 0.040, 0.410, 0.800 ]  # Ángulos modificados para posición de reset
        path_time = 2.0  # Tiempo de movimiento
        success = self.setJointSpacePath(joint_name, joint_angle, path_time)
        joint_angle = [0.01]  # Ángulo para cerrar gripper

        if not self.setToolControl(joint_angle):
            print("[ERR!!] Failed to send service")
            return
        
    def setActuatorState(self, actuator_state):
        try:
            srv = SetActuatorStateRequest()
            srv.set_actuator_state = actuator_state
            response = self.set_actuator_state_client(srv)
            return response.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        
    def __init__(self):
        self.kinematics_pose = None
        self.present_kinematic_position = [0.0, 0.0, 0.0]  # Inicialización segura
        self.open_manipulator_is_moving = False
        self.open_manipulator_actuator_enabled = False
        rospy.init_node('nodo_publicador_pos_reconocimiento', anonymous=True)
        self.joint_state_subscriber = rospy.Subscriber('/gamora/joint_states', JointState, self.joint_states_callback)
        self.open_manipulator_states_sub = rospy.Subscriber("/gamora/states", OpenManipulatorState, self.manipulator_states_callback)
        self.set_actuator_state_client = rospy.ServiceProxy('/gamora/set_actuator_state', SetActuatorState)
        self.set_joint_position_client = rospy.ServiceProxy('/gamora/goal_joint_space_path', SetJointPosition)
        self.set_tool_control_client = rospy.ServiceProxy('/gamora/goal_tool_control', SetJointPosition)
        self.goal_task_space_path_position_only_client = rospy.ServiceProxy('/gamora/goal_task_space_path_position_only', SetKinematicsPose)
        self.open_manipulator_states_sub = rospy.Subscriber("/gamora/states", OpenManipulatorState, self.manipulator_states_callback)

    def joint_states_callback(self, msg):
        temp_angle = [0.0] * 5
        for i in range(len(msg.name)):
            if msg.name[i] == "joint1":
                temp_angle[0] = msg.position[i]
            elif msg.name[i] == "joint2":
                temp_angle[1] = msg.position[i]
            elif msg.name[i] == "joint3":
                temp_angle[2] = msg.position[i]
            elif msg.name[i] == "joint4":
                temp_angle[3] = msg.position[i]
            elif msg.name[i] == "gripper":
                temp_angle[4] = msg.position[i]
        self.present_joint_angle = temp_angle

    def get_open_manipulator_moving_state(self):
        return self.open_manipulator_is_moving

    def get_open_manipulator_actuator_state(self):
        return self.open_manipulator_actuator_enabled

    def manipulator_states_callback(self, msg):
        self.open_manipulator_is_moving = msg.open_manipulator_moving_state == msg.IS_MOVING
        self.open_manipulator_actuator_enabled = msg.open_manipulator_actuator_state == msg.ACTUATOR_ENABLED
        
    def publicar_mensaje(self):
        if not self.setActuatorState(True):  # Llamada para habilitar los actuadores
            print("No se hbilitaron los actuadores")
            return
        self.pos_inicial()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        nodo = movimiento_pos_reconocimiento()
        nodo.publicar_mensaje()
    except rospy.ROSInterruptException:
        pass