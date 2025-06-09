#!/usr/bin/env python3
#SOLO LO LLEVA A LA POSICION DE RECONOCIMIENTO
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point  # Tipo de mensaje para coordenadas
from sensor_msgs.msg import JointState, Image
from open_manipulator_msgs.srv import (
    SetActuatorState, SetActuatorStateRequest, 
    SetJointPosition, SetJointPositionRequest, 
    SetKinematicsPose, SetKinematicsPoseRequest)
from open_manipulator_msgs.msg import JointPosition, KinematicsPose, OpenManipulatorState

class movimiento_pos_reconocimiento:
    
    present_kinematic_position = None  # Inicialización segura
    def setTaskSpacePath(self, kinematics_pose, path_time):
        """
        Envía una solicitud para mover el efector final en el espacio cartesiano.
        :param kinematics_pose: Lista con las coordenadas [x, y, z] de destino.
        :param path_time: Tiempo de ejecución.
        :return: True si la planificación fue exitosa, False si falló.
        """
        print("entra a la funcion de cartesiana")
        try:
            srv = SetKinematicsPoseRequest()
            srv.end_effector_name = "gripper"
            srv.kinematics_pose.pose.position.x = kinematics_pose[0]
            srv.kinematics_pose.pose.position.y = kinematics_pose[1]
            srv.kinematics_pose.pose.position.z = kinematics_pose[2]

            print(f" si le llegan las coordenadas{kinematics_pose}")
            # Mantener la orientación actual del efector final
            srv.kinematics_pose.pose.orientation = self.kinematics_pose.orientation

            srv.path_time = path_time
            response = self.goal_task_space_path_position_only_client(srv)
            return response.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def setJointSpacePath(self, joint_name, joint_angle, path_time):
        """
        Envía una solicitud para mover el manipulador en el espacio articular.
        :param joint_name: Lista con nombres de articulaciones.
        :param joint_angle: Lista con ángulos de destino.
        :param path_time: Tiempo de ejecución.
        :return: True si el servicio fue exitoso, False si falló.
        """
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

    def kinematics_pose_callback(self, msg):
        """
        Callback para recibir mensajes de KinematicsPose.
        Extrae la posición y actualiza el estado.
        """
        temp_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.present_kinematic_position = temp_position
        self.kinematics_pose = msg.pose

    def getPresentKinematicsPose(self):
        """ Retorna la posición cinemática actual. """
        return self.present_kinematic_position

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
        """
        Controla la herramienta (gripper).
        :param joint_angle: Lista con la posición deseada del gripper.
        :return: True si el servicio fue exitoso, False si falló.
        """
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
        """
        Función para manejar el clic del botón "Posición Inicial".
        """
        joint_angle = 0.01
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle = [0.000, 0.000 , 0.000, 1.456]  # Ángulos de la posición inicial
        path_time = 2.0  # Tiempo de movimiento
        success = self.setJointSpacePath(joint_name, joint_angle, path_time)
        joint_angle = [0.01]  # Ángulo para abrir gripper
        joint_angle=[0.01]
        gripper_closed = self.setToolControl(joint_angle) #AQUI AGARRA EL OBJETO

        if success ==True:
            self.logrado = True
            while True:
                if not rospy.is_shutdown():
                    self.publicador_cumplimiento_posicion_reco.publish(self.logrado)

        if not self.setToolControl(joint_angle):
            print("[ERR!!] Failed to send service")
            return
        
    def pos_reco(self):
        kinematics_pose = [0.23, 0.00, 0.0793]
        print(f"{kinematics_pose}")
        path_time=2.0
        success1 = self.setTaskSpacePath(kinematics_pose, path_time)
        
    def setActuatorState(self, actuator_state):
        """
        Activa o desactiva los actuadores del manipulador.
        :param actuator_state: True para habilitar, False para deshabilitar.
        :return: True si fue exitoso, False si falló.
        """
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
        self.process_sim = None  # Para el proceso de la simulación
        self.process_real = None  # Para el proceso real


        #Ejecución de movimiento del robot
        rospy.init_node('nodo_publicador_pos_reconocimiento', anonymous=True)
        self.joint_state_subscriber = rospy.Subscriber('/gamora/joint_states', JointState, self.joint_states_callback)
        self.kinematics_pose_subscriber = rospy.Subscriber('/gamora/gripper/kinematics_pose', KinematicsPose, self.kinematics_pose_callback)
        self.open_manipulator_states_sub = rospy.Subscriber("/gamora/states", OpenManipulatorState, self.manipulator_states_callback)
        #Ejecución de movimiento del robot
        self.set_actuator_state_client = rospy.ServiceProxy('/gamora/set_actuator_state', SetActuatorState)
        self.set_joint_position_client = rospy.ServiceProxy('/gamora/goal_joint_space_path', SetJointPosition)
        self.set_tool_control_client = rospy.ServiceProxy('/gamora/goal_tool_control', SetJointPosition)
        self.goal_task_space_path_position_only_client = rospy.ServiceProxy('/gamora/goal_task_space_path_position_only', SetKinematicsPose)
        self.open_manipulator_states_sub = rospy.Subscriber("/gamora/states", OpenManipulatorState, self.manipulator_states_callback)
        self.publicador_cumplimiento_posicion_reco = rospy.Publisher('/gamora/cumplimiento_posicion_reco', Bool, queue_size=10,latch=True)

    def joint_states_callback(self, msg):
        """
        Callback para recibir mensajes de /joint_states.
        Extrae las posiciones de las articulaciones y actualiza el estado.
        """
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
        """ Retorna si el manipulador está en movimiento. """
        return self.open_manipulator_is_moving

    def get_open_manipulator_actuator_state(self):
        """ Retorna si el actuador del manipulador está habilitado. """
        return self.open_manipulator_actuator_enabled

    def manipulator_states_callback(self, msg):
        """
        Callback para recibir el estado del manipulador.
        """
        self.open_manipulator_is_moving = msg.open_manipulator_moving_state == msg.IS_MOVING
        self.open_manipulator_actuator_enabled = msg.open_manipulator_actuator_state == msg.ACTUATOR_ENABLED
        

    def publicar_mensaje(self):
        position = self.getPresentKinematicsPose()
        if not self.setActuatorState(True):  # Llamada para habilitar los actuadores
            print("No se hbilitaron los actuadores")
            return
    
        self.pos_inicial()

    def run(self):
        """ Mantiene el nodo ROS en ejecución. """
        rospy.spin()

if __name__ == "__main__":
    try:
        nodo = movimiento_pos_reconocimiento()
        nodo.publicar_mensaje()
    except rospy.ROSInterruptException:
        pass