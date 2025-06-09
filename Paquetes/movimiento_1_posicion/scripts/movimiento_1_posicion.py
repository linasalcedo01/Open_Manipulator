#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import (
    SetActuatorState, SetActuatorStateRequest, 
    SetJointPosition, SetJointPositionRequest, 
    SetKinematicsPose, SetKinematicsPoseRequest)
from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState

class movimiento_1_posicion():

    def __init__(self):
        ###############################################
        # POSICION DESTINO
        kinematics_pose=[0.230, -0.166,0.204]
        path_time=2.0 
        ###############################################
        #POSICION INICIAL
        #kinematics_pose=[0.230,0.0,0.204]
        ###############################################
        rospy.init_node('nodo_movimiento_1_posicion', anonymous=True)
        self.present_kinematic_position = [0.0, 0.0, 0.0]
        self.open_manipulator_actuator_enabled = False
        self.kinematics_pose_subscriber = rospy.Subscriber('/gamora/gripper/kinematics_pose', KinematicsPose, self.kinematics_pose_callback)
        self.joint_state_subscriber = rospy.Subscriber('/gamora/joint_states', JointState, self.joint_states_callback)

        self.set_actuator_state_client = rospy.ServiceProxy('/gamora/set_actuator_state', SetActuatorState)
        self.set_tool_control_client = rospy.ServiceProxy('/gamora/goal_tool_control', SetJointPosition)
        self.goal_task_space_path_position_only_client = rospy.ServiceProxy('/gamora/goal_task_space_path_position_only', SetKinematicsPose)
        self.set_joint_position_client = rospy.ServiceProxy('/gamora/goal_joint_space_path', SetJointPosition)


        if not self.setActuatorState(True):  # Llamada para habilitar los actuadores
            print("No se hbilitaron los actuadores")
            return
        else:
            print("SE HABILITARON LOS ACTUADORES")
            rospy.sleep(1)
            self.aka=self.setTaskSpacePath(kinematics_pose,path_time)
            if self.aka==True:
                print(f"¡FELICIDADES, Posición deseada lograda!")
            elif self.aka==False:
                print(f"Ups, El robot no se dirigió a la posición deseada!")

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

    def setTaskSpacePath(self, kinematics_pose, path_time):
        """
        Envía una solicitud para mover el efector final en el espacio cartesiano.
        :param kinematics_pose: Lista con las coordenadas [x, y, z] de destino.
        :param path_time: Tiempo de ejecución.
        :return: True si la planificación fue exitosa, False si falló.
        """
        try:
            srv = SetKinematicsPoseRequest()
            srv.end_effector_name = "gripper"
            srv.kinematics_pose.pose.position.x = kinematics_pose[0]
            srv.kinematics_pose.pose.position.y = kinematics_pose[1]
            srv.kinematics_pose.pose.position.z = kinematics_pose[2]

            #print(f" si le llegan las coordenadas{kinematics_pose}")
            # Mantener la orientación actual del efector final
            srv.kinematics_pose.pose.orientation = self.kinematics_pose.orientation

            srv.path_time = path_time
            response = self.goal_task_space_path_position_only_client(srv)
            return response.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        
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

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        nodo = movimiento_1_posicion()
        nodo.run()
    except rospy.ROSInterruptException:
        pass
