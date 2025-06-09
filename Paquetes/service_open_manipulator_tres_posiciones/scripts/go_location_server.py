#!/usr/bin/env python3

import rospy
import roslaunch
import subprocess, os, signal
from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import (
    SetActuatorState, SetActuatorStateRequest, 
    SetJointPosition, SetJointPositionRequest, 
    SetKinematicsPose, SetKinematicsPoseRequest
)
from open_manipulator_msgs.msg import JointPosition, KinematicsPose, OpenManipulatorState
from service_open_manipulator_tres_posiciones.srv import go_location_manipulator, go_location_manipulatorResponse

class servicio_open_manipulator(object):

    def __init__(self):
        self.present_joint_angle = None
        self.present_kinematic_position = [0.0, 0.0, 0.0]
        self.kinematics_pose = None
        self.open_manipulator_is_moving = False
        self.open_manipulator_actuator_enabled = False
        self.process_sim = None  # Para el proceso de la simulación
        self.process_real = None  # Para el proceso real

        self.joint_state_subscriber = rospy.Subscriber('/gamora/joint_states', JointState, self.joint_states_callback)
        self.kinematics_pose_subscriber = rospy.Subscriber('/gamora/gripper/kinematics_pose', KinematicsPose, self.kinematics_pose_callback)
        self.open_manipulator_states_sub = rospy.Subscriber("/gamora/states", OpenManipulatorState, self.manipulator_states_callback)

        self.go_location_manipulator = rospy.Service('/gamora/go_location_manipulator', go_location_manipulator, self.inicializaMiServicio)
        rospy.loginfo(" Servidor 'go_location_manipulator' esperando solicitudes...")
        self.set_actuator_state_client = rospy.ServiceProxy('/gamora/set_actuator_state', SetActuatorState)
        self.set_joint_position_client = rospy.ServiceProxy('/gamora/goal_joint_space_path', SetJointPosition)
        self.set_tool_control_client = rospy.ServiceProxy('/gamora/goal_tool_control', SetJointPosition)
        self.goal_task_space_path_position_only_client = rospy.ServiceProxy('/gamora/goal_task_space_path_position_only', SetKinematicsPose)

    def inicializaMiServicio(self,req):
        rospy.loginfo(f"Recibida solicitud para posición 1 con kinematics_pose1 = {req.kinematics_pose1}, para posición 2 con kinematics_pose2 = {req.kinematics_pose2}, para posición 3 con kinematics_pose ={req.kinematics_pose3}y con path_time = {req.path_time}")
        kinematics_pose=req.kinematics_pose1
        path_time =req.path_time
        success = self.setTaskSpacePath(kinematics_pose, path_time) #puede haber problema porque se almacena en sucess
        print (f"Se envió kinematics_pose1,{kinematics_pose}")  
        print(success)
        if success==True:
            print ("voya mandar otra posicion")
            rospy.sleep(path_time)  # Espera antes del siguiente
            kinematics_pose=req.kinematics_pose2
            print (f"Se envió kinematics_pose2,{kinematics_pose}")  
            print (kinematics_pose)
            path_time= 3.0
            success1 = self.setTaskSpacePath(kinematics_pose, path_time) 
            print (success1) 
        
        if success1==True:
            print ("voya mandar otra posicion")
            rospy.sleep(path_time)  # Espera antes del siguiente
            kinematics_pose=req.kinematics_pose3
            print (f"Se envió kinematics_pose2,{kinematics_pose}")  
            print (kinematics_pose)
            path_time= 4.0
            success2 = self.setTaskSpacePath(kinematics_pose, path_time) 
            print (success2) 

        response = go_location_manipulatorResponse()  # Crear instancia de respuesta
        response.ok = True  # Asignar el valor correcto a la variable 'ok'
        return response  # Retornar la respuesta correctamente
        


    def joint_states_callback(self, msg):
        """
        Callback para recibir mensajes de /gamora/joint_states.
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
        

    def joint_states_callback(self, msg):
        """ Retorna los ángulos actuales de las articulaciones. """
        return self.present_joint_angle
    
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
        
    def manipulator_states_callback(self, msg):
        """
        Callback para recibir el estado del manipulador.
        """
        self.open_manipulator_is_moving = msg.open_manipulator_moving_state == msg.IS_MOVING
        self.open_manipulator_actuator_enabled = msg.open_manipulator_actuator_state == msg.ACTUATOR_ENABLED

    def get_open_manipulator_moving_state(self):
        """ Retorna si el manipulador está en movimiento. """
        return self.open_manipulator_is_moving
    
    def get_open_manipulator_actuator_state(self):
        """ Retorna si el actuador del manipulador está habilitado. """
        return self.open_manipulator_actuator_enabled
    
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

            # Mantener la orientación actual del efector final
            srv.kinematics_pose.pose.orientation = self.kinematics_pose.orientation

            srv.path_time = path_time
            response = self.goal_task_space_path_position_only_client(srv)
            print("holisssssssssssssssssss funciona el movimiento")
            return response.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        
    def start_robot_controller_real(self):
        """Inicia el nodo del controlador del manipulador."""
        try:
            self.process = subprocess.Popen(
                ["roslaunch", "open_manipulator_controller", "open_manipulator_controller.launch", "use_platform:=true"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Importante para poder terminarlo después
            )
        except Exception as e:
            rospy.logerr(f"Error al lanzar el nodo: {e}")

    def stop_robot_controller_real(self):
        """Detiene el nodo del controlador del manipulador."""
        if self.process:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)  # Terminar el grupo de procesos
            self.process.wait()  # Esperar a que termine
            self.process = None
            print("Nodo del controlador detenido.")
        else:
            print("No hay un nodo en ejecución para detener.")

    def start_robot_controller_sim(self):
        """Inicia el nodo del controlador del manipulador."""
        try:
            self.process = subprocess.Popen(
                ["roslaunch", "open_manipulator_controller", "open_manipulator_controller.launch", "use_platform:=false"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Importante para poder terminarlo después
            )
        except Exception as e:
            rospy.logerr(f"Error al lanzar el nodo: {e}")

    def stop_robot_controller_sim(self):
        """Detiene el nodo del controlador del manipulador."""
        if self.process:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)  # Terminar el grupo de procesos
            self.process.wait()  # Esperar a que termine
            self.process = None
            print("Nodo del controlador detenido.")
        else:
            print("No hay un nodo en ejecución para detener.")
        
    def run(self):
        """ Mantiene el nodo ROS en ejecución. """
        rospy.spin()

##########################################################
def main():
    rospy.init_node('manipulator_service_server_tres_posiciones', anonymous=True)
    rospy.loginfo(" Nodo 'manipulator_service_server' iniciado...")
    
    service_object = servicio_open_manipulator()
    service_object.run()

##########################################################
if __name__ == '__main__':
    main()
   




