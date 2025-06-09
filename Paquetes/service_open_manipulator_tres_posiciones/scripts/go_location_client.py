#!/usr/bin/env python3
import rospy 
from service_open_manipulator_tres_posiciones.srv import go_location_manipulator, go_location_manipulatorRequest


####################################################################################

class client_open_manipulator(object):
    def __init__(self):
        rospy.init_node('manipulator_service_client_tres_posiciones', anonymous=True)
        rospy.loginfo("🚀 Nodo 'manipulator_service_client' iniciado...")
        rospy.loginfo('Wainting for service....')
        rospy.wait_for_service('/gamora/go_location_manipulator')
        print ("hola, ya reconocí el servidor")


        #Se crea un proxy del servicio
        self.client_open_manipulator= rospy.ServiceProxy('/gamora/go_location_manipulator', go_location_manipulator)

        #se definen los tres destinos a los que tiene que ir el robot
        self.kinematics_pose1=[0.230, -0.166,0.204]
        self.kinematics_pose2=[0.155, 0.180,0.203]
        self.kinematics_pose3=[0.255,0.012,0.069]
        self.path_Time=2.0 
        

    def SendPositions(self):

        try:
            # Crear la solicitud de las tres posiciones
            solicitud = go_location_manipulatorRequest()
            solicitud.kinematics_pose1 = self.kinematics_pose1
            solicitud.kinematics_pose2 = self.kinematics_pose2
            solicitud.kinematics_pose3 = self.kinematics_pose3
            solicitud.path_time = self.path_Time


            # Llamar al servicio
            respuesta = self.client_open_manipulator(solicitud)
            rospy.loginfo(f"Respuesta del servidor: {respuesta.ok}")

            #verifica si la rrspuesta fue true
            if respuesta.ok == True:
                rospy.loginfo("Las tres respuestas fueron enviadas exitosamente.")
            else:
                rospy.logwarn(f"Error al enviar los arreglos. Respuesta: {respuesta.ok}")


        except rospy.ServiceException as e:
            rospy.logerr(f"Error al llamar al servicio: {e}")


##########################################################
if __name__ == '__main__':
    client=client_open_manipulator()
    client.SendPositions()