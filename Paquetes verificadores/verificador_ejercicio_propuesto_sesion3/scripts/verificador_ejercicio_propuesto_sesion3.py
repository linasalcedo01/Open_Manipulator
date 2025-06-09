#!/usr/bin/env python3
import os
import subprocess
import sys
import re
import rospy
from sensor_msgs.msg import JointState, Image
from open_manipulator_msgs.srv import (
    SetActuatorState, SetActuatorStateRequest, 
    SetJointPosition, SetJointPositionRequest, 
    SetKinematicsPose, SetKinematicsPoseRequest)
from open_manipulator_msgs.msg import JointPosition, KinematicsPose, OpenManipulatorState

class verificador_ejercicio_propuesto_SESION2(object):

    def __init__(self):
        try:
            rospy.init_node('verificador_ejercicio_propuesto_sesion2', anonymous=True)
            self.cumplimiento_posicion1=None
            self.cumplimiento_posicion2=None
            self.cumplimiento_posicion1_denegada=None
            self.cumplimiento_posicion2_denegada=None
            print("\n_________________________VERIFICADOR EJERCICIO PROPUESTO SESION 3___________________")
            print("\nEste paquete verificador necesita que ingreses la POSICION 1 y POSICION 2 en donde tu robot almacenará los objetos")
            while True:
                respuesta1 = input("POSICIÓN 1: Ingresa la coordenada x,y,z separados por comas: ").strip()
                partes1 = respuesta1.split(',')

                if len(partes1) != 3:
                    print("❌ Debes ingresar exactamente tres valores separados por comas.")
                    continue
                try:
                    arreglo1 = [float(p.strip()) for p in partes1]
                    print("Posición 1 de estantería:", arreglo1)
                except ValueError:
                    print("❌ Todos los valores deben ser números.")
                    continue

                while True:
                    respuesta2 = input("POSICIÓN 2: Ingresa la coordenada x,y,z separados por comas:").strip()
                    partes2 = respuesta2.split(',')

                    if len(partes2) != 3:
                        print("❌ Debes ingresar exactamente tres valores separados por comas.")
                        continue
                    try:
                        arreglo2 = [float(p.strip()) for p in partes2]
                        print("Posición 2 de estantería:", arreglo2)
                        break  # Salimos del segundo while si todo está bien
                    except ValueError:
                        print("❌ Todos los valores deben ser números.")
                        continue

                break  # Salimos del primer while después de que los dos estén bien
            self.kinematics_pose_subscriber = rospy.Subscriber('/gamora/gripper/kinematics_pose', KinematicsPose, self.kinematics_pose_callback)
            self.package_name = 'tracking_dos_objetos'  #colocar aquí el nombre del paquete a analizar :v
            self.PAQUETE = 0.0
            #self.position_deseada1=[0.13, 0.15, 0.07]
            #self.position_deseada2=[0.13, -0.15, 0.07]
            self.position_deseada1=arreglo1
            self.position_deseada2=arreglo2
            arreglo1= None
            arreglo2=None
            self.cal_existencia_paquete = self.existencia_paquete()
            self.cal_existencia_directorio_launch = self.existencia_directorio_launch()
            self.cal_existencia_archivos_launch_py = self.existencia_archivo_launch_py()
            self.cal_permisos_ejecucion_py = self.permisos_ejecucion_py()
            print("\n_____________________________PUNTO ESPECIAL______________________")
            print("\n5.Para el punto 5 se verificará que los objetos se trasladan a coordenadas del espacio de trabajo que escribiste. Es decir, POSICIÓN 1, y POSICIÓN 2. \n Esto se hará monitoreando la posición del efector final")
            print("  5.1 Para verificar cumplimiento lanza tu paquete del ejercicio propuesto")
            respuesta = input("  5.2 Si el robot ya terminó de ejecutar tu paquete, Escribe ok para conocer tu calificación final:").strip().lower()
            while respuesta != "ok": 
                print("comando inválido")
                respuesta = input("Escribe ok para conocer tu calificación final en tiempo real: ").strip().lower()
            if respuesta == 'ok':
                self.cal_cumplimiento_traslado_objetos = self.cumplimiento_traslado_objetos()
            
        except Exception as e:
                print(f"Error en la inicialización: {e}")
                sys.exit(1)   

    def kinematics_pose_callback(self, msg):
        """
        Callback para recibir mensajes de KinematicsPose.
        Extrae la posición y actualiza el estado.
        """
        temp_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.present_kinematic_position = temp_position
        self.kinematics_pose = msg.pose
        #############################################
        tolerancia = 0.01 
    
        x_pos1_deseada_min=self.position_deseada1[0]+tolerancia
        y_pos1_deseada_min=self.position_deseada1[1]+tolerancia
        z_pos1_deseada_min=self.position_deseada1[2]+tolerancia
        x_pos1_deseada_max=self.position_deseada1[0]-tolerancia
        y_pos1_deseada_max=self.position_deseada1[1]-tolerancia
        z_pos1_deseada_max=self.position_deseada1[2]-tolerancia

        x_pos2_deseada_min=self.position_deseada2[0]+tolerancia
        y_pos2_deseada_min=self.position_deseada2[1]+tolerancia
        z_pos2_deseada_min=self.position_deseada2[2]+tolerancia
        x_pos2_deseada_max=self.position_deseada2[0]-tolerancia
        y_pos2_deseada_max=self.position_deseada2[1]-tolerancia
        z_pos2_deseada_max=self.position_deseada2[2]-tolerancia
        #print("Ahora, ya puedes ejecutar tu paquete para monitoriar el efector final ")
        x_robot =self.present_kinematic_position[0]
        y_robot=self.present_kinematic_position[1]
        z_robot=self.present_kinematic_position[2]
        #print(f"{x_robot,y_robot,z_robot}")
        if  x_pos1_deseada_min>x_robot>x_pos1_deseada_max and y_pos1_deseada_min>y_robot>y_pos1_deseada_max and z_pos1_deseada_min>z_robot>z_pos1_deseada_max: 
            self.cumplimiento_posicion1=True
            #print("Felicidades, el robot ha llegado a la POSICION 1 de la estantería")
        elif x_pos2_deseada_min>x_robot>x_pos2_deseada_max and y_pos2_deseada_min>y_robot>y_pos2_deseada_max and z_pos2_deseada_min>z_robot>z_pos2_deseada_max:
            #print("Felicidades, el robot ha llegado a la POSICION 2 de la estantería")
            self.cumplimiento_posicion2=True
        else:
            self.cumplimiento_posicion1_denegada=False
            self.cumplimiento_posicion2_denegada=False


    def getPresentKinematicsPose(self):
        """ Retorna la posición cinemática actual. """

        return self.present_kinematic_position

    def existencia_paquete(self): #aqui falta que verifique las dependencias y todo lo que tiene que tener el paquete
        try:
            package_dir = subprocess.check_output(["rospack", "find", self.package_name], stderr=subprocess.STDOUT).decode().strip()
            package_xml_path = os.path.join(package_dir, 'package.xml')
            cmakelists_path = os.path.join(package_dir, 'CMakeLists.txt')
            if os.path.exists(package_xml_path) and os.path.exists(cmakelists_path):
                existen_depend = subprocess.check_output(["rospack", "depends", self.package_name], stderr=subprocess.STDOUT).decode().splitlines()
                if "message_generation" in existen_depend and "rospy" in existen_depend and "std_msgs" in existen_depend:
                    cal_existencia_paquete= 0.07
                    print("_____________________________RESULTADOS PRELIMINARES______________________")
                    print("1.Creación del paquete, (7%)")
                else:
                    print("_____________________________RESULTADOS PRELIMINARES______________________")
                    print("1.Creación del paquete, (0%)")
                    cal_existencia_paquete= 0.0
        except subprocess.CalledProcessError:
            cal_existencia_paquete= 0.0
            print("_____________________________RESULTADOS PRELIMINARES______________________")
            print("1.Creación del paquete, (0%)")
        return cal_existencia_paquete

 

    def existencia_directorio_launch(self):
        try:
            package_dir = subprocess.check_output(["rospack", "find", self.package_name], stderr=subprocess.STDOUT).decode().strip()
            launch_dir_path = os.path.join(package_dir, "launch")
            if os.path.exists(launch_dir_path) and os.path.isdir(launch_dir_path):
                self.cal_existencia_directorio_launch= 0.07
                print("2.Existencia del directorio Launch, (7%)")
            else:
                print("2.Existencia del directorio Launch, (0%)")
                self.cal_existencia_directorio_launch= 0.0
        except:
                print("2.Existencia del directorio Launch, (0%)")
                self.cal_existencia_directorio_launch= 0.0
        return self.cal_existencia_directorio_launch

    def existencia_archivo_launch_py(self):
        self.existe_archivo_lauch= False
        self.existe_archivo_py=False
        try:
            package_dir = subprocess.check_output(["rospack", "find", self.package_name], stderr=subprocess.STDOUT).decode().strip()
            launch_dir_path = os.path.join(package_dir, "launch")
            if os.path.exists(launch_dir_path) and os.path.isdir(launch_dir_path):
                if os.path.isdir(launch_dir_path):
                    if "reconocimiento_dos_objetos.launch" in os.listdir(launch_dir_path) and "pick_and_place.launch" in os.listdir(launch_dir_path):
                        self.existe_archivo_launch= True
                    else:
                       
                        self.existe_archivo_launch= False
                        
            scripts_dir_path = os.path.join(package_dir, "scripts")

            if os.path.exists(scripts_dir_path) and os.path.isdir(scripts_dir_path):
                if os.path.isdir(scripts_dir_path):
                    if "reconocimiento_dos_objetos.py" in os.listdir(scripts_dir_path) and "posicion_reconocimiento.py" in os.listdir(scripts_dir_path) and "pick_and_place.py" in os.listdir(scripts_dir_path):
                        self.existe_archivo_py=True
                    else:
                      
                        self.existe_archivo_py=False

            if (self.existe_archivo_py==True and self.existe_archivo_launch==True):
                self.cal_existencia_archivos_launch_py= 0.07
                print("3.existencia del archivo launch, existencia del script de Python, (7 %)")
            else:
                print("3.existencia del archivo launch, existencia del script de Python, (0 %)")
                self.cal_existencia_archivos_launch_py= 0.0
        except:
                print("3.existencia del archivo launch, existencia del script de Python, (0 %)")
                self.cal_existencia_archivos_launch_py= 0.0
        return self.cal_existencia_archivos_launch_py

    def permisos_ejecucion_py(self):
        try:
            package_dir = subprocess.check_output(["rospack", "find", self.package_name], stderr=subprocess.STDOUT).decode().strip()
#PARA LOS LAUNCH
            launch_dir_path = os.path.join(package_dir, "launch")
            if os.path.exists(launch_dir_path) and os.path.isdir(launch_dir_path):
                if os.path.isdir(launch_dir_path):
#PARA LOS DOS LAUNCH
                    if "reconocimiento_dos_objetos.launch" in os.listdir(launch_dir_path) and "pick_and_place.launch" in os.listdir(launch_dir_path):
                        self.existe_archivo_launch= True
#PARA LAUNCH 1
                        launch_reco_dos_objetos_path = os.path.join(launch_dir_path, "reconocimiento_dos_objetos.launch")
                        if os.access(launch_reco_dos_objetos_path, os.X_OK):
                            self.launch_reco_dos_objetos_ejecutable = True
                        else:
                            self.launch_reco_dos_objetos_ejecutable = False
#PARA LAUNCH 2
                        launch_pick_And_place_path = os.path.join(launch_dir_path, "pick_and_place.launch")
                        if os.access(launch_pick_And_place_path, os.X_OK):
                            self.launch_pick_And_place_ejecutable = True
                        else:
                            self.launch_pick_And_place_ejecutable = False
                    else:
                       
                        self.existe_archivo_launch= False
# PARA LOS SCRIPTS                        
            scripts_dir_path = os.path.join(package_dir, "scripts")

            if os.path.exists(scripts_dir_path) and os.path.isdir(scripts_dir_path):
                if os.path.isdir(scripts_dir_path):
# PARA LOS DOS SCRIPT
                    if "reconocimiento_dos_objetos.py" in os.listdir(scripts_dir_path) and "posicion_reconocimiento.py" in os.listdir(scripts_dir_path) and "pick_and_place.py" in os.listdir(scripts_dir_path):
                        self.existe_archivo_py=True
#PARA SCRIPT 1
                        posreco_path = os.path.join(scripts_dir_path, "posicion_reconocimiento.py")
                        if os.access(posreco_path, os.X_OK):
                            self.posreco_py_ejecutable = True
                        else:
                            self.posreco_py_ejecutable = False
#PARA SCRIPT 2
                        reco_dos_objetos_path = os.path.join(scripts_dir_path, "reconocimiento_dos_objetos.py")
                        if os.access(reco_dos_objetos_path, os.X_OK):
                            self.reco_dos_objetos_py_ejecutable = True
                        else:
                            self.reco_dos_objetos_py_ejecutable = False
#PARA SCRIPT 3
                        pick_and_place_path = os.path.join(scripts_dir_path, "pick_and_place.py")
                        if os.access(pick_and_place_path, os.X_OK):
                            self.pick_And_place_py_ejecutable = True
                        else:
                            self.pick_And_place_py_ejecutable = False
                    else:
                      
                        self.existe_archivo_py=False
                #print(f"{self.launch_reco_dos_objetos_ejecutable, self.launch_pick_And_place_ejecutable, self.posreco_py_ejecutable, self.reco_dos_objetos_py_ejecutable, self.pick_And_place_py_ejecutable}")
                if (self.posreco_py_ejecutable==True and self.reco_dos_objetos_py_ejecutable==True and self.pick_And_place_py_ejecutable==True and self.launch_reco_dos_objetos_ejecutable==True and self.launch_pick_And_place_ejecutable==True):
                    self.cal_permisos_ejecucion_py= 0.07
                    print("4. permisos de ejecución del script de Python, (7 %)")
                else:
                    print("4. akapermisos de ejecución del script de Python, (0 %)")
                    self.cal_permisos_ejecucion_py= 0.0
            else:
                print("4. permisos de ejecución del script de Python, (0 %)")
                self.cal_permisos_ejecucion_py= 0.0
        except:
                print("4. permisos de ejecución del script de Python, (0 %)")
                self.cal_permisos_ejecucion_py= 0.0
        return self.cal_permisos_ejecucion_py


    def solicitud_prueba_objeto_rojo_derecha(self):
        respuesta = input("Ubica un objeto __ROJO__ en la parte derecha del FOV.Escribe 'ok' para continuar la verificación: ").strip().lower()
        try:
            while respuesta != "ok": 
                print("comando inválido, escribe 'ok' para continuar la verificación:")
                respuesta = input("Ubica un objeto rojo en la parte derecha del FOV.Escribe 'ok' para continuar la verificación: ").strip().lower()
            if respuesta == 'ok':
                proceso = subprocess.run(["rostopic", "echo", "-n", "1", "/gamora/objeto/Centroide"], capture_output=True, text=True, timeout=5)
                salida = proceso.stdout  # Captura la salida del comando
                x_numerico = re.search(r"x:\s*([-\d.]+)", salida)
                y_numerico = re.search(r"y:\s*([-\d.]+)", salida)
                x= float(x_numerico.group(1))
                y= float(y_numerico.group(1))
                "La resolución es 320x240 por lo que el lado izquierda del campo de visión es cuando X es menor de 160"
                if x>160:
                    print("7. Realización de la prueba Objeto rojo a la derecha, (40%)")
                    self.cal_prueba_objeto_rojo_derecha = 0.40
                else:
                    print("6. Realización de la prueba Objeto rojo a la derecha, (0%)")
                    self.cal_prueba_objeto_rojo_derecha = 0.00
            else:
                print("6. Realización de la prueba Objeto rojo a la derecha, (0%)")
                self.cal_prueba_objeto_rojo_derecha = 0.00

        except subprocess.TimeoutExpired:
            print("comando inválido, escribe 'ok' para continuar la verificación:")
            print("HAY ERROR: 6. Realización de la prueba Objeto rojo a la derecha, (0%)")
            self.cal_prueba_objeto_rojo_derecha = 0.00
                
        
            
        return self.cal_prueba_objeto_rojo_derecha

        
    def cumplimiento_traslado_objetos(self):
        if self.cumplimiento_posicion1==True and self.cumplimiento_posicion2==True:
            print("5.Traslado del robot a coordenadas del espacio de trabajo, (72%)")
            self.cal_cumplimiento_traslado_objetos=0.72
        else:
            print("5.Traslado del robot a coordenadas del espacio de trabajo, (0%)")
            self.cal_cumplimiento_traslado_objetos=0.00
        self.PAQUETE = (self.cal_existencia_paquete + self.cal_existencia_directorio_launch + self.cal_existencia_archivos_launch_py + self.cal_permisos_ejecucion_py + self.cal_cumplimiento_traslado_objetos)
        print("\n_____________________________CALIFICACIÓN FINAL______________________")
        print(f"\n La calificación final es {self.PAQUETE * 100}%")
        
#######################################################################################################################################3
if __name__ == "__main__":
    try:
        nodo = verificador_ejercicio_propuesto_SESION2()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


        #except subprocess.TimeoutExpired:
        #        print("7. Realización de la prueba Objeto verde a la izquierda, (0%)")
                

        
#        return self.cal_prueba_objeto_verde_izquierda

