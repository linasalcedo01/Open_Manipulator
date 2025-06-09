#!/usr/bin/env python3
import os
import subprocess
import sys
import re
import rospy
import requests
import json
class verificar_paquetes_actividadguiada1(object):
    def __init__(self):
        try:
            rospy.init_node('verificador_ejercicio_propuesto_sesion1', anonymous=True)
            self.token = rospy.get_param('~token', 'default_value') 
            #rospy.loginfo(f"Token recibido: {self.token}")
            url = "http://192.168.0.106/competencies/control-panel/activity-attempt/update/"
            #url = "http://192.168.18.13:5000/update"
            self.package_name = 'service_open_manipulator_tres_posiciones'  #colocar aquí el nombre del paquete a analizar :v
            self.PAQUETE = 0.0
            self.cal_existencia_paquete = self.existencia_paquete()
            self.cal_existencia_directorio_launch = self.existencia_directorio_launch()
            self.cal_existencia_archivos_launch_py = self.existencia_archivo_launch_py()
            self.cal_permisos_ejecucion_py = self.permisos_ejecucion_py()
            self.cal_existencia_servicio_golocation = self.existencia_servicio_corriendo()
            self.cal_existencia_servicio_ros=self.existencia_servicio_ros()
            self.cal_existencia_mensaje_srv = self.existencia_mensaje_srv()
            
            

            self.PAQUETE = (self.cal_existencia_paquete + self.cal_existencia_directorio_launch + self.cal_existencia_archivos_launch_py + self.cal_permisos_ejecucion_py + self.cal_existencia_servicio_golocation + self.cal_existencia_servicio_ros + self.cal_existencia_mensaje_srv)
            print(f"La calificación final es {self.PAQUETE}%")


            data = {
                "evaluaciones": [
                    {
                        "index": 1,
                        "name": "Creación del paquete",
                        "percentage_grade": self.cal_existencia_paquete
                    },
                    {
                        "index": 2,
                        "name": "Existencia del directorio Launch",
                        "percentage_grade":  self.cal_existencia_directorio_launch
                    },
                    {
                        "index": 3,
                        "name": "Existencia del archivo launch, existencia del script de Python",
                        "percentage_grade": self.cal_existencia_archivos_launch_py
                    },
                    {
                        "index": 4,
                        "name": "Permisos de ejecución del script de Python",
                        "percentage_grade": self.cal_permisos_ejecucion_py
                    },
                    {
                        "index": 5,
                        "name": "Existencia del servicio /go_location_manipulator",
                        "percentage_grade": self.cal_existencia_servicio_ros
                    },
                    {
                        "index": 6,
                        "name": "Existencia del servicio en ROS",
                        "percentage_grade": self.cal_existencia_servicio_golocation
                    },
                    {
                        "index": 7,
                        "name": "Existencia del mensaje personalizado del servicio usando coordenadas XYZ expresadas en flotantes",
                        "percentage_grade": self.cal_existencia_mensaje_srv
                    },
                    {
                        "index": 8,
                        "name": "Calificación final:",
                        "percentage_grade": self.PAQUETE
                    }],
                "token": {"PIN": self.token}
                }
            response=requests.put(url, json=data)
            print(f"Status Code: {response.status_code}")
            print(f"Response Body: {response.text}")
            print("FINALIZADO")
            
            
        except Exception as e:
            print(f"Error en la inicialización: {e}")
            sys.exit(1)  

    def existencia_paquete(self): #aqui falta que verifique las dependencias y todo lo que tiene que tener el paquete
        try:
            package_dir = subprocess.check_output(["rospack", "find", self.package_name], stderr=subprocess.STDOUT).decode().strip()
            package_xml_path = os.path.join(package_dir, "package.xml")
            cmakelists_path = os.path.join(package_dir, "CMakeLists.txt")
            if os.path.exists(package_xml_path) and os.path.exists(cmakelists_path):
                existen_depend = subprocess.check_output(["rospack", "depends", self.package_name], stderr=subprocess.STDOUT).decode().splitlines()
                if "message_generation" in existen_depend and "rospy" in existen_depend and "std_msgs" in existen_depend:
                    self.cal_existencia_paquete= 7
                    print("1.Creación del paquete, (7%)")
                else:
                    print("1.Creación del paquete, (0%)")
                    self.cal_existencia_paquete= 0
        except subprocess.CalledProcessError:
            self.cal_existencia_paquete= 0
            print("1.Creación del paquete, (0%)")
        return self.cal_existencia_paquete


    def existencia_directorio_launch(self):
        try:
            package_dir = subprocess.check_output(["rospack", "find", self.package_name], stderr=subprocess.STDOUT).decode().strip()
            launch_dir_path = os.path.join(package_dir, "launch")
            if os.path.exists(launch_dir_path) and os.path.isdir(launch_dir_path):
                self.cal_existencia_directorio_launch= 7
                print("2.Existencia del directorio Launch, (7%)")
            else:
                print("2.Existencia del directorio Launch, (0%)")
                self.cal_existencia_directorio_launch= 0
        except:
                print("2.Existencia del directorio Launch, (0%)")
                self.cal_existencia_directorio_launch= 0
        return self.cal_existencia_directorio_launch

    def existencia_archivo_launch_py(self):
        self.existe_archivo_lauch= False
        self.existe_archivo_py=False
        try:
            package_dir = subprocess.check_output(["rospack", "find", self.package_name], stderr=subprocess.STDOUT).decode().strip()
            launch_dir_path = os.path.join(package_dir, "launch")
            if os.path.exists(launch_dir_path) and os.path.isdir(launch_dir_path):
                if os.path.isdir(launch_dir_path):
                    if "server_manipulator_launch.launch" in os.listdir(launch_dir_path):
                        self.existe_archivo_launch= True
                    else:
                       
                        self.existe_archivo_launch= False
                        
            scripts_dir_path = os.path.join(package_dir, "scripts")

            if os.path.exists(scripts_dir_path) and os.path.isdir(scripts_dir_path):
                if os.path.isdir(scripts_dir_path):
                    if "go_location_server.py" in os.listdir(scripts_dir_path) and "go_location_client.py" in os.listdir(scripts_dir_path):
                        self.existe_archivo_py=True
                    else:
                      
                        self.existe_archivo_py=False

            if (self.existe_archivo_py==True and self.existe_archivo_launch==True):
                self.cal_existencia_archivos_launch_py= 7
                print("3.existencia del archivo launch, existencia del script de Python, (7 %)")
            else:
                print("3.existencia del archivo launch, existencia del script de Python, (0 %)")
                self.cal_existencia_archivos_launch_py= 0
        except:
                print("3.existencia del archivo launch, existencia del script de Python, (0 %)")
                self.cal_existencia_archivos_launch_py= 0
        return self.cal_existencia_archivos_launch_py

    def permisos_ejecucion_py(self):
        self.existe_archivo_lauch= False
        self.existe_archivo_py=False
        self.launch_ejecutable = False
        self.server_py_ejecutable=False
        self.client_py_ejecutable=False
        try:
            package_dir = subprocess.check_output(["rospack", "find", self.package_name], stderr=subprocess.STDOUT).decode().strip()
            launch_dir_path = os.path.join(package_dir, "launch")
            if os.path.exists(launch_dir_path) and os.path.isdir(launch_dir_path):
                if os.path.isdir(launch_dir_path):
                    if "server_manipulator_launch.launch" in os.listdir(launch_dir_path):
                        self.existe_archivo_launch= True
                        launch_path = os.path.join(launch_dir_path, "server_manipulator_launch.launch")
                        if os.access(launch_path, os.X_OK):
                            self.launch_ejecutable = True
                        else:
                            self.launch_ejecutable = False
                    else:
                       
                        self.existe_archivo_launch= False
                        
            scripts_dir_path = os.path.join(package_dir, "scripts")

            if os.path.exists(scripts_dir_path) and os.path.isdir(scripts_dir_path):
                if os.path.isdir(scripts_dir_path):
                    if "go_location_server.py" in os.listdir(scripts_dir_path) and "go_location_client.py" in os.listdir(scripts_dir_path):
                        self.existe_archivo_py=True
                        server_path = os.path.join(scripts_dir_path, "go_location_server.py")
                        if os.access(server_path, os.X_OK):
                            self.server_py_ejecutable = True
                        else:
                            self.server_py_ejecutable = False
                        client_path = os.path.join(scripts_dir_path, "go_location_server.py")
                        if os.access(client_path, os.X_OK):
                            self.client_py_ejecutable = True
                        else:
                            self.client_py_ejecutable = False
                    else:
                      
                        self.existe_archivo_py=False

            if (self.server_py_ejecutable==True and self.client_py_ejecutable==True and self.launch_ejecutable==True):
                self.cal_permisos_ejecucion_py= 7
                print("4. permisos de ejecución del script de Python, (7 %)")
            else:
                print("4. permisos de ejecución del script de Python, (0 %)")
                self.cal_permisos_ejecucion_py= 0
        except:
                print("4. permisos de ejecución del script de Python, (0 %)")
                self.cal_permisos_ejecucion_py= 0
        return self.cal_permisos_ejecucion_py

    def existencia_servicio_corriendo(self):
        servicio_nombre= '/gamora/go_location_manipulator'
        try:
            servicios = subprocess.check_output(["rosservice", "list"]).decode().split("\n")
            if servicio_nombre in servicios:
                self.cal_existencia_servicio_golocation= 14
                print("6.existencia del servicio /go_location_manipulator, (14 %)")
            else:
                print("6.existencia del servicio /go_location_manipulator, (0 %)")
                self.cal_existencia_servicio_golocation= 0
        except:
                print("6.existencia del servicio /go_location_manipulator, (0 %)")
                self.cal_existencia_servicio_golocation= 0
        return self.cal_existencia_servicio_golocation
    
    def existencia_servicio_ros(self):
        servicio_nombre= '/gamora/go_location_manipulator'
        try:
            package_dir = subprocess.check_output(["rospack", "find", self.package_name], stderr=subprocess.STDOUT).decode().strip()
            srv_dir_path = os.path.join(package_dir, "srv")
            if os.path.exists(srv_dir_path) and os.path.isdir(srv_dir_path):
                if os.path.isdir(srv_dir_path):
                    if "go_location_manipulator.srv" in os.listdir(srv_dir_path):
                        self.cal_existencia_servicio_ros= 7
                        print("5.existencia de la acción en ROS, (7%)")
                else:
                    print("5.existencia de la acción en ROS, (0%)")
                    self.cal_existencia_servicio_ros= 0
        except:
                print("5.existencia de la acción en ROS, (0%)")
                self.cal_existencia_servicio_ros= 0
        return self.cal_existencia_servicio_ros
                       
                        
    
    def existencia_mensaje_srv(self):
        servicio_nombre= '/gamora/go_location_manipulator'
        try: 
            servicios = subprocess.check_output(["rosservice", "list"]).decode().split("\n")
            if servicio_nombre in servicios:
                service_type = subprocess.check_output(["rosservice", "type", servicio_nombre]).decode().strip()
                service_details = subprocess.check_output(["rossrv", "show", service_type]).decode()
                if ("float64[] kinematics_pose1" in service_details and "float64[] kinematics_pose2" in service_details and "float64[] kinematics_pose3" in service_details and "float64 path_time" in service_details and "---" in service_details and "bool ok" in service_details):
                    self.cal_existencia_mensaje_srv= 51
                    print("7.existencia del mensaje personalizado del servicio usando coordenadas XYZ expresadas en flotantes, (51 %)")
                else:
                    self.cal_existencia_mensaje_srv= 0
                    print("7.existencia del mensaje personalizado del servicio usando coordenadas XYZ expresadas en flotantes, (0%)")
            else:
                self.cal_existencia_mensaje_srv= 0
                print("7.existencia del mensaje personalizado del servicio usando coordenadas XYZ expresadas en flotantes, (0%)")
            

        except:
                print("existencia del mensaje personalizado del servicio usando coordenadas XYZ expresadas en flotantes, (0 %)")
                self.cal_existencia_mensaje_srv= 0
        return self.cal_existencia_mensaje_srv
               
#######################################################################################################################################3

# Comprobación de argumentos
if __name__ == "__main__":
    verificacion_actividadguiada1 = verificar_paquetes_actividadguiada1()
    rospy.spin()  # Mantener el nodo activo en ROS