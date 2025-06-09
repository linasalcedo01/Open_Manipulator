#!/usr/bin/env python3
import os
import subprocess
import sys
import re
import rospy
import requests
import json

class verificador_ejercicio_propuesto_SESION2(object):
    def __init__(self):
        try:
            rospy.init_node('verificador_ejercicio_propuesto_sesion2', anonymous=True)
            self.token = rospy.get_param('~token', 'default_value') 
            url = "http://192.168.0.106/competencies/control-panel/activity-attempt/update/"
            self.package_name = 'segmentacion_rojo_verde'  #colocar aquí el nombre del paquete a analizar :v
            self.PAQUETE = 0.0
            self.cal_existencia_paquete = self.existencia_paquete()
            self.cal_existencia_directorio_launch = self.existencia_directorio_launch()
            self.cal_existencia_archivos_launch_py = self.existencia_archivo_launch_py()
            self.cal_permisos_ejecucion_py = self.permisos_ejecucion_py()
            self.cal_existencia_topico_objeto_centroide = self.existencia_topico_objeto_centroide()
            self.cal_solicitud_prueba_objeto_rojo = self.solicitud_prueba_objeto_rojo_derecha()
            self.cal_solicitud_prueba_objeto_verde = self.solicitud_prueba_objeto_verde_izquierda()

            
            self.PAQUETE = (self.cal_existencia_paquete + self.cal_existencia_directorio_launch + self.cal_existencia_archivos_launch_py + self.cal_permisos_ejecucion_py + self. cal_existencia_topico_objeto_centroide + self.cal_solicitud_prueba_objeto_rojo + self.cal_solicitud_prueba_objeto_verde)
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
                        "name": "existencia del tópico /pose_obj.",
                        "percentage_grade": self. cal_existencia_topico_objeto_centroide
                    },
                    {
                        "index": 6,
                        "name": "en el momento de ubicar un objeto rojo en la parte derecha del FOV de la cámara se detectará su posición a través del tópico",
                        "percentage_grade": self.cal_solicitud_prueba_objeto_rojo
                    },
                    {
                        "index": 7,
                        "name": "en el momento de ubicar un objeto verde en la parte izquierda del FOV de la cámara se detectará su posición a través del tópico",
                        "percentage_grade": self.cal_solicitud_prueba_objeto_verde
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
            package_xml_path = os.path.join(package_dir, 'package.xml')
            cmakelists_path = os.path.join(package_dir, 'CMakeLists.txt')
            if os.path.exists(package_xml_path) and os.path.exists(cmakelists_path):
                existen_depend = subprocess.check_output(["rospack", "depends", self.package_name], stderr=subprocess.STDOUT).decode().splitlines()
                if "message_generation" in existen_depend and "rospy" in existen_depend and "std_msgs" in existen_depend:
                    cal_existencia_paquete= 7
                    print("1.Creación del paquete, (7%)")
                else:
                    print("1.Creación del paquete, (0%)")
                    cal_existencia_paquete= 0
        except subprocess.CalledProcessError:
            cal_existencia_paquete= 0
            print("1.Creación del paquete, (0%)")
        return cal_existencia_paquete

 

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
                    if "segmentacion_rojo_verde.launch" in os.listdir(launch_dir_path):
                        self.existe_archivo_launch= True
                    else:
                       
                        self.existe_archivo_launch= False
                        
            scripts_dir_path = os.path.join(package_dir, "scripts")

            if os.path.exists(scripts_dir_path) and os.path.isdir(scripts_dir_path):
                if os.path.isdir(scripts_dir_path):
                    if "segmentacion_rojo_verde.py" in os.listdir(scripts_dir_path) and "posicion_reconocimiento.py" in os.listdir(scripts_dir_path):
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
        self.segmentacion_py_ejecutable=False
        self.posreco_py_ejecutable=False
        try:
            package_dir = subprocess.check_output(["rospack", "find", self.package_name], stderr=subprocess.STDOUT).decode().strip()
            launch_dir_path = os.path.join(package_dir, "launch")
            if os.path.exists(launch_dir_path) and os.path.isdir(launch_dir_path):
                if os.path.isdir(launch_dir_path):
                    if "segmentacion_rojo_verde.launch" in os.listdir(launch_dir_path):
                        self.existe_archivo_launch= True
                        launch_path = os.path.join(launch_dir_path, "segmentacion_rojo_verde.launch")
                        if os.access(launch_path, os.X_OK):
                            self.launch_ejecutable = True
                        else:
                            self.launch_ejecutable = False
                    else:
                       
                        self.existe_archivo_launch= False
                        
            scripts_dir_path = os.path.join(package_dir, "scripts")

            if os.path.exists(scripts_dir_path) and os.path.isdir(scripts_dir_path):
                if os.path.isdir(scripts_dir_path):
                    if "segmentacion_rojo_verde.py" in os.listdir(scripts_dir_path) and "posicion_reconocimiento.py" in os.listdir(scripts_dir_path):
                        self.existe_archivo_py=True
                        server_path = os.path.join(scripts_dir_path, "segmentacion_rojo_verde.py")
                        if os.access(server_path, os.X_OK):
                            self.segmentacion_py_ejecutable = True
                        else:
                            self.segmentacion_py_ejecutable = False
                        client_path = os.path.join(scripts_dir_path, "posicion_reconocimiento.py")
                        if os.access(client_path, os.X_OK):
                            self.posreco_py_ejecutable = True
                        else:
                            self.posreco_py_ejecutable = False
                    else:
                      
                        self.existe_archivo_py=False

            if (self.segmentacion_py_ejecutable==True and self.posreco_py_ejecutable==True and self.launch_ejecutable==True):
                self.cal_permisos_ejecucion_py= 7
                print("4. permisos de ejecución del script de Python, (7 %)")
            else:
                print("4. permisos de ejecución del script de Python, (0 %)")
                self.cal_permisos_ejecucion_py= 0
        except:
                print("4. permisos de ejecución del script de Python, (0 %)")
                self.cal_permisos_ejecucion_py= 0
        return self.cal_permisos_ejecucion_py
    
    def existencia_topico_objeto_centroide(self):
        topic_name = '/gamora/objeto/Centroide'
        topics = rospy.get_published_topics()  # Obtiene todos los tópicos activos
        topic_names = [t[0] for t in topics]  # Extrae solo los nombres
        if topic_name in topic_names:
            print("5. Existencia del tópico /gamora/objeto/Centroide, (7 %)")
            self.cal_existencia_topico_objeto_centroide =7
        else:
            print("5. Existencia del tópico /gamora/objeto/Centroide, (0 %)")
            self.cal_existencia_topico_objeto_centroide =0
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
                #La resolución es 320x240 por lo que el lado izquierda del campo de visión es cuando X es menor de 160
                if x>160:
                    print("7. Realización de la prueba Objeto rojo a la derecha, (40%)")
                    self.cal_prueba_objeto_rojo_derecha = 40
                else:
                    print("6. Realización de la prueba Objeto rojo a la derecha, (0%)")
                    self.cal_prueba_objeto_rojo_derecha = 0
                    print("llega")
            else:
                print("6. Realización de la prueba Objeto rojo a la derecha, (0%)")
                self.cal_prueba_objeto_rojo_derecha = 0

        except subprocess.TimeoutExpired:
            print("comando inválido, escribe 'ok' para continuar la verificación:")
            print("HAY ERROR: 6. Realización de la prueba Objeto rojo a la derecha, (0%)")
            self.cal_prueba_objeto_rojo_derecha = 0
        return self.cal_prueba_objeto_rojo_derecha

        
    def solicitud_prueba_objeto_verde_izquierda(self):
        respuesta = input("Ubica un objeto __VERDE__ en la parte izquierda del FOV.Escribe 'ok' para continuar la verificación: ").strip().lower()
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
                #"La resolución es 320x240 por lo que el lado izquierda del campo de visión es cuando X es menor de 160"
                if x<160:
                    print("7. Realización de la prueba Objeto verde a la izquierda, (25%)")
                    self.cal_prueba_objeto_verde_izquierda = 25

                else:
                    print("7. Realización de la prueba Objeto verde a la izquierda, (0%)")
                    self.cal_prueba_objeto_verde_izquierda = 0
            else:
                print("comando inválido, escribe 'ok' para continuar la verificación:")
                print("7. Realización de la prueba Objeto verde a la izquierda, (0%)")
                self.cal_prueba_objeto_verde_izquierda = 0
        except subprocess.TimeoutExpired:
                print("7. Realización de la prueba Objeto verde a la izquierda, (0%)")
                self.cal_prueba_objeto_verde_izquierda = 0

        
        return self.cal_prueba_objeto_verde_izquierda


#######################################################################################################################################3

def main():
    rospy.init_node('verificador_ejercicio_propuesto_sesion2', anonymous=True)
    
#    rospy.loginfo(" Nodo 'verificador_paquete_actividadguiada1' iniciado...")
    
    verificacion_ejerciciopropuestoSESION2 = verificador_ejercicio_propuesto_SESION2()
    rospy.spin()  # Mantener el nodo activo en ROS

# Comprobación de argumentos
if __name__ == "__main__":
    main()