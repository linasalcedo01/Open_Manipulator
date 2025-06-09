#!/usr/bin/env python3
import os
import subprocess
import sys
import re
import rospy
from action_descarga_bienes_manipulador.msg import descarga_bienesActionGoal, descarga_bienesActionFeedback, descarga_bienesActionResult
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

class verificar_paquetes_actividadguiada1(object):
    def __init__(self):
        try:
            self.True_desocupado=False
            self.color=None
            self.forma=None
            self.texto_feedback=None
            self.texto_resultado=None
            self.desocupado= None
            self.peticion=None
            self.descargando=None
            self.finalizado=None
            self.finalizacion_exitosa=None
            self.package_name = 'action_descarga_bienes_manipulador'  #colocar aquí el nombre del paquete a analizar :v
            self.feedback= rospy.Subscriber('/gamora/descarga_bienes/feedback', descarga_bienesActionFeedback, self.listen_feedback)
            self.result= rospy.Subscriber('/gamora/descarga_bienes/result', descarga_bienesActionResult, self.listen_resultado)
            self.status= rospy.Subscriber('/gamora/descarga_bienes/status', GoalStatusArray, self.listen_status)

            self.goal= rospy.Subscriber('/gamora/descarga_bienes/goal', descarga_bienesActionGoal, self.listen_goal)
            self.PAQUETE = 0.0
            self.cal_existencia_paquete = self.existencia_paquete()
            self.cal_existencia_directorio_launch = self.existencia_directorio_launch()
            self.cal_existencia_archivos_launch_py = self.existencia_archivo_launch_py()
            self.cal_permisos_ejecucion_py = self.permisos_ejecucion_py()
            self.cal_existencia_action = self.existencia_action()
            self.cal_respuesta_retroalimentaciones = self.respuesta_retroalimentaciones()
            
            

            self.PAQUETE = (self.cal_existencia_paquete + self.cal_existencia_directorio_launch + self.cal_existencia_archivos_launch_py + self.cal_permisos_ejecucion_py + self.cal_existencia_action + self.cal_respuesta_retroalimentaciones)
            print(f"La calificación final es {self.PAQUETE * 100}%")
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
                    self.cal_existencia_paquete= 0.07
                    print("1.Creación del paquete, (7%)")
                else:
                    print("1.Creación del paquete, (0%)")
                    self.cal_existencia_paquete= 0.0
        except subprocess.CalledProcessError:
            self.cal_existencia_paquete= 0.0
            print("1.Creación del paquete, (0%)")
        return self.cal_existencia_paquete


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
                    if "client_descarga_bienes.launch" in os.listdir(launch_dir_path) and "server_descarga_bienes.launch" in os.listdir(launch_dir_path):
                        self.existe_archivo_launch= True
                    else:
                       
                        self.existe_archivo_launch= False
                        
            scripts_dir_path = os.path.join(package_dir, "scripts")

            if os.path.exists(scripts_dir_path) and os.path.isdir(scripts_dir_path):
                if os.path.isdir(scripts_dir_path):
                    if "client_descarga_bienes.py" in os.listdir(scripts_dir_path) and "server_descarga_bienes.py" in os.listdir(scripts_dir_path):
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
        self.existe_archivo_lauch= False
        self.existe_archivo_py=False
        self.launch_ejecutable_client = False
        self.launch_ejecutable_server =False
        self.server_py_ejecutable=False
        self.client_py_ejecutable=False
        try:
            package_dir = subprocess.check_output(["rospack", "find", self.package_name], stderr=subprocess.STDOUT).decode().strip()
            launch_dir_path = os.path.join(package_dir, "launch")
            if os.path.exists(launch_dir_path) and os.path.isdir(launch_dir_path):
                if os.path.isdir(launch_dir_path):
                    if "client_descarga_bienes.launch" in os.listdir(launch_dir_path)and "server_descarga_bienes.launch" in os.listdir(launch_dir_path):
                        self.existe_archivo_launch= True
                        client_path = os.path.join(launch_dir_path, "client_descarga_bienes.launch")
                        if os.access(client_path, os.X_OK):
                            self.launch_ejecutable_client = True
                        else:
                            self.launch_ejecutable_client = False
                        server_path = os.path.join(launch_dir_path, "server_descarga_bienes.launch")
                        if os.access(client_path, os.X_OK):
                            self.launch_ejecutable_server = True
                        else:
                            self.launch_ejecutable_server = False
                    else:
                       
                        self.existe_archivo_launch= False
                        
            scripts_dir_path = os.path.join(package_dir, "scripts")

            if os.path.exists(scripts_dir_path) and os.path.isdir(scripts_dir_path):
                if os.path.isdir(scripts_dir_path):
                    if "client_descarga_bienes.py" in os.listdir(scripts_dir_path) and "server_descarga_bienes.py" in os.listdir(scripts_dir_path):
                        self.existe_archivo_py=True
                        server_path = os.path.join(scripts_dir_path, "client_descarga_bienes.py")
                        if os.access(server_path, os.X_OK):
                            self.server_py_ejecutable = True
                        else:
                            self.server_py_ejecutable = False
                        client_path = os.path.join(scripts_dir_path, "server_descarga_bienes.py")
                        if os.access(client_path, os.X_OK):
                            self.client_py_ejecutable = True
                        else:
                            self.client_py_ejecutable = False
                    else:
                      
                        self.existe_archivo_py=False

            if (self.server_py_ejecutable==True and self.client_py_ejecutable==True and self.launch_ejecutable_client==True and self.launch_ejecutable_server==True):
                self.cal_permisos_ejecucion_py= 0.07
                print("4. permisos de ejecución del script de Python, (7 %)")
            else:
                print("4. permisos de ejecución del script de Python, (0 %)")
                self.cal_permisos_ejecucion_py= 0.0
        except:
                print("4. permisos de ejecución del script de Python, (0 %)")
                self.cal_permisos_ejecucion_py= 0.0
        return self.cal_permisos_ejecucion_py


    def existencia_action(self):
        #Se lanza antes de iniciar el cliente, DEBE LANZARSE DESPUES DEL SERVIDOR
        try:
            output = subprocess.check_output(['rostopic', 'list']).decode('utf-8')
            topic_names = output.strip().split('\n')

            action_topics = [
                '/gamora/descarga_bienes/cancel',
                '/gamora/descarga_bienes/feedback',
                '/gamora/descarga_bienes/goal',
                '/gamora/descarga_bienes/result',
                '/gamora/descarga_bienes/status'
            ]

            all_exist = all(topic in topic_names for topic in action_topics)

            if all_exist:
                print("5.Existencia de la acción /gamora/descarga_bienes.(7 %)")
                self.cal_existencia_action = 0.07
            else:
                print("5.Existencia de la acción /gamora/descarga_bienes.(0 %)")
                self.cal_existencia_action = 0.00

        except subprocess.CalledProcessError as e:
            print("existencia del mensaje personalizado del servicio usando coordenadas XYZ expresadas en flotantes, (0 %)")
            self.cal_existencia_mensaje_srv= 0.0

        return self.cal_existencia_action
    
            # Callback para /gamora/descarga_bienes/goal
    def listen_goal(self, msg):
        self.color = msg.goal.color
        self.forma = msg.goal.forma
        return self.color, self.forma
    

    # Callback para /gamora/descarga_bienes/feedback
    def listen_feedback(self, msg):
        self.texto_feedback = msg.feedback.estado_actual
        if self.texto_feedback== 'Desocupado':
            self.desocupado= True
        if self.texto_feedback== 'Descargando':
            self.descargando=True
        if 'Petición' in self.texto_feedback.split() and 'de' in self.texto_feedback.split() and 'descarga' in self.texto_feedback.split() and 'recibida' in self.texto_feedback.split():
            self.peticion = True

        return self.texto_feedback
    
    def listen_resultado(self, msg):
        self.texto_resultado = msg.result.estado_final
        if self.texto_resultado== 'Finalizado':
            self.finalizado= True
        return self.texto_resultado


    def respuesta_retroalimentaciones(self):
        respuesta = input("Por favor cuando finalice la Acción, escribe ok: ").strip().lower()
        try:
            while respuesta != "ok": 
                respuesta = input("comando inválido, escribe 'ok' para continuar la verificación:").strip().lower()
            if respuesta == 'ok':
                #print(f"{self.desocupado, self.descargando, self.finalizado, self.peticion, self.ultimo_estado}")
                if self.desocupado==True and self.descargando==True and self.finalizado==True and self.peticion==True and self.finalizacion_exitosa==True:
                    print("7. Verificación respuesta a Acción y retroalimentaciones, (65 %)")
                    self.cal_respuesta_retroalimentaciones=0.65
                else:
                    print("7. Verificación respuesta a Acción y retroalimentaciones, (0 %)")
                    self.cal_respuesta_retroalimentaciones=0.00


        except subprocess.TimeoutExpired:
            print("7. Verificación respuesta a Acción y retroalimentaciones, (0 %)")
            self.cal_respuesta_retroalimentaciones=0.00
        return self.cal_respuesta_retroalimentaciones
    
    def listen_status(self, msg):
    # Verifica si hay estados en la lista
        if msg.status_list:
            for estado in msg.status_list:
                pass
            self.ultimo_estado = msg.status_list[-1]

            if self.ultimo_estado.status == GoalStatus.SUCCEEDED:
                self.finalizacion_exitosa=True
                

#######################################################################################################################################3

def main():
    rospy.init_node('verificador_fase3p1_atencion_descarga_bienes', anonymous=True)
    
#    rospy.loginfo(" Nodo 'verificador_paquete_actividadguiada1' iniciado...")
    
    verificacion_actividadguiada1 = verificar_paquetes_actividadguiada1()
    rospy.spin()  # Mantener el nodo activo en ROS

# Comprobación de argumentos
if __name__ == "__main__":
    main()