#!/usr/bin/env python3

import sys
from cv_bridge import CvBridge
from PyQt5.QtGui import QImage, QPixmap
from sensor_msgs.msg import Image
import rospy
import numpy as np
import os, time
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QMessageBox, QApplication
from PyQt5.QtCore import QTimer
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from qnode import QNode  # Asegúrate de importar correctamente la clase QNode

def main():
    global seleccion
    seleccion=None
    #incializar variables que necesito para la parte de reco por color
#    global reco_azul
#    global reco_rojo
#    global reco_verde
#    reco_azul=0
#    reco_rojo=0
#    reco_verde=0


    # Ruta del archivo .ui
    pkg_path = os.path.dirname(os.path.realpath(__file__))
    ui_file = os.path.join(pkg_path, '../resource/InterfazEDUARM.ui')

    # Cargar la interfaz
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = uic.loadUi(ui_file)

    # Crear el temporizador
    timer = QTimer(MainWindow)

    # Crear el objeto QNode
    qnode = QNode()

    # Conectar el botón de activación del actuador
    MainWindow.btn_actuator_enable.clicked.connect(lambda: on_btn_actuator_enable_clicked(qnode))

    # Conectar el botón de desactivación del actuador
    MainWindow.btn_actuator_disable.clicked.connect(lambda: on_btn_actuator_disable_clicked(qnode))    

    # Conectar el botón de "connect" al inicio del temporizador
    MainWindow.btn_connect_real.clicked.connect(lambda: on_btn_connect_real_clicked(timer, qnode, MainWindow))

    # Conectar el botón de "disconnect" para detener el temporizador
    MainWindow.btn_disconnect_real.clicked.connect(lambda: on_btn_disconnect_real_clicked(qnode, timer, MainWindow))

    # Conectar el botón de "connect" al inicio del temporizador
    MainWindow.btn_connect_sim.clicked.connect(lambda: on_btn_connect_sim_clicked(timer, qnode, MainWindow))

    # Conectar el botón de "disconnect" para detener el temporizador
    MainWindow.btn_disconnect_sim.clicked.connect(lambda: on_btn_disconnect_sim_clicked(qnode, timer, MainWindow))

    # Conectar el botón de posición inicial
    MainWindow.btn_pos_inicial.clicked.connect(lambda: on_btn_pos_inicial_clicked(qnode))

    # Conectar el botón de posicion reset
    MainWindow.btn_pos_reset.clicked.connect(lambda: on_btn_pos_reset_clicked(qnode))
    # Conectar el botón de cerrar gripper
    MainWindow.btn_cerrar_gripper.clicked.connect(lambda: on_btn_cerrar_gripper_clicked(qnode))

    # Conectar el botón de abrir gripper
    MainWindow.btn_abrir_gripper.clicked.connect(lambda: on_btn_abrir_gripper_clicked(qnode))

    # Conectar el botón de actualizar los angulos en los spinbox
    MainWindow.btn_act_joint_angle.clicked.connect(lambda: on_btn_act_joint_angle_clicked(MainWindow))

    # Conectar el botón de actualizar las coordenadas en los spinbox
    MainWindow.btn_act_cartesian.clicked.connect(lambda: on_btn_act_cartesian_clicked(MainWindow))

    # Conectar el botón de posición inicial
    MainWindow.btn_send_joint_angle.clicked.connect(lambda: on_btn_send_joint_angle_clicked(qnode, MainWindow))

    # Conectar el botón de posición inicial
    MainWindow.btn_send_cartesian.clicked.connect(lambda: on_btn_send_cartesian_clicked(qnode, MainWindow))

    #Conectar el botón de aplicar color para reconocimiento
    MainWindow.btn_apli_color.clicked.connect(lambda: on_btn_apli_color(qnode, MainWindow))

    #Conectar el botón de reset color
    MainWindow.btn_reset_color.clicked.connect(lambda: on_btn_reset_color(qnode, MainWindow))

    #Conectar el botón de aplicar color para reconocimiento
    MainWindow.btn_apli_forma.clicked.connect(lambda: on_btn_apli_forma(qnode, MainWindow))

    #Conectar el botón de reset color
    MainWindow.btn_reset_forma.clicked.connect(lambda: on_btn_reset_forma(qnode, MainWindow))

    #conectar la opción del Qcombobox que elige el topico del robot
    MainWindow.topicos_election.currentTextChanged.connect(lambda: namespace(qnode, MainWindow))




    # Mostrar la interfaz
    MainWindow.show()

    # Iniciar la aplicación
    sys.exit(app.exec_())

#    MainWindow.btn_reset_color.setEnabled(False)
#    MainWindow.btn_reset_forma.setEnabled(False)
#    MainWindow.btn_apli_color.setEnabled(False)
#    MainWindow.btn_apli_forma.setEnabled(False)

def namespace(qnode,MainWindow):
    global seleccion
    seleccion = MainWindow.topicos_election.currentText()
    qnode.seleccion_namespace(seleccion)



def on_btn_apli_forma (qnode, MainWindow):
    MainWindow.btn_reset_color.setEnabled(False)
    MainWindow.btn_apli_color.setEnabled(False)
    canny_inferior = MainWindow.canny_min.value()
    canny_superior = MainWindow.canny_max.value()
    cantidad_vertices = MainWindow.vertices.value()
    qnode.set_reco_forma(canny_inferior, canny_superior, cantidad_vertices)


# Función que usa los valores actualizados
def on_btn_apli_color(qnode, MainWindow):
    MainWindow.btn_reset_forma.setEnabled(False)
    MainWindow.btn_apli_forma.setEnabled(False)
    hsv_inferior = np.array([
        MainWindow.valor_H_inferior.value(),
        MainWindow.valor_S_inferior.value(),
        MainWindow.valor_V_inferior.value()])
    hsv_superior= np.array([
        MainWindow.valor_H_superior.value(),
        MainWindow.valor_S_superior.value(),
        MainWindow.valor_V_superior.value()])
    print(f"{hsv_inferior}")
    print(f"{hsv_superior}")
    qnode.set_reco_color(hsv_inferior, hsv_superior)



def on_btn_reset_color(qnode, MainWindow):
    MainWindow.btn_reset_forma.setEnabled(True)
    MainWindow.btn_apli_forma.setEnabled(True)
    MainWindow.camara_con_reco.clear()
    qnode.set_reset_data()
    
    

def on_btn_reset_forma(qnode, MainWindow):
    MainWindow.camara_con_reco.clear()
    MainWindow.btn_reset_color.setEnabled(True)
    MainWindow.btn_apli_color.setEnabled(True)
    qnode.set_reset_data_forma()
    


def update_camera_display(timer, qnode, MainWindow):
    pixmap = qnode.SetCameraManipulator(True)
    MainWindow.camara_sin_reco.setPixmap(pixmap)  # Actualiza QLabel
    MainWindow.camara_sin_reco.setScaledContents(True)  # Escala la imagen al tamaño del QLabel

def update_camera_display_real(timer, qnode, MainWindow):
    pixmap = qnode.SetCameraManipulator_real(True)
    MainWindow.camara_sin_reco.setPixmap(pixmap)  # Actualiza QLabel
    MainWindow.camara_sin_reco.setScaledContents(True)  # Escala la imagen al tamaño del QLabel
    

def update_camera_reco(qnode, MainWindow):
    pixmap2 = qnode.set_image()
    if pixmap2 is False:
        pass
    else:
        MainWindow.camara_con_reco.setPixmap(pixmap2)  # Actualiza QLabel
        MainWindow.camara_con_reco.setScaledContents(True)  # Escala la imagen al tamaño del QLabel


def update_camera_reco_real(qnode, MainWindow):
    pixmap2 = qnode.set_image_real()
    if pixmap2 is False:
        pass
    else:
        MainWindow.camara_con_reco.setPixmap(pixmap2)  # Actualiza QLabel
        MainWindow.camara_con_reco.setScaledContents(True)  # Escala la imagen al tamaño del QLabel

def update_camera_reco_forma(qnode, MainWindow):
    pixmap5 = qnode.set_image_forma()
    if pixmap5 is False:
        pass
    else:
        MainWindow.camara_con_reco.setPixmap(pixmap5)  # Actualiza QLabel
        MainWindow.camara_con_reco.setScaledContents(True)  # Escala la imagen al tamaño del QLabel

def update_camera_reco_forma_real(qnode, MainWindow):
    pixmap5 = qnode.set_image_forma_real()
    if pixmap5 is False:
        pass
    else:
        MainWindow.camara_con_reco.setPixmap(pixmap5)  # Actualiza QLabel
        MainWindow.camara_con_reco.setScaledContents(True)  # Escala la imagen al tamaño del QLabel



def on_btn_connect_real_clicked(timer, qnode, MainWindow):
    """
    Función que maneja el evento cuando se hace clic en el botón de conexión.
    Inicia el nodo del controlador del manipulador y espera hasta que esté listo.
    """
    pixmap2=None
    pixmap5=None
    #qnode.desconectar_namespace_sim()
    # Esperar hasta que el controlador publique los datos
    timeout = 10  # Tiempo máximo de espera en segundos
    start_time = time.time()
    while not hasattr(qnode, "present_joint_angle") or qnode.present_joint_angle is None:
        elapsed_time = time.time() - start_time
        if elapsed_time > timeout:
            print("Error: Tiempo de espera agotado, el controlador no respondió")

            # Mostrar mensaje en la interfaz
            msg_box = QMessageBox()
            msg_box.setIcon(QMessageBox.Warning)
            msg_box.setWindowTitle("Error de conexión")
            msg_box.setText("No se pudo conectar con el robot dentro del tiempo límite.")
            msg_box.setStandardButtons(QMessageBox.Ok)
            msg_box.exec_()

    #        qnode.stop_robot_controller_real()  # Asegurar que se detiene el proceso
            return  # Salir de la función sin continuar
        

        time.sleep(0.5)  # Esperar medio segundo antes de revisar nuevamente

    if not timer.isActive():
        timer.timeout.connect(lambda: update_camera_display_real(timer, qnode, MainWindow))  # Imprimir cada 10ms
        timer.timeout.connect(lambda: update_camera_reco_real(qnode, MainWindow))  # Imprimir cada 10ms
        timer.timeout.connect(lambda: update_camera_reco_forma_real(qnode, MainWindow))  # Imprimir cada 10ms

        timer.start(10)  # Iniciar el temporizador cada 10 ms

    # Una vez que el controlador está listo, continuar con la conexión
    
    qnode.namespace_real()
    timer.timeout.connect(lambda: timerCallback(qnode, MainWindow))  # Conectar el temporizador al callback
    timer.start(100)  # Iniciar el temporizador cada 100 ms
    MainWindow.state_real.setText("Conectado")
    MainWindow.btn_connect_real.setEnabled(False)
    MainWindow.btn_disconnect_real.setEnabled(True)
    MainWindow.btn_connect_sim.setEnabled(False)
    MainWindow.btn_disconnect_sim.setEnabled(False)
    MainWindow.btn_actuator_enable.setEnabled(True)
    MainWindow.btn_actuator_disable.setEnabled(True)
    MainWindow.btn_send_joint_angle.setEnabled(True)
    MainWindow.btn_send_cartesian.setEnabled(True)
    MainWindow.btn_pos_inicial.setEnabled(True)
    MainWindow.btn_pos_reset.setEnabled(True)
    MainWindow.btn_cerrar_gripper.setEnabled(True)
    MainWindow.btn_abrir_gripper.setEnabled(True)
    MainWindow.btn_act_joint_angle.setEnabled(True)
    MainWindow.btn_act_cartesian.setEnabled(True)
    MainWindow.btn_reset_color.setEnabled(True)
    MainWindow.btn_reset_forma.setEnabled(True)
    MainWindow.btn_apli_color.setEnabled(True)
    MainWindow.btn_apli_forma.setEnabled(True)

def on_btn_connect_sim_clicked(timer, qnode, MainWindow):
    """
    Función que maneja el evento cuando se hace clic en el botón de conexión.
    Inicia el nodo del controlador del manipulador y espera hasta que esté listo.
    """
    qnode.namespace_simulado()
    qnode.start_robot_controller_sim()
    

    # Esperar hasta que el controlador publique los datos
    timeout = 10  # Tiempo máximo de espera en segundos
    start_time = time.time()

    while not hasattr(qnode, "present_joint_angle") or qnode.present_joint_angle is None:
        elapsed_time = time.time() - start_time
        if elapsed_time > timeout:
            print("Error: Tiempo de espera agotado, el controlador no respondió")

            # Mostrar mensaje en la interfaz
            msg_box = QMessageBox()
            msg_box.setIcon(QMessageBox.Warning)
            msg_box.setWindowTitle("Error de conexión")
            msg_box.setText("No se pudo conectar con la simulación dentro del tiempo límite.")
            msg_box.setStandardButtons(QMessageBox.Ok)
            msg_box.exec_()

            qnode.stop_robot_controller_sim()  # Asegurar que se detiene la simulación
            return  # Salir de la función sin continuar

        time.sleep(0.5)  # Esperar medio segundo antes de revisar nuevamente

    if not timer.isActive():
        timer.timeout.connect(lambda: update_camera_display(timer, qnode, MainWindow))  # Imprimir cada 10ms
        timer.timeout.connect(lambda: update_camera_reco(qnode, MainWindow))  # Imprimir cada 10ms
        timer.timeout.connect(lambda: update_camera_reco_forma(qnode, MainWindow))  # Imprimir cada 10ms

        timer.start(10)  # Iniciar el temporizador cada 10 ms


    # Una vez que el controlador está listo, continuar con la conexión
    timer.timeout.connect(lambda: timerCallback(qnode, MainWindow))  # Conectar el temporizador al callback
    timer.start(100)  # Iniciar el temporizador cada 100 ms
    MainWindow.btn_connect_real.setEnabled(False)
    MainWindow.btn_disconnect_real.setEnabled(False)
    MainWindow.state_sim.setText("Conectado")
    MainWindow.btn_connect_sim.setEnabled(False)
    MainWindow.btn_disconnect_sim.setEnabled(True)
    MainWindow.btn_actuator_enable.setEnabled(True)
    MainWindow.btn_actuator_disable.setEnabled(True)
    MainWindow.btn_send_joint_angle.setEnabled(True)
    MainWindow.btn_send_cartesian.setEnabled(True)
    MainWindow.btn_pos_inicial.setEnabled(True)
    MainWindow.btn_pos_reset.setEnabled(True)
    MainWindow.btn_cerrar_gripper.setEnabled(True)
    MainWindow.btn_abrir_gripper.setEnabled(True)
    MainWindow.btn_act_joint_angle.setEnabled(True)
    MainWindow.btn_act_cartesian.setEnabled(True)
    MainWindow.btn_reset_color.setEnabled(True)
    MainWindow.btn_reset_forma.setEnabled(True)
    MainWindow.btn_apli_color.setEnabled(True)
    MainWindow.btn_apli_forma.setEnabled(True)



    

    
def on_btn_disconnect_real_clicked(qnode, timer, MainWindow):
    """
    Función que maneja el evento cuando se hace clic en el botón de desconexión.
    Detiene el temporizador.
    """
    qnode.desconectar_namespace_real()
    MainWindow.btn_connect_sim.setEnabled(True)
    MainWindow.btn_connect_real.setEnabled(True)
    MainWindow.value_joint1.setText("0.00")
    MainWindow.value_joint2.setText("0.00")
    MainWindow.value_joint3.setText("0.00")
    MainWindow.value_joint4.setText("0.00")
    MainWindow.value_gripper.setText("0.00")
    MainWindow.value_x.setText("0.00")
    MainWindow.value_y.setText("0.00")
    MainWindow.value_z.setText("0.00")
    MainWindow.doubleSpinBox_j1.setValue(0.00)
    MainWindow.doubleSpinBox_j2.setValue(0.00)
    MainWindow.doubleSpinBox_j3.setValue(0.00)
    MainWindow.doubleSpinBox_j4.setValue(0.00)
    MainWindow.doubleSpinBox_gripper.setValue(1)
    MainWindow.doubleSpinBox_axisX.setValue(0.00)
    MainWindow.doubleSpinBox_axisY.setValue(0.00)
    MainWindow.doubleSpinBox_axisZ.setValue(0.00)
    MainWindow.doubleSpinBox_gripper2.setValue(1)
    MainWindow.valor_H_inferior.setValue(0)#####################################################################################################
    MainWindow.valor_H_superior.setValue(0)
    MainWindow.valor_S_inferior.setValue(0)
    MainWindow.valor_S_superior.setValue(0)
    MainWindow.valor_V_inferior.setValue(0)
    MainWindow.valor_V_superior.setValue(0)
    MainWindow.state_real.setText("Desconectado")
    MainWindow.btn_connect_real.setEnabled(True)
    MainWindow.btn_disconnect_real.setEnabled(False)
    MainWindow.btn_connect_sim.setEnabled(True)
    MainWindow.btn_disconnect_sim.setEnabled(False)
    MainWindow.btn_actuator_enable.setEnabled(False)
    MainWindow.btn_actuator_disable.setEnabled(False)
    MainWindow.btn_send_joint_angle.setEnabled(False)
    MainWindow.btn_send_cartesian.setEnabled(False)
    MainWindow.btn_pos_inicial.setEnabled(False)
    MainWindow.btn_pos_reset.setEnabled(False)
    MainWindow.btn_cerrar_gripper.setEnabled(False)
    MainWindow.btn_abrir_gripper.setEnabled(False)
    MainWindow.btn_act_joint_angle.setEnabled(False)
    MainWindow.btn_act_cartesian.setEnabled(False)
    MainWindow.btn_reset_color.setEnabled(False)
    MainWindow.btn_reset_forma.setEnabled(False)
    MainWindow.btn_apli_color.setEnabled(False)
    MainWindow.btn_apli_forma.setEnabled(False)
    timer.stop()  # Detener el temporizador
#    print("QTimer stopped.")



def on_btn_disconnect_sim_clicked(qnode, timer, MainWindow):
    """
    Función que maneja el evento cuando se hace clic en el botón de desconexión.
    Detiene el temporizador.
    """
    qnode.stop_robot_controller_sim()
    MainWindow.value_joint1.setText("0.00")
    MainWindow.value_joint2.setText("0.00")
    MainWindow.value_joint3.setText("0.00")
    MainWindow.value_joint4.setText("0.00")
    MainWindow.value_gripper.setText("0.00")
    MainWindow.value_x.setText("0.00")
    MainWindow.value_y.setText("0.00")
    MainWindow.value_z.setText("0.00")
    MainWindow.doubleSpinBox_j1.setValue(0.00)
    MainWindow.doubleSpinBox_j2.setValue(0.00)
    MainWindow.doubleSpinBox_j3.setValue(0.00)
    MainWindow.doubleSpinBox_j4.setValue(0.00)
    MainWindow.doubleSpinBox_gripper.setValue(1)
    MainWindow.doubleSpinBox_axisX.setValue(0.00)
    MainWindow.doubleSpinBox_axisY.setValue(0.00)
    MainWindow.doubleSpinBox_axisZ.setValue(0.00)
    MainWindow.doubleSpinBox_gripper2.setValue(1)
    MainWindow.valor_H_inferior.setValue(0) #####################################################################################################
    MainWindow.valor_H_superior.setValue(0)
    MainWindow.valor_S_inferior.setValue(0)
    MainWindow.valor_S_superior.setValue(0)
    MainWindow.valor_V_inferior.setValue(0)
    MainWindow.valor_V_superior.setValue(0) 
    MainWindow.canny_min.setValue(0) ######################################################
    MainWindow.canny_max.setValue(0)
    MainWindow.vertices.setValue(0)
    MainWindow.state_sim.setText("Desconectado")
    MainWindow.btn_connect_sim.setEnabled(True)
    MainWindow.btn_disconnect_sim.setEnabled(False)
    MainWindow.btn_connect_real.setEnabled(True)
    MainWindow.btn_disconnect_real.setEnabled(False)
    MainWindow.btn_actuator_enable.setEnabled(False)
    MainWindow.btn_actuator_disable.setEnabled(False)
    MainWindow.btn_send_joint_angle.setEnabled(False)
    MainWindow.btn_send_cartesian.setEnabled(False)
    MainWindow.btn_pos_inicial.setEnabled(False)
    MainWindow.btn_pos_reset.setEnabled(False)
    MainWindow.btn_cerrar_gripper.setEnabled(False)
    MainWindow.btn_abrir_gripper.setEnabled(False)
    MainWindow.btn_act_joint_angle.setEnabled(False)
    MainWindow.btn_act_cartesian.setEnabled(False)
    MainWindow.btn_reset_color.setEnabled(False)
    MainWindow.btn_reset_forma.setEnabled(False)
    MainWindow.btn_apli_color.setEnabled(False)
    MainWindow.btn_apli_forma.setEnabled(False)
    timer.stop()  # Detener el temporizador
#    print("QTimer stopped.")
    MainWindow.camara_sin_reco.clear()
    MainWindow.camara_con_reco.clear()



def timerCallback(qnode, MainWindow):
    """
    Aquí obtenemos los ángulos de las articulaciones y actualizamos la interfaz.
    """
    joint_angle = qnode.getPresentJointAngle()
    
    if len(joint_angle) != 5:
        return  # Asegurarse de que el tamaño de la lista sea correcto

    # Actualizar la interfaz con los valores de los ángulos de las articulaciones
    MainWindow.value_joint1.setText(f"{joint_angle[0]:.2f}")
    MainWindow.value_joint2.setText(f"{joint_angle[1]:.2f}")
    MainWindow.value_joint3.setText(f"{joint_angle[2]:.2f}")
    MainWindow.value_joint4.setText(f"{joint_angle[3]:.2f}")
    MainWindow.value_gripper.setText(f"{joint_angle[4]*100:.2f}")

    """
    Aquí obtenemos pose de la cinematica.
    """
    position = qnode.getPresentKinematicsPose()

    if len(position) != 3:
        return  # Asegurarse de que el tamaño de la lista sea correcto
    
    # Actualizar la interfaz con los valores cartesianos de la cinematica
    MainWindow.value_x.setText(f"{position[0]*100:.2f}")
    MainWindow.value_y.setText(f"{position[1]*100:.2f}")
    MainWindow.value_z.setText(f"{position[2]*100:.2f}")


    # Obtener el estado del actuador y del movimiento del robot
    actuator_state = qnode.get_open_manipulator_actuator_state()
    moving_state = qnode.get_open_manipulator_moving_state()

    # Actualizar el estado del actuador en la interfaz
    if actuator_state:
        MainWindow.txt_actuator_state.setText("Actuador habilitado")
        MainWindow.btn_actuator_disable.setEnabled(True)
        MainWindow.btn_actuator_enable.setEnabled(False)
        MainWindow.btn_send_joint_angle.setEnabled(True)
        MainWindow.btn_send_cartesian.setEnabled(True)

    else:
        MainWindow.txt_actuator_state.setText("Actuador deshabilitado")
        MainWindow.btn_actuator_disable.setEnabled(False)
        MainWindow.btn_actuator_enable.setEnabled(True)


    # Actualizar el estado de movimiento en la interfaz
    if moving_state:
        MainWindow.txt_moving_state.setText("Robot en movimiento")
    else:
        MainWindow.txt_moving_state.setText("Robot en pausa")

def on_btn_actuator_enable_clicked(qnode):
    """
    Función para manejar el clic del botón de habilitar actuadores.
    """
    if not qnode.setActuatorState(True):  # Llamada para habilitar los actuadores
        print("[ERR!!] Failed to send service")
        return

    print("Send actuator state to enable")

def on_btn_actuator_disable_clicked(qnode):
    """
    Función para manejar el clic del botón de deshabilitar actuadores.
    """
    if not qnode.setActuatorState(False):  # Llamada para desahabilitar los actuadores
        print("[ERR!!] Failed to send service")
        return
    

def on_btn_pos_inicial_clicked(qnode):
    """
    Función para manejar el clic del botón "Posición Inicial".
    """
    joint_name = ["joint1", "joint2", "joint3", "joint4"]
    joint_angle = [0.0, -1.05, 0.35, 0.70]  # Ángulos de la posición inicial
    path_time = 2.0  # Tiempo de movimiento

    if not qnode.setJointSpacePath(joint_name, joint_angle, path_time):
        print("[ERR!!] Failed to send service")
        return

    print("Send joint angle to home pose")

def on_btn_pos_reset_clicked(qnode):
    """
    Función para manejar el clic del botón "Posición Inicial".
    """
    joint_name = ["joint1", "joint2", "joint3", "joint4"]
    joint_angle = [0.0, 0.04, 0.41, 0.8]  # Ángulos de la posición inicial
    path_time = 2.0  # Tiempo de movimiento

    if not qnode.setJointSpacePath(joint_name, joint_angle, path_time):
        print("[ERR!!] Failed to send service")
        return

    print("Send joint angle to home pose")

def on_btn_cerrar_gripper_clicked(qnode):
    """
    Función para manejar el clic del botón "Cerrar gripper".
    """
    global seleccion 
    if(seleccion=="/gamora" or seleccion=="/nebula"):
        joint_angle = [-0.025]  # Ángulo para cerrar gripper
        soul=qnode.setToolControl(joint_angle)
    else:
        joint_angle = [-0.01]  # Ángulo para cerrar gripper

        soul=qnode.setToolControl(joint_angle)
        if soul==True:
            print(joint_angle)
            rospy.sleep(0.2)
            qnode.Tool_attach(joint_angle) #puede haber problema porque se envia justo despues d eordenar mover el gripper
            print("Send gripper close")
        else:
            print("[ERR!!] Failed to send gripper position")
            return

def on_btn_abrir_gripper_clicked(qnode):
    """
    Función para manejar el clic del botón "abrir gripper".
    """
    global seleccion 
    if(seleccion=="/gamora" or seleccion=="/nebula"):
        joint_angle = [-0.018]  # Ángulo para cerrar gripper
        soul=qnode.setToolControl(joint_angle)
    else:
        joint_angle = [0.01]  # Ángulo para cerrar gripper

        soul=qnode.setToolControl(joint_angle)
        if soul==True:
            print(joint_angle)
            rospy.sleep(0.2)
            qnode.Tool_attach(joint_angle) #puede haber problema porque se envia justo despues d eordenar mover el gripper
            print("Send gripper close")
        else:
            print("[ERR!!] Failed to send gripper position")
            return

def on_btn_act_joint_angle_clicked(MainWindow):
    """
    Función para actualizar los doublespinbox con el valor articular actual, 
    para asi poder enviar los valores angulares deseados tomando de referencia el actual
    """
    MainWindow.doubleSpinBox_j1.setValue(float(MainWindow.value_joint1.text()))
    MainWindow.doubleSpinBox_j2.setValue(float(MainWindow.value_joint2.text()))
    MainWindow.doubleSpinBox_j3.setValue(float(MainWindow.value_joint3.text()))
    MainWindow.doubleSpinBox_j4.setValue(float(MainWindow.value_joint4.text()))
    MainWindow.doubleSpinBox_gripper.setValue(float(MainWindow.value_gripper.text()))

def on_btn_act_cartesian_clicked(MainWindow):
    """
    Función para actualizar los doublespinbox con el valor cartesiano actual, 
    para asi poder enviar los valores cartesianos deseados tomando de referencia el actual
    """
    MainWindow.doubleSpinBox_axisX.setValue(float(MainWindow.value_x.text()))
    MainWindow.doubleSpinBox_axisY.setValue(float(MainWindow.value_y.text()))
    MainWindow.doubleSpinBox_axisZ.setValue(float(MainWindow.value_z.text()))
    MainWindow.doubleSpinBox_gripper2.setValue(float(MainWindow.value_gripper.text()))

def on_btn_send_joint_angle_clicked(qnode, MainWindow):
    """
    Función para enviar los valores articulares y la posición del gripper al controlador.
    """
    joint_name = ["joint1", "joint2", "joint3", "joint4"]
    joint_angle = [
        MainWindow.doubleSpinBox_j1.value(),
        MainWindow.doubleSpinBox_j2.value(),
        MainWindow.doubleSpinBox_j3.value(),
        MainWindow.doubleSpinBox_j4.value(),
    ]
    path_time = MainWindow.doubleSpinBox_time_js.value()

    gripper_angle = [MainWindow.doubleSpinBox_gripper.value()/100]  # Lista con un solo valor para el gripper

    # Enviar comando para mover las articulaciones
    if not qnode.setJointSpacePath(joint_name, joint_angle, path_time):
        print("[ERR!!] Failed to send joint angles")
        return

    # Enviar comando para mover el gripper
    soul=qnode.setToolControl(gripper_angle)
    if soul==True:
        print(gripper_angle)
        rospy.sleep(0.2)
        qnode.Tool_attach(gripper_angle) #puede haber problema porque se envia justo despues d eordenar mover el gripper
        print("Send task pose + gripper position")

    else:
        print("[ERR!!] Failed to send gripper position")
        return

    print("Send joint angle + gripper position")

def on_btn_send_cartesian_clicked(qnode, MainWindow):
    """
    Función para enviar los valores cartesianos y la posición del gripper al controlador.
    """
    kinematics_pose = [
        MainWindow.doubleSpinBox_axisX.value()/100,
        MainWindow.doubleSpinBox_axisY.value()/100,
        MainWindow.doubleSpinBox_axisZ.value()/100,
    ]
    path_time = MainWindow.doubleSpinBox_time_crt.value()

    gripper_angle = [MainWindow.doubleSpinBox_gripper2.value()/100]  # Lista con un solo valor para el gripper

    # Enviar comando para mover las articulaciones
    if not qnode.setTaskSpacePath(kinematics_pose, path_time):
        print("[ERR!!] Failed to send joint angles")
        return

    # Enviar comando para mover el gripper
    soul=qnode.setToolControl(gripper_angle)
    if soul==True:
        print(gripper_angle)
        rospy.sleep(0.2)
        qnode.Tool_attach(gripper_angle) #puede haber problema porque se envia justo despues d eordenar mover el gripper
        print("Send task pose + gripper position")

    else:
        print("[ERR!!] Failed to send gripper position")
        return

if __name__ == "__main__":
    main()
