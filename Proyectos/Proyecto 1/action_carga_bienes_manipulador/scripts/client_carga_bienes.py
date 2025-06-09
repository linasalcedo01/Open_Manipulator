#!/usr/bin/env python3

import rospy
import actionlib

# Importa los mensajes generados desde el archivo Recogida.action
from action_carga_bienes_manipulador.msg import carga_bienesAction, carga_bienesGoal

def feedback_cb(feedback):
    """
    Esta función se ejecuta cada vez que el servidor envía feedback.
    Muestra el estado actual del servidor mientras ejecuta la acción.
    """
    rospy.loginfo("Feedback del servidor: %s", feedback.estado_actual) #########################################################################################

def main():
    # Inicializa el nodo ROS
    rospy.init_node('cliente_recogida')

    # Crea un cliente de acción. Aquí 'recogida' es el nombre del servidor al que se quiere conectar.
    cliente = actionlib.SimpleActionClient('carga_bienes', carga_bienesAction)

    # Espera hasta que el servidor esté disponible
    rospy.loginfo("Esperando al servidor de acción 'recogida'...")
    cliente.wait_for_server()
    rospy.loginfo("Servidor conectado ✅")

    # Crea un objetivo (goal) que se va a enviar al servidor
    goal = carga_bienesGoal()
    #goal.color = "azul"
    #goal.color = " "
    #goal.color = "verde"
    goal.color = "rojo"
    #goal.forma = " "
    goal.forma = "cuadrado"
    #goal.forma = "triangulo"
    #goal.forma = "circulo"


    rospy.loginfo("Enviando objetivo: Color=%s, Forma=%s", 
                  goal.color, goal.forma)

    # Envía el goal al servidor, junto con la función de feedback
    cliente.send_goal(goal, feedback_cb=feedback_cb)

    # Espera hasta que el servidor termine de procesar
    cliente.wait_for_result()

    # Obtiene y muestra el resultado final
    resultado = cliente.get_result()
    rospy.loginfo("Resultado final del servidor: %s", resultado.estado_final) #############################################################################################3

if __name__ == '__main__':
    main()

