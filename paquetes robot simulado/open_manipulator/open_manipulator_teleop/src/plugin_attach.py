#!/usr/bin/env python3
import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


class pluggin_attach:

    def __init__(self):
        rospy.init_node('attach_teleopkey', anonymous=True)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',Attach)
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',Attach)

    def Tool_attach(self,joint_angle): 
        if joint_angle==[-0.01]:
            #while detectedObject is None:
            self.attach_detect()
        if joint_angle== [0.01]:
            #while detectedObject is None:
            self.detach_detect()

    def attach_detect(self):
        if self.detectedObject is not None and self.resetvalue is not True:
            rospy.loginfo(f"Attaching gripper and {self.detectedObject}")
            req = AttachRequest()
            req.model_name_1 = "open_manipulator"
            req.link_name_1 = "gripper_link_sub"
            req.model_name_2 = self.detectedObject
            req.link_name_2 = "link"
            self.attach_srv.call(req)
        else:
            print("objeto no detectado -> no collision")

    def detach_detect(self): #hay problema si no agarra nada
        if self.detectedObject is not None:
            rospy.loginfo(f"Detach gripper and {self.detectedObject}")
            req1 = AttachRequest()
            req1.model_name_1 = "open_manipulator"
            req1.link_name_1 = "gripper_link_sub"
            req1.model_name_2 = self.detectedObject
            req1.link_name_2 = "link"
            self.detach_srv.call(req1)
            

    def get_contacts(self, msg):
        if (len(msg.states) == 0):
            self.resetvalue=True
            #rospy.loginfo("No contacts were detected!")
        else:
            if 'gripper_link_sub' in msg.states[0].collision1_name:
                self.resetvalue=False
                #rospy.loginfo("Collision 1 detected with %s." % msg.states[0].collision2_name.split("::")[0])
                objeto = msg.states[0].collision2_name.split("::")[0]
                objetos_validos = ["esfera_roja", "esfera_azul", "esfera_verde","cubo_rojo", "cubo_verde", "cubo_azul","cubo_verdep", "esfera_rojap"]
                print(f"{objeto}")
                if objeto in objetos_validos:
                    self.detectedObject=objeto
                    

                    
                    
            elif 'gripper_link_sub' in msg.states[0].collision2_name:
                self.resetvalue=False
                #rospy.loginfo("Collision 2 detected with %s." % msg.states[0].collision1_name.split("::")[0])
                objeto = msg.states[0].collision1_name.split("::")[0]
                print(f"{objeto}")
                objetos_validos = ["esfera_roja", "esfera_azul", "esfera_verde","cubo_rojo", "cubo_verde", "cubo_azul","cubo_verdep", "esfera_rojap"]
                if objeto in objetos_validos:
                    self.detectedObject=objeto

    def run(self):
        rospy.spin()        

if __name__ == "__main__":
    try:
        nodo = pluggin_attach()
        nodo.publicar_mensaje()
    except rospy.ROSInterruptException:
        pass