import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pyzbar.pyzbar import decode
from geometry_msgs.msg import Twist

#Clase para el movimiento del robot
class ClienteMoveBase:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.current_zone = None

    def moveTo(self, x, y, zone=None):
        self.current_zone = zone  # Actualizar la zona actual
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)

    def cancelGoal(self):
        self.client.cancel_all_goals()
        self.current_zone = None

    def waitForResult(self):
        state = self.client.get_state()
        while state in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
            rospy.Rate(10).sleep()
            state = self.client.get_state()
        return self.client.get_result()
#Definicion de posiciones de destino segun el mapa usado para el proyecto
def get_position(zone):
    positions = {
        1: (-6.7, -1.2),
        2: (2.7, -4.3),
        3: (10, -3),
        4: (5.86, -10.5),
        5: (14, 3),
        6: (-3, -1.1)  
    }
    return positions.get(zone, None)

#Funcion de busqueda de QR una vez que el robot llega al destino esperado
def rotate_until_qr_found(expected_zone):
    print(f"Entra en búsqueda de QR en la zona: {expected_zone}")
    bridge = CvBridge()
    camera_topic = "/camera/rgb/image_raw"
    qr_verified = False
    
    #Publisher para la rotacion del robot sobre si mismo, haciendo que haga giros de 360 grados para que la camara pueda reconocer el QR
    twist_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    twist = Twist()
    twist.angular.z = 0.5  # Velocidad de rotación baja para poder capturar el QR mientras el robot se mueve

    #Procesado del QR para su interpretacion
    def image_callback(ros_image):
        nonlocal qr_verified
        try:
            cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
            decoded_objects = decode(cv_image)
            for obj in decoded_objects:
                data = obj.data.decode('utf-8')
                rospy.loginfo(f"QR detectado: {data}")
                if data.isdigit() and int(data) == expected_zone:
                    rospy.loginfo("QR correcto encontrado. Deteniendo rotación.")
                    qr_verified = True
                    return
        except Exception as e:
            rospy.logerr(f"Error procesando la imagen: {e}")

    #Suscriptor al topic que da las imagenes provenientes del robot
    rospy.Subscriber(camera_topic, Image, image_callback)

    rate = rospy.Rate(10)
    while not qr_verified:
        twist_pub.publish(twist)
        rate.sleep()
        if cliente.current_zone is None or cliente.current_zone != expected_zone:
            rospy.logwarn("Cambio de destino detectado. Cancelando búsqueda de QR.")
            return False

    twist.angular.z = 0.0   #Detiene el giro del robot una vez que el QR se ha encontrado para dejarlo apuntando hacia la puerta de la tienda
    twist_pub.publish(twist)
    return True

#Funcion para el movimiento del robot a los distintos destinos
def move_to_waypoints(waypoints, cliente):
    for zone in waypoints:
        position = get_position(zone)
        if position:
            rospy.loginfo(f"Moviendo al robot a la posición {position} de la zona {zone}")
            cliente.moveTo(position[0], position[1], zone)
            result = cliente.waitForResult()
            if result and cliente.current_zone == zone:
                rospy.loginfo(f"Meta alcanzada en la zona {zone}. Buscando QR...")
                if rotate_until_qr_found(zone):
                    rospy.loginfo(f"Robot confirmado en la posición correcta de la zona {zone}.")
                else:
                    rospy.logwarn(f"Falló la verificación del QR para la zona {zone}.")
                    break
            else:
                rospy.logwarn(f"No se alcanzó la meta en la zona {zone} o el destino cambió.")
                break
        else:
            rospy.logwarn(f"Zona {zone} no es válida.")

def posicion_callback(msg):
    global cliente
    if isinstance(msg.data, int):
        cliente.cancelGoal()
        if msg.data == 6:
            rospy.logwarn("¡Botón de emergencia activado! Cancelando ruta y moviendo a coordenadas de emergencia.")
            emergencia = get_position(6)
            if emergencia:
                cliente.moveTo(emergencia[0], emergencia[1], 6)
                cliente.waitForResult()
                rospy.loginfo("Robot en posición de emergencia.")
        elif msg.data == 0:
            waypoints = [1, 2, 3, 4, 5]
            move_to_waypoints(waypoints, cliente)
        else:
            rospy.loginfo(f"Nuevo destino recibido: Zona {msg.data}")
            destino = get_position(msg.data)
            if destino:
                cliente.moveTo(destino[0], destino[1], msg.data)
                cliente.waitForResult()
                if cliente.current_zone == msg.data:
                    rotate_until_qr_found(msg.data)
            else:
                rospy.logwarn(f"Zona {msg.data} no es válida.")
    else:
        rospy.logerr("Formato de destino inválido. Debe ser un entero.")

if __name__ == "__main__":
    rospy.init_node('cliente_movebase_waypoints')
    cliente = ClienteMoveBase()
    rospy.Subscriber('/Destinos', Int32, posicion_callback)
    rospy.spin()
