import cv2
import mediapipe as mp
import rospy
from std_msgs.msg import Int32

# Inicialización de MediaPipe Hands
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Inicialización de la captura de video
cap = cv2.VideoCapture(0)

# Inicialización del nodo de ROS
rospy.init_node('publicador_gestos', anonymous=True)
publicador_posicion = rospy.Publisher('/Destinos', Int32, queue_size=10)

# Variable para almacenar el estado del destino
estado_destino = 0
ultimo_estado_destino = None  # Para evitar publicaciones repetidas

def contar_dedos(landmarks):
    dedos_levantados = 0

    # Pulgar
    if landmarks[mp_hands.HandLandmark.THUMB_TIP].x > landmarks[mp_hands.HandLandmark.THUMB_IP].x:
        dedos_levantados += 1

    # Otros dedos
    dedos = [
        mp_hands.HandLandmark.INDEX_FINGER_TIP,
        mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
        mp_hands.HandLandmark.RING_FINGER_TIP,
        mp_hands.HandLandmark.PINKY_TIP,
    ]

    for dedo in dedos:
        if landmarks[dedo].y < landmarks[dedo - 2].y:
            dedos_levantados += 1

    return dedos_levantados

# Definir el reconocimiento de manos con MediaPipe
with mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.5) as hands:
    while cap.isOpened() and not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        # Convertir la imagen de BGR a RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Procesar la imagen para encontrar manos
        results = hands.process(frame_rgb)

        # Convertir la imagen nuevamente a BGR
        frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        # Dibujar los puntos de referencia de las manos
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Obtener los puntos de referencia de los dedos
                landmarks = hand_landmarks.landmark

                # Contar el número de dedos levantados
                dedos_levantados = contar_dedos(landmarks)

                if dedos_levantados == 0:
                    cv2.putText(frame, 'Orden: Ruta Autonoma', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    estado_destino = 0
                elif 1 <= dedos_levantados <= 5:
                    cv2.putText(frame, f'Orden: Destino {dedos_levantados}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    estado_destino = dedos_levantados

                # Publicar solo si el estado ha cambiado
                if estado_destino != ultimo_estado_destino:
                    publicador_posicion.publish(estado_destino)
                    ultimo_estado_destino = estado_destino

        # Mostrar la imagen resultante
        cv2.putText(frame, f'Estado Destino: {estado_destino}', (10, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.imshow('Reconocimiento de Gestos con la Mano', frame)

        # Salir si se presiona la tecla 'q'
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

# Liberar la captura y cerrar ventanas
cap.release()
cv2.destroyAllWindows()
