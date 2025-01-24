import sys
import pyaudio
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QLineEdit,
    QComboBox, QFrame, QSizePolicy, QTableWidget, QTableWidgetItem, QHeaderView, QSlider
)
from PyQt5.QtGui import QColor, QFont
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import rospy

from geometry_msgs.msg import PoseStamped,TwistStamped,Twist, PoseWithCovarianceStamped
from PyQt5.QtWidgets import QCheckBox
import pandas as pd
import os
import speech_recognition as sr
import threading
from PyQt5.QtGui import QPixmap,QImage, QPainter, QBrush,QPen
from PyQt5.QtCore import Qt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
from std_msgs.msg import Bool
from kobuki_msgs.msg import SensorState
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from PyQt5.QtCore import QTimer
import time

# Hoja de estilo para modo claro
light_style = """
QWidget {
    background-color: #f0f0f0;
    color: #000000;
}

QPushButton {
    background-color: #dddddd;
    color: #000000;
    border: none;
    padding: 10px;
    border-radius: 5px;
}

QPushButton:hover {
    background-color: #cccccc;
}

QLineEdit, QTextEdit {
    background-color: #ffffff;
    color: #000000;
    border: 1px solid #cccccc;
}
"""

dark_style = """
QWidget {
    background-color: #2b2b2b;
    color: #f0f0f0;
}

QPushButton {
    background-color: #4a4a4a;
    color: #f0f0ff;
    border: none;
    padding: 10px;
    border-radius: 5px;
}

QPushButton:hover {
    background-color: #6a6a6a;
}

QLineEdit, QTextEdit {
    background-color: #3a3a3a;
    color: #f0f0f0;
    border: 1px solid #6a6a6a;
}
"""

# Directorios
main_directory = os.getcwd()
data_directory = os.path.join(main_directory, "data", "data.csv")

ultima_pose = PoseStamped()
pos_tracker = PoseStamped()
vel_tracker = TwistStamped()
image_camera_turtle = QImage()

#Para 4K => 1600 800 // FULL HD => 800 400
escaladoX = 1600
escaladoY = 800
#PARA 4K => 960 760 // FULL HD => 480 360
escaladotrackerx = 940
escaladotrackery = 810

# Almacenamos la posición enviada anteriormente
def callback_new_position(msg):
    """Callback to update the setpoint based on new position messages"""
    global ultima_pose
    ultima_pose = msg
    return

class TurtleInterface(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setStyleSheet(dark_style)
        self.is_dark = True
        self.toggle_button = None
        self.slider = None
        self.voice_status_label = QLabel()
        self.position_label = QLabel()
        self.velocity_label = QLabel()
        self.altitude_label = QLabel()
        self.pub_posicion = PoseStamped()
        self.posicion_momento = PoseStamped()

        self.posicion_tracker = PoseStamped()
        self.battery_label = QLabel()

        self.velocity_tracker = TwistStamped()
        self.bridge = CvBridge()
        self.camera_display = QLabel()
        self.map_display = QLabel()
        self.pack_ok_pub = Bool()
        self.nivelbateria = int()
        self.positionODOM = float()
        self.orientationODOM = float()
        self.inear_velocityODOM = float()
        self.angular_velocityODOM = float()
        self.posactual = PoseWithCovarianceStamped()
        self.Odom = Odometry()
        self.iniciar_nodos()

        self.setWindowTitle("Interfaz de control")
        self.setGeometry(100, 100, 1200, 800)
        
        # Tab widget to manage different sections
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        
        # Add tabs
        self.tabs.addTab(self.create_voice_command_tab(), "INTERFAZ")

        self.local_pos_pub = rospy.Publisher('turtle_position_command', PoseStamped, queue_size=10)

        self.base_pixmap_tracking = QPixmap(self.map_display.pixmap())
    
    
    def callback_camera_turtle(self, msg):
        global image_camera_turtle

        try:
            # Verificar que el mensaje no sea None
            if msg is None:
                raise ValueError("El mensaje recibido es None.")

            # Intentar convertir el mensaje ROS a una imagen OpenCV
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                raise ValueError(f"Error al convertir el mensaje a imagen OpenCV: {e}")
            
            # Verificar que la imagen no sea None o esté vacía
            if cv_image is None or cv_image.size == 0:
                raise ValueError("La imagen convertida desde el mensaje es None o está vacía.")

            # Verificar que la imagen tenga 3 canales (RGB/BGR)
            if len(cv_image.shape) != 3 or cv_image.shape[2] != 3:
                raise ValueError("La imagen no tiene un formato de 3 canales (RGB/BGR).")

            # Redimensionar la imagen
            try:
                resized_image = cv2.resize(cv_image, (400, 400), interpolation=cv2.INTER_LINEAR)
            except Exception as e:
                raise ValueError(f"Error al redimensionar la imagen: {e}")

            # Convertir la imagen redimensionada a formato RGB para QImage
            try:
                rgb_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
            except Exception as e:
                raise ValueError(f"Error al convertir la imagen a formato RGB: {e}")
            
            # Crear un QImage con los datos de la imagen RGB
            try:
                height, width, channel = rgb_image.shape
                bytes_per_line = channel * width
                image_camera_turtle = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            except Exception as e:
                raise ValueError(f"Error al crear el QImage: {e}")

            # Convertir el QImage a QPixmap y mostrarlo en el QLabel
            try:
                pixmap2 = QPixmap.fromImage(image_camera_turtle)
                self.camera_display.setPixmap(pixmap2)
                self.camera_display.setScaledContents(True)  # Escala la imagen para ajustarse al QLabel
            except Exception as e:
                raise ValueError(f"Error al mostrar la imagen en el QLabel: {e}")

        except Exception as e:
            rospy.logerr(f"Error procesando la imagen: {e}")

    
    def battery_callback(self,msg):
        self.nivelbateria = (msg.battery/16)*10
        self.battery_label.setText(f"Nivel de bateria: {self.nivelbateria:.1f}%")
        
    def velocity_callback(self, msg):
        self.Odom = msg  # Guarda el mensaje recibido

        vel_x = msg.twist.twist.linear.x
        ang_z = msg.twist.twist.angular.z

        # Actualiza la interfaz
        self.velocity_label.setText(f"Vel. lineal: {vel_x:.2f} m/s, angular: {ang_z:.2f} rad/s")

    def odometry_callback(self, msg):
        self.posactual = msg  # Guarda el mensaje recibido

        # Extrae los valores de posición
        x_position = msg.pose.pose.position.x
        y_position = msg.pose.pose.position.y
        z_position = msg.pose.pose.position.z

        # Actualiza la interfaz
        self.position_label.setText(f"Posicion: ({x_position:.2f}, {y_position:.2f}, {z_position:.2f})")
        time.sleep(0.1)
        #self.actualizar_tracker(x_position, y_position)

    def iniciar_nodos(self):
        # Inicializar el nodo de ROS
        rospy.init_node('interfaz_voz_turtle', anonymous=True)
        self.publicador = rospy.Publisher('/Destinos', Int32, queue_size=10,latch=True)

        self.pub_posicion = rospy.Publisher('/new_position', PoseStamped, queue_size=10)
        
        self.camera_turtle = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback_camera_turtle)
        self.mensajes_sensores_base = rospy.Subscriber('/mobile_base/sensors/core', SensorState, self.battery_callback)
        self.odometria = rospy.Subscriber('/odom', Odometry, self.velocity_callback)
        self.amcl = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.odometry_callback)
        self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)


    def create_voice_command_tab(self):
        """Tab for voice commands and predefined routes with an improved design."""
        global pos_tracker,image_camera_turtle
        tab = QWidget()
        layout = QHBoxLayout()

        # Column 1: Camera, Mini Map, and Trutel Status
        column1 = QVBoxLayout()
        
        # Mini Map (Top)
        map_label = QLabel("Mapa del entorno")
        map_label.setAlignment(Qt.AlignCenter)  # Asegurar que el texto está centrado
        map_label.setStyleSheet("font-weight: bold;")  # Aplicar negrita
        self.map_display.setFixedSize(escaladoX,escaladoY)
        self.map_display.setStyleSheet("background-color: lightgray; border: 1px solid black;")
        self.map_display.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        column1.addWidget(map_label)

        # Cargar la imagen del mini mapa
        pixmap = QPixmap("images/MapaTurtlebot.jpg")
        self.base_pixmap_tracking = QPixmap(pixmap)
        self.map_display.setPixmap(pixmap)
        self.map_display.setScaledContents(True)
        column1.addWidget(self.map_display, 2)

        # Camera view (Middle)
        camera_label = QLabel("Camara del robot")
        camera_label.setAlignment(Qt.AlignCenter)
        camera_label.setStyleSheet("font-weight: bold;") 
        self.camera_display.setStyleSheet("background-color: black; border: 1px solid gray;")
        self.camera_display.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.camera_display.setFixedSize(escaladoX,escaladoY)
        column1.addWidget(camera_label)
        column1.addWidget(self.camera_display, 2)

        # Turtle Status (Bottom)
        turtle_status_label = QLabel("Estado del robot")
        turtle_status_label.setAlignment(Qt.AlignCenter)
        turtle_status_label.setStyleSheet("font-weight: bold;")

        x_position = self.posactual.pose.pose.position.x
        y_position = self.posactual.pose.pose.position.y
        z_position = self.posactual.pose.pose.position.z

        vel_x = self.Odom.twist.twist.linear.x
        ang_z = self.Odom.twist.twist.angular.z

        self.position_label = QLabel(f"Posicion: ({x_position}, {y_position}, {z_position})")
        self.velocity_label = QLabel(f"Vel. lineal: {vel_x:.2f} m/s, angular: {ang_z:.2f} rad/s")
        self.battery_label = QLabel(f"Nivel de batería: {self.nivelbateria}%")
        status_section = QVBoxLayout()
        status_section.addWidget(turtle_status_label)
        status_section.addWidget(self.position_label)
        status_section.addWidget(self.velocity_label)
        status_section.addWidget(self.battery_label)
        column1.addLayout(status_section, 1) 

        # Columna 2: Comandos de Voz
        column2 = QVBoxLayout()

        # Crear el título (label)
        map_label = QLabel("Controles")
        map_label.setAlignment(Qt.AlignCenter) 
        map_label.setStyleSheet("font-weight: bold;")  

        # Añadir el título al layout
        column2.addWidget(map_label)

        # Crear los botones de colores
        font = QFont()
        font.setBold(True)

        red_button = QPushButton("CERVECERÍA")
        red_button.setFont(font)

        yellow_button = QPushButton("TIENDA DE ROPA")
        yellow_button.setFont(font)

        green_button = QPushButton("ASEOS")
        green_button.setFont(font)

        blue_button = QPushButton("RESTAURANTE")
        blue_button.setFont(font)

        inicio_button = QPushButton("INICIO")
        inicio_button.setFont(font)

        # Asignar un color a cada botón y conectar a la función 'command_manual' con el color
        red_button.setStyleSheet("background-color: red;")
        yellow_button.setStyleSheet("background-color: orange;")
        green_button.setStyleSheet("background-color: green;")
        blue_button.setStyleSheet("background-color: blue;")
        inicio_button.setStyleSheet("background-color: gray")

        # Conectar cada botón con la función command_manual pasándole el color
        red_button.clicked.connect(lambda: self.command_manual("red"))
        yellow_button.clicked.connect(lambda: self.command_manual("yellow"))
        green_button.clicked.connect(lambda: self.command_manual("green"))
        blue_button.clicked.connect(lambda: self.command_manual("blue"))
        inicio_button.clicked.connect(lambda: self.command_manual("inicio"))
        # Crear un layout horizontal para los botones
        button_layout = QHBoxLayout()
        button_layout.addWidget(red_button)
        button_layout.addWidget(yellow_button)
        button_layout.addWidget(green_button)
        button_layout.addWidget(blue_button)
        button_layout.addWidget(inicio_button)

        # Añadir el layout de los botones al layout principal (column2)
        column2.addLayout(button_layout)

        # Activate Voice Recognition
        activate_voice_btn = QPushButton("Activate Voice Recognition")
        activate_voice_btn.clicked.connect(self.activate_voice_recognition)

        # Status label for voice recognition
        self.voice_status_label = QLabel()
        self.voice_status_label.setStyleSheet("background-color: lightyellow; border: 1px solid gray; font-weight: bold; color: black;")
        self.voice_status_label.setWordWrap(True)

        # Tabla para mostrar el contenido del CSV
        csv_label = QLabel("Command Log:")
        clear_csv_btn = QPushButton("Clear CSV")
        refresh_csv_btn = QPushButton("Refresh Table")
        self.command_table = QTableWidget()
        self.command_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Layout horizontal para los botones
        buttons_layout = QHBoxLayout()
        buttons_layout.addWidget(clear_csv_btn)
        buttons_layout.addWidget(refresh_csv_btn)

        # Cargar datos del CSV y llenar la tabla
        self.populate_command_log_table(self.command_table, data_directory)
        
        # Conectar botones a sus funciones
        clear_csv_btn.clicked.connect(lambda: self.clear_csv(self.command_table))
        refresh_csv_btn.clicked.connect(lambda: self.populate_command_log_table(self.command_table, data_directory))

        # Agregar widgets al layout
        voice_section = QVBoxLayout()
        voice_section.addWidget(activate_voice_btn)
        voice_section.addWidget(self.voice_status_label)
        column2.addLayout(voice_section)
        column2.addWidget(csv_label)
        column2.addLayout(buttons_layout)
        column2.addWidget(self.command_table)
        column2.addStretch(1)

        # Columna 3: Predefined Routes and Indicators
        column3 = QVBoxLayout()
        
        map_label = QLabel("Ajustes")
        map_label.setAlignment(Qt.AlignCenter) 
        map_label.setStyleSheet("font-weight: bold;") 
        column3.addWidget(map_label)
        
        # Botón para cambiar el modo
        self.toggle_button = QPushButton("Alternar tema")
        self.toggle_button.clicked.connect(self.toggle_mode)
        column3.addWidget(self.toggle_button)
        
        #Titulo slicer
        slider_title = QLabel("Velocidad lineal")
        slider_title.setAlignment(Qt.AlignCenter)

        #Slider de velodidad max
        self.slider = QSlider(Qt.Horizontal)  
        self.slider.setMinimum(1)             
        self.slider.setMaximum(5)           
        self.slider.setValue(1)            
        self.slider.setTickPosition(QSlider.TicksBelow)  
        self.slider.setTickInterval(10)   

        #Titulo slicer
        slider_title2 = QLabel("Velocidad angular")
        slider_title2.setAlignment(Qt.AlignCenter)

        #Slider de velodidad max
        self.slider2 = QSlider(Qt.Horizontal)  
        self.slider2.setMinimum(1)             
        self.slider2.setMaximum(5)           
        self.slider2.setValue(1)            
        self.slider2.setTickPosition(QSlider.TicksBelow)  
        self.slider2.setTickInterval(10)  

        ruta_button = QPushButton("RUTA AUTÓNOMA")
        ruta_button.setFont(font)
        ruta_button.setStyleSheet("background-color: purple;")
        ruta_button.clicked.connect(lambda: self.command_manual("ruta_autonoma"))

        emergencia_button = QPushButton("EMERGENCIA!")
        emergencia_button.setFont(font)
        emergencia_button.setStyleSheet("background-color: rgb(139, 0, 0); color: white;")
        emergencia_button.clicked.connect(lambda: self.command_manual("emergencia"))

        # Añade los botones al layout si es necesario
        column3.addWidget(slider_title)
        column3.addWidget(self.slider)
        column3.addWidget(slider_title2)
        column3.addWidget(self.slider2)
        column3.addWidget(ruta_button)
        column3.addWidget(emergencia_button)
        column3.addStretch(1)

        self.slider.valueChanged.connect(lambda value: self.slider_changed("lin", value))
        self.slider2.valueChanged.connect(lambda value: self.slider_changed("ang", value)) 

        # Add all columns to the main layout with corrected stretch factors
        layout.addLayout(column1, 3)
        layout.addWidget(self.create_vertical_separator())
        layout.addLayout(column2, 2)
        layout.addWidget(self.create_vertical_separator())
        layout.addLayout(column3, 2)
        tab.setLayout(layout)
        return tab

    def slider_changed(self, slider, value):
        vel_max = 0.7
        ang_max = 3.14
        nueva_vel = Twist()
        if slider == "lin":
            vel_nueva = vel_max * value/5
            nueva_vel.linear.x = vel_nueva
            print(vel_nueva)

        elif slider == "ang":
            ang_nueva = ang_max * value/5
            nueva_vel.angular.z = ang_nueva
            print(ang_nueva)
        
        else:
            print("ERROR")
        self.velocity_publisher.publish(nueva_vel)

        return

    def actualizar_tracker(self, x, y):
        time.sleep(0.5)
        # Verificar si self.Odom está inicializado y contiene los datos necesarios
        if self.Odom is None or not hasattr(self.Odom, 'pose') or not hasattr(self.Odom.pose, 'pose'):
            rospy.logerr("Odom no está inicializado correctamente o no contiene datos de posición.")
            return

        # Verificar si el QLabel está listo
        if self.map_display.width() == 0 or self.map_display.height() == 0:
            rospy.logerr("El QLabel no tiene dimensiones válidas.")
            return

        # Cargar la imagen base del mapa desde un archivo o atributo
        base_pixmap = QPixmap(self.base_pixmap_tracking)
        if base_pixmap.isNull():
            rospy.logerr("Error al cargar la imagen base del mapa.")
            return

        # Redimensionar el pixmap del mapa al tamaño del QLabel si es necesario
        base_pixmap = base_pixmap.scaled(self.map_display.width(),
                                        self.map_display.height(),
                                        Qt.IgnoreAspectRatio,
                                        Qt.SmoothTransformation)

        # Crear un pixmap final que ocupe todo el QLabel
        final_pixmap = QPixmap(self.map_display.width(), self.map_display.height())

        # Dibujar el mapa escalado sobre el pixmap final
        painter = QPainter(final_pixmap)
        painter.drawPixmap(0, 0, base_pixmap.width(), base_pixmap.height(), base_pixmap)

        # Obtener coordenadas desde self.Odom
        #print("posiciones:", x, y)

        # Escalar coordenadas al tamaño del QLabel
        escala_x = self.map_display.width() / 50.0
        escala_y = self.map_display.height() / 25.0

        x_mapa = escaladotrackerx - y * escala_x
        y_mapa = escaladotrackery + x * escala_y

        # Dibujar el punto en el QPixmap final
        pen = QPen(Qt.blue)
        pen.setWidth(5)
        painter.setPen(pen)
        brush = QBrush(Qt.red, Qt.SolidPattern)
        painter.setBrush(brush)
        radio = 10
        painter.drawEllipse(int(x_mapa - radio), int(y_mapa - radio), int(radio * 2), int(radio * 2))

        painter.end()

        # Asignar el pixmap final al QLabel
        self.map_display.setPixmap(final_pixmap)
        
    def toggle_mode(self):
        if self.is_dark:
            self.setStyleSheet(light_style)
            self.is_dark = False
        else:
            self.setStyleSheet(dark_style)
            self.is_dark = True

    def populate_command_log_table(self, table_widget, file_path):

        """Llenar la tabla de registro de comandos con datos del archivo CSV."""
        data = self.load_csv_data(file_path)
        
        if data is not None and not data.empty:
            # Establecer el número de filas y columnas (+1 para la columna extra 'Ejecutado')
            table_widget.setRowCount(len(data))
            table_widget.setColumnCount(len(data.columns) + 1)

            # Establecer encabezados
            headers = list(data.columns) + ["executed"]
            table_widget.setHorizontalHeaderLabels(headers)

            # Expandir columnas para llenar el espacio disponible
            table_widget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

            # Llenar la tabla con los datos
            for i in range(len(data)):
                for j in range(len(data.columns)):
                    item = QTableWidgetItem(str(data.iat[i, j]))
                    table_widget.setItem(i, j, item)

                # Columna "Ejecutado" con indicación de color
                executed_item = QTableWidgetItem()
                estado_value = data.iat[i, 1]
                if estado_value == 1:
                    executed_item.setBackground(QColor(0, 255, 0))
                    executed_item.setText("Yes")
                elif estado_value == 0:
                    executed_item.setBackground(QColor(255, 0, 0))
                    executed_item.setText("No")
                
                table_widget.setItem(i, len(data.columns), executed_item)
        else:
            # Mostrar mensaje si el CSV está vacío o no se pudo cargar
            table_widget.setRowCount(1)
            table_widget.setColumnCount(1)
            table_widget.setHorizontalHeaderLabels(["Status"])
            item = QTableWidgetItem("Sin entradas recientes")
            table_widget.setItem(0, 0, item)
        
    def clear_csv(self, table_widget):
        """Borrar el contenido del archivo CSV y actualizar la tabla."""
        try:
            # Abrir el archivo y truncar para borrar su contenido
            with open(data_directory, 'w') as file:
                file.write("entrada,estado\n")
            
            # Actualizar la tabla para reflejar el contenido borrado
            self.populate_command_log_table(table_widget, data_directory)
        except Exception as e:
            print(f"Error clearing CSV: {e}")

    def load_csv_data(self, file_path):
        """Cargar datos del archivo CSV y devolverlos como un DataFrame de pandas."""
        try:
            data = pd.read_csv(file_path)
            if data.empty:
                return None
            return data
        except Exception as e:
            return None

    def create_vertical_separator(self):
        """Creates a vertical separator."""
        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        line.setFrameShadow(QFrame.Sunken)
        return line
    
    def command_manual(self, color):
        if color == "red" :
            self.enviar_posicion("color rojo")
        elif color == "yellow":
            self.enviar_posicion("color amarillo")

        elif color == "blue":
            self.enviar_posicion("color azul")

        elif color == "green":
            self.enviar_posicion("color verde")
        elif color == "inicio":
            self.enviar_posicion("inicio")
        elif color == "emergencia":
            self.enviar_posicion("emergencia")
        elif color == "ruta_autonoma":
            self.enviar_posicion("ruta_autonoma")
        else:
            print("Color desconocido")

    def activate_voice_recognition(self):
        
        recognizer = sr.Recognizer()
        with sr.Microphone() as source:
            try:
                self.voice_status_label.setText("Escuchando comandos")
                QApplication.processEvents()
                audio = recognizer.listen(source, timeout=5)
                command = recognizer.recognize_google(audio)
                command = command.lower()
                
                self.enviar_posicion(command)
            except sr.UnknownValueError:
                self.voice_status_label.setText("No se reconoce el comando")
            except sr.RequestError as e:
                self.voice_status_label.setText(f"Error with recognition service: {e}")
            except Exception as e:
                self.voice_status_label.setText(f"Error: {e}")
        QApplication.processEvents()

    # Función para enviar un mensaje PoseStamped según el comando
    def enviar_posicion(self,comando):
        pose = ultima_pose
        # Establecer el encabezado con el frame de referencia
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "mapa"

        # Coordenadas dependiendo del lugar
        if comando == "color azul":
            self.voice_status_label.setText("Turtlebot de camino a restaurante")
            self.publicador.publish(2)
            exito = 1

        elif comando == "color rojo":
            self.voice_status_label.setText(f"Turtlebot de camino a cervecería")
            self.publicador.publish(5)
            exito = 1

        elif comando == "color verde":
            self.voice_status_label.setText(f"Turtlebot de camino a aseos")
            self.publicador.publish(4)
            exito = 1
        
        elif comando == "color amarillo":
            self.voice_status_label.setText(f"Turtlebot de camino a tienda de ropa")
            self.publicador.publish(3)
            exito = 1
        
        elif comando == "inicio":
            self.voice_status_label.setText(f"Turtlebot de camino a inicio")
            self.publicador.publish(1)
            exito = 1

        elif comando == "emergencia":
            self.voice_status_label.setText(f"Parada de emergencia")
            self.publicador.publish(6)
            exito = 1

        elif comando == "ruta_autonoma":
            self.voice_status_label.setText(f"Turtlebot realizando ruta autónoma")
            self.publicador.publish(0)
            exito = 1

        else:
            exito = 0

        # Actualizamos csv
        self.escribir_csv(comando,exito)
        self.populate_command_log_table(self.command_table,data_directory)
        # Publicar el mensaje en el topic /posicion
        self.pub_posicion.publish(pose)
        QApplication.processEvents()

    def escribir_csv(self,comando,exito):

        # Comprobar si el archivo ya existe
        if os.path.exists(data_directory):
            # Leer el archivo existente
            df = pd.read_csv(data_directory)
        else:
            # Crear un DataFrame vacío si el archivo no existe
            df = pd.DataFrame(columns=["comando", "exito"])

        # Crear una nueva entrada
        nueva_fila = pd.DataFrame({"entrada": [comando], "estado": [exito]})

        # Añadir la nueva fila al DataFrame
        df = pd.concat([df, nueva_fila], ignore_index=True)

        # Guardar el DataFrame actualizado en el archivo CSV
        df.to_csv(data_directory, index=False)

# Main execution
if __name__ == "__main__":
    app = QApplication([])
    window = TurtleInterface()
    window.show()
    app.exec_()