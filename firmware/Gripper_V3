#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <kuka_interfaces/msg/gripper_command.h>

// CONFIGURACIÓN DE HARDWARE
#define SERVO1_ID 1
#define SERVO2_ID 2
#define DIR_PIN 4
#define RXD1 16
#define TXD1 17
#define SERVO_BAUD 57600
HardwareSerial ServoSerial(1);

#define POS_ABIERTO   1000  
#define MAX_RECORRIDO 950 

// Variables de micro-ROS
rcl_node_t node;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

kuka_interfaces__msg__GripperCommand pub_msg;
kuka_interfaces__msg__GripperCommand sub_msg;

// Funciones Dynamixel
void enableTorque(uint8_t id);
void writeData(uint8_t id, uint8_t addr, uint8_t value);
void syncWritePosition(uint16_t pos1, uint16_t pos2);
void syncWriteTorque(uint16_t torque1, uint16_t torque2);

// CALLBACK DE COMANDO
void subscription_callback(const void * msgin)
{
  const kuka_interfaces__msg__GripperCommand * msg = (const kuka_interfaces__msg__GripperCommand *)msgin;

  // 1. Mapeo de Fuerza: Subimos el rango para que realmente tenga torque
  // El MX-28 llega a 1023.
  uint16_t fuerza_real = map(msg->force, 0, 100, 700, 1023);

  // 2. Procesamiento del Ángulo
  int32_t val_ros = msg->angle;
  if (val_ros > MAX_RECORRIDO) val_ros = MAX_RECORRIDO;
  if (val_ros < 0) val_ros = 0;

  // LÓGICA DE ESPEJO 
  // Motor 1: Resta del inicio (gira en un sentido)
  // Motor 2: Suma al inicio (gira en sentido opuesto)
  int32_t destino1 = POS_ABIERTO - val_ros;
  int32_t destino2 = POS_ABIERTO + val_ros;

  // 3. Candados de Seguridad Absolutos
  // Evitamos que el motor 1 baje de 50 (peligro de desenredo total)
  // Evitamos que el motor 2 suba de 2000 (evitar vueltas innecesarias)
  destino1 = constrain(destino1, 50, 1200);
  destino2 = constrain(destino2, 50, 2000);

  // 4. Envío a los Motores
  syncWriteTorque(fuerza_real, fuerza_real);
  syncWritePosition((uint16_t)destino1, (uint16_t)destino2);

  // Monitor Serial para ver qué está pasando
  Serial.print("ROS: "); Serial.print(val_ros);
  Serial.print(" | Destino M1: "); Serial.print(destino1);
  Serial.print(" | Destino M2: "); Serial.println(destino2);

  // Confirmación
  pub_msg.angle = msg->angle;
  pub_msg.force = msg->force;
  pub_msg.status = true; 
  rcl_publish(&publisher, &pub_msg, NULL);
}

void setup()
{
  Serial.begin(115200);
  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, RXD1, TXD1);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

  delay(500);

  // Encendido de Torque
  enableTorque(SERVO1_ID);
  enableTorque(SERVO2_ID);

  // POSICIÓN DE SEGURIDAD AL ARRANCAR
  // Ambos motores van a 1000 para tensar hilos y esperar órdenes.
  syncWriteTorque(800, 800);
  syncWritePosition(POS_ABIERTO, POS_ABIERTO);
  
  delay(500);

  // Inicialización de micro-ROS
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);

  rclc_publisher_init_default(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(kuka_interfaces, msg, GripperCommand),
    "esp32_pub");

  rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(kuka_interfaces, msg, GripperCommand),
    "esp32_sub");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA);

  Serial.println("Sistema de Rango Extendido Iniciado");
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);
}

// FUNCIONES INTERNAS DYNAMIXEL

void enableTorque(uint8_t id) {
  writeData(id, 24, 1);
  delay(50);
}

void writeData(uint8_t id, uint8_t addr, uint8_t value) {
  uint8_t packet[8] = {0xFF, 0xFF, id, 4, 3, addr, value, 0};
  uint8_t checksum = ~(id + 4 + 3 + addr + value);
  packet[7] = checksum;
  digitalWrite(DIR_PIN, HIGH);
  ServoSerial.write(packet, 8);
  ServoSerial.flush();
  digitalWrite(DIR_PIN, LOW);
}

void syncWritePosition(uint16_t pos1, uint16_t pos2) {
  uint8_t packet[14] = {0xFF, 0xFF, 0xFE, 10, 0x83, 30, 2, 
                        SERVO1_ID, (uint8_t)(pos1 & 0xFF), (uint8_t)(pos1 >> 8),
                        SERVO2_ID, (uint8_t)(pos2 & 0xFF), (uint8_t)(pos2 >> 8), 0};
  uint8_t checksum = 0;
  for (int i = 2; i < 13; i++) checksum += packet[i];
  packet[13] = ~checksum;
  digitalWrite(DIR_PIN, HIGH);
  ServoSerial.write(packet, 14);
  ServoSerial.flush();
  digitalWrite(DIR_PIN, LOW);
}

void syncWriteTorque(uint16_t torque1, uint16_t torque2) {
  uint8_t packet[14] = {0xFF, 0xFF, 0xFE, 10, 0x83, 14, 2, 
                        SERVO1_ID, (uint8_t)(torque1 & 0xFF), (uint8_t)(torque1 >> 8),
                        SERVO2_ID, (uint8_t)(torque2 & 0xFF), (uint8_t)(torque2 >> 8), 0};
  uint8_t checksum = 0;
  for (int i = 2; i < 13; i++) checksum += packet[i];
  packet[13] = ~checksum;
  digitalWrite(DIR_PIN, HIGH);
  ServoSerial.write(packet, 14);
  ServoSerial.flush();
  digitalWrite(DIR_PIN, LOW);
}
