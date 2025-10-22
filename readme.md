# Instruções do Pacote de Comunicação ROS

## Visão Geral
Este pacote contém três nós principais:

1. **Nó de Processamento**: Lê ângulos das juntas e estados das mãos de um arquivo JSON e publica em tópicos ROS.
2. **Controlador da Mão**: Controla a mão esquerda usando motores Dynamixel, inscrito nos tópicos de estado da mão.
3. **Controlador PCA da Mão**: Controla a mão direita usando servomotores através do PCA9685, inscrito nos tópicos de estado da mão.


## Executando o Sistema

1. Primeiro, carregue sua instalação do ROS2:
```bash
source /opt/ros/humble/setup.bash
```

2. Compile os pacotes:
```bash
cd fei-atena-tcc
colcon build
```

3. Carregue o workspace:
```bash
source install/setup.bash
```

## Tópicos Disponíveis

### Pacote de Comunicação
Para listar todos os tópicos disponíveis:
```bash
ros2 topic list
```

### Tópicos Específicos dos Nós:

#### Tópicos do Nó de Processamento (Publishers):
- `/right/hand_joint` - Estado da mão direita (Bool)
- `/left/hand_joint` - Estado da mão esquerda (Bool)
- `/left/shoulder_pitch` - Ângulo de inclinação do ombro esquerdo (Int32)
- `/left/shoulder_roll` - Ângulo de rotação do ombro esquerdo (Int32)
- `/left/shoulder_yaw` - Ângulo de guinada do ombro esquerdo (Int32)
- `/left/elbow_pitch` - Ângulo de inclinação do cotovelo esquerdo (Int32)
- `/right/shoulder_pitch` - Ângulo de inclinação do ombro direito (Int32)
- `/right/shoulder_roll` - Ângulo de rotação do ombro direito (Int32)
- `/right/shoulder_yaw` - Ângulo de guinada do ombro direito (Int32)
- `/right/elbow_pitch` - Ângulo de inclinação do cotovelo direito (Int32)

#### Tópicos do Controlador da Mão (Subscribers):
- `/left/hand_joint` - Controla o motor Dynamixel da mão esquerda

#### Tópicos do Controlador PCA (Subscribers):
- `/right/hand_joint` - Controla os servomotores da mão direita

### Visualizando Dados dos Tópicos
Para visualizar dados de um tópico específico:
```bash
ros2 topic echo <nome_do_topico>
```

Exemplo:
```bash
ros2 topic echo /cmd_vel
```

### Informações do Tópico
Para obter informações detalhadas sobre um tópico:
```bash
ros2 topic info <nome_do_topico>
```

## Running Nodes
To run specific nodes:

1. Navigation:
```bash
ros2 launch nav2_bringup navigation_launch.xml
```

2. SLAM:
```bash
ros2 launch slam_toolbox online_async_launch.py
```

## Monitoring
To visualize the robot and sensor data:
```bash
ros2 run rviz2 rviz2
```

## Executando Nós Individuais

### 1. Nó de Processamento
Este nó lê ângulos das juntas e estados das mãos de um arquivo JSON (`/home/{user}/Documents/recebido.json`) e os publica em tópicos ROS:
```bash
ros2 run communication_pkg processamento_node
```

### 2. Controlador da Mão (Mão Esquerda)
Controla o motor Dynamixel da mão esquerda. Parâmetros padrão:
- Porta: /dev/ttyUSB0
- Taxa de transmissão: 1000000
- Protocolo: 2.0
- ID do Motor: 9

Para executar com parâmetros padrão:
```bash
ros2 run controller_pkg hand_controller
```

Para executar com parâmetros personalizados:
```bash
ros2 run controller_pkg hand_controller --ros-args -p devicename:=/dev/ttyUSB1 -p baudrate:=57600 -p dxl_id:=1
```

### 3. Controlador PCA da Mão (Mão Direita)
Controla os servomotores da mão direita usando PCA9685:
```bash
ros2 run controller_pkg pca_hand_controller
```

## Testando o Sistema

1. Para testar os estados das mãos, você pode publicar nos tópicos das mãos:
```bash
# Testar mão esquerda (True = aberta, False = fechada)
ros2 topic pub /left/hand_joint std_msgs/msg/Bool "data: true"

# Testar mão direita
ros2 topic pub /right/hand_joint std_msgs/msg/Bool "data: true"
```

2. Para monitorar os ângulos das juntas e estados das mãos:
```bash
# Monitorar estado da mão esquerda
ros2 topic echo /left/hand_joint

# Monitorar estado da mão direita
ros2 topic echo /right/hand_joint

# Monitorar ângulos das juntas (exemplo para inclinação do ombro esquerdo)
ros2 topic echo /left/shoulder_pitch
```
