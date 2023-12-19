# Autonomous-robot


## lab5.py

### Visão Geral

O script `lab5.py` controla uma tartaruga simulada em um ambiente ROS. Ele implementa um controlador proporcional para ajustar a velocidade angular da tartaruga, permitindo que ela se alinhe com uma posição de destino especificada. Além disso, um controlador de velocidade linear move a tartaruga em direção ao objetivo com ajustes proporcionais.

### Uso

1. Certifique-se de que o ROS está instalado e inicializado.
2. Execute o script usando o comando:

    ```bash
    ros2 run <nome_pacote> lab5.py
    ```

3. Defina um novo objetivo para a tartaruga publicando uma mensagem `Pose2D` no tópico `/goal`.

## lab6.py

### Visão Geral

O script `lab6.py` controla autonomamente um robô em um ambiente ROS. Ele utiliza controle proporcional para ajustar as velocidades linear e angular, permitindo que o robô navegue em direção a uma posição de destino especificada. O script define automaticamente um objetivo inicial, e você pode atualizá-lo publicando uma mensagem `Pose2D` no tópico `/goal`.

### Uso

1. Certifique-se de que o ROS está instalado e inicializado.
2. Execute o script usando o comando:

    ```bash
    ros2 run <nome_pacote> lab6.py
    ```

3. O script define automaticamente um objetivo inicial, e você pode atualizá-lo publicando uma mensagem `Pose2D` no tópico `/goal` **(NÃO IMPLEMENTADO AINDA)**.
