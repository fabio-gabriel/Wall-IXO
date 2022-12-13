# Wall-IXO

O projeto do Wall-IXO é de um robô aspirador de pó, semelhante a um Roomba, que consiga detectar obstáculos e evitar colisões. Esse robô utiliza sensores ultrassônicos para detectar a distância dos obstáculos e age de acordo.

## Organização do projeto

O projeto é separado em duas frentes principais:

- Módulo Sensorial (Wildnei Frank, Fábio Gabriel)
- Módulo Motor (Carlos Matheus, Bruna Stefanie)

# Módulo Sensorial

Esse módulo é responsável pelos sensores utilizados pelo Wall-IXO. Utilizam-se 3 sensores ultrassônicos do tipo HC-SR04. O greenpill, quando envia um sinal para o pino TRIGGER, recebe uma resposta do sensor por meio do pino ECHO, e por meio dessa resposta é possível calcular a distância do sensor até o obstáculo.

![Módulo Sensorial](/modulosensorial.png)

![Sensor HC-SR04](/sensor.png)

# Módulo Motor

![Módulo Motor](/modulomotor.png)
