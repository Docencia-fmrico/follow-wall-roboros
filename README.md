[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=6883680&assignment_repo_type=AssignmentRepo)

[![GitHub Action
Status](https://github.com/Docencia-fmrico/follow-wall-roboros/workflows/main/badge.svg)](https://github.com/Docencia-fmrico/follow-wall-roboros)


# folllow_wall

Hola, este es el paquete de follow-wall del grupo Roboros de la asignatura Planificación de Sistemas Cognitivos del Grado de Robótica Software de la universidad URJC.Gracias a este paquete software podras conseguir un comportamiento en el robot TiaGo para que recorra la habitacion de un recinto cerrado pegado a la pared utilizando exclusivamente el sensor laser integrado en el robot.

## Respuesta al laser

La base de el programa es como manejamos la informacion que nos aporta un subscriptor que hemos usado para obetener los datos del laser,el suscriptor va siempre a 60hz independiente de la frecuencia del laser.Dividimos todo el rango del laser en 3 zonas con 2 partes cada una, con eso ayudamos a la reactividad del robot a la hora de tomar decisiones.

Las zonas graficamente se verian tal que asi. Seria una combinacion entre derecha, centro e izquierda y pueden ser la exterior o la interior.
![image](https://user-images.githubusercontent.com/78978241/153917108-8aef5705-0275-425d-8557-bb959f6119cf.png)

Con eso, usando un valor como referencia vamos a ver si ha detectado los suficientes puntos en cada zona para tenerla en cuenta a la hora de tomar una decisión

## Publicando la velocidad

Una vez evaluamos la info del laser lo que hacemos es publicar la velocidad a través de un algoritmo basado en casos, que deciden el valor del topic que recibira TiaGo para la velocidad.

enum actions
{
  TURN_RIGHT,
  TURN_LEFT,
  MOVING_TURN_RIGHT,
  MOVING_TURN_LEFT,
  CONTINUE,
  STOP
};

## Estructura del nodo
 
El programa se basa en un único nodo lifecycle que es el que se suscribe al laser y publica la velocidad

![image](https://user-images.githubusercontent.com/78978241/153957808-719a9b15-e0c1-4e42-bb4f-094b7dedd0a0.png)

## Video demostrativo


[Video](https://urjc-my.sharepoint.com/:v:/g/personal/a_cabreram_2019_alumnos_urjc_es/EY-knV94el1OmfO9Hn5FxBgB7fKtD-otc88CP5NSCffH_w?e=fomcNP)


#2. Añadiendo soporte a kobuki en real  

Añadimos parametros para ajustar facilmente diferentes robots.
Añadimos dos launcher, uno con simulador y los parametros de tiago, y otro con los parametros del kobuki y su remapping.


