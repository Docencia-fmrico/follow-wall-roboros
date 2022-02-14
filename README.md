[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=6883680&assignment_repo_type=AssignmentRepo)

[![GitHub Action
Status](https://github.com/Docencia-fmrico/follow-wall-roboros/workflows/main/badge.svg)](https://github.com/Docencia-fmrico/follow-wall-roboros)


# folllow_wall

Hola, este es el paquete de follow-wall del grupo Roboros de la asignatura Planificación de Sistemas Cognitivos del Grado de Robótica Software de la universidad URJC.Gracias a este paquete software podras conseguir un comportamiento en el robot TiaGo para que recorra la habitacion de un recinto cerrado pegado a la pared utilizando exclusivamente el sensor laser integrado en el robot.

## Funcionamiento de el lifecycle node

La base de el programa es como manejamos la informacion que nos aporta un subscriptor que hemos usado para obetener los datos del laser, dividimos todo el rango del laser en 3 zonas con 2 partes cada una, con eso ayudamos a la reactividad del robot a la hora de tomar decisiones.

Las zonas graficamente se verian tal q asi. Seria una combinacion entre derecha, centro e izquierda y pueden ser la exterior o la interior.
![image](https://user-images.githubusercontent.com/78978241/153916855-e4f79597-ae2b-4f19-a227-0d33c4334655.png)
