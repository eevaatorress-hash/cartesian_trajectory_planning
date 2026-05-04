# cartesian_trajectory_planning
Este es el repositorio para el Lab1 y Lab2 de ampliación de robótica de manipuladores.

# Ejercicio 1
## Fundamento teórico

Para mover el EE en línea recta entre dos puntos $x_0$ y $x_1$, se utiliza una ley temporal basada en un parámetro $\lambda(t) \in [0,1]$, siendo $\lambda(0)=0$ al inicio del movimiento y $\lambda(t_f)=1$ al final.

La interpolación se realiza con la aplicación de la siguiente fórmula:

$x(t)=x_1-\lambda(t)(x_1-x_0)$

## Aplicación práctica

Se da la siguiente información para realizar la interpolación:
~~~
std::pair<tf2::Vector3, tf2::Quaternion> PoseInterpolation(
    const Eigen::Matrix4d &start_pose,
    const Eigen::Matrix4d &end_pose,
    double lambda)
~~~

Se definen todas las variables que se van a utilizar:
~~~
Eigen::Matrix4d pos_interp;    // Posición en función del tiempo y lambda
Eigen::Matrix3d orientacion;   // Orientación en función del tiempo y lambda
tf2::Vector3 p_interp;    // Placeholder for the interpolated position
tf2::Quaternion q_interp; // Placeholder for the interpolated quaternion
~~~

Se aplica la fórmula de intepolación de poses en el espacio cartesiano:
~~~
pos_interp = start_pose + lambda*(end_pose - start_pose);
~~~

Se obtiene la posición y orientación cartesianas a partir de la matriz `pos_interp`:
~~~
p_interp = tf2::Vector3(
    pos_interp(0,3),
    pos_interp(1,3),
    pos_interp(2,3));
        
orientacion << pos_interp(0,0), pos_interp(0,1), pos_interp(0,2),
    pos_interp(1,0), pos_interp(1,1), pos_interp(1,2),
    pos_interp(2,0), pos_interp(2,1), pos_interp(2,2);
~~~

Finalmente la matriz de rotación se transforma en un cuaternión y se devuelve la posición y el cuaternión que representan la pose interpolada:
~~~
q_interp = rot2Quat(orientacion);

return {p_interp, q_interp};
~~~

## Resultados
Tras lanzar el comando `ros2 launch cartesian_trajectory_planning send_trajectory.launch.py` en la terminal, se obtiene el resultado esperado:

![Resultado de la interpolación](/images/resultado_Lab1.png)


# Ejercicio 2
## Fundamento teórico
Cuando se concatenan múltiples trayectorias cartesianas que pasan por distintos puntos en el espacio, se quiere evitar las discontinuidades en velocidades las cuáles pueden resultar en aceleraciones elevadas. Es por ello, que se utilizará el suavizado de trayectorias por puntos intermedios y que ayudarán a conseguir lineas continuas en la velocidad durante esos tramos. Esto se puede observar en la siguiente figura:
![Consecuencias de suavizado en velocidad](/images/smooth_trajectory.png)

En este ejercicio se dividirá la trayectoria en los tres tramos que se observaban en la figura anterior. El primer y último tramo usarán la interpolación de la pose planteada en el ejercicio anterior, mientras que el tramo intermedio seguirá la función del suavizado.

Para la <ins>posición</ins> simplemente se aplicará la siguiente fórmula:

$p(t) = \boldsymbol{p}_1 - \frac{(\tau - t)^2}{4\tau T_1}\Delta \boldsymbol{p}_1 + \frac{(\tau + t)^2}{4\tau T_2}\Delta \boldsymbol{p}_2$

Para la <ins>orientación</ins> se tendrán que tener en cuenta las suavizaciones de tanto $[-\tau, 0]$ y de $[0, \tau]$ con la multiplicación de $qk_1$ y $qk_2$ respectivamente.

$\boldsymbol{q}(t) = \boldsymbol{q}_1 \boldsymbol{qk}_1 \boldsymbol{qk}_2$

Como se puede observar para obtener el cuaternio interpolado primero hemos de calcular $\boldsymbol{qk}_1$ y $\boldsymbol{qk}_2$, se aplicarán las siguientes ecuaciones:

$q_{01} = q_0^{-1} \cdot q_1$

$\theta_{01} = 2\arccos(w_{01})$

$$\mathbf{n}_{01} = \frac{\mathbf{v}_{01}}{\sin(\theta_{01}/2)}$$


$q_{12} = q_1^{-1} \cdot q_2$

$\theta_{12} = 2\arccos(w_{12})$

$\mathbf{n}_{12} = \frac{\mathbf{v}_{12}}{\sin(\theta_{12}/2)}$
