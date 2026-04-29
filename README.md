# cartesian_trajectory_planning

Este es el repositorio para el Lab1 de ampliación de robótica de manipuladores

## Fundamento teórico

Para mover el EE en línea recta entre dos puntos $p_0$ y $p_1$, se utiliza una ley temporal basada en un parámetro $\lambda(t) \in [0,1]$, siendo $\lambda(0)=0$ al inicio del movimiento y $\lambda(t_f)=1$ al final.

La interpolación se realiza con la palicación de la siguiente fórmula:

$p(t)=p_1-\lambda(t)(p_1-p_0)$

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