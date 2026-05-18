# cartesian_trajectory_planning
Este es el repositorio para el Lab1 de ampliación de robótica de manipuladores.

# Ejercicio 1
## Fundamento teórico

Para mover el EE en línea recta entre dos puntos $\boldsymbol{p}_0$ y $\boldsymbol{p}_1$, se utiliza una ley temporal basada en un parámetro $\lambda(t) \in [0,1]$, siendo $\lambda(0)=0$ al inicio del movimiento y $\lambda(t_f)=1$ al final.

La interpolación de la posición se realiza con la aplicación de la siguiente fórmula:

$$\boldsymbol{p}(t)=\boldsymbol{p}_1-\lambda(t)(\boldsymbol{p}_1-\boldsymbol{p}_0)$$

La rotación para pasar de un cuaternión inicial a otro final, viene dada por otro cuaternio:

$$\boldsymbol{q}_0 \cdot \boldsymbol{q}_c = \boldsymbol{q}_1 \Rightarrow \boldsymbol{q}_c = \boldsymbol{q}_0^{-1} \cdot \boldsymbol{q}_1=[w_c,\boldsymbol{v}_c]$$

Esto se puede representar como un giro $\theta$ sobre los ejes $\boldsymbol{n}$:

$$\theta = 2acos(w_c)$$
$$\boldsymbol{n} = \frac{\boldsymbol{v}_c}{sin(\theta/2)}$$

Interpolamos $\theta$ y obtenemos el cuaternión interpolado de la sigueinte forma:

$$\theta_{\lambda}=\lambda(t)\theta$$
$$w_{rot}=cos(\frac{\theta_{\lambda}}{2})$$
$$\boldsymbol{v}_{rot}=\boldsymbol{n} \cdot sin(\frac{\theta_{\lambda}}{2})$$
$$\boldsymbol{q}_{\lambda}=\boldsymbol{q}_0 \cdot \boldsymbol{q}_{rot}$$

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
Eigen::Matrix3d R_start = start_pose.block<3,3>(0,0); // Matriz de rotación del pose inicial
Eigen::Matrix3d R_end = end_pose.block<3,3>(0,0); // Matriz de rotación del pose final
tf2::Vector3 p_interp;    // Placeholder for the interpolated position
tf2::Quaternion q_interp; // Placeholder for the interpolated quaternion
tf2::Vector3 start_p = tf2::Vector3(    // Posición inicial
    start_pose(0,3),        
    start_pose(1,3),
    start_pose(2,3));
tf2::Vector3 end_p = tf2::Vector3(      // Posición final
    end_pose(0,3),
    end_pose(1,3),
    end_pose(2,3));
~~~

Se aplica la fórmula de intepolación de posiciones en el espacio cartesiano:
~~~
p_interp = start_p + lambda*(end_p - start_p); // Posición interpolada usando interpolación lineal
~~~

Se calcula la orientación interpolada:
~~~
tf2::Quaternion q_c = InverseQuaternion(rot2Quat(R_start)) * rot2Quat(R_end);
~~~

Se obtiene el ángulo $\theta$ interpolado sobre los ejes $\boldsymbol{n}$:
~~~
tf2::Quaternion q_c = InverseQuaternion(rot2Quat(R_start)) * rot2Quat(R_end); // Rotación interpolada 
double theta = 2*acos(q_c[3]);
tf2::Vector3 n = tf2::Vector3(q_c[0], q_c[1], q_c[2]) / sin(theta/2);
double theta_interp = lambda*theta;
~~~

Finalmente se calcula el cuaternión interpolado y se devuelven los resultados:
~~~
double s = sin(theta_interp/2);
tf2::Quaternion q_rot = tf2::Quaternion(
    n.x() * s,
    n.y() * s,
    n.z() * s,
    cos(theta_interp/2)
);
q_interp = MuliplyQuaternions(rot2Quat(R_start), q_rot); // Rotación interpolada 
q_interp.normalize();

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

$$
\boldsymbol{p}(t) = \boldsymbol{p}_1 - \frac{(\tau - t)^2}{4\tau T_1}\Delta \boldsymbol{p}_1 + \frac{(\tau + t)^2}{4\tau T_2}\Delta \boldsymbol{p}_2
$$

Para la <ins>orientación</ins> se tendrán que tener en cuenta las suavizaciones de tanto $[-\tau, 0]$ y de $[0, \tau]$ con la multiplicación de $qk_1$ y $qk_2$ respectivamente.

$$
\boldsymbol{q}(t) = \boldsymbol{q}_1 \boldsymbol{qk}_1 \boldsymbol{qk}_2
$$

Como se puede observar para obtener el cuaternio interpolado primero hemos de calcular $\boldsymbol{qk}_1$ y $\boldsymbol{qk}_2$, se aplicarán las siguientes ecuaciones:

$$
\boldsymbol{q}_{01} = \boldsymbol{q}_{01}^{-1} \cdot \boldsymbol{q}_1
$$

$$
\theta_{01} = 2\arccos(w_{01})
$$

$$
\boldsymbol{n}_{01} = \frac{\boldsymbol{v}_{01}}{\sin(\theta_{01}/2)}
$$



$$
\boldsymbol{q}_{12} = \boldsymbol{q}_{1}^{-1} \cdot \boldsymbol{q}_2
$$

$$
\theta_{12} = 2\arccos(w_{12})
$$

$$
\boldsymbol{n}_{12} = \frac{\boldsymbol{v}_{12}}{\sin(\theta_{12}/2)}
$$

Entonces ya se pueden calcular los cuaternios $\boldsymbol{qk}_1$ y $\boldsymbol{qk}_2$.

$$
\theta_{k_1} = \frac{-(\tau - t)^2}{4\tau T_1} \theta_{01}   
\Rightarrow 
\boldsymbol{q}_{k_1} = 
\left[
cos\left(\frac{\theta_{k_1}}{2}\right),\quad 
\boldsymbol{n}_{01} \cdot sin\left(\frac{\theta_{k_1}}{2}\right)
\right]
$$


$$
\theta_{k_2} = \frac{(\tau + t)^2}{4\tau T_1}\theta_{12}
\Rightarrow
\boldsymbol{q}_{k_2} =
\left[
cos\left(\frac{\theta_{k_2}}{2}\right),\quad
\boldsymbol{n}_{12} \cdot sin\left(\frac{\theta_{k_2}}{2}\right)
\right]
$$


## Aplicación práctica
En la práctica se definen las siguientes variables:
~~~
tf2::Vector3 p_interp; // Placeholder for the interpolated position
tf2::Quaternion q_interp;    // Placeholder for the interpolated quaternion

tf2::Vector3 p0 = tf2::Vector3(
    pose_0(0,3),
    pose_0(1,3),
    pose_0(2,3));

tf2::Vector3 p1 = tf2::Vector3(
    pose_1(0,3),
    pose_1(1,3),
    pose_1(2,3));

tf2::Vector3 p2 = tf2::Vector3(
    pose_2(0,3),
    pose_2(1,3),
    pose_2(2,3));

tf2::Quaternion q01;
tf2::Quaternion q12;
tf2::Quaternion qk1;
tf2::Quaternion qk2;
    
Eigen::Matrix3d orientacion0;
orientacion0 << pose_0(0,0), pose_0(0,1), pose_0(0,2),
    pose_0(1,0), pose_0(1,1), pose_0(1,2),
    pose_0(2,0), pose_0(2,1), pose_0(2,2);
tf2::Quaternion q0 = rot2Quat(orientacion0);

Eigen::Matrix3d orientacion1;
orientacion1 << pose_1(0,0), pose_1(0,1), pose_1(0,2),
    pose_1(1,0), pose_1(1,1), pose_1(1,2),
    pose_1(2,0), pose_1(2,1), pose_1(2,2);
tf2::Quaternion q1 = rot2Quat(orientacion1);

Eigen::Matrix3d orientacion2;
orientacion2 << pose_2(0,0), pose_2(0,1), pose_2(0,2),
    pose_2(1,0), pose_2(1,1), pose_2(1,2),
    pose_2(2,0), pose_2(2,1), pose_2(2,2);
tf2::Quaternion q2 = rot2Quat(orientacion2);

double theta01;
double thetak1;
double theta12;
double thetak2;
tf2::Vector3 deltap1;
tf2::Vector3 deltap2;
tf2::Vector3 n01;
tf2::Vector3 n12;
~~~

Se calcula la posición interpolada con:
~~~
p_interp = p1 - deltap1*std::pow((tau - t),2)/(4*tau*T) + deltap2*std::pow((tau + t),2)/(4*tau*T);
~~~

Y finalmente la orientación interpolada con:
~~~
q01 = MuliplyQuaternions(InverseQuaternion(q0),q1);
theta01 = 2*acos(q01[3]);
n01 = tf2::Vector3(q01[0], q01[1], q01[2]) / sin(theta01/2);

thetak1 = -std::pow((tau - t),2)*theta01/(4*tau*T);
            
double s1 = sin(thetak1/2);

qk1 = tf2::Quaternion(     
    n01.x() * s1,
    n01.y() * s1,
    n01.z() * s1,
    cos(thetak1/2)
);

q12 = MuliplyQuaternions(InverseQuaternion(q1),q2);
theta12 = 2*acos(q12[3]);
n12 = tf2::Vector3(q12[0], q12[1], q12[2]) / sin(theta12/2);

thetak2 = std::pow((tau + t),2)*theta12/(4*tau*T);
            
double s2 = sin(thetak2/2);

qk2 = tf2::Quaternion(
    n12.x() * s2,
    n12.y() * s2,
    n12.z() * s2,
    cos(thetak2/2)
);

deltap1 = p1 - p0;
deltap2 = p2 - p1;

q_interp = MuliplyQuaternions(MuliplyQuaternions(q1, qk1), qk2);
~~~


## Resultados
Tras lanzar el comando `ros2 launch cartesian_trajectory_planning r6bot_controller.launch.py` y `ros2 launch cartesian_trajectory_planning send_trajectory.launch.py`, se obtiene el resultado esperado:
![Resultado del suavizado](/images/Trayectoria_suavizada.png)

También se pueden obtener los resultados vistos en las componentes de posición (X,Y,Z) y en las componentes de la orientación (ROLL,PITCH,YAW), todo esto usando el comando `python3 plot_data.py` en la terminal.
![Resultado del suavizado en posición](/images/Trayectorias_posición.png)
![Resultado del suavizado en orientación](/images/Trayectorias_orientación.png)
