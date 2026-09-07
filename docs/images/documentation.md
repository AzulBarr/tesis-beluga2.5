# belugaslam_core
## include/belugaslam_core/submap.hpp
## include/belugaslam_core/particle.hpp
## include/belugaslam_core/fastslam_oc_grid_core.hpp
### BelugaSLAM()
- **Entrada:** 
- **Salida:** 

### sample_motion_model(u)
- **Entrada:** 
- **Salida:** 

### measurement_model_map(z)
- **Entrada:** 
- **Salida:** 

### update_occupancy_grid(z)
- **Entrada:** 
- **Salida:** 

### resample()
- **Entrada:** 
- **Salida:** 

### post_update(finished_events)
- **Entrada:** 
- **Salida:** 
que se encarga de disparar loop closure con los submapas recién cerrados, eligir la hipótesis de mayor peso total, correr PGO si hay nuevas restricciones y armar el mapa global para publicar.

### best_occupancy_grid()
- **Entrada:** 
- **Salida:** 

---
# belugaslam_node
## src/fastslam_oc_grid_node.cpp
### BelugaSLAMNode()
- **Entrada:** ninguna.
- **Salida:** ninguna. 
- Constructor de BelugaSLAMNode.
- Obtiene los parámetros del nodo de ROS 2.
- Llama a la función [setup_slam()](#setup_slam).
- Configura TF, suscriptores y publicadores.

### setup_slam()
- **Entrada:** ninguna.
- **Salida:** ninguna. 
- Obtiene parámetros de ROS 2 y crea una instancia de FastSLAMParams.
- Crea una instancia de los modelos de medición y movimiento.
- Construye la instancia de BelugaSLAM con el constructor [BelugaSLAM()](#belugaslam).

### laser_callback(msg)
- **Entrada:** laser scan.
- **Salida:** ninguna. 

Se activa cada vez que se recibe un scan.

Si no es la primera iteración, si hay transformada odom → base_frame y si la distancia recorrida desde la última iteración es mayor a un umbral: 

- Calcula el control de movimiento con la TF de odometría.
- Convierte el scan a puntos cartesianos en el frame del robot con [laser_to_cartesian(msg)](#laser_to_cartesianmsg).
- Ejecuta el ciclo del filtro: [sample_motion_model(u)](#sample_motion_modelu), [measurement_model_map(z)](#measurement_model_mapz), [update_occupancy_grid(z)](#update_occupancy_gridz), [post_update(finished_events)](#post_updatefinished_events) y [resample()](#resample).
- Calcula covarianza y entropía, y publica partículas, pose, mapa, marcadores y la TF map → odom con: [compute_se2_covariance()](#compute_se2_covariance), [publish_particles(stamp)](#publish_particlesstamp), [compute_entropy()](#compute_entropy), [publish_best_pose(stamp)](#publish_best_posestamp), [publish_map()](#publish_map), [publish_loop_closure_markers(stamp)](#publish_loop_closure_markersstamp), [publish_spatial_split_markers(stamp)](#publish_spatial_split_markersstamp), [publish_uncertainty_map()](#publish_uncertainty_map), [broadcast_map_to_odom(stamp, current_odom)](#broadcast_map_to_odomstamp-current_odom).
- Loguea los tiempos de cada etapa y estadísticas de partículas, hipótesis y submapas.


### laser_to_cartesian(msg)
- **Entrada:** laser scan.
- **Salida:** ninguna.
- Descarta los rayos no finitos o fuera del rango (0.1, range_max).
- Pasa cada uno de polar a cartesiano en el frame del sensor.
- Los transforma con la TF base_link → laser.

### publish_map()
- **Entrada:** ninguna.
- **Salida:** ninguna.
- Toma la grilla de ocupación de la mejor hipótesis con [best_occupancy_grid()](#best_occupancy_grid).
- Arma el mensaje `OccupancyGrid` y lo publica en `/map`.

### publish_best_pose(stamp)
- **Entrada:** timestamp del scan.
- **Salida:** 
Empaqueta la mejor pose del filtro (`slam_->best_pose()`) en un `PoseWithCovarianceStamped` en el frame map, mapea la covarianza SE(2) de 3x3 a la matriz 6x6 de ROS (con 1e-6 en las diagonales de z, roll y pitch para que no sea singular) y la publica, acumulando además la pose en el Path de trayectoria si publish_trajectory está habilitado.

### compute_se2_covariance()
- **Entrada:** 
- **Salida:** 

### publish_particles(stamp)
- **Entrada:** 
- **Salida:** 

### compute_entropy()
- **Entrada:** 
- **Salida:** 

### publish_loop_closure_markers(stamp)
- **Entrada:** 
- **Salida:** 

### publish_spatial_split_markers(stamp)
- **Entrada:** 
- **Salida:** 

### publish_uncertainty_map()
- **Entrada:** 
- **Salida:** 

### broadcast_map_to_odom(stamp, current_odom)
- **Entrada:** 
- **Salida:** 
