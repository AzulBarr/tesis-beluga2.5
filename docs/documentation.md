# belugaslam_core

## [derived_cache.hpp](../belugaslam_core/include/belugaslam_core/derived_cache.hpp)

## [fastslam_oc_grid_core.hpp](../belugaslam_core/include/belugaslam_core/fastslam_oc_grid_core.hpp)
### BelugaSLAM()
- **Entrada:** 
- **Salida:** 

### sample_motion_model(u)
- **Entrada:** 
- **Salida:** 

### measurement_model_map(z)
- **Entrada:** 
- **Salida:** 

### update_occupancy_grid(z, stamp)
- **Entrada:** 
- **Salida:** 

### resample()
- **Entrada:** 
- **Salida:** 

### post_update(finished_events)
- **Entrada:** 
- **Salida:** 

que se encarga de disparar loop closure con los submapas recién cerrados, eligir la hipótesis de mayor peso total, correr PGO si hay nuevas restricciones y armar el mapa global para publicar.
<!-- 
z no se usa: línea 697 es literalmente (void)z. El parámetro está en la firma pero descartado, así que la medición no interviene acá.
Los pasos 1 y 4 están detrás de #if BELUGASLAM_ENABLE_LOOP_CLOSURE. Con loop closure deshabilitado, post_update() se reduce a elegir hipótesis, fijar pose y componer el mapa.
Menor: en la línea 734 se chequea best_hypothesis && antes de usarlo, pero en 750 se hace best_hypothesis->submaps sin chequear. En la práctica nunca es nulo (el id se inicializa desde hypotheses_.front()), pero las dos líneas se contradicen sobre si puede serlo.
-->

### best_occupancy_grid()
- **Entrada:** 
- **Salida:** 

## [grid_config.hpp](../belugaslam_core/include/belugaslam_core/grid_config.hpp)

## [grid_update.hpp](../belugaslam_core/include/belugaslam_core/grid_update.hpp)

## [loop_belief.hpp](../belugaslam_core/include/belugaslam_core/loop_belief.hpp)

## [motion_filter.hpp](../belugaslam_core/include/belugaslam_core/motion_filter.hpp)

## [particle_proposal.hpp](../belugaslam_core/include/belugaslam_core/particle_proposal.hpp)

## [particle.hpp](../belugaslam_core/include/belugaslam_core/particle.hpp)

## [robust_tracking.hpp](../belugaslam_core/include/belugaslam_core/robust_tracking.hpp)

## [submap.hpp](../belugaslam_core/include/belugaslam_core/submap.hpp)

---
# belugaslam_node
## [fastslam_oc_grid_node.cpp](../belugaslam_node/src/fastslam_oc_grid_node.cpp)
### BelugaSLAMNode()
- Constructor de BelugaSLAMNode.
- Obtiene los parámetros del nodo de ROS 2.
- Llama a la función [setup_slam()](#setup_slam).
- Configura TF, suscriptores y publicadores.

### setup_slam()
- **Entrada:** ninguna.
- **Salida:** ninguna. 
- Obtiene parámetros de ROS 2, los chequea y crea una instancia de FastSLAMParams.
- Crea una instancia de los modelos de medición y movimiento.
- Construye la instancia de BelugaSLAM con el constructor [BelugaSLAM()](#belugaslam).

### laser_callback(msg)
- **Entrada:** laser scan.
- **Salida:** ninguna. 

Se activa cada vez que se recibe un scan. Registra métricas de latencia y descarta el scan si llega fuera de orden, si no hay transformada odom → base_frame o si queda sin puntos válidos.

- Calcula el control de movimiento con la TF de odometría.
- Convierte el scan a puntos cartesianos en el frame del robot con [laser_to_cartesian(msg, current_odom)](#laser_to_cartesianmsg-current_odom), que además corrige la distorsión por movimiento (deskew).
- Ejecuta el ciclo del filtro: [sample_motion_model(u)](#sample_motion_modelu), [measurement_model_map(z)](#measurement_model_mapz), [update_occupancy_grid(z, stamp)](#update_occupancy_gridz-stamp), [post_update(finished_events)](#post_updatefinished_events) y [resample()](#resample).
- Mide la innovación de la pose de salida (diferencia entre la pose predicha por odometría y la que devuelve el filtro) y detecta si cambió la hipótesis ganadora.
- Calcula la covarianza y publica pose y TF con [compute_se2_covariance()](#compute_se2_covariance), [publish_best_pose(stamp)](#publish_best_posestamp) y [broadcast_map_to_odom(stamp, current_odom)](#broadcast_map_to_odomstamp-current_odom).
- Registra los tiempos de cada etapa y los contadores en el CSV de performance con [record_performance(stamp, status, start, timing)](#record_performancestamp-status-start-timing).

<!--
El umbral de movimiento (min_update_distance / min_update_angle) ya no se aplica acá: se movió al core, a motion_filter_accepts().
El mapa, las partículas, la entropía, la trayectoria y los marcadores ya no se publican en el callback; los publica publish_visualization() desde un wall timer aparte.
Los cuatro caminos de salida (out_of_order, empty_scan, tf_error, processed) quedan registrados en el CSV.
-->

### laser_to_cartesian(msg, current_odom)
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

### record_performance(stamp, status, start, timing)

### publish_visualization()