# To run the planner

1. launch a world through nav_config (branch potential_gap) 
2. launch egocircle
3. launch potential gap planner through nav_scripts (branch potential_gap) `turtlebot_potentialgap_controller.launch`

Config file in `$(find nav_configs)/config/potentialgap_local_planner_params.yaml`

# Todo: 
1. (Impede immediate performance) Gap parsing seemed to be bugged, need to relook. But is able to run 1.0 regardless, need to fix. 
2. (Impede future performance) Can't handle limited FoV scenarios, mostly due to figuring out where are the goal and where they lies, need to fix this. 
3. Visualization of gaps that are wrapped around need to be fixed.

Runing STDR experiments:
```
rosrun nav_scripts stdr_task_driver.py
```

Go to the `if __main__` part of the `stdr_task_driver.py` to change world configurations.
navigation\_test branch `stdr`
