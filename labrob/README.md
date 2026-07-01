# Using RVIZ

Please use the following command the run RVIZ and change the urdf file's name and  directory path accordingly. You may also need to update the find path for meshes.
``` 
ros2 launch urdf_tutorial display.launch.py model:=/home/kaansnowman/ros2_ws/src/labrob/labrob_desc/urdf/enson.urdf
```

# Error Handling in WSL 

If you get the WARN: COPY MODE warning while opening RVIZ, shutdown or restart the WSL. You may use the following command on powershell 

```
wsl --shutdown
```

