# This is a helper script that is needed because calling os.system(cmd) in pytho
n with multi-line commands does not really work nicely

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose: 
  position: 
    x: 66.0144729614
    y: -76.4052886963
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: -0.507478034911
    w: 0.861664693534"
