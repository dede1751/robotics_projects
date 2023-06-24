define kinect ranger
(
  sensor
  (
    pose [ 0.0 0.0 0.0 0.0 ]
    size [0.0 0.0 0.0 ]
    range  [0 30]
    fov 360.0
    samples 4500
  )
  # generic model properties
  color "black"
  size [ 0.0 0.0 0.0 ]
)




define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  localization "odom"

  odom_error [0.0 0.0 0.0 0.0 ]

  size [ 0.4 0.6 0.0 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "grey"

  kinect()
)