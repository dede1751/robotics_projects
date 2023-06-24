Second Robotics Project -- Andrea Sgobbi, 

    Mapping:
 - It seemed the lidar in use was a Velodyne HDL 32E, so we converted to a laserscan with similar
   resolution according to its spec.
 - We opted to use Slam Toolbox.
 - "Visual" parameter tuning was performed. It seemed best performance was achieved with a slightly
   higher resolution and by performing more frequent updates. This is quite clearly due to the very
   complex school/office environment explored.
 - The lidar, even when converted to 2d, provided far greater accuracy in mapping.

    Navigation:
  