Second Robotics Project -- Andrea Sgobbi, Diego Viganò

    Mapping:
  - It seemed the lidar in use was a Velodyne HDL 32E, so we converted to a laserscan with similar
   resolution according to its spec.
  - We opted to use Slam Toolbox.
  - "Visual" parameter tuning was performed. It seemed best performance was achieved with a slightly
   higher resolution and by performing more frequent updates. This is quite clearly due to the very
   complex school/office environment explored.
  - The lidar, even when converted to 2d, provided far greater accuracy in mapping.

    Navigation:
  - The simulated robot matches the 0.6x0.4 footprint of the original
  - Post processing was used mostly to remove noisy points in the more cramped areas, between the
    desks
  - The NavStack made use of GlobalPlanner and TEBLocalPlanner. We also experimented with NavFn and
    DWALocalPlanner but with worse results.
  - Overall, the robot is able to trace back the trajectory recorded in the bags. It does however
    often struggle with the tight spaces, and takes some time to traverse them. We attributed a lot
    of this to the relatively low quality of the map due to running a single pass over the environment.
    The other main cause is a more conservative approach with robot dimensions and clearances.
  - The waypoints.csv is a roughly accurate simulation of the path within the bags
  - The local planner seems to throw errors when reaching the final waypoint. This does not happen
    in the rest of the path and it still reports reaching the goal.
