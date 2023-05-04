First Robotics Project -- Andrea Sgobbi, Diego Viganò

The odometry model we used is that of a 2.8m wheelbase. We used the following formulae:

    w = velocity * tan(steering_angle) / WHEEL_BASE
    r = WHEEL_BASE / tan(steering_angle)

Assuming constant linear/angular velocity across a single sensor publishing interval, we can
use exact integration to compute odometry from x(k), y(k), th(k), v(k), r(k) and dt.

    x (k+1) =  x(k) + r(k) * (sin(th(k+1)) - sin(th(k))) * dt
    y (k+1) =  y(k) - r(k) * (cos(th(k+1)) - cos(th(k))) * dt
    th(k+1) = th(k) + v(k) * dt / r(k)