RaceParameters = {
    odom_topic = "/odom";
    laser_topic = "/scan";

    laser_location = { 0.2, 0 };

    wall_color = 0x00ff00;
    wall_polynomial_order = 5;
    wall_tolerance = 0.1; -- number of meters within which to consider lidar points part of the same wall

    profiler_config = "config/profiler.lua";
}
