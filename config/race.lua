RaceParameters = {
    ros_topics = {
        odom = "/odom",
        laser = "/scan",
        visualization = "/visualization",
        ackermann = "/ackermann_curvature_drive"
    },

    laser_config = {
        laser_location = {0.2, 0},
        include_out_of_range = false
    },

    sampler_type = "ackermann",
    evaluator_type = "linear"
}
