#pragma once

#include <amrl_msgs/Localization2DMsg.h>
#include <amrl_msgs/VisualizationMsg.h>
#include "dubins.h"


namespace reset{
    struct Position{
        double x, y, theta;
        Position(double x = 0, double y = 0, double theta = 0) : x(x), y(y), theta(theta) {}
    };

    struct Command{
        double v, k;
        Command(double v = 0, double k = 0) : v(v), k(k) {}
    };

    class ResetPolicy{
    private:
        Position *goal_pos = nullptr;
        double vel_cmd = 0, curv_cmd = 0;
        DubinsPath *path = nullptr;
        int state = 0, state_max = 300;
        double rand_v, rand_k;

    public:
        ResetPolicy();
        ~ResetPolicy();
        void NavTargetCallback(const amrl_msgs::Localization2DMsg &msg);
        void LocalizationCallback(const amrl_msgs::Localization2DMsg &msg, amrl_msgs::VisualizationMsg &global_viz_msg_);
        Command ComputeCommand();
    };
}