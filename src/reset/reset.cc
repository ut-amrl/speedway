#include "reset.h"
#include "visualization/visualization.h"
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>

using Eigen::Vector2f;

namespace reset{
    ResetPolicy::ResetPolicy() {}

    ResetPolicy::~ResetPolicy() {
        delete goal_pos;
        goal_pos = nullptr;
        delete path;
        path = nullptr;
    }

    void ResetPolicy::NavTargetCallback(const amrl_msgs::Localization2DMsg &msg){
        double x = msg.pose.x;
        double y = msg.pose.y;
        double theta = msg.pose.theta;
        delete goal_pos;
        goal_pos = new Position(x, y, theta);
        delete path;
        path = nullptr;
    }
        
    void ResetPolicy::LocalizationCallback(const amrl_msgs::Localization2DMsg &msg, amrl_msgs::VisualizationMsg &global_viz_msg_){
        if(goal_pos == nullptr) return;
        if(path == nullptr){
            double q0[] = {msg.pose.x, msg.pose.y, msg.pose.theta};
            double q1[] = {goal_pos->x, goal_pos->y, goal_pos->theta};
            path = new DubinsPath();
            dubins_shortest_path(path, q0, q1, 1.0); // 1.0 is radius of curvature
        }

        auto dubins_callback = [](double q[3], double x, void *user_data){
            std::vector<Vector2f> *path_points = (std::vector<Vector2f>*)user_data;
            path_points->push_back(Vector2f{q[0], q[1]});
            return 0;
        };

        visualization::ClearVisualizationMsg(global_viz_msg_);

        std::vector<Vector2f> path_points;
        dubins_path_sample_many(path, 0.0025, dubins_callback, &path_points); // step size of 0.0025 m
        unsigned closest_pt_idx = 0;
        double closest_pt_dist = 1000;
        for(unsigned i = 1; i<path_points.size(); i++){
            visualization::DrawLine(path_points[i-1], path_points[i], 0x0000ff, global_viz_msg_);
            double dist = sqrt((msg.pose.x-path_points[i][0])*(msg.pose.x-path_points[i][0]) + (msg.pose.y-path_points[i][1])*(msg.pose.y-path_points[i][1]));
            if(dist < closest_pt_dist) {
                closest_pt_dist = dist;
                closest_pt_idx = i;
            }
        }
        // Target is point 5 indices away ...
        double dist_covered = closest_pt_idx * 0.0025;
        double num1 = path->param[0]-dist_covered;
        double num2 = num1+path->param[1];
        double num3 = num2+path->param[2];

        if(num1 > 0.0025) {
            if(path->type == DubinsPathType::LSL || path->type == DubinsPathType::LSR || path->type == DubinsPathType::LRL){
                vel_cmd = 0.5;
                curv_cmd = 1.0;
            }
            else{
                vel_cmd = 0.5;
                curv_cmd = -1.0;
            }
        }else if(num2 > 0.0025){
            if(path->type == DubinsPathType::RLR){
                vel_cmd = 0.5;
                curv_cmd = 1.0;
            }
            else if(path->type == DubinsPathType::LRL){ 
                vel_cmd = 0.5;
                curv_cmd = -1.0;
            }
            else{
                vel_cmd = 0.5;
                curv_cmd = 0.0;
            }
        }else if(num3 > 0.0025){
            if(path->type == DubinsPathType::LSL || path->type == DubinsPathType::RSL || path->type == DubinsPathType::LRL){
                vel_cmd = 0.5;
                curv_cmd = 1.0;
            }
            else{
                vel_cmd = 0.5;
                curv_cmd = -1.0;
            }
        }else{
            vel_cmd = 0.0;
            curv_cmd = 0.0;
        }

        // If dist to goal < threshold, stop
        if(goal_pos != nullptr){
            // Draw the goal state
            Vector2f p{goal_pos->x + 0.5*cos(goal_pos->theta), goal_pos->y + 0.5*sin(goal_pos->theta)};
            visualization::DrawLine(Vector2f{goal_pos->x, goal_pos->y}, p, 0xff00ff, global_viz_msg_);
            Vector2f a{goal_pos->x + 0.42*cos(goal_pos->theta+0.22), goal_pos->y + 0.42*sin(goal_pos->theta+0.22)};
            Vector2f b{goal_pos->x + 0.42*cos(goal_pos->theta-0.22), goal_pos->y + 0.42*sin(goal_pos->theta-0.22)};
            visualization::DrawLine(a, p, 0xff00ff, global_viz_msg_);
            visualization::DrawLine(b, p, 0xff00ff, global_viz_msg_);

            double dist = sqrt((msg.pose.x-goal_pos->x)*(msg.pose.x-goal_pos->x) + (msg.pose.y-goal_pos->y)*(msg.pose.y-goal_pos->y) + (msg.pose.theta-goal_pos->theta)*(msg.pose.theta-goal_pos->theta));
            if(dist < 0.05){
                vel_cmd = 0.0;
                curv_cmd = 0.0;
                delete goal_pos;
                goal_pos = nullptr;
            }
        }
    }

    Command ResetPolicy::ComputeCommand(){
        return Command(vel_cmd, curv_cmd);
    }
}