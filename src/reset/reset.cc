#include "reset.h"
#include "visualization/visualization.h"
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>

using Eigen::Vector2f;
using Eigen::Vector3f;

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
            std::vector<Vector3f> *path_points = (std::vector<Vector3f>*)user_data;
            path_points->push_back(Vector3f{(float)q[0], (float)q[1], (float)q[2]});
            return 0;
        };

        visualization::ClearVisualizationMsg(global_viz_msg_);

        std::vector<Vector3f> path_points;
        dubins_path_sample_many(path, 0.0025, dubins_callback, &path_points); // step size of 0.0025 m
        unsigned closest_pt_idx = 0;
        double closest_pt_dist = 1000;
        for(unsigned i = 1; i<path_points.size(); i++){
            visualization::DrawLine(Vector2f{path_points[i-1][0], path_points[i-1][1]}, Vector2f{path_points[i][0], path_points[i][1]}, 0x0000ff, global_viz_msg_);
            double dist = sqrt((msg.pose.x-path_points[i][0])*(msg.pose.x-path_points[i][0]) + (msg.pose.y-path_points[i][1])*(msg.pose.y-path_points[i][1]));
            if(dist < closest_pt_dist) {
                closest_pt_dist = dist;
                closest_pt_idx = i;
            }
        }

        /* Original open-loop path following code
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
        }*/

        auto draw_state = [&global_viz_msg_](Vector2f pos, double theta, int color = 0xff00ff){
            Vector2f p{pos[0] + 0.5*cos(theta), pos[1] + 0.5*sin(theta)};
            visualization::DrawLine(Vector2f{pos[0], pos[1]}, p, color, global_viz_msg_);
            Vector2f a{pos[0] + 0.42*cos(theta+0.22), pos[1] + 0.42*sin(theta+0.22)};
            Vector2f b{pos[0] + 0.42*cos(theta-0.22), pos[1] + 0.42*sin(theta-0.22)};
            visualization::DrawLine(a, p, color, global_viz_msg_);
            visualization::DrawLine(b, p, color, global_viz_msg_);
        };

        // Select target point 0.5 meter ahead
        unsigned target_idx = closest_pt_idx + (unsigned)(0.5 / 0.0025);
        double projection_time = 1;
        if(target_idx >= path_points.size()){
            target_idx = path_points.size()-1;
            projection_time = (target_idx-closest_pt_idx) * 0.0025 / 0.5;
        }
        double best_curv = 0;
        double min_dist = 10000;
        Vector2f best_end_robot_pos;
        for(double k = -1; k<=1; k+=0.001){
            Vector2f end_robot_pos;
            double end_robot_theta;
            if(k < 0){
                double r = -1/k;
                Vector2f circle_center{msg.pose.x+r*cos(msg.pose.theta-M_PI/2), msg.pose.y+r*sin(msg.pose.theta-M_PI/2)};
                double end_circle_angle = msg.pose.theta + M_PI/2 - (projection_time*0.5 / r);
                end_robot_pos = Vector2f{circle_center[0]+r*cos(end_circle_angle), circle_center[1]+r*sin(end_circle_angle)};
                end_robot_theta = msg.pose.theta - projection_time*0.5 / r;
            }else if(k > 0){
                double r = 1/k;
                Vector2f circle_center{msg.pose.x+r*cos(msg.pose.theta+M_PI/2), msg.pose.y+r*sin(msg.pose.theta+M_PI/2)};
                double end_circle_angle = msg.pose.theta - M_PI/2 + (projection_time*0.5 / r);
                end_robot_pos = Vector2f{circle_center[0]+r*cos(end_circle_angle), circle_center[1]+r*sin(end_circle_angle)};
                end_robot_theta = msg.pose.theta + projection_time*0.5 / r;
            }else{
                end_robot_pos = Vector2f{msg.pose.x+0.5*projection_time*cos(msg.pose.theta), msg.pose.y+0.5*projection_time*sin(msg.pose.theta)};
                end_robot_theta = msg.pose.theta;
            }
            double dist = sqrt((end_robot_pos[0]-path_points[target_idx][0])*(end_robot_pos[0]-path_points[target_idx][0])+(end_robot_pos[1]-path_points[target_idx][1])*(end_robot_pos[1]-path_points[target_idx][1]));
            if(dist < min_dist){
                min_dist = dist;
                best_curv = k;
                best_end_robot_pos = end_robot_pos;
            }
            draw_state(end_robot_pos, end_robot_theta);
        }
        vel_cmd = 0.5;
        curv_cmd = best_curv;
        draw_state(Vector2f{path_points[target_idx][0], path_points[target_idx][1]}, path_points[target_idx][2], 0x000000);

        // If dist to goal < threshold, stop
        if(goal_pos != nullptr){
            // Draw the goal state
            Vector2f p{goal_pos->x + 0.5*cos(goal_pos->theta), goal_pos->y + 0.5*sin(goal_pos->theta)};
            visualization::DrawLine(Vector2f{goal_pos->x, goal_pos->y}, p, 0xff00ff, global_viz_msg_);
            Vector2f a{goal_pos->x + 0.42*cos(goal_pos->theta+0.22), goal_pos->y + 0.42*sin(goal_pos->theta+0.22)};
            Vector2f b{goal_pos->x + 0.42*cos(goal_pos->theta-0.22), goal_pos->y + 0.42*sin(goal_pos->theta-0.22)};
            visualization::DrawLine(a, p, 0xff00ff, global_viz_msg_);
            visualization::DrawLine(b, p, 0xff00ff, global_viz_msg_);

            //double dist = sqrt((msg.pose.x-goal_pos->x)*(msg.pose.x-goal_pos->x) + (msg.pose.y-goal_pos->y)*(msg.pose.y-goal_pos->y) + (sin(msg.pose.theta)-sin(goal_pos->theta))*(sin(msg.pose.theta)-sin(goal_pos->theta)) + (cos(msg.pose.theta)-cos(goal_pos->theta))*(cos(msg.pose.theta)-cos(goal_pos->theta)));
            //double dist = sqrt((msg.pose.x-goal_pos->x)*(msg.pose.x-goal_pos->x) + (msg.pose.y-goal_pos->y)*(msg.pose.y-goal_pos->y));
            if(projection_time < 0.05){
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