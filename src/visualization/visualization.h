#include <amrl_msgs/VisualizationMsg.h>
#include <stdint.h>

#include <eigen3/Eigen/Dense>
#include <string>

namespace visualization {

// Clear all elements in the message.
void ClearVisualizationMsg(amrl_msgs::VisualizationMsg& msg);

// Return new visualization message, with initialized headers and namespace.
amrl_msgs::VisualizationMsg NewVisualizationMessage(const std::string& frame,
                                                    const std::string& ns);

// Add a single point to the visualization message.
void DrawPoint(const Eigen::Vector2f& p, uint32_t color,
               amrl_msgs::VisualizationMsg& msg);

// Add a single line to the visualization message.
void DrawLine(const Eigen::Vector2f& p0, const Eigen::Vector2f& p1,
              uint32_t color, amrl_msgs::VisualizationMsg& msg);

// Add a "X" to the visualization message.
void DrawCross(const Eigen::Vector2f& location, float size, uint32_t color,
               amrl_msgs::VisualizationMsg& msg);

// Add a single line to the visualization message.
void DrawArc(const Eigen::Vector2f& center, float radius, float start_angle,
             float end_angle, uint32_t color, amrl_msgs::VisualizationMsg& msg);

// Add a particle to the visualization message.
void DrawParticle(const Eigen::Vector2f& loc, float angle,
                  amrl_msgs::VisualizationMsg& msg);

void DrawPathOption(const float curvature, const float distance,
                    const float clearance, const uint32_t color,
                    bool show_clearance, amrl_msgs::VisualizationMsg& msg);

}  // namespace visualization
