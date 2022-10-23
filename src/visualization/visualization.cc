#include "visualization.h"

#include <amrl_msgs/ColoredArc2D.h>
#include <amrl_msgs/ColoredLine2D.h>
#include <amrl_msgs/ColoredPoint2D.h>
#include <amrl_msgs/Pose2Df.h>
#include <amrl_msgs/VisualizationMsg.h>
#include <ros/ros.h>

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <string>

using amrl_msgs::ColoredArc2D;
using amrl_msgs::ColoredLine2D;
using amrl_msgs::ColoredPoint2D;
using amrl_msgs::Pose2Df;
using amrl_msgs::VisualizationMsg;
using Eigen::Rotation2Df;
using Eigen::Vector2f;
using std::max;
using std::string;

namespace {
template <class T1, class T2>
void SetPoint(const T1& p1, T2* p2) {
  p2->x = p1.x();
  p2->y = p1.y();
}

}  // namespace

namespace visualization {

// Clear all elements in the message.
void ClearVisualizationMsg(VisualizationMsg& msg) {
  msg.points.clear();
  msg.lines.clear();
  msg.arcs.clear();
}

// Return new visualization message, with initialized headers and namespace.
amrl_msgs::VisualizationMsg NewVisualizationMessage(const string& frame,
                                                    const string& ns) {
  VisualizationMsg msg;
  msg.header.frame_id = frame;
  msg.header.seq = 0;
  msg.ns = ns;
  return msg;
}

void DrawPoint(const Vector2f& p, uint32_t color, VisualizationMsg& msg) {
  ColoredPoint2D point;
  SetPoint(p, &point.point);
  point.color = color;
  msg.points.push_back(point);
}

void DrawLine(const Vector2f& p0, const Vector2f& p1, uint32_t color,
              VisualizationMsg& msg) {
  ColoredLine2D line;
  SetPoint(p0, &line.p0);
  SetPoint(p1, &line.p1);
  line.color = color;
  msg.lines.push_back(line);
}

void DrawCross(const Eigen::Vector2f& location, float size, uint32_t color,
               VisualizationMsg& msg) {
  DrawLine(location + Vector2f(size, size), location - Vector2f(size, size),
           color, msg);
  DrawLine(location + Vector2f(size, -size), location - Vector2f(size, -size),
           color, msg);
}

void DrawArc(const Vector2f& center, float radius, float start_angle,
             float end_angle, uint32_t color, VisualizationMsg& msg) {
  ColoredArc2D arc;
  SetPoint(center, &arc.center);
  arc.radius = radius;
  arc.start_angle = start_angle;
  arc.end_angle = end_angle;
  arc.color = color;
  msg.arcs.push_back(arc);
}

void DrawParticle(const Vector2f& loc, float angle, VisualizationMsg& msg) {
  DrawArc(loc, 0.1, -M_PI, M_PI, 0x40FF0000, msg);
  DrawLine(loc, loc + Rotation2Df(angle) * Vector2f(0.2, 0), 0x40FF0000, msg);
}

void DrawPathOption(const float curvature, const float distance,
                    const float clearance, const uint32_t color,
                    bool show_clearance, VisualizationMsg& msg) {
  // TODO: color by clearance.
  // static const uint32_t kPathColor = 0xC0C0C0;
  if (fabs(curvature) < 0.001) {
    DrawLine(Vector2f(0, 0), Vector2f(distance, 0), color, msg);
    if (show_clearance) {
      DrawLine(Vector2f(0, clearance), Vector2f(distance, clearance), color,
               msg);
      DrawLine(Vector2f(0, -clearance), Vector2f(distance, -clearance), color,
               msg);
    }
  } else {
    const float r = 1.0f / curvature;
    const Vector2f center(0, r);
    const float a = fabs(distance * curvature);
    const float a0 = ((curvature > 0.0f) ? -M_PI_2 : (M_PI_2 - a));
    const float a1 = ((curvature > 0.0f) ? (-M_PI_2 + a) : M_PI_2);
    DrawArc(center, fabs(r), a0, a1, color, msg);
    if (show_clearance) {
      DrawArc(center, max<float>(0, fabs(r) - clearance), a0, a1, color, msg);
      DrawArc(center, max<float>(0, fabs(r) + clearance), a0, a1, color, msg);
    }
  }
}
}  // namespace visualization
