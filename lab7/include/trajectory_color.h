#pragma once

struct PointColor {
  PointColor(float r = 0.0, float g = 1.0, float b = 0.0, float a = 1.0)
      : r_(r), g_(g), b_(b), a_(a) {}
  float r_;
  float g_;
  float b_;
  float a_;
};

// Color for a line.
struct LineColor {
  LineColor(float r = 0.0, float g = 1.0, float b = 0.0, float a = 1.0)
      : r_(r), g_(g), b_(b), a_(a) {}
  float r_;
  float g_;
  float b_;
  float a_;
};

// Color for a trajectory.
struct TrajectoryColor {
  TrajectoryColor(const PointColor& point_color = PointColor(),
                  const LineColor& line_color = LineColor())
      : point_color_(point_color), line_color_(line_color) {}
  PointColor point_color_;
  LineColor line_color_;
};
