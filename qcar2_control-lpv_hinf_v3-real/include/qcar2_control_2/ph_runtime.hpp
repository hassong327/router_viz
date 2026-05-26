#pragma once

#include <array>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "qcar2_msgs_2/msg/ph_quintic_path.hpp"

namespace qcar2_control_2::ph
{

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

inline Vec2 operator+(const Vec2 & a, const Vec2 & b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(const Vec2 & a, const Vec2 & b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(double s, const Vec2 & v) { return {s * v.x, s * v.y}; }

inline double dot(const Vec2 & a, const Vec2 & b) { return a.x * b.x + a.y * b.y; }
inline double norm(const Vec2 & v) { return std::sqrt(v.x * v.x + v.y * v.y); }

inline Vec2 normalize(const Vec2 & v)
{
  const double n = norm(v);
  if (n < 1e-9) {
    return {0.0, 0.0};
  }
  return {v.x / n, v.y / n};
}

inline double clamp(double v, double lo, double hi)
{
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

inline double wrapToPi(double a)
{
  static constexpr double two_pi = 2.0 * M_PI;
  a = std::fmod(a + M_PI, two_pi);
  if (a < 0.0) {
    a += two_pi;
  }
  return a - M_PI;
}

struct EvalResult
{
  Vec2 p{0.0, 0.0};
  Vec2 d1{0.0, 0.0};
  Vec2 d2{0.0, 0.0};
};

struct ClosestResult
{
  double s{0.0};
  Vec2 p{0.0, 0.0};
  double dist{std::numeric_limits<double>::infinity()};
};

class PhPathRuntime
{
public:
  bool updateFromMsg(const qcar2_msgs_2::msg::PhQuinticPath & msg, std::string & invalid_reason);

  bool hasPath() const { return has_path_; }
  bool valid() const { return valid_path_; }
  const std::string & frameId() const { return frame_id_; }
  uint32_t trajId() const { return traj_id_; }
  double totalLength() const { return total_length_m_; }

  EvalResult evalAtS(double s) const;
  Vec2 pointAtRatio(double ratio) const;
  double ratioFromS(double s) const;
  double curvatureFrom(const Vec2 & d1, const Vec2 & d2) const;
  double curvatureAtS(double s) const;

  bool findClosest(
    const Vec2 & query,
    double local_half_window_m,
    int local_seed_count,
    int global_seeds_per_segment,
    int newton_max_iterations,
    double newton_tolerance,
    double global_fallback_dist_m,
    ClosestResult & out);

  void computeLookaheadMaxCurvature(
    double s_start,
    double lookahead_dist,
    int sample_count,
    double & out_actual_lookahead,
    double & out_max_curvature) const;

private:
  struct Segment
  {
    std::array<double, 6> ax{};
    std::array<double, 6> ay{};
    double length_m{0.0};
  };

  static std::array<double, 6> quinticCoeffs(
    double p0,
    double v0,
    double a0,
    double p1,
    double v1,
    double a1);

  static std::array<double, 6> bezierQuinticCoeffs(
    double p0, double p1, double p2, double p3, double p4, double p5);

  static Vec2 normalizeOrFallback(const Vec2 & v, const Vec2 & fallback);
  static double evalPoly(const std::array<double, 6> & a, double u);
  static double evalPolyD1(const std::array<double, 6> & a, double u);
  static double evalPolyD2(const std::array<double, 6> & a, double u);
  static double approxSegmentLength(const std::array<double, 6> & ax, const std::array<double, 6> & ay);

  bool mapS2SegmentU(double s, size_t & seg_idx, double & u) const;
  EvalResult evalSegment(size_t idx, double u) const;

  double distanceSq(const Vec2 & a, const Vec2 & b) const;
  double refineUOnSegment(
    const Vec2 & query,
    size_t seg_idx,
    double u0,
    int newton_max_iterations,
    double newton_tolerance) const;
  void evaluateCandidate(const Vec2 & query, size_t seg_idx, double u, ClosestResult & best) const;
  ClosestResult findClosestGlobal(const Vec2 & query, int global_seeds_per_segment, int newton_max_iterations,
      double newton_tolerance) const;
  ClosestResult findClosestLocal(const Vec2 & query, double s_center, double half_window, int local_seed_count,
      int newton_max_iterations, double newton_tolerance) const;

  bool has_path_{false};
  bool valid_path_{false};
  std::string frame_id_;
  uint32_t traj_id_{0};

  std::vector<Segment> segments_;
  std::vector<double> cumulative_lengths_;
  double total_length_m_{0.0};

  bool have_last_closest_s_{false};
  double last_closest_s_{0.0};
};

}  // namespace qcar2_control_2::ph
