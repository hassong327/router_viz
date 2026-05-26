#include "qcar2_control_2/ph_runtime.hpp"

#include <algorithm>

namespace qcar2_control_2::ph
{

namespace
{
static constexpr double kEpsilon = 1e-9;
static constexpr double kEpsilonSq = 1e-12;
}  // namespace

std::array<double, 6> PhPathRuntime::quinticCoeffs(
  double p0,
  double v0,
  double a0,
  double p1,
  double v1,
  double a1)
{
  const double c0 = p0;
  const double c1 = v0;
  const double c2 = 0.5 * a0;

  const double a_term = p1 - (c0 + c1 + c2);
  const double b_term = v1 - (c1 + 2.0 * c2);
  const double c_term = a1 - (2.0 * c2);

  const double c3 = 10.0 * a_term - 4.0 * b_term + 0.5 * c_term;
  const double c4 = -15.0 * a_term + 7.0 * b_term - c_term;
  const double c5 = 6.0 * a_term - 3.0 * b_term + 0.5 * c_term;
  return {c0, c1, c2, c3, c4, c5};
}

std::array<double, 6> PhPathRuntime::bezierQuinticCoeffs(
  double p0, double p1, double p2, double p3, double p4, double p5)
{
  // Convert a quintic Bezier control polygon to monomial coefficients:
  // B(u) = sum_i c_i * u^i, u in [0, 1].
  return {
    p0,
    5.0 * (p1 - p0),
    10.0 * (p2 - 2.0 * p1 + p0),
    10.0 * (p3 - 3.0 * p2 + 3.0 * p1 - p0),
    5.0 * (p4 - 4.0 * p3 + 6.0 * p2 - 4.0 * p1 + p0),
    (p5 - 5.0 * p4 + 10.0 * p3 - 10.0 * p2 + 5.0 * p1 - p0),
  };
}

Vec2 PhPathRuntime::normalizeOrFallback(const Vec2 & v, const Vec2 & fallback)
{
  const double n = norm(v);
  if (n > kEpsilon) {
    return {v.x / n, v.y / n};
  }
  const double fb = norm(fallback);
  if (fb > kEpsilon) {
    return {fallback.x / fb, fallback.y / fb};
  }
  return {1.0, 0.0};
}

double PhPathRuntime::evalPoly(const std::array<double, 6> & a, double u)
{
  double out = a[5];
  for (int i = 4; i >= 0; --i) {
    out = out * u + a[static_cast<size_t>(i)];
  }
  return out;
}

double PhPathRuntime::evalPolyD1(const std::array<double, 6> & a, double u)
{
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  return
    a[1] +
    2.0 * a[2] * u +
    3.0 * a[3] * u2 +
    4.0 * a[4] * u3 +
    5.0 * a[5] * u4;
}

double PhPathRuntime::evalPolyD2(const std::array<double, 6> & a, double u)
{
  const double u2 = u * u;
  const double u3 = u2 * u;
  return
    2.0 * a[2] +
    6.0 * a[3] * u +
    12.0 * a[4] * u2 +
    20.0 * a[5] * u3;
}

double PhPathRuntime::approxSegmentLength(
  const std::array<double, 6> & ax,
  const std::array<double, 6> & ay)
{
  const int samples = 24;
  Vec2 prev{evalPoly(ax, 0.0), evalPoly(ay, 0.0)};
  double length = 0.0;
  for (int i = 1; i <= samples; ++i) {
    const double u = static_cast<double>(i) / static_cast<double>(samples);
    const Vec2 p{evalPoly(ax, u), evalPoly(ay, u)};
    length += norm(p - prev);
    prev = p;
  }
  return std::max(kEpsilon, length);
}

bool PhPathRuntime::updateFromMsg(
  const qcar2_msgs_2::msg::PhQuinticPath & msg,
  std::string & invalid_reason)
{
  has_path_ = true;
  valid_path_ = false;
  invalid_reason.clear();

  frame_id_ = msg.header.frame_id;
  traj_id_ = static_cast<uint32_t>(msg.traj_id);

  if (frame_id_.empty()) {
    invalid_reason = "Empty frame_id";
    return false;
  }

  if (msg.segments.empty()) {
    invalid_reason = "No segments";
    return false;
  }

  std::vector<Segment> next_segments;
  next_segments.reserve(msg.segments.size());

  double accum_len = 0.0;
  for (const auto & s : msg.segments) {
    Segment seg;

    if (s.control_points.size() == 6) {
      const auto & cp = s.control_points;
      seg.ax = bezierQuinticCoeffs(cp[0].x, cp[1].x, cp[2].x, cp[3].x, cp[4].x, cp[5].x);
      seg.ay = bezierQuinticCoeffs(cp[0].y, cp[1].y, cp[2].y, cp[3].y, cp[4].y, cp[5].y);
      seg.length_m = approxSegmentLength(seg.ax, seg.ay);
    } else {
      const Vec2 p0{s.start_point.x, s.start_point.y};
      const Vec2 p1{s.end_point.x, s.end_point.y};
      const Vec2 chord = p1 - p0;

      const Vec2 t0 = normalizeOrFallback({s.start_tangent.x, s.start_tangent.y}, chord);
      const Vec2 t1 = normalizeOrFallback({s.end_tangent.x, s.end_tangent.y}, chord);

      double scale = norm(chord);
      if (std::isfinite(s.arc_length) && s.arc_length > kEpsilon) {
        scale = s.arc_length;
      }
      scale = std::max(kEpsilon, scale);

      seg.ax = quinticCoeffs(p0.x, t0.x * scale, 0.0, p1.x, t1.x * scale, 0.0);
      seg.ay = quinticCoeffs(p0.y, t0.y * scale, 0.0, p1.y, t1.y * scale, 0.0);

      if (std::isfinite(s.arc_length) && s.arc_length > kEpsilon) {
        seg.length_m = s.arc_length;
      } else {
        seg.length_m = approxSegmentLength(seg.ax, seg.ay);
      }
    }

    accum_len += seg.length_m;
    next_segments.push_back(seg);
  }

  if (accum_len < kEpsilon) {
    invalid_reason = "Invalid total_length_m";
    return false;
  }

  segments_ = std::move(next_segments);
  total_length_m_ = accum_len;

  cumulative_lengths_.clear();
  cumulative_lengths_.reserve(segments_.size());
  double running = 0.0;
  for (const auto & s : segments_) {
    cumulative_lengths_.push_back(running);
    running += s.length_m;
  }

  have_last_closest_s_ = false;
  last_closest_s_ = 0.0;

  valid_path_ = true;
  return true;
}

bool PhPathRuntime::mapS2SegmentU(double s, size_t & seg_idx, double & u) const
{
  if (segments_.empty() || cumulative_lengths_.empty() || total_length_m_ < kEpsilon) {
    return false;
  }

  s = clamp(s, 0.0, total_length_m_);

  seg_idx = 0;
  while (seg_idx + 1 < cumulative_lengths_.size() && cumulative_lengths_[seg_idx + 1] < s) {
    ++seg_idx;
  }

  const double seg_s0 = cumulative_lengths_[seg_idx];
  const double seg_len = std::max(kEpsilon, segments_[seg_idx].length_m);
  u = clamp((s - seg_s0) / seg_len, 0.0, 1.0);
  return true;
}

EvalResult PhPathRuntime::evalSegment(size_t idx, double u) const
{
  EvalResult e;
  if (idx >= segments_.size()) {
    return e;
  }

  u = clamp(u, 0.0, 1.0);
  const auto & seg = segments_[idx];

  e.p.x = evalPoly(seg.ax, u);
  e.p.y = evalPoly(seg.ay, u);
  e.d1.x = evalPolyD1(seg.ax, u);
  e.d1.y = evalPolyD1(seg.ay, u);
  e.d2.x = evalPolyD2(seg.ax, u);
  e.d2.y = evalPolyD2(seg.ay, u);
  return e;
}

EvalResult PhPathRuntime::evalAtS(double s) const
{
  EvalResult e;
  size_t idx = 0;
  double u = 0.0;
  if (!mapS2SegmentU(s, idx, u)) {
    return e;
  }
  return evalSegment(idx, u);
}

Vec2 PhPathRuntime::pointAtRatio(double ratio) const
{
  if (!valid_path_) {
    return {0.0, 0.0};
  }
  const double s = clamp(ratio, 0.0, 1.0) * total_length_m_;
  return evalAtS(s).p;
}

double PhPathRuntime::ratioFromS(double s) const
{
  if (!valid_path_ || total_length_m_ < kEpsilon) {
    return 0.0;
  }
  return clamp(s / total_length_m_, 0.0, 1.0);
}

double PhPathRuntime::curvatureFrom(const Vec2 & d1, const Vec2 & d2) const
{
  const double denom = std::pow(d1.x * d1.x + d1.y * d1.y, 1.5);
  if (denom < kEpsilonSq) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double numer = d1.x * d2.y - d1.y * d2.x;
  return numer / denom;
}

double PhPathRuntime::curvatureAtS(double s) const
{
  const auto e = evalAtS(s);
  return curvatureFrom(e.d1, e.d2);
}

double PhPathRuntime::distanceSq(const Vec2 & a, const Vec2 & b) const
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

double PhPathRuntime::refineUOnSegment(
  const Vec2 & query,
  size_t seg_idx,
  double u0,
  int newton_max_iterations,
  double newton_tolerance) const
{
  const int max_iter = std::max(1, newton_max_iterations);
  double u = clamp(u0, 0.0, 1.0);

  for (int it = 0; it < max_iter; ++it) {
    const auto e = evalSegment(seg_idx, u);
    const Vec2 r = e.p - query;

    const double g = 2.0 * dot(r, e.d1);
    const double h = 2.0 * (dot(e.d1, e.d1) + dot(r, e.d2));

    if (!std::isfinite(g) || std::fabs(g) < newton_tolerance) {
      break;
    }

    double u_next = u;
    if (std::isfinite(h) && std::fabs(h) > kEpsilon) {
      u_next = u - g / h;
    } else {
      u_next = u - 0.05 * std::copysign(1.0, g);
    }

    u_next = clamp(u_next, 0.0, 1.0);
    if (std::fabs(u_next - u) < newton_tolerance) {
      u = u_next;
      break;
    }
    u = u_next;
  }

  return u;
}

void PhPathRuntime::evaluateCandidate(
  const Vec2 & query,
  size_t seg_idx,
  double u,
  ClosestResult & best) const
{
  const auto e = evalSegment(seg_idx, u);
  const double d2 = distanceSq(e.p, query);
  if (d2 < best.dist) {
    best.dist = d2;
    best.p = e.p;
    const double s0 = cumulative_lengths_[seg_idx];
    best.s = s0 + u * std::max(kEpsilon, segments_[seg_idx].length_m);
  }
}

ClosestResult PhPathRuntime::findClosestGlobal(
  const Vec2 & query,
  int global_seeds_per_segment,
  int newton_max_iterations,
  double newton_tolerance) const
{
  ClosestResult best;
  if (!valid_path_) {
    return best;
  }

  const int seeds = std::max(2, global_seeds_per_segment);

  for (size_t seg_idx = 0; seg_idx < segments_.size(); ++seg_idx) {
    for (int i = 0; i < seeds; ++i) {
      double u = static_cast<double>(i) / static_cast<double>(seeds - 1);
      u = refineUOnSegment(query, seg_idx, u, newton_max_iterations, newton_tolerance);
      evaluateCandidate(query, seg_idx, u, best);
    }
  }

  if (std::isfinite(best.dist)) {
    best.dist = std::sqrt(best.dist);
    best.s = clamp(best.s, 0.0, total_length_m_);
  }
  return best;
}

ClosestResult PhPathRuntime::findClosestLocal(
  const Vec2 & query,
  double s_center,
  double half_window,
  int local_seed_count,
  int newton_max_iterations,
  double newton_tolerance) const
{
  ClosestResult best;
  if (!valid_path_ || half_window <= kEpsilon) {
    return best;
  }

  const double s_lo = clamp(s_center - half_window, 0.0, total_length_m_);
  const double s_hi = clamp(s_center + half_window, 0.0, total_length_m_);
  if (s_hi <= s_lo + kEpsilon) {
    return best;
  }

  const int seeds = std::max(3, local_seed_count);
  for (int i = 0; i < seeds; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(seeds - 1);
    const double s_seed = s_lo + t * (s_hi - s_lo);

    size_t seg_idx = 0;
    double u0 = 0.0;
    if (!mapS2SegmentU(s_seed, seg_idx, u0)) {
      continue;
    }

    const double u = refineUOnSegment(query, seg_idx, u0, newton_max_iterations, newton_tolerance);
    evaluateCandidate(query, seg_idx, u, best);
  }

  if (std::isfinite(best.dist)) {
    best.dist = std::sqrt(best.dist);
    best.s = clamp(best.s, 0.0, total_length_m_);
  }
  return best;
}

bool PhPathRuntime::findClosest(
  const Vec2 & query,
  double local_half_window_m,
  int local_seed_count,
  int global_seeds_per_segment,
  int newton_max_iterations,
  double newton_tolerance,
  double global_fallback_dist_m,
  ClosestResult & out)
{
  if (!valid_path_) {
    return false;
  }

  ClosestResult best;
  if (have_last_closest_s_) {
    best = findClosestLocal(
      query,
      last_closest_s_,
      local_half_window_m,
      local_seed_count,
      newton_max_iterations,
      newton_tolerance);

    const bool local_ok = std::isfinite(best.dist);
    const bool fallback_needed =
      (global_fallback_dist_m > kEpsilon) && (!local_ok || best.dist > global_fallback_dist_m);

    if (!fallback_needed && local_ok) {
      out = best;
      last_closest_s_ = best.s;
      have_last_closest_s_ = true;
      return true;
    }
  }

  best = findClosestGlobal(query, global_seeds_per_segment, newton_max_iterations, newton_tolerance);
  if (!std::isfinite(best.dist)) {
    return false;
  }

  out = best;
  last_closest_s_ = best.s;
  have_last_closest_s_ = true;
  return true;
}

void PhPathRuntime::computeLookaheadMaxCurvature(
  double s_start,
  double lookahead_dist,
  int sample_count,
  double & out_actual_lookahead,
  double & out_max_curvature) const
{
  if (!valid_path_ || lookahead_dist < kEpsilon) {
    out_actual_lookahead = 0.0;
    out_max_curvature = std::numeric_limits<double>::quiet_NaN();
    return;
  }

  const double s_end = std::min(total_length_m_, s_start + lookahead_dist);
  if (s_end <= s_start + kEpsilon) {
    out_actual_lookahead = 0.0;
    out_max_curvature = std::numeric_limits<double>::quiet_NaN();
    return;
  }

  const int n = std::max(2, sample_count);
  double max_abs_curvature = 0.0;
  double max_curvature = 0.0;
  double actual_lookahead = 0.0;

  Vec2 prev = evalAtS(s_start).p;
  for (int i = 1; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    const double s = s_start + t * (s_end - s_start);
    const auto e = evalAtS(s);

    actual_lookahead += norm(e.p - prev);
    prev = e.p;

    const double kappa = curvatureFrom(e.d1, e.d2);
    if (std::isfinite(kappa) && std::fabs(kappa) > max_abs_curvature) {
      max_abs_curvature = std::fabs(kappa);
      max_curvature = kappa;
    }
  }

  out_actual_lookahead = actual_lookahead;
  out_max_curvature = max_curvature;
}

}  // namespace qcar2_control_2::ph
