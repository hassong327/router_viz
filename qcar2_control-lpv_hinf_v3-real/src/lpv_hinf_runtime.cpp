#include "qcar2_control_2/lpv_hinf_runtime.hpp"
#include "qcar2_control_2/nlohmann/json.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace qcar2_control_2
{
namespace lpv_hinf
{

// ---------------------------------------------------------------------------
// Generic linear algebra (dynamic size)
// ---------------------------------------------------------------------------

Vector LpvHinfRuntime::matVecMul(const Matrix & A, const Vector & x)
{
  const int rows = static_cast<int>(A.size());
  if (rows == 0) {
    return {};
  }
  const int cols = static_cast<int>(A[0].size());
  Vector r(rows, 0.0);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      r[i] += A[i][j] * x[j];
    }
  }
  return r;
}

double LpvHinfRuntime::dotProduct(const Vector & a, const Vector & b)
{
  const int n = static_cast<int>(std::min(a.size(), b.size()));
  double s = 0.0;
  for (int i = 0; i < n; ++i) {
    s += a[i] * b[i];
  }
  return s;
}

Matrix LpvHinfRuntime::matMatMul(const Matrix & A, const Matrix & B)
{
  const int rows = static_cast<int>(A.size());
  if (rows == 0) {
    return {};
  }
  const int inner = static_cast<int>(A[0].size());
  const int cols  = static_cast<int>(B[0].size());
  Matrix R(rows, std::vector<double>(cols, 0.0));
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      for (int k = 0; k < inner; ++k) {
        R[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return R;
}

// ---------------------------------------------------------------------------
// n×n matrix inverse (Gauss-Jordan with partial pivoting)
// ---------------------------------------------------------------------------

bool LpvHinfRuntime::invertNxN(const Matrix & M, Matrix & Minv)
{
  const int n = static_cast<int>(M.size());
  if (n == 0) {
    return false;
  }

  // Augmented matrix [M | I]
  std::vector<std::vector<double>> aug(n, std::vector<double>(2 * n, 0.0));
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      aug[i][j]     = M[i][j];
      aug[i][j + n] = (i == j) ? 1.0 : 0.0;
    }
  }

  for (int col = 0; col < n; ++col) {
    // Partial pivoting
    int max_row = col;
    double max_val = std::fabs(aug[col][col]);
    for (int row = col + 1; row < n; ++row) {
      if (std::fabs(aug[row][col]) > max_val) {
        max_val = std::fabs(aug[row][col]);
        max_row = row;
      }
    }
    if (max_val < 1e-15) {
      return false;  // singular
    }
    if (max_row != col) {
      std::swap(aug[col], aug[max_row]);
    }

    // Scale pivot row
    const double pivot = aug[col][col];
    for (int j = 0; j < 2 * n; ++j) {
      aug[col][j] /= pivot;
    }

    // Eliminate column
    for (int row = 0; row < n; ++row) {
      if (row == col) {
        continue;
      }
      const double factor = aug[row][col];
      for (int j = 0; j < 2 * n; ++j) {
        aug[row][j] -= factor * aug[col][j];
      }
    }
  }

  // Extract inverse
  Minv.assign(n, std::vector<double>(n, 0.0));
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      Minv[i][j] = aug[i][j + n];
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Tustin (bilinear) discretisation, generalised to n states.
//
//   ima = I − (dt/2)·Ac
//   ipa = I + (dt/2)·Ac
//   Ad  = ima⁻¹ · ipa
//   Bd  = ima⁻¹ · (dt · Bc)
//   Cd  = Cc · ima⁻¹
//   Dd  = Dc + (dt/2) · Cd · Bc
// ---------------------------------------------------------------------------

void LpvHinfRuntime::tustinDiscretize(
  const Matrix & Ac, const Matrix & Bc,
  const Vector & Cc, const Vector & Dc,
  double dt,
  Matrix & Ad, Matrix & Bd, Vector & Cd, Vector & Dd)
{
  const int n  = static_cast<int>(Ac.size());
  const int nu = static_cast<int>(Bc[0].size());
  const double half_dt = 0.5 * dt;

  // ima = I − (dt/2)·Ac,   ipa = I + (dt/2)·Ac
  Matrix ima(n, std::vector<double>(n, 0.0));
  Matrix ipa(n, std::vector<double>(n, 0.0));
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      const double a_scaled = half_dt * Ac[i][j];
      ima[i][j] = ((i == j) ? 1.0 : 0.0) - a_scaled;
      ipa[i][j] = ((i == j) ? 1.0 : 0.0) + a_scaled;
    }
  }

  Matrix ima_inv;
  invertNxN(ima, ima_inv);

  // Ad = ima_inv · ipa
  Ad = matMatMul(ima_inv, ipa);

  // Bd = ima_inv · (dt · Bc)
  Matrix dt_Bc(n, std::vector<double>(nu, 0.0));
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < nu; ++j) {
      dt_Bc[i][j] = dt * Bc[i][j];
    }
  }
  Bd = matMatMul(ima_inv, dt_Bc);

  // Cd = Cc · ima_inv  (1×n × n×n → 1×n)
  Cd.assign(n, 0.0);
  for (int j = 0; j < n; ++j) {
    for (int k = 0; k < n; ++k) {
      Cd[j] += Cc[k] * ima_inv[k][j];
    }
  }

  // Dd = Dc + (dt/2) · Cd · Bc  (1×n × n×nu = 1×nu)
  Dd.assign(nu, 0.0);
  for (int j = 0; j < nu; ++j) {
    double s = 0.0;
    for (int k = 0; k < n; ++k) {
      s += Cd[k] * Bc[k][j];
    }
    Dd[j] = Dc[j] + half_dt * s;
  }
}

// ---------------------------------------------------------------------------
// JSON loading
// ---------------------------------------------------------------------------

bool LpvHinfRuntime::loadFromJson(
  const std::string & json_path, double dt, std::string & error_msg)
{
  std::ifstream ifs(json_path);
  if (!ifs.is_open()) {
    error_msg = "Cannot open JSON file: " + json_path;
    return false;
  }

  nlohmann::json data;
  try {
    data = nlohmann::json::parse(ifs);
  } catch (const nlohmann::json::parse_error & e) {
    error_msg = std::string("JSON parse error: ") + e.what();
    return false;
  }

  // ---- vertices (rho values) ----
  if (!data.contains("vertices") || !data["vertices"].is_array()) {
    error_msg = "JSON missing 'vertices' array";
    return false;
  }
  vertices_ = data["vertices"].get<std::vector<double>>();
  n_vertices_ = static_cast<int>(vertices_.size());
  if (n_vertices_ < 2) {
    error_msg = "Need at least 2 vertices";
    return false;
  }

  // ---- controllers ----
  if (!data.contains("controllers") || !data["controllers"].is_array()) {
    error_msg = "JSON missing 'controllers' array";
    return false;
  }
  const auto & ctrls = data["controllers"];
  if (static_cast<int>(ctrls.size()) != n_vertices_) {
    error_msg = "controllers array size does not match vertices";
    return false;
  }

  // Infer state dimension from the first controller's A matrix.
  if (!ctrls[0].contains("A") || !ctrls[0]["A"].is_array()) {
    error_msg = "First controller missing 'A' matrix";
    return false;
  }
  n_states_ = static_cast<int>(ctrls[0]["A"].size());
  if (n_states_ < 1) {
    error_msg = "Invalid state dimension";
    return false;
  }

  // Detect measurement-channel count from the first controller's B/D schema:
  //   - flat   B (length n)   + scalar  D            → n_inputs = 1 (v3 / v3.1)
  //   - nested B (n × n_in)   + length-n_in array D  → n_inputs > 1 (v4)
  if (!ctrls[0].contains("B") || !ctrls[0]["B"].is_array() ||
      ctrls[0]["B"].empty()) {
    error_msg = "First controller missing or empty 'B' array";
    return false;
  }
  const auto & b0_first_row = ctrls[0]["B"][0];
  n_inputs_ = b0_first_row.is_array()
                ? static_cast<int>(b0_first_row.size())
                : 1;
  if (n_inputs_ < 1) {
    error_msg = "Invalid input dimension";
    return false;
  }

  controllers_.resize(n_vertices_);
  for (int v = 0; v < n_vertices_; ++v) {
    const auto & c = ctrls[v];

    // Validate dimensions on every vertex.
    if (static_cast<int>(c["A"].size()) != n_states_) {
      error_msg = "Inconsistent A row count at vertex " + std::to_string(v);
      return false;
    }

    Matrix Ac(n_states_, std::vector<double>(n_states_, 0.0));
    Matrix Bc(n_states_, std::vector<double>(n_inputs_, 0.0));
    Vector Cc(n_states_, 0.0);
    Vector Dc(n_inputs_, 0.0);

    // A: n×n nested array
    const auto & ja = c["A"];
    for (int i = 0; i < n_states_; ++i) {
      if (static_cast<int>(ja[i].size()) != n_states_) {
        error_msg = "Inconsistent A column count at vertex " + std::to_string(v);
        return false;
      }
      for (int j = 0; j < n_states_; ++j) {
        Ac[i][j] = ja[i][j].get<double>();
      }
    }

    // B: flat n-element (1-channel) or nested n × n_inputs (multi-channel).
    // Stored internally as n × n_inputs Matrix so the generalised
    // tustinDiscretize remains usable for either schema.
    const auto & jb = c["B"];
    if (static_cast<int>(jb.size()) != n_states_) {
      error_msg = "Inconsistent B row count at vertex " + std::to_string(v);
      return false;
    }
    if (n_inputs_ == 1 && !jb[0].is_array()) {
      for (int i = 0; i < n_states_; ++i) {
        Bc[i][0] = jb[i].get<double>();
      }
    } else {
      for (int i = 0; i < n_states_; ++i) {
        if (!jb[i].is_array() ||
            static_cast<int>(jb[i].size()) != n_inputs_) {
          error_msg = "Inconsistent B column count at vertex " + std::to_string(v);
          return false;
        }
        for (int j = 0; j < n_inputs_; ++j) {
          Bc[i][j] = jb[i][j].get<double>();
        }
      }
    }

    // C: flat n-element array
    const auto & jc = c["C"];
    for (int i = 0; i < n_states_; ++i) {
      Cc[i] = jc[i].get<double>();
    }

    // D: scalar (1-channel schema) or length-n_inputs array (multi-channel).
    const auto & jd = c["D"];
    if (n_inputs_ == 1 && !jd.is_array()) {
      Dc[0] = jd.get<double>();
    } else {
      if (!jd.is_array() ||
          static_cast<int>(jd.size()) != n_inputs_) {
        error_msg = "Inconsistent D length at vertex " + std::to_string(v);
        return false;
      }
      for (int j = 0; j < n_inputs_; ++j) {
        Dc[j] = jd[j].get<double>();
      }
    }

    // Tustin discretise and store.
    tustinDiscretize(Ac, Bc, Cc, Dc, dt,
                     controllers_[v].Ad, controllers_[v].Bd,
                     controllers_[v].Cd, controllers_[v].Dd);

    // Zero-initialise state.
    controllers_[v].x.assign(n_states_, 0.0);
  }

  // Steering actuator time constant for the delta_est observer used by
  // the 2-channel measurement y_delta. Falls back to the default if the
  // JSON omits it.
  if (data.contains("tau_delta") && data["tau_delta"].is_number()) {
    tau_d_ = data["tau_delta"].get<double>();
  }
  dt_        = dt;
  delta_est_ = 0.0;

  loaded_ = true;
  return true;
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

void LpvHinfRuntime::reset()
{
  for (auto & ctrl : controllers_) {
    std::fill(ctrl.x.begin(), ctrl.x.end(), 0.0);
  }
  delta_est_ = 0.0;
}

// ---------------------------------------------------------------------------
// Compute
// ---------------------------------------------------------------------------

double LpvHinfRuntime::compute(
  double e_psi,
  double kappa_sched, double v, double kappa_ff)
{
  if (!loaded_ || n_vertices_ < 2) {
    return 0.0;
  }

  // 1. Build the measurement vector.
  //   1-channel (v3 / v3.1):  y = [e_psi]
  //   2-channel (v4):         y = [e_psi, delta_est]
  // delta_est is the internal actuator first-order-lag observer below
  // (QCar2 has no steering encoder, so we cannot use a measured delta).
  Vector y(n_inputs_, 0.0);
  y[0] = e_psi;
  if (n_inputs_ >= 2) {
    y[1] = delta_est_;
  }

  // 2. Scheduling parameter.
  const double rho = std::fabs(kappa_sched) * std::fabs(v);
  double rho_eff = rho * rho_scale;
  rho_eff = std::max(vertices_.front(), std::min(vertices_.back(), rho_eff));

  // 3. Find bracketing vertex pair.
  int idx = 0;
  for (int i = 0; i < n_vertices_ - 1; ++i) {
    if (rho_eff >= vertices_[i + 1]) {
      idx = i + 1;
    } else {
      break;
    }
  }
  idx = std::max(0, std::min(n_vertices_ - 2, idx));

  // 4. Interpolation weight.
  const double rho_lo = vertices_[idx];
  const double rho_hi = vertices_[idx + 1];
  double alpha = 0.0;
  if (rho_hi - rho_lo > 1e-10) {
    alpha = (rho_eff - rho_lo) / (rho_hi - rho_lo);
  }

  // 5a. Output from the two bracketing vertex controllers only
  //     (current-step output uses x before update).
  double u_pair[2] = {0.0, 0.0};
  for (int sel = 0; sel < 2; ++sel) {
    const int j = idx + sel;
    const auto & ctrl = controllers_[j];
    u_pair[sel] = dotProduct(ctrl.Cd, ctrl.x) + dotProduct(ctrl.Dd, y);
  }

  // 5b. State update for ALL N vertex controllers in parallel.
  //     Standard polytopic LPV: each vertex runs its own observer-like
  //     filter on the shared input y; only outputs are interpolated.
  //     Updating only the bracketing pair leaves the other vertices'
  //     states stale, so any near-integrator content built up at one
  //     rho-bracket is dropped when rho crosses into another bracket.
  for (int j = 0; j < n_vertices_; ++j) {
    auto & ctrl = controllers_[j];
    Vector ax = matVecMul(ctrl.Ad, ctrl.x);
    Vector by = matVecMul(ctrl.Bd, y);
    for (int k = 0; k < n_states_; ++k) {
      ctrl.x[k] = ax[k] + by[k];
    }
  }

  // 6. Interpolate vertex outputs.
  const double u_lpv = (1.0 - alpha) * u_pair[0] + alpha * u_pair[1];

  // 7. Apply output_gain to the LPV feedback path.
  double delta_cmd = output_gain * u_lpv;

  // 8. Curvature feedforward — exact Ackermann steady-state.
  if (K_ff != 0.0 && std::isfinite(kappa_ff)) {
    delta_cmd += K_ff * std::atan(L * kappa_ff);
  }

  // 9. Saturate.
  const double delta_cmd_sat =
    std::max(-delta_max, std::min(delta_max, delta_cmd));

  // 10. Actuator-model observer for the 2-channel y_delta measurement:
  //     delta_est_{k+1} = delta_est_k + dt/tau_d * (delta_cmd_k - delta_est_k)
  // Matches the synthesis-side plant.delta channel used to build y_delta
  // (qcar2/matlab/synthesis/build_generalized_plant.m). Uses the final
  // saturated steering command as the input to the lag — same convention
  // as simulate_closed_loop.m.
  if (n_inputs_ >= 2 && tau_d_ > 1e-9) {
    delta_est_ += (dt_ / tau_d_) * (delta_cmd_sat - delta_est_);
  }

  return delta_cmd_sat;
}

}  // namespace lpv_hinf
}  // namespace qcar2_control_2
