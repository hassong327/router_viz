#ifndef QCAR2_CONTROL__LPV_HINF_RUNTIME_HPP_
#define QCAR2_CONTROL__LPV_HINF_RUNTIME_HPP_

#include <string>
#include <vector>

namespace qcar2_control_2
{
namespace lpv_hinf
{

// Both state dimension and measurement-channel count are determined
// dynamically at JSON load time:
//   - flat  B (length n) + scalar  D  → 1-channel (v3 / v3.1):
//       y = [e_psi]
//   - nested B (n × 2)   + length-2 D → 2-channel (v4):
//       y = [e_psi, delta_est]
// QCar2 has no steering encoder, so for 2-channel synthesis the
// runtime supplies y_delta from an actuator first-order-lag observer
// (delta_est_{k+1} = delta_est_k + dt/tau_d * (delta_cmd_k - delta_est_k))
// instead of measured steering.
//
// State dimension also varies between synthesis variants
// (e.g. 5 for v3.1, 6 for v4 with W_n included).

using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

/// Discrete-time state-space controller at a single polytope vertex.
struct VertexController
{
  Matrix Ad;        // n × n
  Matrix Bd;        // n × n_inputs
  Vector Cd;        // size n        (1×n row, flattened)
  Vector Dd;        // size n_inputs (1×n_inputs row, flattened)
  Vector x;         // size n        running state vector
};

/// LPV H-infinity controller with polytopic vertex interpolation.
///
/// Loads continuous-time A, B, C, D matrices from a JSON file,
/// discretises them via Tustin (bilinear) transform, and provides
/// a real-time compute() method that interpolates between the two
/// bracketing vertex controllers.
///
/// State dimension is read from the JSON ("controllers[0].A" size)
/// and may differ between synthesis variants.
class LpvHinfRuntime
{
public:
  /// Load controller from JSON, discretise with the given sample time.
  /// Returns false on error (reason written to \p error_msg).
  bool loadFromJson(const std::string & json_path, double dt,
                    std::string & error_msg);

  /// Reset all vertex controller states to zero.
  void reset();

  /// Compute steering command.
  /// @param e_psi        heading error [rad] (psi_des − psi)
  /// @param kappa_sched  curvature for scheduling parameter rho = |κ|·|v|
  /// @param v            vehicle speed [m/s]
  /// @param kappa_ff     curvature for feedforward term (lookahead)
  /// @return saturated steering command [rad]
  double compute(double e_psi,
                 double kappa_sched, double v, double kappa_ff);

  bool loaded() const { return loaded_; }
  int  numVertices() const { return n_vertices_; }
  int  numStates() const { return n_states_; }
  int  numInputs() const { return n_inputs_; }

  // Tuning parameters (set after loadFromJson or via ROS params).
  double K_ff        = 0.0;     // feedforward magnitude weight
  double rho_scale   = 1.0;     // multiplier on rho before vertex lookup
  double output_gain = 1.0;     // applied to LPV feedback output
  double delta_max   = 0.52;
  double L           = 0.256;

private:
  // Tustin (bilinear) discretisation of one vertex controller.
  // Operates on dynamically-sized matrices.
  static void tustinDiscretize(
    const Matrix & Ac, const Matrix & Bc,
    const Vector & Cc, const Vector & Dc,
    double dt,
    Matrix & Ad, Matrix & Bd, Vector & Cd, Vector & Dd);

  // n×n matrix inverse via Gauss-Jordan with partial pivoting.
  static bool invertNxN(const Matrix & M, Matrix & Minv);

  // Generic linear-algebra helpers.
  static Vector matVecMul(const Matrix & A, const Vector & x);
  static double dotProduct(const Vector & a, const Vector & b);
  static Matrix matMatMul(const Matrix & A, const Matrix & B);

  bool   loaded_     = false;
  int    n_vertices_ = 0;
  int    n_states_   = 0;
  int    n_inputs_   = 1;
  double dt_         = 0.005;   // sample time stored from loadFromJson
  double tau_d_      = 0.16;    // steering actuator time constant from JSON
  double delta_est_  = 0.0;     // actuator-model state for y_delta channel
  std::vector<double>           vertices_;
  std::vector<VertexController> controllers_;
};

}  // namespace lpv_hinf
}  // namespace qcar2_control_2

#endif  // QCAR2_CONTROL__LPV_HINF_RUNTIME_HPP_
