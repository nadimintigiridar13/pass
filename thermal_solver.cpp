#include "thermal/thermal_solver.h"
#include "common/linear_algebra.h"
#include "common/validation.h"
#include <cmath>
#include <algorithm>

namespace workbench {
namespace thermal {

double analytical_solution(double x, const ThermalParams& params) {
    double L = params.L;
    double k = params.k;
    double q = params.q;
    double T0 = params.T_left;
    double TL = params.T_right;
    // T(x) = T0 + (TL - T0)*x/L + q/(2k)*x*(L - x)
    return T0 + (TL - T0) * x / L + (q / (2.0 * k)) * x * (L - x);
}

Result<ThermalResult> solve(const ThermalParams& params) {
    // Validate inputs
    std::string err = validation::validate_thermal_params(
        params.L, params.A, params.k, params.q,
        params.T_left, params.T_right, params.num_elements);
    if (!err.empty()) {
        return Result<ThermalResult>::Err(ErrorCode::InvalidInput, err);
    }

    int N = params.num_elements;
    int num_nodes = N + 1;
    double Le = params.L / N;  // element length
    double k = params.k;
    double A = params.A;
    double q = params.q;

    // Element stiffness coefficient: kA/Le
    double ke_coeff = k * A / Le;
    // Element load coefficient: qA*Le/2
    double fe_coeff = q * A * Le / 2.0;

    // For the 1D rod problem, the global stiffness matrix is tridiagonal.
    // We build it directly in tridiagonal form for Thomas algorithm.
    //
    // Global system K*T = F
    // K is tridiagonal with:
    //   diagonal[i]:      sum of contributions from adjacent elements
    //   sub/super-diag:   -kA/Le (coupling between adjacent nodes)

    linalg::Vector diag(num_nodes, 0.0);     // main diagonal
    linalg::Vector sub(num_nodes - 1, 0.0);  // sub-diagonal (a)
    linalg::Vector sup(num_nodes - 1, 0.0);  // super-diagonal (c)
    linalg::Vector rhs(num_nodes, 0.0);       // right-hand side

    // Assemble element by element
    for (int e = 0; e < N; ++e) {
        int n0 = e;
        int n1 = e + 1;

        // Element stiffness Ke = ke_coeff * [[1, -1], [-1, 1]]
        diag(n0) += ke_coeff;
        diag(n1) += ke_coeff;
        // Off-diagonal coupling
        if (e < N - 1) {
            sup(e) += -ke_coeff;       // K[e][e+1]
        }
        if (e > 0) {
            sub(e - 1) += -ke_coeff;   // K[e][e-1] — wait, we need to be more careful
        }

        // Element load fe = fe_coeff * [1, 1]
        rhs(n0) += fe_coeff;
        rhs(n1) += fe_coeff;
    }

    // Actually, let's be more precise about the tridiagonal assembly.
    // For element e connecting nodes e and e+1:
    //   K[e][e] += ke_coeff
    //   K[e][e+1] += -ke_coeff
    //   K[e+1][e] += -ke_coeff
    //   K[e+1][e+1] += ke_coeff
    //
    // The super-diagonal sup[i] = K[i][i+1]
    // The sub-diagonal sub[i] = K[i+1][i]
    //
    // So for element e:
    //   sup[e] += -ke_coeff  (this is K[e][e+1])
    //   sub[e] += -ke_coeff  (this is K[e+1][e])

    // Let me redo this cleanly
    diag = linalg::Vector(num_nodes, 0.0);
    sub = linalg::Vector(num_nodes - 1, 0.0);
    sup = linalg::Vector(num_nodes - 1, 0.0);
    rhs = linalg::Vector(num_nodes, 0.0);

    for (int e = 0; e < N; ++e) {
        int n0 = e;
        int n1 = e + 1;

        // Stiffness
        diag(n0) += ke_coeff;
        diag(n1) += ke_coeff;
        sup(e) += -ke_coeff;   // K[n0][n1] = K[e][e+1]
        sub(e) += -ke_coeff;   // K[n1][n0] = K[e+1][e]

        // Load
        rhs(n0) += fe_coeff;
        rhs(n1) += fe_coeff;
    }

    // Apply Dirichlet BCs: T(0) = T_left, T(N) = T_right
    // Node 0:
    diag(0) = 1.0;
    sup(0) = 0.0;
    rhs(0) = params.T_left;
    // Also zero out sub-diagonal contribution to node 0 (sub[0] is K[1][0])
    // Modify rhs[1] to account for the known value:
    // Actually for the Thomas algorithm, we modify:
    //   K[1][0] was sub[0] = -ke_coeff
    //   rhs[1] -= sub[0] * T_left  (but we set rhs[0] = T_left, so...)
    //   sub[0] = 0
    // Wait — let me just do the standard BC application properly.

    // Standard approach: for each BC node i with value T_bc:
    //   For all j != i: rhs[j] -= K[j][i] * T_bc
    //   Set row i and col i to 0, K[i][i] = 1, rhs[i] = T_bc

    // Node 0 BC:
    // K[1][0] = sub[0], so rhs[1] -= sub[0] * T_left
    rhs(1) -= sub(0) * params.T_left;
    diag(0) = 1.0;
    sup(0) = 0.0;
    sub(0) = 0.0;
    rhs(0) = params.T_left;

    // Node N BC:
    // K[N-1][N] = sup[N-1], so rhs[N-1] -= sup[N-1] * T_right
    rhs(num_nodes - 2) -= sup(num_nodes - 2) * params.T_right;
    diag(num_nodes - 1) = 1.0;
    sub(num_nodes - 2) = 0.0;
    sup(num_nodes - 2) = 0.0;
    rhs(num_nodes - 1) = params.T_right;

    // Solve using Thomas algorithm
    auto solve_result = linalg::solve_tridiagonal(sub, diag, sup, rhs);
    if (solve_result.has_error()) {
        return Result<ThermalResult>::Err(ErrorCode::SolverError,
            "Thomas algorithm failed: " + solve_result.error().message);
    }

    const auto& T = solve_result.value();

    // Build result
    ThermalResult result;
    result.num_elements = N;
    result.params = params;

    // Node positions and temperatures
    result.node_positions.resize(num_nodes);
    result.temperatures.resize(num_nodes);
    result.exact_temperatures.resize(num_nodes);
    for (int i = 0; i < num_nodes; ++i) {
        double x = i * Le;
        result.node_positions[i] = x;
        result.temperatures[i] = T(i);
        result.exact_temperatures[i] = analytical_solution(x, params);
    }

    // Midpoint data: FE interpolation at element midpoints
    result.midpoint_positions.resize(N);
    result.midpoint_fe_temps.resize(N);
    result.midpoint_exact_temps.resize(N);
    result.midpoint_errors.resize(N);
    result.max_error = 0.0;

    for (int e = 0; e < N; ++e) {
        double x_mid = (e + 0.5) * Le;
        result.midpoint_positions[e] = x_mid;

        // FE interpolation: average of two nodal values (linear element midpoint)
        double T_fe_mid = 0.5 * (T(e) + T(e + 1));
        result.midpoint_fe_temps[e] = T_fe_mid;

        double T_exact_mid = analytical_solution(x_mid, params);
        result.midpoint_exact_temps[e] = T_exact_mid;

        double error = std::abs(T_fe_mid - T_exact_mid);
        result.midpoint_errors[e] = error;
        result.max_error = std::max(result.max_error, error);
    }

    return Result<ThermalResult>::Ok(std::move(result));
}

Result<ConvergenceResult> convergence_study(const ThermalParams& base_params) {
    std::vector<int> mesh_sizes = {10, 20, 40, 80};
    ConvergenceResult conv_result;

    for (int N : mesh_sizes) {
        ThermalParams params = base_params;
        params.num_elements = N;

        auto result = solve(params);
        if (result.has_error()) {
            return Result<ConvergenceResult>::Err(result.error().code,
                "Solver failed for N=" + std::to_string(N) + ": " + result.error().message);
        }

        ConvergencePoint point;
        point.num_elements = N;
        point.max_error = result.value().max_error;
        point.element_size = params.L / N;
        conv_result.points.push_back(point);
        conv_result.solutions.push_back(std::move(result.value()));
    }

    return Result<ConvergenceResult>::Ok(std::move(conv_result));
}

} // namespace thermal
} // namespace workbench
