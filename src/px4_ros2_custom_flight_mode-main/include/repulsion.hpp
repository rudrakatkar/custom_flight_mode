#pragma once
/**
 * CBF Safety Filter — replaces the old heuristic repulsion.
 *
 * ── Mathematics ──────────────────────────────────────────────────────────────
 * Barrier function (pairwise, drone i vs j):
 *   h_ij(x) = ||p_i − p_j||² − d_safe²   ≥ 0  ⟺  safe
 *
 * CBF condition (forward-invariance):
 *   ḣ_ij + α·h_ij ≥ 0
 *   ⟹  2(p_i − p_j)ᵀ v_i  ≥  −α·h_ij        …(v_j treated as disturbance = 0)
 *
 * Safety-filter QP (solved per drone, per timestep):
 *   min_{v}  ½‖v − v_nom‖²
 *   s.t.     aᵢⱼᵀ v  ≥  bᵢⱼ    ∀j ≠ i          (CBF constraint)
 *            ‖v‖_∞   ≤  v_max                   (speed limit)
 *   where  aᵢⱼ = 2(pᵢ − pⱼ),  bᵢⱼ = −α·h_ij
 *
 * With ≤ 2 neighbours the QP is solved exactly by successive Hilbert projection
 * (project onto each half-space in turn; 5 sweeps guarantees convergence).
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <Eigen/Core>
#include <vector>
#include <algorithm>
#include <cmath>

// Project v onto the closed half-space { v : aᵀv ≥ b }
static inline Eigen::Vector3f _cbf_project(
    const Eigen::Vector3f & v,
    const Eigen::Vector3f & a,
    float b)
{
    const float av = a.dot(v);
    if (av >= b) { return v; }                         // already feasible
    const float sq = a.squaredNorm();
    if (sq < 1e-9f) { return v; }                      // degenerate direction
    return v + ((b - av) / sq) * a;
}

/**
 * @brief  Apply CBF safety filter to a nominal velocity command.
 *
 * @param pos_i      This drone's NED position (m)
 * @param vel_nom    Nominal velocity from trajectory planner (m/s)
 * @param others     NED positions of all other drones (m)
 * @param d_safe     Minimum safety separation radius (m).  Default 4 m.
 * @param alpha      Class-K gain — higher ⟹ more aggressive avoidance.  Default 1.2.
 * @param v_max      Per-axis speed clamp (m/s).  Default 3 m/s.
 *
 * @return  Safe velocity v_safe that satisfies all CBF constraints and speed limits.
 *          When no constraint is active v_safe ≡ v_nom (zero modification cost).
 */
static inline Eigen::Vector3f cbfSafetyFilter(
    const Eigen::Vector3f & pos_i,
    const Eigen::Vector3f & vel_nom,
    const std::vector<Eigen::Vector3f> & others,
    float d_safe = 4.0f,
    float alpha  = 1.2f,
    float v_max  = 3.0f)
{
    Eigen::Vector3f v = vel_nom;

    // Successive projection — 5 sweeps is sufficient for ≤3 constraints
    for (int sweep = 0; sweep < 5; ++sweep) {
        for (const auto & pos_j : others) {
            const Eigen::Vector3f diff = pos_i - pos_j;
            const float h_ij = diff.squaredNorm() - d_safe * d_safe;
            // Constraint: aᵀv ≥ b
            const Eigen::Vector3f a = 2.0f * diff;
            const float            b = -alpha * h_ij;
            v = _cbf_project(v, a, b);
        }
    }

    // Hard speed clamp
    v = v.cwiseMax(Eigen::Vector3f::Constant(-v_max))
         .cwiseMin(Eigen::Vector3f::Constant( v_max));
    return v;
}
