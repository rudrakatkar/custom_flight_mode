#pragma once
#include <Eigen/Dense>
#include <vector>
#include <algorithm>

class CBFSafetyFilter {
public:
    CBFSafetyFilter(double d_safe, double gamma) : d_safe_(d_safe), gamma_(gamma) {}

    Eigen::Vector3d filter(const Eigen::Vector3d& p_i, 
                           const Eigen::Vector3d& u_i_nom, 
                           const std::vector<Eigen::Vector3d>& peers_p, 
                           const std::vector<Eigen::Vector3d>& peers_u) 
    {
        Eigen::Vector3d u_safe = u_i_nom;

        for (size_t j = 0; j < peers_p.size(); ++j) {
            Eigen::Vector3d dp = p_i - peers_p[j];
            double dist_sq = dp.squaredNorm();
            double h = dist_sq - (d_safe_ * d_safe_);

            // Prevent division by zero if drones physically crash
            if (dist_sq < 0.01) continue; 

            Eigen::Vector3d A = 2.0 * dp;
            double b = 2.0 * dp.dot(peers_u[j]) - gamma_ * h;

            // If safety is violated by nominal velocity, project it
            if (A.dot(u_safe) < b) {
                u_safe = u_safe + ((b - A.dot(u_safe)) / A.squaredNorm()) * A;
            }
        }
        
        // Velocity magnitude limiting to prevent aggressive discontinuities
        double max_vel = 5.0; // m/s
        if (u_safe.norm() > max_vel) {
            u_safe = u_safe.normalized() * max_vel;
        }

        return u_safe;
    }

private:
    double d_safe_;
    double gamma_;
};
