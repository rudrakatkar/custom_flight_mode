#pragma once
#include <Eigen/Dense>
#include <cmath>

class TrajectoryGenerator {
public:
    // Minimum jerk scaling function: 0 at t=0, 1 at t=T
    static double minJerk(double t, double T) {
        if (t <= 0) return 0.0;
        if (t >= T) return 1.0;
        double tau = t / T;
        return 10 * std::pow(tau, 3) - 15 * std::pow(tau, 4) + 6 * std::pow(tau, 5);
    }

    // Derivative of min jerk for velocity scaling
    static double minJerkVel(double t, double T) {
        if (t <= 0 || t >= T) return 0.0;
        double tau = t / T;
        return (30 * std::pow(tau, 2) - 60 * std::pow(tau, 3) + 30 * std::pow(tau, 4)) / T;
    }

    static void getStraightLine(double t, double T, const Eigen::Vector3d& start, const Eigen::Vector3d& end, 
                                Eigen::Vector3d& pos, Eigen::Vector3d& vel) {
        double s = minJerk(t, T);
        double s_dot = minJerkVel(t, T);
        pos = start + s * (end - start);
        vel = s_dot * (end - start);
    }

    static void getSemiCircle(double t, double T, const Eigen::Vector3d& center, double radius, 
                              double start_angle, double end_angle, 
                              Eigen::Vector3d& pos, Eigen::Vector3d& vel) {
        double s = minJerk(t, T);
        double s_dot = minJerkVel(t, T);
        
        double angle = start_angle + s * (end_angle - start_angle);
        double angle_dot = s_dot * (end_angle - start_angle);

        // Assuming drawing in X-Y plane (NED)
        pos = center + Eigen::Vector3d(radius * std::cos(angle), radius * std::sin(angle), 0.0);
        vel = Eigen::Vector3d(-radius * std::sin(angle) * angle_dot, radius * std::cos(angle) * angle_dot, 0.0);
    }
};
