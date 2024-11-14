#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"

class MyKinoRRT : public amp::KinodynamicRRT {
private:
    int n;                 // Number of samples
    int u_samples;         // Number of control samples
    double pathLength;     // Path length of the planned path

public:
    std::vector<double> velocities;        // Store velocity data
    std::vector<double> steering_angles;   // Store steering angle data
    std::vector<double> times;  
    // Setters and Getters for n and u_samples
    void setNSamples(int n_samples) { n = n_samples; }
    int getNSamples() const { return n; }

    void setUSamples(int u) { u_samples = u; }
    int getUSamples() const { return u_samples; }

    // Getter for path length
    double getPathLength() const { return pathLength; }

    // Override the plan function
    virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;
};

class MySingleIntegrator : public amp::DynamicAgent {
public:
    virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;

    void RungeKutta4(Eigen::VectorXd& state, const Eigen::VectorXd& control, double dt);
};

class MyFirstOrderUnicycle : public amp::DynamicAgent {
public:
    virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;

    void RungeKutta4(Eigen::VectorXd& state, const Eigen::VectorXd& control, double dt);
};

class MySecondOrderUnicycle : public amp::DynamicAgent {
public:
    virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;

    void RungeKutta4(Eigen::VectorXd& state, const Eigen::VectorXd& control, double dt);
};

class MySimpleCar : public amp::DynamicAgent {
public:
    virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;

    void RungeKutta4(Eigen::VectorXd& state, const Eigen::VectorXd& control, double dt);
};
