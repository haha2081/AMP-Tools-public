#include "ManipulatorSkeleton.h"
#include <Eigen/Dense>

struct Point
{

    double x, y;
};

Eigen::Vector2d calculateError(double x1, double y1, double x2, double y2)
{

    return {x1 - x2, y1 - y2};
}

std::vector<std::vector<double>> calculateJacobian(const std::vector<double> &lengths, const Eigen::VectorXd &angles)
{
    int n = angles.size();

    std::vector<std::vector<double>> jacobian(n, std::vector<double>(2, 0.0));

    for (int i = 0; i < n; i++)
    {
        double x_value = 0;
        double y_value = 0;

        for (int j = i; j < n; j++)
        {
            double theta = 0.0;

            for (int k = 0; k <= j; k++)
            {
                theta += angles[k];
            }

            x_value += -lengths[j] * sin(theta);
            y_value += lengths[j] * cos(theta);
        }

        jacobian[i][0] = x_value;
        jacobian[i][1] = y_value;
    }

    return jacobian;
}

MyManipulator2D::MyManipulator2D() : LinkManipulator2D({1, 1}) {}

Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState &state, uint32_t joint_index) const
{
    Eigen::Vector3d current_ref(0.0, 0.0, 1.0);
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();

    for (int i = 0; i <= joint_index; ++i)
    {

        double a = (i > 0) ? m_link_lengths[i - 1] : 0.0;
        double theta = (i < state.size()) ? state[i] : 0.0;

        Eigen::Matrix3d matrix;
        matrix << cos(theta), -sin(theta), a,
            sin(theta), cos(theta), 0.0,
            0.0, 0.0, 1.0;

        T = T * matrix;
    }


    Eigen::Vector3d transformed_position = T * current_ref;

    return Eigen::Vector2d(transformed_position[0], transformed_position[1]);
}



amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d &end_effector_location) const
{

    const double lower_bound = -M_PI;
    const double upper_bound = M_PI;

    double damping_factor = 2.5;
    double alpha = 1.1;
    double tol = 0.00001;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(lower_bound, upper_bound);

    amp::ManipulatorState angles(nLinks());
    amp::ManipulatorState joint_angles;

    const int max_iterations = 1000000;
    const int max_restarts = 50;

    int restart_count = 0;

    while (restart_count < max_restarts)
    {
        for (int i = 0; i < angles.size(); i++)
        {
            angles[i] = dist(gen);
        }

        Eigen::Vector2d position = getJointLocation(angles, nLinks());
        int counter = 0;

        while ((abs(position[0] - end_effector_location[0]) > tol || abs(position[1] - end_effector_location[1]) > tol) && counter < max_iterations)
        {
            counter++;
            position = getJointLocation(angles, nLinks());

            Eigen::Vector2d error = calculateError(end_effector_location[0], end_effector_location[1], position[0], position[1]);

            std::vector<std::vector<double>> temp_jacobian = calculateJacobian(m_link_lengths, angles);

            int rows = temp_jacobian.size();
            int cols = rows > 0 ? temp_jacobian[0].size() : 0;
            Eigen::MatrixXd eigenJacobian(rows, cols);

            for (int i = 0; i < rows; ++i)
            {
                for (int j = 0; j < cols; ++j)
                {
                    eigenJacobian(i, j) = temp_jacobian[i][j];
                }
            }

            Eigen::MatrixXd Jacobian = eigenJacobian.transpose();
            Eigen::MatrixXd temp = Jacobian.transpose() * Jacobian;

            Eigen::MatrixXd damping_identity = damping_factor * damping_factor * Eigen::MatrixXd::Identity(rows, rows);
            Eigen::MatrixXd result = ((temp + damping_identity).inverse() * Jacobian.transpose()) * error;

            for (int i = 0; i < angles.size(); i++)
            {
                angles[i] = angles[i] + alpha * result(i);
            }
        }

        if (abs(position[0] - end_effector_location[0]) <= tol && abs(position[1] - end_effector_location[1]) <= tol)
        {
            std::cout << "Converged in " << counter << " iterations." << std::endl;
            return angles;
        }
        else
        {
            std::cout << "Did not converge within " << max_iterations << " iterations. Restarting..." << std::endl;
            restart_count++;
        }
    }

    std::cout << "Failed to converge after " << max_restarts << " restarts." << std::endl;



    
    return angles; 


}