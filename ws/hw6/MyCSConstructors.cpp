#include "MyCSConstructors.h"
#include <iostream>
////////////////////// THIS IS FROM HW4 //////////////////////
struct Point
{
    double x, y;
};

using Eigen::Vector2d;

int orientation(Vector2d p, Vector2d q, Vector2d r)
{
    double val = (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());
    const double EPSILON = 1e-9;
    if (std::fabs(val) < EPSILON)
        return 0;
    return (val > 0) ? 1 : 2;
}

bool onSegment(Vector2d p, Vector2d q, Vector2d r)
{
    const double EPSILON = 1e-9;
    /// std::cout << "Checking if point " << q << " is on segment " << p << " - " << r << std::endl;
    if (q.x() <= std::max(p.x(), r.x()) + EPSILON && q.x() >= std::min(p.x(), r.x()) - EPSILON &&
        q.y() <= std::max(p.y(), r.y()) + EPSILON && q.y() >= std::min(p.y(), r.y()) - EPSILON)
        return true;
    return false;
}

bool doIntersect(Vector2d p1, Vector2d q1, Vector2d p2, Vector2d q2)
{
    // std::cout << "Checking intersection between segment " << p1 << "-" << q1 << " and " << p2 << "-" << q2 << std::endl;
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4)
        return true;

    if (o1 == 0 && onSegment(p1, p2, q1))
        return true;
    if (o2 == 0 && onSegment(p1, q2, q1))
        return true;
    if (o3 == 0 && onSegment(p2, p1, q2))
        return true;
    if (o4 == 0 && onSegment(p2, q1, q2))
        return true;

    return false;
}

bool lineIntersectsPolygon(Vector2d lineStart, Vector2d lineEnd, const std::vector<Vector2d> &polygon)
{
    int n = polygon.size();
    // std::cout << "Checking line intersection with polygon of " << n << " vertices." << std::endl;

    for (int i = 0; i < n; ++i)
    {
        Vector2d polygonStart = polygon[i];
        Vector2d polygonEnd = polygon[(i + 1) % n];

        if (doIntersect(lineStart, lineEnd, polygonStart, polygonEnd))
        {
            // std::cout << "Intersection detected between line and polygon edge " << polygonStart << " - " << polygonEnd << std::endl;
            return true;
        }
    }
    return false;
}

bool hasNeighbors(const std::vector<std::vector<int>> &grid, int i, int j, int counter)
{
    int rows = grid.size();
    int cols = grid[0].size();
    // std::cout << "Checking neighbors for cell (" << i << ", " << j << ") with value " << counter << std::endl;

    if (i > 0 && grid[i - 1][j] == counter)
        return true;
    if (i < rows - 1 && grid[i + 1][j] == counter)
        return true;
    if (j > 0 && grid[i][j - 1] == counter)
        return true;
    if (j < cols - 1 && grid[i][j + 1] == counter)
        return true;

    return false;
}

bool isValid(int ni, int nj, int rows, int cols)
{
    // std::cout << "Checking if (" << ni << ", " << nj << ") is within bounds (" << rows << ", " << cols << ")" << std::endl;
    return (ni >= 0 && ni < rows && nj >= 0 && nj < cols);
}

bool isPointInPolygon(const Eigen::Vector2d &point, const std::vector<Eigen::Vector2d> &vertices)
{
    int intersections = 0;
    int numVertices = vertices.size();
    // std::cout << "Checking if point " << point << " is inside a polygon with " << numVertices << " vertices." << std::endl;

    for (int i = 0; i < numVertices; ++i)
    {
        const Eigen::Vector2d &v1 = vertices[i];
        const Eigen::Vector2d &v2 = vertices[(i + 1) % numVertices];

        if ((v1.y() > point.y()) != (v2.y() > point.y()))
        {
            double xIntersection = v1.x() + (v2.x() - v1.x()) * (point.y() - v1.y()) / (v2.y() - v1.y());
            // std::cout << "xIntersection: " << xIntersection << std::endl;
            if (point.x() < xIntersection)
            {
                intersections++;
            }
        }
    }

    return (intersections % 2) != 0;
}

void printGrid(const std::vector<std::vector<int>> &grid)
{
    int rows = grid.size();
    int cols = grid[0].size();
    // std::cout << "Printing grid (" << rows << "x" << cols << "):" << std::endl;

    for (int i = rows - 1; i >= 0; i--)
    {
        for (int j = 0; j < cols; j++)
        {
            std::cout << grid[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

/* You can just move these classes to shared folder and include them instead of copying them to hw6 project*/

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const
{
    double grid_size_x = MyGridCSpace2D::size().first;
    double grid_size_y = MyGridCSpace2D::size().second;

    double x0_min = MyGridCSpace2D::x0Bounds().first;
    double x0_max = MyGridCSpace2D::x0Bounds().second; // No subtraction by 1 here
    double x1_min = MyGridCSpace2D::x1Bounds().first;
    double x1_max = MyGridCSpace2D::x1Bounds().second; // No subtraction by 1 here

    // Normalize the coordinates (x0, x1) into [0, 1] based on bounds
    double norm_x0 = (x0 - x0_min) / (x0_max - x0_min);
    double norm_x1 = (x1 - x1_min) / (x1_max - x1_min);

    // Multiply by grid size and floor to get the cell indices
    int cell_x = static_cast<int>(round(norm_x0 * grid_size_x));
    int cell_y = static_cast<int>(round(norm_x1 * grid_size_y));

    // Ensure indices are within the valid range [0, grid_size - 1]
    if (cell_x < 0)
        cell_x = 0;
    if (cell_x >= grid_size_x - 1)
        cell_x = grid_size_x - 1;
    if (cell_y < 0)
        cell_y = 0;
    if (cell_y >= grid_size_y - 1)
        cell_y = grid_size_y - 1;

    return {static_cast<std::size_t>(cell_x), static_cast<std::size_t>(cell_y)};
}

// Override this method for computing all of the boolean collision values for each cell in the cspace

std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D &manipulator, const amp::Environment2D &env)
{
    // std::cout << "Constructing MyManipulatorCSConstructor with grid size " << m_cells_per_dim << std::endl;

    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(
        m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    MyGridCSpace2D &cspace = *cspace_ptr;

    double theta1 = env.x_min, theta2 = env.y_min;
    int grid_size = m_cells_per_dim;

    for (int i = 0; i < grid_size; i++)
    {

        theta2 = env.y_min;
        for (int j = 0; j < grid_size; j++)
        {

            for (int k = 0; k < env.obstacles.size(); k++)
            {

                Vector2d config;
                config << theta1, theta2;

                Eigen::Vector2d joint1 = {manipulator.getJointLocation(config, 1)[0], manipulator.getJointLocation(config, 1)[1]};
                Eigen::Vector2d joint2 = {manipulator.getJointLocation(config, 2)[0], manipulator.getJointLocation(config, 2)[1]};

                if (lineIntersectsPolygon({0, 0}, joint1, env.obstacles[k].verticesCCW()) ||
                    lineIntersectsPolygon(joint1, joint2, env.obstacles[k].verticesCCW()))
                {

                    int x_cell = cspace.getCellFromPoint(theta1, theta2).first;
                    int y_cell = cspace.getCellFromPoint(theta1, theta2).second;

                    cspace(x_cell, y_cell) = true;
                }
            }

            theta2 += (env.y_max - env.y_min) / grid_size;
        }

        theta1 += (env.x_max - env.x_min) / grid_size;
    }

    return cspace_ptr;
}

double wrapToPi(double angle)
{
    const double TWO_PI = 2.0 * M_PI;

    angle = fmod(angle, TWO_PI);
    if (angle < 0)
    {
        angle += TWO_PI;
    }

    if (angle > M_PI)
    {
        angle -= TWO_PI;
    }

    return angle;
}

//////////////////////////////////////////////////////////////

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D &env)
{
    std::cout << "Constructing MyPointAgentCSConstructor with grid size " << m_cells_per_dim << std::endl;

    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    MyGridCSpace2D &cspace = *cspace_ptr;

    double resolution_x = (abs(env.x_max - env.x_min)) / m_cells_per_dim;
    double resolution_y = (abs(env.y_max - env.y_min)) / m_cells_per_dim;

    for (int i = 0; i < m_cells_per_dim; i++)
    {
        for (int j = 0; j < m_cells_per_dim; j++)
        {
            for (int k = 0; k < env.obstacles.size(); k++)
            {

                if (isPointInPolygon(Eigen::Vector2d(i * resolution_x + env.x_min, j * resolution_y + env.y_min), env.obstacles[k].verticesCCW()))
                {
                    cspace(i, j) = true;
                }
            }
        }
    }
    return cspace_ptr;
}
amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d &q_init, const Eigen::Vector2d &q_goal, const amp::GridCSpace2D &grid_cspace, bool isManipulator)
{
    std::cout << "Planning path in C-Space from " << q_init[0] << ", " << q_init[1] << " to " << q_goal[0] << ", " << q_goal[1] << std::endl;

    Eigen::Vector2d q_init_wrapped = q_init;

    amp::Path2D path;
    if (isManipulator)
    {
        q_init_wrapped[0] = wrapToPi(q_init_wrapped[0]);
        q_init_wrapped[1] = wrapToPi(q_init_wrapped[1]);
    }
    Eigen::Vector2d q_goal_wrapped = q_goal;

        if (isManipulator)
    {
        q_goal_wrapped[0] = wrapToPi(q_goal_wrapped[0]);
        q_goal_wrapped[1] = wrapToPi(q_goal_wrapped[1]);
    }


    path.waypoints.push_back(q_init_wrapped);

    // Get the number of rows and columns from the configuration space
    int rows = grid_cspace.size().second;
    int cols = grid_cspace.size().first;

    // Initialize the grid with zeros
    std::vector<std::vector<int>> grid(rows, std::vector<int>(cols, 0));

    // Calculate resolution in x and y directions
    double resolution_x = (grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first) / cols;
    double resolution_y = (grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first) / rows;

    // Fill the grid with collision information
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            double x = j * resolution_x + grid_cspace.x0Bounds().first;
            double y = i * resolution_y + grid_cspace.x1Bounds().first;
            if (grid_cspace.inCollision(x, y))
            {
                grid[i][j] = 1;
            }
        }
    }

    int n = 2;
    std::vector<std::vector<int>> grid_padded = grid;

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            if (grid[i][j] == 1)
            {
                // Obstacle cell found, add padding around it
                for (int di = -n; di <= n; di++)
                {
                    for (int dj = -n; dj <= n; dj++)
                    {
                        if (di * di + dj * dj <= n * n)
                        { 
                            int ni = i + di;
                            int nj = j + dj;
                            if (ni >= 0 && ni < rows && nj >= 0 && nj < cols)
                            {
                                grid_padded[ni][nj] = 1;
                            }
                        }
                    }
                }
            }
        }
    }

    // Replace the original grid with the padded grid
    grid = grid_padded;

    // Get goal and start cells
    auto goal_cell = grid_cspace.getCellFromPoint(q_goal_wrapped[0], q_goal_wrapped[1]);
    auto start_cell = grid_cspace.getCellFromPoint(q_init_wrapped[0], q_init_wrapped[1]);

    // Mark the goal cell
    grid[goal_cell.second][goal_cell.first] = 2;

    // Initialize variables for flood fill
    bool containsZero = true;
    int counter = 2;
    int max_iterations = rows * cols * 10; // Set a maximum number of iterations to avoid infinite loop.
    int iteration_count = 0;

    // Flood fill to mark distances
    while (containsZero && iteration_count < max_iterations)
    {
        containsZero = false;
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                if (grid[j][i] == 0 && hasNeighbors(grid, j, i, counter))
                {
                    grid[j][i] = counter + 1;
                    containsZero = true;
                }
            }
        }
        counter += 1;
        iteration_count++;
    }

    if (iteration_count >= max_iterations)
    {
        std::cerr << "Reached maximum iterations during wavefront. Potential infinite loop detected." << std::endl;
        path.valid = false;

        return path;
    }
    // Validate that the start cell has a path to the goal

    if (grid[start_cell.second][start_cell.first] == 0)
    {
        std::cerr << "No valid path from start to goal." << std::endl;
        path.valid = false;

        return path;
    }

    // Trace back the path from start to goal
    std::pair<std::size_t, std::size_t> current_point = start_cell;
    int value = grid[current_point.second][current_point.first];
    int directions[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    // Set a maximum path length to avoid infinite loop during tracing
    // max_iterations = rows * cols;
    iteration_count = 0;

   std::set<std::pair<int, int>> visited;
while (value != 2 && iteration_count < max_iterations) {
    if (visited.count(current_point) > 0) {
        std::cerr << "Detected loop at point: " << current_point.first << ", " << current_point.second << std::endl;
        path.valid = false;
        return path;
    }
    visited.insert(current_point);
        int min_value = value;
        int next_j = current_point.second;
        int next_i = current_point.first;

        for (int d = 0; d < 4; ++d)
        {
            int nj = current_point.second + directions[d][0];
            int ni = current_point.first + directions[d][1];

            if (isValid(ni, nj, rows, cols))
            {
                if (grid[nj][ni] < min_value && grid[nj][ni] != 1)
                {
                    min_value = grid[nj][ni];
                    next_i = ni;
                    next_j = nj;
                }
            }
        }

        // Move to the next cell
        current_point = std::make_pair(next_i, next_j);
        value = grid[next_j][next_i];

        // Calculate the real-world coordinates of the cell
        double x = next_i * resolution_x + grid_cspace.x0Bounds().first;
        double y = next_j * resolution_y + grid_cspace.x1Bounds().first;

        // if (isManipulator)
        // {
        //     x = wrapToPi(x);
        //     y = wrapToPi(y);
        // }

        // Add the waypoint to the path
        path.waypoints.push_back(Eigen::Vector2d(x, y));

        iteration_count++;
    }

    // Check if maximum iterations were reached during path tracing
    if (iteration_count >= max_iterations)
    {
        std::cerr << "Reached maximum iterations while tracing path. Potential infinite loop detected." << std::endl;
                path.valid = false;

        return path;
    }

    // Add the goal point as the final waypoint
    path.waypoints.push_back(q_goal_wrapped);
    path.waypoints.push_back(q_goal_wrapped);

    // Print the length of the path
    std::cout << "The length of the path is " << path.length() << std::endl;

    
    return path;
}
