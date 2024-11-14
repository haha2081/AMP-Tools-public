// #include "MyKinoRRT.h"

// using Eigen::Vector2d;
// using namespace std;

// double carLength;

// Eigen::VectorXd state_temp;
// Eigen::VectorXd state_temp2;

// struct Node
// {
//     int id;
//     float g = 0;                      // Cost from start to this node
//     float h = 0;                      // Heuristic estimate of cost from this node to the goal
//     float f() const { return g + h; } // Total estimated cost
//     std::vector<int> parent;          // To reconstruct the path

//     bool operator<(const Node &other) const
//     {
//         return f() > other.f();
//     }
// };

// int orientation(Vector2d p, Vector2d q, Vector2d r)
// {
//     double val = (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());
//     const double EPSILON = 1e-9;
//     if (std::fabs(val) < EPSILON)
//         return 0;
//     return (val > 0) ? 1 : 2;
// }

// bool onSegment(Vector2d p, Vector2d q, Vector2d r)
// {
//     const double EPSILON = 1e-9;
//     if (q.x() <= std::max(p.x(), r.x()) + EPSILON && q.x() >= std::min(p.x(), r.x()) - EPSILON &&
//         q.y() <= std::max(p.y(), r.y()) + EPSILON && q.y() >= std::min(p.y(), r.y()) - EPSILON)
//         return true;
//     return false;
// }

// bool doIntersect(Vector2d p1, Vector2d q1, Vector2d p2, Vector2d q2)
// {
//     int o1 = orientation(p1, q1, p2);
//     int o2 = orientation(p1, q1, q2);
//     int o3 = orientation(p2, q2, p1);
//     int o4 = orientation(p2, q2, q1);

//     if (o1 != o2 && o3 != o4)
//         return true;

//     if (o1 == 0 && onSegment(p1, p2, q1))
//         return true;
//     if (o2 == 0 && onSegment(p1, q2, q1))
//         return true;
//     if (o3 == 0 && onSegment(p2, p1, q2))
//         return true;
//     if (o4 == 0 && onSegment(p2, q1, q2))
//         return true;

//     return false;
// }

// bool lineIntersectsPolygon(Vector2d lineStart, Vector2d lineEnd, const std::vector<Vector2d> &polygon)
// {
//     int n = polygon.size();

//     for (int i = 0; i < n; ++i)
//     {
//         Vector2d polygonStart = polygon[i];
//         Vector2d polygonEnd = polygon[(i + 1) % n];

//         if (doIntersect(lineStart, lineEnd, polygonStart, polygonEnd))
//         {
//             return true;
//         }
//     }
//     return false;
// }

// Eigen::VectorXd generateRandomNVector(const std::vector<double> &lower_bounds, const std::vector<double> &upper_bounds)
// {
//     if (lower_bounds.size() != upper_bounds.size())
//     {
//         throw std::invalid_argument("Erororororororrr");
//     }

//     int n = lower_bounds.size();
//     Eigen::VectorXd random_vector(n);
//     std::random_device rd;
//     std::mt19937 gen(rd());
//     for (int i = 0; i < n; ++i)
//     {
//         std::uniform_real_distribution<float> dist(lower_bounds[i], upper_bounds[i]);
//         random_vector[i] = dist(gen);
//     }

//     return random_vector;
// }

// double euclideanDistance(const Eigen::VectorXd &point1, const Eigen::VectorXd &point2)
// {
//     if (point1.size() != point2.size())
//     {
//         throw std::invalid_argument("Both points must have the same number of dimensions.");
//     }

//     return (point1 - point2).norm();
// }

// bool isPointInPolygon(const Eigen::Vector2d &point, const std::vector<Eigen::Vector2d> &vertices)
// {
//     int intersections = 0;
//     int numVertices = vertices.size();

//     for (int i = 0; i < numVertices; ++i)
//     {
//         const Eigen::Vector2d &v1 = vertices[i];
//         const Eigen::Vector2d &v2 = vertices[(i + 1) % numVertices];

//         if ((v1.y() > point.y()) != (v2.y() > point.y()))
//         {
//             double xIntersection = v1.x() + (v2.x() - v1.x()) * (point.y() - v1.y()) / (v2.y() - v1.y());
//             if (point.x() < xIntersection)
//             {
//                 intersections++;
//             }
//         }
//     }

//     return (intersections % 2) != 0;
// }

// void MySingleIntegrator::propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt)
// {
//     state += dt * control;
// }

// std::vector<Eigen::Vector2d> getCarCorners(const Eigen::VectorXd &state, double L, double W)
// {
//     double x = state[0];     // x position of the rear axle center
//     double y = state[1];     // y position of the rear axle center
//     double theta = state[2]; // orientation (angle)

//     Eigen::Matrix2d rotation;
//     rotation << cos(theta), -sin(theta),
//         sin(theta), cos(theta);

//     // Rear axle center is the origin of the local frame
//     Eigen::Vector2d rear_center(x, y);

//     // Half-lengths and half-widths for corner calculations
//     Eigen::Vector2d half_LW(L / 2.0, W / 2.0);

//     // Corners in local coordinates
//     std::vector<Eigen::Vector2d> local_corners = {
//         {-half_LW[0], -half_LW[1]},
//         {half_LW[0], -half_LW[1]},
//         {half_LW[0], half_LW[1]},
//         {-half_LW[0], half_LW[1]}};

//     // Transform corners to global coordinates
//     std::vector<Eigen::Vector2d> global_corners;
//     for (const auto &corner : local_corners)
//     {
//         global_corners.push_back(rear_center + rotation * corner);
//     }

//     return global_corners;
// }

// bool checkCollisionWithObstacles(const Eigen::VectorXd &state, const std::__1::vector<amp::Polygon> &obstacles, double L, double W)
// {
//     std::vector<Eigen::Vector2d> car_corners = getCarCorners(state, L, W);

//     for (const auto &obstacle : obstacles)
//     {
//         for (const auto &corner : car_corners)
//         {
//             if (isPointInPolygon(corner, obstacle.verticesCCW()))
//             {
//                 return true; // A corner is inside the obstacle
//             }
//         }

//         for (size_t i = 0; i < car_corners.size(); ++i)
//         {
//             Eigen::Vector2d car_start = car_corners[i];
//             Eigen::Vector2d car_end = car_corners[(i + 1) % car_corners.size()];

//             for (size_t j = 0; j < obstacle.verticesCCW().size(); ++j)
//             {
//                 Eigen::Vector2d obs_start = obstacle.verticesCCW()[j];
//                 Eigen::Vector2d obs_end = obstacle.verticesCCW()[(j + 1) % obstacle.verticesCCW().size()];

//                 if (lineIntersectsPolygon(car_start, car_end, {obs_start, obs_end}))
//                 {
//                     return true;
//                 }
//             }
//         }
//     }

//     return false; // No collision detected
// }

// amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D &problem, amp::DynamicAgent &agent)
// {
//     amp::KinoPath path;
//     Eigen::VectorXd state = problem.q_init;
//     state_temp2 = state;
//     state_temp = state;

//     carLength = agent.agent_dim.length;

//     double total_time = 0.0;
//     double max_vals = 10;
//     double radius = 2.5;
//     int n = 1000000;
//     double goal_tolerance = 1;
//     double step_size = 1;
//     double goal_probability = 0.05;
//     double distance, distance2;
//     double delta_t = 0.1;

//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_real_distribution<> dis(0.0, 1.0);

//     int counter = 0;
//     int qnear_counter = 0;
//     Eigen::VectorXd qrand, qnear, random_control;
//     std::vector<double> lower_bounds(problem.q_bounds.size());
//     std::vector<double> upper_bounds(problem.q_bounds.size());

//     // Initialize lower and upper bounds for the state space
//     for (int i = 0; i < problem.q_bounds.size(); ++i)
//     {
//         lower_bounds[i] = problem.q_bounds[i].first;
//         upper_bounds[i] = problem.q_bounds[i].second;
//     }

//     std::vector<Eigen::VectorXd> points;
//     std::map<int, Eigen::VectorXd> nodes;

//     std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();

//     path.waypoints.push_back(state);
//     points.push_back(state);
//     nodes[counter] = state;
//     counter++;

//     bool solutionNotFound = true;
//     int max_iterations = n;
//     int iteration_count = 0;

//     Eigen::VectorXd centers(problem.q_goal.size());

//     // Calculate the center of the goal region
//     for (int i = 0; i < problem.q_goal.size(); ++i)
//     {
//         double center = (problem.q_goal[i].first + problem.q_goal[i].second) / 2.0;
//         centers[i] = center;
//     }

//     // Debug: Print the centers of the goal region
//     // std::cout << "Goal Centers: " << centers.transpose() << std::endl;

//     // Initialize control input bounds
//     std::vector<double> u_lower_bounds;
//     std::vector<double> u_upper_bounds;

//     for (const auto &bound : problem.u_bounds)
//     {
//         u_lower_bounds.push_back(bound.first);
//         u_upper_bounds.push_back(bound.second);
//     }

//     // Main loop
//     int debug_interval = 10000; // Adjust this value to print debug information every 'n' iterations

//     while (solutionNotFound && iteration_count < max_iterations)
//     {
//         if (iteration_count % debug_interval == 0)
//         {
//             std::cout << "Iteration: " << iteration_count << std::endl;
//         }

//         distance = std::numeric_limits<double>::infinity();
//         double random_value = dis(gen);

//         // Sample a random state
//         if (random_value < goal_probability)
//         {
//             qrand = centers;
//             if (iteration_count % debug_interval == 0)
//             {
//                 // std::cout << "Sampling goal state as qrand." << std::endl;
//             }
//         }
//         else
//         {
//             qrand = generateRandomNVector(lower_bounds, upper_bounds);
//             if (iteration_count % debug_interval == 0)
//             {
//                 // std::cout << "Sampling random state as qrand: " << qrand.transpose() << std::endl;
//             }
//         }
//         // std::cout << "points size: " << points.size() << std::endl;

//         // Find the nearest node to the random state
//         for (int i = 0; i < points.size(); ++i)
//         {
//             double dist = euclideanDistance(points[i], qrand);
//             //  std::cout << "points[i]: " << points[i].transpose() << ",\n qrand: " << qrand.transpose()  << std::endl;
//             if (dist < distance)
//             {
//                 distance = dist;
//                 qnear = points[i];
//                 qnear_counter = i;
//             }
//         }

//         // std::cout << "qnear: " << qnear.transpose() << std::endl;

//         // Debug: Print nearest node information
//         if (iteration_count % debug_interval == 0)
//         {
//             // std::cout << "Nearest node index: " << qnear_counter << ", Distance: " << distance << std::endl;
//             // std::cout << "qnear: " << qnear.transpose() << std::endl;
//         }

//         // Initialize temporary states
//         state_temp = qnear;
//         state_temp2 = qnear;

//         // Generate a random control input and propagate the state
//         random_control = generateRandomNVector(u_lower_bounds, u_upper_bounds);

//         // std::cout << "before propagation: " << state_temp[0] << ", " << state_temp[1] << std::endl;
//         agent.propagate(state_temp, random_control, delta_t);
//         //  std::cout << "after propagation: "  << state_temp[0] << ", " << state_temp[1] << std::endl;

//         distance2 = euclideanDistance(state_temp, qrand);

//         // Try multiple control inputs to find the best one
//         for (int i = 0; i < max_vals - 1; ++i)
//         {
//             state_temp2 = qnear; // Reset state_temp2 to the nearest state
//             auto random_control2 = generateRandomNVector(u_lower_bounds, u_upper_bounds);

//             agent.propagate(state_temp2, random_control2, delta_t);

//             double dist2 = euclideanDistance(state_temp2, qrand);
//             if (dist2 < distance2)
//             {
//                 random_control = random_control2;
//                 distance2 = dist2;
//                 state_temp = state_temp2;
//             }
//         }

//         // Debug: Print selected control and resulting state
//         if (iteration_count % debug_interval == 0)
//         {
//             // std::cout << "Selected control: " << random_control.transpose() << std::endl;
//             // std::cout << "New state after propagation: " << state_temp.transpose() << std::endl;
//         }

//         // Check for collisions
//         // std::cout << "state_temp: " << state_temp[0] << ", " << state_temp[1] << std::endl;

//         bool collision = checkCollisionWithObstacles(state_temp, problem.obstacles, agent.agent_dim.length, agent.agent_dim.width);

//         if (!collision)
//         {
//             points.push_back(state_temp);
//             nodes[counter] = state_temp;
//             graphPtr->connect(qnear_counter, counter, distance);
//             counter++;

//             // Check if the goal is reached
//             Eigen::VectorXd goal_vector(problem.q_goal.size());
//             for (size_t i = 0; i < problem.q_goal.size(); ++i)
//             {
//                 goal_vector(i) = (problem.q_goal[i].first + problem.q_goal[i].second) / 2.0; // Use the center of the goal bounds
//             }

//             double goal_dist = euclideanDistance(state_temp, goal_vector);
//             if (iteration_count % debug_interval == 0)
//             {
//                 // std::cout << "Distance to goal: " << goal_dist << std::endl;
//             }

//             if (goal_dist <= goal_tolerance)
//             {
//                 // std::cout << "Goal reached at iteration: " << iteration_count << std::endl;
//                 solutionNotFound = false;
//                 break;
//             }

//             if (iteration_count % debug_interval == 0)
//             {
//                 std::cout << "Iteration: " << iteration_count << std::endl;
//                 std::cout << "qrand: " << qrand.transpose() << std::endl;
//                 std::cout << "qnear: " << qnear.transpose() << std::endl;
//                 std::cout << "Distance to qrand: " << distance << std::endl;
//                 std::cout << "Distance to goal: " << goal_dist << std::endl;
//                 if (collision)
//                 {
//                     std::cout << "Collision detected at state: " << state_temp.transpose() << std::endl;
//                 }
//                 else
//                 {
//                     std::cout << "New state added: " << state_temp.transpose() << std::endl;
//                 }
//             }
//         }
//         else
//         {
//             if (iteration_count % debug_interval == 0)
//             {
//                 std::cout << "Collision detected. State not added to the tree." << std::endl;
//             }
//         }

//         iteration_count++;
//     }

//     if (solutionNotFound)
//     {
//         std::cout << "No solution found within iteration limit." << std::endl;
//     }
//     else
//     {
//         std::cout << "Solution found" << std::endl;
//         nodes[n] = centers;

//         std::priority_queue<Node> openList;
//         std::unordered_map<int, Node> openSet;
//         std::unordered_map<int, Node> closedSet;

//         Node start;
//         start.id = 0;
//         start.g = 0;
//         start.h = euclideanDistance(nodes[start.id], nodes[n]);
//         openList.push(start);
//         openSet[start.id] = start;

//         bool goalFound = false;
//         int counter = 0;

//         while (!openList.empty())
//         {
//             Node nBest = openList.top();
//             openList.pop();

//             if (closedSet.find(nBest.id) != closedSet.end())
//             {
//                 continue;
//             }

//             if (openSet.find(nBest.id) != openSet.end() && nBest.g > openSet[nBest.id].g)
//             {
//                 continue;
//             }

//             openSet.erase(nBest.id);
//             closedSet[nBest.id] = nBest;

//             if (euclideanDistance(nodes[nBest.id], nodes[n]) <= goal_tolerance)
//             {
//                 closedSet[n] = nBest;
//                 goalFound = true;
//                 break;
//             }

//             auto children = graphPtr->children(nBest.id);
//             int i = 0;
//             for (const auto &child_id : children)
//             {
//                 double tentative_g = nBest.g + graphPtr->outgoingEdges(nBest.id)[i];
//                 i++;

//                 if (closedSet.find(child_id) != closedSet.end())
//                 {
//                     if (tentative_g >= closedSet[child_id].g)
//                         continue;
//                     else
//                         closedSet.erase(child_id);
//                 }

//                 if (openSet.find(child_id) == openSet.end() || tentative_g < openSet[child_id].g)
//                 {
//                     Node child;
//                     child.id = child_id;
//                     child.g = tentative_g;
//                     child.h = euclideanDistance(nodes[child.id], nodes[n]);
//                     child.parent = nBest.parent;
//                     child.parent.push_back(nBest.id);

//                     openSet[child_id] = child;
//                     openList.push(child);

//                     // std::cout << std::endl;
//                 }
//             }
//             counter++;
//         }

//         if (goalFound)
//         {

//             std::cout << closedSet[n].id << std::endl;

//             std::vector<int> pathl = closedSet[n].parent;

//             // pathl.push_back(n);

//             // std::cout << "Closed Set: ";

//             // std::cout << std::endl;

//             for (int node : pathl)
//             {
//                 // std::cout << node << " ";
//                 // std::cout << "pushing back" << nodes[node] << std::endl;
//                 path.waypoints.push_back(nodes[node]);
//                 path.controls.push_back(random_control);
//                 path.durations.push_back(1);
//             }
//         }
//         else
//         {
//             std::cout << "No path found" << std::endl;
//         }

//         path.waypoints.push_back(centers);
//     }
//     return path;
// }

// void MySimpleCar::propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt)
// {
//     RungeKutta4(state, control, dt);
// }

// void MySimpleCar::RungeKutta4(Eigen::VectorXd &state, const Eigen::VectorXd &control, double dt)
// {
//     Eigen::VectorXd newState(5), w1(5), w2(5), w3(5), w4(5);

//     double x = state(0);
//     double y = state(1);
//     // std::cout << x << " " << y << std::endl;

//     double theta = state(2);
//     double v = state(3);
//     double phi = state(4);
//     // std::cout << "Theta: " << theta << std::endl;

//     double u1 = control(0);
//     double u2 = control(1);

//     double der1 = v * cos(theta);
//     double der2 = v * sin(theta);

//     // double der3 = (v / carLength) * tan(phi);
//     double der3 = (v / 5) * tan(phi);

//     double der4 = u1;
//     double der5 = u2;

//     w1 << der1, der2, der3, der4, der5;

//     w2 << w1(0) + 0.5 * dt * w1(0),
//         w1(1) + 0.5 * dt * w1(1),
//         w1(2) + 0.5 * dt * w1(2),
//         w1(3) + 0.5 * dt * w1(3),
//         w1(4) + 0.5 * dt * w1(4);

//     w3 << w1(0) + 0.5 * dt * w2(0),
//         w1(1) + 0.5 * dt * w2(1),
//         w1(2) + 0.5 * dt * w2(2),
//         w1(3) + 0.5 * dt * w2(3),
//         w1(4) + 0.5 * dt * w2(4);

//     w4 << w1(0) + dt * w3(0),
//         w1(1) + dt * w3(1),
//         w1(2) + dt * w3(2),
//         w1(3) + dt * w3(3),
//         w1(4) + dt * w3(4);

//     newState = state + (dt / 6.0) * (w1 + 2 * w2 + 2 * w3 + w4);

//     // std::cout << newState(0) << " " << newState(1) << std::endl;

//     state = newState;

//     // std::cout << "Theta After: " << state(2) << std::endl;

//     // std::cout << "der3 calculation: " << (carLength) << std::endl;
// }
