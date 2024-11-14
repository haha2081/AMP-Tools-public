// #include "MyKinoRRT.h"

// double carLength;

// // Helper function to scale a vector by a scalar value
//             std::vector<double> scaleVector(const std::vector<double> &vec, double scalar)
//             {
//                 std::vector<double> scaled_vec(vec.size());
//                 std::transform(vec.begin(), vec.end(), scaled_vec.begin(),
//                                [scalar](double val)
//                                { return val * scalar; });
//                 return scaled_vec;
//             }

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

// // Helper function to check if a state is within the goal boundaries
// bool isWithinGoalBoundaries(const Eigen::VectorXd &state, const std::vector<std::pair<double, double>> &goal_boundaries)
// {
//     for (int i = 0; i < state.size(); ++i)
//     {
//         if (state[i] < goal_boundaries[i].first || state[i] > goal_boundaries[i].second)
//         {
//             return false;
//         }
//     }
//     return true;
// }

// Eigen::VectorXd getCtrl(amp::Node current, amp::Node came_from, std::shared_ptr<amp::Graph<std::pair<Eigen::VectorXd, double>>> graph)
// {
//     const auto &children = graph->children(came_from);
//     const auto &edges = graph->outgoingEdges(came_from);

//     int child_index = -1;
//     for (int i = 0; i < children.size(); ++i)
//     {
//         if (children[i] == current)
//         {
//             child_index = i;
//             break;
//         }
//     }

//     if (child_index != -1 && child_index < edges.size())
//     {
//         const auto &edge = edges[child_index].first;
//         return edge;
//         // Further code to use 'edge' as needed
//     }
//     // << "no edge from " << came_from << " to " << current << "\n";
//     return Eigen::VectorXd::Zero(1);
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

// // Create a tree rooted at x0

// Eigen::VectorXd StateSample(std::vector<double> lower_bounds, std::vector<double> upper_bounds, Eigen::VectorXd centers)
// {

//     Eigen::VectorXd qrand;

//     double goal_probability = 0.05;
//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_real_distribution<> dis(0.0, 1.0);
//     double random_value = dis(gen);

//     if (random_value < goal_probability)
//     {
//         qrand = centers;
//     }
//     else
//     {
//         qrand = generateRandomNVector(lower_bounds, upper_bounds);
//     }

//     return qrand;
// }

// bool isPointInsidePolygon(const Eigen::Vector2d &point, const amp::Polygon &polygon)
// {
//     int intersections = 0;
//     size_t n = polygon.verticesCCW().size();
//     for (size_t i = 0; i < n; ++i)
//     {
//         Eigen::Vector2d v1(polygon.verticesCCW()[i].x(), polygon.verticesCCW()[i].y());
//         Eigen::Vector2d v2(polygon.verticesCCW()[(i + 1) % n].x(), polygon.verticesCCW()[(i + 1) % n].y());

//         if ((point.y() > v1.y() != point.y() > v2.y()) &&
//             (point.x() < (v2.x() - v1.x()) * (point.y() - v1.y()) / (v2.y() - v1.y()) + v1.x()))
//         {
//             intersections++;
//         }
//     }
//     return (intersections % 2 != 0);
// }

// bool checkCollision(const Eigen::Vector2d &position, double L, double W, const std::vector<amp::Polygon> &obstacles, double padding = 0.35)
// {
//     // Expand the rectangle vertices by the padding
//     std::vector<Eigen::Vector2d> rectangleVertices;
//     rectangleVertices.emplace_back(position.x() - padding, position.y() - W / 2 - padding);
//     rectangleVertices.emplace_back(position.x() + L + padding, position.y() - W / 2 - padding);
//     rectangleVertices.emplace_back(position.x() + L + padding, position.y() + W / 2 + padding);
//     rectangleVertices.emplace_back(position.x() - padding, position.y() + W / 2 + padding);

//     bool collision = false;

//     for (const auto &obstacle : obstacles)
//     {
//         // Check if any of the rectangle's vertices are inside the polygon
//         for (const auto &vertex : rectangleVertices)
//         {
//             if (isPointInsidePolygon(vertex, obstacle))
//             {
//                 collision = true;
//                 break;
//             }
//         }

//         if (collision)
//             break;

//         for (size_t i = 0; i < obstacle.verticesCCW().size(); ++i)
//         {
//             Eigen::Vector2d obstacleVertex(obstacle.verticesCCW()[i].x(), obstacle.verticesCCW()[i].y());
//             if (isPointInsidePolygon(obstacleVertex, rectangleVertices))
//             {
//                 collision = true;
//                 break;
//             }
//         }

//         if (collision)
//             break;
//     }

//     return collision;
// }

// ////////////////////////////////////////// ---------------------------- CODE STARTS HERE ---------------------------- //////////////////////////

// amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D &problem, amp::DynamicAgent &agent)
// {
//     amp::KinoPath path;
//     carLength = problem.agent_dim.length;
//     Eigen::VectorXd state = problem.q_init;

//     std::shared_ptr<amp::Graph<std::pair<Eigen::VectorXd, double>>> graphPtr;
//     graphPtr = std::make_shared<amp::Graph<std::pair<Eigen::VectorXd, double>>>();

//     std::vector<Eigen::VectorXd> points;
//     std::map<amp::Node, Eigen::VectorXd> nodes;
//     std::map<amp::Node, Eigen::VectorXd> controls;

//     int counter = 0;
//     bool solutionNotFound = true;
//     double pathLengthAccumulator = 0.0; // To accumulate path length

//     int iteration_count = 0;

//     // Variables
//     int control_sample_number = 150;
//     double delta_t = 0.01;
//     double step_size = 0.05;
//     double max_iterations = 1000000;
//     int num_segments = 100;

//        // Ensure n and u_samples are set before planning
  
  
//     // Configure the planner using n and u_samples
  
//     // int control_sample_number = this->getUSamples();
//     // double max_iterations = this->getNSamples();


//     Eigen::VectorXd qnear;
//     int qnear_counter;

//     /////////// Initialize lower and upper bounds for the state space and control space
//     // State bounds
//     std::vector<double> lower_bounds(problem.q_bounds.size());
//     std::vector<double> upper_bounds(problem.q_bounds.size());

//     for (int i = 0; i < problem.q_bounds.size(); ++i)
//     {
//         lower_bounds[i] = problem.q_bounds[i].first;
//         upper_bounds[i] = problem.q_bounds[i].second;
//     }

//     // Control bounds
//     std::vector<double> u_lower_bounds;
//     std::vector<double> u_upper_bounds;

//     for (const auto &bound : problem.u_bounds)
//     {
//         u_lower_bounds.push_back(bound.first);
//         u_upper_bounds.push_back(bound.second);
//     }

//     /////////////////////////////////////////////////////////////////
//     /////////// Calculate the center of the goal region

//     Eigen::VectorXd centers(problem.q_goal.size());
//     for (int i = 0; i < problem.q_goal.size(); ++i)
//     {
//         double center = (problem.q_goal[i].first + problem.q_goal[i].second) / 2.0;
//         centers[i] = center;
//     }

//     /////////////////////////////////////////////////////////////////
//     /////////// Create a tree rooted at x0

//     Eigen::VectorXd start_control(2);
//     start_control << state(3), state(4);

//     path.waypoints.push_back(state);
//     path.controls.push_back(start_control);
//     path.durations.push_back(delta_t);

//     points.push_back(state);
//     nodes[counter] = state;

//     controls[counter] = start_control;

//     counter++;
//     /////////////////////////////////////////////////////////////////
//     /////////// Main loop start

//     while (solutionNotFound && iteration_count < max_iterations)
//     {

//         if (iteration_count % 10000 == 0)
//         {
//             std::cout << "Iteration: " << iteration_count << std::endl;
//         }
//         /////////////////////////////////////////////////////////////////
//         /////////// Random Sample according to goal-bias RRT

//         Eigen::VectorXd qrand = StateSample(lower_bounds, upper_bounds, centers);

//         /////////////////////////////////////////////////////////////////
//         /////////// Find the nearest state in 𝑇 to qrand according to distance p

//         double distance = std::numeric_limits<double>::infinity();

//         for (int i = 0; i < points.size(); ++i)
//         {
//             // std::cout << "i: " << i <<" qrand: " << qrand.transpose() << " point: " << points[i].transpose() << std::endl;

//             double temp_dist1 = euclideanDistance(qrand, points[i]);
//             if (temp_dist1 < distance)
//             {
//                 distance = temp_dist1;
//                 qnear = points[i];
//                 qnear_counter = i;
//             }
//         }

//         // std::cout << "The nearest node out of " << points.size() << " is: " << qnear.transpose() << std::endl;

//         /////////////////////////////////////////////////////////////////
//         /////////// Generate a local trajectory between qnear and qrand by sampling many random controls

//         Eigen::VectorXd best_state;
//         Eigen::VectorXd best_control;
//         double distance2 = std::numeric_limits<double>::infinity();

//         for (int i = 0; i < control_sample_number; i++)
//         {

//             Eigen::VectorXd potential_state;
//             Eigen::VectorXd temp_state = qnear;

//             Eigen::VectorXd random_control = generateRandomNVector(u_lower_bounds, u_upper_bounds);

//             // std::cout << "random_control: " << random_control.transpose() << std::endl;

//             agent.propagate(temp_state, random_control, delta_t);

//             double temp_dist2 = euclideanDistance(temp_state, qrand);

//             // std::cout << "temp_dist2: " << temp_dist2 << std::endl;

//             if (temp_dist2 < distance2)
//             {
//                 distance2 = temp_dist2;
//                 best_state = temp_state;
//                 best_control = random_control;
//             }
//         }

//         // std::cout << "best_control: " << best_control.transpose() << " at distance: " << distance2<<std::endl;

//         /////////////////////////////////////////////////////////////////
//         /////////// Check if the local trajectory is valid and add it to the tree

//         bool collision = false;

//         Eigen::VectorXd qnew = best_state;
//         Eigen::Vector2d start = qnew.head<2>();

//         for (int i = 0; i <= num_segments; ++i)
//         {
//             double t = static_cast<double>(i) / num_segments;
//             Eigen::VectorXd intermediate_state = qnear + t * (qnew - qnear);
//             Eigen::Vector2d position = intermediate_state.head<2>();

//             if (checkCollision(position, problem.agent_dim.length, problem.agent_dim.width, problem.obstacles))
//             {
//                 collision = true;
//                 break;
//             }
//         }

//         if (!collision)
//         {
//             points.push_back(qnew);
//             nodes[counter] = qnew;
//             graphPtr->connect(qnear_counter, counter, std::make_pair(best_control, distance2));
//             pathLengthAccumulator += distance2; // Accumulate the path segment length

//             counter++;

//             if (isWithinGoalBoundaries(qnew, problem.q_goal))
//             {
//                 solutionNotFound = false;
//                 break;
//             }
//         }
//         iteration_count++;
//     }

//     /////////////////////////////////////////////////////////////////

//     if (solutionNotFound)
//     {
//        std::cout << "No solution found within iteration limit." << std::endl;
//         path.valid = false;
//         pathLength = 0.0; // Reset path length if no solution is found
        

//     }
//     else
//     {
//         std::cout << "Solution found at iteration: " << iteration_count << std::endl;
//         path.valid = true;
//         pathLength = pathLengthAccumulator; // Set the calculated path length

//     }

//     if (!solutionNotFound)
//     {

//        // std::cout << "before" << std::endl;
//         nodes[iteration_count] = centers;

//         std::priority_queue<Node> openList;
//         std::unordered_map<int, Node> openSet;
//         std::unordered_map<int, Node> closedSet;

//         Node start;
//         start.id = 0;
//         start.g = 0;
//         start.h = euclideanDistance(nodes[start.id], nodes[iteration_count]);
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

//             if (isWithinGoalBoundaries(nodes[nBest.id], problem.q_goal))
//             {
//                 closedSet[iteration_count] = nBest;
//                 goalFound = true;
//                 break;
//             }

//             auto children = graphPtr->children(nBest.id);
//             int i = 0;
//             for (const auto &child_id : children)
//             {
//                 double tentative_g = nBest.g + graphPtr->outgoingEdges(nBest.id)[i].second;
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
//                     child.h = euclideanDistance(nodes[child.id], nodes[iteration_count]);
//                     child.parent = nBest.parent;
//                     child.parent.push_back(nBest.id);

//                     openSet[child_id] = child;
//                     openList.push(child);

//                     // std::cout << std::endl;
//                 }
//             }
//             counter++;
//         }

//         std::vector<Eigen::VectorXd> way;
//         std::vector<Eigen::VectorXd> con;
//         std::vector<double> dur;

//         if (goalFound)
//         {
//             std::cout << "Goal found" << std::endl;

//             std::vector<int> pathl = closedSet[iteration_count].parent;

//             int temp = 0;

//             auto kes = problem.q_init;

//             for (int node : pathl)
//             {
//                // std::cout << "waypoints: " << nodes[node].transpose() << std::endl;
//                 path.waypoints.push_back(nodes[node]);

//                 if (temp != 0)
//                 {

//                     path.controls.push_back(getCtrl(node, temp, graphPtr));

//                     Eigen::VectorXd de = getCtrl(node, temp, graphPtr);
//                     agent.propagate(kes, de, delta_t);

//                     //std::cout << "control: " << kes.transpose() << " " << getCtrl(node, temp, graphPtr).transpose() << std::endl;

//                     path.durations.push_back(delta_t);
//                 }

//                 // reverse path.controls

//                 temp = node;
//             }

//             auto tempo_afro_beat = kes;
//             int itr = 0;
//             Eigen::VectorXd min_control;
//             double min_dist = std::numeric_limits<double>::infinity();
//             int max_iterations = 10000000; // Set a max iteration limit

//             double perturbation_strength = 1.5; // Strength of control perturbations

            

//             while (itr < max_iterations)
//             {
//                 itr++;

//                 if (itr % 1000000 == 0)
//                 {
//                   //std::cout << "Iteration: " << itr << std::endl;
//                    // std::cout << "Current Min Distance: " << min_dist << std::endl;
//                 }

//                 // Dynamically adjust the range for sampling controls as iterations progress
               
//                 Eigen::VectorXd sample_control = generateRandomNVector(u_lower_bounds, u_upper_bounds);

//                 auto current_state = tempo_afro_beat;

//                 agent.propagate(current_state, sample_control, 0.1);

//                 double dist = euclideanDistance(current_state, centers);

//                 if (dist < min_dist)
//                 {
//                     min_dist = dist;
//                     kes = current_state;          // Update `kes` to the state with the smallest distance
//                     min_control = sample_control; // Store the control that achieves this state
//                 }

//                 // Check for convergence
//                 if (min_dist < 0.1)
//                 {
//                     path.waypoints.push_back(kes);
//                     path.controls.push_back(min_control);
//                     path.durations.push_back(0.1);
//                     //std::cout << "Converged after " << itr << " iterations." << std::endl;
//                     break;
//                 }
//             }

//             if (itr >= max_iterations)
//             {
//                 //std::cout << "Reached max iterations without convergence." << std::endl;
//                  path.waypoints.push_back(kes);
//                 path.controls.push_back(min_control);
//                 path.durations.push_back(0.1);
//             }

//             // std::reverse(path.controls.begin(), path.controls.end());

//             // std::cout << "path.waypoints: " << path.waypoints.size() << "controls: " << path.controls.size() << "durations: " << path.durations.size() << std::endl;

//             path.valid = true;
//         }
//         else
//         {
//             //std::cout << "No path found" << std::endl;
//         }
//     }

//     for (size_t i = 0; i < problem.q_goal.size(); ++i)
//     {
//         //std::cout << "Goal boundary is " << i << ": " << problem.q_goal[i].first << " " << problem.q_goal[i].second << " ";
//     }
//     std::cout << std::endl;

//     return path;
// }



// void MySingleIntegrator::propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt)
// {
//     RungeKutta4(state, control, dt);
// }

// void MyFirstOrderUnicycle::propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt)
// {
//     RungeKutta4(state, control, dt);
// }

// void MySecondOrderUnicycle::propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt)
// {
//     RungeKutta4(state, control, dt);
// }

// void MySimpleCar::propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt)
// {
//     RungeKutta4(state, control, dt);
// }

// ///////////////
// void MySingleIntegrator::RungeKutta4(Eigen::VectorXd &state, const Eigen::VectorXd &control, double dt)
// {
//     // Assuming the state is represented as [x, y] and the control is [u1, u2]
//     Eigen::VectorXd w1(2), w2(2), w3(2), w4(2);

//     double u1 = control(0);
//     double u2 = control(1);

//     w1 << u1, u2;

//     Eigen::VectorXd state2 = state + 0.5 * dt * w1;
//     w2 << u1, u2;

//     Eigen::VectorXd state3 = state + 0.5 * dt * w2;
//     w3 << u1, u2;

//     Eigen::VectorXd state4 = state + dt * w3;
//     w4 << u1, u2;

//     state = state + (dt / 6.0) * (w1 + 2 * w2 + 2 * w3 + w4);
// }

// //////////////////////////

// void MyFirstOrderUnicycle::RungeKutta4(Eigen::VectorXd &state, const Eigen::VectorXd &control, double dt)
// {
//     // Initialize the intermediate vectors
//     Eigen::VectorXd w1(3), w2(3), w3(3), w4(3);

//     // Extract state variables
//     double x = state(0);
//     double y = state(1);
//     double theta = state(2);

//     // Extract control inputs
//     double u_sigma = control(0); // linear velocity control input
//     double u_omega = control(1); // angular velocity control input

//     // Wheel radius
//     double r = 0.25;

//     // Compute w1 using the control inputs and current state
//     w1 << u_sigma * r * cos(theta),
//         u_sigma * r * sin(theta),
//         u_omega;

//     // Compute intermediate states for w2, w3, and w4
//     Eigen::VectorXd state2 = state + 0.5 * dt * w1;
//     w2 << u_sigma * r * cos(state2(2)),
//         u_sigma * r * sin(state2(2)),
//         u_omega;

//     Eigen::VectorXd state3 = state + 0.5 * dt * w2;
//     w3 << u_sigma * r * cos(state3(2)),
//         u_sigma * r * sin(state3(2)),
//         u_omega;

//     Eigen::VectorXd state4 = state + dt * w3;
//     w4 << u_sigma * r * cos(state4(2)),
//         u_sigma * r * sin(state4(2)),
//         u_omega;

//     // Update the state using the RK4 method
//     state = state + (dt / 6.0) * (w1 + 2 * w2 + 2 * w3 + w4);
// }

// /////////////////////////

// void MySecondOrderUnicycle::RungeKutta4(Eigen::VectorXd &state, const Eigen::VectorXd &control, double dt)
// {
//     Eigen::VectorXd w1(5), w2(5), w3(5), w4(5);

//     // Extract state variables
//     double x = state(0);
//     double y = state(1);
//     double theta = state(2);
//     double sigma = state(3);
//     double omega = state(4);

//     // Extract control inputs
//     double u1 = control(0); // control input affecting linear acceleration
//     double u2 = control(1); // control input affecting angular acceleration

//     // Wheel radius
//     double r = 0.25;

//     // Compute w1 using current state and control
//     w1 << sigma * r * cos(theta),
//         sigma * r * sin(theta),
//         omega,
//         u1,
//         u2;

//     // Compute intermediate state for w2
//     Eigen::VectorXd state2 = state + 0.5 * dt * w1;
//     w2 << state2(3) * r * cos(state2(2)),
//         state2(3) * r * sin(state2(2)),
//         state2(4),
//         u1,
//         u2;

//     // Compute intermediate state for w3
//     Eigen::VectorXd state3 = state + 0.5 * dt * w2;
//     w3 << state3(3) * r * cos(state3(2)),
//         state3(3) * r * sin(state3(2)),
//         state3(4),
//         u1,
//         u2;

//     // Compute final state for w4
//     Eigen::VectorXd state4 = state + dt * w3;
//     w4 << state4(3) * r * cos(state4(2)),
//         state4(3) * r * sin(state4(2)),
//         state4(4),
//         u1,
//         u2;

//     // Update the state using the RK4 integration method
//     state = state + (dt / 6.0) * (w1 + 2 * w2 + 2 * w3 + w4);
// }

// void MySimpleCar::RungeKutta4(Eigen::VectorXd &state, const Eigen::VectorXd &control, double dt)
// {
//     Eigen::VectorXd w1(5), w2(5), w3(5), w4(5);

//     double x = state(0);
//     double y = state(1);
//     double theta = state(2);
//     double v = state(3);
//     double phi = state(4);

//     double u1 = control(0);
//     double u2 = control(1);

//     w1 << v * cos(theta),
//         v * sin(theta),
//         (v / carLength) * tan(phi),
//         u1,
//         u2;

//     Eigen::VectorXd state2 = state + 0.5 * dt * w1;
//     w2 << state2(3) * cos(state2(2)),
//         state2(3) * sin(state2(2)),
//         (state2(3) / carLength) * tan(state2(4)),
//         u1,
//         u2;

//     Eigen::VectorXd state3 = state + 0.5 * dt * w2;
//     w3 << state3(3) * cos(state3(2)),
//         state3(3) * sin(state3(2)),
//         (state3(3) / carLength) * tan(state3(4)),
//         u1,
//         u2;

//     Eigen::VectorXd state4 = state + dt * w3;
//     w4 << state4(3) * cos(state4(2)),
//         state4(3) * sin(state4(2)),
//         (state4(3) / carLength) * tan(state4(4)),
//         u1,
//         u2;

//     state = state + (dt / 6.0) * (w1 + 2 * w2 + 2 * w3 + w4);
// }
