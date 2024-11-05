#include "MyMultiAgentPlanners.h"

using Eigen::Vector2d;
/////////////////////////////

double highlevel;

double euclideanDistance(const Eigen::VectorXd &point1, const Eigen::VectorXd &point2)
{
    if (point1.size() != point2.size())
    {
        throw std::invalid_argument("Both points must have the same number of dimensions.");
    }

    return (point1 - point2).norm();
}
Eigen::VectorXd generateRandomNVectorCentral(const Eigen::VectorXd &lower_bounds, const Eigen::VectorXd &upper_bounds)
{
    if (lower_bounds.size() != upper_bounds.size())
    {
        throw std::invalid_argument("Erororororororrr");
    }

    int n = lower_bounds.size();
    Eigen::VectorXd random_vector(n);
    std::random_device rd;
    std::mt19937 gen(rd());
    for (int i = 0; i < n; ++i)
    {
        std::uniform_real_distribution<float> dist(lower_bounds[i], upper_bounds[i]);
        random_vector[i] = dist(gen);
    }

    return random_vector;
}
struct VectorCompare {
    bool operator()(const Eigen::Vector2d &a, const Eigen::Vector2d &b) const {
        if (a[0] == b[0]) {
            return a[1] < b[1];
        }
        return a[0] < b[0];
    }
};


Eigen::Vector2d getCoord(const Eigen::VectorXd &state, int i)
{
    Eigen::Vector2d result;
    result[0] = state[i * 2];
    result[1] = state[(i * 2) + 1];
    // std::cout<< "robot " << i << " is at " << result[0] << ", " << result[1] << "\n";
    return result;
}

bool agentCollisionCentral(const Eigen::VectorXd &agent_pos, const std::vector<amp::CircularAgentProperties> &radii) {
    int num_agents = agent_pos.size() / 2;

    for (int i = 0; i < num_agents; i++) {
        Eigen::Vector2d pos_i = getCoord(agent_pos, i);

        for (int j = 0; j < num_agents; j++) {
            if (i == j) continue;

            Eigen::Vector2d pos_j = getCoord(agent_pos, j);
            double distance = euclideanDistance(pos_i, pos_j);
            double min_distance = radii[i].radius * 1.6 + radii[j].radius * 1.6;  // Reduced factor for collision radius

            if (distance < min_distance) {
                return true; // Collision detected between agents i and j
            }
        }
    }
    return false;
}

// New function to check temporal collisions (swapping of positions)
bool checkTemporalCollision(const Eigen::VectorXd &prev_agent_pos, const Eigen::VectorXd &curr_agent_pos, const std::vector<amp::CircularAgentProperties> &radii) {
    int num_agents = prev_agent_pos.size() / 2;
    
    for (int i = 0; i < num_agents; i++) {
        Eigen::Vector2d prev_pos_i = getCoord(prev_agent_pos, i);
        Eigen::Vector2d curr_pos_i = getCoord(curr_agent_pos, i);

        for (int j = 0; j < num_agents; j++) {
            if (i == j) continue;

            Eigen::Vector2d prev_pos_j = getCoord(prev_agent_pos, j);
            Eigen::Vector2d curr_pos_j = getCoord(curr_agent_pos, j);

            // Check if agents i and j are swapping places or colliding
            if ((prev_pos_i == curr_pos_j && prev_pos_j == curr_pos_i) ||
                euclideanDistance(curr_pos_i, curr_pos_j) < (radii[i].radius * 1.6 + radii[j].radius * 1.6)) {
                return true; // Temporal or spatial collision detected
            }
        }
    }
    return false;
}

// New Reservation Table to prevent space-time overlap
std::map<Eigen::Vector2d, int, VectorCompare> reservationTable;

bool checkReservationTable(const Eigen::VectorXd &agent_pos, int timestep) {
    int num_agents = agent_pos.size() / 2;

    for (int i = 0; i < num_agents; i++) {
        Eigen::Vector2d pos = getCoord(agent_pos, i);
        if (reservationTable.find(pos) != reservationTable.end() && reservationTable[pos] == timestep) {
            return true; // Position is already reserved at this timestep
        }
    }
    return false;
}

void updateReservationTable(const Eigen::VectorXd &agent_pos, int timestep) {
    int num_agents = agent_pos.size() / 2;
    for (int i = 0; i < num_agents; i++) {
        Eigen::Vector2d pos = getCoord(agent_pos, i);
        reservationTable[pos] = timestep; // Reserve this position at this timestep
    }
}






























////////////////////////////////////////////// OLLDLDLDDLL DIVISIION

struct Node
{
    int id;
    float g = 0;                      // Cost from start to this node
    float h = 0;                      // Heuristic estimate of cost from this node to the goal
    float f() const { return g + h; } // Total estimated cost
    std::vector<int> parent;          // To reconstruct the path

    bool operator<(const Node &other) const
    {
        return f() > other.f();
    }
};

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
    if (q.x() <= std::max(p.x(), r.x()) + EPSILON && q.x() >= std::min(p.x(), r.x()) - EPSILON &&
        q.y() <= std::max(p.y(), r.y()) + EPSILON && q.y() >= std::min(p.y(), r.y()) - EPSILON)
        return true;
    return false;
}

bool doIntersect(Vector2d p1, Vector2d q1, Vector2d p2, Vector2d q2)
{
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

// New function to check if a line intersects with a circle
bool lineIntersectsCircle(Vector2d lineStart, Vector2d lineEnd, Vector2d circleCenter, double radius)
{
    Eigen::Vector2d d = lineEnd - lineStart;      // direction vector of the line
    Eigen::Vector2d f = lineStart - circleCenter; // vector from the circle's center to the line start

    double a = d.dot(d);
    double b = 2 * f.dot(d);
    double c = f.dot(f) - radius * radius;

    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0)
    {
        return false; // No intersection
    }
    else
    {
        discriminant = std::sqrt(discriminant);
        double t1 = (-b - discriminant) / (2 * a);
        double t2 = (-b + discriminant) / (2 * a);

        // Check if the intersection points are on the segment
        if (t1 >= 0 && t1 <= 1 || t2 >= 0 && t2 <= 1)
        {
            return true;
        }
    }
    return false;
}

// Updated lineIntersectsPolygon to account for agent's radius
bool lineIntersectsPolygon(Vector2d lineStart, Vector2d lineEnd, const std::vector<Vector2d> &polygon, const Vector2d &circleCenter, double radius)
{
    int n = polygon.size();

    for (int i = 0; i < n; ++i)
    {
        Vector2d polygonStart = polygon[i];
        Vector2d polygonEnd = polygon[(i + 1) % n];

        if (lineIntersectsCircle(polygonStart, polygonEnd, circleCenter, radius))
        {
            return true;
        }
    }
    return false;
}
//////////////////////////////
Eigen::VectorXd generateRandomNVector(const std::vector<double> &lower_bounds, const std::vector<double> &upper_bounds)
{
    if (lower_bounds.size() != upper_bounds.size())
    {
        throw std::invalid_argument("Erororororororrr");
    }

    int n = lower_bounds.size();
    Eigen::VectorXd random_vector(n);
    std::random_device rd;
    std::mt19937 gen(rd());
    for (int i = 0; i < n; ++i)
    {
        std::uniform_real_distribution<float> dist(lower_bounds[i], upper_bounds[i]);
        random_vector[i] = dist(gen);
    }

    return random_vector;
}

//////////////////////////////


////////////////////////////////

bool isPointInPolygon(const Eigen::Vector2d &point, const std::vector<Eigen::Vector2d> &vertices)
{
    int intersections = 0;
    int numVertices = vertices.size();

    for (int i = 0; i < numVertices; ++i)
    {
        const Eigen::Vector2d &v1 = vertices[i];
        const Eigen::Vector2d &v2 = vertices[(i + 1) % numVertices];

        if ((v1.y() > point.y()) != (v2.y() > point.y()))
        {
            double xIntersection = v1.x() + (v2.x() - v1.x()) * (point.y() - v1.y()) / (v2.y() - v1.y());
            if (point.x() < xIntersection)
            {
                intersections++;
            }
        }
    }

    return (intersections % 2) != 0;
}
bool agentCollision(const Eigen::Vector2d &agent_pos_old, const Eigen::Vector2d &agent_pos, const std::vector<amp::Path2D> &agent_paths,
                    double current_time, double time_step, double agent_radius)
{
    double path_id = current_time / time_step;
    int current_index = static_cast<int>(path_id);
    double interpolation_step = 0.0001;
    for (const auto &path : agent_paths)
    {
        if (current_index >= path.waypoints.size() - 1)
        {
            continue;
        }

        Eigen::Vector2d start_pos = path.waypoints[current_index - 1];
        Eigen::Vector2d end_pos = path.waypoints[current_index];

        for (double t = 0.0; t < 1.0; t += interpolation_step)
        {
            Eigen::Vector2d interpolated_pos = start_pos + t * (end_pos - start_pos);
            Eigen::Vector2d interpolated_pos2 = agent_pos + t * (agent_pos_old - agent_pos);

            if (euclideanDistance(interpolated_pos2, interpolated_pos) < agent_radius)
            {
                return true;
            }
        }

        if (euclideanDistance(agent_pos, end_pos) < agent_radius)
        {
            return true;
        }
    }

    return false;
}

//////////////////////////////////////////////
amp::Path2D goalBiasRRT(const amp::CircularAgentProperties &agent, const amp::MultiAgentProblem2D &problem, double &time_size, std::vector<amp::Path2D> &agent_paths)
{
    double total_time = 0.0;
    amp::Path2D agent_path;

    double radius = 2.5;
    int n = 10000;
    double goal_tolerance = 0.5;

    double step_size = time_size;
    double goal_probability = 0.05;
    double distance;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    int counter = 0;

    double qnear_counter;
    Eigen::Vector2d qrand, qnear;
    std::vector<double> lower_bounds = {problem.x_min, problem.y_min};
    std::vector<double> upper_bounds = {problem.x_max, problem.y_max};
    std::vector<Eigen::Vector2d> points;
    std::map<amp::Node, Eigen::Vector2d> nodes;

    std::shared_ptr<amp::Graph<double>> graphPtr;
    graphPtr = std::make_shared<amp::Graph<double>>();

    agent_path.waypoints.push_back(agent.q_init);

    points.push_back(agent.q_init.head<2>());
    nodes[counter] = agent.q_init.head<2>();
    counter++;

    bool solutionNotFound = true;
    int max_iterations = n; // set a limit
    int iteration_count = 0;

    while (solutionNotFound && iteration_count < max_iterations)
    {
        distance = INFINITY;

        double random_value = dis(gen);

        if (random_value < goal_probability)
        {
            qrand = agent.q_goal;
        }
        else
        {
            qrand = generateRandomNVector(lower_bounds, upper_bounds);
        }

        for (int i = 0; i < points.size(); i++)
        {
            if (euclideanDistance(points[i], qrand) <= distance)
            {
                distance = euclideanDistance(points[i], qrand);
                qnear = points[i];
                qnear_counter = i;
            }
        }

        bool collision = false;
        Eigen::Vector2d qnew = qnear + (qrand - qnear).normalized() * step_size;

        if (euclideanDistance(qnear, qnew) <= radius)
        {
            for (const auto &obstacle : problem.obstacles)
            {
                if (lineIntersectsPolygon(qnear, qnew, obstacle.verticesCCW(), qnew, agent.radius * 1.8) ||
                    isPointInPolygon(Vector2d(qnew[0], qnew[1]), obstacle.verticesCCW()))
                {
                    collision = true;
                    break;
                }
            }

            // Check collision with other agents' paths
            for (const auto &path : agent_paths)
            {
                int index = total_time / step_size;

                for (int i = std::max(0, index - 5); i < std::min(index + 5, (int)path.waypoints.size() - 1); ++i)
                {
                    double interpolation_step = 0.001;

                    Eigen::Vector2d start = path.waypoints[i];
                    Eigen::Vector2d end = path.waypoints[i + 1];

                    Eigen::Vector2d interpolated_pos, interpolated_pos2;
                    for (double t = 0.0; t < 1.0; t += interpolation_step)
                    {
                        interpolated_pos = start + t * (end - start);
                        interpolated_pos2 = qnear + t * (qnew - qnear);
                        if (euclideanDistance(interpolated_pos, interpolated_pos2) < agent.radius * 3)
                        {
                            collision = true;
                            break;
                        }
                    }
                }
                if (collision)
                {
                    break;
                }
            }

            // Temporal collision check
            if (agent_paths.size() > 1) {
                for (int i = 0; i < agent_paths.size() - 1; ++i) {
                    if (checkTemporalCollision(agent_paths[i].waypoints.back(), qnew, problem.agent_properties)) {
                        collision = true;
                        break;
                    }
                }
            }

            // Reservation table check
            if (checkReservationTable(qnew, iteration_count)) {
                collision = true;
            }

            if (!collision)
            {
                points.push_back(qnew);
                nodes[counter] = qnew;

                graphPtr->connect(qnear_counter, counter, distance);
                total_time += step_size;
                counter++;

                updateReservationTable(qnew, iteration_count); // Update reservation table

                if (euclideanDistance(qnew, agent.q_goal) < goal_tolerance)
                {
                    solutionNotFound = false;
                    break;
                }
            }
        }

        iteration_count++;
    }

    if (solutionNotFound)
    {
        std::cout << "No solution found within iteration limit." << std::endl;
        agent_path.waypoints.push_back(agent.q_goal);
    }
    
    else
    {
        std::cout << "Solution found" << std::endl;

        nodes[n] = agent.q_goal.head<2>();

        std::priority_queue<Node> openList;
        std::unordered_map<int, Node> openSet;
        std::unordered_map<int, Node> closedSet;

        Node start;
        start.id = 0;
        start.g = 0;
        start.h = euclideanDistance(nodes[start.id], nodes[n]);
        openList.push(start);
        openSet[start.id] = start;

        bool goalFound = false;
        int counter = 0;

        while (!openList.empty())
        {
            Node nBest = openList.top();
            openList.pop();

            if (closedSet.find(nBest.id) != closedSet.end())
            {
                continue;
            }

            if (openSet.find(nBest.id) != openSet.end() && nBest.g > openSet[nBest.id].g)
            {
                continue;
            }

            openSet.erase(nBest.id);
            closedSet[nBest.id] = nBest;

            if (euclideanDistance(nodes[nBest.id], nodes[n]) <= goal_tolerance)
            {
                closedSet[n] = nBest;
                goalFound = true;
                break;
            }

            auto children = graphPtr->children(nBest.id);
            int i = 0;
            for (const auto &child_id : children)
            {
                double tentative_g = nBest.g + graphPtr->outgoingEdges(nBest.id)[i];
                i++;

                if (closedSet.find(child_id) != closedSet.end())
                {
                    if (tentative_g >= closedSet[child_id].g)
                        continue;
                    else
                        closedSet.erase(child_id);
                }

                if (openSet.find(child_id) == openSet.end() || tentative_g < openSet[child_id].g)
                {
                    Node child;
                    child.id = child_id;
                    child.g = tentative_g;
                    child.h = euclideanDistance(nodes[child.id], nodes[n]);
                    child.parent = nBest.parent;
                    child.parent.push_back(nBest.id);

                    openSet[child_id] = child;
                    openList.push(child);

                    // std::cout << std::endl;
                }
            }
            counter++;
        }

        if (goalFound)
        {

            //std::cout << closedSet[n].id << std::endl;

            std::vector<int> pathl = closedSet[n].parent;

            // pathl.push_back(n);

            // std::cout << "Closed Set: ";

            // std::cout << std::endl;

            for (int node : pathl)
            {
                // std::cout << node << " ";
                // std::cout << "pushing back" << nodes[node] << std::endl;
                agent_path.waypoints.push_back(nodes[node]);
            }
            // std::cout << std::endl;
        }
        else
        {
            std::cout << "No path found" << std::endl;
        }

        agent_path.waypoints.push_back(agent.q_goal);
    }


    return agent_path;
}





amp::Path centralRRT(const amp::MultiAgentProblem2D &problem, double &time_size, Eigen::VectorXd &q_init, Eigen::VectorXd &q_goal, Eigen::VectorXd &lower_bounds, Eigen::VectorXd &upper_bounds) {

    double total_time = 0.0;
    amp::Path path;
    double radius = 3; //3
    int n = 500000; //500000
    double goal_tolerance = 1;

    double step_size = time_size;
    double goal_probability = 0.05;
    double distance;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    int counter = 0;

    double qnear_counter;
    Eigen::VectorXd qrand, qnear;

    std::vector<Eigen::VectorXd> points;
    std::map<amp::Node, Eigen::VectorXd> nodes;

    std::shared_ptr<amp::Graph<double>> graphPtr;
    graphPtr = std::make_shared<amp::Graph<double>>();

    path.waypoints.push_back(q_init);

    points.push_back(q_init);
    nodes[counter] = q_init;
    counter++;

    bool solutionNotFound = true;
    int max_iterations = n; // set a limit
    int iteration_count = 0;
    
    Eigen::VectorXd prev_qnear = q_init; // Track previous position for temporal collisions

    while (solutionNotFound && iteration_count < max_iterations) {
        distance = INFINITY;

        double random_value = dis(gen);

        if (random_value < goal_probability) {
            qrand = q_goal;
        } else {
            qrand = generateRandomNVectorCentral(lower_bounds, upper_bounds);
        }

        for (int i = 0; i < points.size(); i++) {
            if (euclideanDistance(points[i], qrand) <= distance) {
                distance = euclideanDistance(points[i], qrand);
                qnear = points[i];
                qnear_counter = i;
            }
        }

        bool collision = false;
        Eigen::VectorXd qnew = qnear + (qrand - qnear).normalized() * step_size;

        if (euclideanDistance(qnear, qnew) <= radius) {
            for (int i = 0; i < problem.numAgents(); i++) {
                Eigen::Vector2d lol = getCoord(qnear, i);
                Eigen::Vector2d lolz = getCoord(qnew, i);

                for (const auto &obstacle : problem.obstacles) {
                    if (lineIntersectsPolygon(lol, lolz, obstacle.verticesCCW(), lolz, problem.agent_properties[i].radius * 2) ||
                        isPointInPolygon(Vector2d(lolz[0], lolz[1]), obstacle.verticesCCW()) || 
                        agentCollisionCentral(qnew, problem.agent_properties) ||
                        checkTemporalCollision(prev_qnear, qnew, problem.agent_properties) || // Check temporal collision
                        checkReservationTable(qnew, iteration_count)) // Check reservation table for space-time conflicts
                    {
                        collision = true;
                        break;
                    }
                }
            }

            if (!collision) {
                points.push_back(qnew);
                nodes[counter] = qnew;

                graphPtr->connect(qnear_counter, counter, distance);
                total_time += step_size;
                counter++;

                updateReservationTable(qnew, iteration_count); // Update reservation table

                if (euclideanDistance(qnew, q_goal) < goal_tolerance) {
                    solutionNotFound = false;
                    break;
                }
            }
        }

        prev_qnear = qnear; // Update previous position
        iteration_count++;
            std::cout << "Iteration: " << iteration_count << std::endl;
        //print every 100000 iteration



    }

    if (solutionNotFound)
    {
        std::cout << "No solution found within iteration limit." << std::endl;

    }
    else
    {
        std::cout << "Solution found" << std::endl;

        nodes[n] = q_goal;

        std::priority_queue<Node> openList;
        std::unordered_map<int, Node> openSet;
        std::unordered_map<int, Node> closedSet;

        Node start;
        start.id = 0;
        start.g = 0;
        start.h = euclideanDistance(nodes[start.id], nodes[n]);
        openList.push(start);
        openSet[start.id] = start;

        bool goalFound = false;
        int counter = 0;

        while (!openList.empty())
        {
            Node nBest = openList.top();
            openList.pop();

            if (closedSet.find(nBest.id) != closedSet.end())
            {
                continue;
            }

            if (openSet.find(nBest.id) != openSet.end() && nBest.g > openSet[nBest.id].g)
            {
                continue;
            }

            openSet.erase(nBest.id);
            closedSet[nBest.id] = nBest;

            if (euclideanDistance(nodes[nBest.id], nodes[n]) <= goal_tolerance)
            {
                closedSet[n] = nBest;
                goalFound = true;
                break;
            }

            auto children = graphPtr->children(nBest.id);
            int i = 0;
            for (const auto &child_id : children)
            {
                double tentative_g = nBest.g + graphPtr->outgoingEdges(nBest.id)[i];
                i++;

                if (closedSet.find(child_id) != closedSet.end())
                {
                    if (tentative_g >= closedSet[child_id].g)
                        continue;
                    else
                        closedSet.erase(child_id);
                }

                if (openSet.find(child_id) == openSet.end() || tentative_g < openSet[child_id].g)
                {
                    Node child;
                    child.id = child_id;
                    child.g = tentative_g;
                    child.h = euclideanDistance(nodes[child.id], nodes[n]);
                    child.parent = nBest.parent;
                    child.parent.push_back(nBest.id);

                    openSet[child_id] = child;
                    openList.push(child);

                    // std::cout << std::endl;
                }
            }
            counter++;
        }

        if (goalFound)
        {

            ///std::cout << closedSet[n].id << std::endl;

            std::vector<int> pathl = closedSet[n].parent;

            // pathl.push_back(n);

            // std::cout << "Closed Set: ";

            // std::cout << std::endl;

            for (int node : pathl)
            {
                // std::cout << node << " ";
                // std::cout << "pushing back" << nodes[node] << std::endl;
                path.waypoints.push_back(nodes[node]);
            }
            // std::cout << std::endl;
        }
        else
        {
            std::cout << "No path found" << std::endl;
        }
    }

    
    highlevel = graphPtr->nodes().size();
    return path;
}

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D &problem)
{
    amp::MultiAgentPath2D path;
    std::vector<amp::Path2D> agent_paths;
    int counter = 0;

    for (const amp::CircularAgentProperties &agent : problem.agent_properties)
    {
        //std::cout << "Running agent: " << counter << std::endl;
        counter++;
        double step_size = 0.3;
        amp::Path2D agent_path = goalBiasRRT(agent, problem, step_size, agent_paths);
        agent_paths.push_back(agent_path);

        path.agent_paths.push_back(agent_path);
    }
    return path;
}



 amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D &problem)
{
    amp::MultiAgentPath2D path;
    int numAgens = problem.numAgents();

    Eigen::VectorXd all_q_init;
    Eigen::VectorXd all_q_goal;
    for (const amp::CircularAgentProperties &agent : problem.agent_properties)
    {
        all_q_init.conservativeResize(all_q_init.size() + 2);
        all_q_init.segment(all_q_init.size() - 2, 2) = agent.q_init;
        all_q_goal.conservativeResize(all_q_goal.size() + 2);
        all_q_goal.segment(all_q_goal.size() - 2, 2) = agent.q_goal;
    }

    std::vector<double> all_radii;
    for (const amp::CircularAgentProperties &agent : problem.agent_properties)
    {
        all_radii.push_back(agent.radius);
    }

    // implement goal rrt

    Eigen::VectorXd lower(2 * numAgens);
    for (int i = 0; i < numAgens; ++i)
    {
        lower[2 * i] = problem.x_min;
        lower[2 * i + 1] = problem.y_min;
    }
    Eigen::VectorXd upper(2 * numAgens);
    for (int i = 0; i < numAgens; ++i)
    {
        upper[2 * i] = problem.x_max;
        upper[2 * i + 1] = problem.y_max;
    }

    // for (const amp::CircularAgentProperties &agent : problem.agent_properties)
    // {
    //     amp::Path2D agent_path;
    //     counter++;
    //     std::cout << "Running agent: " << counter << " with radius: " << agent.radius << std::endl;
    //     agent_path.waypoints = {agent.q_init, agent.q_goal};
    //     path.agent_paths.push_back(agent_path);
    // }

    double step_size = 1;

    amp::Path path_nd = centralRRT(problem, step_size, all_q_init, all_q_goal, lower, upper);
    //std::cout << "Path length: " << path_nd.waypoints.size() <<" "<<path_nd.waypoints[0].size() << std::endl;
    int counter = 0;
    for (const amp::CircularAgentProperties &agent : problem.agent_properties)
    {
           

        amp::Path2D agent_path;
        
        for (int i = 0; i < path_nd.waypoints.size(); i++){

            agent_path.waypoints.push_back({path_nd.waypoints[i][counter], path_nd.waypoints[i][counter + 1]});

        }
        counter += 2;
    //print first and last point of agent path
    //std::cout << agent_path.waypoints[0] << " " << agent_path.waypoints[agent_path.waypoints.size() - 1] << std::endl;
    agent_path.waypoints.push_back(agent.q_goal);
    path.agent_paths.push_back(agent_path);

    }

    pathLength = highlevel;

    return path;
}
