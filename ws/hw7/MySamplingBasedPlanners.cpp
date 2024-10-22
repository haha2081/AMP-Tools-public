#include "MySamplingBasedPlanners.h"
//////////////////////////////

using Eigen::Vector2d;
/////////////////////////////

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

bool lineIntersectsPolygon(Vector2d lineStart, Vector2d lineEnd, const std::vector<Vector2d> &polygon)
{
    int n = polygon.size();

    for (int i = 0; i < n; ++i)
    {
        Vector2d polygonStart = polygon[i];
        Vector2d polygonEnd = polygon[(i + 1) % n];

        if (doIntersect(lineStart, lineEnd, polygonStart, polygonEnd))
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

double euclideanDistance(const Eigen::VectorXd &point1, const Eigen::VectorXd &point2)
{
    if (point1.size() != point2.size())
    {
        throw std::invalid_argument("Both points must have the same number of dimensions.");
    }

    return (point1 - point2).norm();
}
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

//////////////////////////////
// Implement your PRM algorithm here
amp::Path2D MyPRM::plan(const amp::Problem2D &problem)
{
    double radius = connectRadius; // values that work 2.51
    int n = numSamples;       // values that work 2500
    bool smooth = smoothing;
    double goal_tolerance = 0.75; // values that work 0.3


    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    nodes[0] = problem.q_init.head<2>();

    std::vector<double> lower_bounds = {problem.x_min, problem.y_min};
    std::vector<double> upper_bounds = {problem.x_max, problem.y_max};
    std::vector<Eigen::Vector2d> points;

    points.push_back(problem.q_init.head<2>());
    for (int i = 1; i < n; i++)
    {
        Eigen::VectorXd random_vector = generateRandomNVector(lower_bounds, upper_bounds);
        points.push_back(random_vector.head<2>());
        nodes[i] = random_vector.head<2>();
    }

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            double distance = euclideanDistance(points[i], points[j]);

            if (distance < radius)
            {
                bool collision = false;

                for (const auto &obstacle : problem.obstacles)
                {
                    if (lineIntersectsPolygon(points[i], points[j], obstacle.verticesCCW()) || isPointInPolygon(Vector2d(points[j][0], points[j][1]), obstacle.verticesCCW()))
                    {
                        collision = true;
                        break;
                    }
                }
                if (!collision)
                {
                    graphPtr->connect(i, j, distance);
                }
            }
        }
    }

    nodes[n] = problem.q_goal.head<2>();

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

        std::cout << closedSet[n].id << std::endl;

        std::vector<int> pathl = closedSet[n].parent;

        // pathl.push_back(n);

       // std::cout << "Closed Set: ";

       
        // std::cout << std::endl;

        for (int node : pathl)
        {
            //std::cout << node << " ";
            //std::cout << "pushing back" << nodes[node] << std::endl;
            path.waypoints.push_back(nodes[node]);
        }
        //std::cout << std::endl;

        validSolutions = 1;
        pathLength = path.length();
    }
    else
    {
        validSolutions = 0;
        std::cout << "No path found" << std::endl;
    }

    path.waypoints.push_back(problem.q_goal);

    std::cout << "Number of iterations: " << counter << std::endl;
    std::cout << "Old Path Length: " << path.length() << std::endl;
    std::cout << "Path Smoothing set to: " << smooth << std::endl;



    //amp::Visualizer::makeFigure(problem, path, *graphPtr, nodes);
    
if (smooth && validSolutions)
{
    bool pathSmoothed = true;
    
    // Keep trying to smooth the path until no more smoothing is possible
    while (pathSmoothed)
    {
        pathSmoothed = false; // Reset flag to track if any smoothing happens in this iteration

        for (int i = 0; i < path.waypoints.size(); i++)
        {
            int idx1 = rand() % path.waypoints.size();
            int idx2 = rand() % path.waypoints.size();
            
            if (idx1 == idx2) continue; 
            if (idx1 > idx2) std::swap(idx1, idx2); 

            bool collision = false;
            for (const auto &obstacle : problem.obstacles)
            {
                if (lineIntersectsPolygon(path.waypoints[idx1], path.waypoints[idx2], obstacle.verticesCCW()))
                {
                    collision = true;
                    break;
                }
            }

            if (!collision)
            {
                path.waypoints.erase(path.waypoints.begin() + idx1 + 1, path.waypoints.begin() + idx2);
                
                pathSmoothed = true;
                
                break;
            }
        }
    }

    std::cout << "Number of iterations: " << counter << std::endl;
    std::cout << "New Path Length: " << path.length() << std::endl;
   
}

    return path;
}

amp::Path2D MyRRT::plan(const amp::Problem2D &problem)
{
   amp::Path2D path;
double radius = connectRadius; // values that work: 2.51
int n = numSamples;            // values that work: 2500
bool smooth = smoothing;
double goal_tolerance = 0.25;

double step_size = 0.5;
double goal_probability = 0.05;
double distance;

path.waypoints.push_back(problem.q_init);

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> dis(0.0, 1.0);

double qnear_counter;
Eigen::Vector2d qrand, qnear;
std::vector<double> lower_bounds = {problem.x_min, problem.y_min};
std::vector<double> upper_bounds = {problem.x_max, problem.y_max};
std::vector<Eigen::Vector2d> points;
std::map<int, Eigen::Vector2d> nodes_temp;

int counter = 0;

path.waypoints.push_back(problem.q_init);

points.push_back(problem.q_init.head<2>());
nodes[counter] = problem.q_init.head<2>();
counter++;

bool solutionNotFound = true;
int max_iterations = n; // set a limit
int iteration_count = 0;

while (solutionNotFound && iteration_count < max_iterations)
{
    distance = INFINITY;

    // Generate a random value at each iteration
    double random_value = dis(gen);

    if (random_value < goal_probability)
    {
        qrand = problem.q_goal;
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

    // Check if the distance to qnew is within the radius before connecting
    if (euclideanDistance(qnear, qnew) <= radius)
    {
        // Check for collision
        for (const auto &obstacle : problem.obstacles)
        {
            if (lineIntersectsPolygon(qnear, qnew, obstacle.verticesCCW()) || isPointInPolygon(Vector2d(qnew[0], qnew[1]), obstacle.verticesCCW()))
            {
                collision = true;
                break;
            }
        }

        if (!collision)
        {
            points.push_back(qnew);
            nodes[counter] = qnew;
            graphPtr->connect(qnear_counter, counter, distance);  // Connect nodes only within radius
            counter++;

            if (euclideanDistance(qnew, problem.q_goal) < goal_tolerance)
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
    }
    else
    {
        std::cout << "Solution found" << std::endl;

        nodes[n] = problem.q_goal.head<2>();

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

        std::cout << closedSet[n].id << std::endl;

        std::vector<int> pathl = closedSet[n].parent;

        // pathl.push_back(n);

       // std::cout << "Closed Set: ";

       
        // std::cout << std::endl;

        for (int node : pathl)
        {
            //std::cout << node << " ";
            //std::cout << "pushing back" << nodes[node] << std::endl;
            path.waypoints.push_back(nodes[node]);
        }
        //std::cout << std::endl;

        validSolutions = 1;
        pathLength = path.length();
    }
    else
    {
        validSolutions = 0;
        std::cout << "No path found" << std::endl;
    }

    path.waypoints.push_back(problem.q_goal);

    std::cout << "Number of iterations: " << counter << std::endl;
    std::cout << "Old Path Length: " << path.length() << std::endl;
    std::cout << "Path Smoothing set to: " << smooth << std::endl;

    }


    if (smooth && solutionNotFound == false)
    {
        bool pathSmoothed = true;
        
        // Keep trying to smooth the path until no more smoothing is possible
        while (pathSmoothed)
        {
            pathSmoothed = false; // Reset flag to track if any smoothing happens in this iteration

            for (int i = 0; i < path.waypoints.size(); i++)
            {
                int idx1 = rand() % path.waypoints.size();
                int idx2 = rand() % path.waypoints.size();
                
                if (idx1 == idx2) continue; 
                if (idx1 > idx2) std::swap(idx1, idx2); 

                bool collision = false;
                for (const auto &obstacle : problem.obstacles)
                {
                    if (lineIntersectsPolygon(path.waypoints[idx1], path.waypoints[idx2], obstacle.verticesCCW()))
                    {
                        collision = true;
                        break;
                    }
                }

                if (!collision)
                {
                    path.waypoints.erase(path.waypoints.begin() + idx1 + 1, path.waypoints.begin() + idx2);
                    
                    pathSmoothed = true;
                    
                    break;
                }
            }
        }

        std::cout << "Number of iterations: " << counter << std::endl;
        std::cout << "New Path Length: " << path.length() << std::endl;
    
    }

        std::cout << "n: " << n << "r: " << radius << std::endl;


    return path;
}
