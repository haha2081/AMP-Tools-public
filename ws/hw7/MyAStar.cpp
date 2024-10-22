#include "MyAStar.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>

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

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem &problem, const amp::SearchHeuristic &heuristic)
{
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    GraphSearchResult result = {false, {}, 0.0};

    std::priority_queue<Node> openList;

    std::unordered_map<int, Node> openSet;

    std::unordered_map<int, Node> closedSet;

    Node start;
    start.id = problem.init_node;
    start.g = 0.0;
    start.h = heuristic(start.id);

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

        openSet.erase(nBest.id);
        closedSet[nBest.id] = nBest;

        if (nBest.id == problem.goal_node)
        {
            goalFound = true;
            break;
        }

        auto children = problem.graph->children(nBest.id);
        int i = 0;
        for (const auto &child_id : children)
        {
            double tentative_g = nBest.g + problem.graph->outgoingEdges(nBest.id)[i];
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
                child.h = heuristic(child.id);
                child.parent = nBest.parent;
                child.parent.push_back(nBest.id);

                openSet[child_id] = child;
                openList.push(child);
            }
        }

        counter++;
    }

    if (goalFound)
    {
        result.success = true;
        int itr = 0;


        std::vector<int> path = closedSet[problem.goal_node].parent;
        path.push_back(problem.goal_node);
        for (int node : path)
        {
            std::cout << node << " ";
            result.node_path.push_back(node);
            auto chilren = problem.graph->children(node);

            if (itr < path.size() - 1)
            {
                auto it = std::find(chilren.begin(), chilren.end(), path[itr + 1]);
                int index = std::distance(chilren.begin(), it);
                result.path_cost += problem.graph->outgoingEdges(node)[index];
                itr += 1;
            }
        }
        std::cout << std::endl;
    }

    else
    {
        result.success = false;
        std::cout << "No path found" << std::endl;
    }

    result.print();

    std::cout << "Number of iterations: " << counter << std::endl;
    return result;
}