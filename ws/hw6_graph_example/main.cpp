#include "AMPCore.h"

int main() {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Let's connect some nodes with some edges
    amp::Graph<std::string> graph;


// Connections from node 0
graph.connect(0, 1, "1");
graph.connect(0, 2, "1");
graph.connect(0, 3, "1");

// Connections from node 1 (A)
graph.connect(1, 4, "3"); // A to D
graph.connect(1, 5, "3"); // A to E
graph.connect(1, 6, "4"); // A to F

// Connections from node 2 (B)
graph.connect(2, 6, "2"); // B to F
graph.connect(2, 7, "1"); // B to H
graph.connect(2, 8, "2"); // B to I

// Connections from node 3 (C)
graph.connect(3, 9, "1"); // C to J
graph.connect(3, 10, "3"); // C to L
graph.connect(3, 11, "2"); // C to K

// Connections from node 6 (F)
graph.connect(6, 12, "3"); // F to GOAL

// Connections from node 7 (H)
graph.connect(7, 12, "1"); // H to GOAL

// Connections from node 8 (I)
graph.connect(8, 12, "2"); // I to GOAL

// Connections from node 4 (D)
graph.connect(4, 12, "3"); // D to GOAL

// Connections from node 5 (E)
graph.connect(5, 12, "1"); // E to GOAL

// Connections from node 9 (J)
graph.connect(9, 12, "3"); // J to GOAL

// Connections from node 10 (L)
graph.connect(10, 12, "3"); // L to GOAL

// Connections from node 11 (K)
graph.connect(11, 12, "2"); // K to GOAL









    /////////////////// HW
    // graph.connect(0, 1, "3");
    // graph.connect(0, 2,"1");
    // graph.connect(0, 3, "3");
    // graph.connect(0, 4, "1");
    // graph.connect(0, 5, "3");

    // graph.connect(1, 6, "1");
    // graph.connect(1, 7, "3");

    // graph.connect(2, 1, "0");
    // graph.connect(2, 7, "3");
    // graph.connect(2, 8, "2");
    // graph.connect(2, 9, "1");

    // graph.connect(3, 9, "1");
    
    // graph.connect(4, 9, "1");
    // graph.connect(4, 10, "2");
    // graph.connect(4, 11, "3");
    // graph.connect(4, 5, "2");

    // graph.connect(5, 11, "1");
    // graph.connect(5, 12, "1");


    // graph.connect(6, 7, "1");

    // graph.connect(7, 13, "1");
    
    // graph.connect(8, 13, "3");

    // graph.connect(9, 13, "3");

    // graph.connect(10, 13, "3");

    // graph.connect(12, 11, "3");

    // graph.connect(11, 13, "1");

//////////////////////////////
    // Print the nodes inside the graph 
    std::vector<amp::Node> nodes = graph.nodes();
    NEW_LINE;
    INFO("Nodes in graph:");
    for (amp::Node n : nodes)
        INFO(" - " << n);

    // Look at the children of node `1`
    amp::Node node = 1;
    const std::vector<amp::Node>& children = graph.children(node);

    // Print the children
    NEW_LINE;
    INFO("Children of node " << node << ":");
    for (amp::Node n : children)
        INFO(" - " << n);

    // Look at the outgoing edges of node `1`
    const auto& outgoing_edges = graph.outgoingEdges(node);

    // Print the outgoing edges (notice they are always in the same order as the children nodes)
    NEW_LINE;
    INFO("Outgoing edges of node " << node << ":");
    for (const auto& edge : outgoing_edges)
        INFO(" - " << edge);

    // Ok let's get funky and disconnect some nodes!
    graph.disconnect(1, 0); // disconnect any edge between 1 and 0
    graph.disconnect(6, 2, "do"); // disconnect only the edge "do" going from 6 to 2
    NEW_LINE;
    graph.print();

    // Random graph
    std::shared_ptr<amp::Graph<double>> random_graph = amp::GraphTools::generateRandomGraphDouble(10, 0.0, 1.0, 3);
    NEW_LINE;
    random_graph->print("Random Graph");

    return 0;
}