// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW9.h"
#include "hw/HW2.h"
#include "MyKinoRRT.h"

using namespace amp;

// Load problems and map agent for quick testing
std::vector<KinodynamicProblem2D> problems = {HW9::getStateIntProblemWS1(), HW9::getStateIntProblemWS2(), HW9::getFOUniProblemWS1(), HW9::getFOUniProblemWS2(), HW9::getSOUniProblemWS1(), HW9::getSOUniProblemWS2(), HW9::getCarProblemWS1(), HW9::getParkingProblem()};
std::unordered_map<AgentType, std::function<std::shared_ptr<amp::DynamicAgent>()>> agentFactory = {
    {AgentType::SingleIntegrator, []() { return std::make_shared<MySingleIntegrator>(); }},
    {AgentType::FirstOrderUnicycle, []() { return std::make_shared<MyFirstOrderUnicycle>(); }},
    {AgentType::SecondOrderUnicycle, []() { return std::make_shared<MySecondOrderUnicycle>(); }},
    {AgentType::SimpleCar, []() { return std::make_shared<MySimpleCar>(); }}
};

int main(int argc, char** argv) {
    // Select problem, plan, check, and visualize


    int select = 0;
    KinodynamicProblem2D prob = problems[select];
    MyKinoRRT kino_planner;
    // kino_planner.setNSamples(1000000);
    // kino_planner.setUSamples(100);
    KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
    HW9::check(path, prob);
    if (path.valid)
       // Visualizer::makeFigure(prob, path, false); // Set to 'true' to render animation

    Visualizer::showFigures();
    HW9::grade<MyKinoRRT, MySingleIntegrator, MyFirstOrderUnicycle, MySecondOrderUnicycle, MySimpleCar>("Hadi.Hasbini@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple());
    return 0;
}



// #include "AMPCore.h"
// #include "hw/HW9.h"
// #include "hw/HW2.h"
// #include "MyKinoRRT.h"
// #include <chrono>
// #include <numeric> // For std::accumulate

// using namespace amp;

// // Load problems and map agent for quick testing
// std::vector<KinodynamicProblem2D> problems = {HW9::getStateIntProblemWS1(), HW9::getStateIntProblemWS2(), HW9::getFOUniProblemWS1(), HW9::getFOUniProblemWS2(), HW9::getSOUniProblemWS1(), HW9::getSOUniProblemWS2(), HW9::getCarProblemWS1(), HW9::getParkingProblem()};
// std::unordered_map<AgentType, std::function<std::shared_ptr<amp::DynamicAgent>()>> agentFactory = {
//     {AgentType::SingleIntegrator, []() { return std::make_shared<MySingleIntegrator>(); }},
//     {AgentType::FirstOrderUnicycle, []() { return std::make_shared<MyFirstOrderUnicycle>(); }},
//     {AgentType::SecondOrderUnicycle, []() { return std::make_shared<MySecondOrderUnicycle>(); }},
//     {AgentType::SimpleCar, []() { return std::make_shared<MySimpleCar>(); }}
// };

// // Configurations for benchmarking
// std::vector<std::pair<int, int>> configurations = {{50000, 1}, {50000, 5}, {50000, 10}, {50000, 15}};


// int main(int argc, char** argv) {

//     int select = 7; // Selecting a problem for testing
//     KinodynamicProblem2D prob = problems[select];
//     MyKinoRRT kino_planner;
//     kino_planner.setNSamples(1000000);
//     kino_planner.setUSamples(100);
//     auto start_time = std::chrono::high_resolution_clock::now();
//     KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
//     auto end_time = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> elapsed_time = end_time - start_time;

//     HW9::check(path, prob);
//     if (path.valid)
//         Visualizer::makeFigure(prob, path, false); // Set to 'true' to render animation
//         std::cout << "The Path Length for the image is: " << kino_planner.getPathLength() << std::endl;
//         std::cout << "The Computation Time for the image is: " << elapsed_time.count() << std::endl;

//     for (auto& config : configurations) {
//         int n = config.first;
//         int u_samples = config.second;

//         // Set the number of samples and control samples for the planner
//         kino_planner.setNSamples(n);
//         kino_planner.setUSamples(u_samples);

//         std::vector<double> pathLengthList;
//         std::vector<double> computationTimeList;
//         int numValid = 0;
//         std::cout << "Running benchmark for configuration (n=" << n << ", u_samples=" << u_samples << ")..." << std::endl;

//         // Run benchmark 50 times for each configuration
//         for (int i = 0; i < 50; i++) {
//             std::cout << "Iteration " << i<< "..." << std::endl;
//             auto start_time = std::chrono::high_resolution_clock::now();
            
//             // Plan the path using the current configuration
//             KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());

//             auto end_time = std::chrono::high_resolution_clock::now();
//             std::chrono::duration<double> elapsed_time = end_time - start_time;
//             computationTimeList.push_back(elapsed_time.count());

//             if (path.valid) {
//                 pathLengthList.push_back(kino_planner.getPathLength()); // Use the getter to retrieve path length
//                 numValid++;
//             }
//         }

//         double avgPathLength = std::accumulate(pathLengthList.begin(), pathLengthList.end(), 0.0) / pathLengthList.size();
//         double avgComputationTime = std::accumulate(computationTimeList.begin(), computationTimeList.end(), 0.0) / computationTimeList.size();

//         std::cout << "Configuration (n=" << n << ", u_samples=" << u_samples << "):" << std::endl;
//         std::cout << "Average Path Length: " << avgPathLength << std::endl;
//         std::cout << "Average Computation Time: " << avgComputationTime << " seconds" << std::endl;
//         std::cout << "Valid Paths: " << numValid << " out of 50" << std::endl;

//         Visualizer::makeBoxPlot({pathLengthList}, {"Path Length"}, "Path Length Benchmark (n=" + std::to_string(n) + ", u_samples=" + std::to_string(u_samples) + ")", "Path Length", "Length");
//         Visualizer::makeBoxPlot({computationTimeList}, {"Time in s"}, "Computation Time Benchmark (n=" + std::to_string(n) + ", u_samples=" + std::to_string(u_samples) + ")", "Time in s", "Computation Time");
//         Visualizer::makeBarGraph({static_cast<double>(numValid)}, {"Valid Paths"}, "Valid Paths out of 50 (n=" + std::to_string(n) + ", u_samples=" + std::to_string(u_samples) + ")", "Valid Paths", "Valid Paths");
//     }

//     Visualizer::showFigures();


//     // Optional grading function if needed
//     // HW9::grade<MyKinoRRT, MySingleIntegrator, MyFirstOrderUnicycle, MySecondOrderUnicycle, MySimpleCar>("firstName.lastName@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple());
//     return 0;
// }
