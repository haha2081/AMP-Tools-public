// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"

using namespace amp;

#include <iostream>
#include <vector>
#include <chrono>
#include <numeric> // For std::accumulate

void timer_example()
{
    double startTime;
    amp::Timer timer("timer");
    for (int i = 0; i < 5; ++i)
    {
        startTime = timer.now(TimeUnit::ms);
        std::cout << "Press any key to continue...\n";
        std::cin.get();
        std::cout << "Time since last run: " << timer.now(TimeUnit::ms) - startTime << std::endl;
    }
    timer.stop();
    std::cout << "Total time elapsed: " << Profiler::getTotalProfile("timer") << std::endl;
}

int main(int argc, char **argv)
{
    // Run timer example (useful for benchmarking)
    // timer_example();

    // Initialize Workspace 1 with 3 agents
    amp::RNG::seed(amp::RNG::randiUnbounded());

    //std::vector<std::vector<double>> highLevelComp;
    std::vector<std::vector<double>> highLevelPath;
    std::vector<double> highLevelValid;
    double numValid;

    //std::vector<double> averageComputationTimes;
    std::vector<double> averagePathLengths;

    // for (int j = 5; j < 6; j++)
    // {
    //     bool shown = false;
    //     MultiAgentProblem2D problem = HW8::getWorkspace1(j);
    //     std::vector<std::vector<Eigen::Vector2d>> collision_states;

    //     std::vector<double> computationTimeList;
    //     std::vector<double> pathLengthList;
    //     std::cout << "Running Central Planner for m = " << j << "" << std::endl;

    //     for (int i = 0; i < 100; i++)
    //     {
    //         if (i % 50 == 0){
    //             std::cout << i << std::endl;
    //         }

    //         MyCentralPlanner central_planner;
    //         auto start_time = std::chrono::high_resolution_clock::now();

    //         MultiAgentPath2D path = central_planner.plan(problem);

    //         auto end_time = std::chrono::high_resolution_clock::now();
    //         std::chrono::duration<double> elapsed_time = end_time - start_time;
    //         //std::cout << elapsed_time.count() << std::endl;
    //         computationTimeList.push_back(elapsed_time.count());

    //         pathLengthList.push_back(central_planner.getPathLength());

    //         bool isValid = HW8::check(path, problem, collision_states);

    //         if (isValid){
    //             numValid+=1;
    //         }

    //         //std::cout << "valid: " << path.valid << std::endl;

    //         if (!shown && isValid)
    //         {
    //             Visualizer::makeFigure(problem, path, collision_states);
    //             shown = true;
    //         }
    //     }

    //     highLevelValid.push_back(numValid);
    //     // Calculate and store the average computation time and path length for this batch
    //     double avgComputationTime = std::accumulate(computationTimeList.begin(), computationTimeList.end(), 0.0) / computationTimeList.size();
    //     double avgPathLength = std::accumulate(pathLengthList.begin(), pathLengthList.end(), 0.0) / pathLengthList.size();

    //     averageComputationTimes.push_back(avgComputationTime);
    //     averagePathLengths.push_back(avgPathLength);

    //     Visualizer::makeBoxPlot({pathLengthList}, {"Path Length"}, "Path Length Benchmark for m = " + std::to_string(j), "Path Length", "Length");
    //     Visualizer::makeBoxPlot({computationTimeList}, {"Time in s"}, "Computation Time Benchmark for m = " + std::to_string(j), "Time in s", "Computation Time");

    //     highLevelComp.push_back(computationTimeList);
    //     highLevelPath.push_back(pathLengthList);
    // }

    // // Output the averages for each agent count
    // for (int i = 0; i < averageComputationTimes.size(); i++)
    // {
    //     std::cout << "The average computation time for m = " << i + 2 << ": " << averageComputationTimes[i] << " seconds" << std::endl;
    //     std::cout << "The average path length for m = " << i + 2 << ": " << averagePathLengths[i] << " units" << std::endl;
    //     std::cout << "The number of valid paths for m = " << i + 2 << ": " << highLevelValid[i] << std::endl;
    // }


    //////////////////////////////////////////////////   //////////////////////////////////////////////////



    // MultiAgentProblem2D problem = HW8::getWorkspace1(6);
    // std::vector<std::vector<Eigen::Vector2d>> collision_states;
    // MyCentralPlanner central_planner;
    // MultiAgentPath2D path = central_planner.plan(problem);

    // bool isValid = HW8::check(path, problem, collision_states);

    std::vector<std::vector<double>> highLevelComp;

    std::vector<double> averageComputationTimes;

    for (int j = 2; j < 7; j++)
    {

        bool shown = false;
        MultiAgentProblem2D problem = HW8::getWorkspace1(j);
        std::vector<std::vector<Eigen::Vector2d>> collision_states;

        std::vector<double> computationTimeList;
        std::cout << "Running Decentral Planner for m = " << j << "" << std::endl;

        for (int i = 0; i < 100; i++)
        {
            if (i % 50 == 0){
                std::cout << i << std::endl;
            }

            MyDecentralPlanner decentral_planner;
            
            auto start_time = std::chrono::high_resolution_clock::now();

            MultiAgentPath2D path = decentral_planner.plan(problem);

            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_time = end_time - start_time;

            //std::cout << elapsed_time.count() << std::endl;
            computationTimeList.push_back(elapsed_time.count());

            bool isValid = HW8::check(path, problem, collision_states);

            //std::cout << "valid: " << path.valid << std::endl;

    
        }


        double avgComputationTime = std::accumulate(computationTimeList.begin(), computationTimeList.end(), 0.0) / computationTimeList.size();

        averageComputationTimes.push_back(avgComputationTime);

        Visualizer::makeBoxPlot({computationTimeList}, {"Time in s"}, "Computation Time Benchmark for m = " + std::to_string(j), "Time in s", "Computation Time");

        highLevelComp.push_back(computationTimeList);
    }


    // Output the averages for each agent count
    for (int i = 0; i < averageComputationTimes.size(); i++)
    {
        std::cout << "The average computation time for m = " << i + 2 << ": " << averageComputationTimes[i] << " seconds" << std::endl;
        
    }

    

  // Visualizer::makeFigure(problem, path, collision_states);

   //////////////////////////////////////////////////

        // Solve using a centralized approach

        // // Solve using a decentralized approach

        // MultiAgentProblem2D problem = HW8::getWorkspace1(5);
        // std::vector<std::vector<Eigen::Vector2d>> collision_states;
        // MyDecentralPlanner decentral_planner;s
        // MultiAgentPath2D path = decentral_planner.plan(problem);
        // collision_states = {{}};
        // //HW8::generateAndCheck(decentral_planner, path, problem, collision_states);
        // bool isValid = HW8::check(path, problem, collision_states);
        // Visualizer::makeFigure(problem, path, collision_states);

        // Visualize and grade methods
        Visualizer::showFigures();
        //HW8::grade<MyCentralPlanner, MyDecentralPlanner>("hadi.hasbini@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
        return 0;
    }