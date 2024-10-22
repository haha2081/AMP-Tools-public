// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"
#include <chrono>
#include <iostream>
#include <vector>
#include <list>

using namespace amp;

int main(int argc, char **argv)
{
    // Define (n, r) combinations to test
    std::vector<std::pair<int, double>> param_combinations = {
        {200, 1}, {200, 2}, {500, 1}, {500, 2}, {1000, 1}, {1000, 2}};

    // Problem setup
    Problem2D problem = HW2::getWorkspace1();

    // Data collection lists
    std::list<std::vector<double>> pathLengthData;
    std::list<std::vector<double>> computationTimeData;
    std::vector<double> validSolutionsList;
    std::vector<std::string> labels;

    bool benchmark = true;

        if (benchmark){

        // Benchmark results storage
        for (const auto &param : param_combinations)
        {
            int n = param.first;
            double r = param.second;

            double totalValidSolutions = 0;
            std::vector<double> pathLengthList;
            std::vector<double> computationTimeList;

            for (int i = 0; i < 100; ++i)
            {
                MyPRM prm;

                prm.setNumSamples(n);
                prm.setConnectionRadius(r);
                prm.setSmoothing(false);

                auto start_time = std::chrono::high_resolution_clock::now();

                prm.plan(problem);

                auto end_time = std::chrono::high_resolution_clock::now();

                std::chrono::duration<double> elapsed_time = end_time - start_time;
                computationTimeList.push_back(elapsed_time.count());

                if (prm.getValidSolutions() > 0)
                {
                    totalValidSolutions += 1;
                }

                auto length = prm.getPathLength();

                if (length > 0)
                {
                    pathLengthList.push_back(length);
                }
            }

            validSolutionsList.push_back(totalValidSolutions);
            pathLengthData.push_back(std::vector<double>(pathLengthList.begin(), pathLengthList.end()));
            computationTimeData.push_back(std::vector<double>(computationTimeList.begin(), computationTimeList.end()));

            std::ostringstream streamObj;

            streamObj << std::fixed << std::setprecision(2) << r;
            std::string strNumber = streamObj.str();

            labels.push_back("n=" + std::to_string(n) + ", r=" + strNumber);
        }

        Visualizer::makeBarGraph(validSolutionsList, labels, "Valid Solutions Benchmark", "Parameters (n, r)", "Valid Solutions");

        Visualizer::makeBoxPlot(pathLengthData, labels, "Path Length Benchmark", "Parameters (n, r)", "Path Length");

        Visualizer::makeBoxPlot(computationTimeData, labels, "Computation Time Benchmark", "Parameters (n, r)", "Computation Time (seconds)");
        }
        else{

        MyPRM prm;

        prm.setNumSamples(200);
        prm.setConnectionRadius(2);

        prm.setSmoothing(false);

        Visualizer::makeFigure(problem, prm.plan(problem), *prm.getGraph(), prm.getNodes());

    }


    //Generate a random problem and test RRT
    //     if (benchmark){

    //     // Benchmark results storage

    //     int n = 5000;
    //     double r = 0.5;


    //         double totalValidSolutions = 0;
    //         std::vector<double> pathLengthList;
    //         std::vector<double> computationTimeList;

    //         for (int i = 0; i < 100; ++i)
    //         {
    //             MyRRT rrt;

    //             rrt.setNumSamples(n);
    //             rrt.setConnectionRadius(r);
    //             rrt.setSmoothing(true);

    //             auto start_time = std::chrono::high_resolution_clock::now();

    //             rrt.plan(problem);

    //             auto end_time = std::chrono::high_resolution_clock::now();

    //             std::chrono::duration<double> elapsed_time = end_time - start_time;
    //             computationTimeList.push_back(elapsed_time.count());

    //             if (rrt.getValidSolutions() > 0)
    //             {
    //                 totalValidSolutions += 1;
    //             }

    //             auto length = rrt.getPathLength();

    //             if (length > 0)
    //             {
    //                 pathLengthList.push_back(length);
    //             }
    //         }

    //         validSolutionsList.push_back(totalValidSolutions);
    //         pathLengthData.push_back(std::vector<double>(pathLengthList.begin(), pathLengthList.end()));
    //         computationTimeData.push_back(std::vector<double>(computationTimeList.begin(), computationTimeList.end()));

    //         std::ostringstream streamObj;

    //         streamObj << std::fixed << std::setprecision(2) << r;
    //         std::string strNumber = streamObj.str();

    //         labels.push_back("n=" + std::to_string(n) + ", r=" + strNumber);
        

    //     Visualizer::makeBarGraph(validSolutionsList, labels, "Valid Solutions Benchmark", "Parameters (n, r)", "Valid Solutions");

    //     Visualizer::makeBoxPlot(pathLengthData, labels, "Path Length Benchmark", "Parameters (n, r)", "Path Length");

    //     Visualizer::makeBoxPlot(computationTimeData, labels, "Computation Time Benchmark", "Parameters (n, r)", "Computation Time (seconds)");
    //     }
    //     else{

    //     MyRRT rrt;

    //     rrt.setNumSamples(200);
    //     rrt.setConnectionRadius(0.5);

    //     rrt.setSmoothing(true);

    //     Visualizer::makeFigure(problem, rrt.plan(problem), *rrt.getGraph(), rrt.getNodes());


    // }







 
    // HW7::generateAndCheck(rrt, path, problem);

    Visualizer::showFigures();
    // // Grade method
    // HW7::grade<MyPRM, MyRRT>("firstName.lastName@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}