#pragma once

// This includes all of the necessary header files in the toolbox
#include "hw/HW7.h"
#include "AMPCore.h"
#include <memory> // For smart pointers
#include <map>    // For node map

class MyPRM : public amp::PRM2D {
private:
    std::shared_ptr<amp::Graph<double>> graphPtr;
    std::map<amp::Node, Eigen::Vector2d> nodes;

    int validSolutions;
    double pathLength;
    double computationTime;
    int numSamples;
    double connectRadius;

    bool smoothing;
    

public:
    MyPRM() : graphPtr(std::make_shared<amp::Graph<double>>()) {}
    std::shared_ptr<amp::Graph<double>> getGraph() const { return graphPtr; }
    std::map<amp::Node, Eigen::Vector2d> getNodes() const { return nodes; }
    virtual amp::Path2D plan(const amp::Problem2D& problem) override;

    int getValidSolutions() const { return validSolutions; }
    double getPathLength() const { return pathLength; }
    double getComputationTime() const { return computationTime; }

    void setNumSamples(int n) { numSamples = n; }
    int getNumSamples() const { return numSamples; }

    void setConnectionRadius(double r) { connectRadius = r; }
    double getConnectionRadius() const { return connectRadius; }

    void setSmoothing(bool s) { smoothing = s; }

};

class MyRRT : public amp::GoalBiasRRT2D {
private:
    std::shared_ptr<amp::Graph<double>> graphPtr;
    std::map<amp::Node, Eigen::Vector2d> nodes;

    int validSolutions;
    double pathLength;
    double computationTime;
    int numSamples;
    double connectRadius;

    bool smoothing;
    


public:
    MyRRT() : graphPtr(std::make_shared<amp::Graph<double>>()) {}
    std::shared_ptr<amp::Graph<double>> getGraph() const { return graphPtr; }
    std::map<amp::Node, Eigen::Vector2d> getNodes() const { return nodes; }
    virtual amp::Path2D plan(const amp::Problem2D& problem) override;

    int getValidSolutions() const { return validSolutions; }
    double getPathLength() const { return pathLength; }
    double getComputationTime() const { return computationTime; }

    void setNumSamples(int n) { numSamples = n; }
    int getNumSamples() const { return numSamples; }

    void setConnectionRadius(double r) { connectRadius = r; }
    double getConnectionRadius() const { return connectRadius; }

    void setSmoothing(bool s) { smoothing = s; }
};
