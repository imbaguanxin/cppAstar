//
// Created by imbaguanxin on 2019/10/21.
//

#ifndef ASTAR_CPP_WAYPOINTSGENERATOR_HPP
#define ASTAR_CPP_WAYPOINTSGENERATOR_HPP

#include "aStar/aStar.hpp"
#include "pathSimplifier/regressionSimplifier.hpp"

class wayPointsGenerator {

private:

    // grid size and drone size in real world length (in terms of meter)
    float grid_size, drone_size;

    // astar path planner
    aStar astar;

    // regression path simplifier
    regressionSimplifier simplifier;

    // result
    std::vector<glm::vec3> wayPointsPath;

public:

    // default constructor, setting drone size to 0.1m and grid size to 0.1m
    explicit wayPointsGenerator(model::threeDmodel &m);

    // recommended constructor
    wayPointsGenerator(model::threeDmodel &m, float gridSize, float droneSize);

    // model setter
    void setModel(model::threeDmodel &m);

    // drone size setter
    void setDroneSize(float ds);

    // generate points: first use A* algorithm then simplify
    std::vector<glm::vec3> genPoints(glm::vec3 &fromP, glm::vec3 &toP);

    // return the generated points without regenerating.
    std::vector<glm::vec3> getGeneratedPoints();

};


#endif //ASTAR_CPP_WAYPOINTSGENERATOR_HPP
