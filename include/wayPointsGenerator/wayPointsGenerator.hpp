//
// Created by imbaguanxin on 2019/10/21.
//

#ifndef ASTAR_CPP_WAYPOINTSGENERATOR_HPP
#define ASTAR_CPP_WAYPOINTSGENERATOR_HPP

#include "aStar/aStar.hpp"
#include "pathSimplifier/regressionSimplifier.hpp"

class wayPointsGenerator {

private:

    float grid_size, drone_size;

    aStar astar;

    regressionSimplifier simplifier;

    std::vector<glm::vec3> wayPointsPath;

public:

    explicit wayPointsGenerator(model::threeDmodel &m);

    wayPointsGenerator(model::threeDmodel &m, float gridSize, float droneSize);

    void setModel(model::threeDmodel &m);

    void setDroneSize(float ds);

    std::vector<glm::vec3> genPoints(glm::vec3 &fromP, glm::vec3 &toP);

    std::vector<glm::vec3> getGeneratedPoints();

};


#endif //ASTAR_CPP_WAYPOINTSGENERATOR_HPP
