//
// Created by imbaguanxin on 2019/10/21.
//

#include <iostream>
#include "wayPointsGenerator.hpp"

wayPointsGenerator::wayPointsGenerator(model::threeDmodel &m) :
        astar(aStarPathPlanner(m)), simplifier(regressionSimplifier()),
        droneSize(0.1), stepLength(.1), gridSize(1),
        wayPointsPath(std::vector<glm::vec3>()) {}

wayPointsGenerator::wayPointsGenerator(model::threeDmodel &m, float gs, float ds, float sl) :
        astar(aStarPathPlanner(m)), simplifier(regressionSimplifier()),
        droneSize(ds), stepLength(sl), gridSize(gs),
        wayPointsPath(std::vector<glm::vec3>()) {}

void wayPointsGenerator::setStepLength(float sl) {
    stepLength = sl;
}

void wayPointsGenerator::setModel(model::threeDmodel &m) {
    astar.setModel(m);
}

void wayPointsGenerator::setDroneSize(float ds) {
    droneSize = ds;
}

std::vector<glm::vec3> wayPointsGenerator::genPoints(glm::vec3 &fromP, glm::vec3 &toP) {
    float imaginary_droneSize = droneSize / gridSize;
    float imaginary_stepLength = stepLength / gridSize;
    astar.setDroneSize(imaginary_droneSize);
    astar.setStepLength(imaginary_stepLength);
    simplifier.setDroneSize(imaginary_droneSize);
    try {
        astar.planPath(fromP, toP);
        std::vector<glm::vec3> aStarResult = astar.getPath();
        std::vector<glm::vec3> result = simplifier.simplify(aStarResult);
        return result;
    } catch (std::exception &e) {
        std::cout << "Can't find a valid path" << std::endl;
        std::vector<glm::vec3> result = std::vector<glm::vec3>();
        result.emplace_back(fromP);
        return result;
    }
}

std::vector<glm::vec3> wayPointsGenerator::getGeneratedPoints() {
    std::vector<glm::vec3> result = std::vector<glm::vec3>();
    for (auto point : wayPointsPath) {
        result.emplace_back(glm::vec3(point));
    }
    return result;
}


