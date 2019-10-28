//
// Created by imbaguanxin on 2019/10/21.
//

#include <iostream>
#include "wayPointsGenerator.hpp"

wayPointsGenerator::wayPointsGenerator(model::threeDmodel &m) :
        astar(m), simplifier(regressionSimplifier()),
        drone_size(1.0), grid_size(1),
        wayPointsPath(std::vector<glm::vec3>()) {}

wayPointsGenerator::wayPointsGenerator(model::threeDmodel &m, float gs, float ds) :
        astar(m), simplifier(regressionSimplifier()),
        drone_size(ds), grid_size(gs),
        wayPointsPath(std::vector<glm::vec3>()) {}

void wayPointsGenerator::setModel(model::threeDmodel &m) {
    astar.setModel(m);
}

void wayPointsGenerator::setDroneSize(float ds) {
    drone_size = ds;
}

std::vector<glm::vec3> wayPointsGenerator::genPoints(glm::vec3 &fromP, glm::vec3 &toP) {
    float imaginary_droneSize = drone_size / grid_size;
    astar.setDroneSize(imaginary_droneSize);
    simplifier.setDroneSize(imaginary_droneSize);
    try {
        astar.aStarPathPlan(fromP, toP);
        std::list<glm::vec3> aStarResult = astar.getPath();
        std::vector<glm::vec3> vecResult = std::vector<glm::vec3>();
        for (auto point : aStarResult) {
            vecResult.emplace_back(point);
        }
        return simplifier.simplify(vecResult);
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


