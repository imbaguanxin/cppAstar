//
// Created by imbaguanxin on 2019/10/21.
//

#include <iostream>
#include "wayPointsGenerator.hpp"

// default constructor, setting drone size to 0.1m and grid size to 0.1m
wayPointsGenerator::wayPointsGenerator(model::threeDmodel &m) :
        astar(m), simplifier(regressionSimplifier()),
        drone_size(0.1), grid_size(0.1),
        wayPointsPath(std::vector<glm::vec3>()) {}

// recommended constructor
wayPointsGenerator::wayPointsGenerator(model::threeDmodel &m, float gs, float ds) :
        astar(m), simplifier(regressionSimplifier()),
        drone_size(ds), grid_size(gs),
        wayPointsPath(std::vector<glm::vec3>()) {}

// model setter
void wayPointsGenerator::setModel(model::threeDmodel &m) {
    astar.setModel(m);
}

// drone size setter
void wayPointsGenerator::setDroneSize(float ds) {
    drone_size = ds;
}

// generate points: first use A* algorithm then simplify
std::vector<glm::vec3> wayPointsGenerator::genPoints(const glm::vec3 &fromP, const glm::vec3 &toP) {
    float imaginary_droneSize = drone_size / grid_size;
    astar.setDroneSize(imaginary_droneSize);
    simplifier.setDroneSize(imaginary_droneSize);
    try {
        // A* planning
        astar.aStarPathPlan(fromP, toP);
        std::vector<glm::vec3> aStarResult = astar.getPath();
        // simplifier
        wayPointsPath = simplifier.simplify(aStarResult);
        return wayPointsPath;
    } catch (std::exception &e) {
        std::cout << "Can't find a valid path" << std::endl;
        std::vector<glm::vec3> result = std::vector<glm::vec3>();
        result.emplace_back(fromP);
        return result;
    }
}

// return the generated points without regenerating.
std::vector<glm::vec3> wayPointsGenerator::getGeneratedPoints() {
    std::vector<glm::vec3> result = std::vector<glm::vec3>();
    for (auto &point : wayPointsPath) {
        result.emplace_back(glm::vec3(point));
    }
    return result;
}

std::vector<HGJ::vec3f> wayPointsGenerator::getGeneratedPointsHGJ() {
    std::vector<HGJ::vec3f> result;
    for (auto &point: wayPointsPath) {
        result.emplace_back(HGJ::vec3f(point.x, point.y, point.z));
    }
    return result;
}


