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
// input
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
        astar.aStarPathPlan(glm::vec3(fromP) / grid_size, glm::vec3(toP) / grid_size);
        std::vector<glm::vec3> aStarResult = astar.getPath();
        // simplifier
        std::vector<glm::vec3> simplified = simplifier.simplify(aStarResult);
        wayPointsPath.clear();
        for (auto pt : simplified) {
            wayPointsPath.push_back(pt * grid_size);
        }
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

//TODO
// This is how I generate HGJ inputs.
std::vector<HGJ::vec3f> wayPointsGenerator::getGeneratedPointsHGJ() {
    std::vector<HGJ::vec3f> result;
    for (auto &point: wayPointsPath) {
        result.emplace_back(HGJ::vec3f(point.x, point.y, point.z));
    }
    return result;
}

void wayPointsGenerator::setGridSize(float gs) {
    grid_size = gs;
}


