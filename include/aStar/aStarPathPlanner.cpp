//
// Created by imbaguanxin on 2019/9/27.
//

#include <glm/glm.hpp>
#include <glm/gtx/hash.hpp>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <limits>
#include "aStarPathPlanner.h"
#include "model/threeDModel.h"

using namespace std;

/**
 * The default constructor that takes in a model
 * @param m
 */
aStarPathPlanner::aStarPathPlanner(model::threeDmodel &m)
        : model(m), stepLength(0.1), droneSize(0) {
    path = vector<glm::vec3>();
    initDir();
}

/**
 * Initializes the private fields including possibleDir and sixDirs
 */
void aStarPathPlanner::initDir() {
    // up, down, right, left, forward, back
    sixDirs = vector<glm::vec3>();
    sixDirs.emplace_back(glm::vec3(0, 0, 1));
    sixDirs.emplace_back(glm::vec3(0, 0, -1));
    sixDirs.emplace_back(glm::vec3(0, 1, 0));
    sixDirs.emplace_back(glm::vec3(0, -1, 0));
    sixDirs.emplace_back(glm::vec3(1, 0, 0));
    sixDirs.emplace_back(glm::vec3(-1, 0, 0));
//    ====================================================
    possibleDir = vector<glm::vec3>();
    possibleDir.emplace_back(glm::vec3(0, 0, 1));
    possibleDir.emplace_back(glm::vec3(0, 0, -1));
    possibleDir.emplace_back(glm::vec3(0, 1, 0));
    possibleDir.emplace_back(glm::vec3(0, -1, 0));
    possibleDir.emplace_back(glm::vec3(1, 0, 0));
    possibleDir.emplace_back(glm::vec3(-1, 0, 0));
//    =============================================
    possibleDir.emplace_back(glm::vec3(0, 1, 1) / (float) sqrt(2.0));
    possibleDir.emplace_back(glm::vec3(0, 1, -1) / (float) sqrt(2.0));
    possibleDir.emplace_back(glm::vec3(0, -1, 1) / (float) sqrt(2.0));
    possibleDir.emplace_back(glm::vec3(0, -1, -1) / (float) sqrt(2.0));
    possibleDir.emplace_back(glm::vec3(1, 1, 0) / (float) sqrt(2.0));
    possibleDir.emplace_back(glm::vec3(-1, 1, 0) / (float) sqrt(2.0));
    possibleDir.emplace_back(glm::vec3(1, -1, 0) / (float) sqrt(2.0));
    possibleDir.emplace_back(glm::vec3(-1, -1, 0) / (float) sqrt(2.0));
    possibleDir.emplace_back(glm::vec3(1, 0, 1) / (float) sqrt(2.0));
    possibleDir.emplace_back(glm::vec3(-1, 0, 1) / (float) sqrt(2.0));
    possibleDir.emplace_back(glm::vec3(1, 0, -1) / (float) sqrt(2.0));
    possibleDir.emplace_back(glm::vec3(-1, 0, -1) / (float) sqrt(2.0));
//    ==============================================
    possibleDir.emplace_back(glm::vec3(1, 1, 1) / (float) sqrt(3.0));
    possibleDir.emplace_back(glm::vec3(-1, 1, 1) / (float) sqrt(3.0));
    possibleDir.emplace_back(glm::vec3(1, 1, -1) / (float) sqrt(3.0));
    possibleDir.emplace_back(glm::vec3(1, -1, 1) / (float) sqrt(3.0));
    possibleDir.emplace_back(glm::vec3(-1, -1, 1) / (float) sqrt(3.0));
    possibleDir.emplace_back(glm::vec3(1, -1, -1) / (float) sqrt(3.0));
    possibleDir.emplace_back(glm::vec3(-1, 1, -1) / (float) sqrt(3.0));
    possibleDir.emplace_back(glm::vec3(-1, -1, -1) / (float) sqrt(3.0));
}


/**
 * The method that plan the path using A*
 * @param fromP The starting point
 * @param toP The destination
 * @return Whether the path planning is finished correctly
 */
bool aStarPathPlanner::planPath(glm::vec3 fromP, glm::vec3 toP) {
    // check whether the points are within the map and not blocked
    if (model.checkValidPos(fromP.x, fromP.y, fromP.z) &&
        model.checkValidPos(toP.x, toP.y, toP.z) &&
        !model.checkBlocked(fromP.x, fromP.y, fromP.z) &&
        !model.checkBlocked(toP.x, toP.y, toP.z)) {
        return astarPlan(glm::vec3(fromP), glm::vec3(toP));
    } else {
        // If not valid, return the first point so that the pipeline won't stop.
        path = vector<glm::vec3>();
        path.emplace_back(glm::vec3(fromP));
        throw invalid_argument("Invalid input points!");
    }
}

/**
 * The actual A* implementation after position checking
 * @param fromP The starting point of the path
 * @param toP The end point of the path
 * @return Whether the path was successfully planned
 */
bool aStarPathPlanner::astarPlan(glm::vec3 fromP, glm::vec3 toP) {
    path = vector<glm::vec3>();
    unordered_map<glm::vec3, int> passedPoints = unordered_map<glm::vec3, int>();
    glm::vec3 tempStartPoint = glm::vec3(fromP);
    while (true) {
        // keep trying to find the next point
        try {
            glm::vec3 nextPoint = astarFindNext(tempStartPoint, toP, passedPoints);
            if (nextPoint == toP) {
                path.emplace_back(nextPoint);
                return true;
            } else {
                tempStartPoint = nextPoint;
                path.emplace_back(glm::vec3(nextPoint));
            }
        } catch (char *e) {
            // If not able to find a path, check whether the path is empty.
            // If the path is empty, simply put a single start point to keep the next steps working.
            if (path.empty()) {
                path.emplace_back(glm::vec3(fromP));
            }
            return false;
        }
    }
}

/**
 * Find where to go in the next step
 * @param fromP The current position of the drone
 * @param toP The destination of the path planning
 * @param passed The positions where the drone have already been to
 * @return The next position the drone should go
 */
glm::vec3 aStarPathPlanner::astarFindNext(glm::vec3 fromP, glm::vec3 toP, unordered_map<glm::vec3, int> &passed) {
    bool hasNextPointflag = false;
    glm::vec3 nextPoint;
    float minDis = numeric_limits<float>::max();
    // Iterate over the possible directions to find the next point to go
    for (auto const &dir: possibleDir) {
        glm::vec3 possibleNext = glm::vec3(fromP) + (glm::vec3(dir) * stepLength);
        // check whether the destination is close enough.
        if (glm::distance(possibleNext, toP) <= glm::length(dir) * stepLength) {
            return toP;
        }
        // Check whether the next step candidate is within reach
        if (validPosWithDroneSize(possibleNext)) {
            float distanceNext = 0;
            auto mapSearch = passed.find(possibleNext);
            // Calculate the next step candidate's score
            // If the drone has passed the next step candidate, we higher its score.
            if (mapSearch != passed.end()) {
                distanceNext =
                        glm::distance(possibleNext, toP) * mapSearch->second + glm::length(dir) * stepLength;
            } else {
                distanceNext = glm::distance(possibleNext, toP) + glm::length(dir) * stepLength;
            }
            // Find the lowest score as the real candidate.
            if (distanceNext < minDis) {
                minDis = distanceNext;
                nextPoint = possibleNext;
                hasNextPointflag = true;
            }
        }
    }
    // If we didn't find a next point, we throw a exception
    if (!hasNextPointflag) {
        throw invalid_argument("not able to reach");
    } else {
        // If found a next point, mark it to the "passed point map" and then return it.
        auto mapSearch = passed.find(nextPoint);
        if (mapSearch != passed.end()) {
            passed.emplace(nextPoint, mapSearch->second + 1);
        } else {
            passed.emplace(nextPoint, 1);
        }
        return nextPoint;
    }
}

/**
 * Find whether a position is valid considering the drone size
 * @param position The position needs check
 * @return
 */
bool aStarPathPlanner::validPosWithDroneSize(glm::vec3 position) {
    float scale = droneSize;
    // ensure that the drone is good on 6 directions.
    while (scale > 0) {
        for (auto &dir : sixDirs) {
            glm::vec3 toExam = glm::vec3(position) + (glm::vec3(dir) * scale);
            if (model.checkValidPos(toExam)) {
                if (model.checkBlocked(toExam)) {
                    return false;
                }
            } else {
                return false;
            }
        }
        scale -= 1;
    }
    return true;
}

/**
 * After planPath, use getPath to get the result.
 * @return All the points in the planned path
 */
vector<glm::vec3> aStarPathPlanner::getPath() {
    vector<glm::vec3> result = vector<glm::vec3>();
    for (auto &ite : path) {
        result.emplace_back(glm::vec3(ite));
    }
    return result;
}

/**
 * Another version of getting the planned path.
 * @param carrier The container given by the user to get the path.
 */
void aStarPathPlanner::getPath(std::vector<glm::vec3> &carrier) {
    for (auto &ite : path) {
        carrier.emplace_back(glm::vec3(ite));
    }
}

/**
 * step length setter
 * @param length
 */
void aStarPathPlanner::setStepLength(float length) {
    if (length > 0) {
        stepLength = length;
    } else {
        cout << "step should be positive!" << endl;
        stepLength = 0.1;
    }
}

/**
 * Drone size setter
 * @param size
 */
void aStarPathPlanner::setDroneSize(float size) {
    aStarPathPlanner::droneSize = size;
}

/**
 * Model setter
 */
void aStarPathPlanner::setModel(model::threeDmodel &m) {
    model = m;
}


