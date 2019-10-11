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

aStarPathPlanner::aStarPathPlanner(model::threeDmodel &m)
        : model(m), stepLength(0.1), droneSize(0) {
    path = vector<glm::vec3>();
    initDir();
}


void aStarPathPlanner::initDir() {
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

bool aStarPathPlanner::planPath(glm::vec3 fromP, glm::vec3 toP) {
    if (model.checkValidPos(fromP.x, fromP.y, fromP.z) &&
        model.checkValidPos(toP.x, toP.y, toP.z) &&
        !model.checkBlocked(fromP.x, fromP.y, fromP.z) &&
        !model.checkBlocked(toP.x, toP.y, toP.z)) {
        return astarPlan(glm::vec3(fromP), glm::vec3(toP));
    } else {
        throw invalid_argument("Invalid input points!");
    }
}


bool aStarPathPlanner::astarPlan(glm::vec3 fromP, glm::vec3 toP) {
    path = vector<glm::vec3>();
    unordered_map<glm::vec3, int> passedPoints = unordered_map<glm::vec3, int>();
    glm::vec3 tempStartPoint = glm::vec3(fromP);
    while (true) {
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
            return false;
        }
    }
}

glm::vec3 aStarPathPlanner::astarFindNext(glm::vec3 fromP, glm::vec3 toP, unordered_map<glm::vec3, int> &passed) {
    bool hasNextPointflag = false;
    glm::vec3 nextPoint;
    float minDis = numeric_limits<float>::max();
    for (auto const &dir: possibleDir) {
        glm::vec3 possibleNext = glm::vec3(fromP) + (glm::vec3(dir) * stepLength);
        if (glm::distance(possibleNext, toP) <= glm::length(dir) * stepLength) {
            return toP;
        }
        if (validPosWithDroneSize(possibleNext)) {
            float distanceNext = 0;
            auto mapSearch = passed.find(possibleNext);
            if (mapSearch != passed.end()) {
                distanceNext =
                        glm::distance(possibleNext, toP) * mapSearch->second + glm::length(dir) * stepLength;
            } else {
                distanceNext = glm::distance(possibleNext, toP) + glm::length(dir) * stepLength;
            }
            if (distanceNext < minDis) {
                minDis = distanceNext;
                nextPoint = possibleNext;
                hasNextPointflag = true;
            }
        }
    }
    if (!hasNextPointflag) {
        throw invalid_argument("not able to reach");
    } else {
        auto mapSearch = passed.find(nextPoint);
        if (mapSearch != passed.end()) {
            passed.emplace(nextPoint, mapSearch->second + 1);
        } else {
            passed.emplace(nextPoint, 1);
        }
        return nextPoint;
    }
}

bool aStarPathPlanner::validPosWithDroneSize(glm::vec3 position) {
    float scale = droneSize;
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

vector<glm::vec3> aStarPathPlanner::getPath() {
    vector<glm::vec3> result = vector<glm::vec3>();
    for (auto &ite : path) {
        result.emplace_back(glm::vec3(ite));
    }
    return result;
}

void aStarPathPlanner::getPath(std::vector<glm::vec3> &carrier) {
    for (auto &ite : path) {
        carrier.emplace_back(glm::vec3(ite));
    }
}

void aStarPathPlanner::setStepLength(float length) {
    if (length > 0) {
        stepLength = length;
    } else {
        cout << "step should be positive!" << endl;
        stepLength = 0.1;
    }
}

void aStarPathPlanner::setDroneSize(float size) {
    aStarPathPlanner::droneSize = size;
}


