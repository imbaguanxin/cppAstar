//
// Created by 11655 on 2019/9/27.
//

#include <iostream>
#include <math.h>
#include "aStarPathPlanner.h"
#include <limits>

using namespace std;

aStarPathPlanner::aStarPathPlanner(model::threeDmodel &m)
        : model(m), step(1.0) {
    path = vector<glm::vec3>();
    possibleDir = vector<glm::vec3>();
    initDir();
}

void aStarPathPlanner::setStep(float s) {
    if (s > 0) {
        step = s;
    } else {
        cout << "step should be positive!" << endl;
        step = 1;
    }
}

void aStarPathPlanner::initDir() {
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
    map<glm::vec3, int> passedPoints = map<glm::vec3, int>();
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

glm::vec3 aStarPathPlanner::astarFindNext(glm::vec3 fromP, glm::vec3 toP, std::map<glm::vec3, int> &passed) {
    bool hasNextPointflag = false;
    glm::vec3 nextPoint;
    float minDis = numeric_limits<float>::max();
    for (auto const &dir: possibleDir) {
        glm::vec3 possibleNext = glm::vec3(fromP) + (glm::vec3(dir) * (step));
        if (distanceVec3(possibleNext, toP) <= lengthVec3(dir) * step) {
            return toP;
        }
        if (model.checkValidPos(possibleNext.x, possibleNext.y, possibleNext.z) &&
            !model.checkBlocked(possibleNext.x, possibleNext.y, possibleNext.z)) {
            float distanceNext = passed.count(possibleNext) == 1 ?
                                 distanceVec3(possibleNext, toP) * passed[possibleNext] +
                                 lengthVec3(dir) * step :
                                 distanceVec3(possibleNext, toP) + lengthVec3(dir) * step;
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
        if (passed.count(nextPoint) > 0) {
            passed.emplace(&nextPoint, passed[nextPoint] + 1);
        } else {
            passed.emplace(&nextPoint, 1);
        }
        return nextPoint;
    }
}

float aStarPathPlanner::distanceVec3(glm::vec3 const v1, glm::vec3 const v2) {
    return sqrtf(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2) + pow(v1.z - v2.z, 2));
}

float aStarPathPlanner::lengthVec3(glm::vec3 const v) {
    return sqrtf(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}
