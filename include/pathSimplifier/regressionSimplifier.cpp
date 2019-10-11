//
// Created by imbaguanxin on 2019/10/10.
//

#include <stdexcept>
#include <iostream>
#include <glm/ext.hpp>
#include "regressionSimplifier.hpp"


using namespace std;

regressionSimplifier::regressionSimplifier() :
        droneSize(0.0), restriction(7.0) {}

regressionSimplifier::regressionSimplifier(float dSize, float restrict) :
        droneSize(dSize), restriction(restrict) {}

void regressionSimplifier::setDroneSize(float dSize) {
    droneSize = dSize;
}

void regressionSimplifier::setRestriction(float restrict) {
    restriction = restrict;
}

regressionResult regressionSimplifier::lineExame(std::vector<glm::vec3> &oriPath, int start, int end) {
    glm::vec3 from = glm::vec3(oriPath.at(start));
    glm::vec3 dir = glm::vec3(oriPath.at(end)) - from;
    vector<float> distanceCollection = vector<float>();
    for (int i = start + 1; i < end; i++) {
        distanceCollection.emplace_back(pointToLineDistance(from, dir, oriPath.at(i)));
    }
    float var = 0, avg = 0, maxDiff = 0;
    if (distanceCollection.size() > 1) {
        float sum = 0;
        for (auto diff : distanceCollection) {
            if (diff > maxDiff) {
                maxDiff = diff;
            }
            sum += diff;
        }

        avg = sum / (float) distanceCollection.size();

        float varSum = 0;
        for (auto diff2 : distanceCollection) {
            varSum += pow(diff2 - avg, 2.0);
        }
        var = varSum / (float) distanceCollection.size();
    }
    return {var, avg, maxDiff};

}

bool regressionSimplifier::regResChecker(regressionResult &regRes) {
    return regRes.maxDiff < droneSize &&
           regRes.avg < droneSize / 2.0 &&
           regRes.var < droneSize / restriction;
}

float regressionSimplifier::pointToLineDistance(glm::vec3 &from, glm::vec3 &dir, glm::vec3 &point) {
    glm::vec3 fromToPoint = glm::vec3(point) - glm::vec3(from);
    glm::vec3 normFrom = glm::normalize(glm::vec3(fromToPoint));
    glm::vec3 normDir = glm::normalize(glm::vec3(dir));
    float cosTheta = glm::dot(normFrom, normDir);
    float sinTheta = sqrt(1 - cosTheta * cosTheta);
    return abs(glm::length(fromToPoint) * sinTheta);
}


std::vector<glm::vec3> regressionSimplifier::simplify(std::vector<glm::vec3> &oriPath) {
    if (oriPath.size() < 2) throw invalid_argument("nothing to simplify!");
    simplifiedPath = vector<glm::vec3>();
    simplifiedPath.emplace_back(glm::vec3(oriPath.at(0)));
    int start = 0;
    int end = 1;
    while (end < oriPath.size()) {
        regressionResult regRes = lineExame(oriPath, start, end);
        if (regResChecker(regRes)) {
            end += 1;
        } else {
            glm::vec3 add = glm::vec3(oriPath.at(end - 1));
            simplifiedPath.emplace_back(add);
            start = end - 1;
        }
    }
    simplifiedPath.emplace_back(glm::vec3(oriPath.at(oriPath.size() - 1)));
    return simplifiedPath;
}


