//
// Created by imbaguanxin on 2019/10/10.
//

#include <stdexcept>
#include <glm/ext.hpp>
#include "regressionSimplifier.hpp"


using namespace std;

regressionSimplifier::regressionSimplifier() :
        droneSize(0.0), restriction(7.0) {}

regressionSimplifier::regressionSimplifier(float dSize) :
        droneSize(dSize){}

// Drone Size setter
void regressionSimplifier::setDroneSize(float dSize) {
    droneSize = dSize;
}

/**
 * find out the regression result of a list of point.
 * @param oriPath Original path that needs simplification
 * @param start The first point that needs to test in the original path
 * @param end The last point that needs to test in the original path
 * @return the regression result.
 */
regressionResult regressionSimplifier::lineExam(std::vector<glm::vec3> &oriPath, int start, int end) {
    glm::vec3 from = glm::vec3(oriPath.at(start));
    glm::vec3 dir = glm::vec3(oriPath.at(end)) - from;
    vector<float> distanceCollection = vector<float>();
    for (int i = start + 1; i < end; i++) {
        distanceCollection.emplace_back(pointToLineDistance(from, dir, oriPath.at(i)));
    }
    float var = 0, avg = 0, maxDiff = 0;
    if (distanceCollection.size() > 1) {
        // Find average and maximum value
        float sum = 0;
        for (auto diff : distanceCollection) {
            if (diff > maxDiff) {
                maxDiff = diff;
            }
            sum += diff;
        }

        avg = sum / (float) distanceCollection.size();

        // Find the variance
        float varSum = 0;
        for (auto diff2 : distanceCollection) {
            varSum += pow(diff2 - avg, 2.0);
        }
        var = varSum / (float) distanceCollection.size();
    }
    return {var, avg, maxDiff};

}

// Check whether the regression result is good enough.
bool regressionSimplifier::regResChecker(regressionResult &regRes) {
    return regRes.maxDiff < droneSize &&
           regRes.avg < droneSize / 2.0 &&
           regRes.var < droneSize / restriction;
}

/**
 * Find out the distance between a point to a line
 * @param from Part of line representation: a point on that line
 * @param dir Part of line representation: direction of the line
 * @param point The point to check distance.
 * @return the distance
 */
float regressionSimplifier::pointToLineDistance(glm::vec3 &from, glm::vec3 &dir, glm::vec3 &point) {
    glm::vec3 fromToPoint = glm::vec3(point) - glm::vec3(from);
    glm::vec3 normFrom = glm::normalize(glm::vec3(fromToPoint));
    glm::vec3 normDir = glm::normalize(glm::vec3(dir));
    float cosTheta = glm::dot(normFrom, normDir);
    float sinTheta = sqrt(1 - cosTheta * cosTheta);
    return abs(glm::length(fromToPoint) * sinTheta);
}

/**
 * main method that simplifies the path.
 * @param oriPath original path that needs to simplify.
 * @return
 */
std::vector<glm::vec3> regressionSimplifier::simplify(std::vector<glm::vec3> &oriPath) {
    if (oriPath.size() < 2) throw invalid_argument("nothing to simplify!");
    simplifiedPath = vector<glm::vec3>();
    simplifiedPath.emplace_back(glm::vec3(oriPath.at(0)));
    int start = 0;
    int end = 1;
    while (end < oriPath.size()) {
        regressionResult regRes = lineExam(oriPath, start, end);
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


