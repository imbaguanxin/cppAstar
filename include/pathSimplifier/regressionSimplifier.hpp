//
// Created by imbaguanxin on 2019/10/10.
//

#ifndef ASTAR_CPP_REGRESSIONSIMPLIFIER_HPP
#define ASTAR_CPP_REGRESSIONSIMPLIFIER_HPP


#include <vector>
#include <glm/glm.hpp>

/*
 * a struct that stores a regression result: variance, average, max value
 */
struct regressionResult {
    float var, avg, maxDiff;
};

class regressionSimplifier {

private:
    // path after simplification
    std::vector<glm::vec3> simplifiedPath;
    // some parameters including drone size and restriction
    float droneSize;
    // This restriction is going to be effective when grow in exponential
    const float restriction = 7;

public:
    // drone size setter
    void setDroneSize(float dSize);

private:
    // find out the regression result of a list of point.
    static regressionResult lineExam(std::vector<glm::vec3> &oriPath, int start, int end);

    // check whether a regression result is valid.
    bool regResChecker(regressionResult &regRes);

    /**
     * Find out the distance between a point to a line
     * @param from Part of line representation: a point on that line
     * @param dir Part of line representation: direction of the line
     * @param point The point to check distance.
     * @return
     */
    static float pointToLineDistance(glm::vec3 &from, glm::vec3 &dir, glm::vec3 &point);

public:
    // default constructor
    regressionSimplifier();

    // constructor that specifies drone size.
    explicit regressionSimplifier(float dSize);

    // main method that simplify the path.
    std::vector<glm::vec3> simplify(std::vector<glm::vec3> &oriPath);

};


#endif //ASTAR_CPP_REGRESSIONSIMPLIFIER_HPP
