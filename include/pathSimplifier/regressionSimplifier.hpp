//
// Created by imbaguanxin on 2019/10/10.
//

#ifndef ASTAR_CPP_REGRESSIONSIMPLIFIER_HPP
#define ASTAR_CPP_REGRESSIONSIMPLIFIER_HPP


#include <vector>
#include <glm/glm.hpp>

struct regressionResult {
    float var, avg, maxDiff;
};

class regressionSimplifier {
private:
    std::vector<glm::vec3> simplifiedPath;
    float droneSize, restriction;

public:
    void setDroneSize(float dSize);

    void setRestriction(float restrict);

private:
    static regressionResult lineExame(std::vector<glm::vec3> &oriPath, int start, int end);

    bool regResChecker(regressionResult &regRes);


    static float pointToLineDistance(glm::vec3 &from, glm::vec3 &dir, glm::vec3 &point);

public:
    regressionSimplifier();

    regressionSimplifier(float dSize, float restrict);

    std::vector<glm::vec3> simplify(std::vector<glm::vec3> &oriPath);

};


#endif //ASTAR_CPP_REGRESSIONSIMPLIFIER_HPP
