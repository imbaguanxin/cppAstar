//
// Created by imbaguanxin on 2019/9/27.
//



#include <model/threeDModel.h>
#include <map>

#include "glm/vec3.hpp"

class aStarPathPlanner {

private:

    model::threeDmodel model;

    float step;

    std::vector<glm::vec3> possibleDir, path;

    bool astarPlan(glm::vec3 fromP, glm::vec3 toP);

    glm::vec3 astarFindNext(glm::vec3 fromP, glm::vec3 toP, std::map<glm::vec3, int> &passed);

public:

    explicit aStarPathPlanner(model::threeDmodel &m);

    void initDir();

    void setStep(float s);

    bool planPath(glm::vec3 fromP, glm::vec3 toP);
};

