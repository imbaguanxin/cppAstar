//
// Created by imbaguanxin on 2019/10/28.
//

#ifndef ASTAR_CPP_ASTAR_HPP
#define ASTAR_CPP_ASTAR_HPP

#include <glm/glm.hpp>
#include <list>

#include "model/threeDModel.h"
#include "vecNode.hpp"


class aStar {

private:

    model::threeDmodel model, closeTable;
    std::vector<vecNode *> waitingList;
    vecNode *startPoint, *currPoint;
    std::vector<glm::vec3> possibleDir;
    float droneSize;

public:

    explicit aStar(model::threeDmodel &m);

    void setModel(model::threeDmodel &m);

    void setDroneSize(float ds);

    bool aStarPathPlan(glm::vec3 fromP, glm::vec3 toP);

    std::list<glm::vec3> getPath();

private:

    void initDir();

    void clearWaitingAndClosedTable();

    static float calcScore(glm::vec3 fp, glm::vec3 nextStep, glm::vec3 tp);

    void populateWaitingList(vecNode *parent, glm::vec3 toP);

    vecNode *findLeastScorePoint();

    bool validPosition(glm::vec3 pos);

    bool haveBeenTo(glm::vec3 pos);

    void setBeenTo(glm::vec3 pos);

};


#endif //ASTAR_CPP_ASTAR_HPP
