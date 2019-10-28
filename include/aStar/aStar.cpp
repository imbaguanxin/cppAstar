//
// Created by imbaguanxin on 2019/10/28.
//

#include <iostream>
#include "aStar.hpp"

using namespace std;

/**
 * constructor with a 3D model
 * @param m  The 3D model
 */
aStar::aStar(model::threeDmodel &m) :
        model(m),
        closeTable(model::threeDmodel(m.getXlength(), m.getYlength(), m.getZlength())),
        droneSize(1.0),
        startPoint(nullptr), currPoint(nullptr) {
    initDir();
}

/**
 * model setter
 * @param m  The 3D model
 */
void aStar::setModel(model::threeDmodel &m) {
    model = m;
}

/**
 * Drone size setter
 * @param ds
 */
void aStar::setDroneSize(float ds) {
    droneSize = ds;
}

/**
 * path planner
 * @param fromP  The starting point, in terms of grid unit
 * @param toP  The end point, in terms of grid unit
 * @return Whether the algorithm finished successfully
 */
bool aStar::aStarPathPlan(glm::vec3 fromP, glm::vec3 toP) {
    clearWaitingAndClosedTable();
    startPoint = new vecNode(glm::distance(fromP, toP), glm::vec3(fromP), nullptr);
    currPoint = startPoint;
    waitingList.emplace_back(startPoint);
    setBeenTo(fromP);
    while (!waitingList.empty()) {
        currPoint = findLeastScorePoint();
        if (currPoint != nullptr) {
            glm::vec3 point = currPoint->getPos();
            if (glm::distance(point, toP) < droneSize) {
                currPoint = new vecNode(0, toP, currPoint);
                return true;
            } else {
                populateWaitingList(currPoint, toP);
            }
        } else {
            currPoint = startPoint;
            return false;
        }
    }
    if (glm::distance(toP, currPoint->getPos()) < droneSize) {
        currPoint = new vecNode(0, toP, currPoint);
        return true;
    } else {
        return false;
    }
}

/**
 * After running the algorithm, call getPath() to get the result of astar planning
 * @return a list of points in 3D, in terms of grid unit.
 */
std::list<glm::vec3> aStar::getPath() {
    list<glm::vec3> path = list<glm::vec3>();
    vecNode *result = currPoint;
    while (result) {
        path.push_front(result->getPos());
        result = result->getParent();
    }
    return path;
}

// initialize the 26 directions in 3D
void aStar::initDir() {
//    ====================================================
    possibleDir = vector<glm::vec3>();
    possibleDir.emplace_back(glm::vec3(0, 0, 1));
    possibleDir.emplace_back(glm::vec3(0, 0, -1));
    possibleDir.emplace_back(glm::vec3(0, 1, 0));
    possibleDir.emplace_back(glm::vec3(0, -1, 0));
    possibleDir.emplace_back(glm::vec3(1, 0, 0));
    possibleDir.emplace_back(glm::vec3(-1, 0, 0));
//    =============================================
    possibleDir.emplace_back(glm::vec3(0, 1, 1));
    possibleDir.emplace_back(glm::vec3(0, 1, -1));
    possibleDir.emplace_back(glm::vec3(0, -1, 1));
    possibleDir.emplace_back(glm::vec3(0, -1, -1));
    possibleDir.emplace_back(glm::vec3(1, 1, 0));
    possibleDir.emplace_back(glm::vec3(-1, 1, 0));
    possibleDir.emplace_back(glm::vec3(1, -1, 0));
    possibleDir.emplace_back(glm::vec3(-1, -1, 0));
    possibleDir.emplace_back(glm::vec3(1, 0, 1));
    possibleDir.emplace_back(glm::vec3(-1, 0, 1));
    possibleDir.emplace_back(glm::vec3(1, 0, -1));
    possibleDir.emplace_back(glm::vec3(-1, 0, -1));
//    ==============================================
    possibleDir.emplace_back(glm::vec3(1, 1, 1));
    possibleDir.emplace_back(glm::vec3(-1, 1, 1));
    possibleDir.emplace_back(glm::vec3(1, 1, -1));
    possibleDir.emplace_back(glm::vec3(1, -1, 1));
    possibleDir.emplace_back(glm::vec3(-1, -1, 1));
    possibleDir.emplace_back(glm::vec3(1, -1, -1));
    possibleDir.emplace_back(glm::vec3(-1, 1, -1));
    possibleDir.emplace_back(glm::vec3(-1, -1, -1));
}

// wipe out the waiting list and closed table before each path planning.
void aStar::clearWaitingAndClosedTable() {
    // clear the waiting list
    waitingList = vector<vecNode *>();
    // clear the closed table
    closeTable = model::threeDmodel(model.getXlength(), model.getYlength(), model.getZlength());
}

/**
 * calculate the score (f = s + h)
 * s = distance bewteen from point (fp) and next step (nextStep)
 * h = distance between next step (nextStep) and destination (tp)
 */
float aStar::calcScore(glm::vec3 fp, glm::vec3 nextStep, glm::vec3 tp) {
    return glm::distance(fp, nextStep) + glm::distance(nextStep, tp);
}


/**
 * Find the node with the lowest score in waiting list.
 */
vecNode *aStar::findLeastScorePoint() {
    float lowestScore = numeric_limits<float>::max();
    vecNode *result = nullptr;
    int deleteNum = -1;
    // iterate over all nodes in the list
    for (int i = 0; i < waitingList.size(); ++i) {
        vecNode *node = waitingList.at(i);
        if (node->getDistance() < lowestScore) {
            lowestScore = node->getDistance();
            result = node;
            deleteNum = i;
        }
    }
    // Delete the found node in waiting list.
    if (deleteNum > -1) waitingList.erase(waitingList.begin() + deleteNum);
    return result;
}

/**
 * Find whether a position is available to reach in the map model.
 * @param pos The position needs test.
 */
bool aStar::validPosition(glm::vec3 pos) {
    float scale = droneSize;
    // ensure that the drone is good on 26 directions.
    while (scale > 0) {
        for (auto &dir : possibleDir) {
            glm::vec3 toExam = glm::vec3(pos) + (glm::vec3(dir) / glm::length(dir) * scale);
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
 * Find whether the algorithm have already searched the position.
 * @param pos The position needs test.
 */
bool aStar::haveBeenTo(glm::vec3 pos) {
    return closeTable.checkBlocked(pos);
}

/**
 * Register a position as searched position.
 */
void aStar::setBeenTo(glm::vec3 pos) {
    closeTable.setGrid(pos.x, pos.y, pos.z, true);
}

/**
 * Populate the waiting list with all possible steps.
 * @param parent  The parent node
 * @param toP  The destination
 */
void aStar::populateWaitingList(vecNode *parent, glm::vec3 toP) {
    glm::vec3 fp = glm::vec3(parent->getPos());
    if (glm::distance(fp, toP) < droneSize) {
        waitingList.emplace_back((new vecNode(glm::distance(fp, toP), toP, parent)));
    } else {
        // Iterate over all possible directions and place them in waiting list
        for (auto &dir : possibleDir) {
            glm::vec3 possibleNext = glm::vec3(fp) + glm::vec3(dir);
            if (validPosition(possibleNext) && !haveBeenTo(possibleNext)) {
                setBeenTo(possibleNext);
                waitingList.emplace_back(new vecNode(calcScore(fp, possibleNext, toP), possibleNext, parent));
            }
        }
    }
}




