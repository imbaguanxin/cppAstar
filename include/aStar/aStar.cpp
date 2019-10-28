//
// Created by imbaguanxin on 2019/10/28.
//

#include <iostream>
#include "aStar.hpp"

using namespace std;

aStar::aStar(model::threeDmodel &m) :
        model(m),
        closeTable(model::threeDmodel(m.getXlength(), m.getYlength(), m.getZlength())),
        droneSize(1.0),
        startPoint(nullptr), currPoint(nullptr) {
    initDir();
}

void aStar::setModel(model::threeDmodel &m) {
    model = m;
}

void aStar::setDroneSize(float ds) {
    droneSize = ds;
}

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

std::list<glm::vec3> aStar::getPath() {
    list<glm::vec3> path = list<glm::vec3>();
    vecNode *result = currPoint;
    while (result) {
        path.push_front(result->getPos());
        result = result->getParent();
    }
    return path;
}

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

void aStar::clearWaitingAndClosedTable() {
    waitingList = vector<vecNode *>();
    closeTable = model::threeDmodel(model.getXlength(), model.getYlength(), model.getZlength());
}

float aStar::calcScore(glm::vec3 fp, glm::vec3 nextStep, glm::vec3 tp) {
    return glm::distance(fp, nextStep) + glm::distance(nextStep, tp);
}

vecNode *aStar::findLeastScorePoint() {
    float lowestScore = numeric_limits<float>::max();
    vecNode *result = nullptr;
    int deleteNum = -1;
    for (int i = 0; i < waitingList.size(); ++i) {
        vecNode *node = waitingList.at(i);
        if (node->getDistance() < lowestScore) {
            lowestScore = node->getDistance();
            result = node;
            deleteNum = i;
        }
    }
    if (deleteNum > -1) waitingList.erase(waitingList.begin() + deleteNum);
    return result;
}

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

bool aStar::haveBeenTo(glm::vec3 pos) {
    return closeTable.checkBlocked(pos);
}

void aStar::setBeenTo(glm::vec3 pos) {
    closeTable.setGrid(pos.x, pos.y, pos.z, true);
}

void aStar::populateWaitingList(vecNode *parent, glm::vec3 toP) {
    glm::vec3 fp = glm::vec3(parent->getPos());
    if (glm::distance(fp, toP) < droneSize) {
        waitingList.emplace_back((new vecNode(glm::distance(fp, toP), toP, parent)));
    } else {
        for (auto &dir : possibleDir) {
            glm::vec3 possibleNext = glm::vec3(fp) + glm::vec3(dir);
            if (validPosition(possibleNext) && !haveBeenTo(possibleNext)) {
                setBeenTo(possibleNext);
                waitingList.emplace_back(new vecNode(calcScore(fp, possibleNext, toP), possibleNext, parent));
            }
        }
    }
}




