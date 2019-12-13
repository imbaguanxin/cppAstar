//
// Created by imbaguanxin on 2019/10/28.
//

#include <iostream>
#include "aStar.hpp"

using namespace std;
using namespace glm;

/**
 * constructor with a 3D model
 * @param m  The 3D model
 */
aStar::aStar(model::threeDmodel &m) :
        model(m),
        referenceTable(model::threeDmodel(m.getXlength(), m.getYlength(), m.getZlength())),
        droneSize(1.0),
        waitingList(vector<waitingListElement>()) {
    initDir();
}

// initialize the 26 directions in 3D
void aStar::initDir() {
//    ====================================================
    possibleDir = vector<vec3>();
    possibleDir.emplace_back(vec3(0, 0, 1));
    possibleDir.emplace_back(vec3(0, 0, -1));
    possibleDir.emplace_back(vec3(0, 1, 0));
    possibleDir.emplace_back(vec3(0, -1, 0));
    possibleDir.emplace_back(vec3(1, 0, 0));
    possibleDir.emplace_back(vec3(-1, 0, 0));
//    =============================================
    possibleDir.emplace_back(vec3(0, 1, 1));
    possibleDir.emplace_back(vec3(0, 1, -1));
    possibleDir.emplace_back(vec3(0, -1, 1));
    possibleDir.emplace_back(vec3(0, -1, -1));
    possibleDir.emplace_back(vec3(1, 1, 0));
    possibleDir.emplace_back(vec3(-1, 1, 0));
    possibleDir.emplace_back(vec3(1, -1, 0));
    possibleDir.emplace_back(vec3(-1, -1, 0));
    possibleDir.emplace_back(vec3(1, 0, 1));
    possibleDir.emplace_back(vec3(-1, 0, 1));
    possibleDir.emplace_back(vec3(1, 0, -1));
    possibleDir.emplace_back(vec3(-1, 0, -1));
//    ==============================================
    possibleDir.emplace_back(vec3(1, 1, 1));
    possibleDir.emplace_back(vec3(-1, 1, 1));
    possibleDir.emplace_back(vec3(1, 1, -1));
    possibleDir.emplace_back(vec3(1, -1, 1));
    possibleDir.emplace_back(vec3(-1, -1, 1));
    possibleDir.emplace_back(vec3(1, -1, -1));
    possibleDir.emplace_back(vec3(-1, 1, -1));
    possibleDir.emplace_back(vec3(-1, -1, -1));
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

bool aStar::aStarPathPlan(glm::vec3 fromP, glm::vec3 toP) {
    startP = fromP;
    endP = toP;
    clearReferenceTable();
    // normalize
    vec3 tempFrom = vec3(floor(fromP.x), floor(fromP.y), floor(fromP.z));
    vec3 tempTo = vec3(floor(toP.x), floor(toP.y), floor(toP.z));
    // set start point
    waitingListElement startPoint = waitingListElement(tempFrom, glm::distance(tempFrom, tempTo), 0.0);
    // set currentPoint
    waitingListElement currPoint = startPoint;
    waitingList.emplace_back(startPoint);
//    printWaitingList();
    setBeenTo(fromP);
    while (!waitingList.empty()) {
        currPoint = findLeastScorePoint();
        glm::vec3 currPosition = currPoint.getPosition();
        setBeenTo(currPosition);
        if (glm::distance(currPosition, toP) < droneSize) {
            referenceTable.setFather(toP, currPosition);
            return true;
        } else {
            populateWaitingList(currPoint, toP);
        }
//        printWaitingList();
    }
    if (glm::distance(toP, currPoint.getPosition()) < droneSize) {
        referenceTable.setFather(toP, currPoint.getPosition());
        return true;
    } else {
        return false;
    }
}

// wipe out the waiting list and closed table before each path planning.
void aStar::clearReferenceTable() {
    // clear the waiting list
    waitingList = vector<waitingListElement>();
    // clear the closed table
    referenceTable = model::threeDmodel(model.getXlength(), model.getYlength(), model.getZlength());
}

/**
 * Register a position as searched position.
 */
void aStar::setBeenTo(glm::vec3 pos) {
    referenceTable.setGrid(pos.x, pos.y, pos.z, THREE_D_GRID_SEARCHED);
}

/**
 * Find the node with the lowest score in waiting list.
 */
waitingListElement aStar::findLeastScorePoint() {
    float lowestScore = waitingList.at(0).score;
    waitingListElement result = waitingList.at(0);
    int deleteNum = 0;
    // iterate over all nodes in the list
    for (int i = 0; i < waitingList.size(); ++i) {
        waitingListElement curr = waitingList.at(i);
        if (curr.score < lowestScore) {
            lowestScore = curr.score;
            result = curr;
            deleteNum = i;
        }
    }
    // Delete the found node in waiting list.
    cout << "delete: " << deleteNum << endl;
    if (deleteNum > -1) {

        waitingList.erase(waitingList.begin() + deleteNum);
    }
    return result;
}

/**
 * Populate the waiting list with all possible steps.
 * @param parent  The parent node
 * @param toP  The destination
 */
void aStar::populateWaitingList(waitingListElement parent, glm::vec3 toP) {
    glm::vec3 fp = parent.getPosition();
    if (glm::distance(fp, toP) < droneSize) {
//        float h = parent.h + distance(fp, toP);
//        float score = h;
        referenceTable.setFather(toP, fp);
        waitingList.emplace_back(waitingListElement(toP, 0, 0));
    } else {
        // Iterate over all possible directions and place them in waiting list
        for (auto &dir : possibleDir) {
            glm::vec3 possibleNext = glm::vec3(fp) + glm::vec3(dir);
            if (reachableAndHaventBeenTo(possibleNext)) {
                setBeenTo(possibleNext);
                float h = parent.h + distance(fp, possibleNext);
                float score = h + distance(possibleNext, toP);
                referenceTable.setFather(possibleNext, fp);
                waitingList.emplace_back(waitingListElement(possibleNext, score, h));
            }
        }
    }
}

/**
 * Find whether a position is available to reach in the map model.
 * @param pos The position needs test.
 */
bool aStar::reachableAndHaventBeenTo(glm::vec3 pos) {
    if (referenceTable.checkStatus(pos) == THREE_D_GRID_SEARCHED) return false;
    float scale = droneSize;
    // ensure that the drone is good on 26 directions.
    while (scale > 0) {
        for (auto &dir : possibleDir) {
            glm::vec3 toExam = glm::vec3(pos) + (glm::vec3(dir) / glm::length(dir) * scale);
            if (model.checkValidPos(toExam)) {
                if (model.checkStatus(toExam) != THREE_D_GRID_EMPTY) {
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
 * After running the algorithm, call getPath() to get the result of astar planning
 * @return a list of points in 3D, in terms of grid unit.
 */
std::list<glm::vec3> aStar::getPath() {
    list<glm::vec3> path = list<glm::vec3>();
    glm::vec3 curr = endP;
    while (referenceTable.getFather(curr) != curr) {
        path.push_front(curr);
        curr = referenceTable.getFather(curr);
    }
    return path;
}

//
///**
// * model setter
// * @param m  The 3D model
// */
//void aStar::setModel(model::threeDmodel &m) {
//    model = m;
//}
//
///**
// * Drone size setter
// * @param ds
// */
//void aStar::setDroneSize(float ds) {
//    droneSize = ds;
//}
//
///**
// * path planner
// * @param fromP  The starting point, in terms of grid unit
// * @param toP  The end point, in terms of grid unit
// * @return Whether the algorithm finished successfully
// */
//bool aStar::aStarPathPlan(glm::vec3 fromP, glm::vec3 toP) {
//    clearWaitingAndClosedTable();
//    startPoint = new vecNode(glm::distance(fromP, toP), glm::vec3(fromP), nullptr);
//    currPoint = startPoint;
//    waitingList.emplace_back(startPoint);
//    setBeenTo(fromP);
//    while (!waitingList.empty()) {
//        currPoint = findLeastScorePoint();
//        if (currPoint != nullptr) {
//            glm::vec3 point = currPoint->getPos();
//            if (glm::distance(point, toP) < droneSize) {
//                currPoint = new vecNode(0, toP, currPoint);
//                return true;
//            } else {
//                populateWaitingList(currPoint, toP);
//            }
//        } else {
//            currPoint = startPoint;
//            return false;
//        }
//    }
//    if (glm::distance(toP, currPoint->getPos()) < droneSize) {
//        currPoint = new vecNode(0, toP, currPoint);
//        return true;
//    } else {
//        return false;
//    }
//}
//
///**
// * After running the algorithm, call getPath() to get the result of astar planning
// * @return a list of points in 3D, in terms of grid unit.
// */
//std::list<glm::vec3> aStar::getPath() {
//    list<glm::vec3> path = list<glm::vec3>();
//    vecNode *result = currPoint;
//    while (result) {
//        path.push_front(result->getPos());
//        result = result->getParent();
//    }
//    return path;
//}
//
//// initialize the 26 directions in 3D
//void aStar::initDir() {
////    ====================================================
//    possibleDir = vector<glm::vec3>();
//    possibleDir.emplace_back(glm::vec3(0, 0, 1));
//    possibleDir.emplace_back(glm::vec3(0, 0, -1));
//    possibleDir.emplace_back(glm::vec3(0, 1, 0));
//    possibleDir.emplace_back(glm::vec3(0, -1, 0));
//    possibleDir.emplace_back(glm::vec3(1, 0, 0));
//    possibleDir.emplace_back(glm::vec3(-1, 0, 0));
////    =============================================
//    possibleDir.emplace_back(glm::vec3(0, 1, 1));
//    possibleDir.emplace_back(glm::vec3(0, 1, -1));
//    possibleDir.emplace_back(glm::vec3(0, -1, 1));
//    possibleDir.emplace_back(glm::vec3(0, -1, -1));
//    possibleDir.emplace_back(glm::vec3(1, 1, 0));
//    possibleDir.emplace_back(glm::vec3(-1, 1, 0));
//    possibleDir.emplace_back(glm::vec3(1, -1, 0));
//    possibleDir.emplace_back(glm::vec3(-1, -1, 0));
//    possibleDir.emplace_back(glm::vec3(1, 0, 1));
//    possibleDir.emplace_back(glm::vec3(-1, 0, 1));
//    possibleDir.emplace_back(glm::vec3(1, 0, -1));
//    possibleDir.emplace_back(glm::vec3(-1, 0, -1));
////    ==============================================
//    possibleDir.emplace_back(glm::vec3(1, 1, 1));
//    possibleDir.emplace_back(glm::vec3(-1, 1, 1));
//    possibleDir.emplace_back(glm::vec3(1, 1, -1));
//    possibleDir.emplace_back(glm::vec3(1, -1, 1));
//    possibleDir.emplace_back(glm::vec3(-1, -1, 1));
//    possibleDir.emplace_back(glm::vec3(1, -1, -1));
//    possibleDir.emplace_back(glm::vec3(-1, 1, -1));
//    possibleDir.emplace_back(glm::vec3(-1, -1, -1));
//}
//
//// wipe out the waiting list and closed table before each path planning.
//void aStar::clearWaitingAndClosedTable() {
//    // clear the waiting list
//    waitingList = vector<vecNode *>();
//    // clear the closed table
//    closeTable = model::threeDmodel(model.getXlength(), model.getYlength(), model.getZlength());
//}
//
///**
// * calculate the score (f = s + h)
// * s = distance bewteen from point (fp) and next step (nextStep)
// * h = distance between next step (nextStep) and destination (tp)
// */
//float aStar::calcScore(glm::vec3 fp, glm::vec3 nextStep, glm::vec3 tp) {
//    return glm::distance(fp, nextStep) + glm::distance(nextStep, tp);
//}
//
//
///**
// * Find the node with the lowest score in waiting list.
// */
//vecNode *aStar::findLeastScorePoint() {
//    float lowestScore = numeric_limits<float>::max();
//    vecNode *result = nullptr;
//    int deleteNum = -1;
//    // iterate over all nodes in the list
//    for (int i = 0; i < waitingList.size(); ++i) {
//        vecNode *node = waitingList.at(i);
//        if (node->getDistance() < lowestScore) {
//            lowestScore = node->getDistance();
//            result = node;
//            deleteNum = i;
//        }
//    }
//    // Delete the found node in waiting list.
//    if (deleteNum > -1) waitingList.erase(waitingList.begin() + deleteNum);
//    return result;
//}
//
///**
// * Find whether a position is available to reach in the map model.
// * @param pos The position needs test.
// */
//bool aStar::validPosition(glm::vec3 pos) {
//    float scale = droneSize;
//    // ensure that the drone is good on 26 directions.
//    while (scale > 0) {
//        for (auto &dir : possibleDir) {
//            glm::vec3 toExam = glm::vec3(pos) + (glm::vec3(dir) / glm::length(dir) * scale);
//            if (model.checkValidPos(toExam)) {
//                if (model.checkBlocked(toExam)) {
//                    return false;
//                }
//            } else {
//                return false;
//            }
//        }
//        scale -= 1;
//    }
//    return true;
//}
//
///**
// * Find whether the algorithm have already searched the position.
// * @param pos The position needs test.
// */
//bool aStar::haveBeenTo(glm::vec3 pos) {
//    return closeTable.checkBlocked(pos);
//}
//
///**
// * Register a position as searched position.
// */
//void aStar::setBeenTo(glm::vec3 pos) {
//    closeTable.setGrid(pos.x, pos.y, pos.z, true);
//}
//
///**
// * Populate the waiting list with all possible steps.
// * @param parent  The parent node
// * @param toP  The destination
// */
//void aStar::populateWaitingList(vecNode *parent, glm::vec3 toP) {
//    glm::vec3 fp = glm::vec3(parent->getPos());
//    if (glm::distance(fp, toP) < droneSize) {
//        waitingList.emplace_back((new vecNode(glm::distance(fp, toP), toP, parent)));
//    } else {
//        // Iterate over all possible directions and place them in waiting list
//        for (auto &dir : possibleDir) {
//            glm::vec3 possibleNext = glm::vec3(fp) + glm::vec3(dir);
//            if (validPosition(possibleNext) && !haveBeenTo(possibleNext)) {
//                setBeenTo(possibleNext);
//                waitingList.emplace_back(new vecNode(calcScore(fp, possibleNext, toP), possibleNext, parent));
//            }
//        }
//    }
//}




