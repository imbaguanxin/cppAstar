//
// Created by imbaguanxin on 2019/10/28.
//

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
    clearReferenceTable();
    // normalize
    vec3 tempFrom = vec3(floor(fromP.x), floor(fromP.y), floor(fromP.z));
    vec3 tempTo = vec3(floor(toP.x), floor(toP.y), floor(toP.z));
    startP = tempFrom;
    endP = tempTo;
    // set start point
    waitingListElement currPoint = waitingListElement(tempFrom, glm::distance(tempFrom, tempTo), 0.0);
    waitingList.emplace_back(currPoint);
    setBeenTo(startP);
    while (!waitingList.empty()) {
        currPoint = findLeastScorePoint();
        glm::vec3 currPosition = currPoint.getPosition();
        if (currPosition == endP) {
            return true;
        }
        if (glm::distance(currPosition, endP) < droneSize) {
            referenceTable.setFather(endP, currPosition);
            return true;
        } else {
            populateWaitingList(currPoint, endP);
        }
    }
    if (glm::distance(endP, currPoint.getPosition()) < droneSize) {
        referenceTable.setFather(endP, currPoint.getPosition());
        return true;
    } else {
        return false;
    }
}

// wipe out the waiting list and closed table before each path planning.
void aStar::clearReferenceTable() {
    // clear the waiting list
    waitingList = vector<waitingListElement>();
    // clear the reference table
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
    waitingListElement &result = waitingList.at(0);
    int deleteNum = 0;
    // iterate over all nodes in the list
    for (int i = 0; i < waitingList.size(); ++i) {
        waitingListElement &curr = waitingList.at(i);
        if (curr.score < lowestScore) {
            lowestScore = curr.score;
            result = curr;
            deleteNum = i;
        }
    }
    // Delete the found node in waiting list.
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
    if (model.checkValidPos(pos)) {
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
    glm::vec3 curr = endP;
    while (referenceTable.getFather(curr) != curr) {
        path.push_front(curr);
        curr = referenceTable.getFather(curr);
    }
    path.push_front(curr);
    return path;
}