//
// Created by imbaguanxin on 2019/10/28.
//

#ifndef ASTAR_CPP_ASTAR_HPP
#define ASTAR_CPP_ASTAR_HPP

#include <glm/glm.hpp>
#include <list>

#include "model/threeDModel.h"

class waitingListElement {

public:

    waitingListElement(int X, int Y, int Z, float Score, float H) :
            x(X), y(Y), z(Z), score(Score), h(H) {}

    waitingListElement(float X, float Y, float Z, float Score, float H) :
            x(std::floor(X)), y(std::floor(Y)), z(std::floor(Z)), score(Score), h(H) {}

    waitingListElement(glm::vec3 pt, float Score, float H) :
            x(std::floor(pt.x)), y(std::floor(pt.y)), z(std::floor(pt.z)), score(Score), h(H) {}

    glm::vec3 getPosition() {
        return glm::vec3((float)x, (float)y, (float)z);
    }

    void printInfo() {
        std::cout << "(" << x << ", " << y << ", " << z << "), " << score << ", " << h << std::endl;
    }

    int x;
    int y;
    int z;
    float score;
    float h;

};

class aStar {

public:

    // model: the 3D map, closeTable: the table memorize where the algorithm have searched
    model::threeDmodel model, referenceTable;
    // vecNodes that needs check
    std::vector<waitingListElement> waitingList;
    // 26 possible directions that the drone might go.
    std::vector<glm::vec3> possibleDir;
    glm::vec3 startP, endP;
    // the size of the drone in terms of the grid size (drone 20 cm, grid size 10 cm => drone size = 2)
    float droneSize;

public:

    /**
     * constructor with a 3D model
     * @param m  The 3D model
     */
    explicit aStar(model::threeDmodel &m);


    /**
    * model setter
    * @param m  The 3D model
    */
    void setModel(model::threeDmodel &m);


    /**
     * Drone size setter
     * @param ds
     */
    void setDroneSize(float ds);

    /**
     * path planner
     * @param fromP  The starting point, in terms of grid unit
     * @param toP  The end point, in terms of grid unit
     * @return Whether the algorithm finished successfully
     */
    bool aStarPathPlan(glm::vec3 fromP, glm::vec3 toP);

    /**
     * After running the algorithm, call getPath() to get the result of astar planning
     * @return a list of points in 3D, in terms of grid unit.
     */
    std::list<glm::vec3> getPath();


private:

    // initialize the 26 directions in 3D
    void initDir();

    // wipe out the waiting list and closed table before each path planning.
    void clearReferenceTable();

    /**
    * Register a position as searched position.
    */
    void setBeenTo(glm::vec3 pos);

    /**
     * Find the node with the lowest score in waiting list.
     */
    waitingListElement findLeastScorePoint();

    /**
     * Populate the waiting list with all possible steps.
     * @param parent  The parent node
     * @param toP  The destination
     */
    void populateWaitingList(waitingListElement parent, glm::vec3 toP);

    /**
     * Find whether a position is available to reach in the map model.
     * @param pos The position needs test.
     */
    bool reachableAndHaventBeenTo(glm::vec3 pos);

    void printWaitingList(){
        std::cout << "waitingList size:" << waitingList.size() << std::endl;
        for (auto & i : waitingList) {
            i.printInfo();
        }
    }
};

#endif //ASTAR_CPP_ASTAR_HPP
