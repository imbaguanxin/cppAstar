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

    // model: the 3D map, closeTable: the table memorize where the algorithm have searched
    model::threeDmodel model, closeTable;
    // vecNodes that needs check
    std::vector<vecNode *> waitingList;
    // startPoint: the starting point of the linked list that stores the path
    // currPoint: the last point of the linked list that stores the path, which grows as algorithm implements.
    vecNode *startPoint, *currPoint;
    // 26 possible directions that the drone may go.
    std::vector<glm::vec3> possibleDir;
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
    void clearWaitingAndClosedTable();

    /**
     * calculate the score (f = s + h)
     * s = distance bewteen from point (fp) and next step (nextStep)
     * h = distance between next step (nextStep) and destination (tp)
     */
    static float calcScore(glm::vec3 fp, glm::vec3 nextStep, glm::vec3 tp);

    /**
     * Populate the waiting list with all possible steps.
     * @param parent  The parent node
     * @param toP  The destination
     */
    void populateWaitingList(vecNode *parent, glm::vec3 toP);

    /**
     * Find the node with the lowest score in waiting list.
     */
    vecNode *findLeastScorePoint();

    /**
     * Find whether a position is available to reach in the map model.
     * @param pos The position needs test.
     */
    bool validPosition(glm::vec3 pos);

    /**
     * Find whether the algorithm have already searched the position.
     * @param pos The position needs test.
     */
    bool haveBeenTo(glm::vec3 pos);

    /**
     * Register a position as searched position.
     */
    void setBeenTo(glm::vec3 pos);

};


#endif //ASTAR_CPP_ASTAR_HPP
