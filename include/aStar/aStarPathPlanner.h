//
// Created by imbaguanxin on 2019/9/27.
//

#ifndef ASTAR_CPP_ASTARPATHPLANNER_HPP
#define ASTAR_CPP_ASTARPATHPLANNER_HPP

#include <model/threeDModel.h>
#include <unordered_map>

#include "glm/vec3.hpp"

class aStarPathPlanner {

private:
    /*
     * stepLength: the Length of A* goes in each search step
     * droneSize: the size of the drone
     */
    float stepLength, droneSize;

public:
    // stepLength setter
    void setStepLength(float length);

    // droneSize setter
    void setDroneSize(float size);

private:
    // map model
    model::threeDmodel model;

    /*
     * possibleDir: possible directions that A* is going to search. In this implementation: 26 directions possible.
     * path: A field that stores the final path
     * sixDirs: up, down, forward, backward, left right
     */
    std::vector<glm::vec3> possibleDir, path, sixDirs;

    /**
     * Find whether a position is valid considering the drone size
     * @param position The position needs check
     * @return
     */
    bool validPosWithDroneSize(glm::vec3 position);

    /**
     * The actual A* implementation after position checking
     * @param fromP The starting point of the path
     * @param toP The end point of the path
     * @return Whether the path was successfully planned
     */
    bool astarPlan(glm::vec3 fromP, glm::vec3 toP);

    /**
     * Find where to go in the next step
     * @param fromP The current position of the drone
     * @param toP The destination of the path planning
     * @param passed The positions where the drone have already been to
     * @return The next position the drone should go
     */
    glm::vec3 astarFindNext(glm::vec3 fromP, glm::vec3 toP, std::unordered_map<glm::vec3, int> &passed);

    /**
     * Initializing the private fields including possibleDir and sixDirs
     */
    void initDir();

public:

    /**
     * The default constructor that takes in a model
     * @param m
     */
    explicit aStarPathPlanner(model::threeDmodel &m);

    /**
     * The method that plan the path using A*
     * @param fromP The starting point
     * @param toP The destination
     * @return Whether the path planning is finished correctly
     */
    bool planPath(glm::vec3 fromP, glm::vec3 toP);

    /**
     * After planPath, use getPath to get the result.
     * @return All the points in the planned path
     */
    std::vector<glm::vec3> getPath();

    /**
     * Another version of getting the planned path.
     * @param carrier The container given by the user to get the path.
     */
    void getPath(std::vector<glm::vec3> &carrier);

    /**
     * Model setter
     */
    void setModel(model::threeDmodel &m);
};

#endif //ASTAR_CPP_ASTARPATHPLANNER_HPP

