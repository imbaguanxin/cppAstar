//
// Created by imbaguanxin on 2019/10/9.
//

#ifndef ASTAR_CPP_BLOCKCHECKERSIMPLIFIER_HPP
#define ASTAR_CPP_BLOCKCHECKERSIMPLIFIER_HPP

#include "model/threeDModel.h"
#include <glm/glm.hpp>

class blockCheckerSimplifier {

private:

    // mapModel stores a 3D map.
    model::threeDmodel mapModel;

    // the result of simplified path.
    std::vector<glm::vec3> simplifiedPath;

    // The Size of a drone in meter
    float droneSize;

public:

    // A default constructor that set droneSize to 0 and no map model.
    blockCheckerSimplifier();

    // A constructor that sets a model (drone size is set to 0)
    explicit blockCheckerSimplifier(model::threeDmodel &m);

    // model setter
    void setModel(model::threeDmodel &m);

    // simplifies a path based on A* path
    std::vector<glm::vec3> simplify(const std::vector<glm::vec3> &oriPath);

    // drone size setter
    void setDroneSize(float dSize);

private:

    bool checkValidPath(glm::vec3 from, glm::vec3 to);

    std::vector<glm::vec3> findBlockedPieces(glm::vec3 from, glm::vec3 to);

    static bool checkSingleBlock(glm::vec3 &from, glm::vec3 &to, glm::vec3 &blockedPosition);

    static bool checkSingleBlockedHelp(glm::vec4 &s, glm::vec4 &v);

    static std::pair<glm::vec3, glm::vec3> findBoxBound(glm::vec3 from, glm::vec3 to);


};


#endif //ASTAR_CPP_BLOCKCHECKERSIMPLIFIER_HPP
