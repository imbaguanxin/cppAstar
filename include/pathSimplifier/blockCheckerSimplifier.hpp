//
// Created by imbaguanxin on 2019/10/9.
//

#ifndef ASTAR_CPP_BLOCKCHECKERSIMPLIFIER_HPP
#define ASTAR_CPP_BLOCKCHECKERSIMPLIFIER_HPP

#include "model/threeDModel.h"
#include <glm/glm.hpp>

class blockCheckerSimplifier {
private:

    model::threeDmodel mapModel;

    std::vector<glm::vec3> simplifiedPath;

    float droneSize;

public:

    blockCheckerSimplifier();

    explicit blockCheckerSimplifier(model::threeDmodel &m);

    void setModel(model::threeDmodel &m);

    std::vector<glm::vec3> simplify(const std::vector<glm::vec3> &oriPath);

    void setDroneSize(float dSize);

private:

    bool checkValidPath(glm::vec3 from, glm::vec3 to);

    std::vector<glm::vec3> findBlockedPieces(glm::vec3 from, glm::vec3 to);

    static bool checkSingleBlock(glm::vec3 &from, glm::vec3 &to, glm::vec3 &blockedPosition);

    static bool checkSingleBlockedHelp(glm::vec4 &s, glm::vec4 &v);

    static std::pair<glm::vec3, glm::vec3> findBoxBound(glm::vec3 from, glm::vec3 to);


};


#endif //ASTAR_CPP_BLOCKCHECKERSIMPLIFIER_HPP
