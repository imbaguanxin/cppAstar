//
// Created by imbaguanxin on 2019/10/9.
//

#ifndef ASTAR_CPP_PATHSIMPLIFIER_HPP
#define ASTAR_CPP_PATHSIMPLIFIER_HPP

#include "model/threeDModel.h"
#include <glm/glm.hpp>

class pathSimplifier {
private:

    model::threeDmodel mapModel;

    std::vector<glm::vec3> simplifiedPath;

    bool checkValidPath(glm::vec3 from, glm::vec3 to);

    std::vector<glm::vec3> findBlockedPieces(glm::vec3 from, glm::vec3 to);

    bool checkSingleBlock(glm::vec3 from, glm::vec3 to, glm::vec3 blockedPosition);

    static bool checkSingleBlockedHelp(glm::vec4 s, glm::vec4 v);

    static std::pair<glm::vec3, glm::vec3> findBoxBound(glm::vec3 from, glm::vec3 to);

public:
    pathSimplifier();

    explicit pathSimplifier(model::threeDmodel &m);

    void setModel(model::threeDmodel &m);

    void simplify(std::vector<glm::vec3> oriPath);
};


#endif //ASTAR_CPP_PATHSIMPLIFIER_HPP
