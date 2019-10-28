//
// Created by imbaguanxin on 2019/10/24.
//

#ifndef ASTAR_CPP_POSITIONANDDISTANCE_HPP
#define ASTAR_CPP_POSITIONANDDISTANCE_HPP

#include <glm/glm.hpp>

class vecNode {

private:

    float score;
    glm::vec3 position;
    vecNode *parentNode;

public:

    vecNode(float sco, glm::vec3 pos, vecNode *parentPos);

    float getDistance();

    glm::vec3 getPos();

    vecNode *getParent();

};


#endif //ASTAR_CPP_POSITIONANDDISTANCE_HPP
