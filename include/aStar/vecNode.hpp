//
// Created by imbaguanxin on 2019/10/24.
//

#ifndef ASTAR_CPP_POSITIONANDDISTANCE_HPP
#define ASTAR_CPP_POSITIONANDDISTANCE_HPP

#include <glm/glm.hpp>

class vecNode {

private:

    // the score of cost
    float score;
    // the position in 3D
    glm::vec3 position;
    // the position it come from
    vecNode *parentNode;

public:

    /**
     * Build a node and all fields cannot be changed afterwards
     * @param sco The score of cost
     * @param pos  It's position in 3D
     * @param parentPos  It's parent
     */
    vecNode(float sco, glm::vec3 pos, vecNode *parent);

    // score getter
    float getDistance();

    // position getter
    glm::vec3 getPos();

    // parent node getter
    vecNode *getParent();

};


#endif //ASTAR_CPP_POSITIONANDDISTANCE_HPP
