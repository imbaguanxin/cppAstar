#include <iostream>
#include "model/threeDModel.h"

int main() {
    model::threeDmodel model(2, 3, 4);
    model.printInfo();
    return 0;
}