#include "WeldCircleInspection.h"

int main() {
    eType mode = eType::ANODE;

    WeldCircleInspection wci = WeldCircleInspection(mode);

    if (!wci.WeldCircleDetect()) {
        std::cout << " WeldCircleDetect   Failed \n";
        return 0;
    }


    return 0;
}