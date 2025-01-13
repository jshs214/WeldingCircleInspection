#include "WeldCircleInspection.h"

int main() {
    eType mode = eType::CATHODE;

    WeldCircleInspection wci = WeldCircleInspection(mode);

    if (!wci.WeldCircleDetect()) {
        std::cout << " WeldCircleDetect   Failed \n";
        return 0;
    }


    return 0;
}