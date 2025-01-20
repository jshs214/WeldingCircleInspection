#include "WeldCircleInspection.h"

int main() {
    WeldCircleInspection wci = WeldCircleInspection();

    if (!wci.WeldCircleDetect()) {
        std::cout << " WeldCircleDetect   Failed \n";
        return 0;
    }

    return 0;
}