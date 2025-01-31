#include "WeldCircleInspection.h"

int main() {
    WeldCircleInspection wci = WeldCircleInspection();

    if (!wci.WeldCircleDetect()) {
        return 0;
    }

    return 0;
}