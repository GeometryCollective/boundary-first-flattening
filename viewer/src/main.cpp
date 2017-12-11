#include "Viewer.h"

int main(int argc, const char * argv[]) {
    if (argc > 1) Viewer::init(argv[1]);
    else Viewer::init();

    return 0;
}
