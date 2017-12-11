#include "Common.h"

Common common;

Common::Common() {
    cholmod_l_start(&common);
    common.supernodal = CHOLMOD_SUPERNODAL;
}

Common::~Common() {
    cholmod_l_finish(&common);
}

Common::operator cholmod_common*() {
    return &common;
}
