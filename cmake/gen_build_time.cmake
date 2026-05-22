# Auto-generated build timestamp — regenerated on every build
string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S")
file(WRITE "${BUILD_TIME_DIR}/build_time.h"
    "// Auto-generated — updated every build\n"
    "#define BUILD_TIMESTAMP \"${BUILD_TIMESTAMP}\"\n")
