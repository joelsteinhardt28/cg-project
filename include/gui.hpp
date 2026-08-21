#pragma once

#include "constants.hpp"

namespace gui {
    void reset(AppState& state);
    void refreshOffFileList(AppState& state);
    void loadOffFile(AppState& state, int fileIdx);
    void render(AppState& state);
}
