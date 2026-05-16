#pragma once

#include "structs.hpp"
#include "spatial_data_structure.hpp"

namespace gui {
    void refreshOffFileList(AppState& state);
    void render(AppState& state);
    void registerBoundingBox(AppState& state);
}
