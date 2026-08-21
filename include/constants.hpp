#pragma once

#include <string_view>

#include "imgui.h"

#include "structs.hpp"


namespace globalSettings {
    inline constexpr bool showAnyLogs = true;
    inline constexpr bool showDebugLogs = false;
    inline constexpr int64_t scaleFactor = 1e6;  // ! TODO: Make adjustable
}

namespace constants::colors {
    inline constexpr Color mesh             = {0.7f, 0.7f, 0.7f};
    inline constexpr Color kernel           = {0.1f, 0.5f, 0.9f};
    inline constexpr Color cutPlane         = {0.8f, 0.1f, 0.2f};
    inline constexpr Color cutPlaneNormal   = {0.2f, 0.8f, 0.3f};
    inline constexpr Color bbox             = {0.4, 0.4, 0.4};

    // Gui Colors
    inline constexpr ImVec4 guiTitle               = ImVec4(0.075f, 0.929f, 0.922f, 1.0f);
    inline constexpr ImVec4 guiInfo                = ImVec4(0.929f, 0.925f, 0.075f, 1.0f); 

    inline constexpr ImVec4 guiResetButton         = ImVec4(0.820f, 0.220f, 0.220f, 1.0f);
    inline constexpr ImVec4 guiResetButtonHovered  = ImVec4(0.950f, 0.320f, 0.320f, 1.0f);
    inline constexpr ImVec4 guiResetButtonActive   = ImVec4(0.680f, 0.150f, 0.150f, 1.0f);

    inline constexpr ImVec4 guiLimeButton          = ImVec4(0.320f, 0.680f, 0.200f, 1.0f);
    inline constexpr ImVec4 guiLimeButtonHovered   = ImVec4(0.400f, 0.780f, 0.260f, 1.0f);
    inline constexpr ImVec4 guiLimeButtonActive    = ImVec4(0.240f, 0.550f, 0.140f, 1.0f);
}

namespace constants::gui {
    inline constexpr ImVec2 buttonSize             = ImVec2(220.0f, 0.0f);
}

namespace constants::transparencies {
    inline constexpr float mesh             = 0.3f;
    inline constexpr float kernel           = 0.6f;
    inline constexpr float cutPlane         = 0.6f;
}

namespace constants::otherVisuals {
    inline constexpr float bboxRadius       = 0.001f;
    inline constexpr float normalRadius     = 0.003f;
}

namespace constants::polyNames {
    inline constexpr std::string_view mesh             = "Mesh";
    inline constexpr std::string_view pc               = "Points";
    inline constexpr std::string_view kernel           = "Intermediate Kernel";
    inline constexpr std::string_view bbox             = "Bounding Box";
    inline constexpr std::string_view cutPlane         = "Cutting Plane";
    inline constexpr std::string_view cutPlaneNormal   = "Cutting Plane Normal";
}