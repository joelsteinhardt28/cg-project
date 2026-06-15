#pragma once

#include <string>
#include <string_view>

#include "structs.hpp"


namespace globalSettings {
    inline constexpr bool showDebugLogs = false;
}


namespace constants::colors {
    inline constexpr Color mesh = {0.7f, 0.7f, 0.7f};
    inline constexpr Color kernel = {0.1f, 0.5f, 0.9f};
    inline constexpr Color cutPlane = {0.8f, 0.1f, 0.2f};
    inline constexpr Color cutPlaneNormal = {0.2f, 0.8f, 0.3f};
    inline constexpr Color bbox = {0.4, 0.4, 0.4};
}

namespace constants::transparencies {
    inline constexpr float kernel = 0.6f;
    inline constexpr float cutPlane = 0.6f;
}

namespace constants::otherVisuals {
    inline constexpr float bboxRadius = 0.001f;
    inline constexpr float normalRadius = 0.003f;
}


namespace constants::polyNames {
    inline constexpr std::string_view mesh = "Mesh";
    inline constexpr std::string_view pc = "Points";
    inline constexpr std::string_view kernel = "Intermediate Kernel";
    inline constexpr std::string_view bbox = "Bounding Box";
    inline constexpr std::string_view cutPlane = "Cutting Plane";
    inline constexpr std::string_view cutPlaneNormal = "Cutting Plane Normal";
}