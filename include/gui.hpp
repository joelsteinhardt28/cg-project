#pragma once

#include "constants.hpp"

namespace gui {
    /**
     * Resets application state, removes registered structures from Polyscope, and clears mesh state.
     * 
     * @param state The global application state
     */
    void reset(AppState& state);

    /**
     * Scans the target OFF files directory and refreshes the list of available .off files in AppState.
     * 
     * @param state The global application state
     */
    void refreshOffFileList(AppState& state);

    /**
     * Loads an OFF mesh file by index from the discovered offFiles list, populates exact properties,
     * resets state, and registers visual structures in Polyscope.
     * 
     * @param state The global application state
     * @param fileIdx Index of the file in the state.io.offFiles vector
     */
    void loadOffFile(AppState& state, int fileIdx);

    /**
     * Main ImGui rendering callback invoked each frame to display control panels, action buttons,
     * status notifications, and strategy options inside Polyscope.
     * 
     * @param state The global application state
     */
    void render(AppState& state);
}
