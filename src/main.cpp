#include <iostream>

#include "args/args.hxx"
#include "polyscope/polyscope.h"

#include "gui.hpp"

// Application state for shared data
AppState state;

void callback() {
    gui::render(state);
}

int main(int argc, char** argv) {
    // Configure the argument parser
    args::ArgumentParser parser("Computer Graphics 2 Sample Code.");

    // Parse args
    try {
        parser.ParseCLI(argc, argv);
    } catch (const args::Help&) {
        std::cout << parser;
        return 0;
    } catch (const args::ParseError& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }

    // Polyscope options
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
    polyscope::options::shadowBlurIters = 6;

    // Initialize polyscope and its GUI
    polyscope::init();
    polyscope::state::userCallback = callback;
    polyscope::show();

    return 0;
}
