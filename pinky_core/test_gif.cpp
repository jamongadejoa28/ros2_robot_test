#include <iostream>
#include <string>
#include <vector>
#include "stb/stb_image.h"
#include "pinky_core/core/emotion_renderer.h"

int main(int argc, char** argv) {
    auto anim = pinky::LoadAnimatedEmotion(argv[1], 320, 240);
    if (anim.frames.empty()) {
        std::cout << "Empty frames" << std::endl;
    } else {
        std::cout << "Loaded " << anim.frames.size() << " frames" << std::endl;
    }
    return 0;
}
