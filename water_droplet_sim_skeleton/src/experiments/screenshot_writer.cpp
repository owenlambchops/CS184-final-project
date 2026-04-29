#include "wd/experiments/screenshot_writer.h"

#include <glad/glad.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <algorithm>
#include <vector>

namespace wd {

bool saveFramebufferPng(const std::string& path, int width, int height) {
    if (width <= 0 || height <= 0) return false;

    constexpr int channels = 4;
    std::vector<unsigned char> pixels(static_cast<size_t>(width) * static_cast<size_t>(height) * channels);
    std::vector<unsigned char> flipped(pixels.size());

    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());

    const size_t rowBytes = static_cast<size_t>(width) * channels;
    for (int y = 0; y < height; ++y) {
        const auto* src = pixels.data() + static_cast<size_t>(height - 1 - y) * rowBytes;
        auto* dst = flipped.data() + static_cast<size_t>(y) * rowBytes;
        std::copy(src, src + rowBytes, dst);
    }

    return stbi_write_png(path.c_str(), width, height, channels, flipped.data(), width * channels) != 0;
}

} // namespace wd
