#pragma once

#include <filesystem>
#include <glm/fwd.hpp>

namespace wd {

class Shader {
public:
    Shader() = default;
    ~Shader();

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&& other) noexcept;
    Shader& operator=(Shader&& other) noexcept;

    bool load(const std::filesystem::path& vertexPath, const std::filesystem::path& fragmentPath);
    void use() const;
    void reset();

    unsigned int id() const { return program_; }

    void setMat4(const char* name, const glm::mat4& value) const;
    void setVec3(const char* name, const glm::vec3& value) const;
    void setFloat(const char* name, float value) const;
    void setInt(const char* name, int value) const;

private:
    int uniformLocation(const char* name) const;

    unsigned int program_ = 0;
};

} // namespace wd
