#include "wd/render/shader.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>

namespace wd {

namespace {

std::string loadTextFile(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) {
        std::cerr << "Failed to open shader file: " << std::filesystem::absolute(path)
                  << " (cwd: " << std::filesystem::current_path() << ")\n";
        return {};
    }

    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

unsigned int compileShaderStage(unsigned int type, const std::filesystem::path& path) {
    const std::string source = loadTextFile(path);
    if (source.empty()) return 0;

    const char* sourcePtr = source.c_str();
    const unsigned int shader = glCreateShader(type);
    glShaderSource(shader, 1, &sourcePtr, nullptr);
    glCompileShader(shader);

    int success = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (success == GL_TRUE) return shader;

    int logLength = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);
    std::string log(std::max(logLength, 1), '\0');
    glGetShaderInfoLog(shader, logLength, nullptr, log.data());
    std::cerr << "Shader compile failed: " << path << "\n" << log << "\n";
    glDeleteShader(shader);
    return 0;
}

} // namespace

Shader::~Shader() {
    reset();
}

Shader::Shader(Shader&& other) noexcept : program_(std::exchange(other.program_, 0)) {}

Shader& Shader::operator=(Shader&& other) noexcept {
    if (this == &other) return *this;

    reset();
    program_ = std::exchange(other.program_, 0);
    return *this;
}

bool Shader::load(const std::filesystem::path& vertexPath, const std::filesystem::path& fragmentPath) {
    reset();

    const unsigned int vertexShader = compileShaderStage(GL_VERTEX_SHADER, vertexPath);
    if (vertexShader == 0) return false;

    const unsigned int fragmentShader = compileShaderStage(GL_FRAGMENT_SHADER, fragmentPath);
    if (fragmentShader == 0) {
        glDeleteShader(vertexShader);
        return false;
    }

    program_ = glCreateProgram();
    glAttachShader(program_, vertexShader);
    glAttachShader(program_, fragmentShader);
    glLinkProgram(program_);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    int success = GL_FALSE;
    glGetProgramiv(program_, GL_LINK_STATUS, &success);
    if (success == GL_TRUE) return true;

    int logLength = 0;
    glGetProgramiv(program_, GL_INFO_LOG_LENGTH, &logLength);
    std::string log(std::max(logLength, 1), '\0');
    glGetProgramInfoLog(program_, logLength, nullptr, log.data());
    std::cerr << "Program link failed: " << vertexPath << " + " << fragmentPath << "\n"
              << log << "\n";

    reset();
    return false;
}

void Shader::use() const {
    if (program_ != 0) glUseProgram(program_);
}

void Shader::reset() {
    if (program_ != 0) glDeleteProgram(program_);
    program_ = 0;
}

void Shader::setMat4(const char* name, const glm::mat4& value) const {
    const int location = uniformLocation(name);
    if (location >= 0) glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(value));
}

void Shader::setVec3(const char* name, const glm::vec3& value) const {
    const int location = uniformLocation(name);
    if (location >= 0) glUniform3fv(location, 1, glm::value_ptr(value));
}

void Shader::setFloat(const char* name, float value) const {
    const int location = uniformLocation(name);
    if (location >= 0) glUniform1f(location, value);
}

void Shader::setInt(const char* name, int value) const {
    const int location = uniformLocation(name);
    if (location >= 0) glUniform1i(location, value);
}

int Shader::uniformLocation(const char* name) const {
    if (program_ == 0) return -1;
    return glGetUniformLocation(program_, name);
}

} // namespace wd
