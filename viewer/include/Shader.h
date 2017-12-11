#pragma once

#if defined(NANOGUI_GLAD)
    #if defined(NANOGUI_SHARED) && !defined(GLAD_GLAPI_EXPORT)
        #define GLAD_GLAPI_EXPORT
    #endif
    #include <glad/glad.h>
#else
    #if defined(__APPLE__)
        #define GLFW_INCLUDE_GLCOREARB
    #else
        #define GL_GLEXT_PROTOTYPES
    #endif
#endif

#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <string>
#include <iostream>

using namespace std;

class Shader {
public:
    // constructor
    Shader();

    // destructor
    ~Shader();

    // loads vertex and fragment shaders
    void load(const string& vertexSource, const string& fragmentSource);

    // use
    void use();

    // member
    GLuint program;

private:
    // compiles shader
    GLuint compileShader(const string& code, GLenum type);

    // members
    GLuint vertex;
    GLuint fragment;
};
