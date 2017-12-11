#include "Shader.h"

Shader::Shader():
program(0),
vertex(0),
fragment(0)
{

}

Shader::~Shader()
{
    if (vertex) {
        glDetachShader(program, vertex);
        glDeleteShader(vertex);
    }

    if (fragment) {
        glDetachShader(program, fragment);
        glDeleteShader(fragment);
    }

    if (program) {
        glDeleteProgram(program);
    }
}

void Shader::load(const string& vertexSource, const string& fragmentSource)
{
    vertex = compileShader(vertexSource, GL_VERTEX_SHADER);
    fragment = compileShader(fragmentSource, GL_FRAGMENT_SHADER);

    // create program and attach shaders
    program = glCreateProgram();
    if (vertex) glAttachShader(program, vertex);
    if (fragment) glAttachShader(program, fragment);

    GLint success;
    GLchar infoLog[512];

    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(program, 512, NULL, infoLog);
        cerr << infoLog << endl;
    }
}

void Shader::use()
{
    glUseProgram(program);
}

GLuint Shader::compileShader(const string& code, GLenum type)
{
    GLint success;
    GLchar infoLog[512];
    GLuint shader = 0;

    const GLchar *shaderCode = code.c_str();
    shader = glCreateShader(type);
    glShaderSource(shader, 1, &shaderCode, NULL);
    glCompileShader(shader);

    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(program, 512, NULL, infoLog);
        cerr << infoLog << endl;
        shader = 0;
    }

    return shader;
}
