#pragma once

#include "Shader.h"
#include <vec3.hpp>
#include <vector>
#include <unordered_map>
#include <memory>

enum Attribute {
    POSITION,
    BARYCENTER,
    COLOR,
    NORMAL,
    UV,
    PICK_POSITION,
    PICK_COLOR
};

class RenderMesh;

class Buffer {
public:
    // constructor
    Buffer(int N = 0);

    // destructor
    ~Buffer();

    // init
    void bind() const;

    // update
    void update();

    // member
    vector<glm::vec3> buffer;

private:
    // member
    GLuint vbo;
    bool initialized;
};

class RenderMesh {
public:
    // constructor
    RenderMesh();

    // destructor
    ~RenderMesh();

    // update
    void update(const vector<Attribute>& attributes);

    // draw
    void draw(Shader& shader, bool pick = false);

    // members
    shared_ptr<Buffer> positions;
    shared_ptr<Buffer> uvs;
    shared_ptr<Buffer> normals;
    shared_ptr<Buffer> colors;
    shared_ptr<Buffer> barycenters;

    shared_ptr<Buffer> pickPositions;
    shared_ptr<Buffer> pickColors;

private:
    // binds buffers for drawing
    void bindBuffers();

    // binds buffers for picking
    void bindPickBuffers();

    // members
    GLuint vao, pickVao;
    bool initialized;
};
