#pragma once

#include "Shader.h"
#include <glm/vec3.hpp>
#include <vector>
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
	std::vector<glm::vec3> buffer;

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
	void update(const std::vector<Attribute>& attributes);

	// draw
	void draw(Shader& shader, bool pick = false);

	// members
	std::shared_ptr<Buffer> positions;
	std::shared_ptr<Buffer> uvs;
	std::shared_ptr<Buffer> normals;
	std::shared_ptr<Buffer> colors;
	std::shared_ptr<Buffer> barycenters;

	std::shared_ptr<Buffer> pickPositions;
	std::shared_ptr<Buffer> pickColors;

private:
	// binds buffers for drawing
	void bindBuffers();

	// binds buffers for picking
	void bindPickBuffers();

	// members
	GLuint vao, pickVao;
	bool initialized;
};
