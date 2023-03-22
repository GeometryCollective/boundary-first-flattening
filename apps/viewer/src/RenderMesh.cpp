#include "RenderMesh.h"

Buffer::Buffer(int N):
buffer(N),
vbo(0),
initialized(false)
{
	glGenBuffers(1, &vbo);
}

Buffer::~Buffer()
{
	glDeleteBuffers(1, &vbo);
}

void Buffer::bind() const
{
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
}

void Buffer::update()
{
	// bind buffer
	bind();

	// create buffer if it hasn't been initalized
	if (!initialized) {
		glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3)*buffer.size(), NULL, GL_DYNAMIC_DRAW);
		initialized = true;
	}

	// update
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(glm::vec3)*buffer.size(), &buffer[0]);
}

RenderMesh::RenderMesh():
vao(0),
pickVao(0),
initialized(false)
{

}

RenderMesh::~RenderMesh()
{
	glDeleteVertexArrays(1, &vao);
	glDeleteVertexArrays(1, &pickVao);
}

void RenderMesh::bindBuffers()
{
	// generate and bind vao
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	// set vertex attribute pointers
	if (positions) {
		positions->bind();
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid *)0);
		glEnableVertexAttribArray(0);
	}

	if (barycenters) {
		barycenters->bind();
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid *)0);
		glEnableVertexAttribArray(1);
	}

	if (colors) {
		colors->bind();
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid *)0);
		glEnableVertexAttribArray(2);
	}

	if (normals) {
		normals->bind();
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid *)0);
		glEnableVertexAttribArray(3);
	}

	if (uvs) {
		uvs->bind();
		glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid *)0);
		glEnableVertexAttribArray(4);
	}

	// unbind
	glBindVertexArray(0);
}

void RenderMesh::bindPickBuffers()
{
	// generate and bind pick vao
	glGenVertexArrays(1, &pickVao);
	glBindVertexArray(pickVao);

	// set vertex attribute pointers
	if (pickPositions) {
		pickPositions->bind();
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid *)0);
		glEnableVertexAttribArray(0);
	}

	if (pickColors) {
		pickColors->bind();
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid *)0);
		glEnableVertexAttribArray(2);
	}

	// unbind
	glBindVertexArray(0);
}

void RenderMesh::update(const std::vector<Attribute>& attributes)
{
	if (!initialized) {
		bindBuffers();
		bindPickBuffers();
		initialized = true;
	}

	// update buffer data
	for (size_t i = 0; i < attributes.size(); i++) {
		if (attributes[i] == POSITION && positions) positions->update();
		else if (attributes[i] == BARYCENTER && barycenters) barycenters->update();
		else if (attributes[i] == COLOR && colors) colors->update();
		else if (attributes[i] == NORMAL && normals) normals->update();
		else if (attributes[i] == UV && uvs) uvs->update();
		else if (attributes[i] == PICK_POSITION && pickPositions) pickPositions->update();
		else if (attributes[i] == PICK_COLOR && pickColors) pickColors->update();
	}
}

void RenderMesh::draw(Shader& shader, bool pick)
{
	shader.use();
	glBindVertexArray(pick ? pickVao : vao);
	glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(pick ? pickPositions->buffer.size() : positions->buffer.size()));
	glBindVertexArray(0);
}
