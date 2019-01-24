#pragma once

#include "Camera.h"
#include "RenderMesh.h"
#include "Bff.h"
#include <nanogui/nanogui.h>
#include <list>

using namespace nanogui;
using namespace bff;

enum class PlotType {
	constant,
	shaded,
	normals,
	quasiConformalError,
	areaScaling
};

enum class BoundaryType {
	automatic,
	disk,
	editBoundary,
	setBoundaryAngles
};

class ViewPane {
public:
	ViewPane() {}
	ViewPane(double l, double b, double w, double h): left(l), bottom(b), width(w), height(h) {}
	double left, bottom, width, height;
	double aspect() { return width/height; }
};

// The VertexHandle class stores all the data needed for
// a GUI widget associated with a vertex---in particular,
// cone points during cone editing mode, and corners of
// the 2D layout in angle editing mode. A list of all
// VertexHandles is stored as a linked list "handles"
// in the Viewer class
class VertexHandle {
public:
	VertexIter vertex;
	std::shared_ptr<RenderMesh> sphere3D;
	std::shared_ptr<RenderMesh> sphereUV;

	bool operator==(const VertexHandle& handle) const {
		return vertex == handle.vertex;
	}
};

class ModelState {
public:
	ModelState():
	cut(NULL),
	bff(NULL),
	surfaceIsClosed(false),
	surfaceHasGenerators(false),
	mappedToSphere(false),
	flattenedToDisk(false),
	editBoundary(false),
	setBoundaryAngles(false) {}

	std::vector<std::shared_ptr<RenderMesh>> renderMeshes;
	std::vector<std::shared_ptr<RenderMesh>> splineHandles;
	std::vector<std::shared_ptr<RenderMesh>> splineLines;
	std::shared_ptr<RenderMesh> cut;

	std::shared_ptr<BFF> bff;

	bool surfaceIsClosed;
	bool surfaceHasGenerators;
	bool mappedToSphere;
	bool flattenedToDisk;
	bool editBoundary;
	bool setBoundaryAngles;

	Spline spline;
	WedgeIter selectedKnot;
	std::vector<WedgeIter> knots;
	DenseMatrix targetData;

	VertexIter selectedVertexHandle;
	std::list<VertexHandle> handles;
	DenseMatrix coneAngles;
	DenseMatrix boundaryAngles;
};

class Viewer {
public:
	// displays the viewer until the program ends
	static void init(const std::string& objPath = "");

private:
	// initialization
	static void initGLFW();
	static void initGui();
	static void initGL();
	static void initShaders();
	static void initCameras();
	static void initTransforms();
	static void initLights();
	static void initModel();
	static void initRenderMeshes();
	static void initSpline();
	static void initBFF();

	// clear
	static void clearScene();
	static void clear();
	static void clearViewport(const ViewPane& pane);

	// update
	static void updatePlotLabel();
	static void updateAngleControlLabel();
	static void updateAngleTextBoxAndSliderValue(double angle);
	static void updateUniforms(int index, int width, int height);
	static void updateRenderMeshes();
	static void updateSphere(std::shared_ptr<RenderMesh> sphere,
							 const glm::vec3& center, const glm::vec3& color);
	static void addSphere(std::shared_ptr<RenderMesh>& sphere,
						  const glm::vec3& center, const glm::vec3& color);
	static void addSphere(std::vector<std::shared_ptr<RenderMesh>>& spheres,
						  const glm::vec3& center, const glm::vec3& color);
	static void updateLine(std::shared_ptr<RenderMesh> line, const glm::vec3& color, float width,
						   float x1, float y1, float x2, float y2, float z);
	static void addLine(std::vector<std::shared_ptr<RenderMesh>>& lines,
						const glm::vec3& color, float width,
						float x1, float y1, float x2, float y2, float z);
	static void updateCut();
	static void updateInstructionText();
	static void updateViewPanes();
	static void updateCameraZoom();
	static void updateVertexHandles(VertexIter v);
	static void updateVertexHandlesUVs();
	static void update(bool updateZoom = true);
	static void updateSelectedMesh();

	// add and remove spline knots
	static void addKnot(WedgeIter w);
	static void removeKnot(WedgeIter w);
	static void dragKnot(double change);

	// add and remove cut and vertex handles
	static void addVertexHandle(VertexIter v, double newAngle);
	static void removeVertexHandle(VertexIter v);
	static void removeVertexHandles();

	static void placeCones(int S);
	static void normalizeConeAngles();

	static void setDefaultBoundaryAngles();

	static std::vector<VertexIter> getHandleVertices();

	// flatten
	static void performPCA();
	static void flattenWithTargetBoundary(BoundaryType type, bool updateZoom = true);
	static void flattenWithBoundaryAnglesLRU();
	static void flattenWithBoundaryAnglesNormalize();
	static void flattenWithSplineBoundary(bool givenScaleFactors, bool updateZoom = false);
	static void flattenWithCones();
	static void flattenToShape(const std::vector<Vector>& gamma);
	static void mapToSphere();

	// normalized device coordinates
	static Vector ndc(int x, int y, int width, int height);

	// glut callbacks
	static void cursorPosCallback(GLFWwindow *w, double x, double y);
	static void mouseButtonCallback(GLFWwindow *w, int button, int action, int modifiers);
	static void keyCallback(GLFWwindow *w, int key, int scancode, int action, int mods);
	static void scrollCallback(GLFWwindow *w, double x, double y);
	static void charCallback(GLFWwindow *w, unsigned int codepoint);
	static void framebufferSizeCallback(GLFWwindow *w, int width, int height);

	// picking
	static double pickDistance(int index, const Vector& position, const ViewPane& pane);
	static bool didPick(int index, const Vector& position, const ViewPane& pane);
	static WedgeIter closestWedge(const ViewPane& pane, bool onBoundary);
	static void pickKnot(const ViewPane& pane);
	static void pickVertexHandleUV(const ViewPane& pane);
	static bool pickVertexHandle(const ViewPane& pane);
	static void processPickedElement(int index, int pickedId);
	static void pick(int index);

	// display
	static void drawRenderMeshes(const ViewPane& pane, int index);
	static void drawScene();
	static void draw();

	// report error
	static void reportError(const std::string& error);

	// members
	static GLFWwindow *window;
	static Screen *gui;
	static ComboBox *patternComboBox;
	static ComboBox *targetBoundaryComboBox;
	static Label *plotLabel;
	static Label *angleControlLabel;
	static TextBox *vertexHandleCountTextBox;
	static TextBox *angleTextBox;
	static Slider *angleSlider;
	static Label *instructionText;
	static CheckBox *exportNormalizedUVsCheckBox;
	static Button *placeConesButton;

	static int windowWidth;
	static int windowHeight;
	static int framebufferWidth;
	static int framebufferHeight;

	static ViewPane surfacePane;
	static ViewPane uvPane;
	static ViewPane fullPane;

	static GLuint transformUbo;
	static GLuint lightUbo;
	static float patternSize;

	static std::vector<std::shared_ptr<Camera>> cameras;

	static Shader modelShader;
	static Shader flatShader;
	static Shader wireframeShader;
	static bool showWireframe;

	static Vector origUp;
	static Vector mouse, origMouse;
	static int clickX, unclickX;
	static int clickY, unclickY;
	static bool mouseDown;

	static bool pickingEnabled;
	static bool shiftClick;
	static bool ctrlClick;
	static bool altClick;

	static std::string objPath;
	static std::vector<Mesh> model;
	static std::vector<ModelState> modelStates;
	static Mesh *mesh;
	static ModelState *state;
	static int selectedMesh;
	static bool loadedModel;

	static PlotType plotType;
	static bool colorsNeedUpdate;

	static bool mapIsDirty;
	static bool cutSurface;
};
