#pragma once

#include "Camera.h"
#include "RenderMesh.h"
#include "Bff.h"
#include <nanogui/nanogui.h>
#include <list>

using namespace nanogui;

enum class PlotType {
    constant,
    shaded,
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
    shared_ptr<RenderMesh> sphere3D;
    shared_ptr<RenderMesh> sphereUV;

    bool operator==(const VertexHandle& handle) const {
        return vertex == handle.vertex;
    }
};

class Viewer {
public:
    // displays the viewer until the program ends
    static void init(const string& objPath = "");

private:
    // initialization
    static void initGLFW();
    static void initGui();
    static void initGL();
    static void initShaders();
    static void initCameras();
    static void initTransforms();
    static void initLights();
    static void initHalfEdgeMesh();
    static void initMeshes();
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
    static void updateMeshes();
    static void updateSphere(shared_ptr<RenderMesh> sphere,
                             const glm::vec3& center, const glm::vec3& color);
    static void addSphere(shared_ptr<RenderMesh>& sphere,
                          const glm::vec3& center, const glm::vec3& color);
    static void addSphere(vector<shared_ptr<RenderMesh>>& spheres,
                          const glm::vec3& center, const glm::vec3& color);
    static void updateLine(shared_ptr<RenderMesh> line, const glm::vec3& color, float width,
                           float x1, float y1, float x2, float y2, float z);
    static void addLine(vector<shared_ptr<RenderMesh>>& lines, const glm::vec3& color, float width,
                        float x1, float y1, float x2, float y2, float z);
    static void updateCut();
    static void updateInstructionText();
    static void updateViewPanes();
    static void updateCameraZoom();
    static void updateVertexHandles(VertexIter v);
    static void updateVertexHandlesUVs();
    static void update(bool updateZoom = true);

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

    static vector<VertexIter> getHandleVertices();

    // flatten
    static void performPCA();
    static void flattenWithTargetBoundary(BoundaryType type, bool updateZoom = true);
    static void flattenWithBoundaryAnglesLRU();
    static void flattenWithBoundaryAnglesNormalize();
    static void flattenWithSplineBoundary(bool givenScaleFactors, bool updateZoom = false);
    static void flattenWithCones();
    static void flattenToShape(const vector<Vector>& gamma);
    static void mapToSphere();

    // shift, rotate and scale uvs
    static void shiftUVs(double dx, double dy);
    static void rotateUVs(double theta);
    static void scaleUVs(double scale);
    static void rotateScaleShiftUVs();

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
    static void drawRenderMesh(const ViewPane& pane, int index);
    static void drawScene();
    static void draw();

    // report error
    static void reportError(const string& error);

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
    static float patternFactor;
    static float patternSize;

    static vector<shared_ptr<Camera>> cameras;

    static vector<shared_ptr<RenderMesh>> meshes;
    static vector<shared_ptr<RenderMesh>> splineHandles;
    static vector<shared_ptr<RenderMesh>> splineLines;
    static shared_ptr<RenderMesh> cut;

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

    static string objPath;
    static Mesh halfEdgeMesh;
    static bool loadedHalfEdgeMesh;
    static bool surfaceIsClosed;

    static shared_ptr<BFF> bff;
    static PlotType plotType;
    static bool colorsNeedUpdate;

    static Spline spline;
    static WedgeIter selectedKnot;
    static vector<WedgeIter> knots;
    static DenseMatrix targetData;
    static bool editBoundary;

    static VertexIter selectedVertexHandle;
    static std::list<VertexHandle> handles;
    static DenseMatrix coneAngles;
    static DenseMatrix boundaryAngles;
    static bool setBoundaryAngles;
    static bool mapIsDirty;
    static bool cutSurface;

    static bool mappedToSphere;
};
