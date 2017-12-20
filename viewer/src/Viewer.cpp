#include "Viewer.h"
#include "ConePlacement.h"
#include "Cutter.h"
#include "Distortion.h"
#include "ShadersSource.h"
#include "Eigen/Eigenvalues"
#include <iomanip>
#include <limits>

#define EYE 3.5
#define MAX_PICKED_ID 16777215

GLFWwindow* Viewer::window = NULL;
Screen* Viewer::gui = new Screen();
ComboBox* Viewer::patternComboBox = NULL;
ComboBox* Viewer::targetBoundaryComboBox = NULL;
Label* Viewer::plotLabel = NULL;
Label* Viewer::angleControlLabel = NULL;
TextBox* Viewer::vertexHandleCountTextBox = NULL;
TextBox* Viewer::angleTextBox = NULL;
Slider* Viewer::angleSlider = NULL;
Label* Viewer::instructionText = NULL;
CheckBox* Viewer::exportNormalizedUVsCheckBox = NULL;
Button* Viewer::placeConesButton = NULL;

int Viewer::windowWidth = 1300;
int Viewer::windowHeight = 700;
int Viewer::framebufferWidth = -1;
int Viewer::framebufferHeight = -1;

ViewPane Viewer::surfacePane;
ViewPane Viewer::uvPane;
ViewPane Viewer::fullPane;

GLuint Viewer::transformUbo;
GLuint Viewer::lightUbo;
float Viewer::patternFactor = 0.5;
float Viewer::patternSize = 2.0;

vector<shared_ptr<Camera>> Viewer::cameras;

vector<shared_ptr<RenderMesh>> Viewer::meshes;
vector<shared_ptr<RenderMesh>> Viewer::splineHandles;
vector<shared_ptr<RenderMesh>> Viewer::splineLines;
shared_ptr<RenderMesh> Viewer::cut = NULL;

Shader Viewer::modelShader;
Shader Viewer::flatShader;
Shader Viewer::wireframeShader;
bool Viewer::showWireframe = false;

Vector Viewer::origUp;
Vector Viewer::mouse, Viewer::origMouse;
int Viewer::clickX = -1;
int Viewer::clickY = -1;
int Viewer::unclickX = -1;
int Viewer::unclickY = -1;
bool Viewer::mouseDown = false;

bool Viewer::pickingEnabled = false;
bool Viewer::shiftClick = false;
bool Viewer::ctrlClick = false;
bool Viewer::altClick = false;

string Viewer::objPath;
Mesh Viewer::halfEdgeMesh;
bool Viewer::loadedHalfEdgeMesh = false;
bool Viewer::surfaceIsClosed = false;

shared_ptr<BFF> Viewer::bff = NULL;
PlotType Viewer::plotType = PlotType::shaded;
bool Viewer::colorsNeedUpdate = true;

Spline Viewer::spline;
WedgeIter Viewer::selectedKnot;
vector<WedgeIter> Viewer::knots;
DenseMatrix Viewer::targetData;
bool Viewer::editBoundary = false;

VertexIter Viewer::selectedVertexHandle;
list<VertexHandle> Viewer::handles;
DenseMatrix Viewer::coneAngles;
DenseMatrix Viewer::boundaryAngles;
bool Viewer::setBoundaryAngles = false;
bool Viewer::cutSurface = true;
bool Viewer::mapIsDirty = false;

bool Viewer::mappedToSphere = false;

const glm::vec3 white(1.0, 1.0, 1.0);
const glm::vec3 skyblue(0.643, 0.647, 0.815);
const glm::vec3 red(1.0, 0.0, 0.0);
const glm::vec3 orange(1.0, 0.55, 0.0);
const glm::vec3 yellow(1.0, 0.9, 0.1);

void Viewer::init(const string& objPath_)
{
    objPath = objPath_;
    initGLFW();
    initGui();
    initGL();
    initShaders();
    initCameras();
    initTransforms();
    initLights();
    initHalfEdgeMesh();
    initMeshes();
    initBFF();

    // render and poll events
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        draw();
    }

    // clear and terminate
    clear();
    glfwTerminate();
}

static void errorCallback(int error, const char* description)
{
    cerr << description << endl;
}

void Viewer::initGLFW()
{
    glfwSetErrorCallback(errorCallback);
    if (!glfwInit()) {
        cerr << "Unable to initalize glfw" << endl;
        exit(EXIT_FAILURE);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    glfwWindowHint(GLFW_SAMPLES, 0);
    glfwWindowHint(GLFW_RED_BITS, 8);
    glfwWindowHint(GLFW_GREEN_BITS, 8);
    glfwWindowHint(GLFW_BLUE_BITS, 8);
    glfwWindowHint(GLFW_ALPHA_BITS, 8);
    glfwWindowHint(GLFW_STENCIL_BITS, 8);
    glfwWindowHint(GLFW_DEPTH_BITS, 24);
    glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

    // initialize window
    window = glfwCreateWindow(windowWidth, windowHeight, "Boundary First Flattening", NULL, NULL);
    if (!window) {
        cerr << "Unable to initalize glfw window" << endl;
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);
#if defined(NANOGUI_GLAD)
    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
        cerr << "Unable to initalize glad" << endl;
        exit(EXIT_FAILURE);
    }
#endif
    glfwSwapInterval(1);

    // specify callbacks
    glfwSetCursorPosCallback(window, Viewer::cursorPosCallback);
    glfwSetMouseButtonCallback(window, Viewer::mouseButtonCallback);
    glfwSetKeyCallback(window, Viewer::keyCallback);
    glfwSetScrollCallback(window, Viewer::scrollCallback);
    glfwSetCharCallback(window, Viewer::charCallback);
    glfwSetFramebufferSizeCallback(window, Viewer::framebufferSizeCallback);
}

void Viewer::initGui()
{
    gui->initialize(window, true);
    Window *nanoguiWindow = new Window(gui, "");
    nanoguiWindow->setLayout(new GroupLayout());

    Eigen::Vector3f highlightColor = Eigen::Vector3f(yellow.x, yellow.y, yellow.z)*0.9;

    // io
    Button *loadMeshButton = new Button(nanoguiWindow, "Load Mesh");
    loadMeshButton->setFixedHeight(25);
    loadMeshButton->setCallback([] {
        objPath = file_dialog({{"obj", "Wavefront OBJ file"}}, false);
        if (objPath.empty()) return;
        try {
            clearScene();
            initHalfEdgeMesh();
            initMeshes();
            initBFF();

        } catch (...) {
            reportError("Unable to load file: " + objPath);
        }
    });

    Button *exportMeshButton = new Button(nanoguiWindow, "Export Mesh");
    exportMeshButton->setFixedHeight(25);
    exportMeshButton->setCallback([] {
        string file = file_dialog({{"obj", "Wavefront OBJ file"}}, true);
        if (file.empty()) return;
        try {
            if (!halfEdgeMesh.write(file, mappedToSphere, exportNormalizedUVsCheckBox->checked())) {
                reportError("Unable to write file: " + file);
            }

        } catch (...) {
            reportError("Unable to write file: " + file);
        }
    });

    exportNormalizedUVsCheckBox = new CheckBox(nanoguiWindow, "Export Normalized UVs");
    exportNormalizedUVsCheckBox->setFixedHeight(14);

    // plot
    plotLabel = new Label(nanoguiWindow, "Plot", "sans");
    ComboBox *plotComboBox = new ComboBox(nanoguiWindow, {"      Constant      ",
                                                          "       Shaded       ",
                                                          "Conformal Distortion",
                                                          "   Area Distortion  "});
    plotComboBox->setSelectedIndex(static_cast<int>(plotType));
    plotComboBox->setFixedHeight(25);
    plotComboBox->setCallback([](int state) {
        switch (state) {
            case 0:
                plotType = PlotType::constant;
                break;
            case 1:
                plotType = PlotType::shaded;
                break;
            case 2:
                plotType = PlotType::quasiConformalError;
                break;
            case 3:
                plotType = PlotType::areaScaling;
                break;
        }

        if (loadedHalfEdgeMesh) {
            colorsNeedUpdate = true;
            update(false);
        }
    });


    // check size
    patternComboBox = new ComboBox(nanoguiWindow, {"     None     ",
                                                   "     Grid     ",
                                                   " Checkerboard ",
                                                   "    Circles   "});
    patternComboBox->setSelectedIndex(1);
    patternComboBox->setFixedHeight(25);

    Slider *patternSizeSlider = new Slider(nanoguiWindow);
    patternSizeSlider->setRange(make_pair(0.0, 5.0));
    patternSizeSlider->setValue(patternSize);
    patternSizeSlider->setFixedHeight(14);
    patternSizeSlider->setCallback([](float s) { patternSize = s; });

    // wireframe
    CheckBox *wireframeCheckBox = new CheckBox(nanoguiWindow, "Show Wireframe");
    wireframeCheckBox->setCallback([](int state) { showWireframe = !showWireframe; });
    wireframeCheckBox->setChecked(showWireframe);
    wireframeCheckBox->setFixedHeight(14);

    // target boundary
    new Label(nanoguiWindow, "Target Boundary", "sans");
    targetBoundaryComboBox = new ComboBox(nanoguiWindow, {"     Automatic     ",
                                                          "        Disk       ",
                                                          "   Edit Boundary   ",
                                                          "Set Boundary Angles"});
    targetBoundaryComboBox->setSelectedIndex(0);
    targetBoundaryComboBox->setFixedHeight(25);
    targetBoundaryComboBox->setCallback([](int state) {
        if (loadedHalfEdgeMesh) {
            if (surfaceIsClosed) {
                reportError("Only supported on surfaces with boundary");

            } else {
                editBoundary = false;
                if (setBoundaryAngles && state != 3) {
                    removeVertexHandles();
                    angleSlider->setRange(make_pair(-4.0, 1.0));
                    setBoundaryAngles = false;

                } else if (!setBoundaryAngles && state == 3) {
                    if (handles.size() > 0) removeVertexHandles();
                    setBoundaryAngles = true;
                    angleSlider->setRange(make_pair(-0.9, 0.9));
                    setDefaultBoundaryAngles();
                }

                if (state != 2) {
                    flattenWithTargetBoundary(static_cast<BoundaryType>(state));

                } else if (state == 2) {
                    removeVertexHandles();
                    spline.reset();
                    editBoundary = true;
                    targetData = DenseMatrix(bff->data->bN);
                    flattenWithSplineBoundary(true, true);
                }

                placeConesButton->setEnabled(!setBoundaryAngles && state != 2);
            }
        }
    });

    // cones
    angleControlLabel = new Label(nanoguiWindow, "Angle Control             Sum:0π", "sans");
    angleTextBox = new TextBox(nanoguiWindow);
    angleTextBox->setUnits("π");
    angleTextBox->setFormat("[-]?[0-9]*\\.?[0-9]+");
    angleTextBox->setFixedHeight(25);
    angleTextBox->setCallback([](const string& str)->bool {
        if (str == "") return false;

        double angle = stod(str);
        angleSlider->setValue(angle);
        if (setBoundaryAngles) {
            if (angle >= 1.0 || angle <= -1.0) {
                reportError("Boundary angle less than -π and greater than π will result in a bad quality mapping. Hit Reset to undo.");
            }
            
            boundaryAngles(bff->data->bIndex[selectedVertexHandle->wedge()]) = angle*M_PI;
            flattenWithBoundaryAnglesLRU();

        } else {
            if (angle >= 2.0) {
                reportError("Cone angle must be less than 2π.");
                return false;
            }
            
            coneAngles(bff->data->index[selectedVertexHandle->wedge()]) = angle*M_PI;
            if (!surfaceIsClosed || (surfaceIsClosed && handles.size() > 2)) flattenWithCones();
        }

        updateAngleControlLabel();

        return true;
    });

    angleSlider = new Slider(nanoguiWindow);
    angleSlider->setRange(make_pair(-4.0, 1.0));
    angleSlider->setValue(0.5);
    angleSlider->setFixedHeight(14);
    angleSlider->setCallback([](float angle) {
        updateAngleTextBoxAndSliderValue(angle);

        if (setBoundaryAngles) {
            boundaryAngles(bff->data->bIndex[selectedVertexHandle->wedge()]) = angle*M_PI;
            flattenWithBoundaryAnglesLRU();
            
        } else {
            coneAngles(bff->data->index[selectedVertexHandle->wedge()]) = angle*M_PI;
            if (!surfaceIsClosed || (surfaceIsClosed && handles.size() > 2)) flattenWithCones();
        }

        updateAngleControlLabel();
    });

    // cone placement
    new Label(nanoguiWindow, "Automatic Cone Placement (slow)", "sans");
    vertexHandleCountTextBox = new TextBox(nanoguiWindow);
    vertexHandleCountTextBox->setUnits("# Cones");
    vertexHandleCountTextBox->setValue("8");
    vertexHandleCountTextBox->setFormat("[0-9]*");
    vertexHandleCountTextBox->setEditable(true);
    vertexHandleCountTextBox->setFixedHeight(25);
    vertexHandleCountTextBox->setCallback([](const string& str)->bool {
        if (str == "") return false;

        int S = stoi(str);
        if (surfaceIsClosed && S < 3) {
            reportError("Place at least 3 cones to flatten the mesh. It's not possible to cut a closed mesh with fewer than 2 cones. Placing only 2 cones will result in a bad quality mapping.");
            return false;
        }

        if (S <= (int)halfEdgeMesh.vertices.size() - bff->data->bN) return true;

        reportError("Number of cones cannot exceed interior vertices on the mesh");
        return false;
    });

    placeConesButton = new Button(nanoguiWindow, "Place Cones");
    placeConesButton->setFixedHeight(25);
    placeConesButton->setCallback([] {
        int S = stoi(vertexHandleCountTextBox->value());
        placeCones(S);
        if (!surfaceIsClosed && S == 0) {
            BoundaryType type = static_cast<BoundaryType>(targetBoundaryComboBox->selectedIndex());
            flattenWithTargetBoundary(type);
        
        } else {
            flattenWithCones();
        }
    });
    
    Button *removeSeamsButton = new Button(nanoguiWindow, "Remove Seams (BETA)");
    removeSeamsButton->setFixedHeight(25);
    removeSeamsButton->setEnabled(true);
    
    new Label(nanoguiWindow, "Spherical Parameterization");
    Button *sphericalParameterizationButton = new Button(nanoguiWindow, "Map to Sphere");
    sphericalParameterizationButton->setFixedHeight(25);
    sphericalParameterizationButton->setCallback([] { mapToSphere(); });

    new Label(nanoguiWindow, "");
    Button *resetButton = new Button(nanoguiWindow, "Reset");
    resetButton->setFixedHeight(25);
    resetButton->setCallback([] {
        if (!setBoundaryAngles && handles.size() > 0) {
            removeVertexHandles();
            BoundaryType type = static_cast<BoundaryType>(targetBoundaryComboBox->selectedIndex());
            flattenWithTargetBoundary(type);
            
        } else if (setBoundaryAngles) {
            removeVertexHandles();
            setDefaultBoundaryAngles();
            flattenWithTargetBoundary(BoundaryType::setBoundaryAngles);
            
        } else if (editBoundary) {
            knots.clear();
            selectedKnot = halfEdgeMesh.wedges().end();
            splineHandles.clear();
            splineLines.clear();
            spline.knots.clear();
            initSpline();
            targetData = DenseMatrix(bff->data->bN);
            flattenWithSplineBoundary(true);
        }

        mappedToSphere = false;
        cameras[1]->reset();
        updateCameraZoom();
        updatePlotLabel();
    });

    // help text
    instructionText = new Label(nanoguiWindow, "", "sans", 15);
    instructionText->setFixedSize(Eigen::Vector2i(190, 57));
    instructionText->setColor(highlightColor);

    gui->setVisible(true);
    gui->performLayout();
}

void Viewer::initGL()
{
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
}

void Viewer::initShaders()
{
    modelShader.load(modelVert, modelFrag);
    flatShader.load(flatVert, flatFrag);
    wireframeShader.load(wireframeVert, wireframeFrag);
}

void Viewer::initCameras()
{
    for (int i = 0; i < 2; i++) cameras.push_back(shared_ptr<Camera>(new Camera(EYE)));
}

void Viewer::initTransforms()
{
    // get transform indices
    GLuint modelIndex = glGetUniformBlockIndex(modelShader.program, "Transform");
    GLuint flatIndex = glGetUniformBlockIndex(flatShader.program, "Transform");
    GLuint wireframeIndex = glGetUniformBlockIndex(wireframeShader.program, "Transform");

    // bind
    glUniformBlockBinding(modelShader.program, modelIndex, 1);
    glUniformBlockBinding(flatShader.program, flatIndex, 1);
    glUniformBlockBinding(wireframeShader.program, wireframeIndex, 1);

    // create transform buffer
    glGenBuffers(1, &transformUbo);
    glBindBuffer(GL_UNIFORM_BUFFER, transformUbo);
    glBindBufferBase(GL_UNIFORM_BUFFER, 1, transformUbo);
    glBufferData(GL_UNIFORM_BUFFER, 4*sizeof(glm::mat4), NULL, GL_DYNAMIC_DRAW);

    // set the model matrix to the identity (will never change)
    glBufferSubData(GL_UNIFORM_BUFFER, 3*sizeof(glm::mat4), sizeof(glm::mat4), glm::value_ptr(glm::mat4()));

    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    // eye
    modelShader.use();
    glUniform3f(glGetUniformLocation(modelShader.program, "eye"), 0.0, 0.0, -10.0);
}

void Viewer::initLights()
{
    // get light index
    GLuint modelIndex = glGetUniformBlockIndex(modelShader.program, "Light");

    // bind
    glUniformBlockBinding(modelShader.program, modelIndex, 2);

    // add light data
    glGenBuffers(1, &lightUbo);
    glBindBuffer(GL_UNIFORM_BUFFER, lightUbo);
    glBindBufferBase(GL_UNIFORM_BUFFER, 2, lightUbo);
    glBufferData(GL_UNIFORM_BUFFER, 2*sizeof(glm::vec4), NULL, GL_STATIC_DRAW); // std140 alignment
    // light.position
    glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(glm::vec4), glm::value_ptr(glm::vec3(0.0, 2.0, -10.0)));
    // light.color
    glBufferSubData(GL_UNIFORM_BUFFER, sizeof(glm::vec4), sizeof(glm::vec4), glm::value_ptr(glm::vec3(1.0, 1.0, 1.0)));
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void Viewer::initHalfEdgeMesh()
{
    vector<VertexIter> cones = getHandleVertices();
    if ((loadedHalfEdgeMesh = halfEdgeMesh.read(objPath))) {
        // check if mesh contains more than 1 boundary
        if (halfEdgeMesh.boundaries.size() > 1) {
            reportError("Surfaces with holes not supported");
            loadedHalfEdgeMesh = false;

        } else if (halfEdgeMesh.boundaries.size() == 0) {
            surfaceIsClosed = true;

            // check if mesh has genus greater than zero
            if (halfEdgeMesh.eulerCharacteristic() != 2) {
                reportError("Surfaces with genus greater than 0 not supported");
                loadedHalfEdgeMesh = false;
            }
        }

        selectedKnot = halfEdgeMesh.wedges().end();
        selectedVertexHandle = halfEdgeMesh.vertices.end();
        if (handles.size() > 0) {
            for (VertexHandle handle: handles) {
                const Vector& uv = handle.vertex->wedge()->uv;
                const Vector& p = handle.vertex->position;
                addSphere(handle.sphere3D, glm::vec3(p.x, p.y, p.z), yellow);
                addSphere(handle.sphereUV, glm::vec3(uv.x, uv.y, uv.z), yellow);
            }
        }

    } else {
        reportError("Unable to load file: " + objPath);
    }
}

glm::vec3 elementColor(unsigned int pickedId)
{
    return glm::vec3(((pickedId & 0x000000FF) >>  0)/255.0,
                     ((pickedId & 0x0000FF00) >>  8)/255.0,
                     ((pickedId & 0x00FF0000) >> 16)/255.0);
}

void Viewer::initMeshes()
{
    if (loadedHalfEdgeMesh) {
        for (int i = 0; i < 2; i++) meshes.push_back(shared_ptr<RenderMesh>(new RenderMesh()));

        // initialize attributes for both rendermeshes
        int N = (int)halfEdgeMesh.corners.size();
        shared_ptr<Buffer> uvs = shared_ptr<Buffer>(new Buffer(N));
        shared_ptr<Buffer> normals = shared_ptr<Buffer>(new Buffer(N));
        shared_ptr<Buffer> barycenters = shared_ptr<Buffer>(new Buffer(N));
        meshes[0]->positions = shared_ptr<Buffer>(new Buffer(N));
        meshes[0]->uvs = uvs;
        meshes[0]->normals = normals;
        meshes[0]->colors = shared_ptr<Buffer>(new Buffer(N));
        meshes[0]->barycenters = barycenters;
        meshes[0]->pickPositions = shared_ptr<Buffer>(new Buffer(6*N));
        meshes[0]->pickColors = shared_ptr<Buffer>(new Buffer(6*N));
        meshes[1]->positions = uvs;
        meshes[1]->uvs = uvs;
        meshes[1]->normals = normals;
        meshes[1]->colors = shared_ptr<Buffer>(new Buffer(N));
        meshes[1]->barycenters = barycenters;

        vector<Vector> barycenter = {Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0), Vector(0.0, 0.0, 1.0)};
        for (FaceCIter f = halfEdgeMesh.faces.begin(); f != halfEdgeMesh.faces.end(); f++) {
            int index = 3*f->index;

            // set positions, normals and colors for the 1st rendermesh
            // and barycenters for both rendermeshes
            int i = 0;
            HalfEdgeCIter h = f->he;
            do {
                VertexCIter v = h->vertex;
                Vector normal = v->normal();
                for (int j = 0; j < 3; j++) {
                    meshes[0]->positions->buffer[index + i][j] = v->position[j];
                    meshes[0]->colors->buffer[index + i][j] = white[j];
                    normals->buffer[index + i][j] = normal[j];
                    barycenters->buffer[index + i][j] = barycenter[i][j];
                }

                i++;
                h = h->next;
            } while (h != f->he);
        }

        // define picking region for each vertex to be the barycentric dual cell
        for (CornerCIter c = halfEdgeMesh.corners.begin(); c != halfEdgeMesh.corners.end(); c++) {
            int index = 6*c->index;
            VertexCIter v = c->vertex();
            glm::vec3 pickColor = elementColor(v->index);

            // get the three vertices in the triangle
            Vector p1 = v->position;
            Vector p2 = c->next()->vertex()->position;
            Vector p3 = c->prev()->vertex()->position;

            // get the edge and triangle midpoints
            Vector m12 = (p1 + p2)/2.0;
            Vector m13 = (p1 + p3)/2.0;
            Vector m123 = (p1 + p2 + p3)/3.0;

            // draw the quad making up the dual region as a pair of triangles
            vector<Vector> tris = {p1, m12, m123,
                                   p1, m123, m13};

            // give all the triangles the same pick color as this corner
            for (int i = 0; i < 6; i++) {
                meshes[0]->pickPositions->buffer[index + i] = glm::vec3(tris[i].x, tris[i].y, tris[i].z);
                meshes[0]->pickColors->buffer[index + i] = pickColor;
            }
        }

        uvs->update();
        normals->update();
        barycenters->update();
        meshes[0]->update({POSITION, COLOR, PICK_POSITION, PICK_COLOR});
        meshes[1]->update({COLOR});
    }
}

void Viewer::initSpline()
{
    // place 4 handles on the boundary
    int n = 0;
    int nHandles = 0;
    int bN4 = (int)floor(bff->data->bN/4);
    for (WedgeIter w: halfEdgeMesh.cutBoundary()) {
        w->knot = spline.knots.end();

        if (n%bN4 == 0 && nHandles < 4) {
            addKnot(w);
            nHandles++;
        }

        n++;
    }
}

void Viewer::initBFF()
{
    if (loadedHalfEdgeMesh) {
        // flatten with isometric lengths
        bff = shared_ptr<BFF>(new BFF(halfEdgeMesh));
        boundaryAngles = DenseMatrix(bff->data->bN);
        coneAngles = DenseMatrix(bff->data->iN);
        
        initSpline();
        if (!surfaceIsClosed) flattenWithTargetBoundary(BoundaryType::automatic);
    }

    // update gui
    targetBoundaryComboBox->setSelectedIndex(0);
    setBoundaryAngles = false;
    vertexHandleCountTextBox->setValue("8");
    angleTextBox->setValue("Angle");
    angleTextBox->setEditable(false);
    angleSlider->setEnabled(false);
    angleControlLabel->setCaption("Angle Control             Sum:0π");
    placeConesButton->setEnabled(true);
    updateInstructionText();
    updatePlotLabel();
}

void Viewer::clearScene()
{
    meshes.clear();
    splineHandles.clear();
    splineLines.clear();
    spline.knots.clear();
    knots.clear();
    handles.clear();
    cut = NULL;
    bff = NULL;

    loadedHalfEdgeMesh = false;
    surfaceIsClosed = false;
    editBoundary = false;
    cutSurface = true;
    mappedToSphere = false;
    colorsNeedUpdate = true;
    cameras[1]->reset();
}

void Viewer::clear()
{
    clearScene();
    glDeleteBuffers(1, &transformUbo);
    glDeleteBuffers(1, &lightUbo);

    exit(0);
}

void Viewer::clearViewport(const ViewPane& pane)
{
    // set viewport
    glViewport(pane.left, pane.bottom, pane.width, pane.height);
    glScissor(pane.left, pane.bottom, pane.width, pane.height);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_SCISSOR_TEST);

    // clear scene
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDepthFunc(GL_LEQUAL);
}

void Viewer::updatePlotLabel()
{
    if (plotType == PlotType::constant || plotType == PlotType::shaded) {
        plotLabel->setCaption("Plot");

    } else if (!surfaceIsClosed || (surfaceIsClosed && cut) || mappedToSphere) {
        if (plotType == PlotType::quasiConformalError) {
            stringstream label;
            Vector error = Distortion::computeQuasiConformalError(halfEdgeMesh.faces);
            label << setprecision(4) << "Mean:" << error[2] << "   Max:" << error[1];
            plotLabel->setCaption("Plot     " + label.str());

        } else if (plotType == PlotType::areaScaling) {
            stringstream label;
            Vector error = Distortion::computeAreaScaling(halfEdgeMesh.faces);
            label << setprecision(2) << "Min:" << exp(error[0]) << "   Max:" << exp(error[1]);
            plotLabel->setCaption("Plot     " + label.str());
        }

    } else {
        plotLabel->setCaption("Plot");
    }
}

void Viewer::updateAngleControlLabel()
{
    double angleSum = setBoundaryAngles ? boundaryAngles.sum() : coneAngles.sum();
    angleSum /= M_PI;
    stringstream angleSumLabel;
    angleSumLabel << setprecision(3) << angleSum;
    angleControlLabel->setCaption("Angle Control             Sum:" + angleSumLabel.str() + "π");
}

void Viewer::updateAngleTextBoxAndSliderValue(double angle)
{
    stringstream angleLabel;
    angleLabel << setprecision(3) << angle;
    angleTextBox->setValue(angleLabel.str());
    angleSlider->setValue(angle);
}

void Viewer::updateUniforms(int index, int width, int height)
{
    // set camera transformations
    glm::mat4 projection = cameras[index]->projectionMatrix(width, height);
    glm::mat4 view = cameras[index]->viewMatrix();
    glm::mat4 view3D = cameras[0]->viewMatrix();

    glBindBuffer(GL_UNIFORM_BUFFER, transformUbo);
    // projection matrix
    glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(glm::mat4), glm::value_ptr(projection));
    // view matrix
    glBufferSubData(GL_UNIFORM_BUFFER, sizeof(glm::mat4), sizeof(glm::mat4), glm::value_ptr(view));
    // 3D view matrix
    glBufferSubData(GL_UNIFORM_BUFFER, 2*sizeof(glm::mat4), sizeof(glm::mat4), glm::value_ptr(view3D));
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    // set check factor, check size and draw uv sphere is mapped to sphere
    if (index == 0 || (index == 1 && plotType == PlotType::shaded)) {
        modelShader.use();
        glUniform1f(glGetUniformLocation(modelShader.program, "patternFactor"), patternFactor);
        glUniform1f(glGetUniformLocation(modelShader.program, "patternSize"), exp(patternSize));
        glUniform1i(glGetUniformLocation(modelShader.program, "drawUVSphere"), mappedToSphere);
        glUniform1i(glGetUniformLocation(modelShader.program, "pattern"), patternComboBox->selectedIndex());
    }
}

void Viewer::updateMeshes()
{
    // set uvs for the 1st rendermesh and positions and colors for 2nd rendermesh
    shared_ptr<Buffer> uvs = meshes[0]->uvs;
    for (FaceCIter f = halfEdgeMesh.faces.begin(); f != halfEdgeMesh.faces.end(); f++) {
        int index = 3*f->index;

        WedgeCIter w0 = f->he->wedge();
        WedgeCIter w1 = w0->next();
        WedgeCIter w2 = w1->next();
        const Vector& p0 = w0->uv;
        const Vector& p1 = w1->uv;
        const Vector& p2 = w2->uv;
        double s = cross(p1 - p0, p2 - p0).z;

        int i = 0;
        HalfEdgeCIter h = f->he;
        do {
            WedgeCIter w = h->next->wedge();
            glm::vec3 color;

            if (plotType == PlotType::constant) {
                if (s > 0.0 || mappedToSphere) color = skyblue;
                else color = red;

            } else if (plotType == PlotType::shaded) {
                color = white;

            } else if (plotType == PlotType::quasiConformalError) {
                Vector distortion = Distortion::color(f->index, true);
                color = glm::vec3(distortion.x, distortion.y, distortion.z);

            } else if (plotType == PlotType::areaScaling) {
                Vector distortion = Distortion::color(f->index, false);
                color = glm::vec3(distortion.x, distortion.y, distortion.z);
            }

            for (int j = 0; j < 3; j++) {
                uvs->buffer[index + i][j] = w->uv[j];
                meshes[0]->colors->buffer[index + i][j] = color[j];
                meshes[1]->colors->buffer[index + i][j] = color[j];
            }

            i++;
            h = h->next;
        } while (h != f->he);
    }

    uvs->update(); // buffer is shared by both rendermeshes
    if (colorsNeedUpdate) {
        meshes[0]->update({COLOR});
        meshes[1]->update({COLOR});
        colorsNeedUpdate = false;
    }
}

void Viewer::updateSphere(shared_ptr<RenderMesh> sphere, const glm::vec3& center, const glm::vec3& color)
{
    const int nSlices = 10;
    const int nStacks = 10;
    const float dTheta = 2*M_PI/nSlices;
    const float dPhi = M_PI/nSlices;
    const float radius = 0.02;

    if (!sphere->positions) sphere->positions = shared_ptr<Buffer>(new Buffer());
    else sphere->positions->buffer.clear();
    if (!sphere->colors) sphere->colors = shared_ptr<Buffer>(new Buffer());
    else sphere->colors->buffer.clear();

    double phi = 0.0;
    for (int i = 0; i < nSlices; i++) {
        double theta = 0.0;
        for (int j = 0; j < nStacks; j++) {
            glm::vec3 n00(cos(theta + 0*dTheta)*sin(phi + 0*dPhi),
                          sin(theta + 0*dTheta)*sin(phi + 0*dPhi),
                          cos(phi + 0*dPhi));
            glm::vec3 n01(cos(theta + 1*dTheta)*sin(phi + 0*dPhi),
                          sin(theta + 1*dTheta)*sin(phi + 0*dPhi),
                          cos(phi + 0*dPhi));
            glm::vec3 n10(cos(theta + 0*dTheta)*sin(phi + 1*dPhi),
                          sin(theta + 0*dTheta)*sin(phi + 1*dPhi),
                          cos(phi + 1*dPhi));
            glm::vec3 n11(cos(theta + 1*dTheta)*sin(phi + 1*dPhi),
                          sin(theta + 1*dTheta)*sin(phi + 1*dPhi),
                          cos(phi + 1*dPhi));
            glm::vec3 p00 = center + radius*n00;
            glm::vec3 p01 = center + radius*n01;
            glm::vec3 p10 = center + radius*n10;
            glm::vec3 p11 = center + radius*n11;

            sphere->positions->buffer.push_back(p00);
            sphere->positions->buffer.push_back(p01);
            sphere->positions->buffer.push_back(p10);
            sphere->positions->buffer.push_back(p10);
            sphere->positions->buffer.push_back(p01);
            sphere->positions->buffer.push_back(p11);
            for (int k = 0; k < 6; k++) sphere->colors->buffer.push_back(color);

            theta += dTheta;
        }
        phi += dPhi;
    }

    sphere->update({POSITION, COLOR});
}

void Viewer::addSphere(shared_ptr<RenderMesh>& sphere,
                       const glm::vec3& center, const glm::vec3& color)
{
    sphere = shared_ptr<RenderMesh>(new RenderMesh());
    updateSphere(sphere, center, color);
}

void Viewer::addSphere(vector<shared_ptr<RenderMesh>>& spheres,
                       const glm::vec3& center, const glm::vec3& color)
{
    shared_ptr<RenderMesh> sphere = shared_ptr<RenderMesh>(new RenderMesh());
    updateSphere(sphere, center, color);
    spheres.push_back(sphere);
}

void Viewer::updateLine(shared_ptr<RenderMesh> line, const glm::vec3& color, float width,
                        float x1, float y1, float x2, float y2, float z)
{
    if (!line->positions) line->positions = shared_ptr<Buffer>(new Buffer());
    else line->positions->buffer.clear();
    if (!line->colors) line->colors = shared_ptr<Buffer>(new Buffer());
    else line->colors->buffer.clear();

    glm::vec3 n = glm::normalize(glm::vec3(-(y2 - y1), x2 - x1, 0.0f));
    glm::vec3 a = glm::vec3(x1, y1, z);
    glm::vec3 b = glm::vec3(x2, y2, z);

    glm::vec3 p00 = a + width*n;
    glm::vec3 p01 = b + width*n;
    glm::vec3 p10 = b - width*n;
    glm::vec3 p11 = a - width*n;

    line->positions->buffer = {p00, p01, p10, p00, p10, p11};
    for (int k = 0; k < 6; k++) line->colors->buffer.push_back(color);
    line->update({POSITION, COLOR});
}

void Viewer::addLine(vector<shared_ptr<RenderMesh>>& lines, const glm::vec3& color, float width,
                     float x1, float y1, float x2, float y2, float z)
{
    shared_ptr<RenderMesh> line = shared_ptr<RenderMesh>(new RenderMesh());
    updateLine(line, color, width, x1, y1, x2, y2, z);
    lines.push_back(line);
}

void Viewer::updateCut()
{
    vector<VertexIter> cones = getHandleVertices();
    Cutter::cut(cones, halfEdgeMesh);

    cut = shared_ptr<RenderMesh>(new RenderMesh());
    cut->positions = shared_ptr<Buffer>(new Buffer());
    cut->colors = shared_ptr<Buffer>(new Buffer());
    for (EdgeCIter e = halfEdgeMesh.edges.begin(); e != halfEdgeMesh.edges.end(); e++) {
        if (e->onCut) {
            HalfEdgeCIter he = e->he;

            Vector n = 0.5*(he->face->normal() + he->flip->face->normal());
            Vector a = he->vertex->position + 0.001*n;
            Vector b = he->flip->vertex->position + 0.001*n;
            Vector u = b - a;
            Vector v = cross(u, n);
            v.normalize();

            const double width = 0.002;
            Vector p00 = a + width*v;
            Vector p01 = b + width*v;
            Vector p10 = b - width*v;
            Vector p11 = a - width*v;
            vector<Vector> p = {p00, p01, p10, p00, p10, p11};

            for (int i = 0; i < 6; i++) {
                cut->positions->buffer.push_back(glm::vec3(p[i].x, p[i].y, p[i].z));
                cut->colors->buffer.push_back(red);
            }
        }
    }

    cut->update({POSITION, COLOR});
}

void Viewer::updateInstructionText()
{
    if (editBoundary) {
        instructionText->setCaption("Drag (+ shift) handles to change scaling (angles). Click (+ ctrl) vertices on the boundary to add (remove) handles.");

    } else if (setBoundaryAngles) {
        instructionText->setCaption("Click (+ ctrl) vertices on the boundary to set (clear) angle values.\nAlt + drag to pan camera.");

    } else {
        instructionText->setCaption("Click (+ ctrl) vertices on the interior to set (clear) cone angles.\nAlt + drag to pan camera.");
    }
}

void Viewer::updateViewPanes()
{
    surfacePane = ViewPane(framebufferWidth*0.17, 0, framebufferWidth*0.415, framebufferHeight);
    uvPane = ViewPane(framebufferWidth*0.585, 0, framebufferWidth*0.415, framebufferHeight);
    fullPane = ViewPane(0, 0, framebufferWidth, framebufferHeight);
}

void Viewer::updateCameraZoom()
{
    double radius = 0.0;
    for (WedgeCIter w = halfEdgeMesh.wedges().begin(); w != halfEdgeMesh.wedges().end(); w++) {
        radius = max(radius, w->uv.norm());
    }

    cameras[1]->radius = EYE*radius;
    cameras[1]->viewNeedsUpdate = true;
}

void Viewer::updateVertexHandles(VertexIter v)
{
    double angle = 0.5;
    if (handles.size() > 0) {
        VertexHandle handle = handles.front();
        angle = coneAngles(bff->data->index[handle.vertex->wedge()])/M_PI;
    }
    
    if (ctrlClick) {
        removeVertexHandle(v);
        
    } else {
        if (setBoundaryAngles || surfaceIsClosed) addVertexHandle(v, 0.0);
        else addVertexHandle(v, angle);
    }
}

void Viewer::updateVertexHandlesUVs()
{
    for (VertexHandle handle: handles) {
        VertexIter v = handle.vertex;
        glm::vec3 color = v == selectedVertexHandle ? orange : yellow;
        const Vector& uv = v->wedge()->uv;
        updateSphere(handle.sphereUV, glm::vec3(uv.x, uv.y, uv.z), color);
    }
}

void Viewer::update(bool updateZoom)
{
    // compute distortion
    switch (plotType) {
        case PlotType::constant:
            colorsNeedUpdate = true;
            break;
        case PlotType::quasiConformalError:
            Distortion::computeQuasiConformalError(halfEdgeMesh.faces);
            colorsNeedUpdate = true;
            break;
        case PlotType::areaScaling:
            Distortion::computeAreaScaling(halfEdgeMesh.faces);
            colorsNeedUpdate = true;
            break;
        default:
            break;
    }

    // update meshes and instructions
    updateMeshes();
    updateVertexHandlesUVs();
    updateInstructionText();
    updatePlotLabel();
    if (updateZoom) updateCameraZoom();
    draw();
}

void Viewer::addKnot(WedgeIter w)
{
    // update handle colors
    for (int i = 0; i < (int)splineHandles.size(); i++) {
        glm::vec3 color = knots[i] == w ? orange : yellow;
        for (int j = 0; j < (int)splineHandles[i]->colors->buffer.size(); j++) {
            splineHandles[i]->colors->buffer[j] = color;
        }

        for (int j = 0; j < (int)splineLines[2*i]->colors->buffer.size(); j++) {
            splineLines[2*i]->colors->buffer[j] = color;
            splineLines[2*i + 1]->colors->buffer[j] = color;
        }

        splineHandles[i]->update({COLOR});
        splineLines[2*i]->update({COLOR});
        splineLines[2*i + 1]->update({COLOR});
    }

    // add knot if it doesn't exist
    if (w->knot == spline.knots.end()) {
        double L = 0.0;
        for (WedgeCIter w: halfEdgeMesh.cutBoundary()) {
            L += w->he->next->edge->length();
        }

        double l = 0.0;
        for (WedgeCIter ww: halfEdgeMesh.cutBoundary()) {
            if (w == ww) break;
            l += ww->he->next->edge->length();
        }

        double s = 2*M_PI*l/L;
        w->knot = spline.addKnot(s, spline.evaluate(s));
        const Vector& uv = w->uv;
        Vector scaledT = 0.25*exp(scaling(w))*tangent(w);

        addSphere(splineHandles, glm::vec3(uv.x, uv.y, uv.z), orange);
        addLine(splineLines, orange, 0.005, uv.x, uv.y, uv.x + scaledT.x, uv.y + scaledT.y, uv.z);
        addLine(splineLines, orange, 0.005, uv.x, uv.y, uv.x - scaledT.x, uv.y - scaledT.y, uv.z);

        knots.push_back(w);
    }

    selectedKnot = w;
}

void Viewer::removeKnot(WedgeIter w)
{
    if (w->knot != spline.knots.end()) {
        // remove knot from spline
        spline.removeKnot(w->knot->first);
        w->knot = spline.knots.end();
        if (selectedKnot == w) selectedKnot = halfEdgeMesh.wedges().end();

        // update handle list
        int splineHandleIndex = -1;
        for (int i = 0; i < (int)splineHandles.size(); i++) {
            if (knots[i] == w) {
                splineHandleIndex = i;
                break;
            }
        }

        splineHandles.erase(splineHandles.begin() + splineHandleIndex);
        splineLines.erase(splineLines.begin() + 2*splineHandleIndex,
                          splineLines.begin() + 2*(splineHandleIndex + 1));
        knots.erase(knots.begin() + splineHandleIndex);

        // flatten
        flattenWithSplineBoundary(!shiftClick);
    }
}

void Viewer::dragKnot(double change)
{
    if (selectedKnot != halfEdgeMesh.wedges().end()) {
        selectedKnot->knot->second += 3.0*change;

        // flatten
        flattenWithSplineBoundary(!shiftClick);
    }
}

void Viewer::addVertexHandle(VertexIter v, double newAngle)
{
    if ((setBoundaryAngles && !v->onBoundary(false)) || (!setBoundaryAngles && v->onBoundary(false))) {
        return;
    }

    angleTextBox->setEditable(true);
    angleSlider->setEnabled(true);

    // check if vertex v already has a handle
    bool handleExists = false;
    VertexHandle existingHandle;
    for (VertexHandle handle: handles) {
        glm::vec3 color = yellow;

        if (handle.vertex == v) {
            existingHandle = handle;
            color = orange;
            selectedVertexHandle = handle.vertex;
            handleExists = true;

            float angle = setBoundaryAngles ?
                          boundaryAngles(bff->data->bIndex[selectedVertexHandle->wedge()]) :
                          coneAngles(bff->data->index[selectedVertexHandle->wedge()]);
            updateAngleTextBoxAndSliderValue(angle/M_PI);
        }

        for (int i = 0; i < (int)handle.sphere3D->colors->buffer.size(); i++) {
            handle.sphere3D->colors->buffer[i] = color;
            handle.sphereUV->colors->buffer[i] = color;
        }

        handle.sphere3D->update({COLOR});
        handle.sphereUV->update({COLOR});
    }

    if (handleExists) {
        // if vertex v already has a handle, just move it to the front of the list
        handles.remove(existingHandle);
        handles.push_front(existingHandle);

    } else {
        // otherwise, create a new handle for vertex v (putting it at the front of the list)
        selectedVertexHandle = v;

        VertexHandle handle;
        handle.vertex = v;
        const Vector& uv = v->wedge()->uv;
        const Vector& p = v->position;
        addSphere(handle.sphere3D, glm::vec3(p.x, p.y, p.z), orange);
        addSphere(handle.sphereUV, glm::vec3(uv.x, uv.y, uv.z), orange);
        handles.push_front(handle);

        if (setBoundaryAngles) {
            boundaryAngles(bff->data->bIndex[selectedVertexHandle->wedge()]) = newAngle*M_PI;

        } else {
            coneAngles(bff->data->index[selectedVertexHandle->wedge()]) = newAngle*M_PI;
            Cutter::glue(halfEdgeMesh);
            cutSurface = true;
            mapIsDirty = true;
        }

        updateAngleTextBoxAndSliderValue(newAngle);
        updateAngleControlLabel();
    }
}

void Viewer::removeVertexHandle(VertexIter v)
{
    if (surfaceIsClosed && handles.size() == 3) {
        reportError("At least 3 cones must be placed. It's not possible to cut a closed mesh with fewer than 2 cones. Placing only 2 cones will result in a bad quality mapping.");
        return;
    
    } else if (setBoundaryAngles && handles.size() == 3) {
        reportError("At least 3 handles must be placed. Fewer than 3 handles will result in a bad quality mapping.");
        return;
    }
    
    // check if the vertex has a handle
    for (VertexHandle handle: handles) {
        if (handle.vertex == v) {
            if (setBoundaryAngles) {
                boundaryAngles(bff->data->bIndex[handle.vertex->wedge()]) = 0.0;

            } else {
                coneAngles(bff->data->index[handle.vertex->wedge()]) = 0.0;
                Cutter::glue(halfEdgeMesh);
                cutSurface = true;
                mapIsDirty = true;
            }

            updateAngleControlLabel();

            // remove cone from cone list and update angle
            handles.remove(handle);
            if (selectedVertexHandle == v) selectedVertexHandle = halfEdgeMesh.vertices.end();

            // update interface
            if (selectedVertexHandle == halfEdgeMesh.vertices.end() || handles.size() == 0) {
                 selectedVertexHandle = halfEdgeMesh.vertices.end();
                 angleTextBox->setValue("Angle");
                 angleTextBox->setEditable(false);
                 angleSlider->setEnabled(false);
            }

            break;
        }
    }

    if (setBoundaryAngles) flattenWithBoundaryAnglesNormalize();
}

void Viewer::removeVertexHandles()
{
    if (loadedHalfEdgeMesh) {
        // clear all cones and set cone angles to 0
        handles.clear();
        selectedVertexHandle = halfEdgeMesh.vertices.end();
        for (int i = 0; i < (int)boundaryAngles.nRows(); i++) boundaryAngles(i) = 0.0;
        for (int i = 0; i < (int)coneAngles.nRows(); i++) coneAngles(i) = 0.0;

        // update interface
        angleTextBox->setValue("Angle");
        angleTextBox->setEditable(false);
        angleSlider->setEnabled(false);
        angleControlLabel->setCaption("Angle Control             Sum:0π");

        // clear cut
        if (cut) cut = NULL;

        // glue cut
        Cutter::glue(halfEdgeMesh);
        cutSurface = true;
    }
}

void Viewer::placeCones(int S)
{
    removeVertexHandles();
    if (loadedHalfEdgeMesh && S > 0) {
        mapIsDirty = true;
        vector<VertexIter> cones = getHandleVertices();
        ConePlacement::findConesAndPrescribeAngles(S, cones, coneAngles, bff->data, halfEdgeMesh);
        
        for (VertexIter v: cones) {
            VertexHandle handle;
            handle.vertex = v;
            const Vector& uv = v->wedge()->uv;
            const Vector& p = v->position;
            addSphere(handle.sphere3D, glm::vec3(p.x, p.y, p.z), yellow);
            addSphere(handle.sphereUV, glm::vec3(uv.x, uv.y, uv.z), yellow);
            handles.push_back(handle);
        }
    }
    
    updateAngleControlLabel();
}

void Viewer::normalizeConeAngles()
{
    static int nHandles = (int)handles.size();
    double deviation = 4*M_PI - coneAngles.sum();
    if (abs(deviation) > 1e-8) {
        if (cut && nHandles == (int)handles.size()) {
            // use least recently used update when changing cone angles
            VertexHandle handle = handles.back();
            int i = bff->data->index[handle.vertex->wedge()];
            coneAngles(i) += deviation;
            
        } else {
            // divide deviation equally when adding and removing cones
            for (VertexHandle handle: handles) {
                int i = bff->data->index[handle.vertex->wedge()];
                coneAngles(i) += deviation/(double)handles.size();
            }
        }

        if (selectedVertexHandle != halfEdgeMesh.vertices.end()) {
            double angle = coneAngles(bff->data->index[selectedVertexHandle->wedge()])/M_PI;
            updateAngleTextBoxAndSliderValue(angle);
        }

        updateAngleControlLabel();
    }
    
    nHandles = (int)handles.size();
}

void Viewer::setDefaultBoundaryAngles()
{
    // place 4 handles on the boundary
    int n = 0;
    int nHandles = 0;
    int bN4 = (int)floor(bff->data->bN/4);
    for (WedgeIter w: halfEdgeMesh.cutBoundary()) {
        if (n%bN4 == 0 && nHandles < 4) {
            addVertexHandle(w->vertex(), 0.5);
            nHandles++;
        }

        n++;
    }
}

vector<VertexIter> Viewer::getHandleVertices()
{
    vector<VertexIter> handleVertices;
    for (VertexHandle handle: handles) {
        handleVertices.push_back(handle.vertex);
    }

    return handleVertices;
}

void Viewer::performPCA()
{
    int wN = 0;
    Eigen::Matrix2d covariance;
    covariance.setZero();
    for (WedgeIter w = halfEdgeMesh.wedges().begin(); w != halfEdgeMesh.wedges().end(); w++) {
        Eigen::Vector2d uv(w->uv.x, w->uv.y);
        covariance += uv * uv.transpose();
        wN++;
    }

    covariance /= (double)wN;
    Eigen::EigenSolver<Eigen::Matrix2d> solver(covariance);
    Eigen::Matrix2d eigenVectors = -solver.eigenvectors().real();

    for (WedgeIter w = halfEdgeMesh.wedges().begin(); w != halfEdgeMesh.wedges().end(); w++) {
        Eigen::Vector2d uv(w->uv.x, w->uv.y);
        w->uv.x = eigenVectors.col(0).dot(uv);
        w->uv.y = eigenVectors.col(1).dot(uv);
    }
}

void Viewer::flattenWithTargetBoundary(BoundaryType type, bool updateZoom)
{
    if (loadedHalfEdgeMesh && !surfaceIsClosed) {
        if (type != BoundaryType::editBoundary) editBoundary = false;

        // flatten
        if (type == BoundaryType::automatic) {
            removeVertexHandles();
            DenseMatrix u(bff->data->bN);
            bff->flatten(u, true);
            performPCA();
            update();

        } else if (type == BoundaryType::setBoundaryAngles) {
            bff->flatten(boundaryAngles, false);
            update(updateZoom);

        } else if (type == BoundaryType::disk) {
            removeVertexHandles();
            bff->flattenToDisk();
            performPCA();
            update();
        }
    }
}

void Viewer::flattenWithBoundaryAnglesLRU()
{
    // add deviation to least recently used boundary handle
    VertexHandle handle = handles.back();

    double deviation = 2*M_PI - boundaryAngles.sum();
    if (abs(deviation) > 1e-8) {
        boundaryAngles(bff->data->bIndex[handle.vertex->wedge()]) += deviation;
        flattenWithTargetBoundary(BoundaryType::setBoundaryAngles, false);
    }
}

void Viewer::flattenWithBoundaryAnglesNormalize()
{
    // divide deviation equally
    double deviation = 2*M_PI - boundaryAngles.sum();
    if (abs(deviation) > 1e-8) {
        for (VertexHandle handle: handles) {
            int i = bff->data->bIndex[handle.vertex->wedge()];
            boundaryAngles(i) += deviation/(double)handles.size();
        }
        
        if (selectedVertexHandle != halfEdgeMesh.vertices.end()) {
            double angle = boundaryAngles(bff->data->bIndex[selectedVertexHandle->wedge()])/M_PI;
            updateAngleTextBoxAndSliderValue(angle);
        }
        
        updateAngleControlLabel();
        flattenWithTargetBoundary(BoundaryType::setBoundaryAngles);
    }
}

void Viewer::flattenWithSplineBoundary(bool givenScaleFactors, bool updateZoom)
{
    // set boundary data from spline
    double L = 0.0;
    for (WedgeCIter w: halfEdgeMesh.cutBoundary()) {
        L += w->he->next->edge->length();
    }

    double l = 0.0;
    DenseMatrix boundaryData(bff->data->bN);
    for (WedgeCIter w: halfEdgeMesh.cutBoundary()) {
        int i = bff->data->bIndex[w];
        double s = 2*M_PI*l/L;
        double offset = spline.evaluate(s);
        if (!givenScaleFactors) {
            // offset in curvature is the difference between offsets in angles
            offset = spline.evaluate(2*M_PI*(l + w->he->next->edge->length())/L) - offset;
        }

        boundaryData(i) = targetData(i) + offset;
        l += w->he->next->edge->length();
    }

    if (givenScaleFactors) {
        // flatten with target scale factors
        bff->flatten(boundaryData, true);

    } else {
        // flatten with target angles
        bff->closeCurvatures(boundaryData);
        bff->flatten(boundaryData, false);
    }

    // update spline handles
    for (int i = 0; i < (int)knots.size(); i++) {
        WedgeIter w = knots[i];
        const Vector& uv = w->uv;
        Vector scaledT = 0.25*exp(scaling(w))*tangent(w);
        glm::vec3 color = w == selectedKnot ? orange : yellow;

        updateSphere(splineHandles[i], glm::vec3(uv.x, uv.y, uv.z), color);
        updateLine(splineLines[2*i], color, 0.005, uv.x, uv.y, uv.x + scaledT.x, uv.y + scaledT.y, uv.z);
        updateLine(splineLines[2*i + 1], color, 0.005, uv.x, uv.y, uv.x - scaledT.x, uv.y - scaledT.y, uv.z);
    }

    update(updateZoom);
}

void Viewer::flattenWithCones()
{
    if (loadedHalfEdgeMesh) {
        // flatten
        if (surfaceIsClosed) normalizeConeAngles();
        if (cutSurface) updateCut();
        bff->flattenWithCones(coneAngles, cutSurface);

        // update
        cutSurface = false;
        mapIsDirty = false;
        editBoundary = false;
        if (mappedToSphere) cameras[1]->reset();
        mappedToSphere = false;
        update();
    }
}

bool intersect(Vector u1, Vector u2, Vector v1, Vector v2)
{
    double ux = u2.x - u1.x;
    double uy = u2.y - u1.y;
    double vx = v2.x - v1.x;
    double vy = v2.y - v1.y;

    double den = -vx*uy + ux*vy;
    double s = (-uy*(u1.x - v1.x) + ux*(u1.y - v1.y))/den;
    double t = (vx*(u1.y - v1.y) - vy*(u1.x - v1.x))/den;

    return s >= 0 && s <= 1 && t >= 0 && t <= 1;
}

bool selfIntersects(const vector<Vector>& gamma)
{
    int n = (int)gamma.size();
    for (int i = 1; i < n; i++) {
        for (int j = i + 2; j < n; j++) {
            if (intersect(gamma[i], gamma[(i + 1)%n], gamma[j], gamma[(j + 1)%n])) {
                return true;
            }
        }
    }

    return intersect(gamma[n - 2], gamma[n - 1], gamma[0], gamma[1]);
}

void Viewer::flattenToShape(const vector<Vector>& gamma)
{
    // flatten to target gamma if gamma does not self intersect
    if (gamma.size() > 2 && !selfIntersects(gamma)) {
        bff->flattenToShape(gamma);
        update();

    } else {
        reportError("Curve self intersects");
    }
}

void Viewer::mapToSphere()
{
    if (loadedHalfEdgeMesh && surfaceIsClosed) {
        // remove cones
        removeVertexHandles();

        // map to sphere
        bff->mapToSphere();
        mappedToSphere = true;
        patternComboBox->setSelectedIndex(3);

        // update
        update();

    } else {
        reportError("Only supported on surfaces without boundary");
    }
}

Vector Viewer::ndc(int x, int y, int width, int height) {
    double s = 0.17;
    double t = 0.585;
    if (surfaceIsClosed && !cut && !mappedToSphere) {
        s = 0.0;
        t = 0.0;
    }

    return Vector(2.0*(x - width*s)/(width*(1.0 - t)) - 1.0,
                  2.0*(x - width*t)/(width*(1.0 - t)) - 1.0,
                  1.0 - 2.0*y/height);
}

void Viewer::cursorPosCallback(GLFWwindow *w, double x, double y)
{
    // call gui callback event
    if (!gui->cursorPosCallbackEvent(x, y)) {
        glfwGetWindowSize(w, &windowWidth, &windowHeight);
        if (mouseDown) {
            // camera rotation, panning and dragging spline handle
            Vector current = ndc(x, y, windowWidth, windowHeight);
            if (current[2] > -1 && current[2] < 1) {
                float dy = current[2] - mouse[2];

                if (current[0] > -1 && current[0] < 1) {
                    float dx = current[0] - mouse[0];

                    if (altClick) cameras[0]->pan(dx, dy);
                    else cameras[0]->rotate(-dx*M_PI, dy*M_PI);

                } else if (current[1] > -1 && current[1] < 1) {
                    float dx = current[1] - mouse[1];

                    if (editBoundary) {
                        dragKnot(dx);

                    } else if (altClick) {
                        cameras[1]->pan(dx, dy);

                    } else {
                        if (mappedToSphere) {
                            cameras[1]->rotate(-dx*M_PI, dy*M_PI);

                        } else {
                            Vector u = origMouse - Vector(2, 0, 0);
                            Vector v = current - Vector(2, 0, 0);
                            u.y = 0.0;
                            v.y = 0.0;
                            u.x /= uvPane.height;
                            v.x /= uvPane.height;
                            u.z /= uvPane.width;
                            v.z /= uvPane.width;
                            double theta = atan2(cross(u, v).y, dot(u,v));
                            cameras[1]->worldUp.x = cos(theta)*origUp.x - sin(theta)*origUp.y;
                            cameras[1]->worldUp.y = sin(theta)*origUp.x + cos(theta)*origUp.y;
                            cameras[1]->viewNeedsUpdate = true;
                        }
                    }
                }
            }
        }

        mouse = ndc(x, y, windowWidth, windowHeight);
    }
}


void Viewer::mouseButtonCallback(GLFWwindow *w, int button, int action, int modifiers)
{
    // call gui callback event
    if (!gui->mouseButtonCallbackEvent(button, action, modifiers)) {
        // track mouse click
        double x, y;
        glfwGetCursorPos(w, &x, &y);
        mouse = ndc(x, y, windowWidth, windowHeight);
        origMouse = mouse;
        origUp.x = cameras[1]->worldUp.x;
        origUp.y = cameras[1]->worldUp.y;
        origUp.z = cameras[1]->worldUp.z;

        ctrlClick = (bool)(modifiers & GLFW_MOD_CONTROL);
        altClick = (bool)(modifiers & GLFW_MOD_ALT);

        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
            clickX = gui->pixelRatio()*x;
            clickY = gui->pixelRatio()*y;
            mouseDown = true;
            shiftClick = (bool)(modifiers & GLFW_MOD_SHIFT);

            // flatten with target angles
            if (editBoundary && shiftClick) {
                spline.reset();
                targetData = bff->compatibleTarget;
                flattenWithSplineBoundary(false);
            }

        } else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
            unclickX = gui->pixelRatio()*x;
            unclickY = gui->pixelRatio()*y;

            if (editBoundary && shiftClick) {
                // flatten with target scale factors
                spline.reset();
                targetData = bff->compatibleTarget;
                flattenWithSplineBoundary(true);

            } else {
                if (editBoundary) {
                    colorsNeedUpdate = true;
                    updateMeshes();
                }

                // draw scene so that we can perform picking
                pickingEnabled = true;
                drawScene();

                // flatten with cones
                if (!setBoundaryAngles && !editBoundary && mapIsDirty) {
                    if (handles.size() == 0) {
                        BoundaryType type = static_cast<BoundaryType>(targetBoundaryComboBox->selectedIndex());
                        flattenWithTargetBoundary(type);
                    
                    } else if (!surfaceIsClosed || (surfaceIsClosed && handles.size() > 2)) {
                        flattenWithCones();
                    }
                }
            }

            mouseDown = false;
            shiftClick = false;
        }
    }
}

void Viewer::keyCallback(GLFWwindow *w, int key, int scancode, int action, int mods)
{
    // call gui callback event
    gui->keyCallbackEvent(key, scancode, action, mods);

    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(w, GLFW_TRUE);
    }
}

void Viewer::scrollCallback(GLFWwindow *w, double x, double y)
{
    // call gui callback event
    gui->scrollCallbackEvent(x, y);

    if (mouse[0] > -1 && mouse[0] < 1) cameras[0]->zoom(0.1*y);
    else if (mouse[1] > -1 && mouse[1] < 1) cameras[1]->zoom(0.1*y);
}

void Viewer::charCallback(GLFWwindow *w, unsigned int codepoint)
{
    // call gui callback event
    gui->charCallbackEvent(codepoint);
}

void Viewer::framebufferSizeCallback(GLFWwindow *w, int width, int height)
{
    // call gui callback event
    gui->resizeCallbackEvent(width, height);

    // update drawing
    draw();
}

double Viewer::pickDistance(int index, const Vector& position, const ViewPane& pane)
{
    glm::vec3 projected = glm::project(glm::vec3(position.x, position.y, position.z),
                                       cameras[index]->viewMatrix(),
                                       cameras[index]->projectionMatrix(pane.width, pane.height),
                                       glm::vec4(pane.left, pane.bottom, pane.width, pane.height));

    double dx = clickX - projected[0];
    double dy = framebufferHeight - clickY - projected[1];

    return sqrt(dx*dx + dy*dy);
}

bool Viewer::didPick(int index, const Vector& position, const ViewPane& pane)
{
    double d = pickDistance( index, position, pane );
    if (d < 6*EYE/cameras[index]->radius) {
        return true;
    }

    return false;
}

WedgeIter Viewer::closestWedge(const ViewPane& pane, bool onBoundary)
{
    double minDistance = numeric_limits<double>::max();
    WedgeIter minWedge;

    if (onBoundary) {
        for (WedgeIter w: halfEdgeMesh.cutBoundary()) {
            double distance = pickDistance(1, w->uv, pane);

            if (distance < minDistance) {
                minDistance = distance;
                minWedge = w;
            }
        }

    } else {
        for (WedgeIter w  = halfEdgeMesh.wedges().begin(); w != halfEdgeMesh.wedges().end(); w ++) {
            double distance = pickDistance(1, w->uv, pane);

            if (distance < minDistance) {
                minDistance = distance;
                minWedge = w;
            }
        }
    }

    return minWedge;
}

void Viewer::pickKnot(const ViewPane& pane)
{
    WedgeIter w = closestWedge(pane, true);
    if (ctrlClick) removeKnot(w);
    else addKnot(w);
}

void Viewer::pickVertexHandleUV(const ViewPane& pane)
{
    WedgeCIter w = closestWedge(pane, setBoundaryAngles);

    if ((setBoundaryAngles && w->vertex()->onBoundary(false)) ||
        (!setBoundaryAngles && !w->vertex()->onBoundary(false))) {
        updateVertexHandles(w->vertex());
    }
}

bool Viewer::pickVertexHandle(const ViewPane& pane)
{
    for (VertexHandle handle: handles) {
        VertexIter v = handle.vertex;

        if (didPick(0, v->position, pane) || didPick(1, v->wedge()->uv, pane)) {
            updateVertexHandles(v);
            return true;
        }
    }

    return false;
}

void Viewer::processPickedElement(int index, int pickedId)
{
    int nV = (int)halfEdgeMesh.vertices.size();
    if (pickedId < nV) {
        if (index == 0) {
            VertexIter v = halfEdgeMesh.vertices.begin() + pickedId;
            updateVertexHandles(v);
        }
    }
}

void Viewer::pick(int index)
{
    // draw
    meshes[index]->draw(flatShader, true);

    // flush
    glFlush();
    glFinish();
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    // read color
    unsigned char data[4];
    glReadPixels(clickX, framebufferHeight - clickY, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, data);

    // convert color to id
    int pickedId = data[0] + data[1]*256 + data[2]*256*256;
    if (pickedId != MAX_PICKED_ID) processPickedElement(index, pickedId);
}

void Viewer::drawRenderMesh(const ViewPane& pane, int index)
{
    // update uniforms
    updateUniforms(index, pane.width, pane.height);

    if (pickingEnabled) {
        if (clickX >= pane.left && clickX <= pane.left + pane.width &&
            clickX == unclickX && clickY == unclickY) {
            if (!pickVertexHandle(pane)) {
                if (index == 0 && !editBoundary) {
                    // clear viewport and pick
                    clearViewport(pane);
                    pick(index);

                } else if (index == 1 && !mappedToSphere) {
                    if (editBoundary) pickKnot(pane);
                    else pickVertexHandleUV(pane);
                }
            }
        }
    }

    // clear viewport
    clearViewport(pane);

    // draw mesh
    if (index == 0 || plotType == PlotType::shaded) {
        meshes[index]->draw(modelShader);

    } else {
        meshes[index]->draw(flatShader);

        if (plotType == PlotType::constant) {
            glEnable(GL_CULL_FACE);
            glCullFace(GL_FRONT);
            glDisable(GL_DEPTH_TEST);
            meshes[index]->draw(flatShader);
            glEnable(GL_DEPTH_TEST);
            glDisable(GL_CULL_FACE);
        }
    }

    // draw cone and spline handles
    if (index == 0) {
        for (VertexHandle handle: handles) {
            handle.sphere3D->draw( flatShader );
        }

    } else if (index == 1) {
        for (VertexHandle handle: handles) {
            handle.sphereUV->draw( flatShader );
        }

        if (editBoundary) {
            for (int i = 0; i < (int)splineHandles.size(); i++) {
                splineHandles[i]->draw(flatShader);
                splineLines[2*i]->draw(flatShader);
                splineLines[2*i + 1]->draw(flatShader);
            }
        }
    }

    // draw wireframe and cut
    bool showCut = index == 0 && cut;
    if (showWireframe || showCut) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        if (showWireframe) meshes[index]->draw(wireframeShader);
        if (showCut) cut->draw(flatShader);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
}

void Viewer::drawScene()
{
    clearViewport(fullPane);

    if (loadedHalfEdgeMesh) {
        if (!surfaceIsClosed || (surfaceIsClosed && cut) || mappedToSphere) {
            patternFactor = 0.5;
            drawRenderMesh(surfacePane, 0);
            drawRenderMesh(uvPane, 1);

        } else {
            patternFactor = 0.0;
            drawRenderMesh(fullPane, 0);
        }
    }
}

void Viewer::draw()
{
    glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
    updateViewPanes();
    pickingEnabled = false;

    // draw scene
    drawScene();

    // draw gui
    gui->drawWidgets();

    // swap buffers
    glfwSwapBuffers(window);
}

void Viewer::reportError(const string& error)
{
    new MessageDialog(gui, MessageDialog::Type::Warning, "Error", error);
}
