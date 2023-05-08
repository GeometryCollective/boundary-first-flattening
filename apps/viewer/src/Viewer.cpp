#include "Viewer.h"
#include "bff/mesh/MeshIO.h"
#include "bff/project/HoleFiller.h"
#include "bff/project/Generators.h"
#include "bff/project/ConePlacement.h"
#include "bff/project/Cutter.h"
#include "bff/project/Distortion.h"
#include "ShadersSource.h"
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
CheckBox* Viewer::useVtFlagToStoreUVsCheckBox = NULL;
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
float Viewer::patternSize = 2.0;

std::vector<std::shared_ptr<Camera>> Viewer::cameras;

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

std::string Viewer::objPath;
Model Viewer::model;
std::vector<ModelState> Viewer::modelStates;
Mesh* Viewer::mesh = NULL;
ModelState* Viewer::state = NULL;
int Viewer::selectedMesh = 0;
bool Viewer::loadedModel = false;

PlotType Viewer::plotType = PlotType::shaded;
bool Viewer::colorsNeedUpdate = true;

bool Viewer::cutSurface = true;
bool Viewer::mapIsDirty = false;

const glm::vec3 white(1.0, 1.0, 1.0);
const glm::vec3 skyblue(0.643, 0.647, 0.815);
const glm::vec3 red(1.0, 0.0, 0.0);
const glm::vec3 orange(1.0, 0.55, 0.0);
const glm::vec3 yellow(1.0, 0.9, 0.1);

void Viewer::init(const std::string& objPath_)
{
	objPath = objPath_;
	initGLFW();
	initGui();
	initGL();
	initShaders();
	initCameras();
	initTransforms();
	initLights();
	initModel();
	initRenderMeshes();
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

void errorCallback(int error, const char* description)
{
	std::cerr << description << std::endl;
}

void Viewer::initGLFW()
{
	glfwSetErrorCallback(errorCallback);
	if (!glfwInit()) {
		std::cerr << "Unable to initalize glfw" << std::endl;
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
		std::cerr << "Unable to initalize glfw window" << std::endl;
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glfwMakeContextCurrent(window);
#if defined(NANOGUI_GLAD)
	if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
		std::cerr << "Unable to initalize glad" << std::endl;
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
			initModel();
			initRenderMeshes();
			initBFF();

		} catch (...) {
			reportError("Unable to load file: " + objPath);
		}
	});

	Button *exportMeshButton = new Button(nanoguiWindow, "Export UVs");
	exportMeshButton->setFixedHeight(25);
	exportMeshButton->setCallback([] {
		std::string file = file_dialog({{"obj", "Wavefront OBJ file"}}, true);
		if (file.empty()) return;
		try {
			std::vector<uint8_t> isSurfaceMappedToSphere;
			for (int i = 0; i < model.size(); i++) {
				isSurfaceMappedToSphere.push_back(modelStates[i].mappedToSphere ? 1 : 0);
			}

			if (!MeshIO::write(file, model, isSurfaceMappedToSphere,
							   exportNormalizedUVsCheckBox->checked(),
							   !useVtFlagToStoreUVsCheckBox->checked(), 1.0)) {
				reportError("Unable to write file: " + file);
			}

		} catch (...) {
			reportError("Unable to write file: " + file);
		}
	});

	exportNormalizedUVsCheckBox = new CheckBox(nanoguiWindow, "Normalize UVs to [0, 1]");
	exportNormalizedUVsCheckBox->setFixedHeight(14);

	useVtFlagToStoreUVsCheckBox = new CheckBox(nanoguiWindow, "Use 'vt' flag to store UVs");
	useVtFlagToStoreUVsCheckBox->setFixedHeight(14);

	// plot
	plotLabel = new Label(nanoguiWindow, "Plot", "sans");
	ComboBox *plotComboBox = new ComboBox(nanoguiWindow, {"      Constant      ",
														  "       Shaded       ",
														  "       Normal       ",
														  "Conformal Distortion",
														  "   Area Distortion  "});
	plotComboBox->setSelectedIndex(static_cast<int>(plotType));
	plotComboBox->setFixedHeight(25);
	plotComboBox->setCallback([](int box) {
		switch (box) {
			case 0:
				plotType = PlotType::constant;
				break;
			case 1:
				plotType = PlotType::shaded;
				break;
			case 2:
				plotType = PlotType::normals;
				break;
			case 3:
				plotType = PlotType::quasiConformalError;
				break;
			case 4:
				plotType = PlotType::areaScaling;
				break;
		}

		if (loadedModel) {
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
	patternSizeSlider->setRange(std::make_pair(0.0, 5.0));
	patternSizeSlider->setValue(patternSize);
	patternSizeSlider->setFixedHeight(14);
	patternSizeSlider->setCallback([](float s) { patternSize = s; });

	// wireframe
	CheckBox *wireframeCheckBox = new CheckBox(nanoguiWindow, "Show Wireframe");
	wireframeCheckBox->setCallback([](int box) { showWireframe = !showWireframe; });
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
	targetBoundaryComboBox->setCallback([](int box) {
		if (loadedModel) {
			if (state->surfaceIsClosed) {
				reportError("Only supported on surfaces with boundary");

			} else {
				state->editBoundary = false;
				state->flattenedToDisk = false;
				if (state->setBoundaryAngles && box != 3) {
					removeVertexHandles();
					angleSlider->setRange(std::make_pair(-4.0, 1.0));
					state->setBoundaryAngles = false;

				} else if (!state->setBoundaryAngles && box == 3) {
					if (state->handles.size() > 0) removeVertexHandles();
					state->setBoundaryAngles = true;
					angleSlider->setRange(std::make_pair(-0.9, 0.9));
					setDefaultBoundaryAngles();
				}

				if (box != 2) {
					flattenWithTargetBoundary(static_cast<BoundaryType>(box));

				} else if (box == 2) {
					removeVertexHandles();
					state->spline.reset();
					state->editBoundary = true;
					state->targetData = DenseMatrix(state->bff->data->bN);
					flattenWithSplineBoundary(true, true);
				}

				placeConesButton->setEnabled(!state->setBoundaryAngles && box != 2);
			}
		}
	});

	// cones
	angleControlLabel = new Label(nanoguiWindow, "Angle Control             Sum:0π", "sans");
	angleTextBox = new TextBox(nanoguiWindow);
	angleTextBox->setUnits("π");
	angleTextBox->setFormat("[-]?[0-9]*\\.?[0-9]+");
	angleTextBox->setFixedHeight(25);
	angleTextBox->setCallback([](const std::string& str)->bool {
		if (str == "") return false;

		double angle = stod(str);
		angleSlider->setValue(angle);
		if (state->setBoundaryAngles) {
			if (angle >= 1.0 || angle <= -1.0) {
				reportError("Boundary angle less than -π and greater than π will result in a bad quality mapping. Hit Reset to undo.");
			}

			state->boundaryAngles(state->bff->data->bIndex[state->selectedVertexHandle->wedge()]) = angle*M_PI;
			flattenWithBoundaryAnglesLRU();

		} else {
			if (angle >= 2.0) {
				reportError("Cone angle must be less than 2π.");
				return false;
			}

			state->coneAngles(state->bff->data->index[state->selectedVertexHandle->wedge()]) = angle*M_PI;
			if (!state->surfaceIsClosed || (state->surfaceIsClosed && state->handles.size() > 2)) flattenWithCones();
		}

		updateAngleControlLabel();

		return true;
	});

	angleSlider = new Slider(nanoguiWindow);
	angleSlider->setRange(std::make_pair(-4.0, 1.0));
	angleSlider->setValue(0.5);
	angleSlider->setFixedHeight(14);
	angleSlider->setCallback([](float angle) {
		updateAngleTextBoxAndSliderValue(angle);

		if (state->setBoundaryAngles) {
			state->boundaryAngles(state->bff->data->bIndex[state->selectedVertexHandle->wedge()]) = angle*M_PI;
			flattenWithBoundaryAnglesLRU();

		} else {
			state->coneAngles(state->bff->data->index[state->selectedVertexHandle->wedge()]) = angle*M_PI;
			if (!state->surfaceIsClosed || (state->surfaceIsClosed && state->handles.size() > 2)) flattenWithCones();
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
	vertexHandleCountTextBox->setCallback([](const std::string& str)->bool {
		if (str == "" || !state) return false;

		int S = std::stoi(str);
		if (state->surfaceIsClosed && S < 3) {
			reportError("Place at least 3 cones to flatten the mesh. It's not possible to cut a closed mesh with fewer than 2 cones. Placing only 2 cones will result in a bad quality mapping.");
			return false;
		}

		if (S <= (int)mesh->vertices.size() - state->bff->data->bN) return true;

		reportError("Number of cones cannot exceed interior vertices on the mesh");
		return false;
	});

	placeConesButton = new Button(nanoguiWindow, "Place Cones");
	placeConesButton->setFixedHeight(25);
	placeConesButton->setCallback([] {
		if (loadedModel) {
			int S = std::min(std::stoi(vertexHandleCountTextBox->value()), (int)mesh->vertices.size() - state->bff->data->bN);
			placeCones(S);
			if (!state->surfaceIsClosed && S == 0) {
				BoundaryType type = static_cast<BoundaryType>(targetBoundaryComboBox->selectedIndex());
				flattenWithTargetBoundary(type);

			} else {
				flattenWithCones();
			}
		}
	});

	/*
	Button *removeSeamsButton = new Button(nanoguiWindow, "Remove Seams");
	removeSeamsButton->setFixedHeight(25);
	removeSeamsButton->setEnabled(false);
	*/

	new Label(nanoguiWindow, "Spherical Parameterization");
	Button *sphericalParameterizationButton = new Button(nanoguiWindow, "Map to Sphere");
	sphericalParameterizationButton->setFixedHeight(25);
	sphericalParameterizationButton->setCallback([] { mapToSphere(); });

	new Label(nanoguiWindow, "");
	Button *resetButton = new Button(nanoguiWindow, "Reset");
	resetButton->setFixedHeight(25);
	resetButton->setCallback([] {
		if (loadedModel) {
			if (!state->setBoundaryAngles && (state->handles.size() > 0 || state->mappedToSphere)) {
				removeVertexHandles();
				if (!state->surfaceIsClosed) {
					// parameterize automatically for surfaces that are not closed
					BoundaryType type = static_cast<BoundaryType>(targetBoundaryComboBox->selectedIndex());
					flattenWithTargetBoundary(type);

				} else {
					// parameterize with cones for closed surfaces
					vertexHandleCountTextBox->setValue("8");
					int S = std::min(8, (int)mesh->vertices.size() - state->bff->data->bN);
					if (S > 0) {
						placeCones(S);
						flattenWithCones();
					}
				}

			} else if (state->setBoundaryAngles) {
				removeVertexHandles();
				setDefaultBoundaryAngles();
				flattenWithTargetBoundary(BoundaryType::setBoundaryAngles);

			} else if (state->editBoundary) {
				state->knots.clear();
				state->selectedKnot = mesh->wedges().end();
				state->splineHandles.clear();
				state->splineLines.clear();
				state->spline.knots.clear();
				initSpline();
				state->targetData = DenseMatrix(state->bff->data->bN);
				flattenWithSplineBoundary(true);
			}

			state->mappedToSphere = false;
			cameras[1]->reset();
			updateCameraZoom();
			updatePlotLabel();
		}
	});

	// help text
	instructionText = new Label(nanoguiWindow, "", "sans", 15);
	instructionText->setFixedSize(Eigen::Vector2i(190, 60));
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
	for (int i = 0; i < 2; i++) cameras.emplace_back(std::shared_ptr<Camera>(new Camera(EYE)));
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

void Viewer::initModel()
{
	// load model from file
	std::string error;
	if ((loadedModel = MeshIO::read(objPath, model, error))) {
		modelStates.resize(model.size());
		for (int i = 0; i < model.size(); i++) {
			if (model[i].boundaries.size() >= 1) {
				// mesh has boundaries
				int eulerPlusBoundaries = model[i].eulerCharacteristic() + (int)model[i].boundaries.size();

				if (eulerPlusBoundaries == 2) {
					// fill holes if mesh has more than 1 boundary
					if (model[i].boundaries.size() > 1) {
						if (HoleFiller::fill(model[i])) {
							// all holes were filled
							modelStates[i].surfaceIsClosed = true;
						}
					}

				} else {
					// mesh probably has holes and handles
					HoleFiller::fill(model[i], true);
					Generators::compute(model[i]);
					modelStates[i].surfaceHasGenerators = true;
				}

			} else if (model[i].boundaries.size() == 0) {
				if (model[i].eulerCharacteristic() == 2) {
					// mesh is closed
					modelStates[i].surfaceIsClosed = true;

				} else {
					// mesh has handles
					Generators::compute(model[i]);
					modelStates[i].surfaceHasGenerators = true;
				}
			}

			// initialize selected vertex and knot
			modelStates[i].selectedKnot = model[i].wedges().end();
			modelStates[i].selectedVertexHandle = model[i].vertices.end();
			if (modelStates[i].handles.size() > 0) {
				for (VertexHandle handle: modelStates[i].handles) {
					const Vector& uv = handle.vertex->wedge()->uv;
					const Vector& p = handle.vertex->position;
					addSphere(handle.sphere3D, glm::vec3(p.x, p.y, p.z), yellow);
					addSphere(handle.sphereUV, glm::vec3(uv.x, uv.y, uv.z), yellow);
				}
			}
		}

	} else {
		reportError("Unable to load file: " + objPath + ". " + error);
	}
}

glm::vec3 elementColor(unsigned int pickedId)
{
	return glm::vec3(((pickedId & 0x000000FF) >>  0)/255.0,
					 ((pickedId & 0x0000FF00) >>  8)/255.0,
					 ((pickedId & 0x00FF0000) >> 16)/255.0);
}

void Viewer::initRenderMeshes()
{
	if (loadedModel) {
		int nVertices = 0;

		for (int i = 0; i < model.size(); i++) {
			for (int j = 0; j < 2; j++) {
				modelStates[i].renderMeshes.emplace_back(std::shared_ptr<RenderMesh>(new RenderMesh()));
			}

			// initialize attributes for both rendermeshes
			int N = 0;
			for (CornerCIter c = model[i].corners.begin(); c != model[i].corners.end(); c++) {
				if (!c->face()->fillsHole) {
					N++;
				}
			}

			std::shared_ptr<Buffer> uvs(new Buffer(N));
			std::shared_ptr<Buffer> normals(new Buffer(N));
			std::shared_ptr<Buffer> barycenters(new Buffer(N));
			modelStates[i].renderMeshes[0]->positions = std::shared_ptr<Buffer>(new Buffer(N));
			modelStates[i].renderMeshes[0]->uvs = uvs;
			modelStates[i].renderMeshes[0]->normals = normals;
			modelStates[i].renderMeshes[0]->colors = std::shared_ptr<Buffer>(new Buffer(N));
			modelStates[i].renderMeshes[0]->barycenters = barycenters;
			modelStates[i].renderMeshes[0]->pickPositions = std::shared_ptr<Buffer>(new Buffer(6*N));
			modelStates[i].renderMeshes[0]->pickColors = std::shared_ptr<Buffer>(new Buffer(6*N));
			modelStates[i].renderMeshes[1]->positions = uvs;
			modelStates[i].renderMeshes[1]->uvs = uvs;
			modelStates[i].renderMeshes[1]->normals = normals;
			modelStates[i].renderMeshes[1]->colors = std::shared_ptr<Buffer>(new Buffer(N));
			modelStates[i].renderMeshes[1]->barycenters = barycenters;

			std::vector<Vector> barycenter = {Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0), Vector(0.0, 0.0, 1.0)};
			for (FaceCIter f = model[i].faces.begin(); f != model[i].faces.end(); f++) {
				if (!f->fillsHole) {
					int index = 3*f->index;

					// set positions, normals and colors for the 1st rendermesh
					// and barycenters for both rendermeshes
					int j = 0;
					HalfEdgeCIter h = f->halfEdge();
					do {
						VertexCIter v = h->vertex();
						Vector normal = v->normal();
						for (int k = 0; k < 3; k++) {
							modelStates[i].renderMeshes[0]->positions->buffer[index + j][k] = v->position[k];
							modelStates[i].renderMeshes[0]->colors->buffer[index + j][k] = white[k];
							normals->buffer[index + j][k] = normal[k];
							barycenters->buffer[index + j][k] = barycenter[j][k];
						}

						j++;
						h = h->next();
					} while (h != f->halfEdge());
				}
			}

			// define picking region for each vertex to be the barycentric dual cell
			for (CornerCIter c = model[i].corners.begin(); c != model[i].corners.end(); c++) {
				if (!c->face()->fillsHole) {
					int index = 6*c->index;
					VertexCIter v = c->vertex();
					glm::vec3 pickColor = elementColor(nVertices + v->index);

					// get the three vertices in the triangle
					Vector p1 = v->position;
					Vector p2 = c->next()->vertex()->position;
					Vector p3 = c->prev()->vertex()->position;

					// get the edge and triangle midpoints
					Vector m12 = (p1 + p2)/2.0;
					Vector m13 = (p1 + p3)/2.0;
					Vector m123 = (p1 + p2 + p3)/3.0;

					// draw the quad making up the dual region as a pair of triangles
					std::vector<Vector> tris = {p1, m12, m123, p1, m123, m13};

					// give all the triangles the same pick color as this corner
					for (int j = 0; j < 6; j++) {
						modelStates[i].renderMeshes[0]->pickPositions->buffer[index + j] = glm::vec3(tris[j].x, tris[j].y, tris[j].z);
						modelStates[i].renderMeshes[0]->pickColors->buffer[index + j] = pickColor;
					}
				}
			}

			uvs->update();
			normals->update();
			barycenters->update();
			modelStates[i].renderMeshes[0]->update({POSITION, COLOR, PICK_POSITION, PICK_COLOR});
			modelStates[i].renderMeshes[1]->update({COLOR});

			nVertices += (int)model[i].vertices.size();
		}
	}
}

void Viewer::initSpline()
{
	// place at most 4 handles on the boundary
	int n = 0;
	int nHandles = 0;
	int maxHandles = std::max(1, std::min(state->bff->data->bN, 4));
	int modHandles = (int)std::floor(state->bff->data->bN/maxHandles);

	for (WedgeIter w: mesh->cutBoundary()) {
		w->knot = state->spline.knots.end();

		if (n%modHandles == 0 && nHandles < maxHandles) {
			addKnot(w);
			nHandles++;
		}

		n++;
	}
}

void Viewer::initBFF()
{
	if (loadedModel) {
		for (int i = model.size() - 1; i >= 0; i--) {
			mesh = &model[i];
			state = &modelStates[i];

			// flatten with isometric lengths
			state->bff = std::shared_ptr<BFF>(new BFF(*mesh));
			state->boundaryAngles = DenseMatrix(state->bff->data->bN);
			state->coneAngles = DenseMatrix(state->bff->data->iN);

			initSpline();
			if (!state->surfaceIsClosed) {
				// parameterize automatically for surfaces that are not closed
				flattenWithTargetBoundary(BoundaryType::automatic);

			} else {
				// parameterize with cones for closed surfaces
				int S = std::min(8, (int)mesh->vertices.size() - state->bff->data->bN);
				if (S > 0) {
					placeCones(S);
					flattenWithCones();
				}
			}
		}

		updateInstructionText();
		updatePlotLabel();
	}

	// update gui
	targetBoundaryComboBox->setSelectedIndex(0);
	vertexHandleCountTextBox->setValue("8");
	angleTextBox->setValue("Angle");
	angleTextBox->setEditable(false);
	angleSlider->setEnabled(false);
	angleControlLabel->setCaption("Angle Control             Sum:0π");
	placeConesButton->setEnabled(true);
}

void Viewer::clearScene()
{
	model.clear();
	modelStates.clear();
	mesh = NULL;
	state = NULL;
	selectedMesh = 0;
	loadedModel = false;
	cutSurface = true;
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
	if (plotType == PlotType::constant || plotType == PlotType::shaded || plotType == PlotType::normals) {
		plotLabel->setCaption("Plot");

	} else if (!state->surfaceIsClosed || (state->surfaceIsClosed && state->cut) || state->mappedToSphere) {
		if (plotType == PlotType::quasiConformalError) {
			std::stringstream label;
			Vector error = Distortion::computeQuasiConformalError(model);
			label << std::setprecision(4) << "Mean:" << error[2] << "   Max:" << error[1];
			plotLabel->setCaption("Plot     " + label.str());

		} else if (plotType == PlotType::areaScaling) {
			std::stringstream label;
			Vector error = Distortion::computeAreaScaling(model);
			label << std::setprecision(2) << "Min:" << exp(error[0]) << "   Max:" << exp(error[1]);
			plotLabel->setCaption("Plot     " + label.str());
		}

	} else {
		plotLabel->setCaption("Plot");
	}
}

void Viewer::updateAngleControlLabel()
{
	double angleSum = state->setBoundaryAngles ? state->boundaryAngles.sum() : state->coneAngles.sum();
	angleSum /= M_PI;
	std::stringstream angleSumLabel;
	angleSumLabel << std::setprecision(3) << angleSum;
	angleControlLabel->setCaption("Angle Control             Sum:" + angleSumLabel.str() + "π");
}

void Viewer::updateAngleTextBoxAndSliderValue(double angle)
{
	std::stringstream angleLabel;
	angleLabel << std::setprecision(3) << angle;
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
	if (index == 0 || (index == 1 && (plotType == PlotType::shaded || plotType == PlotType::normals))) {
		modelShader.use();
		glUniform1f(glGetUniformLocation(modelShader.program, "patternSize"), exp(patternSize));
		glUniform1i(glGetUniformLocation(modelShader.program, "pattern"), patternComboBox->selectedIndex());
	}

	glUniform1i(glGetUniformLocation(modelShader.program, "shade"), plotType != PlotType::normals);
}

void Viewer::updateRenderMeshes()
{
	for (int i = 0; i < model.size(); i++) {
		// set uvs for the 1st rendermesh and positions and colors for 2nd rendermesh
		std::shared_ptr<Buffer> uvs = modelStates[i].renderMeshes[0]->uvs;
		for (FaceCIter f = model[i].faces.begin(); f != model[i].faces.end(); f++) {
			if (!f->fillsHole) {
				int index = 3*f->index;

				WedgeCIter w0 = f->halfEdge()->wedge();
				WedgeCIter w1 = w0->next();
				WedgeCIter w2 = w1->next();
				const Vector& p0 = w0->uv;
				const Vector& p1 = w1->uv;
				const Vector& p2 = w2->uv;
				double s = cross(p1 - p0, p2 - p0).z;

				int j = 0;
				HalfEdgeCIter h = f->halfEdge();
				do {
					WedgeCIter w = h->next()->wedge();
					glm::vec3 color;

					if (plotType == PlotType::constant) {
						if (s > 0.0 || modelStates[i].mappedToSphere) color = skyblue;
						else color = red;

					} else if (plotType == PlotType::shaded) {
					   color = white;

					} else if (plotType == PlotType::normals) {
						for (int k = 0; k < 3; k++) {
							float nk = modelStates[i].renderMeshes[0]->normals->buffer[index + j][k];
							color[k] = (nk + 1)/2.0;
						}

					} else if (plotType == PlotType::quasiConformalError) {
						Vector distortion = Distortion::color(f, i, true);
						color = glm::vec3(distortion.x, distortion.y, distortion.z);

					} else if (plotType == PlotType::areaScaling) {
						Vector distortion = Distortion::color(f, i, false);
						color = glm::vec3(distortion.x, distortion.y, distortion.z);
					}

					for (int k = 0; k < 3; k++) {
						uvs->buffer[index + j][k] = w->uv[k];
						modelStates[i].renderMeshes[0]->colors->buffer[index + j][k] = color[k];
						modelStates[i].renderMeshes[1]->colors->buffer[index + j][k] = color[k];
					}

					j++;
					h = h->next();
				} while (h != f->halfEdge());
			}
		}

		// only update uvs for selected mesh
		uvs->update(); // buffer is shared by both rendermeshes
		if (colorsNeedUpdate) {
			modelStates[i].renderMeshes[0]->update({COLOR});
			modelStates[i].renderMeshes[1]->update({COLOR});
		}
	}

	colorsNeedUpdate = false;
}

void Viewer::updateSphere(std::shared_ptr<RenderMesh> sphere, const glm::vec3& center, const glm::vec3& color)
{
	const int nSlices = 10;
	const int nStacks = 10;
	const float dTheta = 2*M_PI/nSlices;
	const float dPhi = M_PI/nSlices;
	const float radius = 0.02;

	if (!sphere->positions) sphere->positions = std::shared_ptr<Buffer>(new Buffer());
	else sphere->positions->buffer.clear();
	if (!sphere->colors) sphere->colors = std::shared_ptr<Buffer>(new Buffer());
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

			sphere->positions->buffer.emplace_back(p00);
			sphere->positions->buffer.emplace_back(p01);
			sphere->positions->buffer.emplace_back(p10);
			sphere->positions->buffer.emplace_back(p10);
			sphere->positions->buffer.emplace_back(p01);
			sphere->positions->buffer.emplace_back(p11);
			for (int k = 0; k < 6; k++) sphere->colors->buffer.emplace_back(color);

			theta += dTheta;
		}
		phi += dPhi;
	}

	sphere->update({POSITION, COLOR});
}

void Viewer::addSphere(std::shared_ptr<RenderMesh>& sphere,
					   const glm::vec3& center, const glm::vec3& color)
{
	sphere = std::shared_ptr<RenderMesh>(new RenderMesh());
	updateSphere(sphere, center, color);
}

void Viewer::addSphere(std::vector<std::shared_ptr<RenderMesh>>& spheres,
					   const glm::vec3& center, const glm::vec3& color)
{
	std::shared_ptr<RenderMesh> sphere(new RenderMesh());
	updateSphere(sphere, center, color);
	spheres.emplace_back(sphere);
}

void Viewer::updateLine(std::shared_ptr<RenderMesh> line, const glm::vec3& color, float width,
						float x1, float y1, float x2, float y2, float z)
{
	if (!line->positions) line->positions = std::shared_ptr<Buffer>(new Buffer());
	else line->positions->buffer.clear();
	if (!line->colors) line->colors = std::shared_ptr<Buffer>(new Buffer());
	else line->colors->buffer.clear();

	glm::vec3 n = glm::normalize(glm::vec3(-(y2 - y1), x2 - x1, 0.0f));
	glm::vec3 a = glm::vec3(x1, y1, z);
	glm::vec3 b = glm::vec3(x2, y2, z);

	glm::vec3 p00 = a + width*n;
	glm::vec3 p01 = b + width*n;
	glm::vec3 p10 = b - width*n;
	glm::vec3 p11 = a - width*n;

	line->positions->buffer = {p00, p01, p10, p00, p10, p11};
	for (int k = 0; k < 6; k++) line->colors->buffer.emplace_back(color);
	line->update({POSITION, COLOR});
}

void Viewer::addLine(std::vector<std::shared_ptr<RenderMesh>>& lines,
					 const glm::vec3& color, float width,
					 float x1, float y1, float x2, float y2, float z)
{
	std::shared_ptr<RenderMesh> line(new RenderMesh());
	updateLine(line, color, width, x1, y1, x2, y2, z);
	lines.emplace_back(line);
}

void Viewer::updateCut()
{
	std::vector<VertexIter> cones = getHandleVertices();
	Cutter::cut(cones, *mesh);

	state->cut = std::shared_ptr<RenderMesh>(new RenderMesh());
	state->cut->positions = std::shared_ptr<Buffer>(new Buffer());
	state->cut->colors = std::shared_ptr<Buffer>(new Buffer());
	for (EdgeCIter e = mesh->edges.begin(); e != mesh->edges.end(); e++) {
		if (e->onCut || e->onGenerator) {
			HalfEdgeCIter he = e->halfEdge();

			Vector n = 0.5*(he->face()->normal() + he->flip()->face()->normal());
			Vector a = he->vertex()->position + 0.001*n;
			Vector b = he->flip()->vertex()->position + 0.001*n;
			Vector u = b - a;
			Vector v = cross(u, n);
			v.normalize();

			const double width = 0.002;
			Vector p00 = a + width*v;
			Vector p01 = b + width*v;
			Vector p10 = b - width*v;
			Vector p11 = a - width*v;
			std::vector<Vector> p = {p00, p01, p10, p00, p10, p11};

			for (int i = 0; i < 6; i++) {
				state->cut->positions->buffer.emplace_back(glm::vec3(p[i].x, p[i].y, p[i].z));
				state->cut->colors->buffer.emplace_back(red);
			}
		}
	}

	state->cut->update({POSITION, COLOR});
}

void Viewer::updateInstructionText()
{
	if (state->editBoundary) {
		instructionText->setCaption("Drag (+ shift) handles to change scaling (angles). Click (+ ctrl) bdy vertices to add (remove) handles.");

	} else if (state->setBoundaryAngles) {
		instructionText->setCaption("Click (+ ctrl) bdy vertices to set (clear) angle values.\nAlt + drag to pan camera.");

	} else {
		instructionText->setCaption("Click (+ ctrl) int. vertices to set (clear) cone angles.\nAlt + drag to pan camera.");
	}

	if (model.size() > 1) {
		instructionText->setCaption(instructionText->caption() + "\nChange component with <-/-> keys.");
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
	for (WedgeCIter w = mesh->wedges().begin(); w != mesh->wedges().end(); w++) {
		radius = std::max(radius, w->uv.norm());
	}

	cameras[1]->radius = EYE*radius;
	cameras[1]->viewNeedsUpdate = true;
}

void Viewer::updateVertexHandles(VertexIter v)
{
	double angle = 0.5;
	if (state->handles.size() > 0) {
		VertexHandle handle = state->handles.front();
		angle = state->coneAngles(state->bff->data->index[handle.vertex->wedge()])/M_PI;
	}

	if (ctrlClick) {
		removeVertexHandle(v);

	} else {
		if (state->setBoundaryAngles || state->surfaceIsClosed) addVertexHandle(v, 0.0);
		else addVertexHandle(v, angle);
	}
}

void Viewer::updateVertexHandlesUVs()
{
	for (VertexHandle handle: state->handles) {
		VertexIter v = handle.vertex;
		glm::vec3 color = v == state->selectedVertexHandle ? orange : yellow;
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
			Distortion::computeQuasiConformalError(model);
			colorsNeedUpdate = true;
			break;
		case PlotType::areaScaling:
			Distortion::computeAreaScaling(model);
			colorsNeedUpdate = true;
			break;
		default:
			break;
	}

	// update meshes and instructions
	updateRenderMeshes();
	updateVertexHandlesUVs();
	updateInstructionText();
	updatePlotLabel();
	if (updateZoom) updateCameraZoom();
	draw();
}

void Viewer::updateSelectedMesh()
{
	// set current mesh and state; reset camera and update instructions
	mesh = &model[selectedMesh];
	state = &modelStates[selectedMesh];
	cameras[1]->reset();
	updateCameraZoom();
	updateInstructionText();

	// update target boundary box and enable/disable place cones button
	if (state->setBoundaryAngles) {
		targetBoundaryComboBox->setSelectedIndex(3);
		placeConesButton->setEnabled(false);

	} else if (state->editBoundary) {
		targetBoundaryComboBox->setSelectedIndex(2);
		placeConesButton->setEnabled(false);

	} else {
		targetBoundaryComboBox->setSelectedIndex(state->flattenedToDisk ? 1 : 0);
		placeConesButton->setEnabled(true);
	}

	// update angle text box and slider values based on whether state has handles
	if (state->handles.size() > 0 && state->selectedVertexHandle != mesh->vertices.end()) {
		angleTextBox->setEditable(true);
		angleSlider->setEnabled(true);

		float angle = state->setBoundaryAngles ?
		state->boundaryAngles(state->bff->data->bIndex[state->selectedVertexHandle->wedge()]) :
		state->coneAngles(state->bff->data->index[state->selectedVertexHandle->wedge()]);
		updateAngleTextBoxAndSliderValue(angle/M_PI);

	} else {
		angleTextBox->setEditable(false);
		angleSlider->setEnabled(false);
		angleTextBox->setValue("Angle");
	}
}

void Viewer::addKnot(WedgeIter w)
{
	// update handle colors
	for (int i = 0; i < (int)state->splineHandles.size(); i++) {
		glm::vec3 color = state->knots[i] == w ? orange : yellow;
		for (int j = 0; j < (int)state->splineHandles[i]->colors->buffer.size(); j++) {
			state->splineHandles[i]->colors->buffer[j] = color;
		}

		for (int j = 0; j < (int)state->splineLines[2*i]->colors->buffer.size(); j++) {
			state->splineLines[2*i]->colors->buffer[j] = color;
			state->splineLines[2*i + 1]->colors->buffer[j] = color;
		}

		state->splineHandles[i]->update({COLOR});
		state->splineLines[2*i]->update({COLOR});
		state->splineLines[2*i + 1]->update({COLOR});
	}

	// add knot if it doesn't exist
	if (w->knot == state->spline.knots.end()) {
		double L = 0.0;
		for (WedgeCIter w: mesh->cutBoundary()) {
			L += w->halfEdge()->next()->edge()->length();
		}

		double l = 0.0;
		for (WedgeCIter ww: mesh->cutBoundary()) {
			if (w == ww) break;
			l += ww->halfEdge()->next()->edge()->length();
		}

		double s = 2*M_PI*l/L;
		w->knot = state->spline.addKnot(s, state->spline.evaluate(s));
		const Vector& uv = w->uv;
		Vector scaledT = 0.25*exp(w->scaling())*w->tangent();

		addSphere(state->splineHandles, glm::vec3(uv.x, uv.y, uv.z), orange);
		addLine(state->splineLines, orange, 0.005, uv.x, uv.y, uv.x + scaledT.x, uv.y + scaledT.y, uv.z);
		addLine(state->splineLines, orange, 0.005, uv.x, uv.y, uv.x - scaledT.x, uv.y - scaledT.y, uv.z);

		state->knots.emplace_back(w);
	}

	state->selectedKnot = w;
}

void Viewer::removeKnot(WedgeIter w)
{
	if (w->knot != state->spline.knots.end()) {
		// remove knot from spline
		state->spline.removeKnot(w->knot->first);
		w->knot = state->spline.knots.end();
		if (state->selectedKnot == w) state->selectedKnot = mesh->wedges().end();

		// update handle list
		int splineHandleIndex = -1;
		for (int i = 0; i < (int)state->splineHandles.size(); i++) {
			if (state->knots[i] == w) {
				splineHandleIndex = i;
				break;
			}
		}

		state->splineHandles.erase(state->splineHandles.begin() + splineHandleIndex);
		state->splineLines.erase(state->splineLines.begin() + 2*splineHandleIndex,
								 state->splineLines.begin() + 2*(splineHandleIndex + 1));
		state->knots.erase(state->knots.begin() + splineHandleIndex);

		// flatten
		flattenWithSplineBoundary(!shiftClick);
	}
}

void Viewer::dragKnot(double change)
{
	if (state->selectedKnot != mesh->wedges().end()) {
		state->selectedKnot->knot->second += 3.0*change;

		// flatten
		flattenWithSplineBoundary(!shiftClick);
	}
}

void Viewer::addVertexHandle(VertexIter v, double newAngle)
{
	if ((state->setBoundaryAngles && !v->onBoundary(false)) || (!state->setBoundaryAngles && v->onBoundary(false))) {
		return;
	}

	angleTextBox->setEditable(true);
	angleSlider->setEnabled(true);

	// check if vertex v already has a handle
	bool handleExists = false;
	VertexHandle existingHandle;
	for (VertexHandle handle: state->handles) {
		glm::vec3 color = yellow;

		if (handle.vertex == v) {
			existingHandle = handle;
			color = orange;
			state->selectedVertexHandle = handle.vertex;
			handleExists = true;

			float angle = state->setBoundaryAngles ?
						  state->boundaryAngles(state->bff->data->bIndex[state->selectedVertexHandle->wedge()]) :
						  state->coneAngles(state->bff->data->index[state->selectedVertexHandle->wedge()]);
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
		state->handles.remove(existingHandle);
		state->handles.push_front(existingHandle);

	} else {
		// otherwise, create a new handle for vertex v (putting it at the front of the list)
		state->selectedVertexHandle = v;

		VertexHandle handle;
		handle.vertex = v;
		const Vector& uv = v->wedge()->uv;
		const Vector& p = v->position;
		addSphere(handle.sphere3D, glm::vec3(p.x, p.y, p.z), orange);
		addSphere(handle.sphereUV, glm::vec3(uv.x, uv.y, uv.z), orange);
		state->handles.push_front(handle);

		if (state->setBoundaryAngles) {
			state->boundaryAngles(state->bff->data->bIndex[state->selectedVertexHandle->wedge()]) = newAngle*M_PI;

		} else {
			state->coneAngles(state->bff->data->index[state->selectedVertexHandle->wedge()]) = newAngle*M_PI;
			Cutter::glue(*mesh);
			cutSurface = true;
			mapIsDirty = true;
		}

		updateAngleTextBoxAndSliderValue(newAngle);
		updateAngleControlLabel();
	}
}

void Viewer::removeVertexHandle(VertexIter v)
{
	if (state->surfaceIsClosed && state->handles.size() == 3) {
		reportError("At least 3 cones must be placed. It's not possible to cut a closed mesh with fewer than 2 cones. Placing only 2 cones will result in a bad quality mapping.");
		return;

	} else if (state->setBoundaryAngles && state->handles.size() == 3) {
		reportError("At least 3 handles must be placed. Fewer than 3 handles will result in a bad quality mapping.");
		return;
	}

	// check if the vertex has a handle
	for (VertexHandle handle: state->handles) {
		if (handle.vertex == v) {
			if (state->setBoundaryAngles) {
				state->boundaryAngles(state->bff->data->bIndex[handle.vertex->wedge()]) = 0.0;

			} else {
				state->coneAngles(state->bff->data->index[handle.vertex->wedge()]) = 0.0;
				Cutter::glue(*mesh);
				cutSurface = true;
				mapIsDirty = true;
			}

			updateAngleControlLabel();

			// remove cone from cone list and update angle
			state->handles.remove(handle);
			if (state->selectedVertexHandle == v) state->selectedVertexHandle = mesh->vertices.end();

			// update interface
			if (state->selectedVertexHandle == mesh->vertices.end() || state->handles.size() == 0) {
				 state->selectedVertexHandle = mesh->vertices.end();
				 angleTextBox->setValue("Angle");
				 angleTextBox->setEditable(false);
				 angleSlider->setEnabled(false);
			}

			break;
		}
	}

	if (state->setBoundaryAngles) flattenWithBoundaryAnglesNormalize();
}

void Viewer::removeVertexHandles()
{
	// clear all cones and set cone angles to 0
	state->handles.clear();
	state->selectedVertexHandle = mesh->vertices.end();
	for (int i = 0; i < (int)state->boundaryAngles.nRows(); i++) state->boundaryAngles(i) = 0.0;
	for (int i = 0; i < (int)state->coneAngles.nRows(); i++) state->coneAngles(i) = 0.0;

	// update interface
	angleTextBox->setValue("Angle");
	angleTextBox->setEditable(false);
	angleSlider->setEnabled(false);
	angleControlLabel->setCaption("Angle Control             Sum:0π");

	// clear cut
	if (state->cut) state->cut = NULL;

	// glue cut
	Cutter::glue(*mesh);
	cutSurface = true;

	if (state->surfaceHasGenerators) updateCut();
}

void Viewer::placeCones(int S)
{
	if (loadedModel) {
		removeVertexHandles();
		if (S > 0) {
			mapIsDirty = true;
			std::vector<VertexIter> cones = getHandleVertices();
			ConePlacement::findConesAndPrescribeAngles(S, cones, state->coneAngles, state->bff->data, *mesh);

			for (VertexIter v: cones) {
				VertexHandle handle;
				handle.vertex = v;
				const Vector& uv = v->wedge()->uv;
				const Vector& p = v->position;
				addSphere(handle.sphere3D, glm::vec3(p.x, p.y, p.z), yellow);
				addSphere(handle.sphereUV, glm::vec3(uv.x, uv.y, uv.z), yellow);
				state->handles.emplace_back(handle);
			}
		}

		updateAngleControlLabel();
	}
}

void Viewer::normalizeConeAngles()
{
	int nHandles = (int)state->handles.size();
	double deviation = 4*M_PI - state->coneAngles.sum();
	if (std::abs(deviation) > 1e-8) {
		if (state->cut && nHandles == (int)state->handles.size()) {
			// use least recently used update when changing cone angles
			VertexHandle handle = state->handles.back();
			int i = state->bff->data->index[handle.vertex->wedge()];
			state->coneAngles(i) += deviation;

		} else {
			// divide deviation equally when adding and removing cones
			for (VertexHandle handle: state->handles) {
				int i = state->bff->data->index[handle.vertex->wedge()];
				state->coneAngles(i) += deviation/(double)state->handles.size();
			}
		}

		if (state->selectedVertexHandle != mesh->vertices.end()) {
			double angle = state->coneAngles(state->bff->data->index[state->selectedVertexHandle->wedge()])/M_PI;
			updateAngleTextBoxAndSliderValue(angle);
		}

		updateAngleControlLabel();
	}

	nHandles = (int)state->handles.size();
}

void Viewer::setDefaultBoundaryAngles()
{
	// place at most 4 handles on the boundary
	int n = 0;
	int nHandles = 0;
	int maxHandles = std::max(1, std::min(state->bff->data->bN, 4));
	int modHandles = (int)std::floor(state->bff->data->bN/maxHandles);
	double angle = 2.0/maxHandles;

	for (WedgeIter w: mesh->cutBoundary()) {
		if (n%modHandles == 0 && nHandles < maxHandles) {
			addVertexHandle(w->vertex(), angle);
			nHandles++;
		}

		n++;
	}
}

std::vector<VertexIter> Viewer::getHandleVertices()
{
	std::vector<VertexIter> handleVertices;
	for (VertexHandle handle: state->handles) {
		handleVertices.emplace_back(handle.vertex);
	}

	return handleVertices;
}

void Viewer::flattenWithTargetBoundary(BoundaryType type, bool updateZoom)
{
	if (loadedModel && !state->surfaceIsClosed) {
		if (type != BoundaryType::editBoundary) state->editBoundary = false;

		// flatten
		if (type == BoundaryType::automatic) {
			removeVertexHandles();
			DenseMatrix u(state->bff->data->bN);
			state->bff->flatten(u, true);
			mesh->projectUvsToPcaAxis();
			update();

		} else if (type == BoundaryType::setBoundaryAngles) {
			state->bff->flatten(state->boundaryAngles, false);
			update(updateZoom);

		} else if (type == BoundaryType::disk) {
			removeVertexHandles();
			state->bff->flattenToDisk();
			state->flattenedToDisk = true;
			mesh->projectUvsToPcaAxis();
			update();
		}
	}
}

void Viewer::flattenWithBoundaryAnglesLRU()
{
	// add deviation to least recently used boundary handle
	VertexHandle handle = state->handles.back();

	double deviation = 2*M_PI - state->boundaryAngles.sum();
	if (std::abs(deviation) > 1e-8) {
		state->boundaryAngles(state->bff->data->bIndex[handle.vertex->wedge()]) += deviation;
		flattenWithTargetBoundary(BoundaryType::setBoundaryAngles, false);
	}
}

void Viewer::flattenWithBoundaryAnglesNormalize()
{
	// divide deviation equally
	double deviation = 2*M_PI - state->boundaryAngles.sum();
	if (std::abs(deviation) > 1e-8) {
		for (VertexHandle handle: state->handles) {
			int i = state->bff->data->bIndex[handle.vertex->wedge()];
			state->boundaryAngles(i) += deviation/(double)state->handles.size();
		}

		if (state->selectedVertexHandle != mesh->vertices.end()) {
			double angle = state->boundaryAngles(state->bff->data->bIndex[state->selectedVertexHandle->wedge()])/M_PI;
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
	for (WedgeCIter w: mesh->cutBoundary()) {
		L += w->halfEdge()->next()->edge()->length();
	}

	double l = 0.0;
	DenseMatrix boundaryData(state->bff->data->bN);
	for (WedgeCIter w: mesh->cutBoundary()) {
		int i = state->bff->data->bIndex[w];
		double s = 2*M_PI*l/L;
		double offset = state->spline.evaluate(s);
		if (!givenScaleFactors) {
			// offset in curvature is the difference between offsets in angles
			offset = state->spline.evaluate(2*M_PI*(l + w->halfEdge()->next()->edge()->length())/L) - offset;
		}

		boundaryData(i) = state->targetData(i) + offset;
		l += w->halfEdge()->next()->edge()->length();
	}

	if (givenScaleFactors) {
		// flatten with target scale factors
		state->bff->flatten(boundaryData, true);

	} else {
		// flatten with target angles
		state->bff->closeCurvatures(boundaryData);
		state->bff->flatten(boundaryData, false);
	}

	// update spline handles
	for (int i = 0; i < (int)state->knots.size(); i++) {
		WedgeIter w = state->knots[i];
		const Vector& uv = w->uv;
		Vector scaledT = 0.25*exp(w->scaling())*w->tangent();
		glm::vec3 color = w == state->selectedKnot ? orange : yellow;

		updateSphere(state->splineHandles[i], glm::vec3(uv.x, uv.y, uv.z), color);
		updateLine(state->splineLines[2*i], color, 0.005, uv.x, uv.y, uv.x + scaledT.x, uv.y + scaledT.y, uv.z);
		updateLine(state->splineLines[2*i + 1], color, 0.005, uv.x, uv.y, uv.x - scaledT.x, uv.y - scaledT.y, uv.z);
	}

	update(updateZoom);
}

void Viewer::flattenWithCones()
{
	if (loadedModel && state->handles.size() > 0) {
		// flatten
		if (state->surfaceIsClosed) normalizeConeAngles();
		if (cutSurface) updateCut();
		state->bff->flattenWithCones(state->coneAngles, cutSurface);

		// update
		cutSurface = false;
		mapIsDirty = false;
		state->editBoundary = false;
		if (state->mappedToSphere) cameras[1]->reset();
		state->mappedToSphere = false;
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

bool selfIntersects(const std::vector<Vector>& gamma)
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

void Viewer::flattenToShape(const std::vector<Vector>& gamma)
{
	// flatten to target gamma if gamma does not self intersect
	if (gamma.size() > 2 && !selfIntersects(gamma)) {
		state->bff->flattenToShape(gamma);
		update();

	} else {
		reportError("Curve self intersects");
	}
}

void Viewer::mapToSphere()
{
	if (loadedModel) {
		if (state->surfaceIsClosed) {
			// remove cones
			removeVertexHandles();

			// map to sphere
			state->bff->mapToSphere();
			state->mappedToSphere = true;
			patternComboBox->setSelectedIndex(3);

			// update
			update();

		} else {
			reportError("Only supported on genus 0 (sphere like) surfaces");
		}
	}
}

Vector Viewer::ndc(int x, int y, int width, int height) {
	double s = 0.17;
	double t = 0.585;
	if (state->surfaceIsClosed && !state->cut && !state->mappedToSphere) {
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
	if (!gui->cursorPosCallbackEvent(x, y) && loadedModel) {
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

					if (state->editBoundary) {
						dragKnot(dx);

					} else if (altClick) {
						cameras[1]->pan(dx, dy);

					} else {
						if (state->mappedToSphere) {
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
	if (!gui->mouseButtonCallbackEvent(button, action, modifiers) && loadedModel) {
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
			if (state->editBoundary && shiftClick) {
				state->spline.reset();
				state->targetData = state->bff->compatibleTarget;
				flattenWithSplineBoundary(false);
			}

		} else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
			unclickX = gui->pixelRatio()*x;
			unclickY = gui->pixelRatio()*y;

			if (state->editBoundary && shiftClick) {
				// flatten with target scale factors
				state->spline.reset();
				state->targetData = state->bff->compatibleTarget;
				flattenWithSplineBoundary(true);

			} else {
				if (state->editBoundary) {
					colorsNeedUpdate = true;
					updateRenderMeshes();
				}

				// draw scene so that we can perform picking
				pickingEnabled = true;
				drawScene();

				// flatten with cones
				if (!state->setBoundaryAngles && !state->editBoundary && mapIsDirty) {
					if (state->handles.size() == 0) {
						BoundaryType type = static_cast<BoundaryType>(targetBoundaryComboBox->selectedIndex());
						flattenWithTargetBoundary(type);

					} else if (!state->surfaceIsClosed || (state->surfaceIsClosed && state->handles.size() > 2)) {
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

	} else if (model.size() > 1 && loadedModel) {
		if (key == GLFW_KEY_LEFT && action == GLFW_RELEASE) {
			selectedMesh--;
			if (selectedMesh < 0) selectedMesh = model.size() - 1;
			updateSelectedMesh();

		} else if (key == GLFW_KEY_RIGHT && action == GLFW_RELEASE) {
			selectedMesh++;
			if (selectedMesh == model.size()) selectedMesh = 0;
			updateSelectedMesh();
		}
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

	return std::sqrt(dx*dx + dy*dy);
}

bool Viewer::didPick(int index, const Vector& position, const ViewPane& pane)
{
	double d = pickDistance(index, position, pane);
	if (d < 6*EYE/cameras[index]->radius) {
		return true;
	}

	return false;
}

WedgeIter Viewer::closestWedge(const ViewPane& pane, bool onBoundary)
{
	double minDistance = std::numeric_limits<double>::max();
	WedgeIter minWedge;

	if (onBoundary) {
		for (WedgeIter w: mesh->cutBoundary()) {
			double distance = pickDistance(1, w->uv, pane);

			if (distance < minDistance) {
				minDistance = distance;
				minWedge = w;
			}
		}

	} else {
		for (WedgeIter w  = mesh->wedges().begin(); w != mesh->wedges().end(); w ++) {
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
	WedgeCIter w = closestWedge(pane, state->setBoundaryAngles);

	if ((state->setBoundaryAngles && w->vertex()->onBoundary(false)) ||
		(!state->setBoundaryAngles && !w->vertex()->onBoundary(false))) {
		updateVertexHandles(w->vertex());
	}
}

bool Viewer::pickVertexHandle(const ViewPane& pane)
{
	for (VertexHandle handle: state->handles) {
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
	int nVertices = 0;
	for (int i = 0; i < model.size(); i++) {
		int nV = (int)model[i].vertices.size();

		if (pickedId < nVertices + nV) {
			if (index == 0) {
				if (selectedMesh != i) {
					// update selected mesh
					selectedMesh = i;
					updateSelectedMesh();

				} else if (!modelStates[i].editBoundary) {
					// update vertex handle
					VertexIter v = model[i].vertices.begin() + pickedId - nVertices;
					updateVertexHandles(v);
				}

				break;
			}
		}

		nVertices += nV;
	}
}

void Viewer::pick(int index)
{
	// draw
	for (int i = 0; i < model.size(); i++) {
		modelStates[i].renderMeshes[index]->draw(flatShader, true);
	}

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

void Viewer::drawRenderMeshes(const ViewPane& pane, int index)
{
	// update uniforms
	updateUniforms(index, pane.width, pane.height);

	if (pickingEnabled) {
		if (clickX >= pane.left && clickX <= pane.left + pane.width &&
			clickX == unclickX && clickY == unclickY) {
			if (!pickVertexHandle(pane)) {
				if (index == 0) {
					// clear viewport and pick
					clearViewport(pane);
					pick(index);

				} else if (index == 1 && !state->mappedToSphere) {
					if (state->editBoundary) pickKnot(pane);
					else pickVertexHandleUV(pane);
				}
			}
		}
	}

	// clear viewport
	clearViewport(pane);

	// draw model
	for (int i = 0; i < model.size(); i++) {
		// draw all meshes in 3D pane and only the selected mesh in the UV pane
		if (index == 0 || (index == 1 && i == selectedMesh)) {
			// update per mesh uniforms
			modelShader.use();
			bool drawPattern = !modelStates[i].surfaceIsClosed || (modelStates[i].surfaceIsClosed && modelStates[i].cut) || modelStates[i].mappedToSphere;
			glUniform1f(glGetUniformLocation(modelShader.program, "patternFactor"), drawPattern ? 0.5 : 0.0);
			glUniform1i(glGetUniformLocation(modelShader.program, "drawUVSphere"), modelStates[i].mappedToSphere);
			glUniform1f(glGetUniformLocation(modelShader.program, "ambient"), i != selectedMesh ? 0.4 : 0.1);

			// draw mesh
			if (index == 0 || plotType == PlotType::shaded || plotType == PlotType::normals) {
				modelStates[i].renderMeshes[index]->draw(modelShader);

			} else {
				modelStates[i].renderMeshes[index]->draw(flatShader);

				if (plotType == PlotType::constant) {
					glEnable(GL_CULL_FACE);
					glCullFace(GL_FRONT);
					glDisable(GL_DEPTH_TEST);
					modelStates[i].renderMeshes[index]->draw(flatShader);
					glEnable(GL_DEPTH_TEST);
					glDisable(GL_CULL_FACE);
				}
			}

			// draw cone and spline handles
			if (index == 0) {
				for (VertexHandle handle: modelStates[i].handles) {
					handle.sphere3D->draw(flatShader);
				}

			} else if (index == 1) {
				for (VertexHandle handle: modelStates[i].handles) {
					handle.sphereUV->draw(flatShader);
				}

				if (state->editBoundary) {
					for (int j = 0; j < (int)modelStates[i].splineHandles.size(); j++) {
						modelStates[i].splineHandles[j]->draw(flatShader);
						modelStates[i].splineLines[2*j]->draw(flatShader);
						modelStates[i].splineLines[2*j + 1]->draw(flatShader);
					}
				}
			}

			// draw wireframe and cut
			bool showCut = index == 0 && modelStates[i].cut;
			if (showWireframe) modelStates[i].renderMeshes[index]->draw(wireframeShader);
			if (showCut) modelStates[i].cut->draw(flatShader);
		}
	}
}

void Viewer::drawScene()
{
	clearViewport(fullPane);

	if (loadedModel) {
		drawRenderMeshes(surfacePane, 0);
		if (!state->surfaceIsClosed || (state->surfaceIsClosed && state->cut) || state->mappedToSphere) {
			drawRenderMeshes(uvPane, 1);
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

void Viewer::reportError(const std::string& error)
{
	new MessageDialog(gui, MessageDialog::Type::Warning, "Error", error);
}

int main(int argc, const char *argv[]) {
	if (argc == 2) Viewer::init(argv[1]);
	else std::cout << "USAGE: " << argv[0] << " OBJ_INPUT_PATH" << std::endl;

	return 0;
}
