#include "bff/project/Bff.h"
#include "bff/mesh/MeshIO.h"
#include "bff/project/HoleFiller.h"
#include "bff/project/Generators.h"
#include "bff/project/ConePlacement.h"
#include "bff/project/Cutter.h"

using namespace bff;

void printUsage(const std::string& programName)
{
	std::cout << "USAGE: "
			  << programName
			  << " OBJ_INPUT_PATH "
			  << "OBJ_OUTPUT_PATH "
			  << "[--nCones=N_CONES] "
			  << "[--flattenToDisk] "
			  << "[--mapToSphere] "
			  << "[--normalizeUVs] "
			  << "[--writeOnlyUVs]"
			  << std::endl;
}

bool doesArgExist(const std::string& arg, const std::string& searchStr)
{
	return arg.find(searchStr) != std::string::npos;
}

bool parseArg(const std::string& arg, const std::string& searchStr, std::string& value)
{
	if (doesArgExist(arg, searchStr)) {
		value = arg.substr(arg.find_first_of(searchStr[searchStr.size()-1]) + 1);
		return true;
	}

	return false;
}

void parseArgs(int argc, const char *argv[], std::string& inputPath, std::string& outputPath,
			   int& nCones, bool& flattenToDisk, bool& mapToSphere, bool& normalizeUVs,
			   bool& writeOnlyUVs)
{
	if (argc < 3) {
		// input and/or output path not specified
		printUsage(argv[0]);
		exit(EXIT_FAILURE);

	} else {
		// parse arguments
		inputPath = argv[1];
		outputPath = argv[2];
		std::string nConesStr;

		for (int i = 3; i < argc; i++) {
			if (parseArg(argv[i], "--nCones=", nConesStr)) nCones = std::stoi(nConesStr);
			if (doesArgExist(argv[i], "--flattenToDisk")) flattenToDisk = true;
			if (doesArgExist(argv[i], "--mapToSphere")) mapToSphere = true;
			if (doesArgExist(argv[i], "--normalizeUVs")) normalizeUVs = true;
			if (doesArgExist(argv[i], "--writeOnlyUVs")) writeOnlyUVs = true;
		}
	}

	// if cones are specified, set flattenToDisk and mapToSphere to false
	if (nCones > 0) {
		flattenToDisk = false;
		mapToSphere = false;
	}
}

void loadModel(const std::string& inputPath, Model& model,
			   std::vector<bool>& surfaceIsClosed)
{
	std::string error;
	if (MeshIO::load(inputPath, model, error)) {
		int nMeshes = model.size();
		surfaceIsClosed.resize(nMeshes, false);

		for (int i = 0; i < nMeshes; i++) {
			Mesh& mesh = model[i];
			int nBoundaries = (int)mesh.boundaries.size();

			if (nBoundaries >= 1) {
				// mesh has boundaries
				int eulerPlusBoundaries = mesh.eulerCharacteristic() + nBoundaries;

				if (eulerPlusBoundaries == 2) {
					// fill holes if mesh has more than 1 boundary
					if (nBoundaries > 1) {
						if (HoleFiller::fill(mesh)) {
							// all holes were filled
							surfaceIsClosed[i] = true;
						}
					}

				} else {
					// mesh probably has holes and handles
					HoleFiller::fill(mesh, true);
					Generators::compute(mesh);
				}

			} else if (nBoundaries == 0) {
				if (mesh.eulerCharacteristic() == 2) {
					// mesh is closed
					surfaceIsClosed[i] = true;

				} else {
					// mesh has handles
					Generators::compute(mesh);
				}
			}
		}

	} else {
		std::cerr << "Unable to load file: " << inputPath << ". " << error << std::endl;
		exit(EXIT_FAILURE);
	}
}

void flatten(Model& model, const std::vector<bool>& surfaceIsClosed,
			 int nCones, bool flattenToDisk, bool mapToSphere)
{
	int nMeshes = model.size();
	for (int i = 0; i < nMeshes; i++) {
		Mesh& mesh = model[i];
		BFF bff(mesh);

		if (nCones > 0) {
			std::vector<VertexIter> cones;
			DenseMatrix coneAngles(bff.data->iN);
			int S = std::min(nCones, (int)mesh.vertices.size() - bff.data->bN);

			if (ConePlacement::findConesAndPrescribeAngles(S, cones, coneAngles, bff.data, mesh)
				== ConePlacement::ErrorCode::ok) {
				if (!surfaceIsClosed[i] || cones.size() > 0) {
					Cutter::cut(cones, mesh);
					bff.flattenWithCones(coneAngles, true);
				}
			}

		} else {
			if (surfaceIsClosed[i]) {
				if (mapToSphere) {
					bff.mapToSphere();

				} else {
					std::cerr << "Surface is closed. Either specify nCones or mapToSphere." << std::endl;
					exit(EXIT_FAILURE);
				}

			} else {
				if (flattenToDisk) {
					bff.flattenToDisk();

				} else {
					DenseMatrix u(bff.data->bN);
					bff.flatten(u, true);
				}

				mesh.projectUvsToPcaAxis();
			}
		}
	}
}

void writeModelUVs(const std::string& outputPath, Model& model,
				   const std::vector<bool>& surfaceIsClosed, bool mapToSphere,
				   bool normalizeUVs, bool writeOnlyUVs)
{
	int nMeshes = model.size();
	std::vector<bool> mappedToSphere(nMeshes, false);
	for (int i = 0; i < nMeshes; i++) {
		if (surfaceIsClosed[i]) {
			mappedToSphere[i] = mapToSphere;
		}
	}

	if (!MeshIO::write(outputPath, model, mappedToSphere, normalizeUVs, writeOnlyUVs)) {
		std::cerr << "Unable to write file: " << outputPath << std::endl;
		exit(EXIT_FAILURE);
	}
}

int main(int argc, const char *argv[]) {
	// parse command line options
	std::string inputPath = "";
	std::string outputPath = "";
	int nCones = 0;
	bool flattenToDisk = false;
	bool mapToSphere = false;
	bool normalizeUVs = false;
	bool writeOnlyUVs = false;
	parseArgs(argc, argv, inputPath, outputPath, nCones, flattenToDisk,
			  mapToSphere, normalizeUVs, writeOnlyUVs);

	// load model
	Model model;
	std::vector<bool> surfaceIsClosed;
	loadModel(inputPath, model, surfaceIsClosed);

	// set nCones to 8 for closed surfaces`
	for (int i = 0; i < model.size(); i++) {
		if (surfaceIsClosed[i] && !mapToSphere && nCones < 3) {
			std::cout << "Setting nCones to 8." << std::endl;
			nCones = 8;
		}
	}

	// flatten
	flatten(model, surfaceIsClosed, nCones, flattenToDisk, mapToSphere);

	// write model uvs to output path
	writeModelUVs(outputPath, model, surfaceIsClosed, mapToSphere,
				  normalizeUVs, writeOnlyUVs);

	return 0;
}
