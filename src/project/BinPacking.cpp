#include "bff/project/BinPacking.h"
#include "Rect.h"
#include "GuillotineBinPack.h"
#include <limits>

namespace bff {

using namespace rbp;

bool attemptPacking(int boxLength, double unitsPerInt,
					const std::vector<Rect>& rectangles,
					std::vector<Vector>& newUvIslandCenters,
					std::vector<bool>& isUvIslandFlipped,
					Vector& modelMinBounds, Vector& modelMaxBounds)
{
	int n = (int)rectangles.size();
	modelMinBounds = Vector(std::numeric_limits<double>::max(),
							std::numeric_limits<double>::max());
	modelMaxBounds = Vector(std::numeric_limits<double>::lowest(),
							std::numeric_limits<double>::lowest());
	GuillotineBinPack packer(boxLength, boxLength);

	for (int i = 0; i < n; i++) {
		Rect rect = packer.Insert(rectangles[i].width, rectangles[i].height, true,
								  GuillotineBinPack::FreeRectChoiceHeuristic::RectBestAreaFit,
								  GuillotineBinPack::GuillotineSplitHeuristic::SplitMinimizeArea);

		// check for failure
		if (rect.width == 0 || rect.height == 0) {
			if (rectangles[i].width != 0 && rectangles[i].height != 0) {
				return false;
			}
		}

		// check if flipped
		isUvIslandFlipped[i] = rect.width == rectangles[i].width ? false : true;

		// compute new centers
		newUvIslandCenters[i] = Vector(rect.x + rect.width/2.0, rect.y + rect.height/2.0);
		newUvIslandCenters[i] *= unitsPerInt;
		modelMinBounds.x = std::min((double)rect.x, modelMinBounds.x);
		modelMinBounds.y = std::min((double)rect.y, modelMinBounds.y);
		modelMaxBounds.x = std::max((double)(rect.x + rect.width), modelMaxBounds.x);
		modelMaxBounds.y = std::max((double)(rect.y + rect.height), modelMaxBounds.y);
	}

	return true;
}

void BinPacking::pack(const Model& model,
					  const std::vector<bool>& isSurfaceMappedToSphere,
					  std::vector<Vector>& originalUvIslandCenters,
					  std::vector<Vector>& newUvIslandCenters,
					  std::vector<bool>& isUvIslandFlipped,
					  Vector& modelMinBounds, Vector& modelMaxBounds)
{
	// compute bounding boxes
	int n = model.size();
	double totalArea = 0.0;
	std::vector<Vector> minBounds(n, Vector(std::numeric_limits<double>::max(),
											std::numeric_limits<double>::max()));
	std::vector<Vector> maxBounds(n, Vector(std::numeric_limits<double>::lowest(),
											std::numeric_limits<double>::lowest()));

	for (int i = 0; i < n; i++) {
		// compute island radius
		double radius = 1e-8;
		if (isSurfaceMappedToSphere[i]) {
			for (WedgeCIter w = model[i].wedges().begin(); w != model[i].wedges().end(); w++) {
				radius = std::max(w->uv.norm(), radius);
			}

		} else {
			radius = model[i].radius;
		}

		// scale UVs by radius and compute bounds
		for (WedgeCIter w = model[i].wedges().begin(); w != model[i].wedges().end(); w++) {
			Vector uv = w->uv;
			if (isSurfaceMappedToSphere[i]) {
				uv /= radius;
				uv.x = 0.5 + atan2(uv.z, uv.x)/(2*M_PI);
				uv.y = 0.5 - asin(uv.y)/M_PI;

			} else {
				uv *= radius;
			}

			minBounds[i].x = std::min(uv.x, minBounds[i].x);
			minBounds[i].y = std::min(uv.y, minBounds[i].y);
			maxBounds[i].x = std::max(uv.x, maxBounds[i].x);
			maxBounds[i].y = std::max(uv.y, maxBounds[i].y);
		}

		totalArea += (maxBounds[i].x - minBounds[i].x)*(maxBounds[i].y - minBounds[i].y);
	}

	// quantize boxes
	originalUvIslandCenters.resize(n);
	std::vector<Rect> rectangles(n);
	int minBoxLength = 10000;
	int maxBoxLength = 10000;
	double unitsPerInt = std::sqrt(totalArea)/(double)maxBoxLength;

	for (int i = 0; i < n; i++) {
		int minX = static_cast<int>(std::floor(minBounds[i].x/unitsPerInt));
		int minY = static_cast<int>(std::floor(minBounds[i].y/unitsPerInt));
		int maxX = static_cast<int>(std::ceil(maxBounds[i].x/unitsPerInt));
		int maxY = static_cast<int>(std::ceil(maxBounds[i].y/unitsPerInt));

		int width = maxX - minX;
		int height = maxY - minY;
		rectangles[i] = Rect{minX, minY, width, height};
		originalUvIslandCenters[i].x = (minX + maxX)/2.0;
		originalUvIslandCenters[i].y = (minY + maxY)/2.0;
		originalUvIslandCenters[i] *= unitsPerInt;
	}

	// pack islands
	newUvIslandCenters.resize(n);
	isUvIslandFlipped.resize(n);
	int iter = 0;

	do {
		if (attemptPacking(maxBoxLength, unitsPerInt, rectangles,
						   newUvIslandCenters, isUvIslandFlipped,
						   modelMinBounds, modelMaxBounds)) {
			break;
		}

		minBoxLength = maxBoxLength;
		maxBoxLength = static_cast<int>(std::ceil(minBoxLength*1.2));
		iter++;
	} while (iter < 50);

	if (iter < 50 && n < 5000) {
		// binary search on box length
		minBoxLength = 5000;
		maxBoxLength += 1;

		while (minBoxLength <= maxBoxLength) {
			int boxLength = (minBoxLength + maxBoxLength)/2;
			if (boxLength == minBoxLength) break;

			if (attemptPacking(boxLength, unitsPerInt, rectangles,
							   newUvIslandCenters, isUvIslandFlipped,
							   modelMinBounds, modelMaxBounds)) {
				maxBoxLength = boxLength;

			} else {
				minBoxLength = boxLength;
			}
		}

		attemptPacking(maxBoxLength, unitsPerInt, rectangles,
					   newUvIslandCenters, isUvIslandFlipped,
					   modelMinBounds, modelMaxBounds);
	}

	modelMinBounds *= unitsPerInt;
	modelMaxBounds *= unitsPerInt;
}

} // namespace bff
