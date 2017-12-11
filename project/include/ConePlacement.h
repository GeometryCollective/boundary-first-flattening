#pragma once

#include "Bff.h"

class ConePlacement {
public:
    // finds S cone singularities and prescibes angles
    static void findConesAndPrescribeAngles(int S, vector<VertexIter>& cones, DenseMatrix& coneAngles,
                                            shared_ptr<BFFData> data, Mesh& mesh);

private:
    // initializes the set of cones based on whether the mesh has a boundary
    // or its euler characteristic is non zero
    static void initializeConeSet(vector<VertexIter>& cones, Mesh& mesh);

    // computes target curvatures for the cone set
    static void computeTargetAngles(DenseMatrix& C, const DenseMatrix& K, const SparseMatrix& A,
                                    const vector<VertexIter>& cones, unordered_map<int, bool>& isCone,
                                    WedgeIndex& index, Mesh& mesh);

    // computes scale factors
    static void computeScaleFactors(DenseMatrix& u, const DenseMatrix& C,
                                    const DenseMatrix& K, SparseMatrix& A);

    // normalizes cone angles to sum to 2pi times euler characteristic
    static void normalizeAngles(DenseMatrix& C, double normalizationFactor);
};
