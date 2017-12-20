#pragma once

#include "Mesh.h"
#include "Cholesky.h"
#include <memory>

class BFFData;

class BFF {
public:
    // constructor
    BFF(Mesh& mesh);

    // computes automatic flattening with minimal area distortion
    //    -boundaryData stores either the target angles at boundary vertices (if
    //     givenScaleFactors is false) -OR- the target scale factors (if givenScaleFactors
    //     is true)
    // (resulting flattening is stored in Corner::uv for each corner of this->mesh)
    void flatten(DenseMatrix& boundaryData, bool givenScaleFactors);

    // computes flattening with prescribed cones
    //    - the matrix C is just a Vx1 vector of cone angles (usually zero for most vertices)
    //    - surfaceHasCut should be set to true if the locations of cones changed
    // (resulting flattening is stored in Corner::uv for each corner of this->mesh)
    void flattenWithCones(const DenseMatrix& C, bool surfaceHasNewCut);

    // uniformization over the unit disk
    // (resulting flattening is stored in Corner::uv for each corner of this->mesh)
    void flattenToDisk();

    // conformally maps a genus 0 mesh to a sphere
    // (resulting flattening is stored in Corner::uv for each corner of this->mesh)
    void mapToSphere();
    
    // flattens to target shape
    void flattenToShape(const vector<Vector>& gamma);

    // member that stores the compatible curvatures (scale factors) given target scale factors (curvatures)
    // exposed for direct editing
    DenseMatrix compatibleTarget;

    // pointer to current bff data
    shared_ptr<BFFData> data;
    
    // normalize curvatures to sum to 2pi
    void closeCurvatures(DenseMatrix& ktilde) const;

protected:
    // Copies scale factors u of the uncut surface into a and g that store the interior
    // and boundary scale factors u (resp.) of the cut surface
    void processCut(const DenseMatrix& u, DenseMatrix& a, DenseMatrix& g);

    // computes scale factors u from target boundary edge lengths ltilde
    void computeBoundaryScaleFactors(const DenseMatrix& ltilde, DenseMatrix& u) const;

    // converts dirichlet boundary data g to neumann boundary data h
    void convertDirichletToNeumann(const DenseMatrix& phi, DenseMatrix& g,
                                   DenseMatrix& h, bool surfaceHasCut = false);

    // converts neumann boundary data h to dirichlet boundary data g
    void convertNeumannToDirichlet(const DenseMatrix& phi, const DenseMatrix& h,
                                   DenseMatrix& g);

    // computes target lengths lstar from target scale factors u; returns total length
    double computeTargetBoundaryLengths(const DenseMatrix& u, DenseMatrix& lstar) const;

    // computes dual lengths from target lengths lstar; returns total length
    double computeTargetDualBoundaryLengths(const DenseMatrix& lstar, DenseMatrix& ldual) const;

    // computes target lengths lstar from target uv coordinates; returns total length
    double computeTargetBoundaryLengthsUV(DenseMatrix& lstar) const;

    // computes dual lengths from target uv coordinates; returns total length
    double computeTargetDualBoundaryLengthsUV(DenseMatrix& ldual) const;

    // modifies target lengths lstar to ensure gamma closes
    void closeLengths(const DenseMatrix& lstar, const DenseMatrix& Ttilde, DenseMatrix& ltilde) const;

    // constructs closed loop with prescribed data lstar and ktilde
    void constructBestFitCurve(const DenseMatrix& lstar, const DenseMatrix& ktilde,
                               DenseMatrix& gammaRe, DenseMatrix& gammaIm) const;

    // solves laplace equation with dirichlet boundary values g
    void extendHarmonic(const DenseMatrix& g, DenseMatrix& h);

    // constructs extension of a closed loop gamma
    void extendCurve(const DenseMatrix& gammaRe, const DenseMatrix& gammaIm,
                     DenseMatrix& a, DenseMatrix& b, bool conjugate);

    // normalizes flattening
    void normalize();

    // flattens given target scale factors u and geodesic curvatures ktilde
    void flatten(const DenseMatrix& u, const DenseMatrix& ktilde, bool conjugate);

    // stereographically projects disk of a specified radius to a sphere
    void projectStereographically(VertexCIter pole, double radius, const vector<Vector>& uvs);

    // members
    Mesh& mesh;
    shared_ptr<BFFData> inputSurfaceData;
    shared_ptr<BFFData> cutSurfaceData;
};

class BFFData {
public:
    // constructor
    BFFData(Mesh& mesh);

    // members
    int iN, bN, N; // number of interior, boundary and total wedges
    WedgeIndex index; // wedge indices
    WedgeIndex bIndex; // wedge boundary indices

private:
    // assigns indices to wedges
    void indexWedges();

    // computes integrated gaussian and geodesic curvatures K and k
    void computeIntegratedCurvatures();

    // computes boundary edge lengths l
    void computeBoundaryLengths();

    // builds zero neumann laplace matrix A
    void buildLaplace();

    // computes integrated curvatures and boundary lengths, and builds and prefactors laplace matrix
    void init();

    // members
    Mesh& mesh;
    DenseMatrix K, k, l; // gaussian and geodesic curvatures, length
    SparseMatrix A, Aii, Aib, Abb; // laplace matrix and submatrices

    friend class BFF;
    friend class ConePlacement;
};
