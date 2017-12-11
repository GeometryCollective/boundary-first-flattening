#include "ConePlacement.h"
#include <limits>

void ConePlacement::initializeConeSet(vector<VertexIter>& cones, Mesh& mesh)
{
    if (mesh.boundaries.size() > 0) {
        // designate all boundary vertices as cone vertices
        for (WedgeCIter w: mesh.cutBoundary()) {
            cones.push_back(w->vertex());
        }

    } else if (mesh.eulerCharacteristic() != 0) {
        // surface is closed, select the vertex with largest curvature as a cone
        // singularity if the euler characteristic is greater than 0 and vice versa
        VertexIter cone;
        double curvature = numeric_limits<double>::infinity();
        if (mesh.eulerCharacteristic() > 0) curvature *= -1;

        for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
            double angleDefect = v->angleDefect();
            bool isCandidateCone = mesh.eulerCharacteristic() > 0 ? curvature < angleDefect :
                                                                    curvature > angleDefect;
            if (isCandidateCone) {
                curvature = angleDefect;
                cone = v;
            }
        }

        cones.push_back(cone);
    }
}

void ConePlacement::computeTargetAngles(DenseMatrix& C, const DenseMatrix& K, const SparseMatrix& A,
                                        const vector<VertexIter>& cones, unordered_map<int, bool>& isCone,
                                        WedgeIndex& index, Mesh& mesh)
{
    // collect indices of regular and cone vertices
    int S = (int)cones.size();
    if (S > 0) {
        isCone.clear();
        vector<int> s;
        for (WedgeCIter w: mesh.cutBoundary()) {
            isCone[w->vertex()->index] = true;
            s.push_back(index[w]);
        }

        for (int i = 0; i < S; i++) {
            if (isCone.find(cones[i]->index) == isCone.end()) {
                isCone[cones[i]->index] = true;
                s.push_back(index[cones[i]->wedge()]);
            }
        }

        vector<int> n;
        for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
            if (isCone.find(v->index) == isCone.end()) n.push_back(index[v->wedge()]);
        }

        // extract submatrices
        SparseMatrix Ann = A.submatrix(n, n);
        SparseMatrix Ans = A.submatrix(n, s);
        DenseMatrix Kn = K.submatrix(n);
        DenseMatrix Ks = K.submatrix(s);

        // compute target curvatures
        for (int i = 0; i < S; i++) {
            int I = index[cones[i]->wedge()];

            // solve LGn = δn
            DenseMatrix Gn, Gs(S);
            Gs(i) = 1;
            DenseMatrix delta = -(Ans*Gs);
            Ann.L.solvePositiveDefinite(Gn, delta);

            // Cs = Ks + Gn^T Kn
            C(I) = Ks(i);
            if (n.size() > 0) C(I) += (Gn.transpose()*Kn)(0);
        }
    }
}

void ConePlacement::computeScaleFactors(DenseMatrix& u, const DenseMatrix& C,
                                        const DenseMatrix& K, SparseMatrix& A)
{
    DenseMatrix rhs = -(K - C);
    A.L.solvePositiveDefinite(u, rhs);
}

void ConePlacement::normalizeAngles(DenseMatrix& C, double normalizationFactor)
{
    double sum = 0.0;
    for (int i = 0; i < (int)C.nRows(); i++) sum += C(i);
    for (int i = 0; i < (int)C.nRows(); i++) C(i) *= normalizationFactor/sum;
}

void ConePlacement::findConesAndPrescribeAngles(int S, vector<VertexIter>& cones, DenseMatrix& coneAngles,
                                                shared_ptr<BFFData> data, Mesh& mesh)
{
    int N = data->N;
    SparseMatrix& A(data->A);
    DenseMatrix K = vcat(data->K, data->k);
    WedgeIndex& index(data->index);
    unordered_map<int, bool> isCone;

    // initialize cone set
    initializeConeSet(cones, mesh);
    if (mesh.boundaries.size() == 0) S -= (int)cones.size();

    // compute target angles
    DenseMatrix C(N);
    computeTargetAngles(C, K, A, cones, isCone, index, mesh);

    for (int i = 0; i < S; i++) {
        // compute scale factors
        DenseMatrix u;
        computeScaleFactors(u, C, K, A);

        // add vertex with maximum (abs.) scaling to cone set
        VertexIter cone;
        double maxU = -numeric_limits<double>::infinity();
        for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
            if (!v->onBoundary() && isCone.find(v->index) == isCone.end()) {
                int i = index[v->wedge()];

                double absU = abs(u(i));
                if (maxU < absU) {
                    maxU = absU;
                    cone = v;
                }
            }
        }

        cones.push_back(cone);

        // compute target angles
        computeTargetAngles(C, K, A, cones, isCone, index, mesh);
    }

    if (mesh.boundaries.size() == 0) normalizeAngles(C, 2*M_PI*mesh.eulerCharacteristic());
    else cones = vector<VertexIter>(cones.begin() + (cones.size() - S), cones.end());
    coneAngles = C.submatrix(0, data->iN);
}
