#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include <unsupported/Eigen/KroneckerProduct>
#include "knn.h"

using namespace Eigen;
using namespace std;
typedef SparseMatrix<float> SpMat;
typedef NaturalOrdering<int> Natural;
typedef COLAMDOrdering<int> COLAMD;
typedef AMDOrdering<int> AMD;
typedef knncpp::Matrixi Matrixi;

class ICP
{
    public:
    SpMat A_sp;
    float dist_diff;
    Matrixi indices;
    MatrixXf distances;

    ICP()
    {
        dist_diff = 7e-2;
    } 

    MatrixXf pose2transformation(VectorXf delta)
    {
        MatrixXf R(2, 2);
        VectorXf t(2);
        float c = cos(delta(0));
        float s = sin(delta(0));
        R << c, s,
            -s, c;
        t << delta(1), delta(2);
        MatrixXf T = MatrixXf::Identity(3, 3);
        T.block(0, 0, 2, 2) = R;
        T(seq(0, 1), 2) = t;

        return T;
    }

    VectorXf optimize(MatrixXf corres_source, MatrixXf corres_target, MatrixXf T)
    {
        int n = corres_source.cols();
        
    }

    void icp(MatrixXf target, MatrixXf source)
    {

        knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>> kdtree(target);
        kdtree.setBucketSize(16);
        kdtree.setSorted(true);
        kdtree.setTakeRoot(true);
        kdtree.setMaxDistance(0.07);
        kdtree.setThreads(6);
        kdtree.build();

        indices;
        distances;

    }


};