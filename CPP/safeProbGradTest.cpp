#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include "eiquadprog.hpp"
#include "sparseF1TENTHMPC.hpp"
#include <chrono>
#include <random>
#include <functional>

#define PI 3.1415926535

using namespace Eigen;
using namespace std;

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

MatrixXf traj_data;

class CSVData
{
    public:
    MatrixXf data;
    string filename;

    CSVData(string filename_, MatrixXf data_)
    {
        filename = filename_;
        data = data_;
    }

    void writeToCSVfile()
    {
        ofstream file(filename.c_str());
        file << data.format(CSVFormat);
        file.close();
    }

    MatrixXf readFromCSVfile()
    {
        vector<float> matrixEntries;
        ifstream matrixDataFile(filename);
        string matrixRowString;
        string matrixEntry;
        int matrixRowNumber = 0;
    
        while (getline(matrixDataFile, matrixRowString))
        {
            stringstream matrixRowStringStream(matrixRowString);
            while (getline(matrixRowStringStream, matrixEntry, ','))
            {
                matrixEntries.push_back(stod(matrixEntry));
            }
            matrixRowNumber++;
        }
        
        return Map<Matrix<float, Dynamic, Dynamic, RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
    }
};

class VehicleDynamics
{
    public:
    float m, lf, lr, l, B, Jw, Iz, R, b0, b1, Cl, Cs, T, g, Fz, c1, c2, c3, w1, w2;
    ArrayXf q1, q2, q3, q4, epi;
    VehicleDynamics()
    {
        m = 1500.00;
        lf = 1.070;
        lr = 1.605;
        l = lf + lr;
        B = 1.517/2;
        Iz = 2600.00;
        Jw = 5.00;
        R = 0.316;
        b0 = 1.25;
        b1 = 5;
        g = 9.81;
        Fz = m*g;
        c1 = 3e-2;
        c2 = 3.0639e2;
        c3 = 0.00;
        q1 = ArrayXf::Zero(4); q1 << -1, 1, -1, 1;
        q2 = ArrayXf::Zero(4); q2 << -1, 1, 1, -1;
        q3 = ArrayXf::Zero(4); q3 << 1, 1, 0, 0;
        q4 = ArrayXf::Zero(4); q4 << 0, 0, 1, 1;
        epi = ArrayXf::Constant(4, 1, 1e-4);
        w1 = 0.0;
        w2 = 0.0;
    }

    void brownian(float std)
    {
        unsigned seed = chrono::system_clock::now().time_since_epoch().count();
        default_random_engine generator(seed);
        normal_distribution<double> distribution(0.0, std);
        auto normal = [&] (double) {return distribution(generator);};
        VectorXf d = VectorXf::NullaryExpr(2, normal);
        w1 += 1e-3*d(0);
        w2 += 1e-3*d(1);
    }

    ArrayXf mu(ArrayXf s)
    {
        brownian(14.5e-2);
        return (c1 + (w1) + 1.00e-2)*(1 - exp(-c2*s)) - (c3)*s;
    }

    ArrayXf smoothMax(ArrayXf x1, ArrayXf x2)
    {
        ArrayXf P = ArrayXf::Constant(x1.rows(), x1.cols(), 1e-3);
        return 0.50*(x1 + x2 + sqrt(square(x2 - x1) + P));
    }

    ArrayXf smoothMin(ArrayXf x1, ArrayXf x2)
    {
        ArrayXf P = ArrayXf::Constant(x1.rows(), x1.cols(), 1e-3);
        return -0.50*(-x1 - x2 + sqrt(square(x2 - x1) + P));
    }

    ArrayXf expit(ArrayXf x)
    {
        return (tanh(25*x) + 1)*0.50;
    }

    MatrixXf tireForces(VectorXf x)
    {
        MatrixXf S_stack(4, 2);
        MatrixXf FTire(4, 2);
        float beta = lr/l*tan(x(10));
        ArrayXf vCOG = ArrayXf::Constant(4, 1, x(seq(3, 4)).matrix().norm());
        ArrayXf vR = x(seq(6, 9))*R;
        ArrayXf vTire = vCOG + x(5)*(q1*B*cos(beta) - q1*q2*lf*sin(beta)); 
        float alphaF = -beta + x(10) - lf*x(5)/vCOG(0);
        float alphaR = -beta + lr*x(5)/vCOG(0);
        ArrayXf Sl = (vR*(q3*cos(alphaF) + q4*cos(alphaR)) - vTire) / smoothMax(smoothMax(vR*(q3*cos(alphaF) + q4*cos(alphaR)), vTire), epi);
        ArrayXf Ss = (q3*tan(alphaF) + q4*tan(alphaR))*expit(Sl) + (q3*sin(alphaF) + q4*sin(alphaR))*(vR/vTire)*expit(-Sl);
        S_stack << Sl, Ss;
        ArrayXf Sr = S_stack.rowwise().norm();
        ArrayXf Mu = mu(Sr);
        ArrayXf Fl = Sl*Mu*Fz/Sr;
        ArrayXf Fs = Ss*Mu*Fz/Sr;
        FTire.row(0) = (Fl(seq(0, 1))*cos(alphaF) + Fs(seq(0, 1))*sin(alphaF))*cos(x(10)) - (Fs(seq(0, 1))*cos(alphaF) - Fl(seq(0, 1))*sin(alphaF))*sin(x(10));
        FTire.row(1) = (Fl(seq(0, 1))*cos(alphaF) + Fs(seq(0, 1))*sin(alphaF))*sin(x(10)) + (Fs(seq(0, 1))*cos(alphaF) - Fl(seq(0, 1))*sin(alphaF))*cos(x(10));
        FTire.row(2) = Fl(seq(2, 3))*cos(alphaR) + Fs(seq(2, 3))*sin(alphaR);
        FTire.row(3) = Fs(seq(2, 3))*cos(alphaR) - Fl(seq(2, 3))*sin(alphaR);     
        return FTire;
    }

    VectorXf fs(VectorXf x)
    {
        float Fd = b1*x(3);
        MatrixXf F = tireForces(x);
        VectorXf x_dot(11);
        x_dot << x(3)*cos(x(2)) - x(4)*sin(x(2)),
                 x(3)*sin(x(2)) + x(4)*cos(x(2)),
                 x(3)/l*tan(x(10)),
                 x(4)*x(5) + 1/m*(F(0, 0) + F(0, 1) + F(2, 0) + F(2, 1)) - Fd,
                 -x(3)*x(5) + 1/m*(F(1, 0) + F(1, 1) + F(3, 0) + F(3, 1)),
                 1/Iz*( lf*(F(1, 0) + F(1, 1)) - lr*(F(3, 0) + F(3, 1)) + B*(F(2, 1) + F(0, 1) - F(2, 0) - F(0, 0)) ),
                 1/Jw*( -R*F(0, 0) - b0*x(6)),
                 1/Jw*( -R*F(0, 1) - b0*x(7)),
                 1/Jw*( -R*F(2, 0) - b0*x(8)),
                 1/Jw*( -R*F(2, 1) - b0*x(9)),
                 0;
        return x_dot;
    }

    MatrixXf gs(VectorXf x)
    {
        MatrixXf G = MatrixXf::Zero(x.rows(), 2);
        G(3, 0) = cos(x(10)); G(4, 0) = sin(x(10));
        G(x.rows()-1, 1) = 1.0;
        return G;
    }

    VectorXf f_sys(VectorXf x, VectorXf u)
    {
        return fs(x.array()) + gs(x.array())*u;
    }

    VectorXf ctrl(float t, VectorXf x)
    {
        int k = int(1000*t);
        VectorXf u(2);
        u << 200*(traj_data(3, k) - x(3)),
             2.5*(traj_data(4, k) - x(10));
        return u;
    }

    VectorXf func(float t, VectorXf x)
    {
        VectorXf u = ctrl(t, x);
        return f_sys(x, u);
    }
};

class PID
{
    public:
    float Kp = 0.40;
    float Ki = 0.20;
    float Kd = 0.80;
    float e, e_d, acc;
    PID(float Kp_, float Ki_, float Kd_)
    {
        Kp = Kp_;
        Ki = Ki_;
        Kd = Kd_;
        e = 0.0;
        e_d = 0.00;
        acc = 0.0;
    }

    float compute(float y, float y_ref)
    {
        e = y_ref - y;
        acc += e*1e-3;
        e_d = e;
        return (Kp*e + Ki*acc + Kd*(e - e_d)*1e3);
    }
};

class SafeProbabilityGradient
{
    public:
    float Ts;
    int N_horizon;
    int N_episode;
    float epsilon = 0.10;
    MatrixXd Q, Ge, Gi;
    VectorXd u0, he, hi, u;
    VehicleDynamics vd;

    SafeProbabilityGradient(float Ts_init, int N_horizon_init, int N_episode_init)
    {
        Ts = Ts_init;
        N_horizon = N_horizon_init;
        N_episode = N_episode_init;
    }

    float smoothMax(float x1, float x2)
    {
        return 0.50*(x1 + x2 + sqrt((x2 - x1)*(x2 - x1) + 1e-4));
    }

    float smoothMin(float x1, float x2)
    {
        return -0.50*(-x1 - x2 + sqrt((x2 - x1)*(x2 - x1) + 1e-4));
    }

    VectorXf randNormal(int n, float std)
    {
        unsigned seed = chrono::system_clock::now().time_since_epoch().count();
        default_random_engine generator(seed);
        normal_distribution<double> distribution(0.0, std);
        auto normal = [&] (double) {return distribution(generator);};
        VectorXf d = VectorXf::NullaryExpr(2, normal);
        VectorXf v = VectorXf::Zero(n, 1);
        v(seq(3, 4)) = d;
        return v;
    }

    float phi(VectorXf x)
    {
        float eta = 0.85;
        float zeta = (eta*3e-2*1500*9.8)*(eta*3e-2*1500*9.8);
        MatrixXf F = vd.tireForces(x);
        float Ffl = F(0, 0)*F(0, 0) + F(1, 0)*F(1, 0);
        float Ffr = F(0, 1)*F(0, 1) + F(1, 1)*F(1, 1);
        float Frl = F(2, 0)*F(2, 0) + F(3, 0)*F(3, 0);
        float Frr = F(2, 1)*F(2, 1) + F(3, 1)*F(3, 1);
        float hfl = 1 - (Ffl/zeta);
        float hfr = 1 - (Ffr/zeta);
        float hrl = 1 - (Frl/zeta);
        float hrr = 1 - (Frr/zeta);
        return smoothMin(smoothMin(hfl, hfr), smoothMin(hrl, hrr));
    }

    VectorXf stepODE(float t, VectorXf x)
    {
        int n = x.rows();
        MatrixXf K = MatrixXf::Zero(n, 4);
        K.col(0) = Ts*vd.func(t, x);
        K.col(1) = Ts*vd.func(t + Ts/2, x + K.col(0)/2);
        K.col(2) = Ts*vd.func(t + Ts/2, x + K.col(1)/2);
        K.col(3) = Ts*vd.func(t + Ts, x + K.col(2));
        x += (K.col(0) + 2*K.col(1) + 2*K.col(2) + K.col(3))/6 + Ts*randNormal(n, 2.5);
        return x;

    }

    float F(float t, VectorXf x)
    {
        int k = round(t/Ts);
        VectorXf safeProb = VectorXf::Zero(N_episode);
        VectorXf x_s;
        float p;
        float t_step;
        for (int i = 0; i < N_episode; i++)
        {
            x_s = x;
            p = 1.0;
            t_step = t;
            for (int j = 0; j < N_horizon; j++)
            {
                x_s = stepODE(t_step, x_s);
                t_step += Ts;
                if (phi(x_s) + 0.050 <= -0.325)
                {
                    p = 0.0;
                    break;
                }
            }
            safeProb(i, 0) = p;
        }
        return safeProb.mean();
    }

    VectorXf dF(float t, VectorXf x)
    {
        int n = x.rows();
        VectorXf d_F(n);
        MatrixXf eye = epsilon*MatrixXf::Identity(n, n);
        for (int i = 0; i < n; i++)
        {
            d_F(i) = (F(t, x + eye.col(i)) - F(t, x - eye.col(i)))/2/epsilon;
        }
        return d_F;
    }
};

class SolveODE
{
    public:
    float Ts;
    int N, Nh;
    VectorXf x0, u0;
    MatrixXf Q, R, X_ref, U_ref;
    VehicleDynamics vd;

    VectorXf randNormal(int n, float std)
    {
        unsigned seed = chrono::system_clock::now().time_since_epoch().count();
        default_random_engine generator(seed);
        normal_distribution<double> distribution(0.0, std);
        auto normal = [&] (double) {return distribution(generator);};
        VectorXf d = VectorXf::NullaryExpr(2, normal);
        VectorXf v = VectorXf::Zero(n, 1);
        v(seq(3, 4)) = d;
        return v;
    }

    SolveODE(float Ts_, int N_, VectorXf x0_)
    {
        Ts = Ts_;
        N = N_;
        x0 = x0_;
        u0 = VectorXf::Zero(2);
        Q = MatrixXf::Identity(3, 3)*1.50;
        R = MatrixXf::Identity(2, 2)*1;
        Nh = 40;
    }

    void solve()
    {
        float a = 0.1;
        PID pid1(250, 10, 0.01);
        PID pid2(10, 10, 0);
        PID pid3(0, 0.80*a, 1*a);
        MPC mpc(x0.segment(0, 3), u0, Q, R, Nh);
        SafeProbabilityGradient spg(Ts, 100, 100);
        int n = x0.rows();
        MatrixXf X = MatrixXf::Zero(n, N);
        MatrixXf dF = MatrixXf::Zero(n, N);
        MatrixXf K = MatrixXf::Zero(n, 4);
        VectorXf U_mpc, u_cbf;
        VectorXf u(2);
        X.col(0) = x0;
        dF.col(0) = spg.dF(0, x0);
        float t, phi_dot;
        for (int i = 0; i < N-1; i++)
        {
            t = i*Ts;
            X_ref = traj_data(seq(0, 2), seq(i, i + Nh - 1));
            U_ref = traj_data(seq(3, 4), seq(i, i + Nh - 1));
            U_mpc = mpc.solveMPC(X(seq(0, 2), i), X_ref, U_ref);
            phi_dot = 0*pid3.compute(X(2, i), traj_data(2, i));
            u << pid1.compute(X(3, i), U_mpc(0)),
                 pid2.compute(X(10, i), U_mpc(1) + atan(2.675*phi_dot/X(3, i)));
            u_cbf = u;
            K.col(0) = Ts*vd.f_sys(X.col(i), u_cbf);
            K.col(1) = Ts*vd.f_sys(X.col(i) + K.col(0)/2, u_cbf);
            K.col(2) = Ts*vd.f_sys(X.col(i) + K.col(1)/2, u_cbf);
            K.col(3) = Ts*vd.f_sys(X.col(i) + K.col(2), u_cbf);
            X.col(i+1) = X.col(i) + (K.col(0) + 2*K.col(1) + 2*K.col(2) + K.col(3))/6;
            dF.col(i+1) = spg.dF(t, X.col(i+1));
        }
    CSVData sv1("state_data.csv", X);
    sv1.writeToCSVfile();
    CSVData sv2("prob_grad.csv", dF);
    sv2.writeToCSVfile();
    }
};

int main()
{
    cout << "Solving ODE... " << endl;
    auto start_time = chrono::system_clock::now();

    float v0 = 8.00;
    float R = 0.316;
    float Ts = 1e-3;
    float a_factor = 1.00;
    int N = 10;

    CSVData rd1("traj_data.csv", MatrixXf::Zero(1, 1));
    traj_data = rd1.readFromCSVfile();
    ArrayXf x0(11);
    x0 << traj_data(0, 0), traj_data(1, 0),  traj_data(2, 0),  1.41964e+01,
          -1.38715e-03,  1.30928e-01,  4.45212e+01,  4.51403e+01,
          4.45322e+01,  4.51511e+01,  2.61690e-02;

    SolveODE ode(Ts, N, x0);
    ode.solve();
    
    auto stop_time = chrono::system_clock::now();
    chrono::duration<double> diff = stop_time - start_time;

    cout << "Solve Complete! And time to run was: " << diff.count() << endl;

    return 0;
}