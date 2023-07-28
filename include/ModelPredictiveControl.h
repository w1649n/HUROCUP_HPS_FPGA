#ifndef  __ModelPredictiveControl_H__
#define  __ModelPredictiveControl_H__

#include "eiquadprog.h"
#include <unsupported/Eigen/MatrixFunctions>
#include "Inverse_kinematic.h"

using namespace Eigen;

class ModelPredictiveControl{
private:
	
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Matrix3d A, A_tilde;
    Vector3d B, B_tilde, xt, yt, xt_hat, yt_hat;
    Matrix<double, 1, 3> C;
    Matrix<double, 2, 3> C_tilde;
    Matrix2d Q, S;
    double R;
    int N;
    VectorXd r_y,r_x,u_y,u_x;
    MatrixXd Q_dou_bar, T_dou_bar, R_dou_bar, C_dou_bar, A_dou_hat, H_dou_bar, F_dou_bar, F_dou_bar_T, C_dou_tilde, A_dou_tilde, G_dou_bar_y, G_dou_bar_x, H;
    VectorXd y_max, y_min, x_max, x_min, F,F_T, constraint_vector_y, constraint_vector_x, upper_bound_y, lower_bound_y, upper_bound_x, lower_bound_x, fake_v;
    MatrixXd constraint_matrix_y, constraint_matrix_y_T, constraint_matrix_x, constraint_matrix_x_T, fake_m;
    
    
    // QP_solver QP_;

    void ZeroOrderHold(){
        Matrix3d B_tmp;
        B_tmp.setZero();
        double Ts =0.06;
        double Appr = 0.00001; // 離散積分迭代區間
        for(double t = 0.00001; t <= Ts ; t = t+Appr){
            B_tmp = B_tmp + expm(A * t).real() * Appr;
        }
        B_tilde = B_tmp * B;

        A_tilde = expm(A * Ts).real();

        cout << "A_tilde" << endl << A_tilde << endl << "B_tilde" << endl << B_tilde << endl <<"Zero order hold" << endl;

    }

    MatrixXcd expm(MatrixXd X){
        EigenSolver<MatrixXd> eig(X);
        VectorXcd eigValue = eig.eigenvalues();  //複數特徵值向量
        MatrixXcd eigVector = eig.eigenvectors();//複數特徵值向量矩陣

        MatrixXcd diagEigValue,expEigValue,ExpmEigValue;

            diagEigValue.resize(3,3);
            diagEigValue.setZero();
            diagEigValue.diagonal() = eigValue;

            expEigValue = diagEigValue.exp().diagonal();
            diagEigValue.setZero();
            diagEigValue.real() << 
                expEigValue.real()(0), 0, 0,
                0, expEigValue.real()(1), 0,
                0, 0, expEigValue.real()(2);
            diagEigValue.imag() << 
                expEigValue.imag()(0), 0, 0,
                0, expEigValue.imag()(1), 0,
                0, 0, expEigValue.imag()(2);            
        
            ExpmEigValue = eigVector * diagEigValue * eigVector.inverse();

        
        return ExpmEigValue;
    }

    void init(double g, double CoM_z){
        /* time horizen */
        N = 15;        

        A_tilde.setZero();
        B_tilde.setZero();

        /* 建立模型 */
        A << 0, 1, 0,
             g/CoM_z, 0, -(g/CoM_z),
             0, 0, 0;

        B << 0, 0, 1;
        C_tilde << 1, 0, 0,
                   0, 0, 1;
        C << 0, 0, 1;

        ZeroOrderHold();

        
        /* 初始狀態 */
        xt << 0, 0, 0;
        yt << 0, 0, 0;
        xt_hat << 0, 0, 0;
        yt_hat << 0, 0, 0;

        /* 權重 */
        S << 100, 0, 0, 1;
        Q << 100, 0, 0, 1;
        R = 1;

        Q_dou_bar.resize(C_tilde.cols() * N, C_tilde.cols() * N);
        T_dou_bar.resize(C_tilde.rows() * N, C_tilde.cols() * N);
        R_dou_bar.resize(N,N);

        C_dou_bar.resize(B_tilde.rows() * N, B_tilde.cols() * N);
        A_dou_hat.resize(A_tilde.rows() * N, A_tilde.cols());

        C_dou_tilde.resize(C.rows() * N, B_tilde.cols() * N);
        A_dou_tilde.resize(C.rows() * N, A_tilde.cols());

        Q_dou_bar.setZero();
        T_dou_bar.setZero();
        R_dou_bar.setZero();

        C_dou_bar.setZero();
        A_dou_hat.setZero();

        C_dou_tilde.setZero();
        A_dou_tilde.setZero();

        
        /* matrix.block(i,j,p,q) i,j 起始位置, p,q 區塊大小 */

        for(int i=0; i<=N-1; i++){

            if(i == N-1){
                Q_dou_bar.block(C_tilde.cols() * i, C_tilde.cols() * i, C_tilde.cols(), C_tilde.cols()) =
                C_tilde.transpose() * S * C_tilde;
            }else{
                Q_dou_bar.block(C_tilde.cols() * i, C_tilde.cols() * i, C_tilde.cols(), C_tilde.cols()) =
                C_tilde.transpose() * Q * C_tilde;
            }

            T_dou_bar.block(Q.rows() * i, C_tilde.cols() * i, Q.rows(), C_tilde.cols()) = 
            Q * C_tilde;
            R_dou_bar(i,i) = R;

            for(int l=0; l<=i; l++){

                    C_dou_bar.block(B_tilde.rows() * i, B_tilde.cols() * l, B_tilde.rows(), B_tilde.cols()) = 
                    A_tilde.pow(i-l) * B_tilde;

                    C_dou_tilde.block(C.rows() * i, B_tilde.cols() * l, C.rows(), B_tilde.cols()) = 
                    C * A_tilde.pow(i-1)  * B_tilde;

            }

            A_dou_hat.block(A_tilde.rows() * i, 0, A_tilde.rows(), A_tilde.cols()) = 
            A_tilde.pow(i+1);

            A_dou_tilde.block(C.rows() * i, 0, C.rows(), A_tilde.cols()) = C * A_tilde.pow(i+1);

        }
        

        H_dou_bar.resize(N, N);
        H_dou_bar.setZero();
        F_dou_bar_T.resize(A_dou_hat.cols()+T_dou_bar.rows(), C_dou_bar.cols());
        F_dou_bar_T.setZero();
        F_dou_bar.resize(C_dou_bar.cols(), A_dou_hat.cols()+T_dou_bar.rows());
        F_dou_bar.setZero();

        H_dou_bar = C_dou_bar.transpose() * Q_dou_bar * C_dou_bar + R_dou_bar ;
        F_dou_bar_T.block(0, 0, A_dou_hat.cols(), C_dou_bar.cols()) = A_dou_hat.transpose() * Q_dou_bar * C_dou_bar;
        F_dou_bar_T.block(A_dou_hat.cols(), 0, T_dou_bar.rows(), C_dou_bar.cols()) = -T_dou_bar * C_dou_bar;
        F_dou_bar = F_dou_bar_T.transpose();

        y_max.resize(N, 1);
        y_min.resize(N, 1);
        x_max.resize(N, 1);
        x_min.resize(N, 1);

        y_max.setZero();
        y_min.setZero();
        x_max.setZero();
        x_min.setZero();

        r_x.resize(2*N, 1);
        r_y.resize(2*N, 1);

        r_x.setZero();
        r_y.setZero();

        u_x.resize(2*N, 1);
        u_y.resize(2*N, 1);

        u_x.setZero();
        u_y.setZero();

        G_dou_bar_y.resize(2*N+3, 1);
        G_dou_bar_x.resize(2*N+3, 1);

        G_dou_bar_y.setZero();
        G_dou_bar_x.setZero();

        upper_bound_y.resize(N, 1);
        lower_bound_y.resize(N, 1);

        constraint_matrix_y.resize(C_dou_tilde.rows() * 2, C_dou_tilde.cols());
        constraint_matrix_x.resize(C_dou_tilde.rows() * 2, C_dou_tilde.cols());

        constraint_vector_y.resize(2*N, 1);
        constraint_vector_x.resize(2*N, 1);

        constraint_matrix_y_T.resize(C_dou_tilde.cols(), C_dou_tilde.rows() * 2);
        constraint_matrix_x_T.resize(C_dou_tilde.cols(), C_dou_tilde.rows() * 2);

        H.resize(H_dou_bar.rows(), H_dou_bar.cols());
        
        fake_m.resize(N,1);
        fake_v.resize(1,1);        
        
        fake_m.setZero();
        fake_v.setZero();

    }
    void update(MatrixXd foot, MatrixXd base, MatrixXd constraint_y, MatrixXd constraint_x){

        
        for(int i=0; i<N; i++){
            r_y(i*2) = base(1,i);   //參考質心
            r_y(i*2+1) = foot(1,i); //參考ZMP 
            r_x(i*2) = base(0,i);   //參考質心
            r_x(i*2+1) = foot(0,i); //參考ZMP 

            y_max(i) = constraint_y(0,i); // ZMP
            y_min(i) = constraint_y(1,i);
         
            x_max(i) = constraint_x(0,i); // ZMP
            x_min(i) = constraint_x(1,i);
        }
        
        
        upper_bound_y =  y_max - (A_dou_tilde * yt);
        lower_bound_y =  y_min - (A_dou_tilde * yt);
        upper_bound_x =  x_max - (A_dou_tilde * xt);
        lower_bound_x =  x_min - (A_dou_tilde * xt);

        
        constraint_matrix_y << -C_dou_tilde , C_dou_tilde;
        constraint_matrix_x << -C_dou_tilde , C_dou_tilde;
        constraint_vector_y << upper_bound_y , -lower_bound_y;
        constraint_vector_x << upper_bound_x , -lower_bound_x;

        
        constraint_matrix_y_T = constraint_matrix_y.transpose();
        constraint_matrix_x_T = constraint_matrix_x.transpose();

        
        G_dou_bar_y << yt, r_y;
        G_dou_bar_x << xt, r_x;
    };

    void run(){
        F = F_dou_bar * G_dou_bar_x;
        u_x.setZero();
        H = H_dou_bar;
        // cout << "MPC_run_x" << endl;
        Eigen::solve_quadprog( H, F, fake_m, fake_v, constraint_matrix_x_T, constraint_vector_x, u_x);
        xt = A_tilde * xt + B_tilde * u_x(0);
        
        F.setZero();
        F = F_dou_bar * G_dou_bar_y;
        u_y.setZero();
        H = H_dou_bar;
        
        // cout << constraint_matrix_y_T << endl<< "constraint_vector_y " << endl << constraint_vector_y << endl;
        // cout <<"F_dou_bar " << endl << F_dou_bar<< endl<<"G_dou_bar_y " << endl << G_dou_bar_y << endl <<"F " << endl << F << endl;
        // cout << "H " << endl << H << endl << "F " << endl << F << endl;
        // cout << "H " << endl << H << endl << "F " << endl << F << endl<< "constraint_matrix_y_T " << endl << constraint_matrix_y_T << endl<< "constraint_vector_y " << endl << constraint_vector_y << endl;
        // cout << "MPC_run_y =" << Eigen::solve_quadprog( H, F, fake_m, fake_v, constraint_matrix_y_T, constraint_vector_y, u_y) << endl;
        Eigen::solve_quadprog( H, F, fake_m, fake_v, constraint_matrix_y_T, constraint_vector_y, u_y);
        yt = A_tilde * yt + B_tilde * u_y(0);


    };
    
    void reset(){
        y_max.setZero();
        y_min.setZero();
        x_max.setZero();
        x_min.setZero();

        r_x.setZero();
        r_y.setZero();

        u_x.setZero();
        u_y.setZero();

        G_dou_bar_y.setZero();
        G_dou_bar_x.setZero();

        xt << 0, 0, 0;
        yt << 0, 0, 0;
    }
	void saveState();


};




#endif