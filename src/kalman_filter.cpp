#include "kalman_filter.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    //cout << "KalmanFilter: Predict()" << endl;
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::ContinueUpdate(const VectorXd &y) {
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}


void KalmanFilter::Update(const VectorXd &z) {
    //cout << "KalmanFilter: Update()" << endl;

    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;

    ContinueUpdate(y);
}

#define KF_2PI 2.f * M_PI

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    //cout << "KalmanFilter: UpdateEKF()" << endl;

    double px = x_(0);
    double py = x_(1);
    double vx = x_(2);
    double vy = x_(3);

    double rho = sqrt(px * px + py * py);
    double theta = atan2(py, px);
    double rho_dot = (px * vx + py * vy) / rho;

    VectorXd h = VectorXd(3);
    h << rho, theta, rho_dot;

    VectorXd y = z - h;
    while (y(1) > M_PI || y(1) < -M_PI)
        y(1) = y(1) > M_PI
               ? y(1) - KF_2PI
               : y(1) + KF_2PI;

    ContinueUpdate(y);
}
