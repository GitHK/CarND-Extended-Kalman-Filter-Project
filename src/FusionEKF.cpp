#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    /**
    TODO:
      * Finish initializing the FusionEKF.
      * Set the process and measurement noises
    */

    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    ekf_.P_ = MatrixXd(4, 4);

    // Covariance matrix initialization, velocity is known, thus 1 is a good value for this
    // velocity is unknown adding a big amout of uncertainty 100000
    ekf_.P_ <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    // extra matirxes initialization
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.Q_ = MatrixXd(4, 4);

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        /**
        TODO:
          * Initialize the state ekf_.x_ with the first measurement.
          * Create the covariance matrix.
          * Remember: you'll need to convert radar from polar to cartesian coordinates.
        */

        ekf_.x_ = VectorXd(4);

        // coordinates to be extracted from LIDAR or RADAR
        double x,y;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            cout << "EKF: first measurement RADAR" << endl;

            double rho = measurement_pack.raw_measurements_[0];
            double phi = measurement_pack.raw_measurements_[1];

            x = rho * cos(phi);
            y = rho * sin(phi);
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            cout << "EKF: first measurement LIDAR" << endl;

            x = measurement_pack.raw_measurements_[0];
            y = measurement_pack.raw_measurements_[1];
        }

        // store values
        ekf_.x_ << x, y, 0.f, 0.f;

        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }


    // a bit out of range with RADAR
    // if(measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    // return;

    // very bad accuracy with LIDAR
    //if(measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    //  return;


    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /**
     TODO:
       * Update the state transition matrix F according to the new elapsed time.
        - Time is measured in seconds.
       * Update the process noise covariance matrix.
       * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */

    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    // updating transition matrix F
    ekf_.F_ << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;

    double noise_ax = 9.0;
    double noise_ay = 9.0;

    double dt2 = pow(dt, 2);
    double dt3 = pow(dt, 3);
    double dt4 = pow(dt, 4);
    double dt4_4 = dt4 / 4;
    double dt3_2 = dt3 / 2;


    ekf_.Q_ << dt4_4 * noise_ax, 0, dt3_2 * noise_ax, 0,
            0, dt4_4 * noise_ay, 0, dt3_2 * noise_ay,
            dt3_2 * noise_ax, 0, dt2 * noise_ax, 0,
            0, dt3_2 * noise_ay, 0, dt2 * noise_ay;


    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
     TODO:
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        //cout << "EKF: calling RADAR update function" << endl;

        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        //cout << "EKF: calling LIDAR update function" << endl;

        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
