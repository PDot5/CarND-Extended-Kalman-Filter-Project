#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
* Constructor.
*/
FusionEKF::FusionEKF()
{
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

    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

    // x_ mean variance mapping array for prediction
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

    H_laser_ << 1, 0, 0, 0,
        0, 1, 0, 0;

    ekf_.H_ = MatrixXd(4, 4);
    ekf_.H_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
    /**
    * Initialization
    */
    if (!is_initialized_)
    {
        /**
        * TODO: Initialize the state ekf_.x_ with the first measurement.
        * TODO: Create the covariance matrix.
        * You'll need to convert radar from polar to cartesian coordinates.
        */
        cout << " First Measurement Received";
        // first measurement
        cout
            << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;

        // create process covariance matrix
        ekf_.Q_ = Eigen::MatrixXd(4, 4);

        float px;
        float py;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            // TODO: Convert radar from polar to cartesian coordinates
            //         and initialize state.

            cout << "init radar" << endl;

            float rho = measurement_pack.raw_measurements_[0];
            float phi = measurement_pack.raw_measurements_[1];

            px = rho * cos(phi);
            py = rho * sin(phi);
        }
        if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        {
            // TODO: Initialize state.
            cout << "init laser" << endl;

            px = measurement_pack.raw_measurements_[0];
            py = measurement_pack.raw_measurements_[1];
        }
        // Handle if px, py small
        if (fabs(px) < 0.0001)
        {
            px = 0.1;
            cout << "init px is too small" << endl;
        }
        if (fabs(py) < 0.0001)
        {
            cout << "inti py is too small" << endl;
        }
        ekf_.x_ << px, py, 0, 0;
        cout << "ekf_.x_: " << ekf_.x_ << endl;

        previous_timestamp_ = measurement_pack.timestamp_;
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /**
    * Prediction
    */

    /**
    * TODO: Update the state transition matrix F according to the new elapsed time.
    * Time is measured in seconds.
    * TODO: Update the process noise covariance matrix.
    * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
    */

    cout << "Start predicting" << endl;
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
    cout << "dt: " << dt << endl;
    previous_timestamp_ = measurement_pack.timestamp_;

    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    cout << "Modify F matrix" << endl;

    //Modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    cout << "F_: " << ekf_.F_ << endl;

    // set noise values
    float noise_ax = 9.0;
    float noise_ay = 9.0;

    // update the process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
        0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
        dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
        0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

    cout << "Finished Q Update." << endl;
    ekf_.Predict();
    cout << "Predicted" << endl;
    /**
    * Update
    */

    /**
    * TODO:
    * - Use the sensor type to perform the update step.
    * - Update the state and covariance matrices.
    */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        // TODO: Radar updates

        cout << "Radar update" << endl;

        ekf_.hx_ = VectorXd(3);

        float px = ekf_.x_[0];
        float py = ekf_.x_[1];
        float vx = ekf_.x_[2];
        float vy = ekf_.x_[3];

        float rho;
        float phi;
        float rhodot;

        if (fabs(px) < 0.0001 or fabs(py) < 0.0001)
        {

            if (fabs(px) < 0.0001)
            {
                px = 0.0001;
                cout << "px too small" << endl;
            }

            if (fabs(py) < 0.0001)
            {
                py = 0.0001;
                cout << "py too small" << endl;
            }

            rho = sqrt(px * px + py * py);
            phi = 0;
            rhodot = 0;
        }
        else
        {
            rho = sqrt(px * px + py * py);
            phi = atan2(py, px); //  arc tangent of y/x, in the interval [-pi,+pi] radians.
            rhodot = (px * vx + py * vy) / rho;
        }

        ekf_.hx_ << rho, phi, rhodot;

        // set H_ to Hj when updating with a radar measurement
        Hj_ = tools.CalculateJacobian(ekf_.x_);

        // don't update measurement if we can't compute the Jacobian
        if (Hj_.isZero(0))
        {
            cout << "Hj is zero" << endl;
            return;
        }

        ekf_.H_ = Hj_;

        ekf_.R_ = R_radar_;

        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
    else
    {
        // TODO: Laser updates
        cout << "Laser update" << endl;
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}

// References:
// https://www.haidynmcleod.com/extended-kalman-filter
// https://github.com/jessicayung/self-driving-car-nd
// https://medium.com/@serrano_223/extended-kalman-filters-for-dummies-4168c68e2117
// https://github.com/Heych88/udacity-sdcnd-extended-kalman-filter
// https://cmake.org/install/
// https://tuannguyen68.gitbooks.io/learning-cmake-a-beginner-s-guide/content/chap1/chap1.html
// https://www.youtube.com/watch?v=TpQv0k2ZQjo
// https://www.youtube.com/watch?v=TpQv0k2ZQjo
