#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

//#define DEBUG_LIDAR_KF
#define DEBUG_LIDAR_UKF
//#define DEBUG_OUTPUT

class UKF {
public:

    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    ///* if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;

    ///* if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_;

    VectorXd x_pred_;
    ///* augmented state vector
    VectorXd x_aug_;

    ///* state covariance matrix
    MatrixXd P_;
    MatrixXd P_aug_;

    ///*process noise  matrix
    MatrixXd Q_;

    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_;

    ///* time when the state is true, in us
    long long time_us_;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    ///* Laser measurement noise standard deviation position1 in m
    double std_laspx_;

    ///* Laser measurement noise standard deviation position2 in m
    double std_laspy_;

    ///* Radar measurement noise standard deviation radius in m
    double std_radr_;

    ///* Radar measurement noise standard deviation angle in rad
    double std_radphi_;

    ///* Radar measurement noise standard deviation radius change in m/s
    double std_radrd_ ;

    ///* Weights of sigma points
    VectorXd weights_;

    ///* State dimension
    int n_x_;

    ///* Augmented state dimension
    int n_aug_;

    ///* Sigma point spreading parameter
    double lambda_;

    ///* laser measurement noise covariance matrix
    MatrixXd R_laser_;

    ///* radar measurement noise covariance matrix
    MatrixXd R_radar_;

    ///* the current NIS for radar
    double NIS_radar_;

    ///* the current NIS for laser
    double NIS_laser_;

    /**
    * Constructor
    */
    UKF();

    /**
    * Destructor
    */
    virtual ~UKF();

    /**
    * ProcessMeasurement
    * @param meas_package The latest measurement data of either radar or laser
    */
    void ProcessMeasurement(MeasurementPackage meas_package);

    /**
    * Prediction Predicts sigma points, the state, and the state covariance
    * matrix
    * @param delta_t Time between k and k+1 in s
    */
    void Prediction(double delta_t);

    /**
    * Updates the state and the state covariance matrix using a laser measurement
    * @param meas_package The measurement at k+1
    */
    void UpdateLidar(MeasurementPackage meas_package);

    /**
    * Updates the state and the state covariance matrix using a radar measurement
    * @param meas_package The measurement at k+1
    */
    void UpdateRadar(MeasurementPackage meas_package);

    void GenerateAugmentedSigmaPoints(MatrixXd* Xsig_out);

    void PredictSigmaPoints(const MatrixXd& X_aug_sig, const double delta_t, MatrixXd* X_pred_sig_out);

    void PredictMeanAndCovariance();

    void PredictRadarMeasurement(MatrixXd* Zsig_out, VectorXd* z_pred_out, MatrixXd* S_out);

    void UpdateRadarState(const MatrixXd& Zsig, const VectorXd& z_pred, const MatrixXd& S, const MeasurementPackage& meas_package);

    void PredictLidarMeasurement(MatrixXd* Zsig_out, VectorXd* z_pred_out, MatrixXd* S_out);

    void UpdateLidarState(const MatrixXd& Zsig, const VectorXd& z_pred, const MatrixXd& S, const MeasurementPackage& meas_package);

    void PredictStateInFuture(double delta_t);

};

#endif /* UKF_H */