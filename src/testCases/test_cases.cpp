/*
 * TestCase.cpp
 *
 *  Created on: Apr 24, 2019
 *      Author: mloay
 */

/** @file test_cases.cpp
 *  @ingroup testCases
 *  @brief test cases
 */

/**
 *  @addtogroup testCases
 *  @{
 */

#define _USE_MATH_DEFINES

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <vector>
#include <Eigen/Dense>
#include <math.h>
#include"test_cases.h"
#include "kalmanFilter.h"

Eigen::VectorXd g_t1 (const Eigen::Ref<const Eigen::VectorXd>& mean, const void *p_args=NULL);
Eigen::MatrixXd g_t1_prime (const Eigen::Ref<const Eigen::VectorXd>& mean, const void *p_args=NULL);

//Test cases


/**
 * @brief Test case 1 linearFilter
 *        Test The linear update step.
 *
 */
bool linearFilter(void)
{
    /*******************************************************************************
     *  Initialization                                                             *
     *******************************************************************************/
    //set state dimension
    int n_x = 2;

    //set measurement dimension
    int n_z = 1;

    //create example std::vector for predicted state mean.
    Eigen::VectorXd x = Eigen::VectorXd(n_x);
    x << 0, 0;

    //create example matrix for predicted state covariance.
    Eigen::MatrixXd P = Eigen::MatrixXd(n_x,n_x);
    P.fill(0.0);
    P << 1000, 0, 0, 1000;

    //output matrix
    Eigen::MatrixXd H = Eigen::MatrixXd (n_z, n_x);
    H<< 1, 0;

    //Sensor Noise Covariance matrix
    Eigen::MatrixXd R = Eigen::MatrixXd(n_z, n_z);
    R.fill(0.0);
    R.diagonal()<<1.0;

    //precoess noise covariance matrix Q
    Eigen::MatrixXd Q = Eigen::MatrixXd (n_x, n_x);
    Q<< 0.0, 0.0, 0.0, 0.0;

    /*******************************************************************************
     *  Set correct Answer                                                         *
     *******************************************************************************/
    //Correct Answer
    //x-state
    Eigen::VectorXd x_corr = Eigen::VectorXd(n_x);
    std::vector<Eigen::VectorXd> x_corr_list;
    x_corr << 0.999001,0.0;
    x_corr_list.push_back(x_corr);
    x_corr << 2.998,0.999002;
    x_corr_list.push_back(x_corr);
    x_corr << 3.99967,1.0;
    x_corr_list.push_back(x_corr);

    //P-state
    Eigen::MatrixXd p_corr = Eigen::MatrixXd(n_x,n_x);
    std::vector<Eigen::MatrixXd> p_corr_list;
    p_corr << 1001, 1000, 1000, 1000;
    p_corr_list.push_back(p_corr);
    p_corr << 4.99002, 2.99302, 2.99302, 1.99501;
    p_corr_list.push_back(p_corr);
    p_corr << 2.33189, 0.999168, 0.999168, 0.499501;
    p_corr_list.push_back(p_corr);

    /*******************************************************************************
     *  Set Measurement Input                                                      *
     *******************************************************************************/
    //set the measurement
    std::vector<MeasurementPackage> measurement_pack_list; 
    MeasurementPackage meas_package;

    meas_package.raw_measurements_ = Eigen::VectorXd(1);
    meas_package.raw_measurements_ << 1.0;
    measurement_pack_list.push_back(meas_package);
    meas_package.raw_measurements_ << 2.0;
    measurement_pack_list.push_back(meas_package);
    meas_package.raw_measurements_ << 3.0;
    measurement_pack_list.push_back(meas_package);

    /*******************************************************************************
     *  Run Main Algorithm Loop                                                    *
     *******************************************************************************/
    std::vector<Eigen::VectorXd> x_rest_list;
    std::vector<Eigen::MatrixXd> p_rest_list;
    for (unsigned int n = 0; n < measurement_pack_list.size(); ++n)
    {
        // Calculate Innovation
        Eigen::VectorXd zpred = H * x;
        Eigen::VectorXd z_meas = measurement_pack_list[n].raw_measurements_;
        Eigen::VectorXd Y = z_meas - zpred;

        //Calculate Kalman Gain
        Eigen::MatrixXd K = kalmanFilter::CalculateKalmanGain(P, H, R);

        //perform Update step
        kalmanFilter::update(x, P, Y, H, K);

        //perform Predict step
        kalmanFilter::predict(x, P, Q, g_t1, g_t1_prime);

        // //collect result
        x_rest_list.push_back(x);
        p_rest_list.push_back(P);
    }

    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
        bool r= true;
    for (unsigned int n = 0; n < measurement_pack_list.size(); ++n)
    {
        r = r && ((x_rest_list[n] - x_corr_list[n]).norm() < 0.01);
        r = r && ((p_rest_list[n] - p_corr_list[n]).norm() < 0.001);
    }
    return r;
}

/**
 * @brief Test case 2 trackLinearFilter
 *        Test both linear update step and linear prediction step .
 *
 */
bool trackLinearFilter(void)
{
    /*******************************************************************************
     *  Parse input file                                                         *
     *******************************************************************************/
    // hardcoded input file with laser and radar measurements
    std::string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt";
    std::ifstream in_file(in_file_name_.c_str(), std::ifstream::in);

    if (!in_file.is_open()) 
    {
        std::cout << "Cannot open input file: " << in_file_name_ << std::endl;
    }

    /**********************************************
     *  Set Measurements                          *
     **********************************************/
    // prep the measurement packages (each line represents a measurement at a timestamp)
    std::vector<MeasurementPackage> measurement_pack_list;
    std::string line;
    int i=0;
    while (getline(in_file, line)&& (i<=3))
    {
        MeasurementPackage meas_package;
        std::string sensor_type;
        std::istringstream iss(line);
        long long timestamp;

        // reads first element from the current line
        iss >> sensor_type;

        if (sensor_type.compare("L") == 0)
        {
            // laser measurement
            // read measurements at this timestamp

            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = Eigen::VectorXd(2);
            float px;
            float py;
            iss >> px;
            iss >> py;
            meas_package.raw_measurements_ << px, py;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }
        else if (sensor_type.compare("R") == 0)
        {
            // radar measurement
            // read measurements at this timestamp
            continue;
        }
        i++;
    }

    /*******************************************************************************
     *  Run Kalman Filter and save the output                                    *
     *******************************************************************************/
    std::vector<Eigen::VectorXd> x_rest_list;
    std::vector<Eigen::MatrixXd> p_rest_list;
    // Create a KF instance
    kfApp tracking(4);
    tracking.kd_.x.fill(0);
    tracking.kd_.P = Eigen::MatrixXd(4, 4);
    tracking.kd_.P << 1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1000, 0,
                        0, 0, 0, 1000;

    tracking.noise_ax = 5;
    tracking.noise_ay = 5;
    tracking.std_laspx_ = 0.15;
    tracking.std_laspy_ = 0.15;

    for (unsigned int n = 0; n < measurement_pack_list.size(); ++n)
    {
        // Call the KF-based fusion
        tracking.ProcessMeasurement(measurement_pack_list[n]);

        //collect result
        if(n>0)
        {
            x_rest_list.push_back(tracking.kd_.x);
            p_rest_list.push_back(tracking.kd_.P);
        }
    }
    /*******************************************************************************
     *  Set correct Answer                                                         *
     *******************************************************************************/
    //Correct Answer
    //x-state
    Eigen::VectorXd x_corr = Eigen::VectorXd(4);
    std::vector<Eigen::VectorXd> x_corr_list;
    x_corr << 0.96749,0.405862,4.58427,-1.83232;
    x_corr_list.push_back(x_corr);
    x_corr << 0.958365,0.627631,0.110368, 2.04304;
    x_corr_list.push_back(x_corr);
    x_corr << 1.34291,0.364408, 2.32002,-0.722813;
    x_corr_list.push_back(x_corr);

    //P-state
    Eigen::MatrixXd p_corr = Eigen::MatrixXd(4,4);
    std::vector<Eigen::MatrixXd> p_corr_list;

    p_corr << 0.0224541, 0, 0.204131, 0,
                0, 0.0224541, 0, 0.204131,
                0.204131, 0, 92.7797, 0,
                0, 0.204131, 0, 92.7797;
    p_corr_list.push_back(p_corr);

    p_corr << 0.0220006, 0, 0.210519, 0,
                0, 0.0220006, 0, 0.210519,
                0.210519, 0, 4.08801, 0,
                0, 0.210519, 0, 4.08801;
    p_corr_list.push_back(p_corr);

    p_corr << 0.0185328, 0, 0.109639, 0,
                0, 0.0185328, 0, 0.109639,
                0.109639, 0, 1.10798, 0,
                0, 0.109639, 0, 1.10798;
    p_corr_list.push_back(p_corr);

    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
        bool r= true;

    for (unsigned int n = 0; n < measurement_pack_list.size()-1; ++n)
    {
        r = r && ((x_rest_list[n] - x_corr_list[n]).norm() < 0.01);
        r = r && ((p_rest_list[n] - p_corr_list[n]).norm() < 0.01);
    }
    return r;
}

/**
 * @brief Test case 3 CalculateJacobian
 *        Test the jacobian Calculation.
 *
 */
bool CalculateJacobian(void)
{
    /**********************************************
     *  Set jacobia Inputs                        *
     **********************************************/
    // Create a KF instance
    kfApp tracking(4);
    tracking.kd_.x<< 1, 2, 0.2, 0.4;

    /*******************************************************************************
     *  Calculate the Jacobian                                                     *
     *******************************************************************************/
    Eigen::MatrixXd H = tracking.h_prime_(tracking.kd_.x);

    /*******************************************************************************
     *  Set correct Answer                                                         *
     *******************************************************************************/
    //Correct Answer
    //P-state
    Eigen::MatrixXd Hj_corr = Eigen::MatrixXd(3,4);
    Hj_corr << 0.447214, 0.894427, 0, 0,
                -0.4, 0.2, 0, 0,
                0, 0, 0.447214, 0.894427;

    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
    bool r= true;
    r = r && ((H -Hj_corr).norm() < 0.01);

    return r;
}

/**
 * @brief Test case 4 trackEKF
 *        Test both linear/NoneLinear update step and linear prediction step .
 *
 */
bool trackEKF(void)
{
    /*******************************************************************************
     *  Parse input file                                                         *
     *******************************************************************************/
    // hardcoded input file with laser and radar measurements
    std::string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt";
    std::ifstream in_file(in_file_name_.c_str(), std::ifstream::in);

    if (!in_file.is_open()) 
    {
        std::cout << "Cannot open input file: " << in_file_name_ << std::endl;
    }

    /**********************************************
     *  Set Measurements                          *
     **********************************************/
    // prep the measurement packages (each line represents a measurement at a timestamp)
    std::vector<MeasurementPackage> measurement_pack_list;
    std::string line;
    int i=0;
    while (getline(in_file, line)&& (i<=5))
    {
        MeasurementPackage meas_package;
        std::string sensor_type;
        std::istringstream iss(line);
        long long timestamp;

        // reads first element from the current line
        iss >> sensor_type;

        if (sensor_type.compare("L") == 0)
        {
            // laser measurement
            // read measurements at this timestamp

            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = Eigen::VectorXd(2);
            float px;
            float py;
            iss >> px;
            iss >> py;
            meas_package.raw_measurements_ << px, py;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }
        else if (sensor_type.compare("R") == 0)
        {
            // radar measurement
            // read measurements at this 
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = Eigen::VectorXd(3);
            float ro;
            float phi;
            float ro_dot;
            iss >> ro;
            iss >> phi;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro, phi, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }
        i++;
    }

    /*******************************************************************************
     *  Run Kalman Filter and save the output                                    *
     *******************************************************************************/
    std::vector<Eigen::VectorXd> x_rest_list;
    std::vector<Eigen::MatrixXd> p_rest_list;
    // Create a KF instance
    kfApp tracking(4);
    tracking.kd_.x.fill(0);
    tracking.kd_.P = Eigen::MatrixXd(4, 4);
    tracking.kd_.P << 1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1000, 0,
                        0, 0, 0, 1000;

    tracking.noise_ax = 5;
    tracking.noise_ay = 5;
    tracking.std_laspx_ = 0.15;
    tracking.std_laspy_ = 0.15;

    for (unsigned int n = 0; n < measurement_pack_list.size(); ++n)
    {
        // Call the KF-based fusion
        tracking.ProcessMeasurement(measurement_pack_list[n]);
        //collect result
        if(n>0)
        {
            x_rest_list.push_back(tracking.kd_.x);
            p_rest_list.push_back(tracking.kd_.P);
        }
    }
    /*******************************************************************************
     *  Set correct Answer                                                         *
     *******************************************************************************/
    //Correct Answer
    //x-state
    Eigen::VectorXd x_corr = Eigen::VectorXd(4);
    std::vector<Eigen::VectorXd> x_corr_list;
    x_corr << 0.722628,0.567796,3.70663,-0.56481;
    x_corr_list.push_back(x_corr);
    x_corr << 0.969665,0.413432,5.6934,-2.08598;
    x_corr_list.push_back(x_corr);
    x_corr << 0.984782,0.681457, 2.3165,0.760458;
    x_corr_list.push_back(x_corr);
    x_corr <<  1.03169,0.698205,2.11118,0.916794;
    x_corr_list.push_back(x_corr);
    x_corr << 1.21554,0.670913,2.35642,0.325553;
    x_corr_list.push_back(x_corr);

    //P-state
    Eigen::MatrixXd p_corr = Eigen::MatrixXd(4,4);
    std::vector<Eigen::MatrixXd> p_corr_list;

    p_corr <<   0.0744763,   0.0957463,   0.0140901,  -0.0088403,
                0.0957463,    0.127007, -0.00884025,  0.00923985,
                0.0140901, -0.00884025,     180.933,    -137.793,
                -0.0088403,  0.00923985,    -137.793,     105.334;
    p_corr_list.push_back(p_corr);

    p_corr <<    0.0212348, -0.000763264,     0.275495,    -0.208923,
                -0.000763264,     0.020816,    -0.208923,      0.16087,
                0.275495,    -0.208923,      5.94417,      -4.3339,
                -0.208923,      0.16087,      -4.3339,      3.56638;
    p_corr_list.push_back(p_corr);

    p_corr <<     0.012367,   0.00418933,    0.0424686,   -0.0499424,
                0.00418933,   0.00439293,   0.00839503, -0.000486848,
                0.0424686,   0.00839503,     0.265165  ,   -0.19538,
                -0.0499424, -0.000486848,     -0.19538,     0.490509;
    p_corr_list.push_back(p_corr);

    p_corr <<    0.00974513, 0.000737499,   0.0318128,  -0.0346475,
                0.000737499,  0.00442744, -0.00294044,   0.0215166,
                0.0318128, -0.00294044,    0.198251,   -0.107771,
                -0.0346475,   0.0215166,   -0.107771,    0.387774;
    p_corr_list.push_back(p_corr);

    p_corr << 0.00769929, 0.00194051,  0.0192605, -0.0125547,
                0.00194051, 0.00382965, 0.00150358,   0.011961,
                0.0192605, 0.00150358,   0.109024, -0.0288252,
                -0.0125547,   0.011961, -0.0288252,   0.165914;
    p_corr_list.push_back(p_corr);

    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
        bool r= true;

    for (unsigned int n = 0; n < measurement_pack_list.size()-1; ++n)
    {
        r = r && ((x_rest_list[n] - x_corr_list[n]).norm() < 0.01);
        r = r && ((p_rest_list[n] - p_corr_list[n]).norm() < 0.01);
    }
    return r;
}

/**
 * @brief Test case 5 calculateRMSE
 *        Test the RMSE Calculation.
 *
 */
bool calculateRMSE(void)
{
    /**********************************************
     *  Set RMSE Inputs                        *
     **********************************************/
    std::vector<Eigen::VectorXd> estimations;
    std::vector<Eigen::VectorXd> ground_truth;
    // Tools tools;

    // the input list of estimations
    Eigen::VectorXd e(4);
    e << 1, 1, 0.2, 0.1;
    estimations.push_back(e);
    e << 2, 2, 0.3, 0.2;
    estimations.push_back(e);
    e << 3, 3, 0.4, 0.3;
    estimations.push_back(e);

    // the corresponding list of ground truth values
    Eigen::VectorXd g(4);
    g << 1.1, 1.1, 0.3, 0.2;
    ground_truth.push_back(g);
    g << 2.1, 2.1, 0.4, 0.3;
    ground_truth.push_back(g);
    g << 3.1, 3.1, 0.5, 0.4;
    ground_truth.push_back(g);

    /*******************************************************************************
     *  Calculate the RMSE                                                    *
     *******************************************************************************/
        Eigen::VectorXd rmse(4);
    // rmse = tools.CalculateRMSE(estimations, ground_truth);

    /*******************************************************************************
     *  Set correct Answer                                                         *
     *******************************************************************************/
    //Correct Answer
    //rmse correct
    Eigen::VectorXd rmse_correct(4);
    rmse_correct << 0.1, 0.1, 0.1, 0.1;

    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
        bool r= true;
    r = r && ((rmse - rmse_correct).norm() < 0.01);

    return r;
}

/**
 * @brief g_t1 Function 
 *  which calculates the mean state vector based dynamic model.
 *
 * @param[in] mean
 *  the state vector {VectorXd&}.
 * 
 * @param[in] p_args
 *  Extra arguments {const void *}.
 * 
 * @return F.x
 *  the mean state vector {{VectorXd}}.
 */
Eigen::VectorXd g_t1 (const Eigen::Ref<const Eigen::VectorXd>& mean, const void *p_args)
{
    //create state transition matrix for predicted state covariance.
    Eigen::MatrixXd F = Eigen::MatrixXd(2,2);
    F<< 1, 1, 0, 1;
    return F* mean;

}

/**
 * @brief g_prime the derivative of g_function.
 *  In linear case it shall return the state transition Matrix.
 *  In non-linear it shall return the jacobians. 
 *
 * @param[in] mean
 *  the state vector {VectorXd&}.
 * 
 * @param[in] p_args
 *  Extra arguments {const void *}.
 * 
 * @return F 
 *  the state transition matrix {MatrixXd}.
 */
Eigen::MatrixXd g_t1_prime (const Eigen::Ref<const Eigen::VectorXd>& mean, const void *p_args)
{
    //create state transition matrix for predicted state covariance.
    Eigen::MatrixXd F = Eigen::MatrixXd(2,2);
    F<< 1, 1, 0, 1;
    return F;
}

/**
 *  @}
 */
