#define _USE_MATH_DEFINES
/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

// #include "KalmanConfig.h"
// #ifdef USE_EKF
// #include "kfApp.h"
// #endif
#include<iostream>
#include "tools.h"
#include "kfApp.h"


int main()
{
    /*******************************************************************************
     *  Parse input file                                                         *
     *******************************************************************************/
    
    std::string dataPath("../data/obj_pose-laser-radar-synthetic-input.txt");
    std::cout << dataPath << std::endl; 
    Tools tools;
    tools.parseData(dataPath);

    /*******************************************************************************
     *  Run Kalman Filter and save the output                                    *
     *******************************************************************************/
    // Create a KF instance
    kfApp tracking(4);


    // start filtering from the second frame (the speed is unknown in the first frame)
    size_t N = tools.measurement_pack_list.size();

    for (size_t k = 0; k < N; ++k)
    {
        // convert tracking x vector to cartesian to compare to ground truth
        Eigen::VectorXd temp_ukf_x_cartesian = Eigen::VectorXd(4);

        // Call the KF-based fusion
        tracking.ProcessMeasurement(tools.measurement_pack_list[k]);

            // 2.output the measurements
        if (tools.measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER)
        {
            tools.nis_lidar.push_back(tracking.kd_.nis);
        }
        else if (tools.measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR)
        {
            tools.nis_radar.push_back(tracking.kd_.nis);
        }

        // 1.output the estimation
        double x  = tracking.kd_.x(kfApp::XPOS) ; // pos1 - est
        double y  = tracking.kd_.x(kfApp::YPOS) ; // pos2 - est
        double vx = tracking.kd_.x(kfApp::XVEL) ; // vx -est
        double vy = tracking.kd_.x(kfApp::YVEL) ; // vy -est
        temp_ukf_x_cartesian << x, y, vx, vy;
        tools.estimations.push_back(temp_ukf_x_cartesian);
    }
    // compute the accuracy (RMSE)
    std::cout << "Accuracy - RMSE:" << std::endl; 
    std::cout<< tools.CalculateRMSE(tools.estimations, tools.ground_truth) << std::endl;


    /*******************************************************************************
     *  plot the results                                                           *
     *******************************************************************************/

    return 0;
}

