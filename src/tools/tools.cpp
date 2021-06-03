/*
 * tools.cpp
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */

/** @file tools.cpp
 *  @ingroup Tool
 *  @brief Tools class
 */

/**
 *  @addtogroup Tool
 *  @{
 */

#include "tools.h"

#include <stdlib.h>
#include "Eigen/Dense"
#include "measurement_package.h"
#include "KalmanConfig.h"
#include "ground_truth_package.h"
#include <iostream>
#include <fstream>
#include <iostream>
#include <sstream>
/**
 * @brief kfApp The constructor for Tools.
 *
 */
Tools::Tools() = default;

/**
 * @brief Tools The destructor for Tools.
 *
 */
Tools::~Tools() = default;

// Members
std::vector<MeasurementPackage> measurement_pack_list;
std::vector<Eigen::VectorXd> ground_truth;
std::vector<Eigen::VectorXd> estimations;
/**
 * @brief check_files check if file is exist and it can be opened.
 *
 * @param[in] in_file Handle object to file for(read/write) operations {ifstream}.
 * 
 * @param[in] in_name file name {string}.
 *
 */
void Tools::check_files (std::ifstream& in_file, std::string& in_name)
{
    if (!in_file.is_open())
    {
        std::cerr << "Cannot open input file: " << in_name << std::endl;
        exit(EXIT_FAILURE);
    }
}

void Tools::parseData (const std::string& in_name)
{
    std::string in_file_name_ = in_name;
    std::ifstream in_file_(in_file_name_.c_str(), std::ifstream::in);
    check_files(in_file_, in_file_name_);

    /**********************************************
     *  Set Measurements & grountruth             *
     **********************************************/
    // prep the measurement packages (each line represents a measurement at a timestamp)
    std::string line;

    while (getline(in_file_, line))
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
        //   radar measurement
        //   read measurements at this timestamp

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

        // read ground truth data to compare later
        float x_gt;
        float y_gt;
        float vx_gt;
        float vy_gt;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;
        Eigen::VectorXd gt = Eigen::VectorXd(4);
        gt << x_gt, y_gt, vx_gt, vy_gt;
        ground_truth.push_back(gt);
    }

    if (in_file_.is_open())
    {
        in_file_.close();
    }
}

/**
 * @brief CalculateRMSE Calculated the Root Mean Square.
 *
 * @param[in] estimations The output of the filter {VectorXd}.
 * 
 * @param[in] ground_truth The ground truth corresponding to filter's output {VectorXd}.
 * 
 * @param[out]  Vector contains the RMSE for each state {VectorXd}.
 *
 */
Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                     const std::vector<Eigen::VectorXd> &ground_truth) 
{
  
    Eigen::VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0)
	{
		std::cout << "Invalid estimation or ground_truth data" << std::endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i)
	{

		Eigen::VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();


	//return the result
	return rmse;
}

/**
 *  @}
 */
