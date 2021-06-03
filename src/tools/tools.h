/*
 * tools.h
 *
 *  Created on: Apr 18, 2019
 *      Author: mody
 */

/** @file tools.h
 *  @ingroup Tool
 *  @brief Tools class
 */

/**
 *  @addtogroup Tool
 *  @{
 */

#ifndef TOOLS_H_
#define TOOLS_H_
#include <iostream>
#include <fstream>
#include <iostream>
#include <sstream>
#include "ground_truth_package.h"
#include "measurement_package.h"
#include <vector>
class Tools 
{
	public:
	/**
	* Constructor.
	*/
	Tools();

	/**
	* Destructor.
	*/
	~Tools();

	// Members
	std::vector<MeasurementPackage> measurement_pack_list;
	std::vector<Eigen::VectorXd> ground_truth;
	std::vector<Eigen::VectorXd> estimations;
	std::vector<double> nis_radar;
	std::vector<double> nis_lidar;

	//CalculateRMSE Calculates the root mean square error
	Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                  const std::vector<Eigen::VectorXd> &ground_truth);

	//CalculateRMSE Calculates the root mean square error
	void check_files(std::ifstream& in_file, std::string& in_name);

	//CalculateRMSE Calculates the root mean square error
	void parseData (const std::string& in_name);
	
};

#endif /* TOOLS_H_ */
/**
 *  @}
 */
