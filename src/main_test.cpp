/*
 * TestCase.cpp
 *
 *  Created on: Apr 24, 2019
 *      Author: mloay
 */

#include <gtest/gtest.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <vector>
#include <Eigen/Dense>
#include <math.h>
#include"test_cases.h"

using namespace std;
using namespace Eigen;

TEST(linearFilter,linearFilter)
{
    EXPECT_EQ(linearFilter(),true);
}

TEST(trackLinearFilter,trackLinearFilter)
{
    EXPECT_EQ(trackLinearFilter(),true);
}

TEST(CalculateJacobian,CalculateJacobian)
{
    EXPECT_EQ(CalculateJacobian(),true);
}

TEST(trackEKF,trackEKF)
{
    EXPECT_EQ(trackEKF(),true);
}

TEST(calculateRMSE,calculateRMSE)
{
    EXPECT_EQ(calculateRMSE(),true);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}