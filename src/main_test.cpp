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

TEST(KFTest,linearFilter)
{
    EXPECT_EQ(linearFilter(),true);
}

TEST(KFTest,trackLinearFilter)
{
    EXPECT_EQ(trackLinearFilter(),true);
}

TEST(EKFTest,CalculateJacobian)
{
    EXPECT_EQ(CalculateJacobian(),true);
}

TEST(EKFTest,trackEKF)
{
    EXPECT_EQ(trackEKF(),true);
}

TEST(RMSE,calculateRMSE)
{
    EXPECT_EQ(calculateRMSE(),true);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}