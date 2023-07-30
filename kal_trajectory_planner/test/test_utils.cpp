// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
#include "kal_trajectory_planner/internal/utils.hpp"

#include <chrono>
#include <cmath>
#include <iterator>
#include <memory>
#include <stdexcept>

#include <Eigen/Geometry>
#include <glog/logging.h>
#include "gtest/gtest.h"

#include "kal_trajectory_planner/types.hpp"

namespace kal_trajectory_planner::utils {

TEST(UtilsTest, findClosestPointOnPath) {
    // Mock path
    Path path;
    for (int i = 0; i < 3; i++) {
        path.push_back(Position(i, 0));
    }

    // Position close to path
    Position positionCloseToPath(1, 0.5);

    auto pathIterator = findClosestPointOnPath(path, positionCloseToPath);

    EXPECT_TRUE(pathIterator->isApprox(path[1]));


    // Position in front of path
    Position positionInFrontOfPath(-100, 20);

    pathIterator = findClosestPointOnPath(path, positionInFrontOfPath);

    EXPECT_TRUE(pathIterator->isApprox(path[0]));
}

TEST(UtilsTest, pathSection) {
    // Mock path
    Path path;
    for (int i = 0; i < 10; i++) {
        path.push_back(Position(i, 0));
    }

    // Default use case. Start somewhere on the path and return a section smaller than what's left on the path.
    Path section;
    auto pathIterator = path.begin();
    std::advance(pathIterator, 1);
    double distance = pathSection(section, path, pathIterator, 2);

    EXPECT_EQ(section.size(), 3);
    EXPECT_DOUBLE_EQ(distance, 2);
    EXPECT_DOUBLE_EQ(section.front().x(), 1);

    // Corner case. Start somewhere on the path but the desired length is longer than what's left on the path.
    // In this case, the returned path is shorter than we set it to be.
    pathIterator = path.begin();
    std::advance(pathIterator, 2);
    distance = pathSection(section, path, pathIterator, 10);

    EXPECT_EQ(section.size(), 8);
    EXPECT_DOUBLE_EQ(distance, 7);
    EXPECT_DOUBLE_EQ(section.front().x(), 2);

    // Corner case. Start at the end of path.
    // In this case, the section should only contain that last point.
    pathIterator = path.end();
    std::advance(pathIterator, -1);
    distance = pathSection(section, path, pathIterator, 10);

    EXPECT_EQ(section.size(), 1);
    EXPECT_DOUBLE_EQ(distance, 0);

    // Error case. When the beginning of the section is not actually on the path (path.end() returns the past-the-end
    // element!), we should receive an error.
    pathIterator = path.end();
    EXPECT_THROW(pathSection(section, path, pathIterator, 10), std::out_of_range);
}

TEST(UtilsTest, transformPath) {
    Path path1;
    path1.emplace_back(0, 0);
    path1.emplace_back(1, 2);
    path1.emplace_back(-2, 4);

    // Pure rotation
    Eigen::Translation2d noTranslation(0, 0);
    Eigen::Rotation2Dd rotation(M_PI_2);
    Eigen::Isometry2d transformationPureRotation = noTranslation * rotation;
    transformPath(path1, transformationPureRotation);
    EXPECT_NEAR((path1[0] - Eigen::Vector2d(0, 0)).norm(), 0., 1e-10);
    EXPECT_NEAR((path1[1] - Eigen::Vector2d(-2, 1)).norm(), 0., 1e-10);
    EXPECT_NEAR((path1[2] - Eigen::Vector2d(-4, -2)).norm(), 0., 1e-10);

    // Pure translation
    Eigen::Translation2d translation(1, -1);
    Eigen::Rotation2Dd noRotation(0);
    Eigen::Isometry2d transformationPureTranslation = translation * noRotation;
    transformPath(path1, transformationPureTranslation);
    EXPECT_NEAR((path1[0] - Eigen::Vector2d(1, -1)).norm(), 0., 1e-10);
    EXPECT_NEAR((path1[1] - Eigen::Vector2d(-1, 0)).norm(), 0., 1e-10);
    EXPECT_NEAR((path1[2] - Eigen::Vector2d(-3, -3)).norm(), 0., 1e-10);

    // Combined translation and rotation should deliver the same result as a separate rotation + translation
    Path path2;
    path2.emplace_back(0, 0);
    path2.emplace_back(1, 2);
    path2.emplace_back(-2, 4);
    Eigen::Isometry2d transformationTranslationAndRotation = translation * rotation;
    transformPath(path2, transformationTranslationAndRotation);
    EXPECT_NEAR((path2[0] - path1[0]).norm(), 0., 1e-10);
    EXPECT_NEAR((path2[1] - path1[1]).norm(), 0., 1e-10);
    EXPECT_NEAR((path2[2] - path1[2]).norm(), 0., 1e-10);
}

TEST(UtilsTest, leastSquareFitPolynomialToPoints) {
    std::vector<Eigen::Vector2d> observations;

    // We can't estimate a first degree polyonmial based on one observation
    observations.emplace_back(1, 1);
    EXPECT_THROW(leastSquareFitPolynomialToPoints(observations, 1), std::invalid_argument);

    // Estimate exact zeroth deegree polynomial a.k.a. a scalar
    Eigen::VectorXd polynomialParameters0DegreeExact = leastSquareFitPolynomialToPoints(observations, 0);
    EXPECT_EQ(polynomialParameters0DegreeExact.rows(), 1);
    EXPECT_DOUBLE_EQ(polynomialParameters0DegreeExact(0), 1);

    // Estimate best fit zeroth degree polynomial using 2 obervations
    observations.emplace_back(0, 0);
    Eigen::VectorXd polynomialParameters0DegreeBestFit = leastSquareFitPolynomialToPoints(observations, 0);
    EXPECT_EQ(polynomialParameters0DegreeBestFit.rows(), 1);
    EXPECT_DOUBLE_EQ(polynomialParameters0DegreeBestFit(0), 0.5);

    // Estimate exact first deegree polynomial a.k.a. a line
    Eigen::VectorXd polynomialParameters1stDegree = leastSquareFitPolynomialToPoints(observations, 1);
    EXPECT_EQ(polynomialParameters1stDegree.rows(), 2);
    EXPECT_DOUBLE_EQ(polynomialParameters1stDegree(0), 0);
    EXPECT_DOUBLE_EQ(polynomialParameters1stDegree(1), 1);

    // Create a larger observation vector and fit a 3rd degree polynomial
    observations.clear();
    for (double x = -10; x < 10; x += 0.1) {
        observations.emplace_back(x, x * x * x);
    }
    Eigen::VectorXd polynomialParameters3rdDegree = leastSquareFitPolynomialToPoints(observations, 3);
    EXPECT_EQ(polynomialParameters3rdDegree.rows(), 4);
}


TEST(UtilsTest, pathFromPolynomial) {
    Path path;

    // Define a line
    Eigen::VectorXd polynomialParameters(2);
    polynomialParameters(0) = 0;
    polynomialParameters(1) = 1;
    double xFront = 0;
    double xBack = 10;
    uint16_t numOfPoses = 100;
    pathFromPolynomial(path, polynomialParameters, std::make_tuple(xFront, xBack), numOfPoses);

    EXPECT_EQ(path.size(), numOfPoses);
    for (int i = 0; i < 10; i++) {
        EXPECT_DOUBLE_EQ(path[i].x(), i * 0.1);
        EXPECT_DOUBLE_EQ(path[i].y(), i * 0.1);
    }
}


int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

} // namespace kal_trajectory_planner::utils
