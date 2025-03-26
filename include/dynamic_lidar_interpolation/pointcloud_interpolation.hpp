/**
 * @file point_cloud_interpolation.hpp
 * @brief Header file for the PointCloudInterpolator class which provides functionality to 
 *        interpolate point clouds.
 * 
 * @author Abdalrahman M. Amer
 * @linkedin https://www.linkedin.com/in/abdalrahman-m-amer
 * @github https://github.com/geekgineer
 * 
 * @license AGPL-3.0
 * 
 * This file is part of a project that is licensed under the Affero General Public License 
 * (AGPL) version 3 or later. You should have received a copy of the AGPL license along with 
 * this program. If not, see <https://www.gnu.org/licenses/>.
 * 
 * The software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the AGPL license for more details.
 */

#pragma once

#include <Eigen/Dense>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image_spherical.h>

#include <rclcpp/rclcpp.hpp>

#include "interpolation_algorithms/bilinear_interpolation.hpp"
#include "interpolation_algorithms/spline_interpolation.hpp"
#include "interpolation_algorithms/bilateral_interpolation.hpp"
#include "interpolation_algorithms/edge_aware_interpolation.hpp"
#include "interpolation_algorithms/nearest_neighbor_interpolation.hpp"


namespace pointcloud_interpolation
{
    // Define a constant for degree to radian conversion
    constexpr double DEG2RAD = M_PI / 180.0;

    /**
     * @brief Utility class for creating range images and interpolating point clouds.
     */
    class PointCloudInterpolator
    {
    public:
        PointCloudInterpolator() = default;

        pcl::RangeImage::CoordinateFrame coordinateFrame = pcl::RangeImage::LASER_FRAME;

        /**
         * @brief Interpolates a point cloud to create a denser representation.
         *
         * @param inputCloud The input point cloud.
         * @param angularResX Angular resolution in the X direction (degrees).
         * @param angularResY Angular resolution in the Y direction (degrees).
         * @param maxAngleWidth Maximum angle width (degrees).
         * @param maxAngleHeight Maximum angle height (degrees).
         * @param interpolationMethod Interpolation method ("nearest" or "linear"; default is "linear").
         * @param scaleFactor Scale factor for interpolation.
         * @param minRange Minimum range for filtering.
         * @param maxRange Maximum range for filtering.
         * @param applyVarianceFilter Whether to apply variance filtering.
         * @param maxAllowedVariance Maximum allowed variance for filtering.
         * @param sensorTranslation Translation vector for sensor pose (default is zero vector).
         * @param rotationAngleX Rotation angle around X-axis for transformation in radians (default is 0.0).
         * @param extrapolationValue Value to assign for points outside the interpolation grid (default is NaN).
         * @return pcl::PointCloud<pcl::PointXYZ>::Ptr The interpolated dense point cloud.
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr interpolatePointCloud(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud,
            double angularResX,
            double angularResY,
            double maxAngleWidth,
            double maxAngleHeight,
            const std::string &interpolationMethod = "linear",
            double scaleFactorX = 1.0,
            double scaleFactorY = 2.0,
            double minRange = 0.0,
            double maxRange = std::numeric_limits<double>::max(),
            bool applyVarianceFilter = false,
            double maxAllowedVariance = 50.0,
            const Eigen::Vector3f &sensorTranslation = Eigen::Vector3f::Zero(),
            double rotationAngleX = 0.0,
            double extrapolationValue = std::numeric_limits<double>::quiet_NaN(),
            double min_ang_FOV = 0.0,
            double max_ang_FOV = 360.0)
        {
            // Step 1: Create Range Image
            pcl::RangeImageSpherical rangeImageSpherical;
            Eigen::Affine3f sensorPose = Eigen::Affine3f::Identity();
            sensorPose.translation() = sensorTranslation;

            rangeImageSpherical.createFromPointCloud(
                *inputCloud,
                pcl::deg2rad(static_cast<float>(angularResX)),
                pcl::deg2rad(static_cast<float>(angularResY)),
                pcl::deg2rad(static_cast<float>(maxAngleWidth)),
                pcl::deg2rad(static_cast<float>(maxAngleHeight)),
                sensorPose,
                coordinateFrame,
                0.0f, 0.0f, 0);

            const int imageCols = rangeImageSpherical.width;
            const int imageRows = rangeImageSpherical.height;

            // Step 2: Initialize Matrices for Range and Height Data
            Eigen::MatrixXd rangeMatrix = Eigen::MatrixXd::Constant(imageRows, imageCols, extrapolationValue);
            Eigen::MatrixXd heightMatrix = Eigen::MatrixXd::Constant(imageRows, imageCols, extrapolationValue);

            // Populate range and height matrices
            for (int col = 0; col < imageCols; ++col)
            {
                for (int row = 0; row < imageRows; ++row)
                {
                    pcl::PointWithRange point = rangeImageSpherical.getPoint(col, row);
                    if (!std::isinf(point.range) && point.range >= minRange && point.range <= maxRange)
                    {
                        rangeMatrix(row, col) = static_cast<double>(point.range);
                        heightMatrix(row, col) = static_cast<double>(point.z); // Assuming 'z' represents height
                    }
                }
            }

            // Step 3: Perform 2D Interpolation
            Eigen::MatrixXd interpolatedRange;
            Eigen::MatrixXd interpolatedHeight;

            if (interpolationMethod == "linear")
            {
                interpolatedRange = bilinearInterpolation(rangeMatrix, scaleFactorX, scaleFactorY, extrapolationValue);
                interpolatedHeight = bilinearInterpolation(heightMatrix, scaleFactorX, scaleFactorY, extrapolationValue);
            }
            else if (interpolationMethod == "nearest")
            {
                interpolatedRange = nearestNeighborInterpolation(rangeMatrix, scaleFactorX, scaleFactorY, extrapolationValue);
                interpolatedHeight = nearestNeighborInterpolation(heightMatrix, scaleFactorX, scaleFactorY, extrapolationValue);
            }
            else if (interpolationMethod == "bilateral")
            {
                interpolatedRange = bilateralInterpolation(rangeMatrix, scaleFactorX, scaleFactorY, extrapolationValue);
                interpolatedHeight = bilateralInterpolation(heightMatrix, scaleFactorX, scaleFactorY, extrapolationValue);
            }
            else if (interpolationMethod == "edgeAware")
            {
                interpolatedRange = edgeAwareInterpolation(rangeMatrix, scaleFactorX, scaleFactorY, extrapolationValue);
                interpolatedHeight = edgeAwareInterpolation(heightMatrix, scaleFactorX, scaleFactorY, extrapolationValue);
            }
            else if (interpolationMethod == "spline")
            {
                interpolatedRange = splineInterpolation(rangeMatrix, scaleFactorX, scaleFactorY, extrapolationValue);
                interpolatedHeight = splineInterpolation(heightMatrix, scaleFactorX, scaleFactorY, extrapolationValue);
            }
            else
            {
                // Default to linear interpolation if method is unknown
                interpolatedRange = bilinearInterpolation(rangeMatrix, scaleFactorX, scaleFactorY, extrapolationValue);
                interpolatedHeight = bilinearInterpolation(heightMatrix, scaleFactorX, scaleFactorY, extrapolationValue);
            }

            // Step 4: Post-Processing Interpolated Data
            if (applyVarianceFilter)
            {
                smoothIsolatedPoints(interpolatedRange, interpolatedHeight, extrapolationValue);
                applyVarianceFiltering(interpolatedRange, interpolatedHeight, maxAllowedVariance, extrapolationValue);
            }

            // Step 5: Reconstruct 3D Point Cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr densePointCloud(new pcl::PointCloud<pcl::PointXYZ>());
            densePointCloud->reserve(static_cast<size_t>(interpolatedRange.size()));

            const double azimuthFactor = (2.0 * M_PI) / static_cast<double>(interpolatedRange.cols() - 1);
            const double elevationFactor = M_PI / static_cast<double>(interpolatedRange.rows() - 1);

            for (int row = 0; row < interpolatedRange.rows(); ++row)
            {
                double elevation = M_PI_2 - (M_PI * row) / static_cast<double>(interpolatedRange.rows() - 1);

                for (int col = 0; col < interpolatedRange.cols(); ++col)
                {
                    // Calculate azimuth angle
                    double azimuth = M_PI - (azimuthFactor * col);

                    // Normalize azimuth to [0, 2*PI)
                    if (azimuth < 0.0)
                        azimuth += 2.0 * M_PI;
                    else if (azimuth >= 2.0 * M_PI)
                        azimuth -= 2.0 * M_PI;

                    // Convert FOV boundaries from degrees to radians
                    double min_ang_rad = min_ang_FOV * DEG2RAD;
                    double max_ang_rad = max_ang_FOV * DEG2RAD;

                    // Check if the FOV wraps around 0 radians
                    bool wraps_around = min_ang_rad > max_ang_rad;

                    // Perform the FOV check
                    bool within_FOV = false;
                    if (!wraps_around)
                    {
                        // Simple case: FOV does not wrap around 0 radians
                        if (azimuth >= min_ang_rad && azimuth <= max_ang_rad)
                            within_FOV = true;
                    }
                    else
                    {
                        // Complex case: FOV wraps around 0 radians
                        if (azimuth >= min_ang_rad || azimuth <= max_ang_rad)
                            within_FOV = true;
                    }

                    if (!within_FOV)
                        continue;

                    double range = interpolatedRange(row, col);
                    double height = interpolatedHeight(row, col);

                    if (std::isnan(range) || range <= 0.0)
                        continue;

                    // Calculate horizontal range component
                    double horizontalRange = (range > height) ? std::sqrt(range * range - height * height) : 0.0;

                    // Compute 3D coordinates
                    pcl::PointXYZ point;
                    point.x = horizontalRange * std::cos(azimuth);
                    point.y = horizontalRange * std::sin(azimuth);
                    point.z = height;

                    densePointCloud->points.emplace_back(point);
                }
            }
            densePointCloud->width = interpolatedRange.cols();
            densePointCloud->height = interpolatedRange.rows();
            densePointCloud->is_dense = false;

            // Step 6: Apply Rotation Transformation if Needed
            if (std::abs(rotationAngleX) > 1e-6)
            {
                Eigen::Affine3f rotationTransform = Eigen::Affine3f::Identity();
                rotationTransform.rotate(Eigen::AngleAxisf(static_cast<float>(rotationAngleX), Eigen::Vector3f::UnitX()));
                pcl::transformPointCloud(*densePointCloud, *densePointCloud, rotationTransform);
            }

            return densePointCloud;
        }

    private:
 
        /**
         * @brief Smooths isolated points in the interpolated matrices by zeroing them out.
         *
         * @param rangeMatrix Reference to the interpolated range matrix.
         * @param heightMatrix Reference to the interpolated height matrix.
         * @param extrapolationValue The value used for extrapolation.
         */
        void smoothIsolatedPoints(Eigen::MatrixXd &rangeMatrix, Eigen::MatrixXd &heightMatrix, double extrapolationValue)
        {
            const int rows = rangeMatrix.rows();
            const int cols = rangeMatrix.cols();

            // Temporary matrices to store updated values
            Eigen::MatrixXd rangeTemp = rangeMatrix;
            Eigen::MatrixXd heightTemp = heightMatrix;

            // Iterate through each cell, excluding the border
            for (int i = 1; i < rows - 1; ++i)
            {
                for (int j = 1; j < cols - 1; ++j)
                {
                    double currentRange = rangeMatrix(i, j);
                    if (std::isnan(currentRange) || currentRange == extrapolationValue || currentRange == 0.0)
                    {
                        bool hasValidNeighbor = false;

                        // Check 8-connected neighbors
                        for (int di = -1; di <= 1 && !hasValidNeighbor; ++di)
                        {
                            for (int dj = -1; dj <= 1 && !hasValidNeighbor; ++dj)
                            {
                                if (di == 0 && dj == 0)
                                    continue;

                                double neighborRange = rangeMatrix(i + di, j + dj);
                                if (!std::isnan(neighborRange) && neighborRange > 0.0 && neighborRange != extrapolationValue)
                                {
                                    hasValidNeighbor = true;
                                }
                            }
                        }

                        // Zero out if no valid neighbors
                        if (!hasValidNeighbor)
                        {
                            rangeTemp(i, j) = 0.0;
                            heightTemp(i, j) = 0.0;
                        }
                    }
                }
            }

            // Update original matrices
            rangeMatrix = rangeTemp;
            heightMatrix = heightTemp;
        }

        /**
         * @brief Applies variance filtering to the interpolated matrices.
         *
         * This function examines each element in the range and height matrices. For each element, it considers a 3x3 window
         * around it (handling edge boundaries). It computes the variance of valid values within this window. If the variance
         * exceeds the specified maximum, the corresponding elements in both matrices are set to the extrapolation value.
         *
         * @param rangeMatrix Reference to the interpolated range matrix.
         * @param heightMatrix Reference to the interpolated height matrix.
         * @param maxVariance The maximum allowed variance.
         * @param extrapolationValue The value to assign for elements exceeding the variance threshold.
         */
        void applyVarianceFiltering(Eigen::MatrixXd &rangeMatrix, Eigen::MatrixXd &heightMatrix, double maxVariance, double extrapolationValue)
        {
            // Ensure that both matrices have the same dimensions
            assert(rangeMatrix.rows() == heightMatrix.rows() && rangeMatrix.cols() == heightMatrix.cols());

            const int rows = rangeMatrix.rows();
            const int cols = rangeMatrix.cols();

            // Create copies to store updated values
            Eigen::MatrixXd rangeTemp = rangeMatrix;
            Eigen::MatrixXd heightTemp = heightMatrix;

            // Define the window size (3x3)
            const int windowSize = 3;
            const int halfWindow = windowSize / 2;

            for (int i = 0; i < rows; ++i)
            {
                for (int j = 0; j < cols; ++j)
                {
                    // Initialize fixed-size array for valid values
                    double validValues[9];
                    int validCount = 0;

                    // Define window boundaries, clamping to matrix edges
                    int rowStart = std::max(i - halfWindow, 0);
                    int rowEnd = std::min(i + halfWindow, rows - 1);
                    int colStart = std::max(j - halfWindow, 0);
                    int colEnd = std::min(j + halfWindow, cols - 1);

                    // Iterate through the window
                    for (int m = rowStart; m <= rowEnd && validCount < 9; ++m)
                    {
                        for (int n = colStart; n <= colEnd && validCount < 9; ++n)
                        {
                            double val = rangeMatrix(m, n);
                            if (!std::isnan(val) && val != 0.0 && val != extrapolationValue)
                            {
                                validValues[validCount++] = val;
                            }
                        }
                    }

                    // Calculate variance if enough valid values are present
                    if (validCount > 1)
                    {
                        double sum = 0.0;
                        double sumSq = 0.0;
                        for (int k = 0; k < validCount; ++k)
                        {
                            sum += validValues[k];
                            sumSq += validValues[k] * validValues[k];
                        }
                        double mean = sum / validCount;
                        double variance = (sumSq - (sum * sum) / validCount) / (validCount - 1);

                        // Assign extrapolationValue if variance exceeds the threshold
                        if (variance > maxVariance)
                        {
                            rangeTemp(i, j) = extrapolationValue;
                            heightTemp(i, j) = extrapolationValue;
                        }
                    }
                }
            }

            // Update the original matrices with the filtered values
            rangeMatrix = std::move(rangeTemp);
            heightMatrix = std::move(heightTemp);
        }
    
    };
}