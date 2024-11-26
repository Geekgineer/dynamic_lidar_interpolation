/**
 * @file point_cloud_interpolation.hpp
 * @brief Header file for the PointCloudInterpolator class which provides functionality to 
 *        interpolate point clouds.
 * 
 * @author Abdalrahman M. Amer
 * @linkedin https://www.linkedin.com/in/abdalrahman-m-amer
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

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>

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
            densePointCloud->width = static_cast<uint32_t>(densePointCloud->points.size());
            densePointCloud->height = 1;
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
         * @brief Performs bilinear interpolation on a 2D Eigen matrix.
         *
         * @param input The input matrix.
         * @param scaleFactorX The scaling factor for Horizontal interpolation (X-axis).
         * @param scaleFactorY The scaling factor for Vertical interpolation (Y-axis).
         * @param extrapolationValue The value to assign for out-of-bounds points.
         * @return Eigen::MatrixXd The interpolated matrix.
         */
        Eigen::MatrixXd bilinearInterpolation(const Eigen::MatrixXd &input, double scaleFactorX, double scaleFactorY, double extrapolationValue)
        {
            const int originalRows = input.rows();
            const int originalCols = input.cols();

            // Calculate new dimensions based on different scaling factors
            const int newRows = static_cast<int>(std::ceil(originalRows * scaleFactorY));
            const int newCols = static_cast<int>(std::ceil(originalCols * scaleFactorX));

            // Initialize output matrix with extrapolation values
            Eigen::MatrixXd output = Eigen::MatrixXd::Constant(newRows, newCols, extrapolationValue);

            // Calculate separate scaling ratios for rows and columns
            const double rowScale = (newRows > 1) ? static_cast<double>(originalRows - 1) / (newRows - 1) : 0.0;
            const double colScale = (newCols > 1) ? static_cast<double>(originalCols - 1) / (newCols - 1) : 0.0;

            for (int i = 0; i < newRows; ++i)
            {
                double origRow = rowScale * i;
                int rowLower = static_cast<int>(std::floor(origRow));
                int rowUpper = std::min(rowLower + 1, originalRows - 1);
                double rowFraction = origRow - rowLower;

                for (int j = 0; j < newCols; ++j)
                {
                    double origCol = colScale * j;
                    int colLower = static_cast<int>(std::floor(origCol));
                    int colUpper = std::min(colLower + 1, originalCols - 1);
                    double colFraction = origCol - colLower;

                    double val00 = input(rowLower, colLower);
                    double val01 = input(rowLower, colUpper);
                    double val10 = input(rowUpper, colLower);
                    double val11 = input(rowUpper, colUpper);

                    // Check for invalid values
                    if (std::isnan(val00) || std::isnan(val01) || std::isnan(val10) || std::isnan(val11))
                    {
                        output(i, j) = extrapolationValue;
                        continue;
                    }

                    // Perform bilinear interpolation
                    double interpolatedValue = (1.0 - rowFraction) * (1.0 - colFraction) * val00 +
                                               (1.0 - rowFraction) * colFraction * val01 +
                                               rowFraction * (1.0 - colFraction) * val10 +
                                               rowFraction * colFraction * val11;

                    output(i, j) = interpolatedValue;
                }
            }

            return output;
        }
        /**
         * @brief Performs nearest neighbor interpolation on a 2D Eigen matrix.
         *
         * @param input The input matrix.
         * @param scaleFactorX The scaling factor for horizontal interpolation (X-axis).
         * @param scaleFactorY The scaling factor for vertical interpolation (Y-axis).
         * @param extrapolationValue The value to use for extrapolation (out of bounds).
         * @return Eigen::MatrixXd The interpolated matrix.
         */
        Eigen::MatrixXd nearestNeighborInterpolation(const Eigen::MatrixXd &input, double scaleFactorX, double scaleFactorY, double extrapolationValue)
        {
            const int originalRows = input.rows();
            const int originalCols = input.cols();

            // Calculate new dimensions based on separate scaling factors
            const int newRows = static_cast<int>(std::ceil(originalRows * scaleFactorY));
            const int newCols = static_cast<int>(std::ceil(originalCols * scaleFactorX));

            // Initialize output matrix with extrapolation values
            Eigen::MatrixXd output = Eigen::MatrixXd::Constant(newRows, newCols, extrapolationValue);

            // Calculate separate scaling ratios for rows and columns
            const double rowScale = (newRows > 0) ? static_cast<double>(originalRows) / newRows : 1.0;
            const double colScale = (newCols > 0) ? static_cast<double>(originalCols) / newCols : 1.0;

            for (int i = 0; i < newRows; ++i)
            {
                // Determine the nearest original row
                int nearestRow = std::min(static_cast<int>(std::round(i * rowScale)), originalRows - 1);

                for (int j = 0; j < newCols; ++j)
                {
                    // Determine the nearest original column
                    int nearestCol = std::min(static_cast<int>(std::round(j * colScale)), originalCols - 1);

                    // Assign the nearest value from the input matrix
                    double value = input(nearestRow, nearestCol);

                    // Check for invalid or NaN values and assign the extrapolation value if necessary
                    if (std::isnan(value))
                    {
                        output(i, j) = extrapolationValue;
                    }
                    else
                    {
                        output(i, j) = value;
                    }
                }
            }

            return output;
        }

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
         * @param rangeMatrix Reference to the interpolated range matrix.
         * @param heightMatrix Reference to the interpolated height matrix.
         * @param maxVariance The maximum allowed variance.
         */
        void applyVarianceFiltering(Eigen::MatrixXd &rangeMatrix, Eigen::MatrixXd &heightMatrix, double maxVariance, double extrapolationValue)
        {
            const int rows = rangeMatrix.rows();
            const int cols = rangeMatrix.cols();

            // Temporary matrices to store updated values
            Eigen::MatrixXd rangeTemp = rangeMatrix;
            Eigen::MatrixXd heightTemp = heightMatrix;

            for (int i = 0; i < rows; ++i)
            {
                for (int j = 0; j < cols; ++j)
                {
                    // Define window boundaries (3x3 window)
                    int rowStart = std::max(i - 1, 0);
                    int rowEnd = std::min(i + 1, rows - 1);
                    int colStart = std::max(j - 1, 0);
                    int colEnd = std::min(j + 1, cols - 1);

                    // Extract the window
                    Eigen::MatrixXd window = rangeMatrix.block(rowStart, colStart, rowEnd - rowStart + 1, colEnd - colStart + 1);

                    // Collect valid values from the window
                    std::vector<double> validValues;
                    validValues.reserve(9); // Maximum possible

                    for (int m = 0; m < window.rows(); ++m)
                    {
                        for (int n = 0; n < window.cols(); ++n)
                        {
                            double val = window(m, n);
                            if (!std::isnan(val) && val != 0.0 && val != extrapolationValue)
                            {
                                validValues.push_back(val);
                            }
                        }
                    }

                    // Calculate variance if enough valid values are present
                    if (validValues.size() > 1)
                    {
                        double mean = std::accumulate(validValues.begin(), validValues.end(), 0.0) / validValues.size();
                        double variance = 0.0;
                        for (const auto &val : validValues)
                        {
                            variance += (val - mean) * (val - mean);
                        }
                        variance /= static_cast<double>(validValues.size() - 1);

                        // Zero out if variance exceeds the threshold
                        if (variance > maxVariance)
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
    };
}