
/**
 * @file edge_aware_interpolation.hpp
 * @brief Header file for the edge-aware interpolation function.
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

/**
 * @brief Performs edge-aware interpolation on a 2D Eigen row-major matrix.
 *
 * Edge-aware interpolation preserves edges by adjusting interpolation weights based on
 * edge information derived from gradient magnitudes and directions.
 *
 * @param input The input matrix (Row-major).
 * @param scaleFactorX The scaling factor for horizontal interpolation (X-axis).
 * @param scaleFactorY The scaling factor for vertical interpolation (Y-axis).
 * @param extrapolationValue The value to assign for out-of-bounds or invalid points.
 * @param spatialSigma The sigma value for spatial Gaussian weighting.
 * @param intensitySigma The sigma value for intensity Gaussian weighting.
 * @return Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> The interpolated matrix.
 */
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> edgeAwareInterpolation(
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &input,
    double scaleFactorX,
    double scaleFactorY,
    double extrapolationValue,
    double spatialSigma = 1.0,
    double intensitySigma = 1.0)
{

    // Validate scaling factors
    if (scaleFactorX <= 0.0 || scaleFactorY <= 0.0)
    {
        throw std::invalid_argument("Scale factors must be positive.");
    }

    const int originalRows = input.rows();
    const int originalCols = input.cols();

    // Calculate new dimensions based on scaling factors
    const int newRows = static_cast<int>(std::ceil(originalRows * scaleFactorY));
    const int newCols = static_cast<int>(std::ceil(originalCols * scaleFactorX));

    // Handle edge cases where newRows or newCols might be 0 or negative
    if (newRows <= 0 || newCols <= 0)
    {
        return Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(); // Return an empty matrix
    }

    // Initialize output matrix with extrapolation values
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> output =
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Constant(newRows, newCols, extrapolationValue);

    // Calculate separate scaling ratios for rows and columns
    const double rowScale = (newRows > 1) ? static_cast<double>(originalRows - 1) / (newRows - 1) : 0.0;
    const double colScale = (newCols > 1) ? static_cast<double>(originalCols - 1) / (newCols - 1) : 0.0;

    // Step 1: Compute gradients using Sobel operator
    Eigen::MatrixXd gradX = Eigen::MatrixXd::Zero(originalRows, originalCols);
    Eigen::MatrixXd gradY = Eigen::MatrixXd::Zero(originalRows, originalCols);

    // Sobel kernels
    Eigen::Matrix3d sobelX;
    sobelX << -1, 0, 1,
        -2, 0, 2,
        -1, 0, 1;

    Eigen::Matrix3d sobelY;
    sobelY << -1, -2, -1,
        0, 0, 0,
        1, 2, 1;

    // Apply Sobel operator (ignore borders for simplicity)
    for (int i = 1; i < originalRows - 1; ++i)
    {
        for (int j = 1; j < originalCols - 1; ++j)
        {
            Eigen::Matrix3d window = input.block(i - 1, j - 1, 3, 3);
            double gx = (window.array() * sobelX.array()).sum();
            double gy = (window.array() * sobelY.array()).sum();
            gradX(i, j) = gx;
            gradY(i, j) = gy;
        }
    }

    // Compute gradient magnitude and direction
    Eigen::MatrixXd gradMag = (gradX.array().square() + gradY.array().square()).sqrt();
    Eigen::MatrixXd gradDir(originalRows, originalCols); // In radians

    // Compute gradient directions with correct averaging
    for (int i = 0; i < originalRows; ++i)
    {
        for (int j = 0; j < originalCols; ++j)
        {
            gradDir(i, j) = std::atan2(gradY(i, j), gradX(i, j));
        }
    }

    // Precompute mapping for rows
    struct Mapping
    {
        int lower;
        int upper;
        double fraction;
    };

    std::vector<Mapping> rowMappings(newRows);
    for (int i = 0; i < newRows; ++i)
    {
        double origRow = rowScale * i;
        int rowLower = static_cast<int>(std::floor(origRow));
        rowLower = std::max(0, std::min(rowLower, originalRows - 2));
        rowMappings[i].lower = rowLower;
        rowMappings[i].upper = std::min(rowLower + 1, originalRows - 1);
        rowMappings[i].fraction = origRow - rowLower;
    }

    // Precompute mapping for columns
    std::vector<Mapping> colMappings(newCols);
    for (int j = 0; j < newCols; ++j)
    {
        double origCol = colScale * j;
        int colLower = static_cast<int>(std::floor(origCol));
        colLower = std::max(0, std::min(colLower, originalCols - 2));
        colMappings[j].lower = colLower;
        colMappings[j].upper = std::min(colLower + 1, originalCols - 1);
        colMappings[j].fraction = origCol - colLower;
    }

    // Access input and output data via raw pointers for faster access
    const double *inputData = input.data();
    const double *gradMagData = gradMag.data();
    const double *gradDirData = gradDir.data();
    double *outputData = output.data();

    // Precompute original matrix stride (number of columns for row-major)
    const int originalStride = originalCols;

    // Precompute Gaussian coefficients for spatial and intensity
    double spatialCoeff = -0.5 / (spatialSigma * spatialSigma);
    double intensityCoeff = -0.5 / (intensitySigma * intensitySigma);

    // Perform edge-aware interpolation
    for (int i = 0; i < newRows; ++i)
    {
        const Mapping &rowMap = rowMappings[i];
        int rowLower = rowMap.lower;
        int rowUpper = rowMap.upper;
        double rowFraction = rowMap.fraction;

        // Pointer to the lower and upper rows in the input matrix
        const double *inputRowLower = inputData + rowLower * originalStride;
        const double *inputRowUpper = inputData + rowUpper * originalStride;

        const double *gradMagRowLower = gradMagData + rowLower * originalStride;
        const double *gradMagRowUpper = gradMagData + rowUpper * originalStride;

        const double *gradDirRowLower = gradDirData + rowLower * originalStride;
        const double *gradDirRowUpper = gradDirData + rowUpper * originalStride;

        // Precompute output row offset
        int outputRowOffset = i * newCols;

        for (int j = 0; j < newCols; ++j)
        {
            const Mapping &colMap = colMappings[j];
            int colLower = colMap.lower;
            int colUpper = colMap.upper;
            double colFraction = colMap.fraction;

            // Fetch the four surrounding input values
            double val00 = inputRowLower[colLower];
            double val01 = inputRowLower[colUpper];
            double val10 = inputRowUpper[colLower];
            double val11 = inputRowUpper[colUpper];

            // Fetch gradient magnitudes and directions
            double gradMag00 = gradMagRowLower[colLower];
            double gradMag01 = gradMagRowLower[colUpper];
            double gradMag10 = gradMagRowUpper[colLower];
            double gradMag11 = gradMagRowUpper[colUpper];

            double gradDir00 = gradDirRowLower[colLower];
            double gradDir01 = gradDirRowLower[colUpper];
            double gradDir10 = gradDirRowUpper[colLower];
            double gradDir11 = gradDirRowUpper[colUpper];

            // Compute the average gradient direction using vector averaging
            double sumSin = std::sin(gradDir00) + std::sin(gradDir01) + std::sin(gradDir10) + std::sin(gradDir11);
            double sumCos = std::cos(gradDir00) + std::cos(gradDir01) + std::cos(gradDir10) + std::cos(gradDir11);
            double refGradDir = std::atan2(sumSin, sumCos);

            // Compute directional weights based on alignment with reference gradient
            auto computeDirectionalWeight = [&](double gradDir, double refDir) -> double
            {
                // Compute the smallest difference between two angles
                double diff = std::abs(gradDir - refDir);
                // Wrap around differences greater than pi
                if (diff > M_PI)
                    diff = 2.0 * M_PI - diff;
                // Compute weight using cosine similarity
                return std::cos(diff);
            };

            double w00 = computeDirectionalWeight(gradDir00, refGradDir);
            double w01 = computeDirectionalWeight(gradDir01, refGradDir);
            double w10 = computeDirectionalWeight(gradDir10, refGradDir);
            double w11 = computeDirectionalWeight(gradDir11, refGradDir);

            // Normalize directional weights to [0,1]
            w00 = (w00 + 1.0) / 2.0;
            w01 = (w01 + 1.0) / 2.0;
            w10 = (w10 + 1.0) / 2.0;
            w11 = (w11 + 1.0) / 2.0;

            // Reference intensity value (using bilinearly interpolated value)
            double referenceValue = (val00 * (1.0 - rowFraction) * (1.0 - colFraction) +
                                     val01 * (1.0 - rowFraction) * colFraction +
                                     val10 * rowFraction * (1.0 - colFraction) +
                                     val11 * rowFraction * colFraction);

            // Compute weights using both spatial and intensity differences
            auto computeWeight = [&](double val, double dr_local, double dc_local, double refValue) -> double
            {
                double spatialDistSq = dr_local * dr_local + dc_local * dc_local;
                double intensityDiff = val - refValue;
                double intensityDistSq = intensityDiff * intensityDiff;
                double w_s = std::exp(spatialCoeff * spatialDistSq);
                double w_i = std::exp(intensityCoeff * intensityDistSq);
                return w_s * w_i;
            };

            // Compute weights for the four pixels
            double weight00 = computeWeight(val00, -rowFraction, -colFraction, referenceValue) * w00;
            double weight01 = computeWeight(val01, -rowFraction, 1.0 - colFraction, referenceValue) * w01;
            double weight10 = computeWeight(val10, 1.0 - rowFraction, -colFraction, referenceValue) * w10;
            double weight11 = computeWeight(val11, 1.0 - rowFraction, 1.0 - colFraction, referenceValue) * w11;

            // Compute the interpolated value with edge-aware weights
            double interpolatedValue = (weight00 * val00) +
                                       (weight01 * val01) +
                                       (weight10 * val10) +
                                       (weight11 * val11);

            // Compute the sum of weights
            double weightSum = weight00 + weight01 + weight10 + weight11;

            // Assign the interpolated value if weightSum is positive
            if (weightSum > 0.0)
            {
                outputData[outputRowOffset + j] = interpolatedValue / weightSum;
            }
            else
            {
                // Assign extrapolationValue if no valid neighbors
                outputData[outputRowOffset + j] = extrapolationValue;
            }
        }
    }

    return output;
}