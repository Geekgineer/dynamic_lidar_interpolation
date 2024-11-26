/**
 * @file bilateral_interpolation.hpp
 * @brief Header file for the bilateral interpolation function.
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
 * @brief Performs bilateral interpolation on a 2D Eigen row-major matrix.
 *
 * Bilateral interpolation considers both spatial proximity and intensity similarity
 * to preserve edges while resizing the matrix.
 *
 * @param input The input matrix (Row-major).
 * @param scaleFactorX The scaling factor for horizontal interpolation (X-axis).
 * @param scaleFactorY The scaling factor for vertical interpolation (Y-axis).
 * @param extrapolationValue The value to assign for out-of-bounds or invalid points.
 * @param spatialSigma The sigma value for spatial Gaussian weighting.
 * @param intensitySigma The sigma value for intensity Gaussian weighting.
 * @return Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> The interpolated matrix.
 */

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> bilateralInterpolation(
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &input,
    double scaleFactorX,
    double scaleFactorY,
    double extrapolationValue,
    double spatialSigma = 1.0,
    double intensitySigma = 1.0)
{

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
        rowLower = std::max(0, std::min(rowLower, originalRows - 2)); // Ensure rowLower +1 is valid
        rowMappings[i].lower = rowLower;
        rowMappings[i].upper = rowLower + 1;
        rowMappings[i].fraction = origRow - rowLower;
    }

    // Precompute mapping for columns
    std::vector<Mapping> colMappings(newCols);
    for (int j = 0; j < newCols; ++j)
    {
        double origCol = colScale * j;
        int colLower = static_cast<int>(std::floor(origCol));
        colLower = std::max(0, std::min(colLower, originalCols - 2)); // Ensure colLower +1 is valid
        colMappings[j].lower = colLower;
        colMappings[j].upper = colLower + 1;
        colMappings[j].fraction = origCol - colLower;
    }

    // Access input and output data via raw pointers for faster access
    const double *inputData = input.data();
    double *outputData = output.data();

    // Precompute original matrix stride (number of columns for row-major)
    const int originalStride = originalCols;

    // Precompute Gaussian coefficients for spatial and intensity
    const double spatialCoeff = -0.5 / (spatialSigma * spatialSigma);
    const double intensityCoeff = -0.5 / (intensitySigma * intensitySigma);

    // Perform bilateral interpolation
    for (int i = 0; i < newRows; ++i)
    {
        const Mapping &rowMap = rowMappings[i];
        int rowLower = rowMap.lower;
        int rowUpper = rowMap.upper;
        double rowFraction = rowMap.fraction;

        // Pointer to the lower and upper rows in the input matrix
        const double *inputRowLower = inputData + rowLower * originalStride;
        const double *inputRowUpper = inputData + rowUpper * originalStride;

        // Precompute output row offset
        const int outputRowOffset = i * newCols;

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

            // Reference intensity value
            // Option 1: Use the average of the four surrounding pixels
            double referenceValue = (val00 + val01 + val10 + val11) / 4.0;

            // Option 2: Use the bilinearly interpolated value as the reference
            // double referenceValue = (val00 * (1 - rowFraction) * (1 - colFraction) +
            //                          val01 * (1 - rowFraction) * colFraction +
            //                          val10 * rowFraction * (1 - colFraction) +
            //                          val11 * rowFraction * colFraction);

            // Compute spatial distances for the four pixels
            // (dr, dc) for each pixel relative to the mapped position
            double dr00 = rowFraction;
            double dc00 = colFraction;

            double dr01 = rowFraction;
            double dc01 = colFraction - 1.0;

            double dr10 = rowFraction - 1.0;
            double dc10 = colFraction;

            double dr11 = rowFraction - 1.0;
            double dc11 = colFraction - 1.0;

            // Compute bilateral weights
            auto computeWeight = [&](double val, double dr, double dc) -> double
            {
                double spatialDistSq = dr * dr + dc * dc;
                double intensityDiff = val - referenceValue;
                double intensityDistSq = intensityDiff * intensityDiff;
                double w_s = std::exp(spatialCoeff * spatialDistSq);
                double w_i = std::exp(intensityCoeff * intensityDistSq);
                return w_s * w_i;
            };

            double weight00 = computeWeight(val00, dr00, dc00);
            double weight01 = computeWeight(val01, dr01, dc01);
            double weight10 = computeWeight(val10, dr10, dc10);
            double weight11 = computeWeight(val11, dr11, dc11);

            // Optionally, handle invalid pixels (e.g., NaN, extrapolationValue)
            // Here, we assume all pixels are valid. Modify as needed.
            // Example:
            // weight00 *= (!std::isnan(val00) && val00 != extrapolationValue) ? 1.0 : 0.0;
            // Similarly for weight01, weight10, weight11

            // Compute the weighted sum
            double weightedSum = weight00 * val00 + weight01 * val01 + weight10 * val10 + weight11 * val11;
            double weightSum = weight00 + weight01 + weight10 + weight11;

            // Assign the interpolated value if weightSum is positive
            if (weightSum > 0.0)
            {
                outputData[outputRowOffset + j] = weightedSum / weightSum;
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