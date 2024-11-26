/**
 * @file spline_interpolation.hpp
 * @brief Header file for the spline interpolation function.
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
 * @brief Performs bicubic spline interpolation on a 2D Eigen row-major matrix.
 *
 * Bicubic interpolation uses cubic splines to interpolate the input matrix,
 * providing smoother results compared to bilinear interpolation.
 *
 * @param input The input matrix (Row-major).
 * @param scaleFactorX The scaling factor for horizontal interpolation (X-axis).
 * @param scaleFactorY The scaling factor for vertical interpolation (Y-axis).
 * @param extrapolationValue The value to assign for out-of-bounds or invalid points.
 * @return Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> The interpolated matrix.
 */

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> splineInterpolation(
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &input,
    double scaleFactorX,
    double scaleFactorY,
    double extrapolationValue)
{
    const int originalRows = input.rows();
    const int originalCols = input.cols();

    // Validate scaling factors
    if (scaleFactorX <= 0.0 || scaleFactorY <= 0.0)
    {
        throw std::invalid_argument("Scale factors must be positive.");
    }

    // Calculate new dimensions based on scaling factors
    const int newRows = static_cast<int>(std::ceil(originalRows * scaleFactorY));
    const int newCols = static_cast<int>(std::ceil(originalCols * scaleFactorX));

    // Handle edge cases where newRows or newCols might be 0
    if (newRows <= 0 || newCols <= 0)
    {
        return Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(); // Return an empty matrix
    }

    // Initialize output matrix
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> output(newRows, newCols);
    // No need to initialize; all elements will be set in the loop

    // Calculate scaling ratios, handling the case when newRows or newCols == 1 to avoid division by zero
    double rowScale = (newRows > 1) ? static_cast<double>(originalRows - 1) / (newRows - 1) : 0.0;
    double colScale = (newCols > 1) ? static_cast<double>(originalCols - 1) / (newCols - 1) : 0.0;

    // Access input and output data via raw pointers for faster access
    const double *inputData = input.data();
    double *outputData = output.data();

    // Precompute original matrix stride (number of columns for row-major)
    const int originalStride = originalCols;

    // Bicubic kernel function (Catmull-Rom spline)
    auto cubic = [](double x) -> double
    {
        x = std::abs(x);
        double a = -0.5;
        if (x <= 1.0)
        {
            // Using Horner's method for efficiency
            return ((a + 2.0) * x - (a + 3.0)) * x * x + 1.0;
        }
        else if (x < 2.0)
        {
            return (a * x - 5.0 * a) * x * x + (8.0 * a * x - 4.0 * a);
        }
        else
        {
            return 0.0;
        }
    };

    // Perform bicubic interpolation
    for (int i = 0; i < newRows; ++i)
    {
        // Map the row index to the original matrix
        double origRow = rowScale * i;
        int rowBase = static_cast<int>(std::floor(origRow));
        double rowFraction = origRow - rowBase;

        for (int j = 0; j < newCols; ++j)
        {
            // Map the column index to the original matrix
            double origCol = colScale * j;
            int colBase = static_cast<int>(std::floor(origCol));
            double colFraction = origCol - colBase;

            double interpolatedValue = 0.0;
            double weightSum = 0.0;

            // Iterate over the 4x4 neighborhood
            for (int m = -1; m <= 2; ++m)
            {
                int currentRow = rowBase + m;
                // Clamp the row index
                currentRow = std::max(0, std::min(currentRow, originalRows - 1));

                double wRow = cubic(m - rowFraction);

                for (int n = -1; n <= 2; ++n)
                {
                    int currentCol = colBase + n;
                    // Clamp the column index
                    currentCol = std::max(0, std::min(currentCol, originalCols - 1));

                    double wCol = cubic(n - colFraction);
                    double weight = wRow * wCol;

                    double pixelValue = input(currentRow, currentCol);

                    // Check for NaN
                    if (std::isnan(pixelValue))
                    {
                        pixelValue = extrapolationValue;
                    }

                    interpolatedValue += weight * pixelValue;
                    weightSum += weight;
                }
            }

            // Normalize by the sum of weights
            if (weightSum > 0.0)
            {
                outputData[i * newCols + j] = interpolatedValue / weightSum;
            }
            else
            {
                outputData[i * newCols + j] = extrapolationValue;
            }
        }
    }

    return output;
}