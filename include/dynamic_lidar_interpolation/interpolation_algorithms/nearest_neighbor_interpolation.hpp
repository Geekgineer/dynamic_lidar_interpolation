#pragma once

/**
 * @file nearest_neighbor_interpolation.hpp
 * @brief Header file for the nearest neighbor interpolation function.
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
 * @brief Performs nearest neighbor interpolation on a 2D Eigen row-major matrix.
 *
 * @param input The input matrix (Row-major).
 * @param scaleFactorX The scaling factor for horizontal interpolation (X-axis).
 * @param scaleFactorY The scaling factor for vertical interpolation (Y-axis).
 * @param extrapolationValue The value to assign for out-of-bounds or invalid points.
 * @return Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> The interpolated matrix.
 */
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> nearestNeighborInterpolation(
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &input,
    double scaleFactorX,
    double scaleFactorY,
    double extrapolationValue)
{
    const int originalRows = input.rows();
    const int originalCols = input.cols();

    // Calculate new dimensions based on scaling factors
    const int newRows = static_cast<int>(std::ceil(originalRows * scaleFactorY));
    const int newCols = static_cast<int>(std::ceil(originalCols * scaleFactorX));

    // Handle edge cases where newRows or newCols might be 0
    if (newRows == 0 || newCols == 0)
    {
        return Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(); // Return an empty matrix
    }

    // Initialize output matrix with extrapolation values
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> output =
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Constant(newRows, newCols, extrapolationValue);

    // Calculate separate scaling ratios for rows and columns
    const double rowScale = (newRows > 0) ? static_cast<double>(originalRows) / newRows : 1.0;
    const double colScale = (newCols > 0) ? static_cast<double>(originalCols) / newCols : 1.0;

    // Precompute the nearest row indices
    std::vector<int> nearestRows(newRows);
    for (int i = 0; i < newRows; ++i)
    {
        double mappedRow = i * rowScale;
        int nearestRow = static_cast<int>(std::round(mappedRow));
        nearestRow = std::max(0, std::min(nearestRow, originalRows - 1));
        nearestRows[i] = nearestRow;
    }

    // Precompute the nearest column indices
    std::vector<int> nearestCols(newCols);
    for (int j = 0; j < newCols; ++j)
    {
        double mappedCol = j * colScale;
        int nearestCol = static_cast<int>(std::round(mappedCol));
        nearestCol = std::max(0, std::min(nearestCol, originalCols - 1));
        nearestCols[j] = nearestCol;
    }

    // Access input and output data via raw pointers for faster access
    const double *inputData = input.data();
    double *outputData = output.data();

    // Precompute original matrix stride (number of columns for row-major)
    const int originalStride = originalCols;

    // Perform the interpolation
    for (int i = 0; i < newRows; ++i)
    {
        int nearestRow = nearestRows[i];
        const double *inputRowPtr = inputData + nearestRow * originalStride;

        for (int j = 0; j < newCols; ++j)
        {
            int nearestCol = nearestCols[j];
            double value = inputRowPtr[nearestCol];

            // Assign the value or extrapolationValue if NaN
            // Using a ternary operator to minimize branching
            outputData[i * newCols + j] = std::isnan(value) ? extrapolationValue : value;
        }
    }

    return output;
}