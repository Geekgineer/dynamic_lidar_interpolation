/**
 * @file bilinear_interpolation.hpp
 * @brief Header file for the bilinear interpolation function.
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
 * @brief Performs bilinear interpolation on a 2D Eigen row-major matrix.
 *
 * @param input The input matrix (Row-major).
 * @param scaleFactorX The scaling factor for horizontal interpolation (X-axis).
 * @param scaleFactorY The scaling factor for vertical interpolation (Y-axis).
 * @param extrapolationValue The value to assign for out-of-bounds or invalid points.
 * @return Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> The interpolated matrix.
 */
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> bilinearInterpolation(
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

    // Initialize output matrix without setting all elements to extrapolationValue
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> output(newRows, newCols);
    // No need to initialize; all elements will be set in the loop

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
        rowLower = std::max(0, std::min(rowLower, originalRows - 1));
        if (rowLower >= originalRows - 1)
        {
            rowMappings[i].lower = originalRows - 1;
            rowMappings[i].upper = originalRows - 1;
            rowMappings[i].fraction = 0.0;
        }
        else
        {
            rowMappings[i].lower = rowLower;
            rowMappings[i].upper = rowLower + 1;
            rowMappings[i].fraction = origRow - rowLower;
        }
    }

    // Precompute mapping for columns
    std::vector<Mapping> colMappings(newCols);
    for (int j = 0; j < newCols; ++j)
    {
        double origCol = colScale * j;
        int colLower = static_cast<int>(std::floor(origCol));
        colLower = std::max(0, std::min(colLower, originalCols - 1));
        if (colLower >= originalCols - 1)
        {
            colMappings[j].lower = originalCols - 1;
            colMappings[j].upper = originalCols - 1;
            colMappings[j].fraction = 0.0;
        }
        else
        {
            colMappings[j].lower = colLower;
            colMappings[j].upper = colLower + 1;
            colMappings[j].fraction = origCol - colLower;
        }
    }

    // Access input and output data via raw pointers for faster access
    const double *inputData = input.data();
    double *outputData = output.data();

    // Precompute original matrix stride (number of columns for row-major)
    const int originalStride = originalCols;

    // Perform bilinear interpolation
    for (int i = 0; i < newRows; ++i)
    {
        const Mapping &rowMap = rowMappings[i];
        int rowLower = rowMap.lower;
        int rowUpper = rowMap.upper;
        double rowFraction = rowMap.fraction;

        // Pointers to the lower and upper rows in the input matrix
        const double *inputRowLower = inputData + rowLower * originalStride;
        const double *inputRowUpper = inputData + rowUpper * originalStride;

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

            // Check for invalid (NaN) values
            if (std::isnan(val00) || std::isnan(val01) || std::isnan(val10) || std::isnan(val11))
            {
                outputData[outputRowOffset + j] = extrapolationValue;
                continue;
            }

            // Perform bilinear interpolation
            double interpolatedValue = (1.0 - rowFraction) * (1.0 - colFraction) * val00 +
                                       (1.0 - rowFraction) * colFraction * val01 +
                                       rowFraction * (1.0 - colFraction) * val10 +
                                       rowFraction * colFraction * val11;

            outputData[outputRowOffset + j] = interpolatedValue;
        }
    }

    return output;
}