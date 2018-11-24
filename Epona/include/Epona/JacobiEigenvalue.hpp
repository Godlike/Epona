/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef EPONA_JACOBI_EIGENVALUE_HPP
#define EPONA_JACOBI_EIGENVALUE_HPP

#include <Epona/FloatingPoint.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/glm.hpp>
#include <tuple>

namespace
{
/**
 * @brief Finds maximal absolute value off diagonal matrix element and sets its indices
 *
 * @param[in] mat matrix to search
 * @param[out] i max element row index
 * @param[out] j max element column index
 */
inline std::tuple<uint8_t, uint8_t> FindMaxNormOffDiagonal(glm::mat3 mat)
{
    uint8_t i = 0;
    uint8_t j = 1;

    for (uint8_t p = 0; p < 3; ++p)
    {
        for (uint8_t q = 0; q < 3; ++q)
        {
            if (p != q)
            {
                if (glm::abs(mat[i][j]) < glm::abs(mat[p][q]))
                {
                    i = p;
                    j = q;
                }
            }
        }
    }

    return { i, j };
}

/**
    * @brief Calculates rotation angle for a given matrix and its element
    *
    * @param[in] mat matrix to rotate
    * @param[in] i element row index
    * @param[in] j element column index
    *
    * @return angle in radians
    */
inline float CalculateRotationAngle(glm::mat3 mat, uint8_t i, uint8_t j, float coverageThreshold)
{
    if (glm::abs(mat[i][i] - mat[j][j]) < coverageThreshold)
    {
        return (glm::pi<float>() / 4.0f) * (mat[i][j] > 0 ? 1.0f : -1.0f);
    }

    return 0.5f * glm::atan(2.0f * mat[i][j] / (mat[i][i] - mat[j][j]));
}

/**
    * @brief Makes Givens rotation matrix from the angle and indices
    *
    * @param[in] theta rotation angle in radians
    * @param[in] i row index
    * @param[in] j column index
    *
    * @return Givens rotation matrix
    */
inline glm::mat3 MakeGivensRotationMatrix(float theta, uint8_t i, uint8_t j)
{
    glm::mat3 g(1);

    g[i][i] = glm::cos(theta);
    g[i][j] = glm::sin(theta);
    g[j][i] = -glm::sin(theta);
    g[j][j] = glm::cos(theta);

    return g;
}
} // namespace ::

namespace epona
{

/**
 * @brief Calculates eigenvalues and eigenvectors
 *
 * @param[in] symmetricMatrix target symmetric matrix
 * @param[in] coverageThreshold coverage precision threshold
 * @param[in] maxIterations maximum amount of iteration
 */
inline std::tuple<glm::mat3, glm::vec3> CalculateJacobiEigenvectorsEigenvalue(
    glm::mat3 symmetricMatrix, float coverageThreshold = epona::fp::g_floatingPointThreshold, uint32_t maxIterations = 100
)
{
    glm::mat3 eigenvectors;
    glm::mat3 rotationMatrix;
    uint8_t i;
    uint8_t j;
    uint16_t iterations = 0;

    do 
    {
        std::tie(i, j) = ::FindMaxNormOffDiagonal(symmetricMatrix);

        rotationMatrix = ::MakeGivensRotationMatrix(::CalculateRotationAngle(symmetricMatrix, i, j, coverageThreshold), i, j);
        eigenvectors = eigenvectors * rotationMatrix;
        symmetricMatrix = (glm::transpose(rotationMatrix) * symmetricMatrix) * rotationMatrix;

        std::tie(i, j) = ::FindMaxNormOffDiagonal(symmetricMatrix);
    } 
    while ((++iterations < maxIterations) && (glm::abs(symmetricMatrix[i][j]) > coverageThreshold));

    return std::tuple<glm::mat3, glm::vec3>{ eigenvectors,  glm::vec3{ symmetricMatrix[0][0], symmetricMatrix[1][1], symmetricMatrix[2][2] } };
}

/**
 * @brief Calculates eigenvalues
 *
 * @param[in] symmetricMatrix target symmetric matrix
 * @param[in] coverageThreshold coverage precision threshold
 * @param[in] maxIterations maximum amount of iteration
 */
inline glm::vec3 CalculateJacobiEigenvalue(
    glm::mat3 symmetricMatrix, float coverageThreshold = epona::fp::g_floatingPointThreshold, uint32_t maxIterations = 100
)
{
    glm::mat3 rotationMatrix;
    uint8_t i;
    uint8_t j;
    uint16_t iterations = 0;

    do
    {
        std::tie(i, j) = ::FindMaxNormOffDiagonal(symmetricMatrix);

        rotationMatrix = ::MakeGivensRotationMatrix(::CalculateRotationAngle(symmetricMatrix, i, j, coverageThreshold), i, j);
        symmetricMatrix = (glm::transpose(rotationMatrix) * symmetricMatrix) * rotationMatrix;

        std::tie(i, j) = ::FindMaxNormOffDiagonal(symmetricMatrix);
    } while ((++iterations < maxIterations) && (glm::abs(symmetricMatrix[i][j]) > coverageThreshold));

    return { symmetricMatrix[0][0], symmetricMatrix[1][1], symmetricMatrix[2][2] };
}

/**
 * @brief Calculates eigenvectors
 *
 * @param[in] symmetricMatrix target symmetric matrix
 * @param[in] coverageThreshold coverage precision threshold
 * @param[in] maxIterations maximum amount of iteration
 */
inline glm::mat3 CalculateJacobiEigenvectors(
    glm::mat3 symmetricMatrix, float coverageThreshold = epona::fp::g_floatingPointThreshold, uint32_t maxIterations = 100
)
{
	glm::mat3 eigenvectors(1);
    glm::mat3 rotationMatrix(1);
    uint8_t i = 0;
    uint8_t j = 0;
    uint16_t iterations = 0;

    do
    {
        std::tie(i, j) = ::FindMaxNormOffDiagonal(symmetricMatrix);

        rotationMatrix = ::MakeGivensRotationMatrix(::CalculateRotationAngle(symmetricMatrix, i, j, coverageThreshold), i, j);
        eigenvectors = eigenvectors * rotationMatrix;
        symmetricMatrix = (glm::transpose(rotationMatrix) * symmetricMatrix) * rotationMatrix;

        std::tie(i, j) = ::FindMaxNormOffDiagonal(symmetricMatrix);
    } while ((++iterations < maxIterations) && (glm::abs(symmetricMatrix[i][j]) > coverageThreshold));

    return eigenvectors;
}

} // namespace epona
#endif // EPONA_JACOBI_EIGENVALUE_HPP
