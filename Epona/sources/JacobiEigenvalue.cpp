/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <Epona/JacobiEigenvalue.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/glm.hpp>

using namespace epona;

JacobiEigenvalue::JacobiEigenvalue(glm::mat3 const& symmetricMatrix, float coverageThreshold, uint32_t maxIterations)
    : m_symmetricMatrix{symmetricMatrix}
    , m_coverageThreshold{glm::abs(coverageThreshold)}
    , m_maxIterations{maxIterations}
{
    Calculate();
}

glm::mat3 const& JacobiEigenvalue::GetEigenvectors() const
{
    return m_eigenvectors;
}

glm::vec3 const& JacobiEigenvalue::GetEigenvalues() const
{
    return m_eigenvalues;
}

void JacobiEigenvalue::FindMaxNormOffDiagonal(glm::mat3 const& mat, uint8_t& i, uint8_t& j)
{
    i = 0;
    j = 1;

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
}

float JacobiEigenvalue::CalculateRotationAngle(glm::mat3 const& mat, uint8_t i, uint8_t j) const
{
    if (glm::abs(mat[i][i] - mat[j][j]) < m_coverageThreshold)
    {
        return (glm::pi<float>() / 4.0f) * (mat[i][j] > 0 ? 1.0f : -1.0f);
    }

    return 0.5f * glm::atan(2.0f * mat[i][j] / (mat[i][i] - mat[j][j]));
}

glm::mat3 JacobiEigenvalue::MakeGivensRotationMatrix(float theta, uint8_t i, uint8_t j)
{
    glm::mat3 g;
    g[i][i] = glm::cos(theta);
    g[i][j] = glm::sin(theta);
    g[j][i] = -glm::sin(theta);
    g[j][j] = glm::cos(theta);

    return g;
}

void JacobiEigenvalue::Calculate()
{
    glm::mat3 D(m_symmetricMatrix);
    glm::mat3 S;

    uint16_t iterations = 0;
    bool iterate = true;
    while (iterate)
    {
        uint8_t i, j;
        FindMaxNormOffDiagonal(D, i, j);

        glm::mat3 S1 = MakeGivensRotationMatrix(CalculateRotationAngle(D, i, j), i, j);
        S = S * S1;
        D = (glm::transpose(S1) * D) * S1;

        FindMaxNormOffDiagonal(D, i, j);
        iterate = (++iterations < m_maxIterations) && (glm::abs(D[i][j]) > m_coverageThreshold);
    }

    m_eigenvalues = {D[0][0], D[1][1], D[2][2]};
    m_eigenvectors = S;
}
