/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <Epona/Analysis.hpp>
#include <glm/glm.hpp>

float epona::LineSegmentPointDistance(
    glm::vec3 const& lineStart, glm::vec3 const& lineEnd, glm::vec3 point
)
{
    point = point - lineStart;
    glm::vec3 const lineDirection = glm::normalize(lineEnd - lineStart);
    glm::vec3 const pointLineProjection = glm::dot(lineDirection, point) * lineDirection;
    return glm::distance(point, pointLineProjection);
}

glm::vec3 epona::CalculateBarycentricCoordinates(glm::vec3 p, glm::vec3 a, glm::vec3 b, glm::vec3 c)
{
    glm::vec3 const v0 = b - a;
    glm::vec3 const v1 = c - a;
    glm::vec3 const v2 = p - a;

    float const d00 = glm::dot(v0, v0);
    float const d01 = glm::dot(v0, v1);
    float const d11 = glm::dot(v1, v1);
    float const d20 = glm::dot(v2, v0);
    float const d21 = glm::dot(v2, v1);
    float const denominator = d00 * d11 - d01 * d01;

    glm::vec3 coordinates;
    coordinates.y = (d11 * d20 - d01 * d21) / denominator;
    coordinates.z = (d00 * d21 - d01 * d20) / denominator;
    coordinates.x = 1.0f - coordinates.y - coordinates.z;

    return coordinates;
}
