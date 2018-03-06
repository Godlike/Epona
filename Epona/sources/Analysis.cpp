/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <Epona/Analysis.hpp>
#include <glm/glm.hpp>

double epona::LineSegmentPointDistance(
    glm::dvec3 const& lineStart, glm::dvec3 const& lineEnd, glm::dvec3 point
)
{
    point = point - lineStart;
    glm::dvec3 const lineDirection = glm::normalize(lineEnd - lineStart);
    glm::dvec3 const pointLineProjection = glm::dot(lineDirection, point) * lineDirection;
    return glm::distance(point, pointLineProjection);
}

glm::dvec3 epona::CalculateBarycentricCoordinates(glm::dvec3 p, glm::dvec3 a, glm::dvec3 b, glm::dvec3 c)
{
    glm::dvec3 const v0 = b - a;
    glm::dvec3 const v1 = c - a;
    glm::dvec3 const v2 = p - a;

    double const d00 = glm::dot(v0, v0);
    double const d01 = glm::dot(v0, v1);
    double const d11 = glm::dot(v1, v1);
    double const d20 = glm::dot(v2, v0);
    double const d21 = glm::dot(v2, v1);
    double const denominator = d00 * d11 - d01 * d01;

    glm::dvec3 coordinates;
    coordinates.y = (d11 * d20 - d01 * d21) / denominator;
    coordinates.z = (d00 * d21 - d01 * d20) / denominator;
    coordinates.x = 1.0 - coordinates.y - coordinates.z;

    return coordinates;
}
