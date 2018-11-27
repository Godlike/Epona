/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <Epona/HyperPlane.hpp>

using namespace epona;

HyperPlane::HyperPlane(glm::vec3 const& normal, glm::vec3 const& point, glm::vec3 const* below)
    : m_normal(normal)
    , m_point(point)
    , m_distance(glm::dot(m_normal, m_point))
{
    if (below != nullptr)
    {
        glm::vec3 const outward = point - *below;
        if (glm::dot(outward, m_normal) < 0.0f)
        {
            SetNormal(m_normal * -1.0f);
        }
    }
}

HyperPlane::HyperPlane(
    glm::vec3 const& a, glm::vec3 const& b, glm::vec3 const& c, glm::vec3 const* below
)
    : HyperPlane(glm::normalize(glm::cross(a - c, b - c)), c, below)
{
    assert(!glm::isnan(m_normal.x));
}

HyperPlane::HyperPlane(glm::mat3 const& vertices, glm::vec3 const* below)
    : HyperPlane(vertices[0], vertices[1], vertices[2], below)
{
}

glm::vec3 const& HyperPlane::GetPoint() const
{
    return m_point;
}

glm::vec3 const& HyperPlane::GetNormal() const
{
    return m_normal;
}

float HyperPlane::GetDistance() const
{
    return m_distance;
}

void HyperPlane::SetNormal(glm::vec3 const& normal)
{
    m_normal = normal;
    m_distance = glm::dot(m_normal, m_point);
}

void HyperPlane::SetPoint(glm::vec3 const& point)
{
    m_point = point;
    m_distance = glm::dot(m_normal, m_point);
}

float HyperPlane::Distance(glm::vec3 const& point) const
{
    return glm::abs(SignedDistance(point));
}

float HyperPlane::SignedDistance(glm::vec3 const& point) const
{
    return glm::dot(m_normal, point) - m_distance;
}

bool HyperPlane::RayIntersection(
    glm::vec3 const& rayNormal, glm::vec3 const& rayPoint, glm::vec3& resultPoint
) const
{
    float const rayPlaneProjection = glm::dot(m_normal, rayNormal);

    if (rayPlaneProjection != 0.0f)
    {
        float const t = (glm::dot(m_normal, m_point - rayPoint)) / rayPlaneProjection;
        resultPoint = rayPoint + rayNormal * t;
        return true;
    }

    return false;
}

bool HyperPlane::LineSegmentIntersection(
    glm::vec3 const& lineStart, glm::vec3 const& lineEnd, glm::vec3& resultPoint
) const
{
    if ((glm::dot(lineStart, m_normal) - m_distance)
        * (glm::dot(lineEnd, m_normal) - m_distance) >= 0)
    {
        return false;
    }

    glm::vec3 const lineNormal = glm::normalize(lineEnd - lineStart);

    return RayIntersection(lineNormal, lineStart, resultPoint);
}

glm::vec3 HyperPlane::ClosestPoint(const glm::vec3& point) const
{
    glm::vec3 const closestPoint = point - (glm::dot(point, m_normal) - m_distance) * m_normal;

    return closestPoint;
}
