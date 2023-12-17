/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob, Romain Prévost

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/mesh.h>
#include <nori/bbox.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

Mesh::Mesh() { }

void Mesh::activate() {
    Shape::activate();

    m_pdf.reserve(getPrimitiveCount());
    for(uint32_t i = 0 ; i < getPrimitiveCount() ; ++i) {
        m_pdf.append(surfaceArea(i));
    }
    m_pdf.normalize();
}

void Mesh::sampleSurface(ShapeQueryRecord & sRec, const Point2f & sample) const {
    Point2f s = sample;
    size_t idT = m_pdf.sampleReuse(s.x());

    Vector3f bc = Warp::squareToUniformTriangle(s);

    sRec.p = getInterpolatedVertex(idT,bc);
    if (m_N.size() > 0) {
        sRec.n = getInterpolatedNormal(idT, bc);
    }
    else {
        Point3f p0 = m_V.col(m_F(0, idT));
        Point3f p1 = m_V.col(m_F(1, idT));
        Point3f p2 = m_V.col(m_F(2, idT));
        Normal3f n = (p1-p0).cross(p2-p0).normalized();
        sRec.n = n;
    }
    sRec.pdf = m_pdf.getNormalization();
}
float Mesh::pdfSurface(const ShapeQueryRecord & sRec) const {
    return m_pdf.getNormalization();
}

Point3f Mesh::getInterpolatedVertex(uint32_t index, const Vector3f &bc) const {
    return (bc.x() * m_V.col(m_F(0, index)) +
            bc.y() * m_V.col(m_F(1, index)) +
            bc.z() * m_V.col(m_F(2, index)));
}

Normal3f Mesh::getInterpolatedNormal(uint32_t index, const Vector3f &bc) const {
    return (bc.x() * m_N.col(m_F(0, index)) +
            bc.y() * m_N.col(m_F(1, index)) +
            bc.z() * m_N.col(m_F(2, index))).normalized();
}

float Mesh::surfaceArea(uint32_t index) const {
    uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);

    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    return 0.5f * Vector3f((p1 - p0).cross(p2 - p0)).norm();
}

bool Mesh::rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const {
    uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);
    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    /* Find vectors for two edges sharing v[0] */
    Vector3f edge1 = p1 - p0, edge2 = p2 - p0;

    /* Begin calculating determinant - also used to calculate U parameter */
    Vector3f pvec = ray.d.cross(edge2);

    /* If determinant is near zero, ray lies in plane of triangle */
    float det = edge1.dot(pvec);

    if (det > -1e-8f && det < 1e-8f)
        return false;
    float inv_det = 1.0f / det;

    /* Calculate distance from v[0] to ray origin */
    Vector3f tvec = ray.o - p0;

    /* Calculate U parameter and test bounds */
    u = tvec.dot(pvec) * inv_det;
    if (u < 0.0 || u > 1.0)
        return false;

    /* Prepare to test V parameter */
    Vector3f qvec = tvec.cross(edge1);

    /* Calculate V parameter and test bounds */
    v = ray.d.dot(qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0)
        return false;

    /* Ray intersects triangle -> compute t */
    t = edge2.dot(qvec) * inv_det;

    return t >= ray.mint && t <= ray.maxt;
}

void Mesh::setHitInformation(uint32_t index, const Ray3f &ray, Intersection & its) const {
    /* Find the barycentric coordinates */
    Vector3f bary;
    bary << 1-its.uv.sum(), its.uv;

    /* Vertex indices of the triangle */
    uint32_t idx0 = m_F(0, index), idx1 = m_F(1, index), idx2 = m_F(2, index);

    Point3f p0 = m_V.col(idx0), p1 = m_V.col(idx1), p2 = m_V.col(idx2);

    /* Compute the intersection positon accurately
       using barycentric coordinates */
    its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

    /* Compute proper texture coordinates if provided by the mesh */
    if (m_UV.size() > 0) {
        Point2f uv0 = m_UV.col(idx0);
        Point2f uv1 = m_UV.col(idx1);
        Point2f uv2 = m_UV.col(idx2);
        Vector2f duv02 = uv0 - uv2, duv12 = uv1 - uv2;
        Vector3f dp02 = p0 - p2, dp12 = p1 - p2;
        float det = duv02[0] * duv12[1] - duv02[1] * duv12[0];
        if (det == 0) {
            // should this be (p1-p0).cross(p2-p0) like geoFrame
            // or be in the other direction
            coordinateSystem((p2 - p0).cross(p1 - p0).normalized(), its.dpdu, its.dpdv);
        }
        else {
            float invDet = 1 / det;
            its.dpdu = invDet * (duv12[1] * dp02 - duv02[1] * dp12);
            its.dpdv = invDet * (-duv12[0] * dp02 + duv02[0] * dp12);
        }
        its.uv = bary.x() * uv0 + bary.y() * uv1 + bary.z() * uv2;
    }

    /* Compute the geometry frame */
    its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());

    if (m_N.size() > 0) {
        /* Compute the shading frame. Note that for simplicity,
           the current implementation doesn't attempt to provide
           tangents that are continuous across the surface. That
           means that this code will need to be modified to be able
           use anisotropic BRDFs, which need tangent continuity */
        Normal3f n = (
            bary.x() * m_N.col(idx0) +
            bary.y() * m_N.col(idx1) +
            bary.z() * m_N.col(idx2)).normalized();
        Vector3f s = (its.dpdu + n * n.dot(its.dpdu)).normalized();
        its.shFrame = Frame(s, n.cross(s), n);
    } else {
        its.shFrame = its.geoFrame;
    }

    its.computeDifferentials(ray);
}

BoundingBox3f Mesh::getBoundingBox(uint32_t index) const {
    BoundingBox3f result(m_V.col(m_F(0, index)));
    result.expandBy(m_V.col(m_F(1, index)));
    result.expandBy(m_V.col(m_F(2, index)));
    return result;
}

LightCone Mesh::getLightCone(uint32_t index) const {
    LightCone res;
    uint32_t idx0 = m_F(0, index), idx1 = m_F(1, index), idx2 = m_F(2, index);
    Point3f p0 = m_V.col(idx0), p1 = m_V.col(idx1), p2 = m_V.col(idx2);
    res.axis = (p1-p0).cross(p2-p0).normalized();
    res.theta_o = 0.;
    res.theta_e = M_PI / 2;
    return res;
}

LightCone Mesh::getLightCone() const  {
        LightCone res;
        for (uint32_t i = 0; i < getPrimitiveCount(); i++) res.expandBy(getLightCone(i));
        return res;
    }

Point3f Mesh::getCentroid(uint32_t index) const {
    return (1.0f / 3.0f) *
        (m_V.col(m_F(0, index)) +
         m_V.col(m_F(1, index)) +
         m_V.col(m_F(2, index)));
}


std::string Mesh::toString() const {
    return tfm::format(
        "Mesh[\n"
        "  name = \"%s\",\n"
        "  vertexCount = %i,\n"
        "  triangleCount = %i,\n"
        "  bsdf = %s,\n"
        "  emitter = %s\n"
        "  interior_medium = %s,\n"
        "  exterior_medium = %s,\n"
        "]",
        m_name,
        m_V.cols(),
        m_F.cols(),
        m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
        m_emitter ? indent(m_emitter->toString()) : std::string("null"),
        m_interior ? indent(m_interior->toString()) : std::string("null"),
        m_exterior ? indent(m_exterior->toString()) : std::string("null")
    );
}

NORI_NAMESPACE_END
