/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Romain Prévost

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

#include <nori/shape.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

class Sphere : public Shape {
public:
    Sphere(const PropertyList & propList) {
        m_position = propList.getPoint3("center", Point3f(0.0f));
        m_radius = propList.getFloat("radius", 1.f);
        m_radius2 = m_radius * m_radius;

        m_bbox.expandBy(m_position - Vector3f(m_radius));
        m_bbox.expandBy(m_position + Vector3f(m_radius));
    }

    virtual BoundingBox3f getBoundingBox(uint32_t index) const override { return m_bbox; }

    virtual Point3f getCentroid(uint32_t index) const override { return m_position; }

    virtual bool rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const override {
        Vector3f oc = ray.o - m_position;
        float A = ray.d.dot(ray.d);
        float B = 2 * oc.dot(ray.d);
        float C = oc.dot(oc) - m_radius * m_radius;
        float D = B * B - 4 * A * C;

        if (D < 0) return false;

        float Dsqrt = std::sqrt(D);
        t = (- B - Dsqrt) / (2 * A);
        if (t < ray.mint || t > ray.maxt) t = (-B + Dsqrt) / (2 * A);
        else return true;

        return t >= ray.mint && t <= ray.maxt;
    }

    virtual void setHitInformation(uint32_t index, const Ray3f &ray, Intersection & its) const override {
        its.p = ray(its.t);
        Vector3f dir = (its.p - m_position).normalized();
        its.geoFrame = Frame(dir);
        its.uv = Point2f(
            std::atan2(dir.y(), dir.x()) * INV_TWOPI + 0.5,
            std::asin(dir.z()) * INV_PI + 0.5
        );

        // based of Mitsuba3
        its.dpdu = Vector3f(-dir.y(), dir.x(), 0.0f) * M_PI * 2;
        float rd = its.dpdu.norm();
        float invRd = 1 / rd;
        float cos_phi = dir.x() * invRd;
        float sin_phi = dir.y() * invRd;
        its.dpdv = Vector3f(dir.z() * cos_phi, dir.z() * sin_phi, -rd) * M_PI;
        Vector3f s = (its.dpdu + dir * dir.dot(its.dpdu)).normalized();
        its.shFrame = Frame(s, dir.cross(s), dir);
    }

    virtual void sampleSurface(ShapeQueryRecord & sRec, const Point2f & sample) const override {
        Vector3f q = Warp::squareToUniformSphere(sample);
        sRec.p = m_position + m_radius * q;
        sRec.n = q;
        sRec.pdf = std::pow(1.f/m_radius,2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f,0.0f,1.0f));
    }
    virtual float pdfSurface(const ShapeQueryRecord & sRec) const override {
        return std::pow(1.f/m_radius,2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f,0.0f,1.0f));
    }


    virtual std::string toString() const override {
        return tfm::format(
            "Sphere[\n"
            "  center = %s,\n"
            "  radius = %f,\n"
            "  bsdf = %s,\n"
            "  emitter = %s\n"
            "  interior_medium = %s,\n"
            "  exterior_medium = %s,\n"
            "]",
            m_position.toString(),
            m_radius,
            m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
            m_emitter ? indent(m_emitter->toString()) : std::string("null"),
            m_interior ? indent(m_interior->toString()) : std::string("null"),
            m_exterior ? indent(m_exterior->toString()) : std::string("null")
        );
    }

protected:
    Point3f m_position;
    float m_radius;
    float m_radius2;
};

NORI_REGISTER_CLASS(Sphere, "sphere");
NORI_NAMESPACE_END
