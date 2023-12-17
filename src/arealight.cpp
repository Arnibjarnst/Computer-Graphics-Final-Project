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

#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/shape.h>
#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN

class AreaEmitter : public Emitter {
public:
    AreaEmitter(const PropertyList &props) {
        m_radiance = props.getColor("radiance");
    }

    virtual std::string toString() const override {
        return tfm::format(
                "AreaLight[\n"
                "  radiance = %s,\n"
                "]",
                m_radiance.toString());
    }

    virtual Color3f eval(const EmitterQueryRecord & lRec) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        if (lRec.n.dot(lRec.wi) < 0) return m_radiance;
        return Color3f(0.0f);
    }

    virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        ShapeQueryRecord shape_query(lRec.ref);
        m_shape->sampleSurface(shape_query, sample);
        lRec.p = shape_query.p;
        lRec.n = shape_query.n;
        lRec.wi = (lRec.p - lRec.ref).normalized();
        lRec.pdf = pdf(lRec);
        lRec.shadowRay = Ray3f(lRec.ref, lRec.wi, Epsilon, (lRec.p - lRec.ref).norm() - Epsilon);

        if (!lRec.pdf) return Color3f(0.0f);
        return eval(lRec) / lRec.pdf;
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        float d2 = (lRec.p - lRec.ref).squaredNorm();
        float cos = -lRec.n.dot(lRec.wi);
        if (cos > 0) return m_shape->pdfSurface(ShapeQueryRecord(lRec.ref, lRec.p)) * d2 / cos;
        return 0.0f;
    }


    virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const override {
        ShapeQueryRecord shape_query;
        m_shape->sampleSurface(shape_query, sample1);
        ray.o = shape_query.p;
        ray.d = Frame(shape_query.n).toWorld(Warp::squareToCosineHemisphere(sample2));
        ray.update();

        return M_PI / shape_query.pdf * m_radiance;
    }

    virtual LightCone getLightCone() const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");
        return m_shape->getLightCone();
    }

    virtual BoundingBox3f getBoundingBox() const override {
        if (!m_shape)
            throw NoriException("There is no shape attached to this Area light!");
        return m_shape->getBoundingBox();
    }

    // approx
    float getPower() const override {
        float angle = getLightCone().theta_o + getLightCone().theta_e;
        float solid_angle = 2 * M_PI * (1 - std::cos(angle));
        return m_radiance.maxCoeff() * solid_angle * getBoundingBox().getSurfaceArea();
    }


protected:
    Color3f m_radiance;
};

NORI_REGISTER_CLASS(AreaEmitter, "area")
NORI_NAMESPACE_END