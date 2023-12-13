/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Romain Pr�vost

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

class SpotlightAreaEmitter : public Emitter {
public:
    SpotlightAreaEmitter(const PropertyList& props) {
        m_innerRadiance = props.getColor("innerRadiance", 0.0f);
        m_outerRadiance = props.getColor("outerRadiance", 0.0f);
        m_theta = props.getFloat("theta", 180) / 360 * M_PI;
        m_cosTheta = std::cos(m_theta);
        m_thetaFalloff = props.getFloat("thetaFalloff", 180) / 360 * M_PI;
        m_cosThetaFalloff = std::cos(m_thetaFalloff);

        m_invTransitionWidth = 1 / (m_theta - m_thetaFalloff);

        assert(m_theta >= 0 && m_theta <= 180);
        assert(m_thetaFalloff >= 0 && m_thetaFalloff <= 180);
    }

    virtual std::string toString() const override {
        return tfm::format(
            "Spotlight[\n"
            "  inner radiance = %s\n"
            "  outer radiance = %s\n"
            "  theta = %f\n"
            "  theta_falloff = %f\n"
            "]",
            m_innerRadiance.toString(),
            m_outerRadiance.toString(),
            m_theta * 360 / M_PI,
            m_thetaFalloff * 360 / M_PI
        );
    }

    virtual Color3f eval(const EmitterQueryRecord& lRec) const override {
        if (!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        float cosTheta = lRec.n.dot(-lRec.wi);
        if (cosTheta < 0) return 0.0f;
        if (cosTheta < m_cosTheta) return m_outerRadiance;
        if (cosTheta >= m_cosThetaFalloff) return m_innerRadiance;
        float t = (m_theta - std::acos(cosTheta)) * m_invTransitionWidth;
        return ((float)1 - t) * m_outerRadiance + t * m_innerRadiance;
    }

    virtual Color3f sample(EmitterQueryRecord& lRec, const Point2f& sample) const override {
        if (!m_shape)
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

    virtual float pdf(const EmitterQueryRecord& lRec) const override {
        if (!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        float d2 = (lRec.p - lRec.ref).squaredNorm();
        float cos = -lRec.n.dot(lRec.wi);
        if (cos > 0) return m_shape->pdfSurface(ShapeQueryRecord(lRec.ref, lRec.p)) * d2 / cos;
        return 0.0f;
    }


    virtual Color3f samplePhoton(Ray3f& ray, const Point2f& sample1, const Point2f& sample2) const override {
        throw NoriException("Not Implemented!");
    }

    // TODO if there's time, consider the fallof for theta_e instead of M_PI_2
    LightCone getLightCone() const override {
        if (!m_shape)
            throw NoriException("There is no shape attached to this Area light!");
        return m_shape->getLightCone();
    }

    BoundingBox3f getBoundingBox() const override {
        if (!m_shape)
            throw NoriException("There is no shape attached to this Area light!");
        return m_shape->getBoundingBox();
    }


protected:
    Color3f m_innerRadiance;
    Color3f m_outerRadiance;
    float m_theta;
    float m_cosTheta;
    float m_thetaFalloff;
    float m_cosThetaFalloff;
    float m_invTransitionWidth;
};

NORI_REGISTER_CLASS(SpotlightAreaEmitter, "spotlight_area")
NORI_NAMESPACE_END