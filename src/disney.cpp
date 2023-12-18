/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

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

#include <nori/bsdf.h>
#include <nori/warp.h>
#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

float SchlickFresnel(float u){
    float m = clamp(1 - u, 0.0f, 1.0f);
    float m2 = m * m;
    return m2 * m2 * m; // pow(m,5)
}

float GTR2(float NdotH, float a) {
    float a2 = a * a;
    float t = 1 + (a2 - 1) * NdotH * NdotH;
    return a2 / (M_PI * t * t);
}

float GTR2_aniso(float NdotH, float HdotX, float HdotY, float ax, float ay) {
    return 1 / (M_PI * ax * ay * pow(pow(HdotX / ax, 2) + pow(HdotY / ay, 2) + NdotH * NdotH, 2));
}

float smithG_GGX_aniso(float NdotV, float VdotX, float VdotY, float ax, float ay) {
    return 1 / (NdotV + sqrt(pow(VdotX * ax, 2) + pow(VdotY * ay, 2) + pow(NdotV, 2)));
}


class Disney : public BSDF {
public:
    Disney(const PropertyList& props) {
        // only color has no default argument
        if (props.has("color")) {
            PropertyList l;
            l.setColor("value", props.getColor("color"));
            m_color = static_cast<Texture<Color3f> *>(NoriObjectFactory::createInstance("constant_color", l));
        }
        if (props.has("roughness")) {
            PropertyList lr;
            lr.setFloat("value", props.getFloat("roughness"));
            m_roughness = static_cast<Texture<float> *>(NoriObjectFactory::createInstance("constant_float", lr));
        }
        if (props.has("metallic")) {
            PropertyList lm;
            lm.setFloat("value", props.getFloat("metallic"));
            m_metallic = static_cast<Texture<float> *>(NoriObjectFactory::createInstance("constant_float", lm));
        }
        if (props.has("specular")) {
            PropertyList ls;
            ls.setFloat("value", props.getFloat("specular"));
            m_specular = static_cast<Texture<float> *>(NoriObjectFactory::createInstance("constant_float", ls));
        }
        if (props.has("anisotropic")) {
            PropertyList la;
            la.setFloat("value", props.getFloat("anisotropic"));
            m_anisotropic = static_cast<Texture<float> *>(NoriObjectFactory::createInstance("constant_float", la));
        }
        if (props.has("subsurface")) {
            PropertyList lss;
            lss.setFloat("value", props.getFloat("subsurface"));
            m_subsurface = static_cast<Texture<float> *>(NoriObjectFactory::createInstance("constant_float", lss));
        }
    }

    ~Disney() {
        delete m_color;
        delete m_roughness;
        delete m_metallic;
        delete m_specular;
        delete m_anisotropic;
        delete m_subsurface;
    }

    virtual void activate() {
        if (!m_roughness) {
            PropertyList lr;
            lr.setFloat("value", 0.5f);
            m_roughness = static_cast<Texture<float> *>(NoriObjectFactory::createInstance("constant_float", lr));
        }
        if (!m_metallic) {
            PropertyList lm;
            lm.setFloat("value", 0.0f);
            m_metallic = static_cast<Texture<float> *>(NoriObjectFactory::createInstance("constant_float", lm));
        }
        if (!m_specular) {
            PropertyList ls;
            ls.setFloat("value", 0.5f);
            m_specular = static_cast<Texture<float> *>(NoriObjectFactory::createInstance("constant_float", ls));
        }
        if (!m_anisotropic) {
            PropertyList la;
            la.setFloat("value", 0.0f);
            m_anisotropic = static_cast<Texture<float> *>(NoriObjectFactory::createInstance("constant_float", la));
        }
        if (!m_subsurface) {
            PropertyList lss;
            lss.setFloat("value", 0.0f);
            m_subsurface = static_cast<Texture<float> *>(NoriObjectFactory::createInstance("constant_float", lss));
        }
    }

    virtual void addChild(NoriObject* obj) override {
        switch (obj->getClassType()) {
            case ETexture:
                if (obj->getIdName() == "color") {
                    if (m_color)
                        throw NoriException("There is already a color defined!");
                    m_color = static_cast<Texture<Color3f> *>(obj);
                }
                else if (obj->getIdName() == "roughness") {
                    if (m_roughness)
                        throw NoriException("There is already a roughness defined!");
                    m_roughness = static_cast<Texture<float> *>(obj);
                }
                else if (obj->getIdName() == "metallic") {
                    if (m_metallic)
                        throw NoriException("There is already a metallic defined!");
                    m_metallic = static_cast<Texture<float> *>(obj);
                }
                else if (obj->getIdName() == "specular") {
                    if (m_specular)
                        throw NoriException("There is already a specular defined!");
                    m_specular = static_cast<Texture<float> *>(obj);
                }
                else if (obj->getIdName() == "anisotropic") {
                    if (m_anisotropic)
                        throw NoriException("There is already an anisotropic defined!");
                    m_anisotropic = static_cast<Texture<float> *>(obj);
                }
                else if (obj->getIdName() == "subsurface") {
                    if (m_subsurface)
                        throw NoriException("There is already a subsurface defined!");
                    m_subsurface = static_cast<Texture<float> *>(obj);
                }
                else {
                    throw NoriException("The name of this texture does not match any field!");
                }
                break;

            default:
                throw NoriException("Disney::addChild(<%s>) is not supported!",
                    classTypeName(obj->getClassType()));
            }
    }


    /// Evaluate the BRDF for the given pair of directions
    virtual Color3f eval(const BSDFQueryRecord& bRec) const override {
        float cos_theta_i = bRec.wi.z();
        float cos_theta_o = bRec.wo.z();
        if (cos_theta_i <= 0 || cos_theta_o <= 0) return 0.0f;
        Vector3f wh = (bRec.wi + bRec.wo).normalized();
        float cos_theta_d = wh.dot(bRec.wo);

        Color3f color = m_color->eval(*bRec.its); // linear rgb
        float roughness = m_roughness->eval(*bRec.its);
        float metallic = m_metallic->eval(*bRec.its);
        float specular = m_specular->eval(*bRec.its);
        float anisotropic = m_anisotropic->eval(*bRec.its);
        float subsurface = m_subsurface->eval(*bRec.its);

        Color3f cSpec0 = (1 - metallic) * 0.08f * specular + metallic * color;

        // Diffuse fresnel
        float Fd90 = 0.5 + 2 * roughness * cos_theta_d * cos_theta_d;
        float SFi = SchlickFresnel(cos_theta_i);
        float SFo = SchlickFresnel(cos_theta_o);
        float Fd = lerp(SFi, 1.0f, Fd90) * lerp(SFo, 1.0f, Fd90);

        // subsurface
        float Fss90 = cos_theta_d * cos_theta_d * roughness;
        float Fss = lerp(SFi, 1.0f, Fss90) * lerp(SFo, 1.0f, Fss90);
        float ss = 1.25f * (Fss * (1.0f / (cos_theta_i + cos_theta_o) - 0.5f) + 0.5f);

        // specular
        float aspect = sqrt(1.0f - 0.9f * anisotropic);
        float ax = std::max(0.001f, roughness * roughness / aspect);
        float ay = std::max(0.001f, roughness * roughness * aspect);
        float Ds = GTR2_aniso(wh.z(), wh.x(), wh.y(), ax, ay);
        float FH = SchlickFresnel(cos_theta_d);
        Color3f Fs = (1 - FH) * cSpec0 + FH;
        float Gs = 
            smithG_GGX_aniso(cos_theta_o, bRec.wo.x(), bRec.wo.y(), ax, ay) *
            smithG_GGX_aniso(cos_theta_i, bRec.wi.x(), bRec.wi.y(), ax, ay);

        return lerp(subsurface, Fd, ss) * color * INV_PI * (1 - metallic) + Ds * Fs * Gs;
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    virtual float pdf(const BSDFQueryRecord& bRec) const override {
        if (bRec.measure != ESolidAngle || bRec.wi.z() <= 0 || bRec.wo.z() <= 0)
            return 0.0f;

        float roughness = m_roughness->eval(*bRec.its);
        float metallic = m_metallic->eval(*bRec.its);
        float anisotropic = m_anisotropic->eval(*bRec.its);

        Vector3f wh = (bRec.wi + bRec.wo).normalized();
        float aspect = sqrt(1.0f - 0.9f * anisotropic);
        float ax = std::max(0.001f, roughness * roughness / aspect);
        float ay = std::max(0.001f, roughness * roughness * aspect);
        float w_diff = (1.0f - metallic);
        float w_spec = 1;
        float p_diff = w_diff / (w_diff + w_spec);
        return p_diff * bRec.wo.z() * INV_PI + (1 - p_diff) * Warp::squareToGTR2Anisopdf(wh, ax, ay) * wh.z() / (4 * wh.dot(bRec.wo)); // what is alpha
    }

    /// Sample the BRDF
    virtual Color3f sample(BSDFQueryRecord& bRec, const Point2f& _sample) const override {
        if (bRec.wi.z() <= 0)
            return Color3f(0.0f);
        bRec.measure = ESolidAngle;
        bRec.eta = 1.0f;

        float metallic = m_metallic->eval(*bRec.its);

        float w_diff = (1.0f - metallic);
        float w_spec = 1;
        float p_diff = w_diff / (w_diff + w_spec);
        if (_sample.x() < p_diff) {
            // sample diffuse
            const Point2f sample = Point2f(_sample.x() / p_diff, _sample.y());
            bRec.wo = Warp::squareToCosineHemisphere(sample);
        }
        else {
            // sample specular
            float roughness = m_roughness->eval(*bRec.its);
            float anisotropic = m_anisotropic->eval(*bRec.its);
            const Point2f sample = Point2f((_sample.x() - p_diff) / (1 - p_diff), _sample.y());
            float aspect = sqrt(1.0f - 0.9f * anisotropic);
            float ax = std::max(0.001f, roughness * roughness / aspect);
            float ay = std::max(0.001f, roughness * roughness * aspect);
            Vector3f wh = Warp::squareToGTR2Aniso(sample, ax, ay);
            bRec.wo = 2 * (bRec.wi.dot(wh)) * wh - bRec.wi;
            if (bRec.wo.z() <= 0) return Color3f(0.0f);
        }

        return bRec.wo.z() * eval(bRec) / pdf(bRec);
    }

    virtual std::string toString() const override {
        return tfm::format(
            "Disney[\n"
            "  color = %s,\n"
            "  roughness = %s,\n"
            "  specular = %s,\n"
            "  metallic = %s,\n"
            "  anisotropic = %s\n"
            "  subsurface = %s\n"
            "]",
            m_color->toString(),
            m_roughness->toString(),
            m_specular->toString(),
            m_metallic->toString(),
            m_anisotropic->toString(),
            m_subsurface->toString()
        );
    }
private:
    Texture<Color3f> *m_color = nullptr;
    Texture<float> *m_roughness = nullptr;
    Texture<float> *m_specular = nullptr;
    Texture<float> *m_metallic = nullptr;
    Texture<float> *m_anisotropic = nullptr;
    Texture<float> *m_subsurface = nullptr;
};

NORI_REGISTER_CLASS(Disney, "disney");
NORI_NAMESPACE_END
