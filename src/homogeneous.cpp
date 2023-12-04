#include <nori/medium.h>
#include <nori/frame.h>
#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

/// Homogeneous medium
class Homogeneous : public Medium {
public:
    Homogeneous(const PropertyList& propList) {
        Color3f sigma_a = propList.getColor("sigma_a", Color3f(0.0f));

        Color3f sigma_s = propList.getColor("sigma_s", Color3f(0.0f));

        sigma_t = sigma_a + sigma_s;
        albedo = sigma_s / sigma_t;
    }

    virtual Color3f tr(float t) const override {
        return std::exp(-sigma_t.mean() * t);
    }

    virtual float fp(float sample) const override {
        return -std::log(1 - sample) / sigma_t.mean();
    }

    virtual Color3f sample(MediumQueryRecord& mRec, const Point2f &sample) const override {
        mRec.wo = m_pf->sample(mRec.wi, sample);
        return albedo;
    }

    virtual std::string toString() const override {
        return tfm::format(
            "Homogeneous[\n"
            "  albedo = %s,\n"
            "  sigma_t = %s,\n"
            "  phase = %s\n"
            "]",
            albedo.toString(),
            sigma_t.toString(),
            indent(m_pf->toString())
        );
    }
private:
    Color3f albedo, sigma_t;
};

NORI_REGISTER_CLASS(Homogeneous, "homogeneous");
NORI_NAMESPACE_END
