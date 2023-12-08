#include <nori/medium.h>
#include <nori/frame.h>
#include <nori/texture.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

/// Homogeneous medium
class Homogeneous : public Medium {
public:
    Homogeneous(const PropertyList& propList) {
        Color3f sigma_a = propList.getColor("sigma_a", Color3f(0.0f));

        sigma_s = propList.getColor("sigma_s", Color3f(0.0f));

        sigma_t = sigma_a + sigma_s + Epsilon;
        albedo = sigma_s / sigma_t;
    }

    virtual Color3f tr(float t) const override {
        return (-sigma_t * t).exp();
    }

    virtual float fp(float sample) const override {
        return -std::log(1 - sample) / sigma_t.mean(); // improve
    }

    virtual Color3f sample(MediumQueryRecord& mRec, Sampler *sampler) const override {
        int channel = int(sampler->next1D() * 3);
        mRec.t = -std::log(1 - sampler->next1D()) / sigma_t[channel]; // is 1 - sample necessary ?
        if (mRec.t >= mRec.maxT) {
            return tr(mRec.maxT) /  (-sigma_t * mRec.maxT).exp().mean();
        }
        mRec.wo = m_pf->sample(mRec.wi, sampler->next2D());
        return sigma_s * tr(mRec.t) / (sigma_t * (-sigma_t * mRec.t).exp()).mean();
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
    Color3f albedo, sigma_t, sigma_s;
};

NORI_REGISTER_CLASS(Homogeneous, "homogeneous");
NORI_NAMESPACE_END
