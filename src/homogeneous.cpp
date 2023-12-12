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
    }

    virtual Color3f tr(float t) const override {
        return (-sigma_t * t).exp();
    }

    virtual Color3f sample(MediumQueryRecord& mRec, Sampler *sampler) const override {
        // is 1 - sample necessary
        // is sigma_t.maxCoeff better
        mRec.t = -std::log(1 - sampler->next1D()) / sigma_t[int(sampler->next1D() * 3)];
        if (mRec.t >= mRec.maxT) {
            const Color3f transmittance = tr(mRec.maxT);
            return transmittance /  transmittance.mean();
        }
        mRec.wo = m_pf->sample(mRec.wi, sampler->next2D());
        const Color3f transmittance = tr(mRec.t);
        return sigma_s * transmittance / (sigma_t * transmittance).mean();
    }

    virtual std::string toString() const override {
        return tfm::format(
            "Homogeneous[\n"
            "  sigma_t = %s,\n"
            "  phase = %s\n"
            "]",
            sigma_t.toString(),
            indent(m_pf->toString())
        );
    }
private:
    Color3f sigma_t, sigma_s;
};

NORI_REGISTER_CLASS(Homogeneous, "homogeneous");
NORI_NAMESPACE_END
