#if !defined(__NORI_MEDIUM_H)
#define __NORI_MEDIUM_H

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

struct MediumQueryRecord {
    Vector3f wi;

    Vector3f wo;

    float t;

    float maxT;

    /// Create a new record for sampling the BSDF
    MediumQueryRecord(const Vector3f& wi)
        : wi(wi), maxT(std::numeric_limits<float>::max()) { }

    /// Create a new record for sampling the BSDF
    MediumQueryRecord(const Vector3f& wi, float maxT)
        : wi(wi), maxT(maxT) { }
};

/**
 * \brief Superclass of all mediums
 */
class Medium : public NoriObject {
public:
    /// Release all memory
    virtual ~Medium();

    virtual void addChild(NoriObject* child) override;

    virtual Color3f tr(const float t) const = 0;

    virtual float fp(const float sample) const = 0;

    virtual Color3f sample(MediumQueryRecord& mRec, Sampler *sampler) const = 0;

    virtual EClassType getClassType() const override { return EMedium; }

protected:
    PhaseFunction* m_pf = nullptr; // phase function of the medium
};


class PhaseFunction : public NoriObject {
public:
    virtual Vector3f sample(Vector3f &wi, const Point2f &sample) const = 0;

    virtual EClassType getClassType() const override { return EPhaseFunction; }
};

NORI_NAMESPACE_END

#endif /* __NORI_MEDIUM_H */
