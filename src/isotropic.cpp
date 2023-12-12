#include <nori/medium.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Isotropic : public PhaseFunction {
public:
    Isotropic(const PropertyList& propList) {}

    virtual Vector3f sample(Vector3f &wi, const Point2f& sample) const override {
        return Warp::squareToUniformSphere(sample);
    }

    virtual std::string toString() const override {
        return "Isotropic";
    }

};

NORI_REGISTER_CLASS(Isotropic, "isotropic");
NORI_NAMESPACE_END
