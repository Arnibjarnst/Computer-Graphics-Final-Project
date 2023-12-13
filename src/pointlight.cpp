#include <nori/common.h>
#include <nori/emitter.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN


class PointLight : public Emitter {
public:
    Point3f position;
    Color3f power;

    PointLight(const PropertyList& props) {
        power = props.getColor("power", Color3f(0.0f));
        position = props.getPoint3("position", Point3f(0.0f));
    };

    /**
     * \brief Sample the emitter and return the importance weight (i.e. the
     * value of the Emitter divided by the probability density
     * of the sample with respect to solid angles).
     *
     * \param lRec    An emitter query record (only ref is needed)
     * \param sample  A uniformly distributed sample on \f$[0,1]^2\f$
     *
     * \return The emitter value divided by the probability density of the sample.
     *         A zero value means that sampling failed.
     */
    Color3f sample(EmitterQueryRecord& lRec, const Point2f& sample) const {
        lRec.p = position;
        lRec.wi = (position - lRec.ref);
        float dist = lRec.wi.norm();
        lRec.wi /= dist;
        lRec.shadowRay = Ray3f(lRec.ref, lRec.wi, Epsilon, dist - Epsilon);
        lRec.pdf = 1;
        return eval(lRec) / pdf(lRec);
    };

    /**
     * \brief Evaluate the emitter
     *
     * \param lRec
     *     A record with detailed information on the emitter query
     * \return
     *     The emitter value, evaluated for each color channel
     */
    Color3f eval(const EmitterQueryRecord& lRec) const {
        float dist = (lRec.p - lRec.ref).norm();
        return power / (4 * M_PI * dist * dist);
    };

    /**
     * \brief Compute the probability of sampling \c lRec.p.
     *
     * This method provides access to the probability density that
     * is realized by the \ref sample() method.
     *
     * \param lRec
     *     A record with detailed information on the emitter query
     *
     * \return
     *     A probability/density value
     */
    float pdf(const EmitterQueryRecord& lRec) const {
        return 1;
    };


    /// Sample a photon
    Color3f samplePhoton(Ray3f& ray, const Point2f& sample1, const Point2f& sample2) const {
        throw NoriException("Emitter::samplePhoton(): not implemented!");
    };

    LightCone getLightCone() const override {
        LightCone res;
        res.axis = Vector3f(0.f);
        res.theta_e = M_PI_2;
        res.theta_o = M_PI;
        return res;
    }

    BoundingBox3f getBoundingBox() const override {
        return BoundingBox3f(position);
    }

    float getPower() const override {
        return power.maxCoeff();
    }

    std::string toString() const {
        return "PointLight[]";
    }
};

NORI_REGISTER_CLASS(PointLight, "point");
NORI_NAMESPACE_END
