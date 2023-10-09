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
        lRec.wi = position - lRec.ref;
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
        return power;
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
        const float r = lRec.wi.norm();
        return 4 * M_PI * r * r * r / 3;
    };


    /// Sample a photon
    Color3f samplePhoton(Ray3f& ray, const Point2f& sample1, const Point2f& sample2) const {
        throw NoriException("Emitter::samplePhoton(): not implemented!");
    };

    std::string toString() const {
        return "PointLight[]";
    }
};

NORI_REGISTER_CLASS(PointLight, "point");
NORI_NAMESPACE_END
