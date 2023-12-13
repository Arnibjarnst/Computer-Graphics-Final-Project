#include <nori/common.h>
#include <nori/emitter.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN


class SpotLight : public Emitter {
public:

    SpotLight(const PropertyList& props) {
        m_power = props.getColor("power", Color3f(0.0f));
        m_position = props.getPoint3("position", Point3f(0.0f));
        m_direction = props.getVector3("direction", Vector3f(1, 0, 0)).normalized();
        m_theta = props.getFloat("theta", 180) / 360 * M_PI;
        m_cosTheta = std::cos(m_theta);
        m_thetaFalloff = props.getFloat("thetaFalloff", 180) / 360 * M_PI;
        m_cosThetaFalloff = std::cos(m_thetaFalloff);

        m_invTransitionWidth = 1 / (m_theta - m_thetaFalloff);

        assert(m_theta >= 0 && m_theta <= 180);
        assert(m_thetaFalloff >= 0 && m_thetaFalloff <= 180);
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
        lRec.p = m_position;
        lRec.wi = (m_position - lRec.ref);
        float dist = lRec.wi.norm();
        lRec.wi /= dist;
        lRec.shadowRay = Ray3f(lRec.ref, lRec.wi, Epsilon, dist - Epsilon);
        lRec.pdf = pdf(lRec);
        return eval(lRec) / lRec.pdf;
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
        float cosTheta = m_direction.dot(-lRec.wi);
        if (cosTheta < m_cosTheta) return 0.0f;
        float dist = (lRec.p - lRec.ref).norm();
        if (cosTheta >= m_cosThetaFalloff) return m_power / (dist * dist);
        return m_power * (m_theta - std::acos(cosTheta)) * m_invTransitionWidth /  (dist * dist);
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
        res.axis = m_direction;
        res.theta_e = m_theta;
        res.theta_o = 0.f;
    }

    BoundingBox3f getBoundingBox() const override {
        return BoundingBox3f(m_position);
    }

    float getPower() const override {
        return m_power.maxCoeff();
    }

    std::string toString() const {
        return tfm::format(
            "Spotlight[\n"
            "  position = %s\n"
            "  power = %s,\n"
            "  transform = %s\n"
            "  theta = %f\n"
            "  theta_falloff = %f\n"
            "]",
            m_position.toString(),
            m_power.toString(),
            m_direction.toString(),
            m_theta * 360 / M_PI,
            m_thetaFalloff * 360 / M_PI
        );
    }
private:
    Point3f m_position;
    Color3f m_power;
    Vector3f m_direction;
    float m_theta;
    float m_cosTheta;
    float m_thetaFalloff;
    float m_cosThetaFalloff;
    float m_invTransitionWidth;
};

NORI_REGISTER_CLASS(SpotLight, "spotlight");
NORI_NAMESPACE_END
