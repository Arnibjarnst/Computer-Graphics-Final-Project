#include <nori/medium.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

class HenyeyGreenstein : public PhaseFunction {
public:
    HenyeyGreenstein(const PropertyList& propList) {
        m_g = propList.getFloat("g", 0.0f);
    }

    // TODO: Make more efficient
    virtual Vector3f sample(Vector3f &wi, const Point2f& sample) const override {
        float cosTheta;
        if (std::abs(m_g) < 1e-3)
            cosTheta = 1 - 2 * sample.x();
        else {
            float sqrTerm = (1 - m_g * m_g) /
                (1 - m_g + 2 * m_g * sample.x());
            cosTheta = (1 + m_g * m_g - sqrTerm * sqrTerm) / (2 * m_g);
        }
        float sinTheta = std::sqrt(std::max(0.0f, 1 - cosTheta * cosTheta));
        float phi = 2 * M_PI * sample.y();
        float sinPhi, cosPhi;
        sincosf(phi, &sinPhi, &cosPhi);

        return Frame(wi).toWorld(Vector3f(
            sinTheta * cosPhi,
            sinTheta * sinPhi,
            cosTheta
        ));
	}

    virtual std::string toString() const override {
        return tfm::format(
            "Henyey-Greenstein[\n"
            "  g = %f,\n"
            "]",
            m_g);
    }

private:
    float m_g;
};

NORI_REGISTER_CLASS(HenyeyGreenstein, "henyeyGreenstein");
NORI_NAMESPACE_END
