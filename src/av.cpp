#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AverageVisibility : public Integrator {
public:
    float length;

    AverageVisibility(const PropertyList& props) {
        length = props.getFloat("length", 1.0f);
    }


    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(1.0f);


        Vector3f sample_direction = Warp().sampleUniformHemisphere(sampler, its.shFrame.n);
        Ray3f sample_ray(its.p, sample_direction);
        if (!scene->rayIntersect(sample_ray, its) || its.t > length)
            return Color3f(1.0f);
        return Color3f(0.0f);
    }

    std::string toString() const {
        return "AverageVisibility[" + std::to_string(length) + "]";
    }
};

NORI_REGISTER_CLASS(AverageVisibility, "av");
NORI_NAMESPACE_END