//#include <nori/integrator.h>
//#include <nori/scene.h>
//#include <nori/sampler.h>
//#include <nori/bsdf.h>
//#include <nori/warp.h>
//#include <nori/camera.h>
//
//NORI_NAMESPACE_BEGIN
//
//class VolPathMats : public Integrator {
//public:
//
//    VolPathMats(const PropertyList& props) {}
//    /**
//     * \brief Sample the incident radiance along a ray
//     *
//     * \param scene
//     *    A pointer to the underlying scene
//     * \param sampler
//     *    A pointer to a sample generator
//     * \param ray
//     *    The ray in question
//     * \return
//     *    A (usually) unbiased estimate of the radiance in this direction
//     */
//    virtual Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const override {
//        const long START_ROULETTE = -1;
//
//        Color3f L = 0.0f;
//        Color3f t = 1.0f;
//        Ray3f recursive_ray(ray);
//        Intersection its;
//        const Medium *medium = scene->getCamera()->getMedium();
//
//        for (long i = 0; ; i++) {
//
//            if (!scene->rayIntersect(recursive_ray, its) && medium == nullptr) break;
//
//            float dt = medium != nullptr ? medium->fp(sampler->next1D()) : its.t;
//
//            if (dt < its.t) {
//                if (i > START_ROULETTE) {
//                    float successProbability = std::min(0.99f, t.maxCoeff());
//                    if (sampler->next1D() > successProbability) break;
//                    t /= successProbability;
//                }
//
//                MediumQueryRecord mRec(recursive_ray.d);
//                t *= medium->sample(mRec, sampler->next2D());
//                recursive_ray = Ray3f(recursive_ray.o + dt * mRec.wi, mRec.wo);
//            }
//            else {
//                // light emitted from intersection point
//                if (its.mesh->isEmitter())
//                    L += t * its.mesh->getEmitter()->eval(EmitterQueryRecord(recursive_ray.o, its.p, its.shFrame.n));
//
//                if (i > START_ROULETTE) {
//                    float successProbability = std::min(0.99f, t.maxCoeff());
//                    if (sampler->next1D() > successProbability) break;
//                    t /= successProbability;
//                }
//
//                const BSDF* bsdf = its.mesh->getBSDF();
//                BSDFQueryRecord bsdfQuery = BSDFQueryRecord(its.shFrame.toLocal(-recursive_ray.d));
//                bsdfQuery.uv = its.uv;
//                bsdfQuery.p = its.p;
//
//                t *= bsdf->sample(bsdfQuery, sampler->next2D());
//
//                recursive_ray = Ray3f(its.p, its.shFrame.toWorld(bsdfQuery.wo));
//                if (its.mesh->getInterior() || its.mesh->getExterior()) {
//                    medium = its.shFrame.n.dot(recursive_ray.d) > 0 ? its.mesh->getExterior() : its.mesh->getInterior();
//                }
//            }
//        }
//
//        return L;
//    };
//
//    std::string toString() const {
//        return "VolPathMats[]";
//    }
//};
//
//NORI_REGISTER_CLASS(VolPathMats, "vol_path_mats");
//NORI_NAMESPACE_END
