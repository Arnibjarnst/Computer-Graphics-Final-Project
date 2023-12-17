#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class PathMISIntegrator : public Integrator {
public:
    PathMISIntegrator(const PropertyList &props) {}

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray0) const {
        const long START_ROULETTE = -1;
        const auto emitters_count = scene->getLights().size();

        Color3f li = Color3f(0.0f);
        Color3f t = Color3f(1.0f);
        Ray3f ray = ray0;

        float w_ems = 0.0f;
        float w_mat = 1.0f;

        for (long i = 0; ; i++) {
            // Find ray intersection
            Intersection its;
            if (!scene->rayIntersect(ray, its)) {
                break;
            }
            // If is an emitter, add contribution
            if (its.mesh->isEmitter()) {
                EmitterQueryRecord eqr1 = EmitterQueryRecord(ray.o, its.p, its.shFrame.n);
                li += w_mat * t * its.mesh->getEmitter()->eval(eqr1);
            }
            // Russian roulette
            float success_prob = std::min(0.99f, t.maxCoeff());
            if (i > START_ROULETTE && sampler->next1D() > success_prob) break;
            if (i > START_ROULETTE) t /= success_prob;

            // Sample BSDF
            const BSDF *b1 = its.mesh->getBSDF();
            BSDFQueryRecord bqr1 = BSDFQueryRecord(its.shFrame.toLocal(-ray.d));
            bqr1.uv = its.uv;
            bqr1.p = its.p;
            const Color3f sample_mat_mat = b1->sample(bqr1, sampler->next2D());
            
            // Add direct illumination and prepare next w_mat
            if (bqr1.measure == ESolidAngle){
                // ems
                // sample an emitter
                LightBVHQueryRecord lqr(its.p, its.shFrame.n);
                const Emitter *e = scene->getRandomEmitter(lqr);
                EmitterQueryRecord eqr2 = EmitterQueryRecord(its.p);
                const Color3f sample_ems_ems = e->sample(eqr2, sampler->next2D());

                const Vector3f wo = its.shFrame.toLocal(-ray.d);
                const Vector3f wi = its.shFrame.toLocal(eqr2.shadowRay.d);

                if (!scene->rayIntersect(eqr2.shadowRay)) {
                    //sample bsdf
                    BSDFQueryRecord bqr2 = BSDFQueryRecord(wo, wi, ESolidAngle);
                    bqr2.uv = its.uv;
                    bqr2.p = its.p;
                    const Color3f sample_mat_ems = b1->eval(bqr2);
                    // compute w_ems
                    const float pdf_ems_ems = e->pdf(eqr2) * lqr.pdf;
                    const float pdf_mat_ems = b1->pdf(bqr2);
                    if (pdf_ems_ems + pdf_mat_ems > Epsilon) {
                        w_ems = pdf_ems_ems / (pdf_ems_ems + pdf_mat_ems);
                        // add direct illumination
                        li += w_ems * t * sample_mat_ems * sample_ems_ems * Frame::cosTheta(wi) / lqr.pdf;
                    }
                }
                //mats
                Intersection sec_its;
                Ray3f sec_ray = Ray3f(its.p, its.shFrame.toWorld(bqr1.wo));
                if (scene->rayIntersect(sec_ray, sec_its) && sec_its.mesh->isEmitter()){
                    EmitterQueryRecord eqr3 = EmitterQueryRecord(sec_ray.o, sec_its.p, sec_its.shFrame.n); // eqr3 == (next it) eqr1
                    LightBVHQueryRecord lqrmat(its.p, its.shFrame.n);
                    const Emitter *e = sec_its.mesh->getEmitter();
                    const float pdf_ems_mat = e->pdf(eqr3) * scene->getRandomEmitterPdf(e, lqrmat);
                    const float pdf_mat_mat = b1->pdf(bqr1);
                    w_mat = pdf_mat_mat / (pdf_mat_mat + pdf_ems_mat);
                }
            } else {
                w_ems = 0.0f;
                w_mat = 1.0f;
            }

            // Prepare next iteration
            ray = Ray3f(its.p, its.shFrame.toWorld(bqr1.wo));
            t *= sample_mat_mat;
        }
        return li;
    }

    std::string toString() const {
        return "PathMISIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathMISIntegrator, "path_mis");
NORI_NAMESPACE_END