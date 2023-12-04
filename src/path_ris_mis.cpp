#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

/*
RIS only on BSDF value estimation
Density used for importance: g = pdf_mat
Density used for sampling:   p = Warp::squareToCosineHemisphere
Weight for given sample:     g / p = pdf_mat
*/
class PathRISMISIntegrator : public Integrator {
public:
    PathRISMISIntegrator(const PropertyList &props) {}

    struct SampleRecord {
        // BSDF sample
        Color3f bsdf;
        // Outgoing direction
        Vector3f wo;
        // Weight
        float weight;
        // Density used for importance
        float g;
    };

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray0) const {
        const long START_ROULETTE = -1;
        const int M = 20;
        struct SampleRecord samples[M];
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
            Color3f sample_mat_mat;
            float pdf_mat_mat;
            Vector3f newray_wo;

            const BSDF *b1 = its.mesh->getBSDF();
            const Vector3f wi = its.shFrame.toLocal(-ray.d);
            float weight_sum = 0.f;
            if (b1->isDiffuse()) {
                // Sample cosine hemisphere M times
                for (int j = 0; j < M; j++){
                    Vector3f wo1 = Warp::squareToCosineHemisphere(sampler->next2D());
                    samples[j].wo = wo1;
                    BSDFQueryRecord bqr = BSDFQueryRecord(wi, wo1, ESolidAngle);
                    bqr.p = its.p;
                    bqr.uv = its.uv;
                    samples[j].bsdf = b1->eval(bqr) * Frame::cosTheta(wo1);
                    samples[j].g = b1->pdf(bqr);
                    samples[j].weight = b1->pdf(bqr) / Warp::squareToCosineHemispherePdf(wo1);
                    weight_sum += samples[j].weight;
                }
            }
            if (weight_sum > 0.f) {
                float sample = sampler->next1D() * weight_sum;
                int idx = -1;
                while (sample > 0) {
                    sample -= samples[++idx].weight;
                }
                pdf_mat_mat = samples[idx].g;
                sample_mat_mat = samples[idx].bsdf / samples[idx].g * weight_sum / M;
                newray_wo = its.shFrame.toWorld(samples[idx].wo);

            } else {
                // Importance sampling, if b is not diffuse or weight_sum was zero
                BSDFQueryRecord bqr = BSDFQueryRecord(wi);
                bqr.p = its.p;
                bqr.uv = its.uv;
                pdf_mat_mat = b1->pdf(bqr);
                sample_mat_mat = b1->sample(bqr, sampler->next2D());
                newray_wo = its.shFrame.toWorld(bqr.wo);
            }
            
            // Add direct illumination and prepare next w_mat
            if (b1->isDiffuse()){
                // ems
                // sample an emitter
                const Emitter *e = scene->getRandomEmitter(sampler->next1D());
                EmitterQueryRecord eqr2 = EmitterQueryRecord(its.p);
                const Color3f sample_ems_ems = e->sample(eqr2, sampler->next2D());

                const Vector3f wo2 = its.shFrame.toLocal(-ray.d);
                const Vector3f wi2 = its.shFrame.toLocal(eqr2.shadowRay.d);

                if (!scene->rayIntersect(eqr2.shadowRay)) {
                    //sample bsdf
                    BSDFQueryRecord bqr2 = BSDFQueryRecord(wo2, wi2, ESolidAngle);
                    bqr2.uv = its.uv;
                    bqr2.p = its.p;
                    const Color3f sample_mat_ems = b1->eval(bqr2);
                    // compute w_ems
                    const float pdf_ems_ems = e->pdf(eqr2) / emitters_count;
                    const float pdf_mat_ems = b1->pdf(bqr2);
                    if (pdf_ems_ems + pdf_mat_ems > Epsilon) {
                        w_ems = pdf_ems_ems / (pdf_ems_ems + pdf_mat_ems);
                        // add direct illumination
                        li += w_ems * t * sample_mat_ems * sample_ems_ems * Frame::cosTheta(wi2) * emitters_count;
                    }
                }
                //mats
                Intersection sec_its;
                Ray3f sec_ray = Ray3f(its.p, newray_wo);
                if (scene->rayIntersect(sec_ray, sec_its) && sec_its.mesh->isEmitter()){
                    EmitterQueryRecord eqr3 = EmitterQueryRecord(sec_ray.o, sec_its.p, sec_its.shFrame.n); // eqr3 == (next it) eqr1
                    const float pdf_ems_mat = sec_its.mesh->getEmitter()->pdf(eqr3) / emitters_count;
                    w_mat = pdf_mat_mat / (pdf_mat_mat + pdf_ems_mat);
                }
            } else {
                w_ems = 0.0f;
                w_mat = 1.0f;
            }

            // Prepare next iteration
            ray = Ray3f(its.p, newray_wo);
            t *= sample_mat_mat;
        }
        return li;
    }

    std::string toString() const {
        return "PathRISMISIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathRISMISIntegrator, "path_ris_mis");
NORI_NAMESPACE_END