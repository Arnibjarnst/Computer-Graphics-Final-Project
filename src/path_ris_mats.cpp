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
class PathRISMATIntegrator : public Integrator {
public:
    PathRISMATIntegrator(const PropertyList &props) {}

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
        const int START_ROULETTE = 2; 
        const int M = 5;
        struct SampleRecord samples[M];

        Color3f li = Color3f(0.0f);
        Color3f t = Color3f(1.0f);
        Ray3f ray = ray0;
        for (int i = 0; ; i++) {
            // Find ray intersection
            Intersection its;
            if (!scene->rayIntersect(ray, its)) {
                break;
            }
            // If an emitter is hit, add contribution
            if (its.mesh->isEmitter()) {
                EmitterQueryRecord eqr = EmitterQueryRecord(ray.o, its.p, its.shFrame.n);
                li += t * its.mesh->getEmitter()->eval(eqr);
            }
            // Russian roulette
            float success_prob = std::min(0.99f, t.maxCoeff());
            if (i > START_ROULETTE && sampler->next1D() > success_prob) break;
            if (i > START_ROULETTE) t /= success_prob;

            // Sample BSDF
            const BSDF *b = its.mesh->getBSDF();
            const Vector3f wi = its.shFrame.toLocal(-ray.d);
            if (b->isDiffuse()) {
                // Sample cosine hemisphere M times
                float weight_sum = 0.f;
                for (int j = 0; j < M; j++){
                    Vector3f wo = Warp::squareToCosineHemisphere(sampler->next2D());
                    samples[j].wo = wo;
                    BSDFQueryRecord bqr = BSDFQueryRecord(wi, wo, ESolidAngle);
                    bqr.p = its.p;
                    bqr.uv = its.uv;
                    samples[j].bsdf = b->eval(bqr) * Frame::cosTheta(wo);
                    samples[j].g = b->pdf(bqr);
                    samples[j].weight = b->pdf(bqr) / Warp::squareToCosineHemispherePdf(wo);
                    weight_sum += samples[j].weight;
                }

                if (weight_sum > 0.f) {
                    float sample = sampler->next1D() * weight_sum;
                    int idx = -1;
                    while (sample > 0) {
                        sample -= samples[++idx].weight;
                    }
                    t *= samples[idx].bsdf / samples[idx].g * weight_sum / M;
                    ray = Ray3f(its.p, its.shFrame.toWorld(samples[idx].wo));
                    continue;
                }
            }
            // Importance sampling, if b is not diffuse or weight_sum was zero
            BSDFQueryRecord bqr = BSDFQueryRecord(wi);
            bqr.p = its.p;
            bqr.uv = its.uv;
            t *=  its.mesh->getBSDF()->sample(bqr, sampler->next2D());
            // Prepare next iteration
            ray = Ray3f(its.p, its.shFrame.toWorld(bqr.wo));
            
        }
        return li;
    }
    /*Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        // Find the surface that is visible in the requested direction 
        Intersection its1, its2;
        Ray3f ray1 = ray;
        Ray3f ray2;

        if (!scene->rayIntersect(ray1, its1))
            return Color3f(0.0f);
        
        Color3f le;
        // Light emitted at intersection
        if (its1.mesh->isEmitter()){
            EmitterQueryRecord eqr = EmitterQueryRecord(ray1.o, its1.p, its1.shFrame.n);
            le = its1.mesh->getEmitter()->eval(eqr);
        }

        Color3f li = Color3f(1.0f);
        for (int i = 0; ; i++) {
            BSDFQueryRecord bqr = BSDFQueryRecord(its1.shFrame.toLocal(-ray1.d));
            const BSDF *microfacet = its1.mesh->getBSDF();
            const Color3f sample = microfacet->sample(bqr, sampler->next2D());

            // Generate secondary ray and find the intersection
            ray2 = Ray3f(its1.p, its1.shFrame.toWorld(bqr.wo));
            // No intersection
            if (!scene->rayIntersect(ray2, its2)) 
                break;
            // Intersection with an emitter
            if (its2.mesh->isEmitter()) {
                EmitterQueryRecord eqr = EmitterQueryRecord(its1.p, its2.p, its2.shFrame.n);
                const Color3f radiance = its2.mesh->getEmitter()->eval(eqr);
                li *= sample * radiance;   
                break;  
            }
            // Killed by russian roulette
            if (i > 2 && sample.x() < sampler->next1D() || i > 50){
                li = Color3f(0.0f);
                break;
            } 
            // New iteration
            li *= sample;
            its1 = its2;
            ray1 = ray2;
              
        }
        return li + le;
    }*/

    std::string toString() const {
        return "PathRISMATIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathRISMATIntegrator, "path_ris_mats");
NORI_NAMESPACE_END