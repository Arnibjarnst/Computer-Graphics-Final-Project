/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>
#include <nori/photon.h>

NORI_NAMESPACE_BEGIN

class PhotonMapper : public Integrator {
public:
    /// Photon map data structure
    typedef PointKDTree<Photon> PhotonMap;

    PhotonMapper(const PropertyList &props) {
        /* Lookup parameters */
        m_photonCount  = props.getInteger("photonCount", 1000000);
        m_photonRadius = props.getFloat("photonRadius", 0.0f /* Default: automatic */);
        m_emittedCount = 0;
    }

    virtual void preprocess(const Scene *scene) override {
        cout << "Gathering " << m_photonCount << " photons .. ";
        cout.flush();

        /* Create a sample generator for the preprocess step */
        Sampler *sampler = static_cast<Sampler *>(
            NoriObjectFactory::createInstance("independent", PropertyList()));

        /* Allocate memory for the photon map */
        m_photonMap = std::unique_ptr<PhotonMap>(new PhotonMap());
        m_photonMap->reserve(m_photonCount);

		/* Estimate a default photon radius */
		if (m_photonRadius == 0)
			m_photonRadius = scene->getBoundingBox().getExtents().norm() / 500.0f;

	

		/* How to add a photon?
		 * m_photonMap->push_back(Photon(
		 *	Point3f(0, 0, 0),  // Position
		 *	Vector3f(0, 0, 1), // Direction
		 *	Color3f(1, 2, 3)   // Power
		 * ));
		 */

		// put your code to trace photons here
        while (m_photonMap->size() < m_photonCount) {
            Ray3f ray(Point3f(0.0f), Vector3f(0,0,1));
            Color3f power = scene->getRandomEmitter(sampler->next1D())->samplePhoton(ray, sampler->next2D(), sampler->next2D()) * scene->getLights().size();
            m_emittedCount++;
            Intersection its;
            while (scene->rayIntersect(ray, its)) {
                const BSDF* bsdf = its.mesh->getBSDF();
                if (bsdf->isDiffuse()) {
                    m_photonMap->push_back(Photon(
                        its.p,
                        -ray.d,
                        power
                    ));
                    if (m_photonMap->size() == m_photonCount) break;
                }


                BSDFQueryRecord bsdfQuery = BSDFQueryRecord(its.shFrame.toLocal(-ray.d));
                bsdfQuery.uv = its.uv;
                bsdfQuery.p = its.p;

                Color3f bsdfValue = bsdf->sample(bsdfQuery, sampler->next2D());

                const float successProbability = std::min(1.0f, bsdfValue.maxCoeff());
                if (sampler->next1D() > successProbability) break;
                power *= bsdfValue / successProbability;

                ray = Ray3f(its.p, its.shFrame.toWorld(bsdfQuery.wo));
            }
        }

		/* Build the photon map */
        m_photonMap->build();
    }

    virtual Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const override {
    	
		/* How to find photons?
		 * std::vector<uint32_t> results;
		 * m_photonMap->search(Point3f(0, 0, 0), // lookup position
		 *                     m_photonRadius,   // search radius
		 *                     results);
		 *
		 * for (uint32_t i : results) {
		 *    const Photon &photon = (*m_photonMap)[i];
		 *    cout << "Found photon!" << endl;
		 *    cout << " Position  : " << photon.getPosition().toString() << endl;
		 *    cout << " Power     : " << photon.getPower().toString() << endl;
		 *    cout << " Direction : " << photon.getDirection().toString() << endl;
		 * }
		 */

		// put your code for path tracing with photon gathering here

        Color3f L = 0.0f;
        Color3f t = 1.0f;
        Ray3f recursive_ray(_ray);
        Intersection its;
        while (scene->rayIntersect(recursive_ray, its)) {

            // light emitted from intersection point
            if (its.mesh->isEmitter())
                L += t * its.mesh->getEmitter()->eval(EmitterQueryRecord(recursive_ray.o, its.p, its.shFrame.n));

            const BSDF* bsdf = its.mesh->getBSDF();

            Vector3f wi = its.shFrame.toLocal(-recursive_ray.d);

            if (bsdf->isDiffuse()) {
                std::vector<uint32_t> results;
                m_photonMap->search(its.p,
                    m_photonRadius,
                    results);

                Color3f photonPower = 0.0f;
                for (uint32_t i : results) {
                    const Photon& photon = (*m_photonMap)[i];

                    BSDFQueryRecord photonBsdfQuery(
                        wi,
                        its.shFrame.toLocal(photon.getDirection()),
                        ESolidAngle
                    );
                    photonBsdfQuery.p = its.p;
                    photonBsdfQuery.uv = its.uv;
                    photonPower += photon.getPower() * bsdf->eval(photonBsdfQuery);
                }
                L += t * photonPower * INV_PI / (m_photonRadius * m_photonRadius * m_emittedCount);
                break;
            }

            float successProbability = std::min(0.99f, t.maxCoeff());
            if (sampler->next1D() > successProbability) break;
            t /= successProbability;

            BSDFQueryRecord bsdfQuery = BSDFQueryRecord(wi);
            bsdfQuery.uv = its.uv;
            bsdfQuery.p = its.p;

            t *= bsdf->sample(bsdfQuery, sampler->next2D());

            recursive_ray = Ray3f(its.p, its.shFrame.toWorld(bsdfQuery.wo));
        }

        return L;
    }

    virtual std::string toString() const override {
        return tfm::format(
            "PhotonMapper[\n"
            "  photonCount = %i,\n"
            "  photonRadius = %f\n"
            "]",
            m_photonCount,
            m_photonRadius
        );
    }
private:
    /* 
     * Important: m_photonCount is the total number of photons deposited in the photon map,
     * NOT the number of emitted photons. You will need to keep track of those yourself.
     */ 
    int m_photonCount;
    int m_emittedCount;
    float m_photonRadius;
    std::unique_ptr<PhotonMap> m_photonMap;
};

NORI_REGISTER_CLASS(PhotonMapper, "photonmapper");
NORI_NAMESPACE_END
