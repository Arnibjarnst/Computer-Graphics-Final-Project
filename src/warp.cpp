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

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Vector3f Warp::sampleUniformHemisphere(Sampler *sampler, const Normal3f &pole) {
    // Naive implementation using rejection sampling
    Vector3f v;
    do {
        v.x() = 1.f - 2.f * sampler->next1D();
        v.y() = 1.f - 2.f * sampler->next1D();
        v.z() = 1.f - 2.f * sampler->next1D();
    } while (v.squaredNorm() > 1.f);

    if (v.dot(pole) < 0.f)
        v = -v;
    v /= v.norm();

    return v;
}

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float r = std::sqrt(sample.x());
    float theta = 2 * M_PI * sample.y();
    return Point2f(
        r * std::cos(theta),
        r * std::sin(theta)
    );
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    if (p.squaredNorm() < 1) return INV_PI;
    return 0;
}

Vector3f Warp::squareToUniformSphereCap(const Point2f &sample, float cosThetaMax) {
    float wz = sample.x() * (1 - cosThetaMax) + cosThetaMax;
    float r = std::sqrt(1 - wz * wz);
    float phi = 2 * M_PI * sample.y();
    return Vector3f(
        r * std::cos(phi),
        r * std::sin(phi),
        wz
    );
}

float Warp::squareToUniformSphereCapPdf(const Vector3f &v, float cosThetaMax) {
    if (v.z() >= cosThetaMax) return INV_TWOPI / (1 - cosThetaMax);
    return 0;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float wz = 2 * sample.x() - 1;
    float r = std::sqrt(1 - wz * wz);
    float phi = 2 * M_PI * sample.y();
    return Vector3f(
        r * std::cos(phi),
        r * std::sin(phi),
        wz
    );
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    return INV_FOURPI;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float r = std::sqrt(1 - sample.x() * sample.x());
    float phi = 2 * M_PI * sample.y();
    return Vector3f(
        r * std::cos(phi),
        r * std::sin(phi),
        sample.x()
    );
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    if (v.z() >= 0) return INV_TWOPI;
    return 0;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    float r = std::sqrt(sample.x());
    float theta = 2 * M_PI * sample.y();
    return Vector3f(
        r * std::cos(theta),
        r * std::sin(theta),
        std::sqrt(1 - sample.x())
    );
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    if (v.z() >= 0) return v.z() * INV_PI;
    return 0;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    float z2 = 1 / (1 - alpha * alpha * std::log(1 - sample.x()));
    float r = std::sqrt(1 - z2);
    float phi = 2 * M_PI * sample.y();
    return Vector3f(
        r * std::cos(phi),
        r * std::sin(phi),
        std::sqrt(z2)
    );
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    if (m.z() > 0) {
        float z2 = m.z() * m.z();
        float alpha2 = alpha * alpha;
        return std::exp((z2 - 1) / (alpha2 * z2)) / (M_PI * alpha2 * z2 * z2) * m.z();
    }
    return 0;
}

Vector3f Warp::squareToGTR2(const Point2f& sample, float alpha) {
    float cos_theta_2 = (1 - sample.x()) / (1 + (alpha * alpha - 1) * sample.x());
    float r = std::sqrt(1 - cos_theta_2);
    float phi = 2 * M_PI * sample.y();
    return Vector3f(
        r * std::cos(phi),
        r * std::sin(phi),
        std::sqrt(cos_theta_2)
    );
}

float Warp::squareToGTR2pdf(const Vector3f& v, float alpha) {
    if (v.z() <= 0) return 0.0f;
    float a2 = alpha * alpha;
    float t = 1 + (a2 - 1) * v.z() * v.z();
    return a2 / (M_PI * t * t) * v.z();
}

/// Warp a uniformly distributed square sample to a  Generalized-Trowbridge-Reitz distribution * cosine with eta=2 and the given 'alpha' parameter
Vector3f Warp::squareToGTR2Aniso(const Point2f& sample, float ax, float ay) {
    float r = sqrt(sample.x() / (1 - sample.x()));
    float phi = 2 * M_PI * sample.y();
    return Vector3f(
        r * ax * std::cos(phi),
        r * ay * std::sin(phi),
        1
    ).normalized();
}

/// Probability density of \ref squareToGTR2()
float Warp::squareToGTR2Anisopdf(const Vector3f& v, float ax, float ay) {
    if (v.z() <= 0) return 0.0f;
    return 1 / (M_PI * ax * ay * pow(pow(v.x() / ax, 2) + pow(v.y() / ay, 2) + v.z() * v.z(), 2));
}

Vector3f Warp::squareToUniformTriangle(const Point2f &sample) {
    float su1 = sqrtf(sample.x());
    float u = 1.f - su1, v = sample.y() * su1;
    return Vector3f(u,v,1.f-u-v);
}

NORI_NAMESPACE_END
