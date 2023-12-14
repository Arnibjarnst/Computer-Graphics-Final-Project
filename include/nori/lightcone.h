#if !defined(__NORI_LIGHTCONE_H)
#define __NORI_LIGHTCONE_H

#include <nori/object.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

struct LightCone {
    Vector3f axis;
    float theta_e;
    float theta_o;

    //// Dummy constructor, returns an invalid lightcone
    struct LightCone () 
        : theta_e(-1.f), theta_o(-1.f) {}

    bool isValid() const {
        return theta_o >= 0.f && theta_o <= M_PI && theta_e >= 0.f && theta_e <= M_PI_2;
    }

    float getOrientationCost() const {
        float theta_w = std::min(theta_e + theta_o, M_PI);
        float cos_o = std::cos(theta_o);
        float sin_o = std::cos(theta_o);
        return 2 * M_PI * (1 - cos_o) + M_PI_2 * (2 * theta_w * sin_o - std::cos(theta_o - 2 * theta_w) - 2 * theta_o * sin_o + cos_o);
    }

    struct LightCone merge(LightCone other) const {
        // Either of the operands is invalid
        if (!isValid()) {
            LightCone res;
            res.axis = other.axis;
            res.theta_e = other.theta_e;
            res.theta_o = other.theta_o;
            return res;
        }
        if (!other.isValid()) {
            LightCone res;
            res.axis = axis;
            res.theta_e = theta_e;
            res.theta_o = theta_o;
            return res;
        }
        // if theta_o(b) > theta_o(a), swap(a,b) 
        if (other.theta_o > theta_o) return other.merge(*this);

        LightCone res;
        float theta_d = std::acos(axis.dot(other.axis));
        res.theta_e = std::max(theta_e, other.theta_e);

        if (std::min(theta_d + other.theta_o, M_PI) <= theta_o) {
            res.axis = axis;
            res.theta_o = theta_o;
            return res;
        } 

        res.theta_o = (theta_o + theta_d + other.theta_o) / 2;
        if (res.theta_o >= M_PI) {
            res.axis = axis;
            res.theta_o = M_PI;
            return res;
        }
        float theta_r = res.theta_o - theta_o;
        Vector3f u = axis.cross(other.axis);
        Eigen::Matrix3f rotation = Eigen::AngleAxis(theta_r, u).toRotationMatrix();
        res.axis = rotation * axis;
        return res;
    }

    void expandBy(LightCone other) {
        axis = merge(other).axis;
        theta_o = merge(other).theta_o;
        theta_e = merge(other).theta_e;
    }
};

NORI_NAMESPACE_END
#endif