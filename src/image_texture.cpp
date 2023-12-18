/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Romain Prévost

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

#include <nori/object.h>
#include <nori/texture.h>
#include <nori/shape.h>
#include <stb_image.h>
#include <tbb/tbb.h>

NORI_NAMESPACE_BEGIN

bool DEBUG = false;
const Color3f DEBUG_COLORS[14] = {
    Color3f(47, 79, 79) / 255.0f,
    Color3f(34, 139, 34) / 255.0f,
    Color3f(25, 25, 112) / 255.0f,
    Color3f(139, 0, 0) / 255.0f,
    Color3f(255, 140, 0) / 255.0f,
    Color3f(222, 184, 135) / 255.0f,
    Color3f(0, 255, 0) / 255.0f,
    Color3f(0, 191, 255) / 255.0f,
    Color3f(0, 0, 255) / 255.0f,
    Color3f(255, 0, 255) / 255.0f,
    Color3f(255, 255, 84) / 255.0f,
    Color3f(221, 160, 221) / 255.0f,
    Color3f(255, 20, 147) / 255.0f,
    Color3f(127, 255, 212) / 255.0f
};

enum class WrapMethod {
    Repeat,
    Clamp
};

float inverseGammaCorrect(float value) {
    if (value <= 0.04045f)
        return value * 1.f / 12.92f;
    return std::pow((value + 0.055f) * 1.f / 1.055f, 2.4f);
}

float* convertFloatImage(uint8_t* rgb_image, int width, int height) {
    float* m_map = new float[width * height];
    unsigned char* pixel = rgb_image;
    for (int j = height - 1; j >= 0; j--) { // flip y coordinates
        for (int i = 0; i < width; i++, pixel += 3) {
            m_map[j * width + i] = (pixel[0] + pixel[1] + pixel[2]) / 765.0f;
        }
    }
    return m_map;
}

Color3f* convertColorImage(uint8_t* rgb_image, int width, int height) {
    Color3f* m_map = new Color3f[width * height];
    unsigned char* pixel = rgb_image;
    for (int j = height - 1; j >= 0; j--) { // flip y coordinates
        for (int i = 0; i < width; i++, pixel += 3) {
            m_map[j * width + i] = Color3f(
                inverseGammaCorrect(pixel[0] / 255.0f),
                inverseGammaCorrect(pixel[1] / 255.0f),
                inverseGammaCorrect(pixel[2] / 255.0f)
            );
        }
    }
    return m_map;
}

template <typename T>
class ImageTextureSimple : public Texture<T> {
public:
    ImageTextureSimple(const PropertyList& props) {
        std::string filename = props.getString("filename");
        int bpp;

        uint8_t* rgb_image = stbi_load(filename.c_str(), &m_width, &m_height, &bpp, 3);

        m_map = convertImage(rgb_image);

        std::string wrap = props.getString("wrap", "repeat");
        if (wrap == "repeat") m_wrap = WrapMethod::Repeat;
        else if (wrap == "clamp") m_wrap = WrapMethod::Clamp;

        m_delta = props.getPoint2("delta", Point2f(0));
        m_scale = props.getVector2("scale", Vector2f(1));
    }

    virtual std::string toString() const {
        return tfm::format(
            "ImageTextureSimple[\n"
            "  delta = %s,\n"
            "  scale = %s,\n"
            "]",
            m_delta.toString(),
            m_scale.toString()
        );
    }

    virtual T eval(const Intersection& its) override {
        const Point2f uv_scaled = Point2f(its.uv.x() * m_scale.x(), its.uv.y() * m_scale.y()) + m_delta;
        Point2i ij = uvmap(uv_scaled);
        return m_map[ij.y() * m_width + ij.x()];
    }
private:
    T* convertImage(uint8_t* rgb_image);

    Point2i uvmap(const Point2f& uv) const {
        if (m_wrap == WrapMethod::Clamp)
            return Point2i(
                clamp(int(uv.x() * m_width), 0, m_width - 1),
                clamp(int(uv.y() * m_height), 0, m_height - 1)
            );
        if (m_wrap == WrapMethod::Repeat)
            return Point2i(
                mod(int(floor(uv.x() * m_width)), m_width),
                mod(int(floor(uv.y() * m_height)), m_height)
            );
        return Point2i(0, 0);
    }
protected:
    Point2f m_delta;
    Vector2f m_scale;
    T* m_map;
    int m_width;
    int m_height;
    WrapMethod m_wrap;
};

template <>
Color3f* ImageTextureSimple<Color3f>::convertImage(uint8_t* rgb_image) {
    return convertColorImage(rgb_image, m_width, m_height);
}

template <>
float* ImageTextureSimple<float>::convertImage(uint8_t* rgb_image) {
    return convertFloatImage(rgb_image, m_width, m_height);
}

struct ResampleWeight {
    int firstTexel;
    float weight[4];
};

template <typename T>
class UVArray {
public:
    UVArray(int uRes, int vRes, const T* buf = nullptr) : uRes(uRes), vRes(vRes) {
        buffer = new T[uRes * vRes];
        if (buf) for (int i = 0; i < uRes * vRes; i++) buffer[i] = buf[i];
    }

    int uSize() const {
        return uRes;
    }

    int vSize() const {
        return vRes;
    }

    T &operator()(int u, int v) {
        return buffer[v * uRes + u];
    }
private:
    int uRes;
    int vRes;
    T *buffer;
};


template <typename T>
class MipMap {
public:
    MipMap(const T *img, Point2i &res, WrapMethod wrap) : res(res), wrap(wrap) {
        Point2i newRes((1 << int(ceil(log2(res.x())))), (1 << int(ceil(log2(res.y())))));

        std::unique_ptr<T[]> resampledImage = nullptr;

        if (newRes.x() != res.x() || newRes.y() != res.y()) {
            std::unique_ptr<ResampleWeight[]> sWeights = resampleWeights(res.x(), newRes.x());

            resampledImage.reset(new T[newRes.x() * newRes.y()]);

            tbb::parallel_for(
                tbb::blocked_range<uint32_t>(0u, res.y(), 16), // what is grain size
                [&](const tbb::blocked_range<uint32_t>& range) {
                    for (uint32_t t = range.begin(); t != range.end(); ++t) {
                        for (int s = 0; s < newRes.x(); ++s) {
                            resampledImage[t * newRes.x() + s] = 0.f;
                            for (int j = 0; j < 4; ++j) {
                                int origS = sWeights[s].firstTexel + j;
                                if (wrap == WrapMethod::Repeat)
                                    origS = mod(origS, res.x());
                                else if (wrap == WrapMethod::Clamp)
                                    origS = clamp(origS, 0, res.x() - 1);
                                resampledImage[t * newRes.x() + s] += sWeights[s].weight[j] * img[t * res.x() + origS];
                            }
                        }
                    }
                }
            );

            std::unique_ptr<ResampleWeight[]> tWeights = resampleWeights(res.y(), newRes.y());

            tbb::parallel_for(
                tbb::blocked_range<uint32_t>(0u, newRes.x(), 16), // what is grain size
                [&](const tbb::blocked_range<uint32_t>& range) {
                    T* temp = new T[newRes.y()];
                    for (uint32_t s = range.begin(); s != range.end(); ++s) {
                        for (int t = 0; t < newRes.y(); ++t) {
                            temp[t] = T(0.0f);
                            for (int j = 0; j < 4; ++j) {
                                int offset = tWeights[t].firstTexel + j;
                                if (wrap == WrapMethod::Repeat)
                                    offset = mod(offset, res.y());
                                else if (wrap == WrapMethod::Clamp)
                                    offset = clamp(offset, 0, res.y() - 1);
                                temp[t] += tWeights[t].weight[j] * resampledImage[offset * newRes.x() + s];
                            }
                        }
                        for (int t = 0; t < newRes.y(); ++t)
                            resampledImage[t * newRes.x() + s] = abs(temp[t]);
                    }
                    delete[] temp;
                }
            );

            res = newRes;
        }

        int nLevels = 1 + log2(std::max(res.x(), res.y()));
        pyramid.resize(nLevels);
        
        pyramid[0].reset(
            new UVArray<T>(res.x(), res.y(), resampledImage ? resampledImage.get() : img)
        );

        for (int i = 1; i < nLevels; i++) {
            int uRes = std::max(1, pyramid[i - 1]->uSize() / 2);
            int vRes = std::max(1, pyramid[i - 1]->vSize() / 2);

            pyramid[i].reset(new UVArray<T>(uRes, vRes));

            tbb::parallel_for(
                tbb::blocked_range<uint32_t>(0u, vRes, 16), // what is grain size
                [&](const tbb::blocked_range<uint32_t>& range) {
                    for (uint32_t v = range.begin(); v != range.end(); ++v) {
                        for (int u = 0; u < uRes; u++) {
                            int u2 = 2 * u;
                            int v2 = 2 * v;
                            (*pyramid[i])(u, v) = 0.25f * (
                                eval(i - 1, u2, v2) +
                                eval(i - 1, u2 + 1, v2) +
                                eval(i - 1, u2, v2 + 1) +
                                eval(i - 1, u2 + 1, v2 + 1));
                        }
                    }
                }
            );
        }
    }

    T debug(const Color3f &d) const;

    T Lookup(const Point2f &uv, const Vector2f &duvdx, const Vector2f& duvdy) const {
        float w = std::max(
            std::max(std::abs(duvdx[0]), std::abs(duvdx[1])),
            std::max(std::abs(duvdy[0]), std::abs(duvdy[1]))
        );

        float level = pyramid.size() - 1 + log2(std::max(w, float(1e-8)));

        if (DEBUG) {
            return debug(DEBUG_COLORS[clamp(int(level), 0, pyramid.size())]);
        }

        if (level < 0)
            return triangle(0, uv);
        else if (level >= pyramid.size() - 1) {
            return eval(pyramid.size() - 1, 0, 0);
        }
        else {
            int iLevel = std::floor(level);
            float delta = level - iLevel;
            return (1.0f - delta) * triangle(iLevel, uv) + delta * triangle(iLevel + 1, uv);
        }
        return 0.0f;
    };

private:
    std::unique_ptr<ResampleWeight[]> resampleWeights(int oldRes, int newRes) {
        std::unique_ptr<ResampleWeight[]> weights(new ResampleWeight[newRes]);
        float filterwidth = 2.f;
        for (int i = 0; i < newRes; ++i) {
            float center = (i + .5f) * oldRes / newRes;
            weights[i].firstTexel = std::floor((center - filterwidth) + 0.5f);
            for (int j = 0; j < 4; ++j) {
                float pos = weights[i].firstTexel + j + .5f;
                float x = std::abs((pos - center) / filterwidth);
                if (x < 1e-5f) weights[i].weight[j] = 1;
                else if (x > 1.f) weights[i].weight[j] = 0;
                else {
                    x *= M_PI;
                    float s = std::sin(x * 2) / (x * 2);
                    float lanczos = std::sin(x) / x;
                    weights[i].weight[j] = s * lanczos;
                }
            }
            float normFactor = 1 / (weights[i].weight[0] + weights[i].weight[1] + weights[i].weight[2] + weights[i].weight[3]);
            for (int j = 0; j < 4; ++j) weights[i].weight[j] *= normFactor;
        }
        return weights;
    }

    T eval(int level, int i, int j) const {
        UVArray<T> &l = *pyramid[level];
        if (wrap == WrapMethod::Clamp) {
            i = clamp(i, 0, l.uSize()-1);
            j = clamp(j, 0, l.vSize()-1);
        }
        else if (wrap == WrapMethod::Repeat) {
            i = mod(i, l.uSize());
            j = mod(j, l.vSize());
        }
        else
            throw NoriException("unknown wrap method");

        return l(i, j);
    }

    T triangle(int level, const Point2f& uv) const {
        level = clamp(level, 0, pyramid.size() - 1);
        float u = uv.x() * pyramid[level]->uSize() - 0.5f;
        float v = uv.y() * pyramid[level]->vSize() - 0.5f;
        int u0 = std::floor(u);
        int v0 = std::floor(v);
        float du = u - u0, dv = v - v0;
        return (1 - du) * (1 - dv) * eval(level, u0, v0) +
            (1 - du) * dv * eval(level, u0, v0 + 1) +
            du * (1 - dv) * eval(level, u0 + 1, v0) +
            du * dv * eval(level, u0 + 1, v0 + 1);
    }

    Point2i res;
    std::vector<std::unique_ptr<UVArray<T>>> pyramid;
    const WrapMethod wrap;
};

template <>
Color3f MipMap<Color3f>::debug(const Color3f &d) const {
    return d;
}

template <>
float MipMap<float>::debug(const Color3f &d) const {
    return 1.0f;
}

template <typename T>
class ImageTexture : public Texture<T> {
public:
    ImageTexture(const PropertyList& props) {
        std::string filename = props.getString("filename");
        int bpp;

        uint8_t* rgb_image = stbi_load(filename.c_str(), &m_width, &m_height, &bpp, 3);

        T* m_map = convertImage(rgb_image);

        std::string wrap = props.getString("wrap", "repeat");
        if (wrap == "repeat") m_wrap = WrapMethod::Repeat;
        else if (wrap == "clamp") m_wrap = WrapMethod::Clamp;

        mipmap = new MipMap<T>(m_map, Point2i(m_width, m_height), m_wrap);

        m_delta = props.getPoint2("delta", Point2f(0.0f));
        m_scale = props.getVector2("scale", Vector2f(1.0f));
    }

    virtual T eval(const Intersection &its) override {
        const Point2f uv_scaled = Point2f(its.uv.x() * m_scale.x(), its.uv.y() * m_scale.y()) + m_delta;
        const Vector2f duvdx = Vector2f(its.dudx * m_scale.x(), its.dvdx * m_scale.y());
        const Vector2f duvdy = Vector2f(its.dudy * m_scale.x(), its.dvdy * m_scale.y());

        return mipmap->Lookup(uv_scaled, duvdx, duvdy);
    }

    virtual std::string toString() const {
        return tfm::format(
            "ImageTexture[\n"
            "  delta = %s,\n"
            "  scale = %s,\n"
            "  tex1 = %s,\n"
            "  tex2 = %s,\n"
            "]",
            m_delta.toString(),
            m_scale.toString()
        );
    }
private:
    T* convertImage(uint8_t* rgb_image);
protected:
    Point2f m_delta;
    Vector2f m_scale;
    const MipMap<T>* mipmap;
    int m_width;
    int m_height;
    WrapMethod m_wrap;
};

 template <>
 Color3f* ImageTexture<Color3f>::convertImage(uint8_t* rgb_image) {
     return convertColorImage(rgb_image, m_width, m_height);
 }

 template <>
 float* ImageTexture<float>::convertImage(uint8_t* rgb_image) {
     return convertFloatImage(rgb_image, m_width, m_height);
 }

NORI_REGISTER_TEMPLATED_CLASS(ImageTextureSimple, Color3f, "image_texture_simple_color")
NORI_REGISTER_TEMPLATED_CLASS(ImageTextureSimple, float, "image_texture_simple_float")
NORI_REGISTER_TEMPLATED_CLASS(ImageTexture, Color3f, "image_texture_color")
NORI_REGISTER_TEMPLATED_CLASS(ImageTexture, float, "image_texture_float")
NORI_NAMESPACE_END
