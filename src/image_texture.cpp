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
#include <stb_image.h>

NORI_NAMESPACE_BEGIN

float inverseGammaCorrect(float value) {
    if (value <= 0.04045f)
        return value * 1.f / 12.92f;
    return std::pow((value + 0.055f) * 1.f / 1.055f, 2.4f);
}


class ImageTexture : public Texture<Color3f> {
public:
    ImageTexture(const PropertyList& props) {
        std::string filename = props.getString("filename");
        int bpp;

        uint8_t* rgb_image = stbi_load(filename.c_str(), &m_width, &m_height, &bpp, 3);
        m_map = std::vector<Color3f>(m_width * m_height);

        for (int i = 0; i < m_map.size(); i++) {
            m_map[i] = Color3f(
                inverseGammaCorrect(float(rgb_image[3 * i]) / 255.0),
                inverseGammaCorrect(float(rgb_image[3 * i + 1]) / 255.0),
                inverseGammaCorrect(float(rgb_image[3 * i + 2]) / 255.0)
           );
        }

        m_delta = props.getPoint2("delta", Point2f(0));
        m_scale = props.getVector2("scale", Vector2f(1));
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

    virtual Color3f eval(const Point2f& uv) override {
        Point2i ij = uvmap(uv);
        return m_map[ij.y() * m_width + ij.x()];
    }
private:
    Point2i uvmap(const Point2f& uv) const {
        if (m_wrap == "clamp")
            return Point2i(
                std::max(std::min(int(uv.x() * m_width), m_width - 1), 0),
                std::max(std::min(int((1 - uv.y()) * m_height), m_height - 1), 0)
            );
        if (m_wrap == "repeat")
            return Point2i(
                int((uv.x() - floor(uv.x())) * m_width),
                int(((1 - uv.y()) - floor(uv.y())) * m_height)
            );
    }
protected:
    Point2f m_delta;
    Vector2f m_scale;
    std::vector<Color3f> m_map;
    int m_width;
    int m_height;
    std::string m_wrap = "clamp";
};

NORI_REGISTER_CLASS(ImageTexture, "image_texture")
NORI_NAMESPACE_END
