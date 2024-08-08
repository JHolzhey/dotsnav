using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

public unsafe struct Heightmap {
    int m_Width;
    int m_LengthY;
    NativeArray<float> m_Data;
    float3 m_scale;
    float3 m_offset;

    public Heightmap(int width, int lengthY, NativeArray<float> data, float3 scale, float3 offset) {
        m_Width = width;
        m_LengthY = lengthY;
        m_Data = data;
        m_scale = scale;
        m_offset = offset;
    }

    public int Width() {
        return m_Width;
    }

    public int Length() {
        return m_LengthY;
    }

    public float At(int x, int y) {
        return m_Data[y * m_Width + x];
    }

    public float At(int2 p) {
        return m_Data[p.y * m_Width + p.x];
    }

    public float3 ToWorld(int2 p) {
        return new float3(p.x, At(p.x, p.y), p.y) * m_scale + m_offset;
    }

    public void AutoLevel() {
        float lo = m_Data[0];
        float hi = m_Data[0];
        for (int i = 0; i < m_Data.Length; i++) {
            lo = math.min(lo, m_Data[i]);
            hi = math.max(hi, m_Data[i]);
        }
        if (hi == lo) {
            return;
        }
        for (int i = 0; i < m_Data.Length; i++) {
            m_Data[i] = (m_Data[i] - lo) / (hi - lo);
        }
    }

    public void Invert() {
        for (int i = 0; i < m_Data.Length; i++) {
            m_Data[i] = 1f - m_Data[i];
        }
    }

    public void GammaCurve(float gamma) {
        for (int i = 0; i < m_Data.Length; i++) {
            m_Data[i] = math.pow(m_Data[i], gamma);
        }
    }

    // public void GaussianBlur(int r) {
    //     m_Data = ::GaussianBlur(m_Data, m_Width, m_Height, r);
    // }

    public UnsafeList<float3> Normalmap(float zScale) {
        int w = m_Width - 1;
        int h = m_LengthY - 1;
        UnsafeList<float3> result = new (w * h, Allocator.Temp);
        int i = 0;
        for (int y0 = 0; y0 < h; y0++) {
            int y1 = y0 + 1;
            float yc = y0 + 0.5f;
            for (int x0 = 0; x0 < w; x0++) {
                int x1 = x0 + 1;
                float xc = x0 + 0.5f;
                float z00 = At(x0, y0) * -zScale;
                float z01 = At(x0, y1) * -zScale;
                float z10 = At(x1, y0) * -zScale;
                float z11 = At(x1, y1) * -zScale;
                float zc = (z00 + z01 + z10 + z11) / 4f;
                float3 p00 = new (x0, y0, z00);
                float3 p01 = new (x0, y1, z01);
                float3 p10 = new (x1, y0, z10);
                float3 p11 = new (x1, y1, z11);
                float3 pc = new (xc, yc, zc);
                float3 n0 = MathLib.CalcTriangleNormalCCW(pc, p00, p10);
                float3 n1 = MathLib.CalcTriangleNormalCCW(pc, p10, p11);
                float3 n2 = MathLib.CalcTriangleNormalCCW(pc, p11, p01);
                float3 n3 = MathLib.CalcTriangleNormalCCW(pc, p01, p00);
                result[i] = math.normalize(n0 + n1 + n2 + n3);
                i++;
            }
        }
        return result;
    }

    // public void SaveNormalmap(
    //     std::string &path,
    //     float zScale)
    // {
    //     UnsafeList<float3> nm = Normalmap(zScale);
    //     UnsafeList<uint8_t> data(nm.Length * 3);
    //     int i = 0;
    //     for (float3 n : nm) {
    //         n = (n + 1.f) / 2.f;
    //         data[i++] = uint8_t(n.x * 255);
    //         data[i++] = uint8_t(n.y * 255);
    //         data[i++] = uint8_t(n.z * 255);
    //     }
    //     stbi_write_png(
    //         path.c_str(), m_Width - 1, m_Height - 1, 3,
    //         data.data(), (m_Width - 1) * 3);
    // }

    // public void SaveHillshade(
    //     std::string &path,
    //     float zScale,
    //     float altitude,
    //     float azimuth)
    // {
    //     float3 light = glm::euclidean(glm::vec2(
    //         glm::radians(altitude), glm::radians(-azimuth))).xzy();
    //     UnsafeList<float3> nm = Normalmap(zScale);
    //     UnsafeList<uint8_t> data(nm.Length * 3);
    //     int i = 0;
    //     for (float3 n : nm) {
    //         uint8_t d = glm::clamp(glm::dot(n, light), 0.f, 1.f) * 255;
    //         data[i++] = d;
    //         data[i++] = d;
    //         data[i++] = d;
    //     }
    //     stbi_write_png(
    //         path.c_str(), m_Width - 1, m_Height - 1, 3,
    //         data.data(), (m_Width - 1) * 3);
    // }

    public Pair<int2, float> FindCandidate(
        int2 p0,
        int2 p1,
        int2 p2)
    {
        int edge(int2 a, int2 b, int2 c)
        {
            return (b.x - c.x) * (a.y - c.y) - (b.y - c.y) * (a.x - c.x);
        };

        // triangle bounding box
        int2 min = math.min(math.min(p0, p1), p2);
        int2 max = math.max(math.max(p0, p1), p2);

        // forward differencing variables
        int w00 = edge(p1, p2, min);
        int w01 = edge(p2, p0, min);
        int w02 = edge(p0, p1, min);
        int a01 = p1.y - p0.y;
        int b01 = p0.x - p1.x;
        int a12 = p2.y - p1.y;
        int b12 = p1.x - p2.x;
        int a20 = p0.y - p2.y;
        int b20 = p2.x - p0.x;

        // pre-multiplied z values at vertices
        float a = edge(p0, p1, p2);
        float z0 = At(p0) / a;
        float z1 = At(p1) / a;
        float z2 = At(p2) / a;

        // iterate over pixels in bounding box
        float maxError = 0;
        int2 maxPoint = new int2(0);
        for (int y = min.y; y <= max.y; y++) {
            // compute starting offset
            int dx = 0;
            if (w00 < 0 && a12 != 0) {
                dx = math.max(dx, -w00 / a12);
            }
            if (w01 < 0 && a20 != 0) {
                dx = math.max(dx, -w01 / a20);
            }
            if (w02 < 0 && a01 != 0) {
                dx = math.max(dx, -w02 / a01);
            }

            int w0 = w00 + a12 * dx;
            int w1 = w01 + a20 * dx;
            int w2 = w02 + a01 * dx;

            bool wasInside = false;

            for (int x = min.x + dx; x <= max.x; x++) {
                // check if inside triangle
                if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
                    wasInside = true;

                    // compute z using barycentric coordinates
                    float z = z0 * w0 + z1 * w1 + z2 * w2;
                    float dz = math.abs(z - At(x, y));
                    if (dz > maxError) {
                        maxError = dz;
                        maxPoint = new int2(x, y);
                    }
                } else if (wasInside) {
                    break;
                }

                w0 += a12;
                w1 += a20;
                w2 += a01;
            }

            w00 += b12;
            w01 += b20;
            w02 += b01;
        }

        if (math.all(maxPoint == p0) || math.all(maxPoint == p1) || math.all(maxPoint == p2)) {
            maxError = 0;
        }

        return new Pair<int2, float>(maxPoint, maxError);
    }
};
