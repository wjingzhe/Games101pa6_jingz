//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

class Bounds3
{
  public:
    Vector3f pMin, pMax; // two points to specify the bounding box
    Bounds3()
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    Vector3f Diagonal() const { return pMax - pMin; }
    int maxExtentDimensionIndex() const
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }
    Bounds3 Intersect(const Bounds3& b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    Vector3f Offset(const Vector3f& p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    bool Inside(const Vector3f& p, const Bounds3& b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }
    inline const Vector3f& operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    inline bool IntersectP(const Ray& ray) const;

    // 他们的
    inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
        const std::array<int, 3>& dirisNeg) const;
};



inline bool Bounds3::IntersectP(const Ray& ray) const
{
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // singleAxisDirIsNeg: ray direction(x,y,z), singleAxisDirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects
    
    //jingz 这里要处理
    //return true;

    /*
    * 点法式平面方程：设平面过点 P₀(x₀, y₀, z₀), 其法向量为n = (A, B, C), 则平面的点法式方程为 A(x−x₀) + B(y−y₀) + C(z−z₀) = 0。
    展开后一般式为 Ax + By + Cz + D = 0, 其中 D = −(Ax₀ + By₀ + Cz₀);而AABB的法线为单轴，其他ABC只有一个值非0
    */
    /*
    * 射线方程：射线通常由起点 O (x₁,y₁,z₁) 和方向向量d=(a,b,c) 来定义
    */
    /*
    * 把射线的xyz各分量等于x1+ta形式代入平面方程，提取t的相关项作为系数和常数项：
    * 可求解参数 t 设 N=Aa+Bb+Cc,M=Ax₁+By₁+Cz₁+D, 则方程变为 Nt+M=0。当 N≠0 时，可解得 t=−M/N=−(Ax₁+By₁+Cz₁+D)/(Aa+Bb+Cc)。
    */
    /*
    * AABB盒中ABC系统有2项为0，D项也剩余−Ax₀这种类似，约去ABC，即
    */
    float t_Min_x = (pMin.x - ray.origin.x) * ray.direction_inv.x;
    float t_Min_y = (pMin.y - ray.origin.y) * ray.direction_inv.y;
    float t_Min_z = (pMin.z - ray.origin.z) * ray.direction_inv.z;

    float t_Max_x = (pMax.x - ray.origin.x) * ray.direction_inv.x;
    float t_Max_y = (pMax.y - ray.origin.y) * ray.direction_inv.y;
    float t_Max_z = (pMax.z - ray.origin.z) * ray.direction_inv.z;

    //if (ray.direction.x < 1e-6)//由于射线反向，所以逻辑大小相反了
    //{
    //    float temp = t_Min_x;
    //    t_Min_x = t_Max_x;
    //    t_Max_x = temp;
    //}
    //
    //if (ray.direction.y < 1e-6)//由于射线反向，所以逻辑大小相反了
    //{
    //    float temp = t_Min_y;
    //    t_Min_y = t_Max_y;
    //    t_Max_y = temp;
    //}

    //if (ray.direction.z < 1e-6)//由于射线反向，所以逻辑大小相反了
    //{
    //    float temp = t_Min_z;
    //    t_Min_z = t_Max_z;
    //    t_Max_z = temp;
    //}

    float tEnter = std::max(t_Min_x, std::max(t_Min_y, t_Min_z));
    float tExit = std::min(t_Max_x, std::min(t_Max_y, t_Max_z));

    if (tExit >= 0 && tEnter <= tExit)
        return true;

    return false;
}

inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
    const std::array<int, 3>& dirIsNeg) const
{
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects

    float t_Min_x = (pMin.x - ray.origin.x) * invDir.x;
    float t_Min_y = (pMin.y - ray.origin.y) * invDir.y;
    float t_Min_z = (pMin.z - ray.origin.z) * invDir.z;
    float t_Max_x = (pMax.x - ray.origin.x) * invDir.x;
    float t_Max_y = (pMax.y - ray.origin.y) * invDir.y;
    float t_Max_z = (pMax.z - ray.origin.z) * invDir.z;

    if (dirIsNeg[0])
    {
        float temp = t_Min_x;
        t_Min_x = t_Max_x;
        t_Max_x = temp;
    }

    if (dirIsNeg[1])
    {
        float temp = t_Min_y;
        t_Min_y = t_Max_y;
        t_Max_y = temp;
    }

    if (dirIsNeg[2])
    {
        float temp = t_Min_z;
        t_Min_z = t_Max_z;
        t_Max_z = temp;
    }

    float tEnter = std::max(t_Min_x, std::max(t_Min_y, t_Min_z));
    float tExit = std::min(t_Max_x, std::min(t_Max_y, t_Max_z));

    if (tExit >= 0 && tEnter <= tExit)
        return true;

    return false;
}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
