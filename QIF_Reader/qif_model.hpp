////////////////////////////////////////////////////////////////////////////////
/// Copyright (c) 2010 - 2026 by Tech Soft 3D, Inc.
///
/// The information contained herein is confidential and proprietary to
/// Tech Soft 3D, Inc., and considered a trade secret as defined under civil
/// and criminal statutes. Tech Soft 3D shall pursue its civil and criminal
/// remedies in the event of unauthorized use or misappropriation of its trade
/// secrets. Use of this information by anyone other than authorized employees
/// of Tech Soft 3D, Inc. is granted only under a written non-disclosure
/// agreement, expressly prescribing the scope and manner of such use.
////////////////////////////////////////////////////////////////////////////////

// qif_model.hpp - QIF 3.0 geometry + topology model (Product/GeometrySet + TopologySet)
// and exact evaluators for the supported curve/surface types. Independent of HOOPS Exchange.
//
// Parameterisations (checked numerically against the six NIST QIF 3.0 models shipped in
// github.com/QualityInformationFramework/qif-community/samples/NISTmodels: every co-edge
// p-curve evaluated on its surface lands on the 3D edge curve):
//   Segment      P(t) = Start + t (End - Start)            ('domain' trims inside [0,1])
//   ArcCircular  P(t) = C + R (cos t X + sin t Y)          X = DirBeg, Y = Normal x X
//   ArcConic     P(t) = C + A cos t X + B sin t Y          (form = ELLIPSE)
//   Nurbs        standard rational B-spline, Knots size = #CPs + Order
//   Aggregate    sub-curves concatenated; sub-curve i covers its own domain length
//   Plane        P(u,v) = Origin + u DirU + v DirV
//   Cylinder     P(u,v) = A + R (cos(su u) X + sin(su u) Y) + (sv v) Z   X = DirBeg, Z = axis
//   Cone         same as cylinder with R(h) = Rbottom + (Rtop - Rbottom) h / Length, h = sv v
//   Sphere       P(u,v) = C + R (cos(lat)(cos(lon) X + sin(lon) Y) + sin(lat) Z), lon = su u, lat = sv v
//   Torus        P(u,v) = A + (Rmaj + r cos(lat))(cos(lon) X + sin(lon) Y) + r sin(lat) Z
//   Nurbs23      control points stored with u varying fastest
// Co-edge p-curves follow the co-edge direction; outer loops are counter-clockwise in UV.
#pragma once

#include "qif_xml.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace qif {

constexpr double kPi = 3.14159265358979323846;

struct Vec3
{
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(double a, double b, double c) : x(a), y(b), z(c) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double k) const { return {x * k, y * k, z * k}; }
    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
    double norm() const { return std::sqrt(dot(*this)); }
    Vec3 unit() const { double n = norm(); return n > 0 ? (*this) * (1.0 / n) : *this; }
};

// ---- number parsing ---------------------------------------------------------------
inline std::vector<double> parseDoubles(const std::string& s)
{
    std::vector<double> v;
    const char* p = s.c_str();
    char* end = nullptr;
    for (;;)
    {
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
        if (!*p) break;
        double d = std::strtod(p, &end);
        if (end == p) break;  // not a number: stop
        v.push_back(d);
        p = end;
    }
    return v;
}
inline bool parseVec(const qifxml::Node* n, Vec3& out, int dim = 3)
{
    if (!n) return false;
    auto d = parseDoubles(n->text);
    if ((int)d.size() < dim) return false;
    out = Vec3(d[0], d[1], dim == 3 ? d[2] : 0.0);
    return true;
}
inline bool parseRange(const char* a, double& lo, double& hi)
{
    if (!a) return false;
    auto d = parseDoubles(a);
    if (d.size() < 2) return false;
    lo = d[0]; hi = d[1];
    return true;
}
inline double attrDouble(const qifxml::Node* n, const char* a, double def)
{
    const char* v = n ? n->attr(a) : nullptr;
    if (!v) return def;
    return std::strtod(v, nullptr);
}

// ---- binary arrays (ArrayBinaryType): base64 of little-endian IEEE doubles ---------------------------------------------
inline bool decodeBase64(const std::string& in, std::vector<unsigned char>& out)
{
    static int8_t map[256];
    static bool init = false;
    if (!init)
    {
        for (int i = 0; i < 256; ++i) map[i] = -1;
        const char* a = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        for (int i = 0; i < 64; ++i) map[(unsigned char)a[i]] = (int8_t)i;
        init = true;
    }
    unsigned buf = 0;
    int bits = 0;
    for (unsigned char ch : in)
    {
        if (ch == '=') break;
        if (map[ch] < 0) { if (ch == ' ' || ch == '\n' || ch == '\r' || ch == '\t') continue; return false; }
        buf = (buf << 6) | (unsigned)map[ch];
        bits += 6;
        if (bits >= 8) { bits -= 8; out.push_back((unsigned char)((buf >> bits) & 0xFF)); }
    }
    return true;
}

// Control points from <CPs> (text) or <CPsBinary count=".." sizeElement="8*dim"> (base64 doubles).
inline bool readPoints(const qifxml::Node* core, int dim, std::vector<double>& out, std::string& err)
{
    if (const qifxml::Node* t = core->child("CPs")) { out = parseDoubles(t->text); return true; }
    const qifxml::Node* b = core->child("CPsBinary");
    if (!b) { err = "no control points"; return false; }
    long count = std::atol(b->attr("count") ? b->attr("count") : "0");
    long size = std::atol(b->attr("sizeElement") ? b->attr("sizeElement") : "0");
    if (size != 8L * dim) { err = "CPsBinary sizeElement " + std::to_string(size) + " for dimension " + std::to_string(dim); return false; }
    std::vector<unsigned char> bytes;
    if (!decodeBase64(b->text, bytes)) { err = "CPsBinary is not valid base64"; return false; }
    if ((long)bytes.size() < count * size) { err = "CPsBinary shorter than count x sizeElement"; return false; }
    out.resize((size_t)count * dim);
    for (size_t i = 0; i < out.size(); ++i)
    {
        uint64_t u = 0;
        for (int k = 7; k >= 0; --k) u = (u << 8) | bytes[i * 8 + (size_t)k];   // little-endian
        double d;
        std::memcpy(&d, &u, sizeof d);
        out[i] = d;
    }
    return true;
}

// ---- NURBS evaluation (de Boor, rational in homogeneous space) -------------------------
inline Vec3 evalNurbs(int degree, const std::vector<double>& knots, const std::vector<Vec3>& cps,
                      const std::vector<double>& w, double t)
{
    const int n = (int)cps.size();
    const int p = degree;
    t = std::max(knots[p], std::min(knots[n], t));
    int k = p;
    while (k < n - 1 && t >= knots[k + 1]) ++k;
    std::vector<double> X(p + 1), Y(p + 1), Z(p + 1), W(p + 1);
    for (int j = 0; j <= p; ++j)
    {
        int i = k - p + j;
        double wi = w.empty() ? 1.0 : w[i];
        X[j] = cps[i].x * wi; Y[j] = cps[i].y * wi; Z[j] = cps[i].z * wi; W[j] = wi;
    }
    for (int r = 1; r <= p; ++r)
        for (int j = p; j >= r; --j)
        {
            int i = k - p + j;
            double den = knots[i + p - r + 1] - knots[i];
            double a = den != 0 ? (t - knots[i]) / den : 0.0;
            X[j] = (1 - a) * X[j - 1] + a * X[j];
            Y[j] = (1 - a) * Y[j - 1] + a * Y[j];
            Z[j] = (1 - a) * Z[j - 1] + a * Z[j];
            W[j] = (1 - a) * W[j - 1] + a * W[j];
        }
    return Vec3(X[p] / W[p], Y[p] / W[p], Z[p] / W[p]);
}

// ---- curves --------------------------------------------------------------------------
enum class CurveKind { Segment, ArcCircular, ArcConicEllipse, Nurbs, Aggregate, Unsupported };

struct Curve
{
    CurveKind kind = CurveKind::Unsupported;
    std::string type;        // QIF element name of the core, e.g. "Segment13Core"
    int dim = 3;
    double d0 = 0, d1 = 1;   // parameter domain
    // segment
    Vec3 p0, p1;
    // arcs
    Vec3 center, xdir, normal;
    double radius = 0, a = 0, b = 0;
    // nurbs
    int degree = 0;
    std::vector<double> knots, weights;
    std::vector<Vec3> cps;
    // aggregate
    std::vector<std::pair<Curve, bool>> subs;  // (sub-curve, turned)
    std::string problem;     // why unsupported

    bool ok() const { return kind != CurveKind::Unsupported; }
    Vec3 ydir() const { return normal.cross(xdir); }

    Vec3 eval(double t) const
    {
        switch (kind)
        {
        case CurveKind::Segment: return p0 + (p1 - p0) * t;
        case CurveKind::ArcCircular: return center + (xdir * std::cos(t) + ydir() * std::sin(t)) * radius;
        case CurveKind::ArcConicEllipse: return center + xdir * (a * std::cos(t)) + ydir() * (b * std::sin(t));
        case CurveKind::Nurbs: return evalNurbs(degree, knots, cps, weights, t);
        case CurveKind::Aggregate:
        {
            double acc = d0;
            for (size_t i = 0; i < subs.size(); ++i)
            {
                const Curve& c = subs[i].first;
                double L = c.d1 - c.d0;
                if (t <= acc + L || i + 1 == subs.size())
                {
                    double f = t - acc;
                    return subs[i].second ? c.eval(c.d1 - f) : c.eval(c.d0 + f);
                }
                acc += L;
            }
            return Vec3();
        }
        default: return Vec3();
        }
    }
};

// Parses a curve "core" element (Segment13Core, Nurbs12Core, ...).
inline Curve parseCurveCore(const qifxml::Node* core, int dim)
{
    Curve c;
    c.dim = dim;
    c.type = core->name;
    bool hasDomain = parseRange(core->attr("domain"), c.d0, c.d1);
    const std::string& t = core->name;
    auto P = [&](const char* n, Vec3& v) { return parseVec(core->child(n), v, dim); };
    if (t == "Segment13Core" || t == "Segment12Core")
    {
        if (!P("StartPoint", c.p0) || !P("EndPoint", c.p1)) { c.problem = "segment without points"; return c; }
        if (!hasDomain) { c.d0 = 0; c.d1 = 1; }
        c.kind = CurveKind::Segment;
    }
    else if (t == "ArcCircular13Core")
    {
        c.radius = std::strtod(core->childText("Radius").c_str(), nullptr);
        if (!P("Center", c.center) || !P("DirBeg", c.xdir) || !P("Normal", c.normal) || !hasDomain) { c.problem = "incomplete arc"; return c; }
        c.xdir = c.xdir.unit(); c.normal = c.normal.unit();
        c.kind = CurveKind::ArcCircular;
    }
    else if (t == "ArcConic13Core")
    {
        const char* form = core->attr("form");
        if (!form || std::string(form) != "ELLIPSE") { c.problem = std::string("conic form ") + (form ? form : "?") + " not supported"; return c; }
        c.a = std::strtod(core->childText("A").c_str(), nullptr);
        c.b = std::strtod(core->childText("B").c_str(), nullptr);
        if (!P("Center", c.center) || !P("DirBeg", c.xdir) || !P("Normal", c.normal) || !hasDomain) { c.problem = "incomplete conic"; return c; }
        c.xdir = c.xdir.unit(); c.normal = c.normal.unit();
        c.kind = CurveKind::ArcConicEllipse;
    }
    else if (t == "Nurbs13Core" || t == "Nurbs12Core")
    {
        int order = std::atoi(core->childText("Order").c_str());
        c.knots = parseDoubles(core->childText("Knots"));
        std::vector<double> cp;
        if (!readPoints(core, dim, cp, c.problem)) return c;
        c.weights = parseDoubles(core->childText("Weights"));
        size_t n = cp.size() / dim;
        if (order < 2 || n < (size_t)order || c.knots.size() != n + order || (!c.weights.empty() && c.weights.size() != n))
        { c.problem = "inconsistent NURBS sizes"; return c; }
        for (size_t i = 0; i < n; ++i) c.cps.emplace_back(cp[i * dim], cp[i * dim + 1], dim == 3 ? cp[i * dim + 2] : 0.0);
        c.degree = order - 1;
        if (!hasDomain) { c.d0 = c.knots[c.degree]; c.d1 = c.knots[n]; }
        c.kind = CurveKind::Nurbs;
    }
    else if (t == "Spline13Core" || t == "Spline12Core")
    {
        // Piecewise polynomial: segment i on [k_i, k_i+1] is sum_j C_ij (t - k_i)^j, Orders[i] rows per segment.
        // Converted exactly to a non-rational B-spline (each segment -> Bezier, joined with knots of multiplicity p).
        std::vector<double> K = parseDoubles(core->childText("Knots"));
        std::vector<double> O = parseDoubles(core->childText("Orders"));
        std::vector<double> C = parseDoubles(core->childText("Coefficients"));
        size_t nseg = K.size() > 1 ? K.size() - 1 : 0;
        if (nseg == 0 || O.size() != nseg) { c.problem = "inconsistent spline knots / orders"; return c; }
        int p = 0; size_t rows = 0;
        for (double o : O) { p = std::max(p, (int)o - 1); rows += (size_t)o; }
        if (p < 1 || C.size() != rows * (size_t)dim) { c.problem = "inconsistent spline coefficients"; return c; }
        auto binom = [](int n, int k) { double r = 1; for (int i = 1; i <= k; ++i) r = r * (n - k + i) / i; return r; };
        size_t row = 0;
        for (size_t sgi = 0; sgi < nseg; ++sgi)
        {
            int ord = (int)O[sgi];
            double h = K[sgi + 1] - K[sgi];
            std::vector<Vec3> a(p + 1);
            for (int j = 0; j < ord; ++j, ++row)
            {
                Vec3 cj(C[row * dim], C[row * dim + 1], dim == 3 ? C[row * dim + 2] : 0.0);
                a[j] = cj * std::pow(h, j);
            }
            for (int i = (sgi == 0 ? 0 : 1); i <= p; ++i)
            {
                Vec3 b;
                for (int j = 0; j <= i; ++j) b = b + a[j] * (binom(i, j) / binom(p, j));
                c.cps.push_back(b);
            }
            int mult = sgi == 0 ? p + 1 : p;
            for (int m = 0; m < mult; ++m) c.knots.push_back(K[sgi]);
        }
        for (int m = 0; m <= p; ++m) c.knots.push_back(K.back());
        c.degree = p;
        if (!hasDomain) { c.d0 = K.front(); c.d1 = K.back(); }
        c.kind = CurveKind::Nurbs;
    }
    else if (t == "Aggregate13Core" || t == "Aggregate12Core")
    {
        const qifxml::Node* sc = core->child("SubCurves");
        if (!sc) { c.problem = "aggregate without SubCurves"; return c; }
        double len = 0;
        for (const auto& s : sc->children)
        {
            if (s->name != "SubCurve" || s->children.empty()) continue;
            Curve sub = parseCurveCore(s->children[0].get(), dim);
            if (!sub.ok()) { c.problem = "aggregate sub-curve: " + sub.problem; return c; }
            len += sub.d1 - sub.d0;
            c.subs.emplace_back(std::move(sub), s->attrBool("turned"));
        }
        if (c.subs.empty()) { c.problem = "empty aggregate"; return c; }
        if (!hasDomain) { c.d0 = 0; c.d1 = len; }
        c.kind = CurveKind::Aggregate;
    }
    else
    {
        c.problem = t + " not supported";
    }
    return c;
}

// ---- surfaces ------------------------------------------------------------------------
enum class SurfKind { Plane, Cylinder, Cone, Sphere, Torus, Nurbs, Revolution, Unsupported };

struct Surface
{
    SurfKind kind = SurfKind::Unsupported;
    std::string type;
    Vec3 origin, x, y, z;      // frame: x = DirU/DirBeg/meridian, z = axis/north pole
    Vec3 dirU, dirV;           // plane
    double su = 1, sv = 1;     // scaleU/scaleV (v sign folded in for turnedV)
    double offV = 0;           // torus offsetV
    double r = 0, r2 = 0, length = 0;   // radius / bottom radius, minor or top radius, cone length
    double planeU0 = 0, planeU1 = 0, planeV0 = 0, planeV1 = 0;
    // nurbs
    int degU = 0, degV = 0, nu = 0, nv = 0;
    std::vector<double> ku, kv, w;   // w indexed [j*nu+i] (u fastest, as in QIF)
    std::vector<Vec3> cps;           // same order
    // revolution: P(u, v) = generatrix(u) rotated by angle v about the axis (origin, z); right-handed
    Curve gen;
    double ang0 = 0, ang1 = 2 * kPi;
    std::string problem;

    bool ok() const { return kind != SurfKind::Unsupported; }

    Vec3 eval(double u, double v) const
    {
        switch (kind)
        {
        case SurfKind::Plane: return origin + dirU * u + dirV * v;
        case SurfKind::Cylinder:
        case SurfKind::Cone:
        {
            double ang = su * u, h = sv * v;
            double R = kind == SurfKind::Cylinder ? r : r + (r2 - r) * h / length;
            return origin + (x * std::cos(ang) + y * std::sin(ang)) * R + z * h;
        }
        case SurfKind::Sphere:
        {
            double lon = su * u, lat = sv * v;
            return origin + ((x * std::cos(lon) + y * std::sin(lon)) * std::cos(lat) + z * std::sin(lat)) * r;
        }
        case SurfKind::Torus:
        {
            double lon = su * u, lat = sv * v + offV;
            return origin + (x * std::cos(lon) + y * std::sin(lon)) * (r + r2 * std::cos(lat)) + z * (r2 * std::sin(lat));
        }
        case SurfKind::Nurbs:
        {
            // evaluate in v the rows at fixed u, using homogeneous coordinates
            std::vector<Vec3> col(nv);
            std::vector<double> colw(nv);
            for (int j = 0; j < nv; ++j)
            {
                std::vector<Vec3> row(nu);
                std::vector<double> rw(nu, 1.0);
                for (int i = 0; i < nu; ++i)
                {
                    double wi = w.empty() ? 1.0 : w[j * nu + i];
                    row[i] = cps[j * nu + i] * wi;
                    rw[i] = wi;
                }
                // homogeneous: evaluate numerator (x,y,z) and denominator separately (non-rational eval)
                Vec3 num = evalNurbs(degU, ku, row, {}, u);
                std::vector<Vec3> wrow(nu);
                for (int i = 0; i < nu; ++i) wrow[i] = Vec3(rw[i], 0, 0);
                double den = evalNurbs(degU, ku, wrow, {}, u).x;
                col[j] = num;
                colw[j] = den;
            }
            Vec3 num = evalNurbs(degV, kv, col, {}, v);
            std::vector<Vec3> wcol(nv);
            for (int j = 0; j < nv; ++j) wcol[j] = Vec3(colw[j], 0, 0);
            double den = evalNurbs(degV, kv, wcol, {}, v).x;
            return num * (1.0 / den);
        }
        case SurfKind::Revolution:
        {
            Vec3 p = gen.eval(u) - origin;
            Vec3 a = z * p.dot(z);
            return origin + a + (p - a) * std::cos(v) + z.cross(p) * std::sin(v);
        }
        default: return Vec3();
        }
    }
};

inline Surface parseSurface(const qifxml::Node* e)
{
    Surface s;
    if (e->children.empty()) { s.problem = "empty surface"; return s; }
    const qifxml::Node* c = e->children[0].get();
    for (const auto& ch : e->children)  // core is the child whose name ends with "Core"
        if (ch->name.size() > 4 && ch->name.compare(ch->name.size() - 4, 4, "Core") == 0) { c = ch.get(); break; }
    s.type = c->name;
    s.su = attrDouble(c, "scaleU", 1.0);
    s.sv = attrDouble(c, "scaleV", 1.0);
    if (c->attrBool("turnedV")) s.sv = -s.sv;
    auto num = [&](const char* n) { return std::strtod(c->childText(n).c_str(), nullptr); };
    auto axis = [&](Vec3& o, Vec3& d) {
        const qifxml::Node* a = c->child("Axis");
        return a && parseVec(a->child("AxisPoint"), o) && parseVec(a->child("Direction"), d);
    };
    // Build an orthonormal frame from z and an x hint (x is projected into the plane normal to z).
    auto frame = [&](const Vec3& zz, const Vec3& xx) {
        s.z = zz.unit();
        s.x = (xx - s.z * xx.dot(s.z)).unit();
        s.y = s.z.cross(s.x);
    };
    if (s.type == "Plane23Core")
    {
        if (!parseVec(c->child("Origin"), s.origin) || !parseVec(c->child("DirU"), s.dirU) || !parseVec(c->child("DirV"), s.dirV))
        { s.problem = "incomplete plane"; return s; }
        parseRange(c->attr("domainU"), s.planeU0, s.planeU1);
        parseRange(c->attr("domainV"), s.planeV0, s.planeV1);
        s.kind = SurfKind::Plane;
    }
    else if (s.type == "Cylinder23Core" || s.type == "Cone23Core")
    {
        Vec3 ap, dir, beg;
        const qifxml::Node* sw = c->child("Sweep");
        if (!axis(ap, dir) || !sw || !parseVec(sw->child("DirBeg"), beg)) { s.problem = "incomplete " + s.type; return s; }
        s.origin = ap;
        frame(dir, beg);
        if (s.type == "Cylinder23Core") { s.r = num("Diameter") / 2; s.kind = SurfKind::Cylinder; }
        else
        {
            s.r = num("DiameterBottom") / 2; s.r2 = num("DiameterTop") / 2; s.length = num("Length");
            if (s.length <= 0) { s.problem = "cone with non-positive Length"; return s; }
            s.kind = SurfKind::Cone;
        }
    }
    else if (s.type == "Sphere23Core")
    {
        const qifxml::Node* sw = c->child("LatitudeLongitudeSweep");
        Vec3 mer, pole;
        if (!parseVec(c->child("Location"), s.origin) || !sw || !parseVec(sw->child("DirMeridianPrime"), mer) || !parseVec(sw->child("DirNorthPole"), pole))
        { s.problem = "incomplete sphere"; return s; }
        frame(pole, mer);
        s.r = num("Diameter") / 2;
        s.kind = SurfKind::Sphere;
    }
    else if (s.type == "Torus23Core")
    {
        Vec3 ap, dir, mer;
        const qifxml::Node* sw = c->child("LatitudeLongitudeSweep");
        if (!axis(ap, dir) || !sw || !parseVec(sw->child("DirMeridianPrime"), mer)) { s.problem = "incomplete torus"; return s; }
        s.origin = ap;
        frame(dir, mer);
        s.r = num("DiameterMajor") / 2;
        s.r2 = num("DiameterMinor") / 2;
        s.offV = attrDouble(c, "offsetV", 0.0);
        s.kind = SurfKind::Torus;
    }
    else if (s.type == "Nurbs23Core")
    {
        int ou = std::atoi(c->childText("OrderU").c_str()), ov = std::atoi(c->childText("OrderV").c_str());
        s.ku = parseDoubles(c->childText("KnotsU"));
        s.kv = parseDoubles(c->childText("KnotsV"));
        std::vector<double> cp;
        if (!readPoints(c, 3, cp, s.problem)) return s;
        s.w = parseDoubles(c->childText("Weights"));
        s.nu = (int)s.ku.size() - ou;
        s.nv = (int)s.kv.size() - ov;
        if (ou < 2 || ov < 2 || s.nu < ou || s.nv < ov || cp.size() != (size_t)s.nu * s.nv * 3 || (!s.w.empty() && s.w.size() != (size_t)s.nu * s.nv))
        { s.problem = "inconsistent NURBS surface sizes"; return s; }
        for (size_t i = 0; i < cp.size() / 3; ++i) s.cps.emplace_back(cp[3 * i], cp[3 * i + 1], cp[3 * i + 2]);
        s.degU = ou - 1; s.degV = ov - 1;
        s.kind = SurfKind::Nurbs;
    }
    else if (s.type == "Revolution23Core")
    {
        Vec3 ap, dir;
        const qifxml::Node* g = c->child("Generatrix");
        if (!axis(ap, dir) || !g || g->children.empty()) { s.problem = "incomplete revolution"; return s; }
        s.origin = ap;
        frame(dir, std::fabs(dir.unit().x) < 0.9 ? Vec3(1, 0, 0) : Vec3(0, 1, 0));
        parseRange(c->attr("angle"), s.ang0, s.ang1);
        s.gen = parseCurveCore(g->children[0].get(), 3);
        if (!s.gen.ok()) { s.problem = "revolution generatrix: " + s.gen.problem; return s; }
        s.kind = SurfKind::Revolution;
    }
    else s.problem = s.type + " not supported";
    return s;
}

// ---- topology ------------------------------------------------------------------------
struct Color  { bool set = false; int r = 0, g = 0, b = 0;
                bool operator==(const Color& o) const { return set == o.set && r == o.r && g == o.g && b == o.b; }
                bool operator<(const Color& o) const { return std::make_tuple(set, r, g, b) < std::make_tuple(o.set, o.r, o.g, o.b); } };
inline Color parseColor(const char* a)
{
    Color c;
    if (!a) return c;
    auto d = parseDoubles(a);
    if (d.size() >= 3) { c.set = true; c.r = (int)d[0]; c.g = (int)d[1]; c.b = (int)d[2]; }
    return c;
}
struct Attribute { std::string type, name, value; };   // type = element name (AttributeStr, AttributeD1, ...)
inline std::vector<Attribute> parseAttributes(const qifxml::Node* owner)
{
    std::vector<Attribute> v;
    const qifxml::Node* a = owner ? owner->child("Attributes") : nullptr;
    if (!a) return v;
    for (const auto& c : a->children)
    {
        Attribute at;
        at.type = c->name;
        if (c->attr("name")) at.name = c->attr("name");
        if (c->attr("value")) at.value = c->attr("value");
        else at.value = c->text;   // AttributeUser etc. carry content
        v.push_back(at);
    }
    return v;
}

struct CoEdge { long edge = 0; bool turned = false; long pcurve = 0; };
struct Loop   { std::vector<CoEdge> coedges; std::string form; };
struct Face   { long surface = 0; std::vector<long> loops; bool turned = false; bool hasOuter = true; std::string label; Color color; bool hidden = false; };
struct Edge   { long curve = 0; long vbeg = 0, vend = 0; Color color; std::string label; };
struct Shell  { std::vector<long> faces; bool closed = false; bool turned = false; };
struct Body   { long id = 0; std::string label, form; bool hidden = false, hasTransform = false; Color color;
                std::vector<long> shells, faces, edges; std::vector<Attribute> attributes; };
struct CoordSystem { long id = 0; std::string label; bool hidden = false; Vec3 origin, x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1}; Color color; };
struct Layer  { long id = 0; std::string label; int index = 0; bool hidden = false; std::vector<long> elements; };

struct Model
{
    std::map<long, Vec3> points;
    std::map<long, Curve> curves3d, curves2d;
    std::map<long, Surface> surfaces;
    std::map<long, long> vertexPoint;   // vertex id -> point id
    std::map<long, Edge> edges;
    std::map<long, Loop> loops;
    std::map<long, Face> faces;
    std::map<long, Shell> shells;
    std::vector<Body> bodies;
    std::vector<CoordSystem> coordSystems;
    std::vector<Layer> layers;
    std::vector<Attribute> partAttributes;     // Part attributes + header fields (ModelNumber, Description, ...)
    // The whole document stays available for the PMI / view layers, with an index of every element that has an id.
    std::unique_ptr<qifxml::Node> doc;
    std::map<long, const qifxml::Node*> byId;
    double unitToMeter = 0.001;   // FileUnits/PrimaryUnits/LinearUnit factor (default mm)
    std::string linearUnitName = "mm (default)";
    std::string partName;
    int partCount = 0, assemblyCount = 0, componentCount = 0;
    std::vector<std::string> notes;   // unsupported/ignored content

    Vec3 vertex(long vid) const
    {
        auto v = vertexPoint.find(vid);
        if (v == vertexPoint.end()) return Vec3();
        auto p = points.find(v->second);
        return p == points.end() ? Vec3() : p->second;
    }
};

inline long idOf(const qifxml::Node* n) { const char* a = n ? n->attr("id") : nullptr; return a ? std::atol(a) : 0; }
inline long refId(const qifxml::Node* n) { return n ? std::atol(n->childText("Id").c_str()) : 0; }
inline std::vector<long> idList(const qifxml::Node* n)
{
    std::vector<long> v;
    if (!n) return v;
    for (const auto& c : n->children) if (c->name == "Id") v.push_back(std::atol(c->text.c_str()));
    return v;
}

// Collects every element with a given name below 'n' (depth-first).
inline void findAll(const qifxml::Node* n, const char* name, std::vector<const qifxml::Node*>& out)
{
    if (n->name == name) out.push_back(n);
    for (const auto& c : n->children) findAll(c.get(), name, out);
}

inline std::string firstCoreChild(const qifxml::Node* e)
{
    return e->children.empty() ? std::string() : e->children[0]->name;
}

inline void indexIds(const qifxml::Node* n, std::map<long, const qifxml::Node*>& idx)
{
    if (const char* a = n->attr("id")) idx[std::atol(a)] = n;
    for (const auto& c : n->children) indexIds(c.get(), idx);
}

inline void buildModel(const qifxml::Node* root, Model& m)
{
    indexIds(root, m.byId);
    if (root->name != "QIFDocument") m.notes.push_back("root element is <" + root->name + ">, expected <QIFDocument>");
    // Units
    if (const auto* fu = root->child("FileUnits"))
        if (const auto* pu = fu->child("PrimaryUnits"))
            if (const auto* lu = pu->child("LinearUnit"))
            {
                m.linearUnitName = lu->childText("UnitName");
                if (const auto* uc = lu->child("UnitConversion"))
                {
                    double f = std::strtod(uc->childText("Factor").c_str(), nullptr);
                    if (f > 0) m.unitToMeter = f;
                }
                else if (m.linearUnitName == "meter") m.unitToMeter = 1.0;
            }
    const qifxml::Node* prod = root->child("Product");
    if (!prod) return;
    // Product structure (only reported; v1 converts the geometry sets of the Product)
    if (const auto* ps = prod->child("PartSet"))
        for (const auto& p : ps->children)
            if (p->name == "Part") { ++m.partCount; if (m.partName.empty() && p->attr("label")) m.partName = p->attr("label"); }
    if (const auto* ps = prod->child("PartSet"))
        for (const auto& p : ps->children)
            if (p->name == "Part")
            {
                m.partAttributes = parseAttributes(p.get());
                auto hdr = [&](const char* label, const std::string& v) { if (!v.empty() && v != " ") m.partAttributes.push_back({"Header", label, v}); };
                if (const auto* h = p->child("Header"))
                {
                    if (const auto* f = h->child("File")) hdr("File", f->childText("Name"));
                    hdr("Description", h->childText("Description"));
                }
                hdr("ModelNumber", p->childText("ModelNumber"));
                hdr("Version", p->childText("Version"));
                break;   // first part only (single-part files)
            }
    if (const auto* cs = prod->child("CoordinateSystemSet"))
        for (const auto& c : cs->children)
        {
            CoordSystem k;
            k.id = idOf(c.get());
            if (c->attr("label")) k.label = c->attr("label");
            k.hidden = c->attrBool("hidden");
            k.color = parseColor(c->attr("color"));
            if (const auto* core = c->child("CoordinateSystemCore"))
            {
                parseVec(core->child("Origin"), k.origin);
                if (const auto* r = core->child("Rotation"))
                { parseVec(r->child("XDirection"), k.x); parseVec(r->child("YDirection"), k.y); parseVec(r->child("ZDirection"), k.z); }
            }
            m.coordSystems.push_back(k);
        }
    if (const auto* ls = prod->child("LayerSet"))
        for (const auto& l : ls->children)
        {
            Layer y;
            y.id = idOf(l.get());
            if (l->attr("label")) y.label = l->attr("label");
            y.index = (int)attrDouble(l.get(), "index", 0);
            y.hidden = l->attrBool("hidden");
            y.elements = idList(l->child("ElementIds"));
            m.layers.push_back(y);
        }
    if (const auto* as = prod->child("AssemblySet")) for (const auto& a : as->children) if (a->name == "Assembly") ++m.assemblyCount;
    if (const auto* cs = prod->child("ComponentSet")) for (const auto& c : cs->children) if (c->name == "Component") ++m.componentCount;

    if (const auto* gs = prod->child("GeometrySet"))
    {
        for (const auto& set : gs->children)
        {
            if (set->name == "PointSet")
            {
                for (const auto& p : set->children)
                {
                    Vec3 v;
                    if (p->name == "Point" && parseVec(p->child("XYZ"), v)) m.points[idOf(p.get())] = v;
                }
            }
            else if (set->name == "Curve13Set" || set->name == "Curve12Set")
            {
                int dim = set->name == "Curve13Set" ? 3 : 2;
                for (const auto& cv : set->children)
                {
                    if (cv->children.empty()) continue;
                    const qifxml::Node* core = nullptr;
                    for (const auto& ch : cv->children)
                        if (ch->name.size() > 4 && ch->name.compare(ch->name.size() - 4, 4, "Core") == 0) { core = ch.get(); break; }
                    if (!core) continue;
                    Curve c = parseCurveCore(core, dim);
                    (dim == 3 ? m.curves3d : m.curves2d)[idOf(cv.get())] = std::move(c);
                }
            }
            else if (set->name == "SurfaceSet")
            {
                for (const auto& sf : set->children) m.surfaces[idOf(sf.get())] = parseSurface(sf.get());
            }
        }
    }
    if (const auto* ts = prod->child("TopologySet"))
    {
        for (const auto& set : ts->children)
        {
            if (set->name == "VertexSet")
                for (const auto& v : set->children) m.vertexPoint[idOf(v.get())] = refId(v->child("Point"));
            else if (set->name == "EdgeSet")
                for (const auto& e : set->children)
                {
                    Edge ed;
                    ed.curve = refId(e->child("Curve"));
                    ed.vbeg = refId(e->child("VertexBeg"));
                    ed.vend = refId(e->child("VertexEnd"));
                    ed.color = parseColor(e->attr("color"));
                    if (e->attr("label")) ed.label = e->attr("label");
                    m.edges[idOf(e.get())] = ed;
                }
            else if (set->name == "LoopSet")
                for (const auto& l : set->children)
                {
                    Loop lp;
                    if (l->attr("form")) lp.form = l->attr("form");
                    if (const auto* ces = l->child("CoEdges"))
                        for (const auto& ce : ces->children)
                        {
                            if (ce->name != "CoEdge") continue;
                            CoEdge c;
                            const auto* eo = ce->child("EdgeOriented");
                            c.edge = refId(eo);
                            c.turned = eo && eo->attrBool("turned");
                            c.pcurve = refId(ce->child("Curve12"));
                            lp.coedges.push_back(c);
                        }
                    m.loops[idOf(l.get())] = lp;
                }
            else if (set->name == "FaceSet")
                for (const auto& f : set->children)
                {
                    Face fc;
                    if (f->name != "Face") { m.notes.push_back(f->name + " (FaceSet) not supported"); continue; }
                    fc.surface = refId(f->child("Surface"));
                    fc.loops = idList(f->child("LoopIds"));
                    fc.turned = f->attrBool("turned");
                    fc.hasOuter = f->attrBool("hasOuter", true);
                    if (f->attr("label")) fc.label = f->attr("label");
                    fc.color = parseColor(f->attr("color"));
                    fc.hidden = f->attrBool("hidden");
                    m.faces[idOf(f.get())] = fc;
                }
            else if (set->name == "ShellSet")
                for (const auto& s : set->children)
                {
                    Shell sh;
                    sh.faces = idList(s->child("FaceIds"));
                    sh.closed = s->attrBool("closed");
                    sh.turned = s->attrBool("turned");
                    m.shells[idOf(s.get())] = sh;
                }
            else if (set->name == "BodySet")
                for (const auto& b : set->children)
                {
                    Body bd;
                    bd.id = idOf(b.get());
                    if (b->attr("label")) bd.label = b->attr("label");
                    if (b->attr("form")) bd.form = b->attr("form");
                    bd.hidden = b->attrBool("hidden");
                    bd.hasTransform = b->child("Transform") != nullptr;
                    bd.shells = idList(b->child("ShellIds"));
                    bd.faces = idList(b->child("FaceIds"));
                    bd.edges = idList(b->child("EdgeIds"));
                    bd.color = parseColor(b->attr("color"));
                    bd.attributes = parseAttributes(b.get());
                    m.bodies.push_back(bd);
                }
            else if (set->name != "PointCloudSet" && set->name != "BodySet")
                m.notes.push_back(set->name + " ignored");
        }
    }
}

} // namespace qif
