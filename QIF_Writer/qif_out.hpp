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

// qif_out.hpp - small helpers for QIF_Writer: vectors / matrices, exact number formatting, an XML element builder and
// NURBS evaluators used by the writer's self-check. Independent of HOOPS Exchange.
#pragma once

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <type_traits>
#include <vector>

namespace qo {

constexpr double kPi = 3.14159265358979323846;

struct Vec3
{
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(double a, double b, double c) : x(a), y(b), z(c) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double k) const { return {x * k, y * k, z * k}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
    double norm() const { return std::sqrt(dot(*this)); }
    Vec3 unit() const { double n = norm(); return n > 0 ? (*this) * (1.0 / n) : *this; }
};

// Row-vector convention (PRC and the other CodeCraft tools): p' = p * M, translation in row 3.
struct Mat4
{
    double m[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    Mat4 operator*(const Mat4& o) const
    {
        Mat4 r;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
            {
                double s = 0;
                for (int k = 0; k < 4; ++k) s += m[i][k] * o.m[k][j];
                r.m[i][j] = s;
            }
        return r;
    }
    Vec3 point(const Vec3& p) const
    {
        return {p.x * m[0][0] + p.y * m[1][0] + p.z * m[2][0] + m[3][0],
                p.x * m[0][1] + p.y * m[1][1] + p.z * m[2][1] + m[3][1],
                p.x * m[0][2] + p.y * m[1][2] + p.z * m[2][2] + m[3][2]};
    }
    Vec3 dir(const Vec3& v) const
    {
        return {v.x * m[0][0] + v.y * m[1][0] + v.z * m[2][0],
                v.x * m[0][1] + v.y * m[1][1] + v.z * m[2][1],
                v.x * m[0][2] + v.y * m[1][2] + v.z * m[2][2]};
    }
    double det3() const
    {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }
    bool identity(double tol = 1e-14) const
    {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                if (std::fabs(m[i][j] - (i == j ? 1.0 : 0.0)) > tol) return false;
        return true;
    }
    static Mat4 scale(double s) { Mat4 r; r.m[0][0] = r.m[1][1] = r.m[2][2] = s; return r; }
};

// ---- numbers: shortest text that reads back to the same double (exact round trip) ----------------------------------
inline std::string num(double v)
{
    if (v == 0) return "0";   // also folds -0
    char buf[64];
    auto r = std::to_chars(buf, buf + sizeof buf, v);
    return std::string(buf, r.ptr);
}
inline std::string nums(const std::vector<double>& v)
{
    std::string s;
    s.reserve(v.size() * 18);
    for (size_t i = 0; i < v.size(); ++i) { if (i) s += ' '; s += num(v[i]); }
    return s;
}
inline std::string vec(const Vec3& p) { return num(p.x) + " " + num(p.y) + " " + num(p.z); }
inline std::string vec2(double u, double v) { return num(u) + " " + num(v); }

inline std::string esc(const std::string& s)
{
    std::string o;
    o.reserve(s.size());
    for (unsigned char c : s)
    {
        switch (c)
        {
        case '&': o += "&amp;"; break;
        case '<': o += "&lt;"; break;
        case '>': o += "&gt;"; break;
        case '"': o += "&quot;"; break;
        case '\'': o += "&apos;"; break;
        default:
            if (c < 0x20 && c != '\t' && c != '\n' && c != '\r') o += ' ';   // not allowed in XML 1.0
            else o += (char)c;
        }
    }
    return o;
}

// ---- XML element builder: E("Face").a("id", 3).c(E("Surface").c(E("Id").t("4"))).str() ------------------------------
struct E
{
    std::string name, attrs, text;
    std::vector<E> kids;
    explicit E(std::string n) : name(std::move(n)) {}
    E& a(const char* k, const std::string& v) { attrs += std::string(" ") + k + "=\"" + esc(v) + "\""; return *this; }
    E& a(const char* k, const char* v) { return a(k, std::string(v)); }
    template <typename I, typename = std::enable_if_t<std::is_integral<I>::value>>
    E& a(const char* k, I v) { return a(k, std::to_string(v)); }
    E& t(const std::string& s) { text += esc(s); return *this; }
    E& raw(const std::string& s) { text += s; return *this; }      // already escaped / numeric text
    E& c(E e) { kids.push_back(std::move(e)); return *this; }
    E& c(const char* n, const std::string& textValue) { kids.push_back(E(n)); kids.back().t(textValue); return *this; }
    E& ref(const char* n, long id) { E r(n); r.c(E("Id").raw(std::to_string(id))); return c(std::move(r)); }
    E& ids(const char* n, const std::vector<long>& v)
    {
        E r(n);
        r.a("n", v.size());
        for (long id : v) r.c(E("Id").raw(std::to_string(id)));
        return c(std::move(r));
    }
    void write(std::string& out, int depth) const
    {
        const std::string ind((size_t)depth * 2, ' ');
        out += ind + "<" + name + attrs;
        if (kids.empty() && text.empty()) { out += "/>\n"; return; }
        out += ">";
        if (kids.empty()) { out += text + "</" + name + ">\n"; return; }
        out += "\n";
        if (!text.empty()) out += ind + "  " + text + "\n";
        for (const E& k : kids) k.write(out, depth + 1);
        out += ind + "</" + name + ">\n";
    }
};

// ---- NURBS evaluation (de Boor in homogeneous space) for the self-check ---------------------------------------------
inline Vec3 evalNurbs(int p, const std::vector<double>& U, const std::vector<Vec3>& P, const std::vector<double>& W, double t)
{
    const int n = (int)P.size();
    if (n == 0 || (int)U.size() < n + p + 1) return Vec3();
    t = std::max(U[p], std::min(U[n], t));
    int k = p;
    while (k < n - 1 && t >= U[k + 1]) ++k;
    std::vector<double> X(p + 1), Y(p + 1), Z(p + 1), H(p + 1);
    for (int j = 0; j <= p; ++j)
    {
        const int i = k - p + j;
        const double w = W.empty() ? 1.0 : W[i];
        X[j] = P[i].x * w; Y[j] = P[i].y * w; Z[j] = P[i].z * w; H[j] = w;
    }
    for (int r = 1; r <= p; ++r)
        for (int j = p; j >= r; --j)
        {
            const int i = k - p + j;
            const double den = U[i + p - r + 1] - U[i];
            const double a = den != 0 ? (t - U[i]) / den : 0.0;
            X[j] = (1 - a) * X[j - 1] + a * X[j];
            Y[j] = (1 - a) * Y[j - 1] + a * Y[j];
            Z[j] = (1 - a) * Z[j - 1] + a * Z[j];
            H[j] = (1 - a) * H[j - 1] + a * H[j];
        }
    return Vec3(X[p] / H[p], Y[p] / H[p], Z[p] / H[p]);
}

// Surface with control points stored u fastest (QIF order): P[j * nu + i].
inline Vec3 evalNurbsSurface(int pu, int pv, const std::vector<double>& KU, const std::vector<double>& KV, int nu, int nv,
                             const std::vector<Vec3>& P, const std::vector<double>& W, double u, double v)
{
    // Rows in u at each v index (homogeneous), then the column in v.
    std::vector<Vec3> col((size_t)nv);
    std::vector<double> colw((size_t)nv);
    for (int j = 0; j < nv; ++j)
    {
        std::vector<Vec3> rowH((size_t)nu), rowW((size_t)nu);
        for (int i = 0; i < nu; ++i)
        {
            const double w = W.empty() ? 1.0 : W[(size_t)j * nu + i];
            rowH[i] = P[(size_t)j * nu + i] * w;
            rowW[i] = Vec3(w, 0, 0);
        }
        col[j] = evalNurbs(pu, KU, rowH, {}, u);
        colw[j] = evalNurbs(pu, KU, rowW, {}, u).x;
    }
    std::vector<Vec3> wc((size_t)nv);
    for (int j = 0; j < nv; ++j) wc[j] = Vec3(colw[j], 0, 0);
    const Vec3 numr = evalNurbs(pv, KV, col, {}, v);
    const double den = evalNurbs(pv, KV, wc, {}, v).x;
    return den != 0 ? numr * (1.0 / den) : numr;
}

// Symmetric 3x3 eigen decomposition (Jacobi): eigenvalues ascending, eigenvectors as columns of vecs.
inline void eigenSym3(double a[3][3], double vals[3], Vec3 vecs[3])
{
    double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int sweep = 0; sweep < 50; ++sweep)
    {
        const double off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
        if (off < 1e-30) break;
        for (int p = 0; p < 2; ++p)
            for (int q = p + 1; q < 3; ++q)
            {
                if (std::fabs(a[p][q]) < 1e-300) continue;
                const double th = (a[q][q] - a[p][p]) / (2 * a[p][q]);
                const double t = (th >= 0 ? 1.0 : -1.0) / (std::fabs(th) + std::sqrt(th * th + 1));
                const double c = 1 / std::sqrt(t * t + 1), s = t * c;
                for (int k = 0; k < 3; ++k)
                {
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; ++k)
                {
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 3; ++k)
                {
                    const double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
    }
    int idx[3] = {0, 1, 2};
    std::sort(idx, idx + 3, [&](int i, int j) { return a[i][i] < a[j][j]; });
    for (int r = 0; r < 3; ++r)
    {
        vals[r] = a[idx[r]][idx[r]];
        vecs[r] = Vec3(v[0][idx[r]], v[1][idx[r]], v[2][idx[r]]);
    }
}

}  // namespace qo
