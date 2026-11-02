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

////////////////////////////////////////////////////////////////////////////////
/// \example QIF_Reader
/// QIF 3.0 to PRC converter using the HOOPS Exchange creation API.
///
/// 1. read the QIF file (built-in reader: qif_xml.hpp, qif_model.hpp)
/// 2. create the PRC model entity by entity (geometry, topology, structure, appearance, PMI)
/// 3. A3DAsmModelFileExportToPrcFile, then re-import and verify
////////////////////////////////////////////////////////////////////////////////

/***********************************************************************************************************************
 * QIF_Reader - converts the exact B-rep geometry of a QIF 3.0 file to PRC with the HOOPS Exchange public API.
 *
 *   QIF is read by a small built-in reader (qif_xml.hpp, qif_model.hpp); the PRC model is built entity by entity
 *   (surfaces, curves, vertices, edges, co-edges, loops, faces, shells, connex, B-rep, part, product occurrence,
 *   model file) and written with A3DAsmModelFileExportToPrcFile.
 *
 * Based on   : samples/prc_model/CreatePRCBrepWithGeometry (B-rep creation pattern),
 *              samples/hello_world/ImportExport (load / export / status reporting)
 * Grounded in: include/A3DSDKGeometrySrf.h, A3DSDKGeometryCrv.h, A3DSDKTopology.h, A3DSDKRepItems.h,
 *              A3DSDKStructure.h, A3DSDKAdvancedTools.h (A3DSurfEvaluate, A3DCrvEvaluate), A3DSDKTools.h,
 *              docs: A3DUVParameterizationData (reparameterisation formula), "QIF Reader" (native beta reader),
 *              QIF 3.0 schema (qif-community/bindings/CPP-Kramer/schema, *.xsdOrig files)
 *
 * Usage: QIF_Reader <input.qif> <output.prc> [options]   (run without arguments for the option list)
 * Exit codes: 0 ok | 1 usage | 2 input/parse/SDK error | 3 unsupported (output extension, nothing convertible)
 *             4 export or re-import failed | 5 verification failed (--max-dev / --compare-native)
 ***********************************************************************************************************************/

#ifndef NOMINMAX
#define NOMINMAX   // keep <windows.h> (pulled in by the SDK loader) from defining min/max macros
#endif
#define INITIALIZE_A3D_API
#include <A3DSDKIncludes.h>
#include <hoops_license.h>

#include "qif_model.hpp"

#include <cctype>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <filesystem>
#include <functional>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using qif::Vec3;

enum ExitCode { kOk = 0, kUsage = 1, kInput = 2, kUnsupported = 3, kExport = 4, kVerify = 5 };

// ---------------------------------------------------------------------------------------------------------------------
// Options and reporting
// ---------------------------------------------------------------------------------------------------------------------
struct Options
{
    std::string input, output;
    bool force = false;
    bool noPmi = false;           // geometry, structure and appearance only
    bool skipHidden = false;      // drop bodies flagged hidden="true" (default: converted, shown hidden)
    bool quiet = false;
    double maxDev = -1;           // fail (exit 5) if a geometry check deviates more than this (model units)
    bool compareNative = false;   // compare with HOOPS Exchange's own (beta) QIF reader
    double nativeTol = 1e-3;      // relative tolerance for --compare-native
};

static bool g_quiet = false;
#define LOG(...) do { if (!g_quiet) printf(__VA_ARGS__); } while (0)

// Signatures as declared by A3DCallbackReportMessage / Warning / Error in the SDK headers.
static A3DInt32 SdkMsg(A3DUTF8Char* m) { if (m && !g_quiet) printf("  [HOOPS] %s", m); return 0; }
static A3DInt32 SdkWarn(A3DUTF8Char* code, A3DUTF8Char* m) { printf("  [HOOPS warning %s] %s", code ? code : "", m ? m : ""); return 0; }
static A3DInt32 SdkErr(A3DUTF8Char* code, A3DUTF8Char* m) { printf("  [HOOPS error %s] %s", code ? code : "", m ? m : ""); return 0; }

static void PrintStatus(const char* what, A3DStatus st)
{
    const A3DUTF8Char* msg = A3DMiscGetErrorMsg(st);
    printf("ERROR: %s failed: status %d (%s)\n", what, (int)st, msg ? msg : "?");
}

static void PrintUsage()
{
    printf("Usage: QIF_Reader <input.qif> <output.prc> [options]\n"
           "Converts the exact B-rep geometry of a QIF 3.0 file to PRC.\n"
           "Options:\n"
           "  --force               overwrite <output.prc> if it exists\n"
           "  --no-pmi              do not convert PMI and views\n"
           "  --dump-pmi            print the semantic PMI read back from the written PRC\n"
           "  --no-semantic         graphical PMI only (no semantic GD&T, dimensions, datums, notes)\n"
           "  --skip-hidden         do not convert bodies marked hidden=\"true\" (default: converted as hidden)\n"
           "  --max-dev <d>         exit 5 if a PRC curve/surface deviates from the QIF definition by more than d\n"
           "                        (model units; checked with A3DSurfEvaluate / A3DCrvEvaluate)\n"
           "  --compare-native      also load the QIF with HOOPS Exchange's own (beta) QIF reader and compare\n"
           "                        face count, area and volume; exit 5 if they differ more than --native-tol\n"
           "  --native-tol <r>      relative tolerance for --compare-native (default 1e-3)\n"
           "  --quiet               print only warnings, errors and the summary\n"
           "Exit codes: 0 ok, 1 usage, 2 input/parse error, 3 unsupported, 4 export failed, 5 verification failed\n");
}

// Geometry checks: PRC entity evaluated through the SDK vs. the QIF definition.
struct DevStat { double maxDev = 0; long n = 0; };
struct Report
{
    std::map<std::string, long> created;            // PRC entities created, by QIF type
    std::map<std::string, DevStat> dev;             // deviation by QIF type
    std::map<std::string, long> skipped;            // reasons -> count
    long faces = 0, facesSkipped = 0, bodies = 0, bodiesSkipped = 0, edgesNo3d = 0, tolerantEdges = 0;
    void skip(const std::string& why) { ++skipped[why]; }
    void devAdd(const std::string& k, double d) { auto& s = dev[k]; s.maxDev = std::max(s.maxDev, d); ++s.n; }
};

// ---------------------------------------------------------------------------------------------------------------------
// Small A3D helpers
// ---------------------------------------------------------------------------------------------------------------------
static A3DVector3dData V3(const Vec3& v)
{
    A3DVector3dData d;
    A3D_INITIALIZE_DATA(A3DVector3dData, d);
    d.m_dX = v.x; d.m_dY = v.y; d.m_dZ = v.z;
    return d;
}
static Vec3 FromA3D(const A3DVector3dData& d) { return Vec3(d.m_dX, d.m_dY, d.m_dZ); }

static void SetTrsf(A3DMiscCartesianTransformationData& t, const Vec3& origin, const Vec3& x, const Vec3& y)
{
    t.m_sOrigin = V3(origin);
    t.m_sXVector = V3(x);
    t.m_sYVector = V3(y);
    t.m_sScale = V3(Vec3(1, 1, 1));
    t.m_ucBehaviour = kA3DTransformationTranslate | kA3DTransformationRotate;
}

// For periodic angular parameters: offset (multiple of 2 pi) that brings the smallest implicit value into [0, 2pi).
static double PeriodicShift(double minImplicit)
{
    if (minImplicit >= -1e-9) return 0.0;
    return 2 * qif::kPi * std::ceil(-minImplicit / (2 * qif::kPi) - 1e-12);
}

struct UVBox
{
    double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
    bool valid() const { return u0 <= u1 && v0 <= v1; }
    void add(double u, double v) { u0 = std::min(u0, u); u1 = std::max(u1, u); v0 = std::min(v0, v); v1 = std::max(v1, v); }
};

// The PRC cone "semi angle" form: v is either the height along the axis or the slant length along the generatrix.
// Detected once at start-up from the SDK itself (see DetectConeConvention).
static bool g_coneVIsSlant = false;

// ---------------------------------------------------------------------------------------------------------------------
// Geometry: QIF -> PRC
// ---------------------------------------------------------------------------------------------------------------------
static A3DCrvNurbs* MakeNurbsCurve(int degree, const std::vector<double>& knots, const std::vector<Vec3>& cps,
                                   const std::vector<double>& weights, bool is2D, A3DEBSplineCurveForm form)
{
    std::vector<A3DVector3dData> pts;
    for (const auto& p : cps) pts.push_back(V3(is2D ? Vec3(p.x, p.y, 0) : p));
    bool rational = false;
    for (double w : weights) if (std::fabs(w - 1.0) > 1e-14) rational = true;
    std::vector<double> k(knots), w(weights);
    A3DCrvNurbsData d = A3D_MAKE_DATA(A3DCrvNurbsData);
    d.m_bIs2D = is2D;
    d.m_bRational = rational;
    d.m_uiDegree = (A3DUns32)degree;
    d.m_uiCtrlSize = (A3DUns32)pts.size();
    d.m_pCtrlPts = pts.data();
    d.m_uiWeightSize = rational ? (A3DUns32)w.size() : 0;
    d.m_pdWeights = rational ? w.data() : nullptr;
    d.m_uiKnotSize = (A3DUns32)k.size();
    d.m_pdKnots = k.data();
    d.m_eKnotType = kA3DKnotTypeUnspecified;
    d.m_eCurveForm = form;
    A3DCrvNurbs* c = nullptr;
    return A3DCrvNurbsCreate(&d, &c) == A3D_SUCCESS ? c : nullptr;
}

// Aggregate of Segment / Nurbs sub-curves -> one NURBS curve with exactly the aggregate's parameterisation.
static bool AggregateToNurbs(const qif::Curve& c, qif::Curve& out, std::string& why)
{
    struct Piece { int deg; std::vector<double> k, w; std::vector<Vec3> p; };
    std::vector<Piece> pieces;
    int maxDeg = 1;
    bool rational = false;
    for (const auto& sc : c.subs)
    {
        const qif::Curve& s = sc.first;
        Piece pc;
        if (s.kind == qif::CurveKind::Segment) { pc.deg = 1; pc.k = {s.d0, s.d0, s.d1, s.d1}; pc.p = {s.eval(s.d0), s.eval(s.d1)}; }
        else if (s.kind == qif::CurveKind::Nurbs)
        {
            size_t n = s.cps.size();
            if (std::fabs(s.d0 - s.knots[s.degree]) > 1e-12 || std::fabs(s.d1 - s.knots[n]) > 1e-12) { why = "aggregate with a trimmed NURBS sub-curve"; return false; }
            pc.deg = s.degree; pc.k = s.knots; pc.p = s.cps; pc.w = s.weights;
            if (!pc.w.empty()) rational = true;
        }
        else { why = "aggregate mixing arcs with other curve types"; return false; }
        if (sc.second)   // turned: reverse the piece, parameter t -> d0 + d1 - t
        {
            double a = pc.k.front(), b = pc.k.back();
            std::reverse(pc.p.begin(), pc.p.end());
            std::reverse(pc.w.begin(), pc.w.end());
            std::vector<double> k;
            for (auto it = pc.k.rbegin(); it != pc.k.rend(); ++it) k.push_back(a + b - *it);
            pc.k = k;
        }
        maxDeg = std::max(maxDeg, pc.deg);
        pieces.push_back(pc);
    }
    double acc = c.d0;
    for (auto& pc : pieces)
    {
        if (pc.deg < maxDeg)
        {
            if (pc.deg != 1) { why = "aggregate needing general degree elevation"; return false; }
            // a line elevated to degree p keeps its linear parameterisation: equally spaced control points
            Vec3 a = pc.p.front(), b = pc.p.back();
            double k0 = pc.k.front(), k1 = pc.k.back();
            pc.p.clear(); pc.k.clear();
            for (int i = 0; i <= maxDeg; ++i) pc.p.push_back(a + (b - a) * ((double)i / maxDeg));
            for (int i = 0; i <= maxDeg; ++i) pc.k.push_back(k0);
            for (int i = 0; i <= maxDeg; ++i) pc.k.push_back(k1);
            pc.w.clear();
            pc.deg = maxDeg;
        }
        if (rational && pc.w.empty()) pc.w.assign(pc.p.size(), 1.0);
        double shift = acc - pc.k.front();
        acc += pc.k.back() - pc.k.front();
        for (double& k : pc.k) k += shift;
    }
    out = qif::Curve();
    out.kind = qif::CurveKind::Nurbs;
    out.type = c.type;
    out.dim = c.dim;
    out.degree = maxDeg;
    for (size_t i = 0; i < pieces.size(); ++i)
    {
        Piece& pc = pieces[i];
        if (i == 0) { out.knots = pc.k; out.cps = pc.p; out.weights = pc.w; continue; }
        if ((pc.p.front() - out.cps.back()).norm() > 1e-6 * (1 + pc.p.front().norm())) { why = "aggregate with gaps"; return false; }
        if (rational) { double f = out.weights.back() / pc.w.front(); for (double& w : pc.w) w *= f; }
        out.knots.pop_back();                                                   // joint keeps multiplicity = degree
        out.knots.insert(out.knots.end(), pc.k.begin() + maxDeg + 1, pc.k.end());
        out.cps.insert(out.cps.end(), pc.p.begin() + 1, pc.p.end());
        if (rational) out.weights.insert(out.weights.end(), pc.w.begin() + 1, pc.w.end());
    }
    out.d0 = c.d0;
    out.d1 = acc;
    return true;
}

// Builds a PRC curve with the SAME parameterisation as the QIF curve on [c.d0, c.d1].
static A3DCrvBase* MakeCurve(const qif::Curve& c, bool is2D, std::string& why)
{
    using qif::CurveKind;
    switch (c.kind)
    {
    case CurveKind::Segment:
    {
        // Exact: a degree-1 B-spline whose knots are the QIF domain keeps the QIF parameter.
        if (c.d1 <= c.d0) { why = "degenerate segment domain"; return nullptr; }
        return MakeNurbsCurve(1, {c.d0, c.d0, c.d1, c.d1}, {c.eval(c.d0), c.eval(c.d1)}, {}, is2D, kA3DBSplineCurveFormPolyline);
    }
    case CurveKind::Nurbs:
        return MakeNurbsCurve(c.degree, c.knots, c.cps, c.weights, is2D, kA3DBSplineCurveFormUnspecified);
    case CurveKind::ArcCircular:
    case CurveKind::ArcConicEllipse:
    {
        if (is2D) { why = "2D arcs not supported"; return nullptr; }
        double shift = PeriodicShift(c.d0);
        if (c.kind == CurveKind::ArcCircular)
        {
            A3DCrvCircleData d = A3D_MAKE_DATA(A3DCrvCircleData);
            d.m_bIs2D = false;
            SetTrsf(d.m_sTrsf, c.center, c.xdir, c.ydir());
            d.m_sParam.m_sInterval.m_dMin = c.d0;
            d.m_sParam.m_sInterval.m_dMax = c.d1;
            d.m_sParam.m_dCoeffA = 1.0;
            d.m_sParam.m_dCoeffB = shift;
            d.m_dRadius = c.radius;
            A3DCrvCircle* p = nullptr;
            return A3DCrvCircleCreate(&d, &p) == A3D_SUCCESS ? p : nullptr;
        }
        A3DCrvEllipseData d = A3D_MAKE_DATA(A3DCrvEllipseData);
        d.m_bIs2D = false;
        SetTrsf(d.m_sTrsf, c.center, c.xdir, c.ydir());
        d.m_sParam.m_sInterval.m_dMin = c.d0;
        d.m_sParam.m_sInterval.m_dMax = c.d1;
        d.m_sParam.m_dCoeffA = 1.0;
        d.m_sParam.m_dCoeffB = shift;
        d.m_dXRadius = c.a;
        d.m_dYRadius = c.b;
        A3DCrvEllipse* p = nullptr;
        return A3DCrvEllipseCreate(&d, &p) == A3D_SUCCESS ? p : nullptr;
    }
    case CurveKind::Aggregate:
    {
        bool allSegments = true, allArcs = true;
        for (const auto& s : c.subs)
        {
            allSegments = allSegments && s.first.kind == CurveKind::Segment;
            allArcs = allArcs && s.first.kind == CurveKind::ArcCircular && !s.second;
        }
        if (allSegments)
        {
            // Polyline through the break points; knots = cumulative sub-domain lengths -> same parameter as QIF.
            std::vector<Vec3> pts;
            std::vector<double> knots;
            double t = c.d0;
            knots.push_back(t);
            for (size_t i = 0; i < c.subs.size(); ++i)
            {
                const auto& s = c.subs[i].first;
                bool turned = c.subs[i].second;
                Vec3 a = s.eval(turned ? s.d1 : s.d0), b = s.eval(turned ? s.d0 : s.d1);
                if (i == 0) { pts.push_back(a); knots.push_back(t); }
                else if ((a - pts.back()).norm() > 1e-6 * (1 + a.norm())) { why = "aggregate with gaps"; return nullptr; }
                t += s.d1 - s.d0;
                pts.push_back(b);
                knots.push_back(t);
            }
            knots.push_back(t);
            return MakeNurbsCurve(1, knots, pts, {}, is2D, kA3DBSplineCurveFormPolyline);
        }
        if (allArcs && !is2D)
        {
            // Consecutive arcs of one circle (same centre, radius and normal, continuous): one circle.
            const qif::Curve& f = c.subs.front().first;
            double total = 0;
            for (size_t i = 0; i < c.subs.size(); ++i)
            {
                const auto& s = c.subs[i].first;
                if ((s.center - f.center).norm() > 1e-9 * (1 + f.radius) || std::fabs(s.radius - f.radius) > 1e-9 * (1 + f.radius) ||
                    s.normal.dot(f.normal) < 1 - 1e-9)
                { why = "aggregate of arcs on different circles"; return nullptr; }
                if (i > 0)
                {
                    const auto& p = c.subs[i - 1].first;
                    if ((p.eval(p.d1) - s.eval(s.d0)).norm() > 1e-6 * (1 + f.radius)) { why = "aggregate of arcs with gaps"; return nullptr; }
                }
                total += s.d1 - s.d0;
            }
            qif::Curve circle = f;
            circle.d1 = f.d0 + total;
            return MakeCurve(circle, false, why);
        }
        // Mixed segments and NURBS: one NURBS with the same parameter (lines degree-elevated, pieces joined C0).
        qif::Curve joined;
        if (!AggregateToNurbs(c, joined, why)) return nullptr;
        return MakeCurve(joined, is2D, why);
    }
    default:
        why = c.problem.empty() ? "unsupported curve" : c.problem;
        return nullptr;
    }
}

// Builds a PRC surface whose final (u,v) parameterisation equals the QIF one on 'box'.
static A3DSurfBase* MakeSurface(const qif::Surface& s, const UVBox& box, std::string& why)
{
    using qif::SurfKind;
    A3DUVParameterizationData param;
    A3D_INITIALIZE_DATA(A3DUVParameterizationData, param);
    param.m_sUVDomain.m_sMin.m_dX = box.u0;
    param.m_sUVDomain.m_sMax.m_dX = box.u1;
    param.m_sUVDomain.m_sMin.m_dY = box.v0;
    param.m_sUVDomain.m_sMax.m_dY = box.v1;
    param.m_bSwapUV = false;
    param.m_dUCoeffA = 1; param.m_dUCoeffB = 0; param.m_dVCoeffA = 1; param.m_dVCoeffB = 0;

    // Angular u (longitude) = su * u, shifted by a multiple of 2 pi into the canonical range.
    auto angularU = [&]() {
        param.m_dUCoeffA = s.su;
        param.m_dUCoeffB = PeriodicShift(std::min(s.su * box.u0, s.su * box.u1));
    };

    switch (s.kind)
    {
    case SurfKind::Plane:
    {
        double a = s.dirU.norm(), b = s.dirV.norm();
        Vec3 x = s.dirU.unit(), y = s.dirV.unit();
        if (a > 0 && b > 0 && std::fabs(x.dot(y)) < 1e-9)
        {
            A3DSurfPlaneData d = A3D_MAKE_DATA(A3DSurfPlaneData);
            SetTrsf(d.m_sTrsf, s.origin, x, y);
            param.m_dUCoeffA = a;
            param.m_dVCoeffA = b;
            d.m_sParam = param;
            A3DSurfPlane* p = nullptr;
            if (A3DSurfPlaneCreate(&d, &p) == A3D_SUCCESS) return p;
            why = "A3DSurfPlaneCreate failed";
            return nullptr;
        }
        // Skewed plane: exact bilinear B-spline patch over the used domain (same u,v).
        std::vector<A3DVector3dData> pts = {V3(s.eval(box.u0, box.v0)), V3(s.eval(box.u0, box.v1)),
                                            V3(s.eval(box.u1, box.v0)), V3(s.eval(box.u1, box.v1))};
        std::vector<double> ku = {box.u0, box.u0, box.u1, box.u1}, kv = {box.v0, box.v0, box.v1, box.v1};
        A3DSurfNurbsData d = A3D_MAKE_DATA(A3DSurfNurbsData);
        d.m_eKnotType = kA3DKnotTypeUnspecified;
        d.m_eSurfaceForm = kA3DBSplineSurfaceFormPlane;
        d.m_uiUDegree = 1; d.m_uiVDegree = 1;
        d.m_uiUKnotSize = 4; d.m_uiVKnotSize = 4;
        d.m_uiUCtrlSize = 2; d.m_uiVCtrlSize = 2;
        d.m_pdUKnots = ku.data(); d.m_pdVKnots = kv.data();
        d.m_pdWeights = nullptr;
        d.m_pCtrlPts = pts.data();
        A3DSurfNurbs* p = nullptr;
        if (A3DSurfNurbsCreate(&d, &p) == A3D_SUCCESS) return p;
        why = "A3DSurfNurbsCreate failed (skewed plane)";
        return nullptr;
    }
    case SurfKind::Cylinder:
    {
        A3DSurfCylinderData d = A3D_MAKE_DATA(A3DSurfCylinderData);
        SetTrsf(d.m_sTrsf, s.origin, s.x, s.y);
        angularU();
        param.m_dVCoeffA = s.sv;   // height = sv * v
        d.m_sParam = param;
        d.m_dRadius = s.r;
        A3DSurfCylinder* p = nullptr;
        if (A3DSurfCylinderCreate(&d, &p) == A3D_SUCCESS) return p;
        why = "A3DSurfCylinderCreate failed";
        return nullptr;
    }
    case SurfKind::Cone:
    {
        double semi = std::atan2(s.r2 - s.r, s.length);   // radius grows by tan(semi) per unit of height
        A3DSurfConeData d = A3D_MAKE_DATA(A3DSurfConeData);
        SetTrsf(d.m_sTrsf, s.origin, s.x, s.y);
        angularU();
        param.m_dVCoeffA = g_coneVIsSlant ? s.sv / std::cos(semi) : s.sv;
        d.m_sParam = param;
        d.m_dRadius = s.r;
        d.m_dSemiAngle = semi;
        d.m_bInsideOut = false;
        A3DSurfCone* p = nullptr;
        if (A3DSurfConeCreate(&d, &p) == A3D_SUCCESS) return p;
        why = "A3DSurfConeCreate failed";
        return nullptr;
    }
    case SurfKind::Sphere:
    {
        A3DSurfSphereData d = A3D_MAKE_DATA(A3DSurfSphereData);
        SetTrsf(d.m_sTrsf, s.origin, s.x, s.y);
        angularU();
        param.m_dVCoeffA = s.sv;   // latitude
        d.m_sParam = param;
        d.m_dRadius = s.r;
        A3DSurfSphere* p = nullptr;
        if (A3DSurfSphereCreate(&d, &p) == A3D_SUCCESS) return p;
        why = "A3DSurfSphereCreate failed";
        return nullptr;
    }
    case SurfKind::Torus:
    {
        A3DSurfTorusData d = A3D_MAKE_DATA(A3DSurfTorusData);
        SetTrsf(d.m_sTrsf, s.origin, s.x, s.y);
        angularU();
        param.m_dVCoeffA = s.sv;
        param.m_dVCoeffB = s.offV + PeriodicShift(std::min(s.sv * box.v0, s.sv * box.v1) + s.offV);
        d.m_sParam = param;
        d.m_dMajorRadius = s.r;
        d.m_dMinorRadius = s.r2;
        A3DSurfTorus* p = nullptr;
        if (A3DSurfTorusCreate(&d, &p) == A3D_SUCCESS) return p;
        why = "A3DSurfTorusCreate failed";
        return nullptr;
    }
    case SurfKind::Nurbs:
    {
        // QIF stores control points with u varying fastest; PRC wants (Pu0v0, Pu0v1, ..., Pu1v0, ...).
        std::vector<A3DVector3dData> pts(s.cps.size());
        std::vector<double> w;
        bool rational = false;
        for (double x : s.w) if (std::fabs(x - 1.0) > 1e-14) rational = true;
        if (rational) w.resize(s.w.size());
        for (int i = 0; i < s.nu; ++i)
            for (int j = 0; j < s.nv; ++j)
            {
                pts[(size_t)i * s.nv + j] = V3(s.cps[(size_t)j * s.nu + i]);
                if (rational) w[(size_t)i * s.nv + j] = s.w[(size_t)j * s.nu + i];
            }
        std::vector<double> ku(s.ku), kv(s.kv);
        A3DSurfNurbsData d = A3D_MAKE_DATA(A3DSurfNurbsData);
        d.m_eKnotType = kA3DKnotTypeUnspecified;
        d.m_eSurfaceForm = kA3DBSplineSurfaceFormUnspecified;
        d.m_uiUDegree = (A3DUns32)s.degU; d.m_uiVDegree = (A3DUns32)s.degV;
        d.m_uiUKnotSize = (A3DUns32)ku.size(); d.m_uiVKnotSize = (A3DUns32)kv.size();
        d.m_uiUCtrlSize = (A3DUns32)s.nu; d.m_uiVCtrlSize = (A3DUns32)s.nv;
        d.m_pdUKnots = ku.data(); d.m_pdVKnots = kv.data();
        d.m_pdWeights = rational ? w.data() : nullptr;
        d.m_pCtrlPts = pts.data();
        A3DSurfNurbs* p = nullptr;
        if (A3DSurfNurbsCreate(&d, &p) == A3D_SUCCESS) return p;
        why = "A3DSurfNurbsCreate failed";
        return nullptr;
    }
    case SurfKind::Revolution:
    {
        // QIF: u = generatrix parameter, v = rotation angle. The PRC revolution's own (u, v) convention is not spelled
        // out in the API reference, so the mapping is calibrated: candidate parameterisations are built and the one
        // whose A3DSurfEvaluate matches the QIF formula on the used domain is kept.
        std::string cwhy;
        double best = 1e300;
        double scale = 1.0;
        for (double u : {box.u0, box.u1}) for (double v : {box.v0, box.v1}) scale = std::max(scale, (s.eval(u, v) - s.origin).norm());
        double shift = PeriodicShift(box.v0);
        for (int swap = 0; swap < 2; ++swap)
            for (double sh : {0.0, shift, -shift})
            {
                A3DCrvBase* gen = MakeCurve(s.gen, false, cwhy);
                if (!gen) { why = "revolution generatrix: " + cwhy; return nullptr; }
                A3DSurfRevolutionData d = A3D_MAKE_DATA(A3DSurfRevolutionData);
                SetTrsf(d.m_sTrsf, Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0));
                d.m_sTrsf.m_ucBehaviour = kA3DTransformationIdentity;
                d.m_pCurve = gen;
                d.m_sOrigin = V3(s.origin);
                d.m_sDirection = V3(s.z);
                A3DUVParameterizationData q = param;
                q.m_bSwapUV = swap == 1;
                q.m_dVCoeffB = sh;   // angle offset (multiple of 2 pi)
                d.m_sParam = q;
                A3DSurfRevolution* p = nullptr;
                if (A3DSurfRevolutionCreate(&d, &p) != A3D_SUCCESS || !p) continue;
                double dev = 0;
                for (int i = 0; i <= 2; ++i)
                    for (int j = 0; j <= 2; ++j)
                    {
                        double u = box.u0 + (box.u1 - box.u0) * i / 2, v = box.v0 + (box.v1 - box.v0) * j / 2;
                        A3DVector2dData uv; A3D_INITIALIZE_DATA(A3DVector2dData, uv);
                        uv.m_dX = u; uv.m_dY = v;
                        A3DVector3dData r[2]; A3D_INITIALIZE_DATA(A3DVector3dData, r[0]); A3D_INITIALIZE_DATA(A3DVector3dData, r[1]);
                        dev = A3DSurfEvaluate(p, &uv, 0, r) == A3D_SUCCESS ? std::max(dev, (FromA3D(r[0]) - s.eval(u, v)).norm()) : 1e300;
                    }
                if (dev <= 1e-9 * scale) return p;
                best = std::min(best, dev);
                A3DEntityDelete(p);
            }
        why = "no PRC revolution parameterisation matches the QIF one (best deviation " + std::to_string(best) + ")";
        return nullptr;
    }
    default:
        why = s.problem.empty() ? "unsupported surface" : s.problem;
        return nullptr;
    }
}

// Determines how this SDK's PRC cone interprets v (height along the axis, or length along the generatrix).
static void DetectConeConvention()
{
    A3DSurfConeData d = A3D_MAKE_DATA(A3DSurfConeData);
    SetTrsf(d.m_sTrsf, Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0));
    d.m_sParam.m_dUCoeffA = 1; d.m_sParam.m_dVCoeffA = 1;
    d.m_sParam.m_sUVDomain.m_sMax.m_dX = 2 * qif::kPi;
    d.m_sParam.m_sUVDomain.m_sMin.m_dY = -1;
    d.m_sParam.m_sUVDomain.m_sMax.m_dY = 1;
    d.m_dRadius = 1;
    d.m_dSemiAngle = std::atan(0.5);
    A3DSurfCone* cone = nullptr;
    if (A3DSurfConeCreate(&d, &cone) != A3D_SUCCESS || !cone) return;
    A3DVector2dData uv;
    A3D_INITIALIZE_DATA(A3DVector2dData, uv);
    uv.m_dX = 0; uv.m_dY = 1;
    A3DVector3dData r[2]; A3D_INITIALIZE_DATA(A3DVector3dData, r[0]); A3D_INITIALIZE_DATA(A3DVector3dData, r[1]);
    if (A3DSurfEvaluate(cone, &uv, 0, r) == A3D_SUCCESS)
        g_coneVIsSlant = std::fabs(r[0].m_dZ - std::cos(d.m_dSemiAngle)) < 1e-6;   // height form would give z = 1
    LOG("PRC cone v parameter: %s\n", g_coneVIsSlant ? "length along the generatrix" : "height along the axis");
    A3DEntityDelete(cone);
}

// ---------------------------------------------------------------------------------------------------------------------
// Topology: one PRC B-rep (A3DRiBrepModel) per QIF body
// ---------------------------------------------------------------------------------------------------------------------
// Where a QIF face / edge ended up in PRC: B-rep + indices as A3DMiscReferenceOnTopology expects them
// (face: global face index; edge: face index, loop index, co-edge index).
struct TopoRef { A3DTopoBrepData* brep = nullptr; A3DRiBrepModel* ri = nullptr; std::vector<A3DUns32> idx; long body = 0; };

class BrepBuilder
{
public:
    BrepBuilder(const qif::Model& m, Report& r, const std::map<long, UVBox>& boxes) : m_(m), r_(r), boxes_(boxes)
    {
        // Tolerant models (e.g. healed QIF exports): an edge curve may end some distance from its vertices. That gap
        // becomes the PRC edge and vertex tolerance so that HOOPS Exchange treats the topology as connected.
        for (const auto& e : m_.edges)
        {
            auto c = m_.curves3d.find(e.second.curve);
            if (c == m_.curves3d.end() || !c->second.ok()) continue;
            double g0 = (c->second.eval(c->second.d0) - m_.vertex(e.second.vbeg)).norm();
            double g1 = (c->second.eval(c->second.d1) - m_.vertex(e.second.vend)).norm();
            edgeGap_[e.first] = std::max(g0, g1);
            vertexGap_[e.second.vbeg] = std::max(vertexGap_[e.second.vbeg], g0);
            vertexGap_[e.second.vend] = std::max(vertexGap_[e.second.vend], g1);
            maxGap = std::max(maxGap, std::max(g0, g1));
        }
    }
    double maxGap = 0;   // largest vertex-to-curve gap in the model

    std::map<long, TopoRef> faceRefs, edgeRefs;   // filled for every converted face / edge
    std::map<long, A3DTopoFace*> faceEntities;    // QIF face id -> PRC face

    A3DRiBrepModel* build(const qif::Body& body)
    {
        verts_.clear(); edges_.clear(); edgeCoEdges_.clear(); surfs_.clear();
        std::vector<long> bodyFaces, bodyEdges;   // refs created for this body (brep pointer patched at the end)
        A3DUns32 faceIndex = 0;
        std::vector<std::pair<std::vector<long>, std::pair<bool, bool>>> shellDefs;   // faces, (closed, turned)
        for (long sid : body.shells)
        {
            auto it = m_.shells.find(sid);
            if (it == m_.shells.end()) { r_.skip("body references a missing shell"); continue; }
            shellDefs.push_back({it->second.faces, {it->second.closed, it->second.turned}});
        }
        if (shellDefs.empty() && !body.faces.empty()) shellDefs.push_back({body.faces, {false, false}});   // open sheet body

        bool allClosed = true;
        std::vector<A3DTopoShell*> shells;
        for (const auto& sd : shellDefs)
        {
            std::vector<A3DTopoFace*> faces;
            std::vector<A3DUns8> orient;
            for (long fid : sd.first)
            {
                auto fit = m_.faces.find(fid);
                if (fit == m_.faces.end()) { r_.skip("missing face"); continue; }
                A3DTopoFace* f = makeFace(fit->second, faceIndex, bodyEdges);
                if (!f) { ++r_.facesSkipped; continue; }
                ++r_.faces;
                faceRefs[fid] = TopoRef{nullptr, nullptr, {faceIndex}, body.id};
                faceEntities[fid] = f;
                bodyFaces.push_back(fid);
                ++faceIndex;
                faces.push_back(f);
                orient.push_back((fit->second.turned != sd.second.second) ? 0 : 1);
            }
            if (faces.empty()) continue;
            bool closed = sd.second.first && faces.size() == sd.first.size();
            allClosed = allClosed && closed;
            A3DTopoShellData d = A3D_MAKE_DATA(A3DTopoShellData);
            d.m_bClosed = closed;
            d.m_uiFaceSize = (A3DUns32)faces.size();
            d.m_ppFaces = faces.data();
            d.m_pucOrientationWithShell = orient.data();
            A3DTopoShell* sh = nullptr;
            if (A3DTopoShellCreate(&d, &sh) == A3D_SUCCESS) shells.push_back(sh);
        }
        if (shells.empty()) return nullptr;

        // Co-edges of the same edge form one cyclic neighbour list (manifold: usually 2).
        for (auto& e : edgeCoEdges_)
        {
            auto& v = e.second;
            if (v.size() < 2) continue;
            for (size_t i = 0; i < v.size(); ++i) A3DTopoCoEdgeSetNeighbour(v[i], v[(i + 1) % v.size()]);
        }

        A3DTopoConnexData cd = A3D_MAKE_DATA(A3DTopoConnexData);
        cd.m_uiShellSize = (A3DUns32)shells.size();
        cd.m_ppShells = shells.data();
        A3DTopoConnex* connex = nullptr;
        if (A3DTopoConnexCreate(&cd, &connex) != A3D_SUCCESS) return nullptr;

        A3DTopoBrepDataData bd = A3D_MAKE_DATA(A3DTopoBrepDataData);
        bd.m_uiConnexSize = 1;
        bd.m_ppConnexes = &connex;
        A3DTopoBrepData* brep = nullptr;
        if (A3DTopoBrepDataCreate(&bd, &brep) != A3D_SUCCESS) return nullptr;

        A3DTopoContextData ctx = A3D_MAKE_DATA(A3DTopoContextData);
        ctx.m_ucBehaviour = kA3DContextOuterLoopFirst | kA3DContextHasAnalytics;
        ctx.m_dTolerance = 1e-3;
        ctx.m_dGranularity = 1e-3;
        ctx.m_bHaveSmallestFaceThickness = false;
        ctx.m_bHaveScale = true;
        ctx.m_dScale = 1.0;
        A3DTopoContext* context = nullptr;
        if (A3DTopoContextCreate(&ctx, &context) == A3D_SUCCESS)
        {
            A3DTopoBodyData body3 = A3D_MAKE_DATA(A3DTopoBodyData);
            body3.m_pContext = context;
            A3DTopoBodySet(brep, &body3);
        }

        A3DRiBrepModelData rd = A3D_MAKE_DATA(A3DRiBrepModelData);
        rd.m_pBrepData = brep;
        rd.m_bSolid = allClosed;   // QIF form="MIXED" bodies with closed shells are solids too
        A3DRiBrepModel* ri = nullptr;
        if (A3DRiBrepModelCreate(&rd, &ri) != A3D_SUCCESS) return nullptr;
        for (long fid : bodyFaces) { faceRefs[fid].brep = brep; faceRefs[fid].ri = ri; }
        for (long eid : bodyEdges) { edgeRefs[eid].brep = brep; edgeRefs[eid].ri = ri; }
        return ri;
    }

private:
    const qif::Model& m_;
    Report& r_;
    const std::map<long, UVBox>& boxes_;
    static constexpr double kTolFloor = 1e-6;   // gaps below this are exact
    std::map<long, double> edgeGap_, vertexGap_;
    std::map<long, A3DTopoVertex*> verts_;
    std::map<long, A3DTopoEdge*> edges_;
    std::map<long, std::vector<A3DTopoCoEdge*>> edgeCoEdges_;
    std::map<long, A3DSurfBase*> surfs_;

    void checkCurve(const qif::Curve& c, A3DCrvBase* prc, const char* tag)
    {
        for (int i = 0; i <= 4; ++i)
        {
            double t = c.d0 + (c.d1 - c.d0) * i / 4.0;
            A3DVector3dData r[2]; A3D_INITIALIZE_DATA(A3DVector3dData, r[0]); A3D_INITIALIZE_DATA(A3DVector3dData, r[1]);
            if (A3DCrvEvaluate(prc, t, 0, r) != A3D_SUCCESS) { r_.devAdd(std::string(tag) + c.type + " (evaluate failed)", 0); return; }
            Vec3 q = c.eval(t), p = FromA3D(r[0]);
            if (c.dim == 2) { q.z = 0; p.z = 0; }
            r_.devAdd(std::string(tag) + c.type, (p - q).norm());
        }
    }

    A3DTopoVertex* vertex(long vid)
    {
        auto it = verts_.find(vid);
        if (it != verts_.end()) return it->second;
        A3DTopoUniqueVertexData d = A3D_MAKE_DATA(A3DTopoUniqueVertexData);
        d.m_sPoint = V3(m_.vertex(vid));
        auto g = vertexGap_.find(vid);
        d.m_dTolerance = (g != vertexGap_.end() && g->second > kTolFloor) ? g->second * 1.5 : 0;
        A3DTopoUniqueVertex* v = nullptr;
        A3DTopoUniqueVertexCreate(&d, &v);
        return verts_[vid] = v;
    }

    A3DTopoEdge* edge(long eid)
    {
        auto it = edges_.find(eid);
        if (it != edges_.end()) return it->second;
        auto e = m_.edges.find(eid);
        if (e == m_.edges.end()) return edges_[eid] = nullptr;
        A3DTopoEdgeData d = A3D_MAKE_DATA(A3DTopoEdgeData);
        if (auto g = edgeGap_.find(eid); g != edgeGap_.end() && g->second > kTolFloor) { d.m_dTolerance = g->second * 1.5; ++r_.tolerantEdges; }
        d.m_pStartVertex = vertex(e->second.vbeg);
        d.m_pEndVertex = vertex(e->second.vend);
        auto c = m_.curves3d.find(e->second.curve);
        if (c != m_.curves3d.end())
        {
            std::string why;
            A3DCrvBase* crv = MakeCurve(c->second, false, why);
            if (crv)
            {
                ++r_.created["edge curve " + c->second.type];
                checkCurve(c->second, crv, "edge ");
                d.m_p3dCurve = crv;
                d.m_bHasTrimDomain = true;
                d.m_sInterval.m_dMin = c->second.d0;
                d.m_sInterval.m_dMax = c->second.d1;
            }
            else { ++r_.edgesNo3d; r_.skip("edge kept without 3D curve: " + why); }
        }
        A3DTopoEdge* pe = nullptr;
        A3DTopoEdgeCreate(&d, &pe);
        return edges_[eid] = pe;
    }

    A3DSurfBase* surface(long sid, std::string& why)
    {
        auto it = surfs_.find(sid);
        if (it != surfs_.end()) { if (!it->second) why = "surface not convertible"; return it->second; }
        auto s = m_.surfaces.find(sid);
        auto b = boxes_.find(sid);
        A3DSurfBase* p = nullptr;
        if (s == m_.surfaces.end()) why = "missing surface";
        else if (b == boxes_.end() || !b->second.valid()) why = "surface without usable p-curves";
        else
        {
            p = MakeSurface(s->second, b->second, why);
            if (p)
            {
                ++r_.created["surface " + s->second.type];
                // Check at the corners and centre of the used domain.
                const UVBox& bx = b->second;
                double us[3] = {bx.u0, 0.5 * (bx.u0 + bx.u1), bx.u1}, vs[3] = {bx.v0, 0.5 * (bx.v0 + bx.v1), bx.v1};
                for (double u : us)
                    for (double v : vs)
                    {
                        A3DVector2dData uv;
                        A3D_INITIALIZE_DATA(A3DVector2dData, uv);
                        uv.m_dX = u; uv.m_dY = v;
                        A3DVector3dData r[2]; A3D_INITIALIZE_DATA(A3DVector3dData, r[0]); A3D_INITIALIZE_DATA(A3DVector3dData, r[1]);
                        if (A3DSurfEvaluate(p, &uv, 0, r) == A3D_SUCCESS)
                            r_.devAdd("surface " + s->second.type, (FromA3D(r[0]) - s->second.eval(u, v)).norm());
                        else
                            r_.devAdd("surface " + s->second.type + " (evaluate failed)", 0);
                    }
            }
        }
        return surfs_[sid] = p;
    }

    A3DTopoFace* makeFace(const qif::Face& f, A3DUns32 faceIndex, std::vector<long>& bodyEdges)
    {
        std::string why;
        A3DSurfBase* srf = surface(f.surface, why);
        if (!srf) { r_.skip("face skipped: " + why); return nullptr; }
        // Check every p-curve first so that a face is either converted completely or not at all.
        for (long lid : f.loops)
        {
            auto l = m_.loops.find(lid);
            if (l == m_.loops.end()) { r_.skip("face skipped: missing loop"); return nullptr; }
            for (const auto& ce : l->second.coedges)
            {
                auto pc = m_.curves2d.find(ce.pcurve);
                if (ce.pcurve == 0 || pc == m_.curves2d.end()) { r_.skip("face skipped: co-edge without p-curve"); return nullptr; }
                if (!pc->second.ok()) { r_.skip("face skipped: p-curve " + pc->second.problem); return nullptr; }
                if (m_.edges.find(ce.edge) == m_.edges.end()) { r_.skip("face skipped: missing edge"); return nullptr; }
            }
        }
        std::vector<A3DTopoLoop*> loops;
        std::vector<std::pair<long, A3DTopoCoEdge*>> made;
        std::vector<std::pair<long, std::vector<A3DUns32>>> positions;   // edge -> (face, loop, co-edge) indices
        for (long lid : f.loops)
        {
            std::vector<A3DTopoCoEdge*> coedges;
            const A3DUns32 loopIndex = (A3DUns32)loops.size();
            for (const auto& ce : m_.loops.at(lid).coedges)
            {
                const qif::Curve& pc = m_.curves2d.at(ce.pcurve);
                std::string cwhy;
                A3DCrvBase* uv = MakeCurve(pc, true, cwhy);
                if (!uv) { r_.skip("face skipped: p-curve " + cwhy); return nullptr; }
                ++r_.created["p-curve " + pc.type];
                checkCurve(pc, uv, "p-curve ");
                A3DTopoCoEdgeData d = A3D_MAKE_DATA(A3DTopoCoEdgeData);
                d.m_pEdge = edge(ce.edge);
                d.m_pUVCurve = uv;
                d.m_ucOrientationWithLoop = ce.turned ? 0 : 1;   // edge runs against the loop when 'turned'
                d.m_ucOrientationUVWithLoop = 1;                   // QIF p-curves follow the co-edge direction
                d.m_pNeighbor = nullptr;
                A3DTopoCoEdge* pce = nullptr;
                if (A3DTopoCoEdgeCreate(&d, &pce) != A3D_SUCCESS || !pce) { r_.skip("face skipped: co-edge creation failed"); return nullptr; }
                positions.push_back({ce.edge, {faceIndex, loopIndex, (A3DUns32)coedges.size()}});
                coedges.push_back(pce);
                made.emplace_back(ce.edge, pce);
            }
            A3DTopoLoopData ld = A3D_MAKE_DATA(A3DTopoLoopData);
            ld.m_ucOrientationWithSurface = 1;   // outer loops are counter-clockwise in UV, inner loops clockwise
            ld.m_uiCoEdgeSize = (A3DUns32)coedges.size();
            ld.m_ppCoEdges = coedges.data();
            A3DTopoLoop* loop = nullptr;
            if (A3DTopoLoopCreate(&ld, &loop) != A3D_SUCCESS) { r_.skip("face skipped: loop creation failed"); return nullptr; }
            loops.push_back(loop);
        }
        A3DTopoFaceData fd = A3D_MAKE_DATA(A3DTopoFaceData);
        fd.m_pSurface = srf;
        fd.m_bHasTrimDomain = false;
        fd.m_uiLoopSize = (A3DUns32)loops.size();
        fd.m_ppLoops = loops.empty() ? nullptr : loops.data();
        fd.m_uiOuterLoopIndex = (f.hasOuter && !loops.empty()) ? 0 : A3D_LOOP_UNKNOWN_OUTER_INDEX;
        fd.m_dTolerance = 0;
        A3DTopoFace* face = nullptr;
        if (A3DTopoFaceCreate(&fd, &face) != A3D_SUCCESS) { r_.skip("face skipped: A3DTopoFaceCreate failed"); return nullptr; }
        for (auto& e : made) edgeCoEdges_[e.first].push_back(e.second);
        for (auto& p : positions)
            if (!edgeRefs.count(p.first)) { edgeRefs[p.first] = TopoRef{nullptr, nullptr, p.second, 0}; bodyEdges.push_back(p.first); }
        return face;
    }
};

#include "prc_appearance.hpp"
#include "prc_semantic.hpp"
#include "prc_pmi.hpp"

// UV bounding box of every surface, from the p-curves of the faces that use it.
static std::map<long, UVBox> ComputeSurfaceBoxes(const qif::Model& m)
{
    std::map<long, UVBox> boxes;
    for (const auto& f : m.faces)
    {
        UVBox& b = boxes[f.second.surface];
        for (long lid : f.second.loops)
        {
            auto l = m.loops.find(lid);
            if (l == m.loops.end()) continue;
            for (const auto& ce : l->second.coedges)
            {
                auto pc = m.curves2d.find(ce.pcurve);
                if (pc == m.curves2d.end() || !pc->second.ok()) continue;
                const qif::Curve& c = pc->second;
                for (int i = 0; i <= 32; ++i)
                {
                    Vec3 uv = c.eval(c.d0 + (c.d1 - c.d0) * i / 32.0);
                    b.add(uv.x, uv.y);
                }
                if (c.kind == qif::CurveKind::Nurbs)   // the control polygon bounds the curve
                    for (const auto& p : c.cps) b.add(p.x, p.y);
            }
        }
    }
    return boxes;
}

// ---------------------------------------------------------------------------------------------------------------------
// Model statistics (used on the re-imported PRC and on the native QIF import)
// ---------------------------------------------------------------------------------------------------------------------
struct ModelStats { long attributedFaces = 0; long markups = 0, views = 0, linkedMarkups = 0, semanticMarkups = 0; long breps = 0, faces = 0; double unitMm = 1, area = 0, volume = 0; bool propsOk = false, exact = false; std::string propsNote; };

static void CountRi(const A3DRiRepresentationItem* ri, ModelStats& st)
{
    A3DEEntityType t = kA3DTypeUnknown;
    A3DEntityGetType(ri, &t);
    if (t == kA3DTypeRiSet)
    {
        A3DRiSetData d = A3D_MAKE_DATA(A3DRiSetData);
        if (A3DRiSetGet(ri, &d) != A3D_SUCCESS) return;
        for (A3DUns32 i = 0; i < d.m_uiRepItemsSize; ++i) CountRi(d.m_ppRepItems[i], st);
        A3DRiSetGet(nullptr, &d);
    }
    else if (t == kA3DTypeRiBrepModel)
    {
        A3DRiBrepModelData d = A3D_MAKE_DATA(A3DRiBrepModelData);
        if (A3DRiBrepModelGet(ri, &d) != A3D_SUCCESS) return;
        ++st.breps;
        A3DTopoBrepDataData bd = A3D_MAKE_DATA(A3DTopoBrepDataData);
        if (d.m_pBrepData && A3DTopoBrepDataGet(d.m_pBrepData, &bd) == A3D_SUCCESS)
        {
            for (A3DUns32 c = 0; c < bd.m_uiConnexSize; ++c)
            {
                A3DTopoConnexData cd = A3D_MAKE_DATA(A3DTopoConnexData);
                if (A3DTopoConnexGet(bd.m_ppConnexes[c], &cd) != A3D_SUCCESS) continue;
                for (A3DUns32 s = 0; s < cd.m_uiShellSize; ++s)
                {
                    A3DTopoShellData sd = A3D_MAKE_DATA(A3DTopoShellData);
                    if (A3DTopoShellGet(cd.m_ppShells[s], &sd) != A3D_SUCCESS) continue;
                    st.faces += sd.m_uiFaceSize;
                    for (A3DUns32 f = 0; f < sd.m_uiFaceSize; ++f)
                    {
                        A3DRootBaseData rb = A3D_MAKE_DATA(A3DRootBaseData);
                        if (A3DRootBaseGet(sd.m_ppFaces[f], &rb) == A3D_SUCCESS) { if (rb.m_uiSize) ++st.attributedFaces; A3DRootBaseGet(nullptr, &rb); }
                    }
                    A3DTopoShellGet(nullptr, &sd);
                }
                A3DTopoConnexGet(nullptr, &cd);
            }
            A3DTopoBrepDataGet(nullptr, &bd);
        }
        A3DRiBrepModelGet(nullptr, &d);
    }
}

static bool g_dumpPmi = false;   // --dump-pmi: print the semantic content read back from the written PRC

static std::string EntityName(const A3DEntity* e)
{
    A3DRootBaseData rb = A3D_MAKE_DATA(A3DRootBaseData);
    std::string n;
    if (A3DRootBaseGet(e, &rb) == A3D_SUCCESS) { if (rb.m_pcName) n = rb.m_pcName; A3DRootBaseGet(nullptr, &rb); }
    return n;
}

static void DumpSemantic(const A3DMkpMarkup* mk, A3DEEntityType t)
{
    std::string name = EntityName(mk);
    if (t == kA3DTypeMarkupGDT)
    {
        A3DMarkupGDTData g = A3D_MAKE_DATA(A3DMarkupGDTData);
        if (A3DMarkupGDTGet(mk, &g) != A3D_SUCCESS) return;
        for (A3DUns32 i = 0; i < g.m_uiNumberOfMarkupTolerances; ++i)
        {
            A3DMDFeatureControlFrameData f = A3D_MAKE_DATA(A3DMDFeatureControlFrameData);
            if (A3DMDFeatureControlFrameGet(g.m_ppsMarkupTolerances[i], &f) != A3D_SUCCESS) continue;
            for (A3DUns32 r = 0; r < f.m_uiNumberOfRows; ++r)
            {
                A3DMDFCFDrawingRowData d = A3D_MAKE_DATA(A3DMDFCFDrawingRowData);
                if (A3DMDFCFDrawingRowGet(f.m_ppRows[r], &d) != A3D_SUCCESS) continue;
                double tol = 0; int zone = 0, mod = 0;
                if (d.m_psSemanticValue)
                {
                    A3DMDFCFToleranceValueData v = A3D_MAKE_DATA(A3DMDFCFToleranceValueData);
                    if (A3DMDFCFToleranceValueGet(d.m_psSemanticValue, &v) == A3D_SUCCESS)
                    { tol = v.m_sValue.m_dValue; zone = (int)v.m_eValueType; mod = (int)v.m_eModifier; A3DMDFCFToleranceValueGet(nullptr, &v); }
                }
                std::string datums;
                for (A3DMDFCFRowDatum* rd : {d.m_psPrimaryRef, d.m_psSecondaryRef, d.m_psTertiaryRef})
                {
                    if (!rd) continue;
                    A3DMDFCFRowDatumData x = A3D_MAKE_DATA(A3DMDFCFRowDatumData);
                    if (A3DMDFCFRowDatumGet(rd, &x) == A3D_SUCCESS)
                    { datums += (datums.empty() ? "" : ",") + std::string(x.m_pcDatum ? x.m_pcDatum : "") + (x.m_eModifier ? "/" + std::to_string((int)x.m_eModifier) : ""); A3DMDFCFRowDatumGet(nullptr, &x); }
                }
                printf("PMI-GDT %s: type=%d tol=%g zone=%d mod=%d datums=%s\n", name.c_str(), (int)d.m_eType, tol, zone, mod, datums.c_str());
                A3DMDFCFDrawingRowGet(nullptr, &d);
            }
            A3DMDFeatureControlFrameGet(nullptr, &f);
        }
        A3DMarkupGDTGet(nullptr, &g);
    }
    else if (t == kA3DTypeMarkupDimension)
    {
        A3DMarkupDimensionData d = A3D_MAKE_DATA(A3DMarkupDimensionData);
        if (A3DMarkupDimensionGet(mk, &d) != A3D_SUCCESS) return;
        double sup = 0, inf = 0;
        if (d.m_pMainValue)
        {
            A3DMDDimensionValueData v = A3D_MAKE_DATA(A3DMDDimensionValueData);
            if (A3DMDDimensionValueGet(d.m_pMainValue, &v) == A3D_SUCCESS) { sup = v.m_dTolNumSup; inf = v.m_dTolNumInf; A3DMDDimensionValueGet(nullptr, &v); }
        }
        printf("PMI-DIM %s: type=%d value=%g tol=%g/%g symbol=%d\n", name.c_str(), (int)d.m_eType, d.m_dValue, sup, inf, (int)d.m_eSymbol);
        A3DMarkupDimensionGet(nullptr, &d);
    }
    else if (t == kA3DTypeMarkupDatum)
    {
        A3DMarkupDatumData d = A3D_MAKE_DATA(A3DMarkupDatumData);
        if (A3DMarkupDatumGet(mk, &d) != A3D_SUCCESS) return;
        printf("PMI-DATUM %s: %s target=%d\n", name.c_str(), d.m_pcUpText ? d.m_pcUpText : "", (int)d.m_bTarget);
        A3DMarkupDatumGet(nullptr, &d);
    }
    else if (t == kA3DTypeMarkupText)
    {
        A3DMarkupTextData d = A3D_MAKE_DATA(A3DMarkupTextData);
        if (A3DMarkupTextGet(mk, &d) != A3D_SUCCESS) return;
        printf("PMI-TEXT %s: lines=%u\n", name.c_str(), d.m_uiLinesSize);
        A3DMarkupTextGet(nullptr, &d);
    }
}

static void CountAnnotation(const A3DMkpAnnotationEntity* a, ModelStats& st, int depth = 0)
{
    if (!a || depth > 16) return;
    A3DEEntityType t = kA3DTypeUnknown;
    A3DEntityGetType(a, &t);
    if (t == kA3DTypeMkpAnnotationSet)
    {
        A3DMkpAnnotationSetData d = A3D_MAKE_DATA(A3DMkpAnnotationSetData);
        if (A3DMkpAnnotationSetGet(a, &d) != A3D_SUCCESS) return;
        for (A3DUns32 i = 0; i < d.m_uiAnnotationsSize; ++i) CountAnnotation(d.m_ppAnnotations[i], st, depth + 1);
        A3DMkpAnnotationSetGet(nullptr, &d);
    }
    else if (t == kA3DTypeMkpAnnotationItem)
    {
        ++st.markups;
        A3DMkpAnnotationItemData d = A3D_MAKE_DATA(A3DMkpAnnotationItemData);
        if (A3DMkpAnnotationItemGet(a, &d) == A3D_SUCCESS && d.m_pMarkup)
        {
            A3DEEntityType mt = kA3DTypeUnknown;
            A3DEntityGetType(d.m_pMarkup, &mt);
            if (mt == kA3DTypeMarkupGDT || mt == kA3DTypeMarkupDimension || mt == kA3DTypeMarkupDatum || mt == kA3DTypeMarkupText)
            {
                ++st.semanticMarkups;
                if (g_dumpPmi) DumpSemantic(d.m_pMarkup, mt);
            }
            A3DMkpMarkupData md = A3D_MAKE_DATA(A3DMkpMarkupData);
            if (A3DMkpMarkupGet(d.m_pMarkup, &md) == A3D_SUCCESS)
            {
                if (md.m_uiLinkedItemsSize > 0) ++st.linkedMarkups;
                A3DMkpMarkupGet(nullptr, &md);
            }
            A3DMkpAnnotationItemGet(nullptr, &d);
        }
    }
}

static void CountPo(const A3DAsmProductOccurrence* po, ModelStats& st, int depth = 0)
{
    if (!po || depth > 64) return;
    A3DAsmProductOccurrenceData d = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
    if (A3DAsmProductOccurrenceGet(po, &d) != A3D_SUCCESS) return;
    const A3DAsmPartDefinition* part = d.m_pPart;
    if (!part && d.m_pPrototype)   // part held by the prototype
    {
        A3DAsmProductOccurrenceData p = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
        const A3DAsmProductOccurrence* proto = d.m_pPrototype;
        while (proto && A3DAsmProductOccurrenceGet(proto, &p) == A3D_SUCCESS)
        {
            if (p.m_pPart) { part = p.m_pPart; A3DAsmProductOccurrenceGet(nullptr, &p); break; }
            proto = p.m_pPrototype;
            A3DAsmProductOccurrenceGet(nullptr, &p);
        }
    }
    if (part)
    {
        A3DAsmPartDefinitionData pd = A3D_MAKE_DATA(A3DAsmPartDefinitionData);
        if (A3DAsmPartDefinitionGet(part, &pd) == A3D_SUCCESS)
        {
            for (A3DUns32 i = 0; i < pd.m_uiRepItemsSize; ++i) CountRi(pd.m_ppRepItems[i], st);
            for (A3DUns32 i = 0; i < pd.m_uiAnnotationsSize; ++i) CountAnnotation(pd.m_ppAnnotations[i], st);
            st.views += pd.m_uiViewsSize;
            A3DAsmPartDefinitionGet(nullptr, &pd);
        }
    }
    if (d.m_pExternalData) CountPo(d.m_pExternalData, st, depth + 1);
    for (A3DUns32 i = 0; i < d.m_uiPOccurrencesSize; ++i) CountPo(d.m_ppPOccurrences[i], st, depth + 1);
    A3DAsmProductOccurrenceGet(nullptr, &d);
}

static ModelStats Analyze(A3DAsmModelFile* mf)
{
    ModelStats st;
    A3DAsmModelFileData d = A3D_MAKE_DATA(A3DAsmModelFileData);
    if (A3DAsmModelFileGet(mf, &d) == A3D_SUCCESS)
    {
        st.unitMm = d.m_dUnit > 0 ? d.m_dUnit : 1.0;
        for (A3DUns32 i = 0; i < d.m_uiPOccurrencesSize; ++i) CountPo(d.m_ppPOccurrences[i], st);
        A3DAsmModelFileGet(nullptr, &d);
    }
    // Exact (B-rep geometry) first; if the SDK refuses, fall back to its tessellation-based computation.
    A3DPhysicalPropertiesData pp = A3D_MAKE_DATA(A3DPhysicalPropertiesData);
    pp.m_bUseGeometryOnRiBRep = true;
    A3DStatus ps = A3DComputeModelFilePhysicalProperties(mf, &pp);
    st.exact = ps == A3D_SUCCESS;
    if (ps != A3D_SUCCESS)
    {
        st.propsNote = "exact properties failed: " + std::to_string((int)ps) + " " + A3DMiscGetErrorMsg(ps) + "; values from tessellation";
        pp = A3D_MAKE_DATA(A3DPhysicalPropertiesData);
        pp.m_bUseGeometryOnRiBRep = false;
        ps = A3DComputeModelFilePhysicalProperties(mf, &pp);
        if (ps != A3D_SUCCESS) st.propsNote = "physical properties failed: " + std::to_string((int)ps) + " " + A3DMiscGetErrorMsg(ps);
    }
    if (ps == A3D_SUCCESS)
    {
        st.propsOk = true;
        // Values are in model-file units: report in mm.
        st.area = pp.m_dSurface * st.unitMm * st.unitMm;
        st.volume = pp.m_bVolumeComputed ? pp.m_dVolume * st.unitMm * st.unitMm * st.unitMm : 0.0;
    }
    return st;
}

static void PrintStats(const char* tag, const ModelStats& s)
{
    printf("%-28s B-reps %ld, faces %ld", tag, s.breps, s.faces);
    if (s.propsOk) printf(", area %.6g mm2, volume %.6g mm3", s.area, s.volume);
    if (s.attributedFaces) printf(", %ld faces with attributes", s.attributedFaces);
    if (s.markups || s.views) printf(", markups %ld (%ld linked, %ld semantic), views %ld", s.markups, s.linkedMarkups, s.semanticMarkups, s.views);
    if (!s.propsNote.empty()) printf(" (%s)", s.propsNote.c_str());
    printf("\n");
}

// ---------------------------------------------------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------------------------------------------------
static int Run(const Options& opt)
{
    // ---- arguments that can be rejected before any work
    fs::path inPath = fs::u8path(opt.input), outPath = fs::u8path(opt.output);
    std::string ext = outPath.extension().u8string();
    for (auto& ch : ext) ch = (char)std::tolower((unsigned char)ch);
    if (ext != ".prc") { printf("ERROR: unsupported output format '%s' (only .prc is written)\n", ext.c_str()); return kUnsupported; }
    std::error_code ec;
    if (!fs::is_regular_file(inPath, ec)) { printf("ERROR: input file not found: %s\n", opt.input.c_str()); return kInput; }
    if (fs::exists(outPath, ec) && !opt.force) { printf("ERROR: output exists: %s (use --force to overwrite)\n", opt.output.c_str()); return kUsage; }

    // ---- read and parse the QIF
    qif::Model model;
    try
    {
        std::ifstream f(inPath, std::ios::binary);
        if (!f) { printf("ERROR: cannot open %s\n", opt.input.c_str()); return kInput; }
        std::stringstream ss;
        ss << f.rdbuf();
        auto root = qifxml::parseDocument(ss.str());
        qif::buildModel(root.get(), model);
        model.doc = std::move(root);   // kept: views and PMI are read from the document later
    }
    catch (const std::exception& e)
    {
        printf("ERROR: cannot read QIF: %s\n", e.what());
        return kInput;
    }
    LOG("QIF: %zu surfaces, %zu 3D curves, %zu p-curves, %zu edges, %zu faces, %zu shells, %zu bodies; unit %s (%g m)\n",
        model.surfaces.size(), model.curves3d.size(), model.curves2d.size(), model.edges.size(), model.faces.size(),
        model.shells.size(), model.bodies.size(), model.linearUnitName.c_str(), model.unitToMeter);
    if (model.partCount > 1 || model.assemblyCount || model.componentCount)
        printf("WARNING: %d parts, %d assemblies, %d components: assembly structure is not converted; all bodies go into one part\n",
               model.partCount, model.assemblyCount, model.componentCount);
    for (const auto& n : model.notes) LOG("note: %s\n", n.c_str());

    // ---- build PRC: B-reps, wire bodies, coordinate systems, appearance
    DetectConeConvention();
    Report rep;
    StyleCache styles;
    std::map<long, UVBox> boxes = ComputeSurfaceBoxes(model);
    BrepBuilder builder(model, rep, boxes);
    std::vector<A3DRiRepresentationItem*> items;
    std::vector<A3DMiscEntityReference*> entityRefs;   // per-face colours (product occurrence level)
    std::map<long, int> layerOf;                       // QIF element id -> QIF layer index
    for (const auto& l : model.layers) for (long e : l.elements) layerOf[e] = l.index;
    auto layerFor = [&](long id) { auto it = layerOf.find(id); return it == layerOf.end() ? A3D_DEFAULT_LAYER : (A3DUns16)it->second; };
    long hiddenSkipped = 0, wireBodies = 0;
    for (const auto& b : model.bodies)
    {
        if (b.hidden && opt.skipHidden) { ++hiddenSkipped; continue; }
        if (b.hasTransform) rep.skip("body Transform ignored (no TransformSet support yet)");
        std::vector<qif::Attribute> attrs = b.attributes;
        if (!b.form.empty()) attrs.push_back({"QIF", "QIF body form", b.form});
        for (const auto& l : model.layers)
            if (std::find(l.elements.begin(), l.elements.end(), b.id) != l.elements.end()) attrs.push_back({"QIF", "QIF layer", l.label});
        if (b.shells.empty() && b.faces.empty())
        {
            if (b.edges.empty()) continue;
            A3DRiRepresentationItem* set = MakeWireBody(model, b, styles, rep);
            if (!set) continue;
            SetNameAndAttributes(set, !b.label.empty() ? b.label : "Wire body " + std::to_string(b.id), attrs, b.id);
            SetGraphics(set, styles.get(b.color), !b.hidden, layerFor(b.id));
            items.push_back(set);
            ++wireBodies;
            continue;
        }
        A3DRiBrepModel* ri = builder.build(b);
        if (!ri) { ++rep.bodiesSkipped; continue; }
        ++rep.bodies;
        items.push_back(ri);
        // Body colour = the most common face colour (QIF often leaves the body white and colours the faces);
        // faces with another colour get an entity reference.
        std::map<qif::Color, long> faceColors;
        for (const auto& fr : builder.faceRefs)
            if (fr.second.ri == ri) { const auto& f = model.faces.at(fr.first); if (f.color.set) ++faceColors[f.color]; }
        qif::Color bodyColor = b.color;
        long best = 0;
        for (const auto& fc : faceColors) if (fc.second > best) { best = fc.second; bodyColor = fc.first; }
        SetNameAndAttributes(ri, !b.label.empty() ? b.label : "Body " + std::to_string(b.id), attrs, b.id);
        SetGraphics(ri, styles.get(bodyColor), !b.hidden, layerFor(b.id));
        for (const auto& fr : builder.faceRefs)
        {
            if (fr.second.ri != ri) continue;
            const auto& f = model.faces.at(fr.first);
            if (!f.color.set || f.color == bodyColor) continue;
            if (auto* er = MakeFaceColorReference(fr.second.brep, fr.second.idx[0], styles.get(f.color))) { entityRefs.push_back(er); ++rep.created["face colour reference"]; }
        }
    }
    for (const auto& k : model.coordSystems)
    {
        A3DRiRepresentationItem* cs = MakeCoordinateSystem(k);
        if (!cs) { rep.skip("coordinate system creation failed"); continue; }
        SetNameAndAttributes(cs, k.label, {}, k.id);
        SetGraphics(cs, styles.get(k.color), !k.hidden, layerFor(k.id));
        items.push_back(cs);
        ++rep.created["coordinate system"];
    }
    // QIF features (FeatureNominals) -> attributes on the faces they are made of
    long featureFaces = 0;
    if (const qifxml::Node* feats = model.doc ? model.doc->child("Features") : nullptr)
        if (const qifxml::Node* noms = feats->child("FeatureNominals"))
        {
            std::map<long, std::vector<qif::Attribute>> perFace;
            for (const auto& fn : noms->children)
            {
                std::string fname = fn->childText("Name");
                if (fname.empty()) fname = fn->name + " " + std::to_string(qif::idOf(fn.get()));
                std::string ftype = fn->name.size() > 14 ? fn->name.substr(0, fn->name.size() - 14) : fn->name;   // strip "FeatureNominal"
                for (long eid : qif::idList(fn->child("EntityInternalIds")))
                    if (builder.faceEntities.count(eid))
                        perFace[eid].push_back({"QIF", "QIF feature", fname + " (" + ftype + ", id " + std::to_string(qif::idOf(fn.get())) + ")"});
            }
            for (auto& pf : perFace)
            {
                const auto& f = model.faces.at(pf.first);
                SetNameAndAttributes(builder.faceEntities[pf.first], f.label == " " ? std::string() : f.label, pf.second, pf.first);
                ++featureFaces;
            }
        }
    if (featureFaces) LOG("Features: %ld faces carry QIF feature attributes\n", featureFaces);

    PmiResult pmi;
    if (!opt.noPmi)
    {
        pmi = BuildPmiAndViews(model, builder, styles);
        printf("PMI: %ld markups (%ld text lines, %ld links to faces/edges, %ld unresolved links), %ld saved views, %ld annotation planes\n",
            pmi.stats.markups, pmi.stats.texts, pmi.stats.linked, pmi.stats.unresolved, pmi.savedViews, pmi.annotationViews);
        if (g_semantic)
        {
            long nsem = 0;
            for (const auto& c : g_semStats.created) nsem += c.second;
            printf("Semantic PMI: %ld markups with a semantic definition", nsem);
            for (const auto& c : g_semStats.created) printf(", %ld %s", c.second, c.first.c_str());
            printf("\n");
            for (const auto& f : g_semStats.failed)
                if (f.first.compare(0, 10, "note lines") == 0) printf("note: %s (x%ld)\n", f.first.c_str(), f.second);
                else printf("WARNING: semantic %s not created (x%ld), kept as graphical markup\n", f.first.c_str(), f.second);
        }
        if (pmi.stats.noFont) printf("WARNING: font 'Arial' not available to HOOPS Exchange: %ld markups have no text\n", pmi.stats.noFont);
    }
    if (rep.tolerantEdges) LOG("Tolerant model: %ld edges end up to %.3g from their vertices; used as edge/vertex tolerance\n", rep.tolerantEdges, builder.maxGap);
    if (wireBodies) LOG("Wire bodies converted: %ld\n", wireBodies);
    if (hiddenSkipped) LOG("note: %ld hidden bodies skipped (--skip-hidden)\n", hiddenSkipped);

    // ---- report geometry checks
    double worst = 0;
    LOG("Geometry check (PRC entity evaluated by HOOPS Exchange vs. QIF definition, model units):\n");
    for (const auto& d : rep.dev)
    {
        LOG("  %-40s %6ld samples  max deviation %.3g\n", d.first.c_str(), d.second.n, d.second.maxDev);
        worst = std::max(worst, d.second.maxDev);
    }
    for (const auto& s : rep.skipped) printf("WARNING: %s (x%ld)\n", s.first.c_str(), s.second);
    printf("Converted %ld bodies, %ld faces (%ld faces skipped, %ld bodies skipped, %ld edges without 3D curve); worst deviation %.3g\n",
           rep.bodies, rep.faces, rep.facesSkipped, rep.bodiesSkipped, rep.edgesNo3d, worst);
    if (items.empty()) { printf("ERROR: nothing convertible in %s\n", opt.input.c_str()); return kUnsupported; }

    A3DAsmPartDefinitionData pd = A3D_MAKE_DATA(A3DAsmPartDefinitionData);
    pd.m_uiRepItemsSize = (A3DUns32)items.size();
    pd.m_ppRepItems = items.data();
    pd.m_uiAnnotationsSize = (A3DUns32)pmi.annotations.size();
    pd.m_ppAnnotations = pmi.annotations.empty() ? nullptr : pmi.annotations.data();
    pd.m_uiViewsSize = (A3DUns32)pmi.views.size();
    pd.m_ppViews = pmi.views.empty() ? nullptr : pmi.views.data();
    A3DAsmPartDefinition* part = nullptr;
    A3DStatus st = A3DAsmPartDefinitionCreate(&pd, &part);
    if (st != A3D_SUCCESS) { PrintStatus("A3DAsmPartDefinitionCreate", st); return kExport; }

    A3DAsmProductOccurrenceData pod = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
    pod.m_pPart = part;
    pod.m_uiEntityReferenceSize = (A3DUns32)entityRefs.size();
    pod.m_ppEntityReferences = entityRefs.empty() ? nullptr : entityRefs.data();
    pod.m_bUnitFromCAD = true;
    pod.m_dUnit = model.unitToMeter * 1000.0;
    A3DAsmProductOccurrence* po = nullptr;
    st = A3DAsmProductOccurrenceCreate(&pod, &po);
    if (st != A3D_SUCCESS) { PrintStatus("A3DAsmProductOccurrenceCreate", st); return kExport; }
    std::string poName = !model.partName.empty() ? model.partName : inPath.stem().u8string();
    std::vector<qif::Attribute> poAttrs = model.partAttributes;
    poAttrs.push_back({"QIF", "QIF source", inPath.filename().u8string()});
    poAttrs.push_back({"QIF", "QIF linear unit", model.linearUnitName});
    SetNameAndAttributes(po, poName, poAttrs);

    A3DAsmModelFileData mfd = A3D_MAKE_DATA(A3DAsmModelFileData);
    mfd.m_uiPOccurrencesSize = 1;
    mfd.m_ppPOccurrences = &po;
    mfd.m_bUnitFromCAD = true;
    mfd.m_dUnit = model.unitToMeter * 1000.0;   // PRC unit is mm-based
    A3DAsmModelFile* mf = nullptr;
    st = A3DAsmModelFileCreate(&mfd, &mf);
    if (st != A3D_SUCCESS) { PrintStatus("A3DAsmModelFileCreate", st); return kExport; }

    // ---- export to a temporary file, then rename
    fs::path tmp = outPath;
    tmp += ".partial.prc";
    A3DRWParamsExportPrcData ep = A3D_MAKE_DATA(A3DRWParamsExportPrcData);
    ep.m_bCompressBrep = false;
    ep.m_bCompressTessellation = false;
    ep.m_bRemoveBRep = false;
    ep.m_bRemoveAttributes = false;
    st = A3DAsmModelFileExportToPrcFile(mf, &ep, tmp.u8string().c_str(), nullptr);
    A3DAsmModelFileDelete(mf);
    if (st != A3D_SUCCESS) { PrintStatus("A3DAsmModelFileExportToPrcFile", st); fs::remove(tmp, ec); return kExport; }
    if (fs::exists(outPath, ec)) fs::remove(outPath, ec);
    fs::rename(tmp, outPath, ec);
    if (ec) { printf("ERROR: cannot move %s to %s: %s\n", tmp.u8string().c_str(), opt.output.c_str(), ec.message().c_str()); return kExport; }
    printf("Wrote %s\n", opt.output.c_str());

    // ---- re-import what was written
    A3DRWParamsLoadData lp = A3D_MAKE_DATA(A3DRWParamsLoadData);
    lp.m_sGeneral.m_bReadSolids = true;
    lp.m_sGeneral.m_bReadSurfaces = true;
    lp.m_sGeneral.m_eReadGeomTessMode = kA3DReadGeomOnly;
    lp.m_sGeneral.m_bReadHiddenObjects = true;   // hidden bodies were written too
    lp.m_sGeneral.m_bReadWireframes = true;
    lp.m_sGeneral.m_bReadPmis = true;
    A3DAsmModelFile* back = nullptr;
    st = A3DAsmModelFileLoadFromFile(opt.output.c_str(), &lp, &back);
    if (st != A3D_SUCCESS || !back) { PrintStatus("re-import of the written PRC", st); return kExport; }
    ModelStats out = Analyze(back);
    A3DAsmModelFileDelete(back);
    PrintStats("Re-imported PRC:", out);
    if (out.breps != rep.bodies || out.faces != rep.faces)
    { printf("ERROR: re-imported PRC has %ld B-reps / %ld faces, expected %ld / %ld\n", out.breps, out.faces, rep.bodies, rep.faces); return kExport; }

    int rc = kOk;
    if (opt.maxDev >= 0 && worst > opt.maxDev) { printf("VERIFY FAILED: worst deviation %.3g > --max-dev %.3g\n", worst, opt.maxDev); rc = kVerify; }

    // ---- optional comparison with HOOPS Exchange's own QIF reader
    if (opt.compareNative)
    {
        A3DRWParamsLoadData np = A3D_MAKE_DATA(A3DRWParamsLoadData);
        np.m_sGeneral.m_bReadSolids = true;
        np.m_sGeneral.m_bReadSurfaces = true;
        np.m_sGeneral.m_bReadHiddenObjects = !opt.skipHidden;
        np.m_sGeneral.m_eReadGeomTessMode = kA3DReadGeomOnly;
        A3DAsmModelFile* nat = nullptr;
        st = A3DAsmModelFileLoadFromFile(opt.input.c_str(), &np, &nat);
        if (st != A3D_SUCCESS || !nat) { PrintStatus("native QIF import (beta reader)", st); return kVerify; }
        ModelStats ns = Analyze(nat);
        A3DAsmModelFileDelete(nat);
        PrintStats("Native QIF reader:", ns);
        auto rel = [](double a, double b) { double m = std::max(std::fabs(a), std::fabs(b)); return m > 0 ? std::fabs(a - b) / m : 0.0; };
        double dA = rel(out.area, ns.area), dV = rel(out.volume, ns.volume);
        // Face counts must match. Area and volume are compared only when BOTH sides were computed exactly from the
        // B-rep: a tessellation-based fallback (e.g. a B-rep the SDK rejects) is not a fair reference.
        bool comparable = out.exact && ns.exact;
        printf("Difference vs native reader: faces %ld vs %ld", out.faces, ns.faces);
        if (comparable) printf(", area %.3g, volume %.3g (relative)\n", dA, dV);
        else printf("; area/volume not compared (%s)\n", !out.exact ? "this converter's B-rep is not exact-computable" : "native reader's B-rep is not exact-computable");
        if (out.faces != ns.faces || !out.exact || (comparable && (dA > opt.nativeTol || dV > opt.nativeTol)))
        { printf("VERIFY FAILED: differs from the native QIF reader beyond --native-tol %.3g\n", opt.nativeTol); rc = kVerify; }
        else if (!comparable) printf("Native comparison: face counts match; native B-rep could not be measured exactly\n");
    }
    return rc;
}

static int Main(const std::vector<std::string>& args)
{
    Options opt;
    std::vector<std::string> pos;
    for (size_t i = 1; i < args.size(); ++i)
    {
        const std::string& a = args[i];
        auto needValue = [&](double& v) {
            if (i + 1 >= args.size()) return false;
            char* end = nullptr;
            v = std::strtod(args[++i].c_str(), &end);
            return end && *end == 0;
        };
        if (a == "--force") opt.force = true;
        else if (a == "--skip-hidden") opt.skipHidden = true;
        else if (a == "--no-pmi") opt.noPmi = true;
        else if (a == "--no-semantic") g_semantic = false;
        else if (a == "--dump-pmi") g_dumpPmi = true;
        else if (a == "--debug") g_debug = true;
        else if (a == "--quiet") opt.quiet = true;
        else if (a == "--compare-native") opt.compareNative = true;
        else if (a == "--max-dev") { if (!needValue(opt.maxDev)) { PrintUsage(); return kUsage; } }
        else if (a == "--native-tol") { if (!needValue(opt.nativeTol)) { PrintUsage(); return kUsage; } }
        else if (a.size() > 2 && a.compare(0, 2, "--") == 0) { printf("ERROR: unknown option %s\n", a.c_str()); PrintUsage(); return kUsage; }
        else pos.push_back(a);
    }
    if (pos.size() != 2) { PrintUsage(); return kUsage; }
    opt.input = pos[0];
    opt.output = pos[1];
    g_quiet = opt.quiet;

    if (!A3DSDKLoadLibraryA(HOOPS_BINARY_DIRECTORY)) { printf("ERROR: cannot load HOOPS Exchange from %s\n", HOOPS_BINARY_DIRECTORY); return kInput; }
    A3DStatus st = A3DLicPutUnifiedLicense(HOOPS_LICENSE);
    if (st != A3D_SUCCESS) { PrintStatus("A3DLicPutUnifiedLicense", st); A3DSDKUnloadLibrary(); return kInput; }
    st = A3DDllInitialize(A3D_DLL_MAJORVERSION, A3D_DLL_MINORVERSION);
    if (st != A3D_SUCCESS) { PrintStatus("A3DDllInitialize", st); A3DSDKUnloadLibrary(); return kInput; }
    A3DDllSetCallbacksReport(SdkMsg, SdkWarn, SdkErr);

    int rc = Run(opt);

    A3DDllTerminate();
    A3DSDKUnloadLibrary();
    return rc;
}

#ifdef _WIN32
int wmain(int argc, wchar_t** argv)
{
    std::vector<std::string> args;
    for (int i = 0; i < argc; ++i) args.push_back(fs::path(argv[i]).u8string());   // UTF-16 -> UTF-8
    return Main(args);
}
#else
int main(int argc, char** argv)
{
    return Main(std::vector<std::string>(argv, argv + argc));
}
#endif
