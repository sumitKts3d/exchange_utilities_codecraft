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
/// \example QIF_Writer
/// CAD / PRC to QIF 3.0 writer using the HOOPS Exchange public API.
///
/// 1. load any CAD file with HOOPS Exchange
/// 2. collect exact B-rep, structure and PMI from the model
/// 3. write the QIF 3.0 document and self-check it
////////////////////////////////////////////////////////////////////////////////

/***********************************************************************************************************************
 * QIF_Writer — writes any file HOOPS Exchange reads (PRC, STEP AP242, CATIA V5, NX, Creo, SolidWorks, JT, ...) as a
 * QIF 3.0 document, with the HOOPS Exchange public API only.
 *
 *   PRC (HOOPS Exchange)                        QIF 3.0
 *   ------------------------------------------  ------------------------------------------------------------------------
 *   model-file unit                             FileUnits / PrimaryUnits / LinearUnit (factor to metre); angles in degree
 *   product occurrences (flattened)             one Part (PartSet + RootPart); every B-rep instance is a Body placed in
 *                                               model space (occurrence locations and B-rep context scale applied)
 *   exact B-rep (A3DRiBrepModel)                GeometrySet + TopologySet: Points, Nurbs13 edge curves, Nurbs12 p-curves,
 *                                               Nurbs23 surfaces, Vertex/Edge/Loop/Face/Shell/Body. The B-rep is first
 *                                               adapted to NURBS with A3DCopyAndAdaptBrepModel (UV and 3D curves
 *                                               recomputed, same topology); loops are written counter-clockwise in UV
 *                                               with the outer loop first, co-edge p-curves follow the co-edge.
 *   colours, visibility, names, attributes      Face / Body color, Body hidden, label, Attributes (AttributeStr)
 *   semantic PMI                                Characteristics: GD&T (Definition + Nominal, tolerance, zone,
 *                                               material condition, datum reference frame), sizes (Diameter / Radius /
 *                                               Length / Angle / DistanceBetween with +/- tolerance), DatumDefinitions,
 *                                               DatumTargetDefinitions, Notes (NoteSet); linked faces / edges as
 *                                               OtherNonShapeFeatureNominal EntityInternalIds
 *   PMI presentation (markup tessellation)      VisualizationSet / PMIDisplay: polylines, filled areas and text outlines
 *                                               in the markup's own annotation plane (AnnotationViewSet), Reference to
 *                                               the semantic entity
 *   views with a camera                         ViewSet / CameraSet + SavedViewSet (visible annotations, active view)
 *
 *   The document is written to <out>.partial.qif and renamed after success; existing outputs need --force.
 *   Self-check: every co-edge p-curve, evaluated on its NURBS surface, must land on the edge vertices (exit 5 if not).
 *
 *   Grounded in: HOOPS Exchange 2026.8 headers A3DSDKTools.h (A3DCopyAndAdaptBrepModel), A3DSDKTopology.h,
 *   A3DSDKGeometryCrv.h / Srf.h (A3DCrvNurbsData, A3DSurfNurbsData, *GetAsNurbs), A3DSDKMarkup*.h (semantic PMI),
 *   A3DSDKGraphics.h (cascaded attributes); samples viewers/Viewer/traverse (tree, markup tessellation);
 *   the QIF 3.0 NIST models and their reading by ../QIF_Reader (element names, parameterisations, PMIDisplay layout).
 ***********************************************************************************************************************/
#define INITIALIZE_A3D_API
#include <A3DSDKIncludes.h>
#include <hoops_license.h>

#include "qif_out.hpp"

#include <cctype>
#include <cstdio>
#include <cstring>
#include <tuple>
#include <ctime>
#include <filesystem>
#include <functional>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using qo::E;
using qo::Mat4;
using qo::Vec3;

enum ExitCode { kOk = 0, kUsage = 1, kInput = 2, kUnsupported = 3, kWriteFailed = 4, kVerify = 5 };

struct Options
{
    std::string input, output;
    bool force = false, quiet = false, verbose = false, skipHidden = false, pmi = true;
    double maxGap = 0;   // 0: automatic (1e-4 of the model size, at least 1e-6)
};

static bool g_quiet = false, g_verbose = false;
static A3DInt32 SdkMsg(A3DUTF8Char* m) { if (m && g_verbose) printf("  [HOOPS] %s", m); return 0; }
static A3DInt32 SdkWarn(A3DUTF8Char* c, A3DUTF8Char* m) { if (g_verbose) printf("  [HOOPS warning %s] %s", c ? c : "", m ? m : ""); return 0; }
static A3DInt32 SdkErr(A3DUTF8Char* c, A3DUTF8Char* m) { if (!g_quiet) printf("  [HOOPS error %s] %s", c ? c : "", m ? m : ""); return 0; }

static void PrintStatus(const char* what, A3DStatus st) { printf("ERROR: %s failed: %d %s\n", what, (int)st, A3DMiscGetErrorMsg(st)); }

static void PrintUsage()
{
    printf("QIF_Writer - writes any CAD file HOOPS Exchange reads as QIF 3.0 (exact B-rep, structure, colours, semantic\n"
           "and graphical PMI, saved views).\n\n"
           "Usage: QIF_Writer <input CAD file> <output.qif> [options]\n"
           "  --force          overwrite the output\n"
           "  --skip-hidden    do not write hidden bodies (default: written with hidden=\"true\")\n"
           "  --no-pmi         geometry, structure and appearance only\n"
           "  --max-gap <d>    exit 5 if a p-curve end misses its vertex by more than d model units (default: automatic)\n"
           "  --quiet          warnings, errors and summary only\n"
           "  --verbose        also print the HOOPS Exchange reader messages\n"
           "Exit codes: 0 ok, 1 usage / output exists, 2 input, 3 nothing to write, 4 write failed, 5 self-check failed.\n");
}

// =====================================================================================================================
// HOOPS Exchange helpers
// =====================================================================================================================
class ExchangeSession
{
public:
    ExchangeSession()
    {
        if (!A3DSDKLoadLibraryA(HOOPS_BINARY_DIRECTORY)) { printf("ERROR: cannot load HOOPS Exchange from %s\n", HOOPS_BINARY_DIRECTORY); return; }
        loaded_ = true;
        A3DStatus st = A3DLicPutUnifiedLicense(HOOPS_LICENSE);
        if (st != A3D_SUCCESS) { PrintStatus("A3DLicPutUnifiedLicense", st); return; }
        st = A3DDllInitialize(A3D_DLL_MAJORVERSION, A3D_DLL_MINORVERSION);
        if (st != A3D_SUCCESS) { PrintStatus("A3DDllInitialize", st); return; }
        A3DDllSetCallbacksReport(SdkMsg, SdkWarn, SdkErr);
        ok_ = true;
    }
    ~ExchangeSession()
    {
        if (ok_) A3DDllTerminate();
        if (loaded_) A3DSDKUnloadLibrary();
    }
    bool Ok() const { return ok_; }

private:
    bool loaded_ = false, ok_ = false;
};

static Vec3 V(const A3DVector3dData& d) { return Vec3(d.m_dX, d.m_dY, d.m_dZ); }

static std::string EntityName(const A3DEntity* e)
{
    A3DRootBaseData bd = A3D_MAKE_DATA(A3DRootBaseData);
    if (!e || A3DRootBaseGet(e, &bd) != A3D_SUCCESS) return "";
    std::string n = bd.m_pcName ? bd.m_pcName : "";
    A3DRootBaseGet(nullptr, &bd);
    return n;
}

static Mat4 FromPrcTransform(const A3DMiscTransformation* t)
{
    Mat4 m;
    if (!t) return m;
    A3DEEntityType type = kA3DTypeUnknown;
    A3DEntityGetType(t, &type);
    if (type == kA3DTypeMiscCartesianTransformation)
    {
        A3DMiscCartesianTransformationData d = A3D_MAKE_DATA(A3DMiscCartesianTransformationData);
        if (A3DMiscCartesianTransformationGet(t, &d) != A3D_SUCCESS) return m;
        Vec3 X = V(d.m_sXVector), Y = V(d.m_sYVector), Z = X.cross(Y);
        if (d.m_ucBehaviour & kA3DTransformationMirror) Z = -Z;
        const double s[3] = {d.m_sScale.m_dX, d.m_sScale.m_dY, d.m_sScale.m_dZ};
        const Vec3 rows[3] = {X, Y, Z};
        for (int r = 0; r < 3; ++r)
        {
            m.m[r][0] = rows[r].x * s[r]; m.m[r][1] = rows[r].y * s[r]; m.m[r][2] = rows[r].z * s[r]; m.m[r][3] = 0;
        }
        m.m[3][0] = d.m_sOrigin.m_dX; m.m[3][1] = d.m_sOrigin.m_dY; m.m[3][2] = d.m_sOrigin.m_dZ; m.m[3][3] = 1;
        A3DMiscCartesianTransformationGet(nullptr, &d);
    }
    else if (type == kA3DTypeMiscGeneralTransformation)
    {
        A3DMiscGeneralTransformationData d = A3D_MAKE_DATA(A3DMiscGeneralTransformationData);
        if (A3DMiscGeneralTransformationGet(t, &d) != A3D_SUCCESS) return m;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) m.m[i][j] = d.m_adCoeff[i * 4 + j];
        A3DMiscGeneralTransformationGet(nullptr, &d);
    }
    return m;
}

// Product occurrence rules of the SDK samples (TreeTraverse): own part / children / location, else along the
// prototype and external-data chain.
static A3DAsmPartDefinition* EffectivePart(const A3DAsmProductOccurrence* po)
{
    A3DAsmProductOccurrenceData d = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
    if (!po || A3DAsmProductOccurrenceGet(po, &d) != A3D_SUCCESS) return nullptr;
    A3DAsmPartDefinition* part = d.m_pPart;
    for (A3DAsmProductOccurrence* proto = d.m_pPrototype; !part && proto;)
    {
        A3DAsmProductOccurrenceData pd = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
        if (A3DAsmProductOccurrenceGet(proto, &pd) != A3D_SUCCESS) break;
        part = pd.m_pPart;
        proto = pd.m_pPrototype ? pd.m_pPrototype : pd.m_pExternalData;
        A3DAsmProductOccurrenceGet(nullptr, &pd);
    }
    if (!part && d.m_uiPOccurrencesSize == 0 && d.m_pExternalData) part = EffectivePart(d.m_pExternalData);
    A3DAsmProductOccurrenceGet(nullptr, &d);
    return part;
}

static std::vector<A3DAsmProductOccurrence*> EffectiveChildren(const A3DAsmProductOccurrence* po)
{
    std::vector<A3DAsmProductOccurrence*> kids;
    A3DAsmProductOccurrenceData d = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
    if (!po || A3DAsmProductOccurrenceGet(po, &d) != A3D_SUCCESS) return kids;
    kids.assign(d.m_ppPOccurrences, d.m_ppPOccurrences + d.m_uiPOccurrencesSize);
    for (A3DAsmProductOccurrence* proto = d.m_pPrototype; kids.empty() && proto;)
    {
        A3DAsmProductOccurrenceData pd = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
        if (A3DAsmProductOccurrenceGet(proto, &pd) != A3D_SUCCESS) break;
        kids.assign(pd.m_ppPOccurrences, pd.m_ppPOccurrences + pd.m_uiPOccurrencesSize);
        proto = pd.m_pPrototype;
        A3DAsmProductOccurrenceGet(nullptr, &pd);
    }
    if (d.m_pExternalData)
    {
        if (kids.empty()) kids = EffectiveChildren(d.m_pExternalData);
        else kids.push_back(d.m_pExternalData);
    }
    A3DAsmProductOccurrenceGet(nullptr, &d);
    return kids;
}

static const A3DMiscTransformation* EffectiveLocation(const A3DAsmProductOccurrence* po)
{
    A3DAsmProductOccurrenceData d = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
    if (!po || A3DAsmProductOccurrenceGet(po, &d) != A3D_SUCCESS) return nullptr;
    const A3DMiscTransformation* loc = d.m_pLocation;
    if (!loc && d.m_pPrototype) loc = EffectiveLocation(d.m_pPrototype);
    else if (!loc && d.m_pExternalData) loc = EffectiveLocation(d.m_pExternalData);
    A3DAsmProductOccurrenceGet(nullptr, &d);
    return loc;
}

static std::string OccurrenceName(const A3DAsmProductOccurrence* po)
{
    for (const A3DAsmProductOccurrence* p = po; p;)
    {
        std::string n = EntityName(p);
        if (!n.empty()) return n;
        A3DAsmProductOccurrenceData d = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
        if (A3DAsmProductOccurrenceGet(p, &d) != A3D_SUCCESS) break;
        p = d.m_pPrototype ? d.m_pPrototype : d.m_pExternalData;
        A3DAsmProductOccurrenceGet(nullptr, &d);
    }
    return "";
}

struct Rgb { bool set = false; int r = 0, g = 0, b = 0; };

static Rgb ColorOfIndex(A3DUns32 rgbIndex)
{
    Rgb c;
    A3DGraphRgbColorData cd = A3D_MAKE_DATA(A3DGraphRgbColorData);
    if (rgbIndex != A3D_DEFAULT_COLOR_INDEX && A3DGlobalGetGraphRgbColorData(rgbIndex, &cd) == A3D_SUCCESS)
    {
        c.set = true;
        c.r = (int)std::lround(cd.m_dRed * 255); c.g = (int)std::lround(cd.m_dGreen * 255); c.b = (int)std::lround(cd.m_dBlue * 255);
    }
    return c;
}

static Rgb ColorOfStyle(const A3DGraphStyleData& sd)
{
    if (!sd.m_bMaterial) return ColorOfIndex(sd.m_uiRgbColorIndex);
    A3DGraphMaterialData md = A3D_MAKE_DATA(A3DGraphMaterialData);
    if (A3DGlobalGetGraphMaterialData(sd.m_uiRgbColorIndex, &md) != A3D_SUCCESS) return Rgb();
    return ColorOfIndex(md.m_uiDiffuse);
}

static Rgb ColorOfStyleIndex(A3DUns32 style)
{
    if (style == A3D_DEFAULT_STYLE_INDEX) return Rgb();
    A3DGraphStyleData sd = A3D_MAKE_DATA(A3DGraphStyleData);
    if (A3DGlobalGetGraphStyleData(style, &sd) != A3D_SUCCESS) return Rgb();
    return ColorOfStyle(sd);
}

// Own colour of an entity with graphics (a B-rep face), or none.
static Rgb OwnColor(const A3DEntity* e)
{
    Rgb c;
    A3DRootBaseWithGraphicsData gd = A3D_MAKE_DATA(A3DRootBaseWithGraphicsData);
    if (!e || A3DRootBaseWithGraphicsGet(e, &gd) != A3D_SUCCESS) return c;
    if (gd.m_pGraphics)
    {
        A3DGraphicsData g = A3D_MAKE_DATA(A3DGraphicsData);
        if (A3DGraphicsGet(gd.m_pGraphics, &g) == A3D_SUCCESS)
        {
            c = ColorOfStyleIndex(g.m_uiStyleIndex);
            A3DGraphicsGet(nullptr, &g);
        }
    }
    A3DRootBaseWithGraphicsGet(nullptr, &gd);
    return c;
}

static std::string ColorText(const Rgb& c) { return std::to_string(c.r) + " " + std::to_string(c.g) + " " + std::to_string(c.b); }

// PRC attributes of an entity -> (name, value) pairs "Title:Key".
static std::vector<std::pair<std::string, std::string>> PrcAttributes(const A3DEntity* e)
{
    std::vector<std::pair<std::string, std::string>> out;
    A3DRootBaseData bd = A3D_MAKE_DATA(A3DRootBaseData);
    if (!e || A3DRootBaseGet(e, &bd) != A3D_SUCCESS) return out;
    for (A3DUns32 i = 0; i < bd.m_uiSize; ++i)
    {
        A3DMiscAttributeData ad = A3D_MAKE_DATA(A3DMiscAttributeData);
        if (A3DMiscAttributeGet(bd.m_ppAttributes[i], &ad) != A3D_SUCCESS) continue;
        const std::string title = ad.m_pcTitle ? ad.m_pcTitle : "";
        for (A3DUns32 k = 0; k < ad.m_uiSize; ++k)
        {
            const A3DMiscSingleAttributeData& s = ad.m_asSingleAttributesData[k];
            std::string key = s.m_pcTitle ? s.m_pcTitle : "";
            std::string val = s.m_pcData ? s.m_pcData : "";
            if (s.m_eType == kA3DModellerAttributeTypeInt && s.m_pcData) val = std::to_string(*(const A3DInt32*)s.m_pcData);
            else if (s.m_eType == kA3DModellerAttributeTypeReal && s.m_pcData) val = qo::num(*(const A3DDouble*)s.m_pcData);
            else if (s.m_eType == kA3DModellerAttributeTypeTime && s.m_pcData) val = std::to_string(*(const A3DInt32*)s.m_pcData);
            const std::string name = title.empty() ? key : key.empty() || key == title ? title : title + ":" + key;
            if (!name.empty()) out.push_back({name, val});
        }
        A3DMiscAttributeGet(nullptr, &ad);
    }
    A3DRootBaseGet(nullptr, &bd);
    return out;
}

static E AttributesElement(const std::vector<std::pair<std::string, std::string>>& attrs)
{
    E a("Attributes");
    a.a("n", attrs.size());
    for (const auto& kv : attrs) a.c(E("AttributeStr").a("name", kv.first).a("value", kv.second));
    return a;
}

// NURBS data of a curve or surface: the entity itself when it is a NURBS, else its exact NURBS form in the same
// parameterisation (so that p-curves and trim intervals stay valid).
struct CurveNurbs
{
    int degree = 0;
    std::vector<double> knots, weights;
    std::vector<Vec3> cps;
    bool ok = false;
};
static CurveNurbs GetCurveNurbs(const A3DCrvBase* c, double tol)
{
    CurveNurbs out;
    if (!c) return out;
    A3DCrvNurbsData d = A3D_MAKE_DATA(A3DCrvNurbsData);
    A3DEEntityType t = kA3DTypeUnknown;
    A3DEntityGetType(c, &t);
    A3DStatus st = t == kA3DTypeCrvNurbs ? A3DCrvNurbsGet(c, &d) : A3DCrvBaseGetAsNurbs(c, tol, true, &d);
    if (st != A3D_SUCCESS) return out;
    out.degree = (int)d.m_uiDegree;
    out.knots.assign(d.m_pdKnots, d.m_pdKnots + d.m_uiKnotSize);
    for (A3DUns32 i = 0; i < d.m_uiCtrlSize; ++i) out.cps.push_back(V(d.m_pCtrlPts[i]));
    if (d.m_bRational && d.m_uiWeightSize == d.m_uiCtrlSize) out.weights.assign(d.m_pdWeights, d.m_pdWeights + d.m_uiWeightSize);
    out.ok = out.degree >= 1 && !out.cps.empty() && out.knots.size() == out.cps.size() + out.degree + 1;
    A3DCrvNurbsGet(nullptr, &d);
    return out;
}

static void ReverseNurbs(CurveNurbs& c)
{
    std::reverse(c.cps.begin(), c.cps.end());
    std::reverse(c.weights.begin(), c.weights.end());
    const double a = c.knots.front(), b = c.knots.back();
    std::vector<double> k(c.knots.size());
    for (size_t i = 0; i < k.size(); ++i) k[i] = a + b - c.knots[c.knots.size() - 1 - i];
    c.knots = k;
}

struct SurfNurbs
{
    int du = 0, dv = 0, nu = 0, nv = 0;
    std::vector<double> ku, kv, w;   // w in QIF order
    std::vector<Vec3> cps;           // QIF order: u fastest
    bool ok = false;
};
static SurfNurbs GetSurfNurbs(const A3DSurfBase* s, double tol, const Mat4& M)
{
    SurfNurbs out;
    if (!s) return out;
    A3DSurfNurbsData d = A3D_MAKE_DATA(A3DSurfNurbsData);
    A3DEEntityType t = kA3DTypeUnknown;
    A3DEntityGetType(s, &t);
    A3DStatus st = t == kA3DTypeSurfNurbs ? A3DSurfNurbsGet(s, &d) : A3DSurfBaseWithDomainGetAsNurbs(s, nullptr, tol, true, &d);
    if (st != A3D_SUCCESS) return out;
    out.du = (int)d.m_uiUDegree; out.dv = (int)d.m_uiVDegree;
    out.nu = (int)d.m_uiUCtrlSize; out.nv = (int)d.m_uiVCtrlSize;
    out.ku.assign(d.m_pdUKnots, d.m_pdUKnots + d.m_uiUKnotSize);
    out.kv.assign(d.m_pdVKnots, d.m_pdVKnots + d.m_uiVKnotSize);
    out.cps.resize((size_t)out.nu * out.nv);
    if (d.m_pdWeights) out.w.resize(out.cps.size());
    // PRC stores (u0v0, u0v1, ..., u1v0, ...) = v fastest; QIF wants u fastest.
    for (int i = 0; i < out.nu; ++i)
        for (int j = 0; j < out.nv; ++j)
        {
            out.cps[(size_t)j * out.nu + i] = M.point(V(d.m_pCtrlPts[(size_t)i * out.nv + j]));
            if (d.m_pdWeights) out.w[(size_t)j * out.nu + i] = d.m_pdWeights[(size_t)i * out.nv + j];
        }
    bool rational = false;
    for (double x : out.w) if (std::fabs(x - 1.0) > 1e-14) rational = true;
    if (!rational) out.w.clear();
    out.ok = out.du >= 1 && out.dv >= 1 && out.ku.size() == (size_t)out.nu + out.du + 1 && out.kv.size() == (size_t)out.nv + out.dv + 1;
    A3DSurfNurbsGet(nullptr, &d);
    return out;
}

static double BrepScale(const A3DTopoBrepData* brep)
{
    double s = 1.0;
    A3DTopoBodyData bd = A3D_MAKE_DATA(A3DTopoBodyData);
    if (brep && A3DTopoBodyGet(brep, &bd) == A3D_SUCCESS)
    {
        if (bd.m_pContext)
        {
            A3DTopoContextData cd = A3D_MAKE_DATA(A3DTopoContextData);
            if (A3DTopoContextGet(bd.m_pContext, &cd) == A3D_SUCCESS)
            {
                if (cd.m_bHaveScale && cd.m_dScale > 0) s = cd.m_dScale;
                A3DTopoContextGet(nullptr, &cd);
            }
        }
        A3DTopoBodyGet(nullptr, &bd);
    }
    return s;
}

#include "qif_pmi.hpp"        // PRC markups -> PmiItem (tessellation, semantic content, links)
#include "qif_document.hpp"   // the QIF document: geometry, topology, structure, PMI, views

// =====================================================================================================================
// main
// =====================================================================================================================
static std::string Lower(std::string s) { for (char& c : s) c = (char)std::tolower((unsigned char)c); return s; }

static std::string NewUuid()
{
    std::random_device rd;
    std::mt19937_64 g(((uint64_t)rd() << 32) ^ rd() ^ (uint64_t)std::time(nullptr));
    uint64_t a = g(), b = g();
    a = (a & 0xFFFFFFFFFFFF0FFFull) | 0x0000000000004000ull;
    b = (b & 0x3FFFFFFFFFFFFFFFull) | 0x8000000000000000ull;
    char s[40];
    snprintf(s, sizeof s, "%08x-%04x-%04x-%04x-%012llx", (unsigned)(a >> 32), (unsigned)((a >> 16) & 0xFFFF), (unsigned)(a & 0xFFFF),
             (unsigned)(b >> 48), (unsigned long long)(b & 0xFFFFFFFFFFFFull));
    return s;
}

static int Run(const Options& opt)
{
    std::error_code ec;
    const fs::path inPath = fs::u8path(opt.input), outPath = fs::u8path(opt.output);
    if (Lower(outPath.extension().u8string()) != ".qif") { printf("ERROR: unsupported output format '%s' (expected .qif)\n", outPath.extension().u8string().c_str()); return kUsage; }
    if (!fs::exists(inPath, ec)) { printf("ERROR: input file not found: %s\n", opt.input.c_str()); return kInput; }
    if (fs::exists(outPath, ec) && !opt.force) { printf("ERROR: output exists: %s (use --force to overwrite)\n", opt.output.c_str()); return kUsage; }

    A3DRWParamsLoadData lp = A3D_MAKE_DATA(A3DRWParamsLoadData);
    lp.m_sGeneral.m_eReadGeomTessMode = kA3DReadGeomAndTess;
    lp.m_sGeneral.m_bReadSolids = true;
    lp.m_sGeneral.m_bReadSurfaces = true;
    lp.m_sGeneral.m_bReadWireframes = false;
    lp.m_sGeneral.m_bReadPmis = opt.pmi;
    lp.m_sGeneral.m_bReadAttributes = true;
    lp.m_sGeneral.m_bReadHiddenObjects = true;
    lp.m_sGeneral.m_bReadConstructionAndReferences = false;
    lp.m_sGeneral.m_bReadFeature = false;
    lp.m_sPmi.m_bAlwaysSubstituteFont = false;
    lp.m_sTessellation.m_eTessellationLevelOfDetail = kA3DTessLODMedium;
    lp.m_sMultiEntries.m_bLoadDefault = true;
    A3DAsmModelFile* mf = nullptr;
    A3DStatus st = A3DAsmModelFileLoadFromFile(opt.input.c_str(), &lp, &mf);
    if ((st != A3D_SUCCESS && st != A3D_LOAD_MISSING_COMPONENTS && st != A3D_LOAD_MULTI_MODELS_CADFILE) || !mf)
    {
        PrintStatus("A3DAsmModelFileLoadFromFile", st);
        if (mf) A3DAsmModelFileDelete(mf);
        return kInput;
    }
    if (st == A3D_LOAD_MISSING_COMPONENTS) printf("WARNING: some referenced files of the assembly were not found; they are not written\n");

    QifDocument doc(opt);
    doc.uuid = NewUuid();
    doc.sourceName = inPath.filename().u8string();
    const bool built = doc.Build(mf);
    const SourceProps props = SourceProperties(mf);
    A3DAsmModelFileDelete(mf);
    if (!built) return kUnsupported;

    printf("Input: %s (unit %g mm)\n", doc.sourceName.c_str(), doc.unitMm);
    printf("Source model: B-reps %ld, faces %ld", doc.st.sourceBreps, doc.st.sourceFaces);
    if (props.ok) printf(", area %.6g mm2, volume %.6g mm3", props.area, props.volume);
    printf("\n");
    if (doc.st.bodies == 0) { printf("ERROR: no exact B-rep body to write (QIF holds B-rep geometry; tessellated-only models are not written)\n"); return kUnsupported; }

    const std::string xml = doc.Xml();
    fs::path partial = outPath;
    partial.replace_extension(".partial.qif");
    {
        FILE* f = nullptr;
#ifdef _WIN32
        f = _wfopen(partial.wstring().c_str(), L"wb");
#else
        f = fopen(partial.u8string().c_str(), "wb");
#endif
        if (!f) { printf("ERROR: cannot write %s\n", partial.u8string().c_str()); return kWriteFailed; }
        const size_t n = fwrite(xml.data(), 1, xml.size(), f);
        const bool good = fclose(f) == 0 && n == xml.size();
        if (!good) { printf("ERROR: writing %s failed\n", partial.u8string().c_str()); fs::remove(partial, ec); return kWriteFailed; }
    }
    fs::remove(outPath, ec);
    fs::rename(partial, outPath, ec);
    if (ec) { printf("ERROR: cannot rename %s: %s\n", partial.u8string().c_str(), ec.message().c_str()); return kWriteFailed; }

    doc.PrintSummary();
    printf("Written: %s (%.1f kB)\n", outPath.u8string().c_str(), xml.size() / 1024.0);
    // A p-curve end must meet its vertex within the limit, or within 1.5 x the tolerance of a tolerant edge.
    const double limit = opt.maxGap > 0 ? opt.maxGap : std::max(1e-6, 1e-4 * doc.modelSize);
    long over = 0, tolerant = 0;
    for (const auto& g : doc.st.gaps)
    {
        if (g.first <= limit) continue;
        if (g.first <= 1.5 * g.second) ++tolerant;
        else
        {
            ++over;
            if (!opt.quiet && over <= 5) printf("  gap %.4g at a co-edge whose edge tolerance is %.4g\n", g.first, g.second);
        }
    }
    printf("Self-check: worst p-curve end to vertex gap %.3g (limit %.3g), %ld co-edges checked", doc.st.worstGap, limit, doc.st.checkedCoEdges);
    if (tolerant) printf(", %ld within their tolerant edge's tolerance", tolerant);
    printf("\n");
    if (over) { printf("VERIFY FAILED: %ld p-curve ends do not meet their vertices\n", over); return kVerify; }
    printf("Check: OK\n");
    return kOk;
}

static int Main(const std::vector<std::string>& args)
{
    Options opt;
    std::vector<std::string> pos;
    for (size_t i = 1; i < args.size(); ++i)
    {
        const std::string& a = args[i];
        if (a == "--force") opt.force = true;
        else if (a == "--quiet") opt.quiet = true;
        else if (a == "--verbose") opt.verbose = true;
        else if (a == "--skip-hidden") opt.skipHidden = true;
        else if (a == "--no-pmi") opt.pmi = false;
        else if (a == "--max-gap")
        {
            if (i + 1 >= args.size() || (opt.maxGap = std::strtod(args[i + 1].c_str(), nullptr)) <= 0) { printf("ERROR: --max-gap expects a positive number\n"); return kUsage; }
            ++i;
        }
        else if (a == "--help" || a == "-h") { PrintUsage(); return kUsage; }
        else if (a.size() > 2 && a.compare(0, 2, "--") == 0) { printf("ERROR: unknown option %s\n", a.c_str()); PrintUsage(); return kUsage; }
        else pos.push_back(a);
    }
    if (pos.size() != 2) { PrintUsage(); return kUsage; }
    opt.input = pos[0];
    opt.output = pos[1];
    g_quiet = opt.quiet;
    g_verbose = opt.verbose && !opt.quiet;
    ExchangeSession hoops;
    if (!hoops.Ok()) return kInput;
    return Run(opt);
}

#ifdef _WIN32
int wmain(int argc, wchar_t** argv)
{
    std::vector<std::string> args;
    for (int i = 0; i < argc; ++i) args.push_back(fs::path(argv[i]).u8string());   // UTF-16 -> UTF-8
    return Main(args);
}
#else
int main(int argc, char** argv) { return Main(std::vector<std::string>(argv, argv + argc)); }
#endif
