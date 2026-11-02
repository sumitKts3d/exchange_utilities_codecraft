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

/***********************************************************************************************************************
 * hoops_usd.hpp — conventions and helpers shared by USD_Reader (USD -> PRC) and USD_Writer (CAD/PRC -> USD).
 *
 * Everything that must be identical in both directions lives here, so a round trip is lossless by construction:
 *   - the matrix convention between GfMatrix4d and PRC transformations,
 *   - units (USD metersPerUnit <-> PRC unit in mm) and the Y-up -> Z-up rotation,
 *   - the appearance model (Look) and the roughness <-> shininess mapping,
 *   - prim-name and attribute-namespace conventions,
 *   - HOOPS Exchange start-up, message callbacks, exit codes,
 *   - PRC inspection: tessellation decoding (every A3DTessFaceData layout), style text, tree dump, triangle count.
 *
 * Include it after <A3DSDKIncludes.h> (INITIALIZE_A3D_API defined in exactly one translation unit) and pxr headers.
 * Grounded in HOOPS Exchange 2026.8: A3DSDKTessellation.h / A3DSDKEnums.h (a3d_tessfacetype), samples
 * viewers/Viewer/traverse/TessConnector.cpp, TreeTraverse.cpp, viewers/MeshViewer/model/he/product_occurrence.cpp.
 ***********************************************************************************************************************/
#pragma once

#include <pxr/pxr.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/tf/stringUtils.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <set>
#include <string>
#include <vector>

namespace hoops_usd
{
PXR_NAMESPACE_USING_DIRECTIVE

// ---------------------------------------------------------------------------------------------------------------------
// Exit codes, logging, HOOPS Exchange session
// ---------------------------------------------------------------------------------------------------------------------
enum ExitCode { kOk = 0, kUsage = 1, kInput = 2, kUnsupported = 3, kExport = 4, kVerify = 5 };

inline bool g_quiet = false;     // only warnings, errors and the summary
inline bool g_verbose = false;   // also the HOOPS Exchange reader / writer messages
#define LOG(...) do { if (!hoops_usd::g_quiet) printf(__VA_ARGS__); } while (0)

inline A3DInt32 SdkMsg(A3DUTF8Char* m) { if (m && g_verbose) printf("  [HOOPS] %s", m); return 0; }
inline A3DInt32 SdkWarn(A3DUTF8Char* c, A3DUTF8Char* m) { printf("  [HOOPS warning %s] %s", c ? c : "", m ? m : ""); return 0; }
inline A3DInt32 SdkErr(A3DUTF8Char* c, A3DUTF8Char* m) { printf("  [HOOPS error %s] %s", c ? c : "", m ? m : ""); return 0; }

inline void PrintStatus(const char* what, A3DStatus st)
{
    const A3DUTF8Char* msg = A3DMiscGetErrorMsg(st);
    printf("ERROR: %s failed: status %d (%s)\n", what, (int)st, msg ? msg : "?");
}

// Throws the A3DStatus of a failed call (caught once, at the top of the conversion).
#define CHECK(call) do { A3DStatus s_ = (call); if (s_ != A3D_SUCCESS) { hoops_usd::PrintStatus(#call, s_); throw s_; } } while (0)

// Loads and initialises HOOPS Exchange; terminates it on destruction.
class ExchangeSession
{
public:
    ExchangeSession(const char* binDir, const char* license)
    {
        if (!A3DSDKLoadLibraryA(binDir)) { printf("ERROR: cannot load HOOPS Exchange from %s\n", binDir); return; }
        loaded_ = true;
        A3DStatus st = A3DLicPutUnifiedLicense(license);
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

// ---------------------------------------------------------------------------------------------------------------------
// Matrices: GfMatrix4d is row-major with row vectors (p' = p * M, translation in row 3). PRC general transformations
// use the same 16 coefficients in the same order; a PRC cartesian transformation is rows X*sx, Y*sy, +-Z*sz, origin.
// ---------------------------------------------------------------------------------------------------------------------
inline bool IsIdentity(const GfMatrix4d& m, double tol = 1e-12)
{
    const GfMatrix4d id(1.0);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (std::fabs(m[i][j] - id[i][j]) > tol) return false;
    return true;
}

inline A3DMiscTransformation* ToPrcTransform(const GfMatrix4d& m)
{
    A3DMiscGeneralTransformationData td = A3D_MAKE_DATA(A3DMiscGeneralTransformationData);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) td.m_adCoeff[i * 4 + j] = m[i][j];
    A3DMiscGeneralTransformation* t = nullptr;
    CHECK(A3DMiscGeneralTransformationCreate(&td, &t));
    return t;
}

inline GfMatrix4d FromPrcTransform(const A3DMiscTransformation* t)
{
    GfMatrix4d m(1.0);
    if (!t) return m;
    A3DEEntityType type = kA3DTypeUnknown;
    A3DEntityGetType(t, &type);
    if (type == kA3DTypeMiscCartesianTransformation)
    {
        A3DMiscCartesianTransformationData d = A3D_MAKE_DATA(A3DMiscCartesianTransformationData);
        if (A3DMiscCartesianTransformationGet(t, &d) != A3D_SUCCESS) return m;
        GfVec3d X(d.m_sXVector.m_dX, d.m_sXVector.m_dY, d.m_sXVector.m_dZ);
        GfVec3d Y(d.m_sYVector.m_dX, d.m_sYVector.m_dY, d.m_sYVector.m_dZ);
        GfVec3d Z = GfCross(X, Y);
        if (d.m_ucBehaviour & kA3DTransformationMirror) Z = -Z;
        m.SetRow(0, GfVec4d(X[0] * d.m_sScale.m_dX, X[1] * d.m_sScale.m_dX, X[2] * d.m_sScale.m_dX, 0));
        m.SetRow(1, GfVec4d(Y[0] * d.m_sScale.m_dY, Y[1] * d.m_sScale.m_dY, Y[2] * d.m_sScale.m_dY, 0));
        m.SetRow(2, GfVec4d(Z[0] * d.m_sScale.m_dZ, Z[1] * d.m_sScale.m_dZ, Z[2] * d.m_sScale.m_dZ, 0));
        m.SetRow(3, GfVec4d(d.m_sOrigin.m_dX, d.m_sOrigin.m_dY, d.m_sOrigin.m_dZ, 1));
        A3DMiscCartesianTransformationGet(nullptr, &d);
    }
    else if (type == kA3DTypeMiscGeneralTransformation)
    {
        A3DMiscGeneralTransformationData d = A3D_MAKE_DATA(A3DMiscGeneralTransformationData);
        if (A3DMiscGeneralTransformationGet(t, &d) != A3D_SUCCESS) return m;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) m[i][j] = d.m_adCoeff[i * 4 + j];
        A3DMiscGeneralTransformationGet(nullptr, &d);
    }
    return m;
}

// Y-up -> Z-up: (x, y, z) -> (x, -z, y), a +90 degree rotation about X.
inline GfMatrix4d YUpToZUp()
{
    return GfMatrix4d(1, 0, 0, 0,
                      0, 0, 1, 0,
                      0, -1, 0, 0,
                      0, 0, 0, 1);
}

// ---------------------------------------------------------------------------------------------------------------------
// Units: PRC units are millimetres per model unit, USD metersPerUnit is metres per stage unit.
// ---------------------------------------------------------------------------------------------------------------------
inline double PrcUnitFromMetersPerUnit(double metersPerUnit) { return metersPerUnit * 1000.0; }
inline double MetersPerUnitFromPrcUnit(double mmPerUnit) { return mmPerUnit / 1000.0; }

// ---------------------------------------------------------------------------------------------------------------------
// Appearance
// ---------------------------------------------------------------------------------------------------------------------
inline constexpr A3DUns32 kNoStyle = 0xFFFFFFFFu;

struct Look
{
    enum Kind { None, Color, Material } kind = None;
    GfVec3d diffuse{0.18, 0.18, 0.18}, specular{0, 0, 0}, emissive{0, 0, 0};
    double shininess = 0, opacity = 1;

    std::string Key() const
    {
        return TfStringPrintf("%d|%.4f,%.4f,%.4f|%.4f,%.4f,%.4f|%.4f,%.4f,%.4f|%.3f|%.4f", (int)kind, diffuse[0], diffuse[1],
                              diffuse[2], specular[0], specular[1], specular[2], emissive[0], emissive[1], emissive[2],
                              shininess, opacity);
    }
};

// UsdPreviewSurface roughness (0 smooth .. 1 rough) <-> PRC shininess (0 .. 128, higher is shinier).
inline double ShininessFromRoughness(double r) { r = std::clamp(r, 0.0, 1.0); return 128.0 * (1 - r) * (1 - r); }
inline double RoughnessFromShininess(double s) { s = std::clamp(s, 0.0, 128.0); return 1.0 - std::sqrt(s / 128.0); }

// The look a PRC style resolves to (colour or material, transparency). Textures are reported as their material.
inline Look LookFromStyle(const A3DGraphStyleData& sd)
{
    Look l;
    auto rgb = [](A3DUns32 idx) {
        A3DGraphRgbColorData c = A3D_MAKE_DATA(A3DGraphRgbColorData);
        if (idx == A3D_DEFAULT_COLOR_INDEX || A3DGlobalGetGraphRgbColorData(idx, &c) != A3D_SUCCESS) return GfVec3d(0.75);
        return GfVec3d(c.m_dRed, c.m_dGreen, c.m_dBlue);
    };
    if (sd.m_bMaterial)
    {
        A3DBool isTexture = false;
        A3DGlobalIsMaterialTexture(sd.m_uiRgbColorIndex, &isTexture);
        A3DGraphMaterialData md = A3D_MAKE_DATA(A3DGraphMaterialData);
        if (!isTexture && A3DGlobalGetGraphMaterialData(sd.m_uiRgbColorIndex, &md) == A3D_SUCCESS)
        {
            l.kind = Look::Material;
            l.diffuse = rgb(md.m_uiDiffuse);
            l.specular = rgb(md.m_uiSpecular);
            l.emissive = rgb(md.m_uiEmissive);
            l.shininess = md.m_dShininess;
            l.opacity = md.m_dDiffuseAlpha;
        }
        else
            l.kind = Look::None;
    }
    else if (sd.m_uiRgbColorIndex != A3D_DEFAULT_COLOR_INDEX)
    {
        l.kind = Look::Color;
        l.diffuse = rgb(sd.m_uiRgbColorIndex);
    }
    if (sd.m_bIsTransparencyDefined) l.opacity = std::min(l.opacity, sd.m_ucTransparency / 255.0);
    return l;
}

// ---------------------------------------------------------------------------------------------------------------------
// Names and attributes
// ---------------------------------------------------------------------------------------------------------------------
inline const char* const kUserPropertiesNs = "userProperties:";   // PRC attributes <-> USD custom attributes
inline const char* const kHoopsNs = "hoops:";                     // data only meaningful to HOOPS (face ids, ...)

inline std::string ValidPrimName(const std::string& s)
{
    std::string n = TfMakeValidIdentifier(s);
    return n.empty() ? std::string("_") : n;
}

// Keeps sibling names unique: "Bolt", "Bolt_1", "Bolt_2", ...
class SiblingNames
{
public:
    std::string Unique(const std::string& wanted)
    {
        std::string base = ValidPrimName(wanted), n = base;
        for (int i = 1; used_.count(n); ++i) n = base + "_" + std::to_string(i);
        used_.insert(n);
        return n;
    }

private:
    std::set<std::string> used_;
};

inline std::string Lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return (char)std::tolower(c); });
    return s;
}

// ---------------------------------------------------------------------------------------------------------------------
// PRC tessellation decoding: every A3DTessFaceData layout -> triangles of {normal index, point index} corners.
// Indices are into A3DTess3DData::m_pdNormals and A3DTessBaseData::m_pdCoords (already multiplied by 3).
// ---------------------------------------------------------------------------------------------------------------------
struct Corner { A3DUns32 n, p; };
using TriangleSink = std::function<void(const Corner&, const Corner&, const Corner&)>;

// Returns false when the face uses a layout this decoder does not know (polyfaces, reserved for future use).
inline bool DecodeTessFace(const A3DTess3DData& td, A3DUns32 face, const TriangleSink& emit)
{
    const A3DTessFaceData& f = td.m_psFaceTessData[face];
    if (f.m_uiSizesTriangulatedSize == 0) return true;
    const A3DUns32* idx = td.m_puiTriangulatedIndexes + f.m_uiStartTriangulated;
    const A3DUns32* sizes = f.m_puiSizesTriangulated;
    A3DUns32 s = 0;
    const A3DUns32 T = f.m_uiTextureCoordIndexesSize;   // texture indices per point (textured layouts)
    const A3DUns16 fl = f.m_usUsedEntitiesFlags;
    if (fl & (kA3DTessFaceDataPolyface | kA3DTessFaceDataPolyfaceOneNormal | kA3DTessFaceDataPolyfaceTextured |
              kA3DTessFaceDataPolyfaceOneNormalTextured))
        return false;

    // A point with its own normal: {N, T..., P}. A point under a shared normal: {T..., P}.
    auto vtx = [&](const A3DUns32*& p, bool textured) { Corner c{p[0], p[1 + (textured ? T : 0)]}; p += 2 + (textured ? T : 0); return c; };
    auto pnt = [&](const A3DUns32*& p, A3DUns32 n, bool textured) { Corner c{n, p[textured ? T : 0]}; p += 1 + (textured ? T : 0); return c; };

    for (int pass = 0; pass < 2; ++pass)   // pass 0: untextured layouts, pass 1: textured (same order as the bits)
    {
        const bool tex = pass == 1;
        const A3DUns16 bTri = tex ? kA3DTessFaceDataTriangleTextured : kA3DTessFaceDataTriangle;
        const A3DUns16 bFan = tex ? kA3DTessFaceDataTriangleFanTextured : kA3DTessFaceDataTriangleFan;
        const A3DUns16 bStr = tex ? kA3DTessFaceDataTriangleStripeTextured : kA3DTessFaceDataTriangleStripe;
        const A3DUns16 bTri1 = tex ? kA3DTessFaceDataTriangleOneNormalTextured : kA3DTessFaceDataTriangleOneNormal;
        const A3DUns16 bFan1 = tex ? kA3DTessFaceDataTriangleFanOneNormalTextured : kA3DTessFaceDataTriangleFanOneNormal;
        const A3DUns16 bStr1 = tex ? kA3DTessFaceDataTriangleStripeOneNormalTextured : kA3DTessFaceDataTriangleStripeOneNormal;

        auto strip = [&](const std::vector<Corner>& v) {
            for (size_t i = 0; i + 2 < v.size(); ++i)
                if (i % 2 == 0) emit(v[i], v[i + 1], v[i + 2]);
                else emit(v[i + 1], v[i], v[i + 2]);
        };
        auto fan = [&](const std::vector<Corner>& v) {
            for (size_t i = 1; i + 1 < v.size(); ++i) emit(v[0], v[i], v[i + 1]);
        };
        // Fans / strips: count, then per entity its point count (OneNormal: kA3DTessFaceDataNormalSingle = one normal).
        auto entities = [&](bool oneNormalKind, bool isStrip) {
            const A3DUns32 count = sizes[s++] & kA3DTessFaceDataNormalMask;
            for (A3DUns32 e = 0; e < count; ++e)
            {
                const bool single = oneNormalKind && (sizes[s] & kA3DTessFaceDataNormalSingle);
                const A3DUns32 n = sizes[s++] & kA3DTessFaceDataNormalMask;
                std::vector<Corner> v;
                v.reserve(n);
                if (single) { const A3DUns32 nrm = *idx++; for (A3DUns32 k = 0; k < n; ++k) v.push_back(pnt(idx, nrm, tex)); }
                else for (A3DUns32 k = 0; k < n; ++k) v.push_back(vtx(idx, tex));
                isStrip ? strip(v) : fan(v);
            }
        };

        if (fl & bTri)
        {
            const A3DUns32 n = sizes[s++];
            for (A3DUns32 t = 0; t < n; ++t) { Corner a = vtx(idx, tex), b = vtx(idx, tex), c = vtx(idx, tex); emit(a, b, c); }
        }
        if (fl & bFan) entities(false, false);
        if (fl & bStr) entities(false, true);
        if (fl & bTri1)
        {
            const bool single = (sizes[s] & kA3DTessFaceDataNormalSingle) != 0;   // one normal for all triangles
            const A3DUns32 n = sizes[s++] & kA3DTessFaceDataNormalMask;
            A3DUns32 shared = single ? *idx++ : 0;
            for (A3DUns32 t = 0; t < n; ++t)
            {
                const A3DUns32 nrm = single ? shared : *idx++;
                Corner a = pnt(idx, nrm, tex), b = pnt(idx, nrm, tex), c = pnt(idx, nrm, tex);
                emit(a, b, c);
            }
        }
        if (fl & bFan1) entities(true, false);
        if (fl & bStr1) entities(true, true);
    }
    return true;
}

inline long long TessTriangles(const A3DTess3D* t)
{
    A3DTess3DData d = A3D_MAKE_DATA(A3DTess3DData);
    if (A3DTess3DGet(t, &d) != A3D_SUCCESS) return 0;
    long long n = 0;
    for (A3DUns32 f = 0; f < d.m_uiFaceTessSize; ++f) DecodeTessFace(d, f, [&](const Corner&, const Corner&, const Corner&) { ++n; });
    A3DTess3DGet(nullptr, &d);
    return n;
}

// ---------------------------------------------------------------------------------------------------------------------
// PRC product structure rules (HOOPS Exchange samples TreeTraverse.cpp / MeshViewer product_occurrence.cpp):
// an occurrence without its own part / children / location uses its prototype's (recursively), then external data.
// ---------------------------------------------------------------------------------------------------------------------
inline A3DAsmPartDefinition* EffectivePart(const A3DAsmProductOccurrence* po)
{
    A3DAsmProductOccurrenceData d = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
    if (!po || A3DAsmProductOccurrenceGet(po, &d) != A3D_SUCCESS) return nullptr;
    // Sample GetPart: own part, else the first part found along prototype / external links of the prototype chain,
    // else (no own children) the external data's part.
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

inline std::vector<A3DAsmProductOccurrence*> EffectiveChildren(const A3DAsmProductOccurrence* po)
{
    std::vector<A3DAsmProductOccurrence*> kids;
    A3DAsmProductOccurrenceData d = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
    if (!po || A3DAsmProductOccurrenceGet(po, &d) != A3D_SUCCESS) return kids;
    kids.assign(d.m_ppPOccurrences, d.m_ppPOccurrences + d.m_uiPOccurrencesSize);
    // Children of the prototype chain (sample CollectSons: the prototypes' own children only).
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

inline const A3DMiscTransformation* EffectiveLocation(const A3DAsmProductOccurrence* po)
{
    A3DAsmProductOccurrenceData d = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
    if (!po || A3DAsmProductOccurrenceGet(po, &d) != A3D_SUCCESS) return nullptr;
    const A3DMiscTransformation* loc = d.m_pLocation;
    if (!loc && d.m_pPrototype) loc = EffectiveLocation(d.m_pPrototype);
    else if (!loc && d.m_pExternalData) loc = EffectiveLocation(d.m_pExternalData);
    A3DAsmProductOccurrenceGet(nullptr, &d);
    return loc;
}

// ---------------------------------------------------------------------------------------------------------------------
// PRC inspection used by both tools' verification: tree dump, drawn triangles, unique meshes.
// ---------------------------------------------------------------------------------------------------------------------
inline std::string StyleText(A3DUns32 style)
{
    if (style == A3D_DEFAULT_STYLE_INDEX || style == kNoStyle) return "";
    A3DGraphStyleData sd = A3D_MAKE_DATA(A3DGraphStyleData);
    if (A3DGlobalGetGraphStyleData(style, &sd) != A3D_SUCCESS) return "";
    A3DUns32 colorIndex = sd.m_uiRgbColorIndex;
    std::string kind = "color";
    if (sd.m_bMaterial)
    {
        A3DGraphMaterialData md = A3D_MAKE_DATA(A3DGraphMaterialData);
        if (A3DGlobalGetGraphMaterialData(sd.m_uiRgbColorIndex, &md) != A3D_SUCCESS) return " material(?)";
        colorIndex = md.m_uiDiffuse;
        kind = "material";
    }
    A3DGraphRgbColorData cd = A3D_MAKE_DATA(A3DGraphRgbColorData);
    A3DGlobalGetGraphRgbColorData(colorIndex, &cd);
    std::string s = TfStringPrintf(" %s(%.3g,%.3g,%.3g)", kind.c_str(), cd.m_dRed, cd.m_dGreen, cd.m_dBlue);
    if (sd.m_bIsTransparencyDefined) s += TfStringPrintf(" alpha=%.3g", sd.m_ucTransparency / 255.0);
    return s;
}

inline bool EntityGraphics(const A3DEntity* e, A3DUns32& style, bool& shown)
{
    A3DRootBaseWithGraphicsData bd = A3D_MAKE_DATA(A3DRootBaseWithGraphicsData);
    if (A3DRootBaseWithGraphicsGet(e, &bd) != A3D_SUCCESS || !bd.m_pGraphics) return false;
    A3DGraphicsData gd = A3D_MAKE_DATA(A3DGraphicsData);
    if (A3DGraphicsGet(bd.m_pGraphics, &gd) != A3D_SUCCESS) return false;
    style = gd.m_uiStyleIndex;
    shown = (gd.m_usBehaviour & kA3DGraphicsShow) != 0;
    return true;
}

struct PrcContent
{
    long occurrences = 0;
    long long triangles = 0;
    std::set<const void*> uniqueTess;
};

// Tess3D representation items of a part, through RiSets.
inline void CollectTessItems(const A3DRiRepresentationItem* ri, std::vector<const A3DRiRepresentationItem*>& out)
{
    A3DEEntityType t = kA3DTypeUnknown;
    A3DEntityGetType(ri, &t);
    if (t == kA3DTypeRiSet)
    {
        A3DRiSetData sd = A3D_MAKE_DATA(A3DRiSetData);
        if (A3DRiSetGet(ri, &sd) == A3D_SUCCESS)
        {
            for (A3DUns32 i = 0; i < sd.m_uiRepItemsSize; ++i) CollectTessItems(sd.m_ppRepItems[i], out);
            A3DRiSetGet(nullptr, &sd);
        }
        return;
    }
    out.push_back(ri);
}

inline void InspectPrc(const A3DAsmProductOccurrence* po, int depth, bool dump, PrcContent& a)
{
    ++a.occurrences;
    A3DAsmProductOccurrenceData d = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
    if (A3DAsmProductOccurrenceGet(po, &d) != A3D_SUCCESS) return;
    const A3DAsmPartDefinition* part = EffectivePart(po);
    const std::vector<A3DAsmProductOccurrence*> kids = EffectiveChildren(po);

    std::string line;
    if (dump)
    {
        A3DRootBaseData bd = A3D_MAKE_DATA(A3DRootBaseData);
        A3DRootBaseGet(po, &bd);
        line = std::string(depth * 2, ' ') + (bd.m_pcName ? bd.m_pcName : "(unnamed)");
        if (d.m_pPrototype) line += " ->prototype";
        if (d.m_pLocation) line += " [xform]";
        A3DUns32 style = 0;
        bool shown = true;
        if (EntityGraphics(po, style, shown)) { if (!shown) line += " [hidden]"; line += StyleText(style); }
        for (A3DUns32 i = 0; i < bd.m_uiSize; ++i)
        {
            A3DMiscAttributeData ad = A3D_MAKE_DATA(A3DMiscAttributeData);
            if (A3DMiscAttributeGet(bd.m_ppAttributes[i], &ad) != A3D_SUCCESS) continue;
            for (A3DUns32 k = 0; k < ad.m_uiSize; ++k)
            {
                const A3DMiscSingleAttributeData& s = ad.m_asSingleAttributesData[k];
                std::string t = s.m_pcTitle ? s.m_pcTitle : "";
                if (t == "path" || t == "type") continue;   // keep the dump short
                line += " {" + t + "=" + (s.m_pcData ? s.m_pcData : "") + "}";
            }
            A3DMiscAttributeGet(nullptr, &ad);
        }
        A3DRootBaseGet(nullptr, &bd);
    }

    if (part)
    {
        A3DAsmPartDefinitionData pd = A3D_MAKE_DATA(A3DAsmPartDefinitionData);
        if (A3DAsmPartDefinitionGet(part, &pd) == A3D_SUCCESS)
        {
            std::vector<const A3DRiRepresentationItem*> items;
            for (A3DUns32 i = 0; i < pd.m_uiRepItemsSize; ++i) CollectTessItems(pd.m_ppRepItems[i], items);
            for (const A3DRiRepresentationItem* item : items)
            {
                A3DRiRepresentationItemData rd = A3D_MAKE_DATA(A3DRiRepresentationItemData);
                if (A3DRiRepresentationItemGet(item, &rd) != A3D_SUCCESS) continue;
                A3DEEntityType et = kA3DTypeUnknown;
                if (rd.m_pTessBase && A3DEntityGetType(rd.m_pTessBase, &et) == A3D_SUCCESS && et == kA3DTypeTess3D)
                {
                    const long long n = TessTriangles(rd.m_pTessBase);
                    a.triangles += n;
                    if (dump && !a.uniqueTess.count(rd.m_pTessBase))
                    {
                        line += TfStringPrintf(" | mesh %lld triangles", n);
                        A3DUns32 st = 0;
                        bool sh = true;
                        if (EntityGraphics(item, st, sh)) line += StyleText(st);
                        A3DTess3DData td = A3D_MAKE_DATA(A3DTess3DData);
                        if (A3DTess3DGet(rd.m_pTessBase, &td) == A3D_SUCCESS)
                        {
                            std::set<std::string> faceStyles;
                            bool vcol = false;
                            for (A3DUns32 f = 0; f < td.m_uiFaceTessSize; ++f)
                            {
                                const A3DTessFaceData& fd = td.m_psFaceTessData[f];
                                if (fd.m_uiStyleIndexesSize > 0) faceStyles.insert(StyleText(fd.m_puiStyleIndexes[0]));
                                if (fd.m_uiRGBAVerticesSize > 0) vcol = true;
                            }
                            if (!faceStyles.empty())
                            {
                                line += " faces:";
                                for (const std::string& fs_ : faceStyles) line += fs_;
                            }
                            if (vcol) line += " vertex-colors";
                            A3DTess3DGet(nullptr, &td);
                        }
                    }
                    else if (dump) line += TfStringPrintf(" | shared mesh %lld triangles", n);
                    a.uniqueTess.insert(rd.m_pTessBase);
                }
                A3DRiRepresentationItemGet(nullptr, &rd);
            }
            A3DAsmPartDefinitionGet(nullptr, &pd);
        }
    }
    if (dump) printf("%s\n", line.c_str());
    for (const A3DAsmProductOccurrence* k : kids) InspectPrc(k, depth + 1, dump, a);
    A3DAsmProductOccurrenceGet(nullptr, &d);
}

// World bounding box of the visible tessellation (A3DMiscComputeBoundingBox), with tiny values snapped to 0 for
// printing ("-6e-17" -> 0).
struct Box { GfVec3d min{0.0}, max{0.0}; bool ok = false; };

inline Box PrcBoundingBox(const A3DAsmModelFile* mf)
{
    Box b;
    A3DBoundingBoxData bb = A3D_MAKE_DATA(A3DBoundingBoxData);
    A3DMiscComputeBoundingBoxOptions bo = A3D_MAKE_DATA(A3DMiscComputeBoundingBoxOptions);
    b.ok = A3DMiscComputeBoundingBox(mf, &bo, &bb) == A3D_SUCCESS && bb.m_sMin.m_dX <= bb.m_sMax.m_dX;
    b.min = GfVec3d(bb.m_sMin.m_dX, bb.m_sMin.m_dY, bb.m_sMin.m_dZ);
    b.max = GfVec3d(bb.m_sMax.m_dX, bb.m_sMax.m_dY, bb.m_sMax.m_dZ);
    return b;
}

inline std::string BoxText(const Box& b)
{
    double scale = 1.0;
    for (int i = 0; i < 3; ++i) scale = std::max({scale, std::fabs(b.min[i]), std::fabs(b.max[i])});
    auto snap = [&](double v) { return std::fabs(v) < 1e-9 * scale ? 0.0 : v; };
    return TfStringPrintf("(%.6g %.6g %.6g)-(%.6g %.6g %.6g)", snap(b.min[0]), snap(b.min[1]), snap(b.min[2]), snap(b.max[0]),
                          snap(b.max[1]), snap(b.max[2]));
}

inline double BoxDeviation(const Box& a, const Box& b)
{
    double dev = 0;
    for (int i = 0; i < 3; ++i) dev = std::max({dev, std::fabs(a.min[i] - b.min[i]), std::fabs(a.max[i] - b.max[i])});
    return dev;
}

}   // namespace hoops_usd
