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
/// \example USD_Reader
/// OpenUSD (.usd/.usda/.usdc/.usdz) to PRC converter using OpenUSD and HOOPS Exchange.
///
/// 1. open the stage with the OpenUSD toolkit
/// 2. create the PRC product structure, meshes, materials and attributes
/// 3. A3DAsmModelFileExportToPrcFile, then re-import and verify
////////////////////////////////////////////////////////////////////////////////

/***********************************************************************************************************************
 * USD_Reader — converts an OpenUSD stage (.usd / .usda / .usdc / .usdz) to PRC with HOOPS Exchange.
 *
 *   USD is read with the OpenUSD toolkit (UsdStage, UsdGeom, UsdShade); the PRC model file is built with the HOOPS
 *   Exchange public API (A3DTess3DCreate, A3DRiPolyBrepModelCreate, A3DAsmPartDefinitionCreate,
 *   A3DAsmProductOccurrenceCreate, A3DAsmModelFileCreate) and written with A3DAsmModelFileExportToPrcFile.
 *
 *   USD                                   PRC
 *   ------------------------------------  ---------------------------------------------------------------------------
 *   stage (upAxis, metersPerUnit)         model file unit (mm) + root product occurrence (Y-up turned to Z-up)
 *   Xform / Scope / any prim              product occurrence (name, local transform, attributes)
 *   Mesh                                  part -> A3DRiPolyBrepModel -> A3DTess3D (polygons triangulated)
 *   Cube Sphere Cylinder(_1) Cone         same, tessellated here (--segments)
 *   Capsule(_1) Plane
 *   displayColor / displayOpacity         styles (constant), per-face styles (uniform), vertex colours (vertex/faceVarying)
 *   UsdPreviewSurface / standard_surface  PRC materials; GeomSubset material bindings become per-face materials
 *   instanceable prims (native instancing) one shared prototype product occurrence, one occurrence per instance
 *   PointInstancer                        one prototype occurrence per prototype, one occurrence per instance
 *   variant sets                          the composed selection (authored default, or --variant overrides)
 *   visibility / purpose                  hidden occurrences / proxy and guide prims skipped (options)
 *   custom attributes, kind, prim type    PRC attributes on the occurrence
 *
 *   Grounded in: HOOPS Exchange 2026.8 samples CreatePRCCubes (CreateModelFile.cpp, CreateTessellatedCubes.cpp,
 *   modules/CreateGraphics.cpp), headers A3DSDKTessellation.h, A3DSDKStructure.h, A3DSDKGraphics.h, A3DSDKMisc.h,
 *   A3DSDKRootEntities.h; OpenUSD 25.08 headers (usdGeom, usdShade).
 ***********************************************************************************************************************/
#define INITIALIZE_A3D_API
#include <A3DSDKIncludes.h>
#include <hoops_license.h>

#include <pxr/pxr.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/range3d.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec4f.h>
#include <pxr/base/tf/errorMark.h>
#include <pxr/base/tf/stringUtils.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/vt/types.h>
#include <pxr/base/vt/value.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/usd/editContext.h>
#include <pxr/usd/usd/modelAPI.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/variantSets.h>
#include <pxr/usd/usdGeom/capsule.h>
#include <pxr/usd/usdGeom/capsule_1.h>
#include <pxr/usd/usdGeom/cone.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/cylinder_1.h>
#include <pxr/usd/usdGeom/gprim.h>
#include <pxr/usd/usdGeom/imageable.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/plane.h>
#include <pxr/usd/usdGeom/pointInstancer.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/subset.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xformCache.h>
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>
#include <pxr/usd/usdShade/nodeGraph.h>
#include <pxr/usd/usdShade/shader.h>
#include <pxr/usd/usdShade/utils.h>

#include "hoops_usd.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace hoops_usd;
namespace fs = std::filesystem;


// ---------------------------------------------------------------------------------------------------------------------
// Options, logging
// ---------------------------------------------------------------------------------------------------------------------
struct VariantOverride { std::string primPath, set, selection; };

struct Options
{
    std::string input, output;
    bool force = false, quiet = false, verbose = false, dumpTree = false, skipHidden = false;
    bool includeProxy = false, includeGuide = false, keepUpAxis = false;
    bool timeGiven = false;
    double time = 0;
    int segments = 32;                      // around the axis of revolved primitives (multiple of 4)
    std::vector<VariantOverride> variants;
};





static void PrintUsage()
{
    printf("Usage: USD_Reader <input.usd|usda|usdc|usdz> <output.prc> [options]\n"
           "Converts an OpenUSD stage to PRC: hierarchy, meshes, primitives, colours, materials, instancing,\n"
           "point instancers, variants, visibility and custom attributes.\n"
           "Options:\n"
           "  --force                  overwrite <output.prc> if it exists\n"
           "  --variant <path>{set=v}  select variant v of variant set 'set' on prim <path> (repeatable),\n"
           "                           e.g. --variant \"/World/Bolt{size=large}\"; default: the authored selections\n"
           "  --time <t>               time code to read (default: the earliest authored sample / default values)\n"
           "  --skip-hidden            do not convert invisible prims (default: converted as hidden occurrences)\n"
           "  --include-proxy          also convert prims with purpose 'proxy' (default: skipped)\n"
           "  --include-guide          also convert prims with purpose 'guide' (default: skipped)\n"
           "  --keep-up-axis           keep a Y-up stage Y-up (default: rotated to Z-up, the CAD convention)\n"
           "  --segments <n>           facets around the axis of Sphere/Cylinder/Cone/Capsule (multiple of 4, default 32)\n"
           "  --dump-tree              print the product structure read back from the written PRC\n"
           "  --quiet                  print only warnings, errors and the summary\n"
           "  --verbose                also print the HOOPS Exchange reader/writer messages\n"
           "Exit codes: 0 ok, 1 usage, 2 input/read error, 3 unsupported format, 4 PRC creation/export failed,\n"
           "            5 verification of the written PRC failed\n");
}

// ---------------------------------------------------------------------------------------------------------------------
// Small USD helpers
// ---------------------------------------------------------------------------------------------------------------------
static UsdTimeCode g_time = UsdTimeCode::EarliestTime();

template <class T> static T AttrOr(const UsdAttribute& a, const T& def)
{
    T v;
    return (a && a.Get(&v, g_time)) ? v : def;
}

static bool ToVec3(const VtValue& v, GfVec3d& out)
{
    if (v.IsHolding<GfVec3f>()) { out = GfVec3d(v.UncheckedGet<GfVec3f>()); return true; }
    if (v.IsHolding<GfVec3d>()) { out = v.UncheckedGet<GfVec3d>(); return true; }
    if (v.IsHolding<GfVec4f>()) { GfVec4f c = v.UncheckedGet<GfVec4f>(); out = GfVec3d(c[0], c[1], c[2]); return true; }
    if (v.IsHolding<float>()) { double f = v.UncheckedGet<float>(); out = GfVec3d(f, f, f); return true; }
    if (v.IsHolding<double>()) { double f = v.UncheckedGet<double>(); out = GfVec3d(f, f, f); return true; }
    return false;
}

static bool ToScalar(const VtValue& v, double& out)
{
    if (v.IsHolding<float>()) { out = v.UncheckedGet<float>(); return true; }
    if (v.IsHolding<double>()) { out = v.UncheckedGet<double>(); return true; }
    if (v.IsHolding<int>()) { out = v.UncheckedGet<int>(); return true; }
    if (v.IsHolding<GfVec3f>()) { GfVec3f c = v.UncheckedGet<GfVec3f>(); out = (c[0] + c[1] + c[2]) / 3; return true; }
    return false;
}

static std::string ValueToString(const VtValue& v)
{
    if (v.IsHolding<std::string>()) return v.UncheckedGet<std::string>();
    if (v.IsHolding<TfToken>()) return v.UncheckedGet<TfToken>().GetString();
    if (v.IsHolding<SdfAssetPath>()) return v.UncheckedGet<SdfAssetPath>().GetAssetPath();
    std::ostringstream os;
    os << v;
    std::string s = os.str();
    if (s.size() > 1000) s = s.substr(0, 1000) + "...";
    return s;
}


// ---------------------------------------------------------------------------------------------------------------------
// Appearance: a resolved look becomes a global PRC style (deduplicated)
// ---------------------------------------------------------------------------------------------------------------------


struct Report
{
    std::map<std::string, long> primTypes, skipped;
    long occurrences = 0, meshes = 0, primitives = 0, instances = 0, pointInstances = 0, hidden = 0;
    long prototypes = 0, materials = 0, styles = 0, texturedInputs = 0, emptyDropped = 0, attributes = 0;
    long badFaces = 0;
    long long uniqueTriangles = 0;
};

class StyleTable
{
public:
    explicit StyleTable(Report& r) : rep_(r) {}

    A3DUns32 Get(const Look& l)
    {
        if (l.kind == Look::None) return kNoStyle;
        const std::string key = l.Key();
        auto it = cache_.find(key);
        if (it != cache_.end()) return it->second;

        A3DGraphStyleData sd = A3D_MAKE_DATA(A3DGraphStyleData);
        sd.m_dWidth = 0.1;
        if (l.kind == Look::Color)
        {
            sd.m_bMaterial = false;
            sd.m_uiRgbColorIndex = Rgb(l.diffuse);
        }
        else
        {
            A3DGraphMaterialData md = A3D_MAKE_DATA(A3DGraphMaterialData);
            md.m_uiAmbient = Rgb(l.diffuse * 0.2);
            md.m_uiDiffuse = Rgb(l.diffuse);
            md.m_uiSpecular = Rgb(l.specular);
            md.m_uiEmissive = Rgb(l.emissive);
            md.m_dAmbientAlpha = md.m_dDiffuseAlpha = md.m_dSpecularAlpha = md.m_dEmissiveAlpha = l.opacity;
            md.m_dShininess = l.shininess;
            A3DUns32 mi = 0;
            CHECK(A3DGlobalInsertGraphMaterial(&md, &mi));
            sd.m_bMaterial = true;
            sd.m_uiRgbColorIndex = mi;
            ++rep_.materials;
        }
        if (l.opacity < 1.0)
        {
            sd.m_bIsTransparencyDefined = true;
            sd.m_ucTransparency = (A3DUns8)std::lround(std::clamp(l.opacity, 0.0, 1.0) * 255.0);
        }
        A3DUns32 si = 0;
        CHECK(A3DGlobalInsertGraphStyle(&sd, &si));
        ++rep_.styles;
        cache_[key] = si;
        return si;
    }

private:
    A3DUns32 Rgb(const GfVec3d& c)
    {
        A3DGraphRgbColorData cd = A3D_MAKE_DATA(A3DGraphRgbColorData);
        cd.m_dRed = std::clamp(c[0], 0.0, 1.0);
        cd.m_dGreen = std::clamp(c[1], 0.0, 1.0);
        cd.m_dBlue = std::clamp(c[2], 0.0, 1.0);
        A3DUns32 idx = 0;
        CHECK(A3DGlobalInsertGraphRgbColor(&cd, &idx));
        return idx;
    }
    std::map<std::string, A3DUns32> cache_;
    Report& rep_;
};

static A3DGraphics* MakeGraphics(A3DUns32 style, bool shown)
{
    A3DGraphicsData gd = A3D_MAKE_DATA(A3DGraphicsData);
    gd.m_uiLayerIndex = A3D_DEFAULT_LAYER;
    gd.m_uiStyleIndex = style == kNoStyle ? A3D_DEFAULT_STYLE_INDEX : style;
    gd.m_usBehaviour = shown ? kA3DGraphicsShow : 0;
    gd.m_usBehaviour |= kA3DGraphicsSonHeritShow;
    if (style != kNoStyle) gd.m_usBehaviour |= kA3DGraphicsSonHeritColor;
    A3DGraphics* g = nullptr;
    CHECK(A3DGraphicsCreate(&gd, &g));
    return g;
}

static void SetGraphics(A3DEntity* e, A3DUns32 style, bool shown)
{
    A3DRootBaseWithGraphicsData bd = A3D_MAKE_DATA(A3DRootBaseWithGraphicsData);
    bd.m_pGraphics = MakeGraphics(style, shown);
    CHECK(A3DRootBaseWithGraphicsSet(e, &bd));
}

struct AttrKV { std::string key, value; };

static void SetNameAndAttributes(A3DEntity* e, const std::string& name, const std::string& title,
                                 const std::vector<AttrKV>& kvs, Report& rep)
{
    std::vector<A3DMiscSingleAttributeData> singles;
    for (const AttrKV& kv : kvs)
    {
        A3DMiscSingleAttributeData s = A3D_MAKE_DATA(A3DMiscSingleAttributeData);
        s.m_bTitleIsInt = false;
        s.m_pcTitle = const_cast<A3DUTF8Char*>(kv.key.c_str());
        s.m_eType = kA3DModellerAttributeTypeString;
        s.m_pcData = const_cast<A3DUTF8Char*>(kv.value.c_str());
        singles.push_back(s);
    }
    A3DMiscAttribute* attr = nullptr;
    if (!singles.empty())
    {
        A3DMiscAttributeData ad = A3D_MAKE_DATA(A3DMiscAttributeData);
        ad.m_bTitleIsInt = false;
        ad.m_pcTitle = const_cast<A3DUTF8Char*>(title.c_str());
        ad.m_uiSize = (A3DUns32)singles.size();
        ad.m_asSingleAttributesData = singles.data();
        CHECK(A3DMiscAttributeCreate(&ad, &attr));
        rep.attributes += (long)singles.size();
    }
    A3DRootBaseData bd = A3D_MAKE_DATA(A3DRootBaseData);
    bd.m_pcName = const_cast<A3DUTF8Char*>(name.c_str());
    if (attr) { bd.m_uiSize = 1; bd.m_ppAttributes = &attr; }
    CHECK(A3DRootBaseSet(e, &bd));
    if (attr) A3DEntityDelete(attr);
}


// ---------------------------------------------------------------------------------------------------------------------
// Polygon mesh (USD side) -> triangle mesh (PRC side)
// ---------------------------------------------------------------------------------------------------------------------
struct PolyMesh
{
    std::vector<GfVec3d> P;
    std::vector<int> counts, idx;
    std::vector<GfVec3d> N;
    TfToken normalInterp;          // empty: compute; vertex / faceVarying / uniform / constant
    bool leftHanded = false, smooth = true;
    std::set<int> holes;
};

struct TriGroup                    // one PRC tessellated face: triangles sharing a style
{
    A3DUns32 style = kNoStyle;
    std::vector<A3DUns32> idx;     // {normal*3, point*3} per corner
    std::vector<A3DUns8> rgba;     // 4 per corner, or empty
};

struct TriMesh
{
    std::vector<double> coords, normals;
    std::vector<TriGroup> groups;
    long long Triangles() const
    {
        long long n = 0;
        for (const TriGroup& g : groups) n += (long long)g.idx.size() / 6;
        return n;
    }
};

// Per-face style and optional per-corner colour (corner = face-vertex index into idx).
struct FaceAppearance
{
    std::function<A3DUns32(int face)> faceStyle;
    std::function<bool(int face, int corner, int point, A3DUns8 rgba[4])> cornerColor;   // may be empty
};

static GfVec3d PolygonNormal(const PolyMesh& m, int start, int count)
{
    GfVec3d n(0, 0, 0);   // Newell
    for (int k = 0; k < count; ++k)
    {
        const GfVec3d& a = m.P[m.idx[start + k]];
        const GfVec3d& b = m.P[m.idx[start + (k + 1) % count]];
        n[0] += (a[1] - b[1]) * (a[2] + b[2]);
        n[1] += (a[2] - b[2]) * (a[0] + b[0]);
        n[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    return n;
}

static bool Triangulate(const PolyMesh& m, const FaceAppearance& app, TriMesh& out, Report& rep)
{
    const int nPts = (int)m.P.size();
    const int nFaces = (int)m.counts.size();
    out.coords.reserve(m.P.size() * 3);
    for (const GfVec3d& p : m.P) { out.coords.push_back(p[0]); out.coords.push_back(p[1]); out.coords.push_back(p[2]); }

    // Normals: choose the array and how a corner maps into it.
    enum { kVertex, kCorner, kFace, kConst } nmode = kVertex;
    std::vector<GfVec3d> normals;
    size_t nCorners = 0;
    for (int c : m.counts) nCorners += (size_t)std::max(c, 0);
    const TfToken& ni = m.normalInterp;
    if (!ni.IsEmpty() && ((ni == UsdGeomTokens->vertex || ni == UsdGeomTokens->varying) && m.N.size() == m.P.size()))
    { normals = m.N; nmode = kVertex; }
    else if (!ni.IsEmpty() && ni == UsdGeomTokens->faceVarying && m.N.size() == nCorners) { normals = m.N; nmode = kCorner; }
    else if (!ni.IsEmpty() && ni == UsdGeomTokens->uniform && m.N.size() == (size_t)nFaces) { normals = m.N; nmode = kFace; }
    else if (!ni.IsEmpty() && ni == UsdGeomTokens->constant && !m.N.empty()) { normals.assign(1, m.N[0]); nmode = kConst; }
    else
    {
        // No usable authored normals: smooth (area-weighted) for subdivision surfaces, flat for "none".
        const double sign = m.leftHanded ? -1.0 : 1.0;
        if (m.smooth)
        {
            normals.assign(m.P.size(), GfVec3d(0, 0, 0));
            int start = 0;
            for (int f = 0; f < nFaces; start += std::max(m.counts[f], 0), ++f)
            {
                if (m.counts[f] < 3) continue;
                bool ok = true;
                for (int k = 0; k < m.counts[f]; ++k) if (m.idx[start + k] < 0 || m.idx[start + k] >= nPts) ok = false;
                if (!ok) continue;
                GfVec3d fn = PolygonNormal(m, start, m.counts[f]) * sign;
                for (int k = 0; k < m.counts[f]; ++k) normals[m.idx[start + k]] += fn;
            }
            nmode = kVertex;
        }
        else
        {
            normals.assign(nFaces, GfVec3d(0, 0, 1));
            int start = 0;
            for (int f = 0; f < nFaces; start += std::max(m.counts[f], 0), ++f)
            {
                if (m.counts[f] < 3) continue;
                bool ok = true;
                for (int k = 0; k < m.counts[f]; ++k) if (m.idx[start + k] < 0 || m.idx[start + k] >= nPts) ok = false;
                if (ok) normals[f] = PolygonNormal(m, start, m.counts[f]) * sign;
            }
            nmode = kFace;
        }
    }
    out.normals.reserve(normals.size() * 3);
    for (GfVec3d n : normals)
    {
        double len = n.GetLength();
        if (len > 1e-30) n /= len; else n = GfVec3d(0, 0, 1);
        out.normals.push_back(n[0]); out.normals.push_back(n[1]); out.normals.push_back(n[2]);
    }

    std::map<A3DUns32, size_t> groupOf;
    bool anyColor = false;
    int start = 0;
    for (int f = 0; f < nFaces; start += std::max(m.counts[f], 0), ++f)
    {
        const int c = m.counts[f];
        if (c < 3 || start + c > (int)m.idx.size()) { if (c > 0 && c < 3) ++rep.badFaces; continue; }
        if (m.holes.count(f)) continue;
        bool ok = true;
        for (int k = 0; k < c; ++k) if (m.idx[start + k] < 0 || m.idx[start + k] >= nPts) ok = false;
        if (!ok) { ++rep.badFaces; continue; }

        A3DUns32 style = app.faceStyle ? app.faceStyle(f) : kNoStyle;
        auto git = groupOf.find(style);
        if (git == groupOf.end()) { git = groupOf.emplace(style, out.groups.size()).first; out.groups.emplace_back(); out.groups.back().style = style; }
        TriGroup& g = out.groups[git->second];

        auto corner = [&](int k) {
            const int pi = m.idx[start + k];
            A3DUns32 nIdx = 0;
            switch (nmode)
            {
            case kVertex: nIdx = (A3DUns32)pi; break;
            case kCorner: nIdx = (A3DUns32)(start + k); break;
            case kFace: nIdx = (A3DUns32)f; break;
            case kConst: nIdx = 0; break;
            }
            g.idx.push_back(nIdx * 3);
            g.idx.push_back((A3DUns32)pi * 3);
            A3DUns8 col[4] = {255, 255, 255, 255};
            if (app.cornerColor && app.cornerColor(f, start + k, pi, col)) anyColor = true;
            g.rgba.insert(g.rgba.end(), col, col + 4);
        };
        for (int k = 1; k + 1 < c; ++k)   // fan triangulation (convex / star-shaped polygons)
        {
            corner(0);
            if (m.leftHanded) { corner(k + 1); corner(k); }
            else { corner(k); corner(k + 1); }
        }
    }
    if (!anyColor) for (TriGroup& g : out.groups) g.rgba.clear();
    out.groups.erase(std::remove_if(out.groups.begin(), out.groups.end(), [](const TriGroup& g) { return g.idx.empty(); }),
                     out.groups.end());
    if (out.groups.empty()) return false;

    // Keep only the points the triangles use (holes, unused points); per-vertex normals follow their points.
    std::vector<A3DUns32> remap(m.P.size(), 0xFFFFFFFFu);
    std::vector<double> coords;
    std::vector<double> vnormals;
    for (TriGroup& g : out.groups)
        for (size_t k = 1; k < g.idx.size(); k += 2)
        {
            A3DUns32& r = remap[g.idx[k] / 3];
            if (r == 0xFFFFFFFFu)
            {
                const size_t o = g.idx[k];
                r = (A3DUns32)(coords.size() / 3);
                coords.insert(coords.end(), out.coords.begin() + o, out.coords.begin() + o + 3);
                if (nmode == kVertex) vnormals.insert(vnormals.end(), out.normals.begin() + o, out.normals.begin() + o + 3);
            }
            if (nmode == kVertex) g.idx[k - 1] = r * 3;
            g.idx[k] = r * 3;
        }
    out.coords.swap(coords);
    if (nmode == kVertex) out.normals.swap(vnormals);
    return true;
}

static A3DRiPolyBrepModel* CreatePolyBrep(const TriMesh& tm, A3DUns32 riStyle, bool shown)
{
    std::vector<A3DUns32> tri;
    std::vector<A3DTessFaceData> faces;
    std::vector<A3DUns32> triCounts(tm.groups.size()), styles(tm.groups.size());
    for (size_t i = 0; i < tm.groups.size(); ++i)
    {
        const TriGroup& g = tm.groups[i];
        A3DTessFaceData fd;
        A3D_INITIALIZE_DATA(A3DTessFaceData, fd);
        fd.m_usUsedEntitiesFlags = kA3DTessFaceDataTriangle;
        fd.m_uiStartTriangulated = (A3DUns32)tri.size();
        triCounts[i] = (A3DUns32)(g.idx.size() / 6);
        fd.m_uiSizesTriangulatedSize = 1;
        fd.m_puiSizesTriangulated = &triCounts[i];
        fd.m_uiTextureCoordIndexesSize = 0;
        if (g.style != kNoStyle)
        {
            styles[i] = g.style;
            fd.m_uiStyleIndexesSize = 1;
            fd.m_puiStyleIndexes = &styles[i];
        }
        if (!g.rgba.empty())
        {
            fd.m_bIsRGBA = true;
            fd.m_uiRGBAVerticesSize = (A3DUns32)g.rgba.size();
            fd.m_pucRGBAVertices = const_cast<A3DUns8*>(g.rgba.data());
        }
        tri.insert(tri.end(), g.idx.begin(), g.idx.end());
        faces.push_back(fd);
    }

    A3DTess3DData td;
    A3D_INITIALIZE_DATA(A3DTess3DData, td);
    td.m_bHasFaces = true;
    td.m_uiNormalSize = (A3DUns32)tm.normals.size();
    td.m_pdNormals = const_cast<A3DDouble*>(tm.normals.data());
    td.m_uiTriangulatedIndexSize = (A3DUns32)tri.size();
    td.m_puiTriangulatedIndexes = tri.data();
    td.m_uiFaceTessSize = (A3DUns32)faces.size();
    td.m_psFaceTessData = faces.data();
    A3DTess3D* tess = nullptr;
    CHECK(A3DTess3DCreate(&td, &tess));

    A3DTessBaseData bd;
    A3D_INITIALIZE_DATA(A3DTessBaseData, bd);
    bd.m_bIsCalculated = false;
    bd.m_uiCoordSize = (A3DUns32)tm.coords.size();
    bd.m_pdCoords = const_cast<A3DDouble*>(tm.coords.data());
    CHECK(A3DTessBaseSet(tess, &bd));

    A3DRiPolyBrepModelData pd = A3D_MAKE_DATA(A3DRiPolyBrepModelData);
    pd.m_bIsClosed = false;
    A3DRiPolyBrepModel* ri = nullptr;
    CHECK(A3DRiPolyBrepModelCreate(&pd, &ri));
    A3DRiRepresentationItemData rd = A3D_MAKE_DATA(A3DRiRepresentationItemData);
    rd.m_pTessBase = tess;
    rd.m_pCoordinateSystem = nullptr;
    CHECK(A3DRiRepresentationItemSet(ri, &rd));
    if (riStyle != kNoStyle || !shown) SetGraphics(ri, riStyle, shown);
    return ri;
}

// ---------------------------------------------------------------------------------------------------------------------
// Tessellation of the USD intrinsic primitives (local frame: spine along w, then mapped to the prim's axis)
// ---------------------------------------------------------------------------------------------------------------------
struct ProfilePt { double r, z, nr, nz; };   // ring radius / height and the outward normal (radial, axial)

static void AddRevolved(PolyMesh& m, const std::vector<ProfilePt>& prof, int seg)
{
    // Rings of 'seg' points; rings of radius 0 still get 'seg' coincident points with their own normals so the
    // triangles that touch a pole keep a correct normal per corner.
    std::vector<int> ringStart;
    for (const ProfilePt& q : prof)
    {
        ringStart.push_back((int)m.P.size());
        for (int j = 0; j < seg; ++j)
        {
            double a = 2.0 * M_PI * j / seg, ca = std::cos(a), sa = std::sin(a);
            m.P.emplace_back(q.r * ca, q.r * sa, q.z);
            m.N.emplace_back(q.nr * ca, q.nr * sa, q.nz);
        }
    }
    for (size_t i = 0; i + 1 < prof.size(); ++i)
    {
        const bool lowPole = prof[i].r <= 0, highPole = prof[i + 1].r <= 0;
        if (lowPole && highPole) continue;
        for (int j = 0; j < seg; ++j)
        {
            int a = ringStart[i] + j, b = ringStart[i] + (j + 1) % seg;
            int c = ringStart[i + 1] + (j + 1) % seg, d = ringStart[i + 1] + j;
            if (lowPole) { m.counts.push_back(3); m.idx.insert(m.idx.end(), {a, c, d}); }
            else if (highPole) { m.counts.push_back(3); m.idx.insert(m.idx.end(), {a, b, d}); }
            else { m.counts.push_back(4); m.idx.insert(m.idx.end(), {a, b, c, d}); }
        }
    }
}

static void AddDisk(PolyMesh& m, double r, double z, bool up, int seg)
{
    if (r <= 0) return;
    int s = (int)m.P.size();
    for (int j = 0; j < seg; ++j)
    {
        double a = 2.0 * M_PI * j / seg;
        m.P.emplace_back(r * std::cos(a), r * std::sin(a), z);
        m.N.emplace_back(0, 0, up ? 1 : -1);
    }
    m.counts.push_back(seg);
    for (int j = 0; j < seg; ++j) m.idx.push_back(up ? s + j : s + (seg - j) % seg);
}

static void AddSphereBand(std::vector<ProfilePt>& prof, double r, double zc, double phi0, double phi1, int steps, bool skipFirst)
{
    for (int i = skipFirst ? 1 : 0; i <= steps; ++i)
    {
        double phi = phi0 + (phi1 - phi0) * i / steps, cp = std::cos(phi), sp = std::sin(phi);
        if (std::fabs(cp) < 1e-12) cp = 0;   // exact poles (a zero-radius ring becomes a triangle fan)
        if (std::fabs(sp) < 1e-12) sp = 0;   // exact equator
        prof.push_back({r * cp, zc + r * sp, cp, sp});
    }
}

static void MapAxis(PolyMesh& m, const TfToken& axis)
{
    // (u, v, w) with w along the spine -> cyclic permutation keeps the orientation.
    auto map = [&](GfVec3d& p) {
        if (axis == UsdGeomTokens->x) p = GfVec3d(p[2], p[0], p[1]);
        else if (axis == UsdGeomTokens->y) p = GfVec3d(p[1], p[2], p[0]);
    };
    for (GfVec3d& p : m.P) map(p);
    for (GfVec3d& n : m.N) map(n);
}

static void FinishCornerNormals(PolyMesh& m)
{
    // The generators store one normal per point; corners reference points, so the normals are per vertex.
    m.normalInterp = UsdGeomTokens->vertex;
}

static bool TessellatePrimitive(const UsdPrim& p, int seg, PolyMesh& m, std::string& type)
{
    const int rings = std::max(2, seg / 2);   // even: the equator is a ring, so extents are exact
    if (UsdGeomCube cube{p})
    {
        type = "Cube";
        double h = AttrOr<double>(cube.GetSizeAttr(), 2.0) / 2;
        static const int F[6][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}};
        static const double Nf[6][3] = {{0, 0, -1}, {0, 0, 1}, {0, -1, 0}, {1, 0, 0}, {0, 1, 0}, {-1, 0, 0}};
        const GfVec3d C[8] = {{-h, -h, -h}, {h, -h, -h}, {h, h, -h}, {-h, h, -h}, {-h, -h, h}, {h, -h, h}, {h, h, h}, {-h, h, h}};
        for (int f = 0; f < 6; ++f)
        {
            m.counts.push_back(4);
            for (int k = 0; k < 4; ++k)
            {
                m.idx.push_back((int)m.P.size());
                m.P.push_back(C[F[f][k]]);
                m.N.emplace_back(Nf[f][0], Nf[f][1], Nf[f][2]);
            }
        }
        FinishCornerNormals(m);
        return true;
    }
    if (UsdGeomSphere s{p})
    {
        type = "Sphere";
        double r = AttrOr<double>(s.GetRadiusAttr(), 1.0);
        std::vector<ProfilePt> prof;
        AddSphereBand(prof, r, 0, -M_PI / 2, M_PI / 2, rings, false);
        AddRevolved(m, prof, seg);
        FinishCornerNormals(m);
        return true;
    }
    auto revolveTapered = [&](double rb, double rt, double h, bool capBottom, bool capTop) {
        double dz = h, dr = rt - rb, len = std::sqrt(dz * dz + dr * dr);
        double nr = len > 0 ? dz / len : 1, nz = len > 0 ? -dr / len : 0;
        AddRevolved(m, {{rb, -h / 2, nr, nz}, {rt, h / 2, nr, nz}}, seg);
        if (capBottom) AddDisk(m, rb, -h / 2, false, seg);
        if (capTop) AddDisk(m, rt, h / 2, true, seg);
    };
    if (UsdGeomCylinder c{p})
    {
        type = "Cylinder";
        double r = AttrOr<double>(c.GetRadiusAttr(), 1.0), h = AttrOr<double>(c.GetHeightAttr(), 2.0);
        revolveTapered(r, r, h, true, true);
        MapAxis(m, AttrOr<TfToken>(c.GetAxisAttr(), UsdGeomTokens->z));
        FinishCornerNormals(m);
        return true;
    }
    if (UsdGeomCylinder_1 c{p})
    {
        type = "Cylinder_1";
        double rt = AttrOr<double>(c.GetRadiusTopAttr(), 1.0), rb = AttrOr<double>(c.GetRadiusBottomAttr(), 1.0);
        double h = AttrOr<double>(c.GetHeightAttr(), 2.0);
        revolveTapered(rb, rt, h, true, true);
        MapAxis(m, AttrOr<TfToken>(c.GetAxisAttr(), UsdGeomTokens->z));
        FinishCornerNormals(m);
        return true;
    }
    if (UsdGeomCone c{p})
    {
        type = "Cone";
        double r = AttrOr<double>(c.GetRadiusAttr(), 1.0), h = AttrOr<double>(c.GetHeightAttr(), 2.0);
        revolveTapered(r, 0.0, h, true, false);
        MapAxis(m, AttrOr<TfToken>(c.GetAxisAttr(), UsdGeomTokens->z));
        FinishCornerNormals(m);
        return true;
    }
    auto capsule = [&](double rb, double rt, double h) {
        std::vector<ProfilePt> prof;
        int half = std::max(1, rings / 2);
        AddSphereBand(prof, rb, -h / 2, -M_PI / 2, 0, half, false);
        AddSphereBand(prof, rt, h / 2, 0, M_PI / 2, half, false);
        AddRevolved(m, prof, seg);
    };
    if (UsdGeomCapsule c{p})
    {
        type = "Capsule";
        double r = AttrOr<double>(c.GetRadiusAttr(), 0.5), h = AttrOr<double>(c.GetHeightAttr(), 1.0);
        capsule(r, r, h);
        MapAxis(m, AttrOr<TfToken>(c.GetAxisAttr(), UsdGeomTokens->z));
        FinishCornerNormals(m);
        return true;
    }
    if (UsdGeomCapsule_1 c{p})
    {
        type = "Capsule_1";
        double rt = AttrOr<double>(c.GetRadiusTopAttr(), 0.5), rb = AttrOr<double>(c.GetRadiusBottomAttr(), 0.5);
        double h = AttrOr<double>(c.GetHeightAttr(), 1.0);
        capsule(rb, rt, h);
        MapAxis(m, AttrOr<TfToken>(c.GetAxisAttr(), UsdGeomTokens->z));
        FinishCornerNormals(m);
        return true;
    }
    if (UsdGeomPlane pl{p})
    {
        type = "Plane";
        double w = AttrOr<double>(pl.GetWidthAttr(), 2.0) / 2, l = AttrOr<double>(pl.GetLengthAttr(), 2.0) / 2;
        TfToken axis = AttrOr<TfToken>(pl.GetAxisAttr(), UsdGeomTokens->z);
        // UsdGeomPlane: axis X -> width along z, length along y; Y -> width x, length z; Z -> width x, length y.
        GfVec3d U, V, Nn;
        if (axis == UsdGeomTokens->x) { U = GfVec3d(0, 0, 1); V = GfVec3d(0, 1, 0); }
        else if (axis == UsdGeomTokens->y) { U = GfVec3d(1, 0, 0); V = GfVec3d(0, 0, 1); }
        else { U = GfVec3d(1, 0, 0); V = GfVec3d(0, 1, 0); }
        Nn = GfVec3d(axis == UsdGeomTokens->x, axis == UsdGeomTokens->y, axis != UsdGeomTokens->x && axis != UsdGeomTokens->y);
        GfVec3d c[4] = {-U * w - V * l, U * w - V * l, U * w + V * l, -U * w + V * l};
        bool flip = GfDot(GfCross(U, V), Nn) < 0;
        m.counts.push_back(4);
        for (int k = 0; k < 4; ++k)
        {
            int kk = flip ? 3 - k : k;
            m.idx.push_back((int)m.P.size());
            m.P.push_back(c[kk]);
            m.N.push_back(Nn);
        }
        FinishCornerNormals(m);
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------------------------------------------------
// Converter
// ---------------------------------------------------------------------------------------------------------------------
struct Node                          // mirror of the PRC tree, used to compute what the PRC must contain
{
    GfMatrix4d local{1.0};
    int part = -1;
    std::vector<Node*> kids;
    Node* proto = nullptr;
    bool hidden = false;
};

struct PartInfo { std::vector<double> coords; long long triangles = 0; };

struct Built { A3DAsmProductOccurrence* po = nullptr; Node* node = nullptr; };

class Converter
{
public:
    Converter(const Options& o, UsdStageRefPtr s) : opt_(o), stage_(s), xc_(g_time), styles_(rep_) {}

    Report rep_;
    std::deque<Node> nodes_;
    std::vector<PartInfo> parts_;
    Node* root_ = nullptr;

    A3DAsmModelFile* Build(double metersPerUnit, const TfToken& upAxis)
    {
        std::vector<A3DAsmProductOccurrence*> kids;
        Node* rootNode = NewNode();
        for (const UsdPrim& c : stage_->GetPseudoRoot().GetFilteredChildren(UsdPrimDefaultPredicate))
        {
            Built b = ConvertPrim(c, false, false);
            if (b.po) { kids.push_back(b.po); rootNode->kids.push_back(b.node); }
        }
        if (kids.empty()) return nullptr;

        A3DAsmProductOccurrenceData rd = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
        rd.m_uiPOccurrencesSize = (A3DUns32)kids.size();
        rd.m_ppPOccurrences = kids.data();
        rd.m_bUnitFromCAD = true;
        rd.m_dUnit = PrcUnitFromMetersPerUnit(metersPerUnit);
        if (upAxis == UsdGeomTokens->y && !opt_.keepUpAxis)
        {
            // Y-up -> Z-up: (x, y, z) -> (x, -z, y), a +90 degree rotation about X (row-vector form).
            const GfMatrix4d m = YUpToZUp();
            rd.m_pLocation = ToPrcTransform(m);
            rootNode->local = m;
        }
        A3DAsmProductOccurrence* rootPo = nullptr;
        CHECK(A3DAsmProductOccurrenceCreate(&rd, &rootPo));
        ++rep_.occurrences;

        std::string name = stage_->GetDefaultPrim() ? stage_->GetDefaultPrim().GetName().GetString()
                                                    : fs::u8path(opt_.input).stem().u8string();
        char mpu[64];
        snprintf(mpu, sizeof mpu, "%.10g", metersPerUnit);
        std::vector<AttrKV> kv = {{"source", fs::u8path(opt_.input).filename().u8string()},
                                  {"upAxis", upAxis.GetString()},
                                  {"metersPerUnit", mpu},
                                  {"OpenUSD", TfStringPrintf("%d.%d.%d", PXR_MAJOR_VERSION, PXR_MINOR_VERSION, PXR_PATCH_VERSION)}};
        if (stage_->GetDefaultPrim()) kv.push_back({"defaultPrim", stage_->GetDefaultPrim().GetPath().GetString()});
        SetNameAndAttributes(rootPo, name, "USD stage", kv, rep_);
        root_ = rootNode;

        A3DAsmModelFileData md = A3D_MAKE_DATA(A3DAsmModelFileData);
        md.m_bUnitFromCAD = true;
        md.m_dUnit = PrcUnitFromMetersPerUnit(metersPerUnit);
        md.m_uiPOccurrencesSize = 1;
        md.m_ppPOccurrences = &rootPo;
        A3DAsmModelFile* mf = nullptr;
        CHECK(A3DAsmModelFileCreate(&md, &mf));
        return mf;
    }

private:
    const Options& opt_;
    UsdStageRefPtr stage_;
    UsdGeomXformCache xc_;
    StyleTable styles_;
    std::map<SdfPath, Built> nativeProtos_;      // UsdPrim::GetPrototype() -> shared PRC prototype
    std::map<SdfPath, Built> piProtos_;          // PointInstancer prototype prim -> shared PRC prototype
    std::map<SdfPath, std::pair<Look, bool>> materialLooks_;   // material -> (look, diffuse comes from displayColor)

    Node* NewNode() { nodes_.emplace_back(); return &nodes_.back(); }

    // ----- materials ---------------------------------------------------------------------------------------------
    bool ReadInput(const UsdShadeShader& sh, const char* name, VtValue& v, bool* fromDisplayColor = nullptr)
    {
        UsdShadeInput in = sh.GetInput(TfToken(name));
        if (!in) return false;
        UsdShadeAttributeVector attrs = in.GetValueProducingAttributes();
        if (attrs.empty()) return false;
        const UsdAttribute& a = attrs[0];
        if (UsdShadeUtils::GetType(a.GetName()) == UsdShadeAttributeType::Input) return a.Get(&v, g_time);
        // Connected to the output of another node: a primvar reader or a texture.
        UsdShadeShader src(a.GetPrim());
        TfToken id;
        src.GetShaderId(&id);
        if (TfStringStartsWith(id.GetString(), "UsdPrimvarReader"))
        {
            VtValue var;
            UsdShadeInput vi = src.GetInput(TfToken("varname"));
            if (vi) for (const UsdAttribute& va : vi.GetValueProducingAttributes()) { va.Get(&var, g_time); break; }
            std::string vn = var.IsHolding<TfToken>() ? var.UncheckedGet<TfToken>().GetString()
                           : var.IsHolding<std::string>() ? var.UncheckedGet<std::string>() : "";
            if (vn == "displayColor" && fromDisplayColor) *fromDisplayColor = true;
            UsdShadeInput fb = src.GetInput(TfToken("fallback"));
            return fb && fb.Get(&v, g_time);
        }
        ++rep_.texturedInputs;   // a texture: its colour is not converted; use its fallback when authored
        UsdShadeInput fb = src.GetInput(TfToken("fallback"));
        return fb && fb.Get(&v, g_time);
    }

    // Returns false when the material has no surface shader this converter understands.
    bool MaterialLook(const UsdShadeMaterial& mat, Look& look, bool& diffuseFromDisplayColor)
    {
        auto it = materialLooks_.find(mat.GetPath());
        if (it != materialLooks_.end()) { look = it->second.first; diffuseFromDisplayColor = it->second.second; return look.kind != Look::None; }
        Look l;
        bool fromDc = false;
        UsdShadeShader sh;
        for (const TfToken& ctx : {UsdShadeTokens->universalRenderContext, TfToken("mtlx")})
            if ((sh = mat.ComputeSurfaceSource(TfTokenVector{ctx}))) break;
        TfToken id;
        if (sh) sh.GetShaderId(&id);
        VtValue v;
        GfVec3d c;
        double d;
        if (id == TfToken("UsdPreviewSurface"))
        {
            l.kind = Look::Material;
            if (ReadInput(sh, "diffuseColor", v, &fromDc) && ToVec3(v, c)) l.diffuse = c;
            if (ReadInput(sh, "emissiveColor", v) && ToVec3(v, c)) l.emissive = c;
            int useSpec = 0;
            if (ReadInput(sh, "useSpecularWorkflow", v) && v.IsHolding<int>()) useSpec = v.UncheckedGet<int>();
            double metallic = 0, rough = 0.5;
            if (ReadInput(sh, "metallic", v) && ToScalar(v, d)) metallic = d;
            if (ReadInput(sh, "roughness", v) && ToScalar(v, d)) rough = d;
            if (useSpec) { l.specular = GfVec3d(0); if (ReadInput(sh, "specularColor", v) && ToVec3(v, c)) l.specular = c; }
            else l.specular = GfVec3d(0.04) * (1 - metallic) + l.diffuse * metallic;
            l.shininess = ShininessFromRoughness(rough);
            if (ReadInput(sh, "opacity", v) && ToScalar(v, d)) l.opacity = d;
        }
        else if (TfStringStartsWith(id.GetString(), "ND_standard_surface") || TfStringStartsWith(id.GetString(), "ND_open_pbr_surface"))
        {
            const bool openPbr = TfStringStartsWith(id.GetString(), "ND_open_pbr_surface");
            l.kind = Look::Material;
            double base = openPbr ? 1.0 : 1.0;
            GfVec3d bc(0.8);
            if (ReadInput(sh, openPbr ? "base_weight" : "base", v) && ToScalar(v, d)) base = d;
            if (ReadInput(sh, "base_color", v, &fromDc) && ToVec3(v, c)) bc = c;
            l.diffuse = bc * base;
            double metallic = 0, rough = 0.2;
            if (ReadInput(sh, openPbr ? "base_metalness" : "metalness", v) && ToScalar(v, d)) metallic = d;
            if (ReadInput(sh, "specular_roughness", v) && ToScalar(v, d)) rough = d;
            l.specular = GfVec3d(0.04) * (1 - metallic) + l.diffuse * metallic;
            l.shininess = ShininessFromRoughness(rough);
            double ew = 0;
            GfVec3d ec(1.0);
            if (ReadInput(sh, openPbr ? "emission_luminance" : "emission", v) && ToScalar(v, d)) ew = openPbr ? std::min(d / 1000.0, 1.0) : d;
            if (ReadInput(sh, "emission_color", v) && ToVec3(v, c)) ec = c;
            l.emissive = ec * ew;
            if (ReadInput(sh, openPbr ? "geometry_opacity" : "opacity", v) && ToScalar(v, d)) l.opacity = d;
        }
        materialLooks_[mat.GetPath()] = {l, fromDc};
        look = l;
        diffuseFromDisplayColor = fromDc;
        return l.kind != Look::None;
    }

    // ----- gprim appearance --------------------------------------------------------------------------------------
    struct ColorSource
    {
        TfToken interp, ointerp;
        VtVec3fArray c;
        VtFloatArray o;
        bool Has() const { return !c.empty(); }
    };

    ColorSource DisplayColor(const UsdPrim& p)
    {
        ColorSource cs;
        UsdGeomPrimvarsAPI api(p);
        UsdGeomPrimvar pc = api.FindPrimvarWithInheritance(TfToken("displayColor"));
        if (pc && pc.HasAuthoredValue() && pc.ComputeFlattened(&cs.c, g_time)) cs.interp = pc.GetInterpolation();
        UsdGeomPrimvar po = api.FindPrimvarWithInheritance(TfToken("displayOpacity"));
        if (po && po.HasAuthoredValue() && po.ComputeFlattened(&cs.o, g_time)) cs.ointerp = po.GetInterpolation();
        return cs;
    }

    // Builds the part (RI) for a gprim. Returns nullptr when there is nothing drawable.
    A3DAsmPartDefinition* BuildPart(const UsdPrim& p, Node* node, bool shown)
    {
        PolyMesh pm;
        std::string ptype;
        bool isMesh = false;
        if (UsdGeomMesh mesh{p})
        {
            isMesh = true;
            ptype = "Mesh";
            VtVec3fArray pts;
            VtIntArray fvc, fvi, holes;
            mesh.GetPointsAttr().Get(&pts, g_time);
            mesh.GetFaceVertexCountsAttr().Get(&fvc, g_time);
            mesh.GetFaceVertexIndicesAttr().Get(&fvi, g_time);
            mesh.GetHoleIndicesAttr().Get(&holes, g_time);
            pm.P.assign(pts.begin(), pts.end());
            pm.counts.assign(fvc.begin(), fvc.end());
            pm.idx.assign(fvi.begin(), fvi.end());
            pm.holes.insert(holes.begin(), holes.end());
            pm.leftHanded = AttrOr<TfToken>(mesh.GetOrientationAttr(), UsdGeomTokens->rightHanded) == UsdGeomTokens->leftHanded;
            pm.smooth = AttrOr<TfToken>(mesh.GetSubdivisionSchemeAttr(), UsdGeomTokens->catmullClark) != UsdGeomTokens->none;
            UsdGeomPrimvar np = UsdGeomPrimvarsAPI(p).GetPrimvar(UsdGeomTokens->normals);
            VtVec3fArray n;
            if (np && np.HasAuthoredValue() && np.ComputeFlattened(&n, g_time)) pm.normalInterp = np.GetInterpolation();
            else if (mesh.GetNormalsAttr().HasAuthoredValue() && mesh.GetNormalsAttr().Get(&n, g_time)) pm.normalInterp = mesh.GetNormalsInterpolation();
            pm.N.assign(n.begin(), n.end());
            ++rep_.meshes;
        }
        else if (TessellatePrimitive(p, opt_.segments, pm, ptype))
        {
            pm.leftHanded = AttrOr<TfToken>(UsdGeomGprim(p).GetOrientationAttr(), UsdGeomTokens->rightHanded) == UsdGeomTokens->leftHanded;
            ++rep_.primitives;
        }
        else return nullptr;

        // ---- appearance: bound material (mesh, then GeomSubsets), else displayColor/displayOpacity
        const int nFaces = (int)pm.counts.size();
        Look meshLook;
        bool meshDiffuseFromDc = false, meshHasMaterial = false;
        if (UsdShadeMaterial mat = UsdShadeMaterialBindingAPI(p).ComputeBoundMaterial())
            meshHasMaterial = MaterialLook(mat, meshLook, meshDiffuseFromDc);

        std::vector<A3DUns32> faceStyle(nFaces, kNoStyle);
        std::vector<char> faceUsesDc(nFaces, !meshHasMaterial || meshDiffuseFromDc);
        std::vector<Look> faceLook(nFaces, meshLook);
        bool perFace = false;
        if (isMesh)
        {
            for (const UsdGeomSubset& sub : UsdShadeMaterialBindingAPI(p).GetMaterialBindSubsets())
            {
                UsdShadeMaterial sm = UsdShadeMaterialBindingAPI(sub.GetPrim()).ComputeBoundMaterial();
                Look sl;
                bool sdc = false;
                if (!sm || !MaterialLook(sm, sl, sdc)) continue;
                VtIntArray ind;
                sub.GetIndicesAttr().Get(&ind, g_time);
                for (int f : ind)
                    if (f >= 0 && f < nFaces) { faceLook[f] = sl; faceUsesDc[f] = sdc; perFace = true; }
            }
        }

        ColorSource dc = DisplayColor(p);
        const bool dcUniform = dc.Has() && dc.interp == UsdGeomTokens->uniform && (int)dc.c.size() == nFaces;
        const bool dcVertex = dc.Has() && (dc.interp == UsdGeomTokens->vertex || dc.interp == UsdGeomTokens->varying) && dc.c.size() == pm.P.size();
        size_t nCorners = pm.idx.size();
        const bool dcCorner = dc.Has() && dc.interp == UsdGeomTokens->faceVarying && dc.c.size() == nCorners;
        auto opacityAt = [&](int f, int corner, int point) -> float {
            if (dc.o.empty()) return 1.0f;
            if (dc.ointerp == UsdGeomTokens->uniform && (int)dc.o.size() == nFaces) return dc.o[f];
            if ((dc.ointerp == UsdGeomTokens->vertex || dc.ointerp == UsdGeomTokens->varying) && dc.o.size() == pm.P.size()) return dc.o[point];
            if (dc.ointerp == UsdGeomTokens->faceVarying && dc.o.size() == nCorners) return dc.o[corner];
            return dc.o[0];
        };
        const bool constantOpacity = dc.o.empty() || dc.ointerp == UsdGeomTokens->constant || dc.o.size() == 1;

        for (int f = 0; f < nFaces; ++f)
        {
            Look l = faceLook[f];
            if (faceUsesDc[f] && dc.Has())
            {
                if (dcUniform) { perFace = true; }
                const GfVec3f c = dcUniform ? dc.c[f] : dc.c[0];
                if (!dcVertex && !dcCorner)
                {
                    if (l.kind == Look::None) l.kind = Look::Color;
                    l.diffuse = GfVec3d(c);
                    if (constantOpacity) l.opacity = dc.o.empty() ? 1.0 : dc.o[0];
                    else if (dc.ointerp == UsdGeomTokens->uniform && (int)dc.o.size() == nFaces) { l.opacity = dc.o[f]; perFace = true; }
                }
            }
            faceStyle[f] = styles_.Get(l);
        }

        A3DUns32 riStyle = kNoStyle;
        FaceAppearance app;
        if (perFace) app.faceStyle = [&](int f) { return faceStyle[f]; };
        else if (nFaces > 0) riStyle = faceStyle[0];
        if (dcVertex || dcCorner)
        {
            app.cornerColor = [&](int f, int corner, int point, A3DUns8 rgba[4]) -> bool {
                if (!faceUsesDc[f]) return false;
                const GfVec3f c = dcVertex ? dc.c[point] : dc.c[corner];
                float o = opacityAt(f, corner, point);
                rgba[0] = (A3DUns8)std::lround(std::clamp(c[0], 0.f, 1.f) * 255);
                rgba[1] = (A3DUns8)std::lround(std::clamp(c[1], 0.f, 1.f) * 255);
                rgba[2] = (A3DUns8)std::lround(std::clamp(c[2], 0.f, 1.f) * 255);
                rgba[3] = (A3DUns8)std::lround(std::clamp(o, 0.f, 1.f) * 255);
                return true;
            };
        }

        TriMesh tm;
        if (!Triangulate(pm, app, tm, rep_)) return nullptr;
        A3DRiPolyBrepModel* ri = CreatePolyBrep(tm, riStyle, true);

        PartInfo pi;
        pi.coords = tm.coords;
        pi.triangles = tm.Triangles();
        rep_.uniqueTriangles += pi.triangles;
        node->part = (int)parts_.size();
        parts_.push_back(std::move(pi));

        A3DAsmPartDefinitionData pd = A3D_MAKE_DATA(A3DAsmPartDefinitionData);
        pd.m_uiRepItemsSize = 1;
        A3DRiRepresentationItem* items[1] = {ri};
        pd.m_ppRepItems = items;
        A3DAsmPartDefinition* part = nullptr;
        CHECK(A3DAsmPartDefinitionCreate(&pd, &part));
        (void)shown;
        return part;
    }

    // ----- attributes --------------------------------------------------------------------------------------------
    std::vector<AttrKV> PrimAttributes(const UsdPrim& p)
    {
        std::vector<AttrKV> kv;
        kv.push_back({"path", p.GetPath().GetString()});
        if (!p.GetTypeName().IsEmpty()) kv.push_back({"type", p.GetTypeName().GetString()});
        TfToken kind;
        if (UsdModelAPI(p).GetKind(&kind) && !kind.IsEmpty()) kv.push_back({"kind", kind.GetString()});
        if (UsdGeomImageable img{p})
        {
            TfToken purpose = AttrOr<TfToken>(img.GetPurposeAttr(), UsdGeomTokens->default_);
            if (purpose != UsdGeomTokens->default_) kv.push_back({"purpose", purpose.GetString()});
        }
        for (const std::string& vs : p.GetVariantSets().GetNames())
            kv.push_back({"variant:" + vs, p.GetVariantSets().GetVariantSelection(vs)});
        for (const UsdAttribute& a : p.GetAuthoredAttributes())
        {
            const std::string n = a.GetName().GetString();
            if (!a.IsCustom() && !TfStringStartsWith(n, "userProperties:")) continue;
            VtValue v;
            if (!a.Get(&v, g_time)) continue;
            kv.push_back({n, ValueToString(v)});
        }
        return kv;
    }

    // ----- prims -------------------------------------------------------------------------------------------------
    bool SkippedByPurpose(const UsdPrim& p)
    {
        UsdGeomImageable img(p);
        if (!img) return false;
        TfToken purpose = AttrOr<TfToken>(img.GetPurposeAttr(), UsdGeomTokens->default_);
        if (purpose == UsdGeomTokens->guide && !opt_.includeGuide) { ++rep_.skipped["purpose guide"]; return true; }
        if (purpose == UsdGeomTokens->proxy && !opt_.includeProxy) { ++rep_.skipped["purpose proxy"]; return true; }
        return false;
    }

    GfMatrix4d LocalMatrix(const UsdPrim& p, bool inPrototype)
    {
        if (!UsdGeomXformable(p)) return GfMatrix4d(1.0);
        bool resets = false;
        GfMatrix4d m = xc_.GetLocalTransformation(p, &resets);
        if (resets && !inPrototype && p.GetParent())
            m = xc_.GetLocalToWorldTransform(p) * xc_.GetLocalToWorldTransform(p.GetParent()).GetInverse();
        return m;
    }

    Built NativePrototype(const UsdPrim& proto)
    {
        auto it = nativeProtos_.find(proto.GetPath());
        if (it != nativeProtos_.end()) return it->second;
        nativeProtos_[proto.GetPath()] = Built{};   // guards against cycles
        Built b;
        std::vector<A3DAsmProductOccurrence*> kids;
        Node* n = NewNode();
        for (const UsdPrim& c : proto.GetFilteredChildren(UsdPrimDefaultPredicate))
        {
            Built cb = ConvertPrim(c, true, false);
            if (cb.po) { kids.push_back(cb.po); n->kids.push_back(cb.node); }
        }
        if (!kids.empty())
        {
            A3DAsmProductOccurrenceData d = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
            d.m_uiPOccurrencesSize = (A3DUns32)kids.size();
            d.m_ppPOccurrences = kids.data();
            CHECK(A3DAsmProductOccurrenceCreate(&d, &b.po));
            SetNameAndAttributes(b.po, "Prototype " + proto.GetPath().GetString(), "USD", {}, rep_);
            b.node = n;
            ++rep_.prototypes;
        }
        nativeProtos_[proto.GetPath()] = b;
        return b;
    }

    Built PointInstancerPrototype(const UsdPrim& target)
    {
        auto it = piProtos_.find(target.GetPath());
        if (it != piProtos_.end()) return it->second;
        piProtos_[target.GetPath()] = Built{};
        // The prototype root's own transform is part of the per-instance matrices (IncludeProtoXform).
        Built b = ConvertPrim(target, true, true);
        if (b.po) ++rep_.prototypes;
        piProtos_[target.GetPath()] = b;
        return b;
    }

    void ConvertPointInstancer(const UsdGeomPointInstancer& pi, std::vector<A3DAsmProductOccurrence*>& kids, Node* node)
    {
        SdfPathVector targets;
        pi.GetPrototypesRel().GetForwardedTargets(&targets);
        VtIntArray protoIndices;
        pi.GetProtoIndicesAttr().Get(&protoIndices, g_time);
        VtArray<GfMatrix4d> xforms;
        if (!pi.ComputeInstanceTransformsAtTime(&xforms, g_time, g_time, UsdGeomPointInstancer::IncludeProtoXform,
                                                UsdGeomPointInstancer::IgnoreMask))
        {
            printf("WARNING: %s: cannot compute the instance transforms; instances skipped\n", pi.GetPath().GetText());
            return;
        }
        // Inactive instances are dropped; invisible ones become hidden occurrences.
        VtInt64Array ids, invisible;
        pi.GetIdsAttr().Get(&ids, g_time);
        pi.GetInvisibleIdsAttr().Get(&invisible, g_time);
        const std::set<int64_t> invisibleSet(invisible.begin(), invisible.end());
        const std::vector<bool> mask = pi.ComputeMaskAtTime(g_time);   // false: inactive or invisible

        std::vector<Built> protos;
        for (const SdfPath& t : targets)
        {
            UsdPrim tp = stage_->GetPrimAtPath(t);
            protos.push_back(tp ? PointInstancerPrototype(tp) : Built{});
        }
        for (size_t i = 0; i < protoIndices.size() && i < xforms.size(); ++i)
        {
            int pidx = protoIndices[i];
            if (pidx < 0 || pidx >= (int)protos.size() || !protos[pidx].po) continue;
            int64_t id = i < ids.size() ? ids[i] : (int64_t)i;
            const bool hidden = invisibleSet.count(id) > 0;
            if (!mask.empty() && i < mask.size() && !mask[i] && !hidden) { ++rep_.skipped["point instance inactive"]; continue; }
            if (hidden && opt_.skipHidden) { ++rep_.skipped["point instance invisible"]; continue; }

            A3DAsmProductOccurrenceData d = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
            d.m_pPrototype = protos[pidx].po;
            if (!IsIdentity(xforms[i])) d.m_pLocation = ToPrcTransform(xforms[i]);
            A3DAsmProductOccurrence* po = nullptr;
            CHECK(A3DAsmProductOccurrenceCreate(&d, &po));
            SetNameAndAttributes(po, TfStringPrintf("%s[%zu]", pi.GetPrim().GetName().GetText(), i), "USD",
                                 {{"pointInstancer", pi.GetPath().GetString()}, {"protoIndex", std::to_string(pidx)},
                                  {"id", std::to_string(id)}}, rep_);
            if (hidden) { SetGraphics(po, kNoStyle, false); ++rep_.hidden; }
            kids.push_back(po);
            Node* n = NewNode();
            n->local = xforms[i];
            n->proto = protos[pidx].node;
            n->hidden = hidden;
            node->kids.push_back(n);
            ++rep_.pointInstances;
            ++rep_.occurrences;
        }
    }

    Built ConvertPrim(const UsdPrim& p, bool inPrototype, bool skipOwnXform)
    {
        Built out;
        const std::string type = p.GetTypeName().IsEmpty() ? "(typeless)" : p.GetTypeName().GetString();
        ++rep_.primTypes[type];
        if (SkippedByPurpose(p)) return out;

        bool hidden = false;
        if (UsdGeomImageable img{p})
            hidden = AttrOr<TfToken>(img.GetVisibilityAttr(), UsdGeomTokens->inherited) == UsdGeomTokens->invisible;
        if (hidden && opt_.skipHidden) { ++rep_.skipped["invisible"]; return out; }

        const bool imageable = (bool)UsdGeomImageable(p);
        if (!imageable && !p.GetTypeName().IsEmpty())
        {
            // Materials, shaders, lights' non-geometry, render settings ...: not geometry. GeomSubsets are read
            // with their mesh.
            if (!UsdGeomSubset(p) && !UsdShadeMaterial(p) && !UsdShadeShader(p) && !p.IsA<UsdShadeNodeGraph>())
                ++rep_.skipped["non-geometry " + type];
            return out;
        }

        Node* node = NewNode();
        node->hidden = hidden;
        if (!skipOwnXform) node->local = LocalMatrix(p, inPrototype);

        A3DAsmProductOccurrenceData d = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
        std::vector<A3DAsmProductOccurrence*> kids;

        if (p.IsInstance())
        {
            Built proto = NativePrototype(p.GetPrototype());
            if (!proto.po) { ++rep_.emptyDropped; return out; }
            d.m_pPrototype = proto.po;
            node->proto = proto.node;
            ++rep_.instances;
        }
        else if (UsdGeomPointInstancer pi{p})
        {
            ConvertPointInstancer(pi, kids, node);
            // Children of a point instancer that are not prototypes are still drawn.
            SdfPathVector targets;
            pi.GetPrototypesRel().GetForwardedTargets(&targets);
            for (const UsdPrim& c : p.GetFilteredChildren(UsdPrimDefaultPredicate))
            {
                bool holdsPrototype = false;
                for (const SdfPath& t : targets) if (t.HasPrefix(c.GetPath())) holdsPrototype = true;
                if (holdsPrototype) continue;
                Built cb = ConvertPrim(c, inPrototype, false);
                if (cb.po) { kids.push_back(cb.po); node->kids.push_back(cb.node); }
            }
        }
        else
        {
            if (UsdGeomGprim(p))
            {
                const bool supported = p.IsA<UsdGeomMesh>() || p.IsA<UsdGeomCube>() || p.IsA<UsdGeomSphere>() ||
                                       p.IsA<UsdGeomCylinder>() || p.IsA<UsdGeomCylinder_1>() || p.IsA<UsdGeomCone>() ||
                                       p.IsA<UsdGeomCapsule>() || p.IsA<UsdGeomCapsule_1>() || p.IsA<UsdGeomPlane>();
                if (supported) d.m_pPart = BuildPart(p, node, !hidden);
                else ++rep_.skipped["unsupported " + type];
            }
            for (const UsdPrim& c : p.GetFilteredChildren(UsdPrimDefaultPredicate))
            {
                Built cb = ConvertPrim(c, inPrototype, false);
                if (cb.po) { kids.push_back(cb.po); node->kids.push_back(cb.node); }
            }
        }

        if (!d.m_pPart && !d.m_pPrototype && kids.empty()) { ++rep_.emptyDropped; return out; }

        d.m_uiPOccurrencesSize = (A3DUns32)kids.size();
        d.m_ppPOccurrences = kids.empty() ? nullptr : kids.data();
        if (!IsIdentity(node->local)) d.m_pLocation = ToPrcTransform(node->local);
        CHECK(A3DAsmProductOccurrenceCreate(&d, &out.po));
        // USD_Writer keeps the original CAD name in displayName (prim names must be identifiers).
        const std::string shown = p.HasAuthoredDisplayName() ? p.GetDisplayName() : p.GetName().GetString();
        SetNameAndAttributes(out.po, shown, "USD", PrimAttributes(p), rep_);
        if (hidden) { SetGraphics(out.po, kNoStyle, false); ++rep_.hidden; }
        ++rep_.occurrences;
        out.node = node;
        return out;
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Expected content (from the mirror tree) and what the written PRC actually contains
// ---------------------------------------------------------------------------------------------------------------------
struct Content
{
    long occurrences = 0;
    long long triangles = 0;
    GfRange3d box;
    long long pointsTransformed = 0;
    bool boxComplete = true;
};

static void Expect(const Converter& cv, const Node* n, const GfMatrix4d& parent, bool hidden, Content& c, bool countSelf)
{
    const GfMatrix4d w = n->local * parent;
    hidden = hidden || n->hidden;
    if (countSelf) ++c.occurrences;
    // Same rule as PRC: an occurrence without its own part / children uses its prototype's (recursively).
    int part = n->part;
    const std::vector<Node*>* kids = &n->kids;
    for (const Node* p = n->proto; p && (part < 0 || kids->empty()); p = p->proto)
    {
        if (part < 0) part = p->part;
        if (kids->empty()) kids = &p->kids;
    }
    if (part >= 0)
    {
        const PartInfo& pi = cv.parts_[part];
        c.triangles += pi.triangles;
        if (!hidden)
        {
            if (c.pointsTransformed > 200000000LL) c.boxComplete = false;
            else
                for (size_t i = 0; i + 2 < pi.coords.size(); i += 3)
                    c.box.UnionWith(w.Transform(GfVec3d(pi.coords[i], pi.coords[i + 1], pi.coords[i + 2])));
            c.pointsTransformed += (long long)pi.coords.size() / 3;
        }
    }
    for (const Node* k : *kids) Expect(cv, k, w, hidden, c, true);
}






// ---------------------------------------------------------------------------------------------------------------------
// Main flow
// ---------------------------------------------------------------------------------------------------------------------

static bool ParseVariant(const std::string& s, VariantOverride& v)
{
    // <primPath>{<set>=<selection>}
    size_t b = s.find('{'), e = s.rfind('}'), eq = s.find('=', b == std::string::npos ? 0 : b);
    if (b == std::string::npos || e == std::string::npos || eq == std::string::npos || !(b < eq && eq < e) || e + 1 != s.size()) return false;
    v.primPath = s.substr(0, b);
    v.set = s.substr(b + 1, eq - b - 1);
    v.selection = s.substr(eq + 1, e - eq - 1);
    return !v.primPath.empty() && !v.set.empty();
}

static int Run(const Options& opt)
{
    std::error_code ec;
    const fs::path inPath = fs::u8path(opt.input), outPath = fs::u8path(opt.output);
    const std::string inExt = Lower(inPath.extension().u8string()), outExt = Lower(outPath.extension().u8string());
    if (outExt != ".prc") { printf("ERROR: unsupported output format '%s' (only .prc)\n", outExt.c_str()); return kUnsupported; }
    if (!fs::exists(inPath, ec)) { printf("ERROR: input file not found: %s\n", opt.input.c_str()); return kInput; }
    if (inExt != ".usd" && inExt != ".usda" && inExt != ".usdc" && inExt != ".usdz")
    { printf("ERROR: unsupported input format '%s' (expected .usd, .usda, .usdc or .usdz)\n", inExt.c_str()); return kUnsupported; }
    if (fs::exists(outPath, ec) && !opt.force) { printf("ERROR: output exists: %s (use --force to overwrite)\n", opt.output.c_str()); return kUsage; }

    // ---- open the stage
    UsdStageRefPtr stage;
    {
        TfErrorMark mark;
        stage = UsdStage::Open(opt.input, UsdStage::LoadAll);
        if (!stage)
        {
            std::string why;
            for (auto it = mark.GetBegin(); it != mark.GetEnd(); ++it) why += "\n  " + it->GetCommentary();
            mark.Clear();
            printf("ERROR: cannot open USD stage %s%s\n", opt.input.c_str(), why.c_str());
            return kInput;
        }
        if (!mark.IsClean())
        {
            for (auto it = mark.GetBegin(); it != mark.GetEnd(); ++it) printf("WARNING: USD: %s\n", it->GetCommentary().c_str());
            mark.Clear();
        }
    }

    // ---- variant overrides (authored in the session layer, the source files are not changed)
    for (const VariantOverride& v : opt.variants)
    {
        UsdPrim p = SdfPath::IsValidPathString(v.primPath) ? stage->GetPrimAtPath(SdfPath(v.primPath)) : UsdPrim();
        if (!p) { printf("ERROR: --variant: no prim %s\n", v.primPath.c_str()); return kInput; }
        if (!p.GetVariantSets().HasVariantSet(v.set)) { printf("ERROR: --variant: %s has no variant set '%s'\n", v.primPath.c_str(), v.set.c_str()); return kInput; }
        UsdVariantSet vs = p.GetVariantSet(v.set);
        std::vector<std::string> names = vs.GetVariantNames();
        if (std::find(names.begin(), names.end(), v.selection) == names.end())
        { printf("ERROR: --variant: '%s' is not a variant of %s{%s} (%s)\n", v.selection.c_str(), v.primPath.c_str(), v.set.c_str(), TfStringJoin(names, ", ").c_str()); return kInput; }
        UsdEditContext ctx(stage, stage->GetSessionLayer());
        vs.SetVariantSelection(v.selection);
    }

    if (opt.timeGiven) g_time = UsdTimeCode(opt.time);
    const double mpu = UsdGeomGetStageMetersPerUnit(stage);
    const TfToken up = UsdGeomGetStageUpAxis(stage);
    printf("USD stage: %s (upAxis %s, metersPerUnit %g, time %s)\n", inPath.filename().u8string().c_str(), up.GetText(), mpu,
           opt.timeGiven ? TfStringPrintf("%g", opt.time).c_str() : "earliest");
    for (const UsdPrim& p : stage->Traverse())
        for (const std::string& set : p.GetVariantSets().GetNames())
        {
            UsdVariantSet vs = p.GetVariantSet(set);
            LOG("Variant %s{%s=%s} (options: %s)\n", p.GetPath().GetText(), set.c_str(), vs.GetVariantSelection().c_str(),
                TfStringJoin(vs.GetVariantNames(), ", ").c_str());
        }

    // ---- build the PRC model file
    Converter cv(opt, stage);
    A3DAsmModelFile* mf = nullptr;
    try { mf = cv.Build(mpu, up); }
    catch (A3DStatus) { return kExport; }
    if (!mf) { printf("ERROR: the stage contains no convertible geometry\n"); return kInput; }

    const Report& r = cv.rep_;
    LOG("Prim types:");
    for (const auto& kv : r.primTypes) LOG(" %s=%ld", kv.first.c_str(), kv.second);
    LOG("\n");
    printf("Converted: occurrences %ld, meshes %ld, primitives %ld, native instances %ld, point instances %ld, prototypes %ld, hidden %ld\n",
           r.occurrences, r.meshes, r.primitives, r.instances, r.pointInstances, r.prototypes, r.hidden);
    printf("Appearance: styles %ld, materials %ld, textured inputs approximated %ld; attributes %ld\n", r.styles, r.materials, r.texturedInputs, r.attributes);
    if (!r.skipped.empty())
    {
        printf("Skipped:");
        for (const auto& kv : r.skipped) printf(" %s=%ld", kv.first.c_str(), kv.second);
        printf("\n");
    }
    if (r.badFaces) printf("WARNING: %ld degenerate or invalid faces ignored\n", r.badFaces);

    Content exp;
    Expect(cv, cv.root_, GfMatrix4d(1.0), false, exp, true);
    printf("Triangles: unique %lld, drawn %lld\n", r.uniqueTriangles, exp.triangles);

    // ---- export to a temporary file, then rename
    fs::path tmp = outPath;
    tmp += ".partial.prc";
    A3DRWParamsExportPrcData ep = A3D_MAKE_DATA(A3DRWParamsExportPrcData);
    ep.m_bCompressBrep = false;
    ep.m_bCompressTessellation = false;
    ep.m_bRemoveBRep = false;
    ep.m_bRemoveAttributes = false;
    A3DStatus st = A3DAsmModelFileExportToPrcFile(mf, &ep, tmp.u8string().c_str(), nullptr);
    A3DAsmModelFileDelete(mf);
    if (st != A3D_SUCCESS) { PrintStatus("A3DAsmModelFileExportToPrcFile", st); fs::remove(tmp, ec); return kExport; }
    if (fs::exists(outPath, ec)) fs::remove(outPath, ec);
    fs::rename(tmp, outPath, ec);
    if (ec) { printf("ERROR: cannot move %s to %s: %s\n", tmp.u8string().c_str(), opt.output.c_str(), ec.message().c_str()); return kExport; }
    printf("Wrote %s\n", opt.output.c_str());

    // ---- re-import what was written and compare with what was converted
    A3DRWParamsLoadData lp = A3D_MAKE_DATA(A3DRWParamsLoadData);
    lp.m_sGeneral.m_eReadGeomTessMode = kA3DReadGeomAndTess;
    lp.m_sGeneral.m_bReadSolids = true;
    lp.m_sGeneral.m_bReadSurfaces = true;
    lp.m_sGeneral.m_bReadHiddenObjects = true;
    lp.m_sGeneral.m_bReadAttributes = true;
    A3DAsmModelFile* back = nullptr;
    st = A3DAsmModelFileLoadFromFile(opt.output.c_str(), &lp, &back);
    if (st != A3D_SUCCESS || !back) { PrintStatus("re-import of the written PRC", st); return kVerify; }

    PrcContent act;
    A3DAsmModelFileData md = A3D_MAKE_DATA(A3DAsmModelFileData);
    A3DAsmModelFileGet(back, &md);
    const double prcUnit = md.m_dUnit;
    if (opt.dumpTree) printf("PRC tree:\n");
    for (A3DUns32 i = 0; i < md.m_uiPOccurrencesSize; ++i) InspectPrc(md.m_ppPOccurrences[i], 1, opt.dumpTree, act);
    A3DAsmModelFileGet(nullptr, &md);
    const Box bb = PrcBoundingBox(back);
    A3DAsmModelFileDelete(back);
    printf("Re-imported PRC: unit %g mm, occurrences %ld, unique meshes %zu, triangles %lld, bbox %s\n",
           prcUnit, act.occurrences, act.uniqueTess.size(), act.triangles, BoxText(bb).c_str());

    int rc = kOk;
    if (act.occurrences != exp.occurrences || act.triangles != exp.triangles)
    {
        printf("VERIFY FAILED: PRC has %ld occurrences / %lld triangles, expected %ld / %lld\n", act.occurrences, act.triangles,
               exp.occurrences, exp.triangles);
        rc = kVerify;
    }
    if (exp.boxComplete && !exp.box.IsEmpty())
    {
        const GfVec3d emin = exp.box.GetMin(), emax = exp.box.GetMax();
        const double tol = 1e-5 * std::max(1.0, (emax - emin).GetLength());
        Box eb;
        eb.min = emin;
        eb.max = emax;
        const double dev = BoxDeviation(bb, eb);
        if (!bb.ok || dev > tol)
        {
            printf("VERIFY FAILED: PRC bounding box differs from the USD world-space geometry by %.3g (expected (%.6g %.6g %.6g)-(%.6g %.6g %.6g))\n",
                   dev, emin[0], emin[1], emin[2], emax[0], emax[1], emax[2]);
            rc = kVerify;
        }
    }
    else if (!exp.boxComplete) printf("Note: bounding-box check skipped (more than 2e8 instanced points)\n");
    if (rc == kOk) printf("Check: OK (occurrences, triangles and world bounding box match the USD stage)\n");
    return rc;
}

static int Main(const std::vector<std::string>& args)
{
    Options opt;
    std::vector<std::string> pos;
    for (size_t i = 1; i < args.size(); ++i)
    {
        const std::string& a = args[i];
        auto next = [&](std::string& v) { if (i + 1 >= args.size()) return false; v = args[++i]; return true; };
        std::string v;
        if (a == "--force") opt.force = true;
        else if (a == "--quiet") opt.quiet = true;
        else if (a == "--verbose") opt.verbose = true;
        else if (a == "--dump-tree") opt.dumpTree = true;
        else if (a == "--skip-hidden") opt.skipHidden = true;
        else if (a == "--include-proxy") opt.includeProxy = true;
        else if (a == "--include-guide") opt.includeGuide = true;
        else if (a == "--keep-up-axis") opt.keepUpAxis = true;
        else if (a == "--variant")
        {
            VariantOverride vo;
            if (!next(v) || !ParseVariant(v, vo)) { printf("ERROR: --variant expects <primPath>{<set>=<selection>}\n"); return kUsage; }
            opt.variants.push_back(vo);
        }
        else if (a == "--time")
        {
            char* end = nullptr;
            if (!next(v) || (opt.time = std::strtod(v.c_str(), &end), !end || *end)) { printf("ERROR: --time expects a number\n"); return kUsage; }
            opt.timeGiven = true;
        }
        else if (a == "--segments")
        {
            if (!next(v)) { PrintUsage(); return kUsage; }
            opt.segments = std::atoi(v.c_str());
            if (opt.segments < 4 || opt.segments % 4 != 0 || opt.segments > 4096) { printf("ERROR: --segments must be a multiple of 4 between 4 and 4096\n"); return kUsage; }
        }
        else if (a.size() > 2 && a.compare(0, 2, "--") == 0) { printf("ERROR: unknown option %s\n", a.c_str()); PrintUsage(); return kUsage; }
        else pos.push_back(a);
    }
    if (pos.size() != 2) { PrintUsage(); return kUsage; }
    opt.input = pos[0];
    opt.output = pos[1];
    g_quiet = opt.quiet;
    g_verbose = opt.verbose && !opt.quiet;

    ExchangeSession hoops(HOOPS_BINARY_DIRECTORY, HOOPS_LICENSE);
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
int main(int argc, char** argv)
{
    return Main(std::vector<std::string>(argv, argv + argc));
}
#endif
