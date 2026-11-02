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
/// \example USD_Writer
/// CAD / PRC to OpenUSD (.usda/.usdc/.usd/.usdz) writer using HOOPS Exchange and OpenUSD.
///
/// 1. load any CAD file with HOOPS Exchange
/// 2. write the USD stage (meshes, structure, PMI, cameras, looks)
/// 3. re-open the written file and compare it with the HOOPS Exchange model
////////////////////////////////////////////////////////////////////////////////

/***********************************************************************************************************************
 * USD_Writer — writes any CAD file HOOPS Exchange reads (PRC, STEP, CATIA, NX, JT, SolidWorks, ...) as OpenUSD
 * (.usda / .usdc / .usd / .usdz).
 *
 *   The model is loaded with HOOPS Exchange (B-rep tessellated at load, --lod); the USD stage is written with the
 *   OpenUSD toolkit. Conventions shared with USD_Reader (matrices, units, looks, names, attributes) come from
 *   ../usd_common/hoops_usd.hpp, so CAD -> USD -> PRC keeps structure, placement, colours and instancing.
 *
 *   PRC                                        USD
 *   -----------------------------------------  -------------------------------------------------------------------
 *   model file unit (mm per unit)              metersPerUnit; upAxis Z (CAD convention)
 *   product occurrence tree                    Xform prims: valid unique names, original name in displayName,
 *                                              location as xformOp:transform, PRC attributes as userProperties:*
 *   identical sub-trees / parts used twice+    one prim under the abstract /Prototypes, instanceable references
 *   tessellated representation items           Mesh: triangles, indexed face-varying normals, subdivisionScheme none,
 *                                              extent, primvars:hoops:faceId (PRC face index per triangle),
 *                                              doubleSided for open shells
 *   resolved styles (A3DMiscCascadedAttributes) colours -> displayColor/displayOpacity (constant or per triangle);
 *                                              PRC materials -> UsdPreviewSurface, per-face via GeomSubsets
 *   hidden / removed                           visibility = invisible / not written
 *   product configurations (multi-entry CAD)   variant set "configuration" on the root prim (default selected)
 *   PMI markups (tessellation, leaders, text)  per markup an Xform under a "PMI" Scope (purpose guide by default):
 *                                              polylines -> BasisCurves, triangles -> Mesh, text -> glyph outlines
 *                                              (A3DGlobalFontTextTessellationGet); type, label, text and linked
 *                                              topology as hoops:pmi:* attributes
 *   views (A3DMkpView)                         Camera prims under a "Views" Scope; hoops:view:annotations
 *                                              relationship; variant set "view" on the root prim showing only the
 *                                              annotations of the selected view
 *   --layout parts                             each part (and each shared sub-assembly) in its own file under
 *                                              <output stem>_parts/, with its own materials, referenced by relative
 *                                              asset path (instanceable when used twice or more); --payloads
 *                                              makes the assembly's references payloads (load on demand)
 *   --layout layers                            the main file stacks one layer per kind of data under
 *                                              <output stem>_layers/: views, pmi, looks, geometry (mute a layer to
 *                                              drop that data)
 *
 *   Grounded in: HOOPS Exchange 2026.8 samples viewers/Viewer/traverse (TreeTraverse.cpp, TessConnector.cpp,
 *   VisitorCascadedAttribute.cpp, MarkupTraverse.cpp, MarkupTessConnector.cpp, ViewTraverse.cpp),
 *   reading_modes/LoadMultiConfigCADFile, viewers/MeshViewer/model/he; A3DSDKMarkup.h, A3DCommonReadWrite.h
 *   (A3DRWParamsMultiEntriesData); headers A3DSDKGraphics.h (cascaded attributes),
 *   A3DSDKTessellation.h, A3DCommonReadWrite.h; OpenUSD 25.08 headers (usd, usdGeom, usdShade, usdUtils).
 ***********************************************************************************************************************/
#define INITIALIZE_A3D_API
#include <A3DSDKIncludes.h>
#include <hoops_license.h>

#include <pxr/pxr.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/range3d.h>
#include <pxr/base/tf/errorMark.h>
#include <pxr/base/tf/stringUtils.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/vt/dictionary.h>
#include <pxr/base/vt/types.h>
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/listOp.h>
#include <pxr/usd/sdf/reference.h>
#include <pxr/usd/sdf/schema.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/references.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/editContext.h>
#include <pxr/usd/usd/variantSets.h>
#include <pxr/usd/pcp/errors.h>
#include <pxr/usd/sdf/layerUtils.h>
#include <pxr/usd/usd/payloads.h>
#include <pxr/usd/usd/references.h>
#include <pxr/usd/usdGeom/basisCurves.h>
#include <pxr/usd/usdGeom/camera.h>
#include <pxr/usd/usdGeom/curves.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/scope.h>
#include <pxr/usd/usdGeom/subset.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/xformCache.h>
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>
#include <pxr/usd/usdShade/shader.h>
#include <pxr/usd/usdUtils/dependencies.h>
#include <pxr/usd/usdUtils/usdzPackage.h>

#include "hoops_usd.hpp"

#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace hoops_usd;
namespace fs = std::filesystem;

// ---------------------------------------------------------------------------------------------------------------------
// Options
// ---------------------------------------------------------------------------------------------------------------------
struct Options
{
    std::string input, output;
    bool force = false, quiet = false, verbose = false, dumpTree = false;
    bool skipHidden = false, instancing = true;
    std::string layout = "monolithic";          // monolithic | parts | layers
    bool payloads = false;                      // --layout parts: payloads instead of references
    std::string pmi = "guide";                  // guide | default | none
    bool allViews = false;                      // also the views of nested parts that show no annotation
    bool allConfigurations = true;              // false: the default configuration only
    std::string configuration;                  // a single named configuration
    A3DETessellationLevelOfDetail lod = kA3DTessLODMedium;
    std::string lodName = "medium";
};

static bool g_debugPmi = false;   // --debug-pmi (diagnostic, undocumented): print the markup tessellation codes

static void PrintUsage()
{
    printf("Usage: USD_Writer <input CAD file> <output.usda|usdc|usd|usdz> [options]\n"
           "Writes any file HOOPS Exchange reads (PRC, STEP, CATIA, NX, JT, SolidWorks, ...) as OpenUSD:\n"
           "assembly structure, tessellated geometry, colours, materials, instancing, visibility and attributes.\n"
           "Options:\n"
           "  --force                 overwrite the output if it exists\n"
           "  --lod <level>           B-rep tessellation: extralow, low, medium (default), high, extrahigh\n"
           "  --no-instancing         write every occurrence in full (default: shared parts/sub-assemblies are\n"
           "                          written once under /Prototypes and referenced as instanceable prims)\n"
           "  --layout <layout>       monolithic (default: one file); parts (each part and shared sub-assembly in\n"
           "                          its own file under <output stem>_parts/, referenced from the assembly file);\n"
           "                          layers (one layer per kind of data under <output stem>_layers/: geometry,\n"
           "                          looks, pmi, views, stacked by the main file)\n"
           "  --payloads              with --layout parts: the assembly loads its parts as payloads (on demand)\n"
           "  --skip-hidden           do not write hidden objects (default: written with visibility = invisible)\n"
           "  --pmi <mode>            PMI and views: guide (default: written with purpose guide), default (always\n"
           "                          drawn), none (not read)\n"
           "  --all-views             also write the views of nested parts that show no annotation\n"
           "  --default-configuration write only the default configuration of a multi-configuration file\n"
           "                          (default: every configuration, as the variant set 'configuration')\n"
           "  --configuration <name>  write only the named configuration\n"
           "  --dump-tree             print the USD prim tree read back from the written file\n"
           "  --quiet / --verbose     fewer messages / also print the HOOPS Exchange reader messages\n"
           "Exit codes: 0 ok, 1 usage, 2 input/read error, 3 unsupported output format, 4 USD write failed,\n"
           "            5 verification of the written USD failed\n");
}

// ---------------------------------------------------------------------------------------------------------------------
// Hashing (structure signatures for instancing)
// ---------------------------------------------------------------------------------------------------------------------
struct Hash
{
    uint64_t h = 1469598103934665603ULL;
    Hash& Bytes(const void* p, size_t n)
    {
        const unsigned char* b = static_cast<const unsigned char*>(p);
        for (size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 1099511628211ULL; }
        return *this;
    }
    template <class T> Hash& Pod(const T& v) { return Bytes(&v, sizeof v); }
    Hash& Str(const std::string& s) { Pod(s.size()); return Bytes(s.data(), s.size()); }
    Hash& Mat(const GfMatrix4d& m) { return Bytes(m.GetArray(), sizeof(double) * 16); }
};

// ---------------------------------------------------------------------------------------------------------------------
// Analysis model: what will be written, independent of USD
// ---------------------------------------------------------------------------------------------------------------------
struct TessMesh                   // a decoded A3DTess3D (shared by every occurrence that uses it)
{
    VtVec3fArray points, normals;
    VtIntArray faceVertexIndices, normalIndices, faceIds;   // per corner, per corner, per triangle
    std::vector<int> triangleFace;                           // PRC face index of each triangle (== faceIds)
    A3DUns32 prcFaces = 0;
    long long badFaces = 0;
    uint64_t content = 0;                                    // hash of the decoded data (same in every load)
};

struct MeshUse                    // a tessellated representation item in one occurrence context
{
    std::string name;
    int tess = -1;
    GfMatrix4d local{1.0};
    bool hidden = false, doubleSided = false;
    std::vector<Look> faceLooks;  // one per PRC face
    uint64_t sig = 0;             // per load (tessellation index)
    uint64_t content = 0;         // load-independent (tessellation data): shares part files across configurations
};

struct AttrValue { std::string name; VtValue value; };

struct PmiItem                    // one markup (annotation item) with its leaders, in its occurrence's space
{
    std::string name, type, label, text;
    int subType = 0;
    std::vector<std::string> linked;                 // "<target occurrence>:<entity type>[:indices]"
    std::vector<std::vector<GfVec3d>> lines, glyphs;  // polylines; text outlines
    std::vector<GfVec3d> triangles;                  // 3 points per triangle
    GfVec3d color{0, 0, 0};
    bool screenAligned = false;                      // had frame-draw / face-view / fixed-size content
    const void* entity = nullptr;                    // the annotation item, as referenced by views
};

struct ViewItem
{
    std::string name;
    bool isDefault = false, isAnnotationView = false, hasCamera = false, ortho = false;
    GfVec3d location{0.0}, lookAt{0.0}, up{0, 0, 1};
    double xFov = 0, yFov = 0, zNear = 0, zFar = 0;
    std::vector<const void*> annotations;            // annotation items shown by the view
};

struct Node
{
    std::string name;
    GfMatrix4d local{1.0};
    bool hidden = false;          // hidden while its parent is shown
    std::vector<AttrValue> attrs;
    std::vector<MeshUse> meshes;
    std::vector<Node*> kids;
    std::vector<PmiItem> pmi;
    std::vector<ViewItem> views;
    bool hasPmi = false;          // this node or a descendant has PMI / views (never instanced: views address it)
    uint64_t origin = 0, sig = 0, key = 0;
    uint64_t content = 0;         // load-independent content hash (0 when hasPmi)
};

struct Stats
{
    long occurrences = 0, meshes = 0, hiddenNodes = 0, hiddenMeshes = 0, removed = 0, attributes = 0;
    long long triangles = 0, badFaces = 0, skippedFaces = 0;
    long prototypes = 0, instances = 0, materials = 0, partFiles = 0, partReferences = 0, payloads = 0, layerFiles = 0;
    long pmi = 0, views = 0, cameras = 0, textFailures = 0, pmiCurves = 0, viewVariants = 0, viewsSkipped = 0;
};

// ---------------------------------------------------------------------------------------------------------------------
// PRC -> analysis model
// ---------------------------------------------------------------------------------------------------------------------
class Analyzer
{
public:
    Analyzer(const Options& o, int configId, double modelUnitMm) : opt_(o), configId_(configId), modelUnit_(modelUnitMm) {}

    std::deque<Node> nodes;
    std::vector<TessMesh> tess;
    Stats st;

    Node* Build(const A3DAsmModelFile* mf, const std::string& rootName)
    {
        A3DAsmModelFileData md = A3D_MAKE_DATA(A3DAsmModelFileData);
        CHECK(A3DAsmModelFileGet(mf, &md));
        A3DMiscCascadedAttributes* rootAttr = nullptr;
        CHECK(A3DMiscCascadedAttributesCreate(&rootAttr));
        Node* root = NewNode();
        root->name = rootName;
        for (A3DUns32 i = 0; i < md.m_uiPOccurrencesSize; ++i)
            if (Node* n = Occurrence(md.m_ppPOccurrences[i], rootAttr, true, 0)) root->kids.push_back(n);
        A3DMiscCascadedAttributesDelete(rootAttr);
        A3DAsmModelFileGet(nullptr, &md);
        Signature(root);
        return root;
    }

    // One configuration of a multi-configuration file: the container's cascaded attributes, then the configuration.
    Node* BuildConfiguration(const A3DAsmProductOccurrence* container, const A3DAsmProductOccurrence* config)
    {
        A3DMiscCascadedAttributes* rootAttr = nullptr;
        CHECK(A3DMiscCascadedAttributesCreate(&rootAttr));
        A3DMiscCascadedAttributes* cAttr = nullptr;
        CHECK(A3DMiscCascadedAttributesCreate(&cAttr));
        CHECK(A3DMiscCascadedAttributesPush(cAttr, container, rootAttr));
        // The container's location applies to the configuration (the container is not written as a prim).
        Node* n = Occurrence(config, cAttr, true, 0);
        A3DMiscCascadedAttributesDelete(cAttr);
        A3DMiscCascadedAttributesDelete(rootAttr);
        if (n)
        {
            n->local = n->local * FromPrcTransform(EffectiveLocation(container));
            Signature(n);
        }
        return n;
    }

    static std::string EntityNameOf(const A3DEntity* e) { return EntityName(e); }

private:
    const Options& opt_;
    int configId_ = 0;
    double modelUnit_ = 1.0;   // mm per model-file unit: every occurrence is expressed in it
    std::map<const void*, int> tessIndex_;

    Node* NewNode() { nodes.emplace_back(); return &nodes.back(); }

    static std::string EntityName(const A3DEntity* e)
    {
        A3DRootBaseData bd = A3D_MAKE_DATA(A3DRootBaseData);
        if (!e || A3DRootBaseGet(e, &bd) != A3D_SUCCESS) return "";
        std::string n = bd.m_pcName ? bd.m_pcName : "";
        A3DRootBaseGet(nullptr, &bd);
        return n;
    }

    static std::string OccurrenceName(const A3DAsmProductOccurrence* po)
    {
        // Own name, else the name along the prototype / external-data chain.
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

    // PRC attributes -> USD custom attributes "userProperties:<title>:<key>" (typed when PRC says so).
    void Attributes(const A3DEntity* e, std::vector<AttrValue>& out)
    {
        A3DRootBaseData bd = A3D_MAKE_DATA(A3DRootBaseData);
        if (!e || A3DRootBaseGet(e, &bd) != A3D_SUCCESS) return;
        for (A3DUns32 i = 0; i < bd.m_uiSize; ++i)
        {
            A3DMiscAttributeData ad = A3D_MAKE_DATA(A3DMiscAttributeData);
            if (A3DMiscAttributeGet(bd.m_ppAttributes[i], &ad) != A3D_SUCCESS) continue;
            const std::string title = ad.m_pcTitle && !ad.m_bTitleIsInt ? ad.m_pcTitle : "";
            for (A3DUns32 k = 0; k < ad.m_uiSize; ++k)
            {
                const A3DMiscSingleAttributeData& s = ad.m_asSingleAttributesData[k];
                std::string key = s.m_pcTitle && !s.m_bTitleIsInt ? s.m_pcTitle : TfStringPrintf("item%u", k);
                std::string name = kUserPropertiesNs;
                if (!title.empty() && title != "USD") name += ValidPrimName(title) + ":";   // "USD" = USD_Reader's own block
                name += ValidPrimName(key);
                VtValue v;
                switch (s.m_eType)
                {
                case kA3DModellerAttributeTypeInt: if (s.m_pcData) v = VtValue((int)*reinterpret_cast<const A3DInt32*>(s.m_pcData)); break;
                case kA3DModellerAttributeTypeReal: if (s.m_pcData) v = VtValue(*reinterpret_cast<const A3DDouble*>(s.m_pcData)); break;
                case kA3DModellerAttributeTypeTime: if (s.m_pcData) v = VtValue((int64_t)*reinterpret_cast<const A3DUns32*>(s.m_pcData)); break;
                default: v = VtValue(std::string(s.m_pcData ? s.m_pcData : "")); break;
                }
                if (!v.IsEmpty()) out.push_back({name, v});
            }
            A3DMiscAttributeGet(nullptr, &ad);
        }
        if (bd.m_pcPersistentId && *bd.m_pcPersistentId)
            out.push_back({std::string(kUserPropertiesNs) + "hoops:persistentId", VtValue(std::string(bd.m_pcPersistentId))});
        A3DRootBaseGet(nullptr, &bd);
    }

    int Tess(const A3DTess3D* t)
    {
        auto it = tessIndex_.find(t);
        if (it != tessIndex_.end()) return it->second;
        TessMesh m;
        A3DTessBaseData bd = A3D_MAKE_DATA(A3DTessBaseData);
        A3DTess3DData td = A3D_MAKE_DATA(A3DTess3DData);
        if (A3DTessBaseGet(t, &bd) != A3D_SUCCESS || A3DTess3DGet(t, &td) != A3D_SUCCESS) { tessIndex_[t] = -1; return -1; }
        const A3DUns32 nPts = bd.m_uiCoordSize / 3, nNrm = td.m_uiNormalSize / 3;
        m.points.resize(nPts);
        for (A3DUns32 i = 0; i < nPts; ++i) m.points[i] = GfVec3f((float)bd.m_pdCoords[3 * i], (float)bd.m_pdCoords[3 * i + 1], (float)bd.m_pdCoords[3 * i + 2]);
        m.normals.resize(nNrm);
        for (A3DUns32 i = 0; i < nNrm; ++i) m.normals[i] = GfVec3f((float)td.m_pdNormals[3 * i], (float)td.m_pdNormals[3 * i + 1], (float)td.m_pdNormals[3 * i + 2]);
        m.prcFaces = td.m_uiFaceTessSize;
        for (A3DUns32 f = 0; f < td.m_uiFaceTessSize; ++f)
        {
            const bool ok = DecodeTessFace(td, f, [&](const Corner& a, const Corner& b, const Corner& c) {
                const Corner* cs[3] = {&a, &b, &c};
                for (const Corner* k : cs)
                    if (k->p / 3 >= nPts || (nNrm && k->n / 3 >= nNrm)) { ++m.badFaces; return; }
                for (const Corner* k : cs)
                {
                    m.faceVertexIndices.push_back((int)(k->p / 3));
                    m.normalIndices.push_back(nNrm ? (int)(k->n / 3) : 0);
                }
                m.faceIds.push_back((int)f);
            });
            if (!ok) ++st.skippedFaces;
        }
        st.badFaces += m.badFaces;
        A3DTess3DGet(nullptr, &td);
        A3DTessBaseGet(nullptr, &bd);
        Hash ch;
        ch.Bytes(m.points.cdata(), m.points.size() * sizeof(GfVec3f)).Bytes(m.normals.cdata(), m.normals.size() * sizeof(GfVec3f));
        ch.Bytes(m.faceVertexIndices.cdata(), m.faceVertexIndices.size() * sizeof(int)).Bytes(m.normalIndices.cdata(), m.normalIndices.size() * sizeof(int));
        ch.Bytes(m.faceIds.cdata(), m.faceIds.size() * sizeof(int));
        m.content = ch.h;
        int idx = m.faceVertexIndices.empty() ? -1 : (int)tess.size();
        if (idx >= 0) tess.push_back(std::move(m));
        tessIndex_[t] = idx;
        return idx;
    }

    static bool IsOpenShell(const A3DRiRepresentationItem* ri)
    {
        A3DEEntityType t = kA3DTypeUnknown;
        A3DEntityGetType(ri, &t);
        bool open = true;
        if (t == kA3DTypeRiBrepModel)
        {
            A3DRiBrepModelData d = A3D_MAKE_DATA(A3DRiBrepModelData);
            if (A3DRiBrepModelGet(ri, &d) == A3D_SUCCESS) { open = !d.m_bSolid; A3DRiBrepModelGet(nullptr, &d); }
        }
        else if (t == kA3DTypeRiPolyBrepModel)
        {
            A3DRiPolyBrepModelData d = A3D_MAKE_DATA(A3DRiPolyBrepModelData);
            if (A3DRiPolyBrepModelGet(ri, &d) == A3D_SUCCESS) { open = !d.m_bIsClosed; A3DRiPolyBrepModelGet(nullptr, &d); }
        }
        return open;
    }

    // Representation items (through RiSets) with the cascaded attributes of their context.
    void Items(const A3DRiRepresentationItem* ri, const A3DMiscCascadedAttributes* father, bool fatherShown,
               const std::string& partName, Node* node)
    {
        A3DMiscCascadedAttributes* attr = nullptr;
        CHECK(A3DMiscCascadedAttributesCreate(&attr));
        CHECK(A3DMiscCascadedAttributesPush(attr, ri, father));
        A3DMiscCascadedAttributesData cad = A3D_MAKE_DATA(A3DMiscCascadedAttributesData);
        CHECK(A3DMiscCascadedAttributesGet(attr, &cad));
        const bool removed = cad.m_bRemoved, shown = cad.m_bShow;
        A3DMiscCascadedAttributesGet(nullptr, &cad);
        if (removed || (!shown && opt_.skipHidden)) { ++st.removed; A3DMiscCascadedAttributesDelete(attr); return; }

        A3DEEntityType type = kA3DTypeUnknown;
        A3DEntityGetType(ri, &type);
        if (type == kA3DTypeRiSet)
        {
            A3DRiSetData sd = A3D_MAKE_DATA(A3DRiSetData);
            if (A3DRiSetGet(ri, &sd) == A3D_SUCCESS)
            {
                for (A3DUns32 i = 0; i < sd.m_uiRepItemsSize; ++i) Items(sd.m_ppRepItems[i], attr, shown, partName, node);
                A3DRiSetGet(nullptr, &sd);
            }
            A3DMiscCascadedAttributesDelete(attr);
            return;
        }

        A3DRiRepresentationItemData rd = A3D_MAKE_DATA(A3DRiRepresentationItemData);
        if (A3DRiRepresentationItemGet(ri, &rd) == A3D_SUCCESS)
        {
            A3DEEntityType tt = kA3DTypeUnknown;
            if (rd.m_pTessBase && A3DEntityGetType(rd.m_pTessBase, &tt) == A3D_SUCCESS && tt == kA3DTypeTess3D)
            {
                const int ti = Tess(rd.m_pTessBase);
                if (ti >= 0)
                {
                    MeshUse mu;
                    mu.tess = ti;
                    mu.name = EntityName(ri);
                    if (mu.name.empty()) mu.name = partName.empty() ? "Mesh" : partName;
                    mu.hidden = !shown && fatherShown;
                    mu.doubleSided = IsOpenShell(ri);
                    if (rd.m_pCoordinateSystem)
                    {
                        A3DRiCoordinateSystemData cs = A3D_MAKE_DATA(A3DRiCoordinateSystemData);
                        if (A3DRiCoordinateSystemGet(rd.m_pCoordinateSystem, &cs) == A3D_SUCCESS)
                        {
                            mu.local = FromPrcTransform(cs.m_pTransformation);
                            A3DRiCoordinateSystemGet(nullptr, &cs);
                        }
                    }
                    // Resolved style of each PRC face (face style, else the inherited one).
                    A3DTess3DData td = A3D_MAKE_DATA(A3DTess3DData);
                    if (A3DTess3DGet(rd.m_pTessBase, &td) == A3D_SUCCESS)
                    {
                        mu.faceLooks.resize(td.m_uiFaceTessSize);
                        for (A3DUns32 f = 0; f < td.m_uiFaceTessSize; ++f)
                        {
                            A3DMiscCascadedAttributes* fa = nullptr;
                            CHECK(A3DMiscCascadedAttributesCreate(&fa));
                            A3DMiscCascadedAttributesData fd = A3D_MAKE_DATA(A3DMiscCascadedAttributesData);
                            if (A3DMiscCascadedAttributesPushTessFace(fa, ri, rd.m_pTessBase, &td.m_psFaceTessData[f], f, attr) == A3D_SUCCESS &&
                                A3DMiscCascadedAttributesGet(fa, &fd) == A3D_SUCCESS)
                            {
                                mu.faceLooks[f] = LookFromStyle(fd.m_sStyle);
                                A3DMiscCascadedAttributesGet(nullptr, &fd);
                            }
                            A3DMiscCascadedAttributesDelete(fa);
                        }
                        A3DTess3DGet(nullptr, &td);
                    }
                    Hash h;
                    h.Pod(ti).Mat(mu.local).Pod(mu.hidden).Pod(mu.doubleSided).Str(mu.name);
                    for (const Look& l : mu.faceLooks) h.Str(l.Key());
                    mu.sig = h.h;
                    Hash c;
                    c.Pod(ti >= 0 ? tess[ti].content : 0).Mat(mu.local).Pod(mu.hidden).Pod(mu.doubleSided).Str(mu.name);
                    for (const Look& l : mu.faceLooks) c.Str(l.Key());
                    mu.content = c.h;
                    node->meshes.push_back(std::move(mu));
                }
            }
            A3DRiRepresentationItemGet(nullptr, &rd);
        }
        A3DMiscCascadedAttributesDelete(attr);
    }

    // ----- PMI ---------------------------------------------------------------------------------------------------
    static const char* MarkupTypeName(A3DEMarkupType t)
    {
        switch (t)
        {
        case kA3DMarkupTypeText: return "Text";
        case kA3DMarkupTypeDimension: return "Dimension";
        case kA3DMarkupTypeArrow: return "Arrow";
        case kA3DMarkupTypeBalloon: return "Balloon";
        case kA3DMarkupTypeCircleCenter: return "CircleCenter";
        case kA3DMarkupTypeCoordinate: return "Coordinate";
        case kA3DMarkupTypeDatum: return "Datum";
        case kA3DMarkupTypeFastener: return "Fastener";
        case kA3DMarkupTypeGdt: return "GDT";
        case kA3DMarkupTypeLocator: return "Locator";
        case kA3DMarkupTypeMeasurementPoint: return "MeasurementPoint";
        case kA3DMarkupTypeRoughness: return "Roughness";
        case kA3DMarkupTypeWelding: return "Welding";
        case kA3DMarkupTypeTable: return "Table";
        case kA3DMarkupTypeOther: return "Other";
        default: return "Unknown";
        }
    }

    // Wire tessellations of a glyph (A3DGlobalFontTextTessellationGet returns an RiSet of wire items).
    static void WirePolylines(const A3DRiRepresentationItem* ri, double dx, std::vector<std::vector<GfVec3d>>& out)
    {
        std::vector<const A3DRiRepresentationItem*> items;
        CollectTessItems(ri, items);
        for (const A3DRiRepresentationItem* it : items)
        {
            A3DRiRepresentationItemData rd = A3D_MAKE_DATA(A3DRiRepresentationItemData);
            if (A3DRiRepresentationItemGet(it, &rd) != A3D_SUCCESS) continue;
            A3DEEntityType tt = kA3DTypeUnknown;
            if (rd.m_pTessBase && A3DEntityGetType(rd.m_pTessBase, &tt) == A3D_SUCCESS && tt == kA3DTypeTess3DWire)
            {
                A3DTessBaseData bd = A3D_MAKE_DATA(A3DTessBaseData);
                A3DTess3DWireData wd = A3D_MAKE_DATA(A3DTess3DWireData);
                if (A3DTessBaseGet(rd.m_pTessBase, &bd) == A3D_SUCCESS && A3DTess3DWireGet(rd.m_pTessBase, &wd) == A3D_SUCCESS)
                {
                    const A3DUns32 nPts = bd.m_uiCoordSize / 3;
                    auto P = [&](A3DUns32 idx3) { return GfVec3d(bd.m_pdCoords[idx3] + dx, bd.m_pdCoords[idx3 + 1], bd.m_pdCoords[idx3 + 2]); };
                    if (wd.m_uiSizesWiresSize == 0)
                    {
                        std::vector<GfVec3d> pl;
                        for (A3DUns32 k = 0; k < nPts; ++k) pl.push_back(P(3 * k));
                        if (pl.size() > 1) out.push_back(pl);
                    }
                    for (A3DUns32 i = 0; i < wd.m_uiSizesWiresSize;)
                    {
                        const A3DUns32 head = wd.m_puiSizesWires[i];
                        const A3DUns32 n = head & ~(A3DUns32)(kA3DTess3DWireDataIsClosing | kA3DTess3DWireDataIsContinuous);
                        std::vector<GfVec3d> pl;
                        for (A3DUns32 k = 1; k <= n && i + k < wd.m_uiSizesWiresSize; ++k)
                            if (wd.m_puiSizesWires[i + k] / 3 < nPts) pl.push_back(P(wd.m_puiSizesWires[i + k]));
                        if ((head & kA3DTess3DWireDataIsClosing) && !pl.empty()) pl.push_back(pl.front());
                        if ((head & kA3DTess3DWireDataIsContinuous) && !out.empty() && !pl.empty()) out.back().insert(out.back().end(), pl.begin(), pl.end());
                        else if (pl.size() > 1) out.push_back(pl);
                        i += n + 1;
                    }
                    A3DTess3DWireGet(nullptr, &wd);
                    A3DTessBaseGet(nullptr, &bd);
                }
            }
            A3DRiRepresentationItemGet(nullptr, &rd);
        }
    }

    // Text as glyph outlines, laid out along +X from the origin of the current markup matrix and scaled so the string
    // spans the width stored in the tessellation.
    void TextGlyphs(const A3DFontKeyData& font, const std::string& text, double width, double height, const GfMatrix4d& M, PmiItem& p)
    {
        // Glyph outlines per character; characters without a glyph (spaces, unsupported) advance by an estimate.
        struct Glyph { std::vector<std::vector<GfVec3d>> strokes; double width = 0; bool ok = false; };
        std::vector<Glyph> glyphs;
        for (size_t i = 0; i < text.size();)
        {
            const unsigned char c = (unsigned char)text[i];
            const size_t len = c < 0x80 ? 1 : (c >> 5) == 6 ? 2 : (c >> 4) == 14 ? 3 : (c >> 3) == 30 ? 4 : 1;
            const std::string ch = text.substr(i, len);
            i += len;
            Glyph g;
            A3DRiSet* set = nullptr;
            A3DDouble cw = 0;
            if (A3DGlobalFontTextTessellationGet(&font, ch.c_str(), &set, &cw) == A3D_SUCCESS)
            {
                g.ok = true;
                g.width = cw;
                if (set) { WirePolylines(set, 0.0, g.strokes); A3DEntityDelete(set); }
            }
            glyphs.push_back(std::move(g));
        }
        double sum = 0;
        int n = 0;
        for (const Glyph& g : glyphs) if (g.ok && g.width > 0) { sum += g.width; ++n; }
        if (n == 0) { ++st.textFailures; return; }
        const double fallback = 0.6 * sum / n;
        std::vector<std::vector<GfVec3d>> strokes;
        double x = 0;
        for (Glyph& g : glyphs)
        {
            for (auto& pl : g.strokes)
            {
                for (GfVec3d& q : pl) q[0] += x;
                strokes.push_back(std::move(pl));
            }
            x += g.ok && g.width > 0 ? g.width : fallback;
        }
        if (strokes.empty()) return;
        double s = 1.0;
        if (x > 0 && width > 0) s = width / x;
        else if (height > 0)
        {
            double ymax = 0;
            for (const auto& pl : strokes) for (const GfVec3d& q : pl) ymax = std::max(ymax, q[1]);
            if (ymax > 0) s = height / ymax;
        }
        for (auto& pl : strokes)
        {
            for (GfVec3d& q : pl) q = M.Transform(q * s);
            p.glyphs.push_back(std::move(pl));
        }
    }

    // A3DTessMarkup code stream (HOOPS Exchange sample MarkupTessConnector.cpp): polylines, polygons, triangles,
    // matrix push/pop, colour, font and text. Camera-dependent modes (face view, frame draw, fixed size) are drawn
    // in model space.
    void DecodeMarkupTess(const A3DTessMarkup* tess, PmiItem& p)
    {
        if (!tess) return;
        A3DTessMarkupData md = A3D_MAKE_DATA(A3DTessMarkupData);
        A3DTessBaseData bd = A3D_MAKE_DATA(A3DTessBaseData);
        if (A3DTessMarkupGet(tess, &md) != A3D_SUCCESS) return;
        if (A3DTessBaseGet(tess, &bd) != A3D_SUCCESS) { A3DTessMarkupGet(nullptr, &md); return; }
        if (md.m_pcLabel && *md.m_pcLabel && p.label.empty()) p.label = md.m_pcLabel;
        for (A3DUns32 t = 0; t < md.m_uiTextsSize; ++t)
            if (md.m_ppcTexts[t] && *md.m_ppcTexts[t]) p.text += (p.text.empty() ? "" : "\n") + std::string(md.m_ppcTexts[t]);

        std::vector<GfMatrix4d> stack{GfMatrix4d(1.0)};
        std::vector<GfMatrix4d> modeStack;   // camera-dependent modes: content anchored at a model-space point
        A3DFontKeyData font = A3D_MAKE_DATA(A3DFontKeyData);
        const double* d = bd.m_pdCoords;
        const double* dEnd = bd.m_pdCoords + bd.m_uiCoordSize;
        const A3DUns32* codes = md.m_puiCodes;
        for (A3DUns32 i = 0; i + 1 < md.m_uiCodesSize;)
        {
            const A3DUns32 code = codes[i];
            const A3DUns32 cnt = code & kA3DMarkupIntegerMask;
            const A3DUns32 nd = codes[i + 1];
            if (d + nd > dEnd || i + 2 + cnt > md.m_uiCodesSize) break;   // inconsistent stream
            const GfMatrix4d M = stack.back();
            auto P = [&](const double* q) { return M.Transform(GfVec3d(q[0], q[1], q[2])); };
            if (g_debugPmi)
                printf("  [pmi] code 0x%08X extra=%d matrix=%d cnt=%u doubles=%u%s\n", code, (code & kA3DMarkupIsExtraData) ? (int)A3D_DECODE_EXTRA_DATA(code) : -1,
                       (code & kA3DMarkupIsMatrix) ? 1 : 0, cnt, nd, cnt ? TfStringPrintf(" int0=%u", codes[i + 2]).c_str() : "");
            if (code & kA3DMarkupIsExtraData)
            {
                const A3DUns32 type = A3D_DECODE_EXTRA_DATA(code);
                if (type == A3D_DECODE_EXTRA_DATA(kA3DMarkupFaceViewMask) || type == A3D_DECODE_EXTRA_DATA(kA3DMarkupFrameDrawMask) ||
                    type == A3D_DECODE_EXTRA_DATA(kA3DMarkupFixedSizeMask))
                {
                    // Mode switch, not a container (sample MarkupTessConnector.cpp): with 3 doubles it starts the mode
                    // at that anchor point, without doubles it ends it. The codes that follow are read normally; the
                    // screen-aligned content is drawn in the plane of the current matrix at the anchor.
                    if (nd >= 3)
                    {
                        GfMatrix4d T(1.0);
                        T.SetTranslate(GfVec3d(d[0], d[1], d[2]));
                        modeStack.push_back(stack.back());
                        stack.push_back(T * stack.back());
                        p.screenAligned = true;
                        d += 3;
                    }
                    else if (!modeStack.empty() && stack.size() > 1)
                    {
                        stack.pop_back();
                        modeStack.pop_back();
                    }
                    i += 2;
                    continue;
                }
                switch (type)
                {
                case A3D_DECODE_EXTRA_DATA(kA3DMarkupTrianglesMask):
                    for (A3DUns32 k = 0; k + 9 <= nd; k += 9) { p.triangles.push_back(P(d + k)); p.triangles.push_back(P(d + k + 3)); p.triangles.push_back(P(d + k + 6)); }
                    break;
                case A3D_DECODE_EXTRA_DATA(kA3DMarkupPolygonMask):
                {
                    std::vector<GfVec3d> pl;
                    for (A3DUns32 k = 0; k + 3 <= nd; k += 3) pl.push_back(P(d + k));
                    if (pl.size() > 2) pl.push_back(pl.front());
                    if (pl.size() > 1) p.lines.push_back(pl);
                    break;
                }
                case A3D_DECODE_EXTRA_DATA(kA3DMarkupColorMask):
                    if (cnt >= 1)
                    {
                        A3DGraphRgbColorData c = A3D_MAKE_DATA(A3DGraphRgbColorData);
                        if (A3DGlobalGetGraphRgbColorData(codes[i + 2], &c) == A3D_SUCCESS) p.color = GfVec3d(c.m_dRed, c.m_dGreen, c.m_dBlue);
                    }
                    break;
                case A3D_DECODE_EXTRA_DATA(kA3DMarkupFontMask):
                    if (cnt >= 2)
                    {
                        font.m_iFontFamilyIndex = (A3DInt32)codes[i + 2];
                        font.m_iFontStyleIndex = (A3DInt32)((codes[i + 3] & kA3DFontKeyStyle) >> 24);
                        font.m_iFontSizeIndex = (A3DInt32)((codes[i + 3] & kA3DFontKeySize) >> 12);
                        font.m_cAttributes = (A3DInt8)(codes[i + 3] & kA3DFontKeyAttrib);
                    }
                    break;
                case A3D_DECODE_EXTRA_DATA(kA3DMarkupTextMask):
                    if (cnt >= 1 && nd >= 2 && codes[i + 2] < md.m_uiTextsSize && md.m_ppcTexts[codes[i + 2]])
                        TextGlyphs(font, md.m_ppcTexts[codes[i + 2]], d[0], d[1], M, p);
                    break;
                default: break;   // points, cylinders, symbols, stipple, width, camera-dependent modes
                }
            }
            else if (code & kA3DMarkupIsMatrix)
            {
                // A matrix starts a block (its count and doubles include the codes inside): only its 16 doubles are
                // consumed here, the inner codes follow; a matrix code without doubles ends the block.
                if (nd >= 16)
                {
                    GfMatrix4d L;
                    for (int r = 0; r < 4; ++r)
                        for (int c = 0; c < 4; ++c) L[r][c] = d[r * 4 + c];
                    stack.push_back(L * stack.back());
                    d += 16;
                }
                else if (stack.size() > 1) stack.pop_back();
                i += 2;
                continue;
            }
            else
            {
                std::vector<GfVec3d> pl;
                for (A3DUns32 k = 0; k + 3 <= nd; k += 3) pl.push_back(P(d + k));
                if (pl.size() > 1) p.lines.push_back(pl);
            }
            d += nd;
            i += cnt + 2;
        }
        A3DTessBaseGet(nullptr, &bd);
        A3DTessMarkupGet(nullptr, &md);
    }

    std::string LinkedText(const A3DMiscMarkupLinkedItem* li)
    {
        A3DMiscMarkupLinkedItemData ld = A3D_MAKE_DATA(A3DMiscMarkupLinkedItemData);
        if (!li || A3DMiscMarkupLinkedItemGet(li, &ld) != A3D_SUCCESS) return "";
        std::string s = ld.m_pTargetProductOccurrence ? OccurrenceName(ld.m_pTargetProductOccurrence) : "";
        const A3DEntity* ref = ld.m_pReference;
        A3DEEntityType t = kA3DTypeUnknown;
        if (ref) A3DEntityGetType(ref, &t);
        if (t == kA3DTypeMiscEntityReference)
        {
            A3DMiscEntityReferenceData er = A3D_MAKE_DATA(A3DMiscEntityReferenceData);
            if (A3DMiscEntityReferenceGet(ref, &er) == A3D_SUCCESS)
            {
                ref = er.m_pEntity;
                A3DMiscEntityReferenceGet(nullptr, &er);
                t = kA3DTypeUnknown;
                if (ref) A3DEntityGetType(ref, &t);
            }
        }
        if (t == kA3DTypeMiscReferenceOnTopology)
        {
            A3DMiscReferenceOnTopologyData rt = A3D_MAKE_DATA(A3DMiscReferenceOnTopologyData);
            if (A3DMiscReferenceOnTopologyGet(ref, &rt) == A3D_SUCCESS)
            {
                const A3DUTF8Char* tn = A3DMiscGetEntityTypeMsg(rt.m_eTopoItemType);
                s += std::string(":") + (tn ? tn : "topology");
                for (A3DUns32 k = 0; k < rt.m_uiSize; ++k) s += (k ? "," : ":") + std::to_string(rt.m_puiAdditionalIndexes[k]);
                A3DMiscReferenceOnTopologyGet(nullptr, &rt);
            }
        }
        else if (ref)
        {
            const A3DUTF8Char* tn = A3DMiscGetEntityTypeMsg(t);
            s += std::string(":") + (tn ? tn : "entity");
            const std::string rn = EntityName(ref);
            if (!rn.empty()) s += ":" + rn;
        }
        A3DMiscMarkupLinkedItemGet(nullptr, &ld);
        return s;
    }

    void Annotation(const A3DMkpAnnotationEntity* e, std::vector<PmiItem>& out)
    {
        A3DEEntityType t = kA3DTypeUnknown;
        if (!e || A3DEntityGetType(e, &t) != A3D_SUCCESS) return;
        if (t == kA3DTypeMkpAnnotationSet)
        {
            A3DMkpAnnotationSetData sd = A3D_MAKE_DATA(A3DMkpAnnotationSetData);
            if (A3DMkpAnnotationSetGet(e, &sd) != A3D_SUCCESS) return;
            for (A3DUns32 i = 0; i < sd.m_uiAnnotationsSize; ++i) Annotation(sd.m_ppAnnotations[i], out);
            A3DMkpAnnotationSetGet(nullptr, &sd);
            return;
        }
        if (t != kA3DTypeMkpAnnotationItem) return;   // annotation references only point at other annotations
        for (const PmiItem& q : out) if (q.entity == e) return;   // listed by both the occurrence and its part
        A3DMkpAnnotationItemData id = A3D_MAKE_DATA(A3DMkpAnnotationItemData);
        if (A3DMkpAnnotationItemGet(e, &id) != A3D_SUCCESS) return;
        if (id.m_pMarkup)
        {
            A3DMkpMarkupData md = A3D_MAKE_DATA(A3DMkpMarkupData);
            if (A3DMkpMarkupGet(id.m_pMarkup, &md) == A3D_SUCCESS)
            {
                PmiItem p;
                p.entity = e;
                p.type = MarkupTypeName(md.m_eType);
                p.subType = (int)md.m_eSubType;
                p.name = EntityName(e);
                if (p.name.empty()) p.name = EntityName(id.m_pMarkup);
                DecodeMarkupTess(md.m_pTessellation, p);
                for (A3DUns32 l = 0; l < md.m_uiLeadersSize; ++l)
                {
                    A3DMkpLeaderData ld = A3D_MAKE_DATA(A3DMkpLeaderData);
                    if (md.m_ppLeaders[l] && A3DMkpLeaderGet(md.m_ppLeaders[l], &ld) == A3D_SUCCESS)
                    {
                        DecodeMarkupTess(ld.m_pTessellation, p);
                        A3DMkpLeaderGet(nullptr, &ld);
                    }
                }
                for (A3DUns32 l = 0; l < md.m_uiLinkedItemsSize; ++l)
                {
                    const std::string lt = LinkedText(md.m_ppLinkedItems[l]);
                    if (!lt.empty()) p.linked.push_back(lt);
                }
                if (p.name.empty()) p.name = p.label.empty() ? p.type : p.label;
                out.push_back(std::move(p));
                ++st.pmi;
                A3DMkpMarkupGet(nullptr, &md);
            }
        }
        A3DMkpAnnotationItemGet(nullptr, &id);
    }

    static void FlattenItems(const A3DMkpAnnotationEntity* e, std::vector<const void*>& out)
    {
        A3DEEntityType t = kA3DTypeUnknown;
        if (!e || A3DEntityGetType(e, &t) != A3D_SUCCESS) return;
        if (t == kA3DTypeMkpAnnotationItem) { out.push_back(e); return; }
        if (t != kA3DTypeMkpAnnotationSet) return;
        A3DMkpAnnotationSetData sd = A3D_MAKE_DATA(A3DMkpAnnotationSetData);
        if (A3DMkpAnnotationSetGet(e, &sd) != A3D_SUCCESS) return;
        for (A3DUns32 i = 0; i < sd.m_uiAnnotationsSize; ++i) FlattenItems(sd.m_ppAnnotations[i], out);
        A3DMkpAnnotationSetGet(nullptr, &sd);
    }

    void View(const A3DMkpView* v, std::vector<ViewItem>& out, int depth)
    {
        A3DMkpViewData vd = A3D_MAKE_DATA(A3DMkpViewData);
        if (!v || A3DMkpViewGet(v, &vd) != A3D_SUCCESS) return;
        ViewItem w;
        w.name = EntityName(v);
        if (w.name.empty()) w.name = TfStringPrintf("View%zu", out.size() + 1);
        w.isDefault = vd.m_bIsDefaultView;
        w.isAnnotationView = vd.m_bIsAnnotationView;
        for (A3DUns32 i = 0; i < vd.m_uiAnnotationsSize; ++i) FlattenItems(vd.m_ppAnnotations[i], w.annotations);
        // Views of nested parts (every SolidWorks part has its standard views) are written only when they show
        // annotations, unless --all-views.
        if (depth > 0 && w.annotations.empty() && !opt_.allViews) { ++st.viewsSkipped; A3DMkpViewGet(nullptr, &vd); return; }
        if (vd.m_pSceneDisplayParameters)
        {
            A3DGraphSceneDisplayParametersData sd = A3D_MAKE_DATA(A3DGraphSceneDisplayParametersData);
            if (A3DGraphSceneDisplayParametersGet(vd.m_pSceneDisplayParameters, &sd) == A3D_SUCCESS)
            {
                A3DGraphCameraData cd = A3D_MAKE_DATA(A3DGraphCameraData);
                if (sd.m_pCamera && A3DGraphCameraGet(sd.m_pCamera, &cd) == A3D_SUCCESS)
                {
                    w.hasCamera = true;
                    w.ortho = cd.m_bOrthographic;
                    w.location = GfVec3d(cd.m_sLocation.m_dX, cd.m_sLocation.m_dY, cd.m_sLocation.m_dZ);
                    w.lookAt = GfVec3d(cd.m_sLookAt.m_dX, cd.m_sLookAt.m_dY, cd.m_sLookAt.m_dZ);
                    w.up = GfVec3d(cd.m_sUp.m_dX, cd.m_sUp.m_dY, cd.m_sUp.m_dZ);
                    w.xFov = cd.m_dXFovy;
                    w.yFov = cd.m_dYFovy;
                    w.zNear = cd.m_dZNear;
                    w.zFar = cd.m_dZFar;
                    A3DGraphCameraGet(nullptr, &cd);
                }
                A3DGraphSceneDisplayParametersGet(nullptr, &sd);
            }
        }
        out.push_back(std::move(w));
        ++st.views;
        A3DMkpViewGet(nullptr, &vd);
    }

    // Annotations and views of an occurrence: its own, else its prototype's (sample TreeTraverse.cpp).
    void OccurrencePmi(const A3DAsmProductOccurrence* po, Node* n, int depth)
    {
        bool gotAnnotations = false, gotViews = false;
        for (const A3DAsmProductOccurrence* p = po; p && (!gotAnnotations || !gotViews);)
        {
            A3DAsmProductOccurrenceData d = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
            if (A3DAsmProductOccurrenceGet(p, &d) != A3D_SUCCESS) break;
            if (!gotAnnotations && d.m_uiAnnotationsSize)
            {
                for (A3DUns32 i = 0; i < d.m_uiAnnotationsSize; ++i) Annotation(d.m_ppAnnotations[i], n->pmi);
                gotAnnotations = true;
            }
            if (!gotViews && d.m_uiViewsSize)
            {
                for (A3DUns32 i = 0; i < d.m_uiViewsSize; ++i) View(d.m_ppViews[i], n->views, depth);
                gotViews = true;
            }
            const A3DAsmProductOccurrence* next = d.m_pPrototype;
            A3DAsmProductOccurrenceGet(nullptr, &d);
            p = next;
        }
    }

    void PartPmi(const A3DAsmPartDefinition* part, Node* n, int depth)
    {
        A3DAsmPartDefinitionData pd = A3D_MAKE_DATA(A3DAsmPartDefinitionData);
        if (A3DAsmPartDefinitionGet(part, &pd) != A3D_SUCCESS) return;
        for (A3DUns32 i = 0; i < pd.m_uiAnnotationsSize; ++i) Annotation(pd.m_ppAnnotations[i], n->pmi);
        for (A3DUns32 i = 0; i < pd.m_uiViewsSize; ++i) View(pd.m_ppViews[i], n->views, depth);
        A3DAsmPartDefinitionGet(nullptr, &pd);
    }

    Node* Occurrence(const A3DAsmProductOccurrence* po, const A3DMiscCascadedAttributes* father, bool fatherShown, int depth)
    {
        A3DMiscCascadedAttributes* attr = nullptr;
        CHECK(A3DMiscCascadedAttributesCreate(&attr));
        CHECK(A3DMiscCascadedAttributesPush(attr, po, father));
        A3DMiscCascadedAttributesEntityReferencePush(attr, po, nullptr);
        A3DMiscCascadedAttributesData cad = A3D_MAKE_DATA(A3DMiscCascadedAttributesData);
        CHECK(A3DMiscCascadedAttributesGet(attr, &cad));
        const bool removed = cad.m_bRemoved, shown = cad.m_bShow;
        A3DMiscCascadedAttributesGet(nullptr, &cad);
        if (removed || (!shown && opt_.skipHidden)) { ++st.removed; A3DMiscCascadedAttributesDelete(attr); return nullptr; }

        Node* n = NewNode();
        n->name = OccurrenceName(po);
        n->hidden = !shown && fatherShown;
        n->local = FromPrcTransform(EffectiveLocation(po));
        // An occurrence's own unit (m_dUnit) is informational: its location and content are already expressed in the
        // model-file unit of the load (checked on SolidWorks assemblies mixing inch and mm documents).
        Attributes(po, n->attrs);

        Hash origin;
        A3DAsmPartDefinition* part = EffectivePart(po);
        origin.Pod(part);
        if (part)
        {
            A3DMiscCascadedAttributes* pattr = nullptr;
            CHECK(A3DMiscCascadedAttributesCreate(&pattr));
            CHECK(A3DMiscCascadedAttributesPush(pattr, part, attr));
            if (n->name.empty()) n->name = EntityName(part);
            A3DAsmPartDefinitionData pd = A3D_MAKE_DATA(A3DAsmPartDefinitionData);
            if (A3DAsmPartDefinitionGet(part, &pd) == A3D_SUCCESS)
            {
                const std::string partName = EntityName(part);
                for (A3DUns32 i = 0; i < pd.m_uiRepItemsSize; ++i) Items(pd.m_ppRepItems[i], pattr, shown, partName, n);
                A3DAsmPartDefinitionGet(nullptr, &pd);
            }
            Attributes(part, n->attrs);
            if (opt_.pmi != "none") PartPmi(part, n, depth);
            A3DMiscCascadedAttributesDelete(pattr);
        }
        if (opt_.pmi != "none") OccurrencePmi(po, n, depth);
        for (A3DAsmProductOccurrence* k : EffectiveChildren(po))
        {
            origin.Pod(k);
            if (Node* kn = Occurrence(k, attr, shown, depth + 1)) n->kids.push_back(kn);
        }
        A3DMiscCascadedAttributesDelete(attr);
        if (n->name.empty()) n->name = "Occurrence";
        n->origin = origin.h;
        return n;
    }

    // Bottom-up content signature: identical content from the same PRC source can be shared.
    void Signature(Node* n)
    {
        Hash h, c;
        for (const MeshUse& m : n->meshes) { h.Pod(m.sig); c.Pod(m.content); }
        n->hasPmi = !n->pmi.empty() || !n->views.empty();
        for (Node* k : n->kids)
        {
            Signature(k);
            h.Str(k->name).Mat(k->local).Pod(k->hidden).Pod(k->sig);
            c.Str(k->name).Mat(k->local).Pod(k->hidden).Pod(k->content);
            for (const AttrValue& a : k->attrs) { h.Str(a.name).Str(TfStringify(a.value)); c.Str(a.name).Str(TfStringify(a.value)); }
            n->hasPmi |= k->hasPmi;
        }
        n->sig = h.h;
        n->content = n->hasPmi ? 0 : c.h;
        // Keys are per configuration (each configuration is a separate HOOPS Exchange load). Sub-trees with PMI or
        // views are never shared: views must be able to address their annotations.
        n->key = n->hasPmi ? Hash().Pod(configId_).Pod(n).h : Hash().Pod(configId_).Pod(n->origin).Pod(n->sig).h;
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Analysis model -> USD stage
// ---------------------------------------------------------------------------------------------------------------------
class UsdEmitter
{
public:
    UsdEmitter(const Options& o, UsdStageRefPtr s, Stats& st) : opt_(o), stage_(s), st_(st) {}

    // --layout parts: part files are written into <dir> (a staging folder) and referenced as "./<folder>/<file>"
    // from the main file and as "./<file>" between part files. <ext> is ".usda" or ".usdc".
    void SetPartFiles(const fs::path& dir, const std::string& folder, const std::string& ext)
    {
        partsDir_ = dir;
        partsFolder_ = folder;
        partExt_ = ext;
    }

    // --layout layers: one layer per kind of data in <dir>, stacked by the main file as "./<folder>/<kind><ext>".
    // Call before Begin.
    void SetLayers(const fs::path& dir, const std::string& folder, const std::string& ext)
    {
        static const char* kinds[kLayerCount] = {"geometry", "looks", "pmi", "views"};
        for (int k = 0; k < kLayerCount; ++k)
        {
            layerFiles_[k] = std::string(kinds[k]) + ext;
            layers_[k] = SdfLayer::CreateNew((dir / fs::u8path(layerFiles_[k])).u8string());
            if (!layers_[k]) throw std::runtime_error("cannot create the layer " + layerFiles_[k]);
        }
        layersFolder_ = folder;
        // Strongest first: views and PMI over looks over geometry (they only add prims and visibility opinions).
        std::vector<std::string> sub;
        for (int k : {kViews, kPmi, kLooks, kGeometry}) sub.push_back("./" + folder + "/" + layerFiles_[k]);
        stage_->GetRootLayer()->SetSubLayerPaths(sub);
        stage_->SetEditTarget(UsdEditTarget(layers_[kGeometry]));
    }

    // --layout layers: drops empty layers from the stack, saves the others; false (with a message) if one fails.
    bool SaveLayers()
    {
        if (!layers_[kGeometry]) return true;
        std::vector<std::string> sub;
        for (int k : {kViews, kPmi, kLooks, kGeometry})
        {
            if (layers_[k]->GetPseudoRoot()->GetNameChildren().empty()) continue;
            // Stage metadata on every layer, so each one opens on its own with the right units.
            layers_[k]->SetField(SdfPath::AbsoluteRootPath(), UsdGeomTokens->metersPerUnit, VtValue(mpu_));
            layers_[k]->SetField(SdfPath::AbsoluteRootPath(), UsdGeomTokens->upAxis, VtValue(UsdGeomTokens->z));
            if (k == kGeometry) layers_[k]->SetDefaultPrim(rootPath_.GetNameToken());
            if (!layers_[k]->Save()) { printf("ERROR: cannot save the layer %s\n", layerFiles_[k].c_str()); return false; }
            sub.push_back("./" + layersFolder_ + "/" + layerFiles_[k]);
            writtenLayers_.push_back(layerFiles_[k]);
            ++st_.layerFiles;
        }
        stage_->GetRootLayer()->SetSubLayerPaths(sub);
        return true;
    }
    const std::vector<std::string>& WrittenLayers() const { return writtenLayers_; }

    // Saves every part file; false (with a message) if one fails.
    bool SaveParts()
    {
        for (auto& [key, pf] : partFiles_)
            if (!pf.stage->GetRootLayer()->Save()) { printf("ERROR: cannot save the part file %s\n", pf.file.c_str()); return false; }
        return true;
    }

    // Stage metadata and the root prim (the default prim).
    void Begin(const std::string& rootName, double metersPerUnit, const std::string& source)
    {
        mpu_ = metersPerUnit;
        source_ = source;
        StageMetadata(stage_);
        rootPath_ = SdfPath::AbsoluteRootPath().AppendChild(TfToken(ValidPrimName(rootName)));
        UsdGeomXform rx = UsdGeomXform::Define(stage_, rootPath_);
        if (rx.GetPrim().GetName().GetString() != rootName) rx.GetPrim().SetDisplayName(rootName);
        stage_->SetDefaultPrim(rx.GetPrim());
        looksPath_ = rootPath_.AppendChild(TfToken("Looks"));
        rootNames_.Unique("Looks");
    }

    const SdfPath& RootPath() const { return rootPath_; }

    void StageMetadata(const UsdStageRefPtr& s) const
    {
        UsdEditContext rootLayer(s, UsdEditTarget(s->GetRootLayer()));   // stage metadata lives in the root layer
        UsdGeomSetStageMetersPerUnit(s, mpu_);
        UsdGeomSetStageUpAxis(s, UsdGeomTokens->z);
        VtDictionary info;
        info["creator"] = VtValue(std::string("USD_Writer (HOOPS Exchange + OpenUSD)"));
        info["source"] = VtValue(source_);
        s->GetRootLayer()->SetCustomLayerData(info);
    }

    // A single model: its top-level occurrences under the root prim.
    void EmitModel(const Analyzer& an, const Node* model)
    {
        an_ = &an;
        // Instancing: count how often each content key is reached in the de-duplicated tree.
        if (opt_.instancing) for (const Node* k : model->kids) Count(k);
        for (const Node* k : model->kids) EmitNode(k, rootPath_, rootNames_);
        EmitViewVariants();
    }

    // Configurations: variant set "configuration" on the root prim; each variant holds one configuration's tree.
    // Prototypes and materials are authored outside the variants (root layer), so variants share them.
    std::vector<std::string> EmitConfigurations(const std::vector<std::pair<const Analyzer*, const Node*>>& cfgs,
                                                const std::vector<std::string>& names, size_t defaultIndex)
    {
        UsdEditContext geometry(stage_, Target(kGeometry, true));
        UsdPrim root = stage_->GetPrimAtPath(rootPath_);
        UsdVariantSet vs = root.GetVariantSets().AddVariantSet("configuration");
        SiblingNames vnames;
        std::vector<std::string> variantNames;
        for (size_t k = 0; k < cfgs.size(); ++k)
        {
            const std::string vn = vnames.Unique(names[k]);
            variantNames.push_back(vn);
            vs.AddVariant(vn);
            vs.SetVariantSelection(vn);
            an_ = cfgs[k].first;
            UsdEditContext ctx(vs.GetVariantEditContext());
            varSel_ = rootPath_.AppendVariantSelection("configuration", vn);
            SiblingNames siblings = rootNames_;
            if (opt_.instancing) Count(cfgs[k].second);
            EmitNode(cfgs[k].second, rootPath_, siblings);
            varSel_ = SdfPath();
        }
        vs.SetVariantSelection(variantNames[defaultIndex]);
        VtStringArray original(names.begin(), names.end());
        root.CreateAttribute(TfToken("hoops:configurations"), SdfValueTypeNames->StringArray, true).Set(original);
        viewRecords_.clear();   // per-view PMI visibility is only written for single-configuration files
        return variantNames;
    }

private:
    const Options& opt_;
    UsdStageRefPtr stage_;
    const Analyzer* an_ = nullptr;
    Stats& st_;
    SiblingNames rootNames_;

    struct ViewRecord { std::string name; bool isDefault; const Node* node; std::set<SdfPath> visible; };
    std::map<std::pair<const Node*, const void*>, SdfPath> pmiPaths_;   // (occurrence, annotation item) -> prim
    std::map<const Node*, std::vector<SdfPath>> nodePmi_;
    std::vector<ViewRecord> viewRecords_;
    SdfPath rootPath_, looksPath_;
    std::map<uint64_t, int> refCount_;
    std::map<uint64_t, SdfPath> protoPaths_;
    std::map<std::string, SdfPath> materials_;
    SiblingNames protoNames_, materialNames_;
    double mpu_ = 0.001;
    std::string source_;
    // --layout layers
    enum { kGeometry, kLooks, kPmi, kViews, kLayerCount };
    SdfLayerRefPtr layers_[kLayerCount];
    std::string layerFiles_[kLayerCount], layersFolder_;
    std::vector<std::string> writtenLayers_;
    SdfPath varSel_;   // the configuration variant being authored (empty outside variants)

    // Where data of one kind goes: its layer (the root layer when not layered), inside the current configuration
    // variant unless <outsideVariants>.
    UsdEditTarget Target(int kind, bool outsideVariants = false) const
    {
        SdfLayerHandle l = layers_[kind] ? SdfLayerHandle(layers_[kind]) : stage_->GetRootLayer();
        return varSel_.IsEmpty() || outsideVariants ? UsdEditTarget(l) : UsdEditTarget::ForLocalDirectVariant(l, varSel_);
    }
    // --layout parts
    struct PartFile { std::string file; UsdStageRefPtr stage; };
    fs::path partsDir_;
    std::string partsFolder_, partExt_;
    std::map<uint64_t, PartFile> partFiles_;
    SiblingNames partNames_;
    std::set<std::string> partFilesLower_;   // file names are case-insensitive on Windows
    bool inPart_ = false;                    // authoring into a part file (references are then "./<file>")

    static bool HasContent(const Node* n) { return !n->meshes.empty() || !n->kids.empty(); }

    // --layout parts: a part (an occurrence with geometry) or a shared sub-assembly goes to its own file. Sub-trees with
    // PMI or views stay in the main file (views and view variants address their markups by path).
    bool InPartFile(const Node* n) const
    {
        return opt_.layout == "parts" && HasContent(n) && !n->hasPmi && (!n->meshes.empty() || Shared(n));
    }

    // Writes the content of <n> once into its part file (root prim = default prim, own Looks) and returns the asset
    // path to reference it from the layer being authored.
    std::string PartAsset(const Node* n)
    {
        const std::string prefix = inPart_ ? "./" : "./" + partsFolder_ + "/";
        // Keyed by content, so identical parts of different configurations (separate loads) share one file.
        auto it = partFiles_.find(n->content);
        if (it != partFiles_.end()) return prefix + it->second.file;

        std::string primName = partNames_.Unique(n->name), file = primName + partExt_;
        for (int i = 1; partFilesLower_.count(Lower(file)); ++i) file = primName + "_f" + std::to_string(i) + partExt_;
        partFilesLower_.insert(Lower(file));
        UsdStageRefPtr ps = UsdStage::CreateNew((partsDir_ / fs::u8path(file)).u8string());
        if (!ps) throw std::runtime_error("cannot create the part file " + file);
        partFiles_[n->content] = PartFile{file, ps};
        ++st_.partFiles;
        StageMetadata(ps);
        const SdfPath root = SdfPath::AbsoluteRootPath().AppendChild(TfToken(primName));
        UsdGeomXform rx = UsdGeomXform::Define(ps, root);
        if (primName != n->name) rx.GetPrim().SetDisplayName(n->name);
        ps->SetDefaultPrim(rx.GetPrim());

        // Author the content in the part file: its own stage, materials and names.
        std::swap(stage_, ps);
        SdfPath looks = root.AppendChild(TfToken("Looks"));
        std::swap(looksPath_, looks);
        std::map<std::string, SdfPath> mats;
        std::swap(materials_, mats);
        SiblingNames matNames;
        std::swap(materialNames_, matNames);
        const bool wasInPart = inPart_;
        inPart_ = true;
        SdfPath variant;
        std::swap(varSel_, variant);   // a part file has no configuration variants
        EmitContent(n, root, "Looks");
        std::swap(varSel_, variant);
        inPart_ = wasInPart;
        std::swap(materialNames_, matNames);
        std::swap(materials_, mats);
        std::swap(looksPath_, looks);
        std::swap(stage_, ps);
        return prefix + partFiles_[n->content].file;
    }

    void Count(const Node* n)
    {
        if (!HasContent(n)) return;
        if (refCount_[n->key]++ > 0) return;   // a repeated sub-tree is only explored once
        for (const Node* k : n->kids) Count(k);
    }

    bool Shared(const Node* n) const
    {
        if (!opt_.instancing || !HasContent(n)) return false;
        auto it = refCount_.find(n->key);
        return it != refCount_.end() && it->second > 1;
    }

    static void SetTransform(const UsdGeomXformable& x, const GfMatrix4d& m)
    {
        if (!IsIdentity(m)) x.AddTransformOp().Set(m);
    }

    void SetAttributes(const UsdPrim& p, const std::vector<AttrValue>& attrs)
    {
        for (const AttrValue& a : attrs)
        {
            SdfValueTypeName t = a.value.IsHolding<int>() ? SdfValueTypeNames->Int
                               : a.value.IsHolding<double>() ? SdfValueTypeNames->Double
                               : a.value.IsHolding<int64_t>() ? SdfValueTypeNames->Int64 : SdfValueTypeNames->String;
            UsdAttribute attr = p.CreateAttribute(TfToken(a.name), t, /*custom*/ true);
            if (attr) { attr.Set(a.value); ++st_.attributes; }
        }
    }

    SdfPath Material(const Look& l)
    {
        auto it = materials_.find(l.Key());
        if (it != materials_.end()) return it->second;
        UsdEditContext looksLayer(stage_, Target(kLooks, true));   // outside any variant
        if (!stage_->GetPrimAtPath(looksPath_)) UsdGeomScope::Define(stage_, looksPath_);
        const std::string nm = materialNames_.Unique(TfStringPrintf("Material_%02X%02X%02X", (int)std::lround(std::clamp(l.diffuse[0], 0.0, 1.0) * 255),
                                                                    (int)std::lround(std::clamp(l.diffuse[1], 0.0, 1.0) * 255),
                                                                    (int)std::lround(std::clamp(l.diffuse[2], 0.0, 1.0) * 255)));
        const SdfPath mp = looksPath_.AppendChild(TfToken(nm));
        UsdShadeMaterial mat = UsdShadeMaterial::Define(stage_, mp);
        UsdShadeShader sh = UsdShadeShader::Define(stage_, mp.AppendChild(TfToken("PreviewSurface")));
        sh.CreateIdAttr(VtValue(TfToken("UsdPreviewSurface")));
        auto c3 = [](const GfVec3d& c) { return GfVec3f((float)c[0], (float)c[1], (float)c[2]); };
        sh.CreateInput(TfToken("diffuseColor"), SdfValueTypeNames->Color3f).Set(c3(l.diffuse));
        sh.CreateInput(TfToken("emissiveColor"), SdfValueTypeNames->Color3f).Set(c3(l.emissive));
        sh.CreateInput(TfToken("useSpecularWorkflow"), SdfValueTypeNames->Int).Set(1);
        sh.CreateInput(TfToken("specularColor"), SdfValueTypeNames->Color3f).Set(c3(l.specular));
        sh.CreateInput(TfToken("roughness"), SdfValueTypeNames->Float).Set((float)RoughnessFromShininess(l.shininess));
        sh.CreateInput(TfToken("opacity"), SdfValueTypeNames->Float).Set((float)l.opacity);
        UsdShadeOutput surf = sh.CreateOutput(TfToken("surface"), SdfValueTypeNames->Token);
        mat.CreateSurfaceOutput().ConnectToSource(surf);
        materials_[l.Key()] = mp;
        ++st_.materials;
        return mp;
    }

    void EmitMesh(const MeshUse& mu, const SdfPath& parent, SiblingNames& names)
    {
        const TessMesh& tm = an_->tess[mu.tess];
        const SdfPath path = parent.AppendChild(TfToken(names.Unique(mu.name)));
        UsdGeomMesh mesh = UsdGeomMesh::Define(stage_, path);
        UsdPrim prim = mesh.GetPrim();
        if (prim.GetName().GetString() != mu.name) prim.SetDisplayName(mu.name);
        SetTransform(mesh, mu.local);
        const size_t nTri = tm.faceIds.size();
        mesh.CreatePointsAttr().Set(tm.points);
        mesh.CreateFaceVertexCountsAttr().Set(VtIntArray(nTri, 3));
        mesh.CreateFaceVertexIndicesAttr().Set(tm.faceVertexIndices);
        mesh.CreateSubdivisionSchemeAttr().Set(UsdGeomTokens->none);
        if (mu.doubleSided) mesh.CreateDoubleSidedAttr().Set(true);
        VtVec3fArray extent;
        if (UsdGeomPointBased::ComputeExtent(tm.points, &extent)) mesh.CreateExtentAttr().Set(extent);
        UsdGeomPrimvarsAPI pv(prim);
        if (!tm.normals.empty())
        {
            UsdGeomPrimvar n = pv.CreatePrimvar(UsdGeomTokens->normals, SdfValueTypeNames->Normal3fArray, UsdGeomTokens->faceVarying);
            n.Set(tm.normals);
            n.SetIndices(tm.normalIndices);
        }
        UsdGeomPrimvar fid = pv.CreatePrimvar(TfToken(std::string(kHoopsNs) + "faceId"), SdfValueTypeNames->IntArray, UsdGeomTokens->uniform);
        fid.Set(tm.faceIds);
        if (mu.hidden) { mesh.CreateVisibilityAttr().Set(UsdGeomTokens->invisible); ++st_.hiddenMeshes; }

        // ---- appearance: colours as displayColor/displayOpacity, PRC materials as UsdPreviewSurface
        const std::vector<Look>& fl = mu.faceLooks;
        auto lookOf = [&](int f) -> const Look& { static const Look none; return f >= 0 && f < (int)fl.size() ? fl[f] : none; };
        bool uniformLook = true;
        for (size_t t = 1; t < nTri && uniformLook; ++t) uniformLook = lookOf(tm.faceIds[t]).Key() == lookOf(tm.faceIds[0]).Key();
        bool anyColor = false, anyMaterial = false;
        for (const Look& l : fl) { anyColor |= l.kind != Look::None; anyMaterial |= l.kind == Look::Material; }
        if (anyColor && nTri > 0)
        {
            const Look& l0 = lookOf(tm.faceIds[0]);
            if (uniformLook)
            {
                mesh.CreateDisplayColorPrimvar(UsdGeomTokens->constant).Set(VtVec3fArray(1, GfVec3f(l0.diffuse)));
                if (l0.opacity < 1) mesh.CreateDisplayOpacityPrimvar(UsdGeomTokens->constant).Set(VtFloatArray(1, (float)l0.opacity));
            }
            else
            {
                VtVec3fArray col(nTri);
                VtFloatArray op(nTri);
                bool anyOpacity = false;
                for (size_t t = 0; t < nTri; ++t)
                {
                    const Look& l = lookOf(tm.faceIds[t]);
                    col[t] = l.kind == Look::None ? GfVec3f(0.75f) : GfVec3f(l.diffuse);
                    op[t] = (float)l.opacity;
                    anyOpacity |= l.opacity < 1;
                }
                mesh.CreateDisplayColorPrimvar(UsdGeomTokens->uniform).Set(col);
                if (anyOpacity) mesh.CreateDisplayOpacityPrimvar(UsdGeomTokens->uniform).Set(op);
            }
        }
        if (anyMaterial && nTri > 0)
        {
            UsdShadeMaterialBindingAPI bind = UsdShadeMaterialBindingAPI::Apply(prim);
            const Look& l0 = lookOf(tm.faceIds[0]);
            if (uniformLook) bind.Bind(UsdShadeMaterial(stage_->GetPrimAtPath(Material(l0))));
            else
            {
                std::map<std::string, std::pair<Look, VtIntArray>> groups;   // material look -> triangles
                for (size_t t = 0; t < nTri; ++t)
                {
                    const Look& l = lookOf(tm.faceIds[t]);
                    if (l.kind != Look::Material) continue;
                    auto& g = groups[l.Key()];
                    g.first = l;
                    g.second.push_back((int)t);
                }
                SiblingNames subsetNames;
                for (auto& [key, g] : groups)
                {
                    const SdfPath mp = Material(g.first);
                    UsdGeomSubset sub = UsdGeomSubset::CreateGeomSubset(mesh, TfToken(subsetNames.Unique(mp.GetName())),
                                                                        UsdGeomTokens->face, g.second,
                                                                        UsdShadeTokens->materialBind, UsdGeomTokens->nonOverlapping);
                    UsdShadeMaterialBindingAPI::Apply(sub.GetPrim()).Bind(UsdShadeMaterial(stage_->GetPrimAtPath(mp)));
                }
            }
        }
    }

    void EmitContent(const Node* n, const SdfPath& path, const char* reserved = nullptr)
    {
        SiblingNames names;
        if (reserved) names.Unique(reserved);
        for (const MeshUse& m : n->meshes) EmitMesh(m, path, names);
        for (const Node* k : n->kids) EmitNode(k, path, names);
        if (!n->pmi.empty()) EmitPmi(n, path, names);
        if (!n->views.empty()) EmitViews(n, path, names);
    }

    // ----- PMI -----------------------------------------------------------------------------------------------------
    static VtVec3fArray Extent(const VtVec3fArray& pts)
    {
        GfRange3d r;
        for (const GfVec3f& q : pts) r.UnionWith(GfVec3d(q));
        VtVec3fArray e(2);
        e[0] = GfVec3f(r.GetMin());
        e[1] = GfVec3f(r.GetMax());
        return e;
    }

    void Curves(const SdfPath& p, const std::vector<std::vector<GfVec3d>>& lines, const GfVec3d& color)
    {
        UsdGeomBasisCurves bc = UsdGeomBasisCurves::Define(stage_, p);
        bc.CreateTypeAttr().Set(UsdGeomTokens->linear);
        VtIntArray counts;
        VtVec3fArray pts;
        for (const auto& l : lines)
        {
            if (l.size() < 2) continue;
            counts.push_back((int)l.size());
            for (const GfVec3d& q : l) pts.push_back(GfVec3f(q));
        }
        bc.CreateCurveVertexCountsAttr().Set(counts);
        bc.CreatePointsAttr().Set(pts);
        if (!pts.empty()) bc.CreateExtentAttr().Set(Extent(pts));
        bc.CreateDisplayColorPrimvar(UsdGeomTokens->constant).Set(VtVec3fArray(1, GfVec3f(color)));
        ++st_.pmiCurves;
    }

    void EmitPmi(const Node* n, const SdfPath& parent, SiblingNames& names)
    {
        UsdEditContext pmiLayer(stage_, Target(kPmi));
        const SdfPath scope = parent.AppendChild(TfToken(names.Unique("PMI")));
        UsdGeomScope sc = UsdGeomScope::Define(stage_, scope);
        if (opt_.pmi == "guide") sc.CreatePurposeAttr().Set(UsdGeomTokens->guide);
        SiblingNames pn;
        for (const PmiItem& p : n->pmi)
        {
            const SdfPath pp = scope.AppendChild(TfToken(pn.Unique(p.name)));
            UsdGeomXform x = UsdGeomXform::Define(stage_, pp);
            UsdPrim prim = x.GetPrim();
            if (prim.GetName().GetString() != p.name) prim.SetDisplayName(p.name);
            prim.CreateAttribute(TfToken("hoops:pmi:type"), SdfValueTypeNames->String, true).Set(p.type);
            prim.CreateAttribute(TfToken("hoops:pmi:subType"), SdfValueTypeNames->Int, true).Set(p.subType);
            if (!p.label.empty()) prim.CreateAttribute(TfToken("hoops:pmi:label"), SdfValueTypeNames->String, true).Set(p.label);
            if (!p.text.empty()) prim.CreateAttribute(TfToken("hoops:pmi:text"), SdfValueTypeNames->String, true).Set(p.text);
            if (p.screenAligned) prim.CreateAttribute(TfToken("hoops:pmi:screenAligned"), SdfValueTypeNames->Bool, true).Set(true);
            if (!p.linked.empty())
                prim.CreateAttribute(TfToken("hoops:pmi:linked"), SdfValueTypeNames->StringArray, true).Set(VtStringArray(p.linked.begin(), p.linked.end()));
            if (!p.lines.empty()) Curves(pp.AppendChild(TfToken("Lines")), p.lines, p.color);
            if (!p.glyphs.empty()) Curves(pp.AppendChild(TfToken("Text")), p.glyphs, p.color);
            if (p.triangles.size() >= 3)
            {
                UsdGeomMesh m = UsdGeomMesh::Define(stage_, pp.AppendChild(TfToken("Fill")));
                VtVec3fArray pts;
                for (const GfVec3d& q : p.triangles) pts.push_back(GfVec3f(q));
                const size_t nt = pts.size() / 3;
                pts.resize(nt * 3);
                VtIntArray idx(nt * 3);
                for (size_t k = 0; k < idx.size(); ++k) idx[k] = (int)k;
                m.CreatePointsAttr().Set(pts);
                m.CreateFaceVertexCountsAttr().Set(VtIntArray(nt, 3));
                m.CreateFaceVertexIndicesAttr().Set(idx);
                m.CreateSubdivisionSchemeAttr().Set(UsdGeomTokens->none);
                m.CreateDoubleSidedAttr().Set(true);
                m.CreateExtentAttr().Set(Extent(pts));
                m.CreateDisplayColorPrimvar(UsdGeomTokens->constant).Set(VtVec3fArray(1, GfVec3f(p.color)));
            }
            pmiPaths_[{n, p.entity}] = pp;
            nodePmi_[n].push_back(pp);
            ++st_.pmi;
        }
    }

    // ----- views ---------------------------------------------------------------------------------------------------
    void EmitViews(const Node* n, const SdfPath& parent, SiblingNames& names)
    {
        UsdEditContext viewsLayer(stage_, Target(kViews));
        const SdfPath scope = parent.AppendChild(TfToken(names.Unique("Views")));
        UsdGeomScope::Define(stage_, scope);
        SiblingNames vn;
        for (const ViewItem& v : n->views)
        {
            const SdfPath vp = scope.AppendChild(TfToken(vn.Unique(v.name)));
            UsdGeomCamera cam = UsdGeomCamera::Define(stage_, vp);
            UsdPrim prim = cam.GetPrim();
            if (prim.GetName().GetString() != v.name) prim.SetDisplayName(v.name);
            if (v.hasCamera)
            {
                // USD cameras look down -Z with +Y up.
                GfVec3d z = v.location - v.lookAt, up = v.up;
                if (z.GetLength() > 1e-12)
                {
                    z.Normalize();
                    GfVec3d x = GfCross(up, z);
                    if (x.GetLength() < 1e-9) x = GfCross(std::fabs(z[0]) < 0.9 ? GfVec3d(1, 0, 0) : GfVec3d(0, 1, 0), z);
                    x.Normalize();
                    const GfVec3d y = GfCross(z, x);
                    GfMatrix4d m(x[0], x[1], x[2], 0, y[0], y[1], y[2], 0, z[0], z[1], z[2], 0, v.location[0], v.location[1], v.location[2], 1);
                    cam.AddTransformOp().Set(m);
                }
                const double xf = v.xFov > 0 ? v.xFov : v.yFov, yf = v.yFov > 0 ? v.yFov : v.xFov;
                if (v.ortho)
                {
                    cam.CreateProjectionAttr().Set(UsdGeomTokens->orthographic);
                    // Orthographic apertures are in tenths of a scene unit.
                    if (xf > 0) cam.CreateHorizontalApertureAttr().Set((float)(xf * 10.0));
                    if (yf > 0) cam.CreateVerticalApertureAttr().Set((float)(yf * 10.0));
                }
                else if (xf > 0 && xf < M_PI)
                {
                    const double focal = 50.0;
                    cam.CreateProjectionAttr().Set(UsdGeomTokens->perspective);
                    cam.CreateFocalLengthAttr().Set((float)focal);
                    cam.CreateHorizontalApertureAttr().Set((float)(2 * focal * std::tan(xf / 2)));
                    cam.CreateVerticalApertureAttr().Set((float)(2 * focal * std::tan(yf / 2)));
                }
                if (v.zFar > v.zNear && v.zNear > 0) cam.CreateClippingRangeAttr().Set(GfVec2f((float)v.zNear, (float)v.zFar));
            }
            prim.CreateAttribute(TfToken("hoops:view:hasCamera"), SdfValueTypeNames->Bool, true).Set(v.hasCamera);
            prim.CreateAttribute(TfToken("hoops:view:isDefault"), SdfValueTypeNames->Bool, true).Set(v.isDefault);
            prim.CreateAttribute(TfToken("hoops:view:isAnnotationView"), SdfValueTypeNames->Bool, true).Set(v.isAnnotationView);
            ViewRecord rec{v.name, v.isDefault, n, {}};
            if (!v.annotations.empty())
            {
                UsdRelationship rel = prim.CreateRelationship(TfToken("hoops:view:annotations"), true);
                for (const void* e : v.annotations)
                {
                    auto it = pmiPaths_.find({n, e});
                    if (it == pmiPaths_.end()) continue;
                    rel.AddTarget(it->second);
                    rec.visible.insert(it->second);
                }
            }
            if (!rec.visible.empty()) viewRecords_.push_back(rec);
            ++st_.cameras;
        }
    }

    // Variant set "view" on the root prim: "All" shows every annotation; each view shows only its own.
    void EmitViewVariants()
    {
        if (viewRecords_.empty()) return;
        UsdEditContext pmiLayer(stage_, Target(kPmi, true));   // the per-view visibility goes with the PMI
        UsdPrim root = stage_->GetPrimAtPath(rootPath_);
        UsdVariantSet vs = root.GetVariantSets().AddVariantSet("view");
        SiblingNames vn;
        const std::string all = vn.Unique("All");
        vs.AddVariant(all);
        for (const ViewRecord& r : viewRecords_)
        {
            const std::string name = vn.Unique(r.name);
            vs.AddVariant(name);
            vs.SetVariantSelection(name);
            UsdEditContext ctx(vs.GetVariantEditContext());
            for (const SdfPath& p : nodePmi_[r.node])
                if (!r.visible.count(p)) UsdGeomImageable(stage_->GetPrimAtPath(p)).CreateVisibilityAttr().Set(UsdGeomTokens->invisible);
            ++st_.viewVariants;
        }
        vs.SetVariantSelection(all);
    }

    SdfPath Prototype(const Node* n)
    {
        auto it = protoPaths_.find(n->key);
        if (it != protoPaths_.end()) return it->second;
        UsdEditContext geometryLayer(stage_, Target(kGeometry, true));   // outside any variant
        static const SdfPath protos("/Prototypes");
        if (!stage_->GetPrimAtPath(protos)) stage_->CreateClassPrim(protos);
        const SdfPath p = protos.AppendChild(TfToken(protoNames_.Unique(n->name)));
        UsdGeomXform::Define(stage_, p);
        protoPaths_[n->key] = p;
        ++st_.prototypes;
        EmitContent(n, p);
        return p;
    }

    void EmitNode(const Node* n, const SdfPath& parent, SiblingNames& names)
    {
        const SdfPath path = parent.AppendChild(TfToken(names.Unique(n->name)));
        UsdGeomXform x = UsdGeomXform::Define(stage_, path);
        UsdPrim prim = x.GetPrim();
        if (prim.GetName().GetString() != n->name) prim.SetDisplayName(n->name);
        SetTransform(x, n->local);
        SetAttributes(prim, n->attrs);
        if (n->hidden) x.CreateVisibilityAttr().Set(UsdGeomTokens->invisible);
        if (InPartFile(n))
        {
            const std::string asset = PartAsset(n);
            if (opt_.payloads && !inPart_) { prim.GetPayloads().AddPayload(asset); ++st_.payloads; }   // the assembly's parts only
            else prim.GetReferences().AddReference(asset);
            ++st_.partReferences;
            if (Shared(n)) { prim.SetInstanceable(true); ++st_.instances; }
        }
        else if (Shared(n))
        {
            const SdfPath proto = Prototype(n);
            prim.GetReferences().AddInternalReference(proto);
            prim.SetInstanceable(true);
            ++st_.instances;
        }
        else EmitContent(n, path);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Verification: expected content (from PRC) vs. what the written USD stage contains
// ---------------------------------------------------------------------------------------------------------------------
struct Content
{
    long meshes = 0, pmi = 0, views = 0;
    long long triangles = 0;
    GfRange3d box;
};

static void Expect(const Analyzer& an, const Node* n, const GfMatrix4d& parent, bool hidden, Content& c, Stats& st)
{
    const GfMatrix4d w = n->local * parent;
    hidden = hidden || n->hidden;
    ++st.occurrences;
    if (n->hidden) ++st.hiddenNodes;
    c.pmi += (long)n->pmi.size();
    c.views += (long)n->views.size();
    for (const MeshUse& m : n->meshes)
    {
        const TessMesh& tm = an.tess[m.tess];
        ++c.meshes;
        c.triangles += (long long)tm.faceIds.size();
        if (hidden || m.hidden) continue;
        const GfMatrix4d mw = m.local * w;
        std::vector<char> used(tm.points.size(), 0);
        for (int i : tm.faceVertexIndices) used[i] = 1;
        for (size_t i = 0; i < tm.points.size(); ++i)
            if (used[i]) c.box.UnionWith(mw.Transform(GfVec3d(tm.points[i])));
    }
    for (const Node* k : n->kids) Expect(an, k, w, hidden, c, st);
}

struct UsdContent
{
    long meshes = 0, prims = 0, instancePrims = 0, pmi = 0, cameras = 0;
    long long triangles = 0;
    GfRange3d box;
};

static void DumpTree(const UsdStageRefPtr& stage)
{
    printf("USD tree:\n");
    for (UsdPrim p : UsdPrimRange::Stage(stage, UsdPrimAllPrimsPredicate))
    {
        const int depth = (int)p.GetPath().GetPathElementCount();
        std::string line = std::string(depth * 2, ' ') + p.GetName().GetString();
        if (!p.GetTypeName().IsEmpty()) line += " <" + p.GetTypeName().GetString() + ">";
        if (p.IsAbstract() && depth == 1) line += " (class)";
        if (p.HasAuthoredDisplayName()) line += " \"" + p.GetDisplayName() + "\"";
        for (const std::string& vs : p.GetVariantSets().GetNames())
            line += " {" + vs + "=" + p.GetVariantSets().GetVariantSelection(vs) + " of " +
                    TfStringJoin(p.GetVariantSet(vs).GetVariantNames(), ",") + "}";
        if (p.IsInstance())
        {
            SdfReferenceListOp refs;
            p.GetMetadata(SdfFieldKeys->References, &refs);
            for (const SdfReference& r : refs.GetAppliedItems()) line += " ->" + r.GetPrimPath().GetString();
        }
        TfToken tok;
        if (UsdGeomImageable(p) && UsdGeomImageable(p).GetVisibilityAttr().Get(&tok) && tok == UsdGeomTokens->invisible) line += " [invisible]";
        if (UsdGeomImageable(p) && UsdGeomImageable(p).GetPurposeAttr().Get(&tok) && tok != UsdGeomTokens->default_) line += " [" + tok.GetString() + "]";
        if (UsdGeomMesh m{p})
        {
            VtIntArray counts;
            m.GetFaceVertexCountsAttr().Get(&counts);
            line += TfStringPrintf(" | %zu triangles", counts.size());
            VtVec3fArray dc;
            UsdGeomPrimvar dcp = m.GetDisplayColorPrimvar();
            if (dcp && dcp.Get(&dc) && !dc.empty())
                line += dc.size() == 1 ? TfStringPrintf(" color(%.3g,%.3g,%.3g)", dc[0][0], dc[0][1], dc[0][2])
                                       : TfStringPrintf(" colors[%zu]", dc.size());
        }
        if (UsdGeomBasisCurves bc{p})
        {
            VtIntArray counts;
            bc.GetCurveVertexCountsAttr().Get(&counts);
            line += TfStringPrintf(" | %zu curves", counts.size());
        }
        std::string sv;
        if (p.GetAttribute(TfToken("hoops:pmi:type")).Get(&sv)) line += " pmi=" + sv;
        if (p.GetAttribute(TfToken("hoops:pmi:text")).Get(&sv))
        {
            std::string t = TfStringReplace(sv, "\n", " / ");
            if (t.size() > 40) t = t.substr(0, 40) + "...";
            line += " text=\"" + t + "\"";
        }
        UsdRelationship ar = p.GetRelationship(TfToken("hoops:view:annotations"));
        SdfPathVector targets;
        if (ar && ar.GetTargets(&targets)) line += TfStringPrintf(" annotations=%zu", targets.size());
        UsdRelationship br = p.GetRelationship(UsdShadeTokens->materialBinding);
        targets.clear();
        if (br && br.GetTargets(&targets) && !targets.empty()) line += " material=" + targets[0].GetName();
        int custom = 0;
        for (const UsdAttribute& a : p.GetAuthoredAttributes())
            if (a.IsCustom() && !TfStringStartsWith(a.GetName().GetString(), "hoops:")) ++custom;
        if (custom) line += TfStringPrintf(" {%d attributes}", custom);
        printf("%s\n", line.c_str());
    }
}

// Geometry (PMI excluded), PMI markups and cameras of the composed stage, through instances.
static UsdContent MeasureUsd(const UsdStageRefPtr& stage)
{
    UsdContent c;
    UsdGeomXformCache xc;
    UsdPrimRange range = UsdPrimRange::Stage(stage, UsdTraverseInstanceProxies(UsdPrimDefaultPredicate));
    for (auto it = range.begin(); it != range.end(); ++it)
    {
        const UsdPrim& p = *it;
        ++c.prims;
        if (p.IsInstance()) ++c.instancePrims;
        if (p.HasAttribute(TfToken("hoops:pmi:type"))) { ++c.pmi; it.PruneChildren(); continue; }
        if (p.IsA<UsdGeomCamera>()) { ++c.cameras; continue; }
        UsdGeomMesh mesh(p);
        if (!mesh) continue;
        ++c.meshes;
        VtIntArray counts, idx;
        VtVec3fArray pts;
        mesh.GetFaceVertexCountsAttr().Get(&counts);
        mesh.GetFaceVertexIndicesAttr().Get(&idx);
        mesh.GetPointsAttr().Get(&pts);
        for (int k : counts) c.triangles += std::max(0, k - 2);
        if (mesh.ComputeVisibility() == UsdGeomTokens->invisible) continue;
        const GfMatrix4d w = xc.GetLocalToWorldTransform(p);
        std::vector<char> used(pts.size(), 0);
        for (int i : idx) if (i >= 0 && i < (int)pts.size()) used[i] = 1;
        for (size_t i = 0; i < pts.size(); ++i)
            if (used[i]) c.box.UnionWith(w.Transform(GfVec3d(pts[i])));
    }
    return c;
}

// Compares one composed configuration of the written stage with what HOOPS Exchange loaded for it.
static bool CheckContent(const char* label, const UsdContent& got, const Content& exp, const Box& prcBox)
{
    Box usdBox;
    usdBox.ok = !got.box.IsEmpty();
    if (usdBox.ok) { usdBox.min = got.box.GetMin(); usdBox.max = got.box.GetMax(); }
    printf("%s: meshes %ld, triangles %lld, PMI %ld, cameras %ld, bbox %s (HOOPS Exchange: %s)\n", label, got.meshes,
           got.triangles, got.pmi, got.cameras, BoxText(usdBox).c_str(), BoxText(prcBox).c_str());
    bool ok = true;
    if (got.meshes != exp.meshes || got.triangles != exp.triangles)
    { printf("VERIFY FAILED: %s: USD has %ld meshes / %lld triangles, the model %ld / %lld\n", label, got.meshes, got.triangles, exp.meshes, exp.triangles); ok = false; }
    if (got.pmi != exp.pmi || got.cameras != exp.views)
    { printf("VERIFY FAILED: %s: USD has %ld PMI / %ld cameras, the model %ld / %ld\n", label, got.pmi, got.cameras, exp.pmi, exp.views); ok = false; }
    if (prcBox.ok)
    {
        double scale = 0;
        for (int i = 0; i < 3; ++i) scale = std::max({scale, std::fabs(prcBox.min[i]), std::fabs(prcBox.max[i])});
        const double tol = 1e-5 * std::max(1e-9, (prcBox.max - prcBox.min).GetLength()) + 1e-6 * scale;   // float points
        const double dev = BoxDeviation(usdBox, prcBox);
        if (!usdBox.ok || dev > tol)
        { printf("VERIFY FAILED: %s: USD world bounding box differs from HOOPS Exchange's by %.3g (tolerance %.3g)\n", label, dev, tol); ok = false; }
    }
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------
// Main flow
// ---------------------------------------------------------------------------------------------------------------------
struct Loaded                           // one HOOPS Exchange load, analysed
{
    std::string name;                   // configuration name ("" for a single-model file)
    std::unique_ptr<Analyzer> an;
    Node* node = nullptr;               // model root, or the configuration occurrence
    Box box;
    Content exp;
};

struct ConfigInfo { std::vector<std::string> names; size_t defaultIndex = 0; };

// Multi-entry files: the root container's children flagged as configurations (sample LoadMultiConfigCADFile).
static ConfigInfo FindConfigurations(const A3DAsmModelFile* mf, const A3DAsmProductOccurrence** container)
{
    ConfigInfo ci;
    *container = nullptr;
    A3DAsmModelFileData md = A3D_MAKE_DATA(A3DAsmModelFileData);
    if (A3DAsmModelFileGet(mf, &md) != A3D_SUCCESS) return ci;
    if (md.m_uiPOccurrencesSize == 1)
    {
        A3DAsmProductOccurrenceData rd = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
        if (A3DAsmProductOccurrenceGet(md.m_ppPOccurrences[0], &rd) == A3D_SUCCESS)
        {
            for (A3DUns32 i = 0; i < rd.m_uiPOccurrencesSize; ++i)
            {
                A3DAsmProductOccurrenceData cd = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
                if (A3DAsmProductOccurrenceGet(rd.m_ppPOccurrences[i], &cd) != A3D_SUCCESS) continue;
                if (cd.m_uiProductFlags & A3D_PRODUCT_FLAG_CONFIG)
                {
                    if (cd.m_uiProductFlags & A3D_PRODUCT_FLAG_DEFAULT) ci.defaultIndex = ci.names.size();
                    ci.names.push_back(Analyzer::EntityNameOf(rd.m_ppPOccurrences[i]));
                }
                A3DAsmProductOccurrenceGet(nullptr, &cd);
            }
            if (!ci.names.empty()) *container = md.m_ppPOccurrences[0];
            A3DAsmProductOccurrenceGet(nullptr, &rd);
        }
    }
    A3DAsmModelFileGet(nullptr, &md);
    return ci;
}

static A3DRWParamsLoadData LoadParams(const Options& opt)
{
    A3DRWParamsLoadData lp = A3D_MAKE_DATA(A3DRWParamsLoadData);
    lp.m_sGeneral.m_eReadGeomTessMode = kA3DReadGeomAndTess;
    lp.m_sGeneral.m_bReadSolids = true;
    lp.m_sGeneral.m_bReadSurfaces = true;
    lp.m_sGeneral.m_bReadWireframes = false;
    lp.m_sGeneral.m_bReadPmis = opt.pmi != "none";
    lp.m_sGeneral.m_bReadAttributes = true;
    // Hidden objects are read when PMI / views are wanted (A3DRWParamsGeneralData warning), then skipped if asked.
    lp.m_sGeneral.m_bReadHiddenObjects = !opt.skipHidden || opt.pmi != "none";
    lp.m_sGeneral.m_bReadConstructionAndReferences = false;
    lp.m_sGeneral.m_bReadFeature = false;
    lp.m_sTessellation.m_eTessellationLevelOfDetail = opt.lod;
    return lp;
}

static A3DStatus Load(const Options& opt, const std::string* entry, A3DAsmModelFile** mf)
{
    A3DRWParamsLoadData lp = LoadParams(opt);
    A3DUTF8Char* entries[1] = {nullptr};
    if (entry)
    {
        entries[0] = const_cast<A3DUTF8Char*>(entry->c_str());
        lp.m_sMultiEntries.m_bLoadDefault = false;
        lp.m_sMultiEntries.m_uiEntriesSize = 1;
        lp.m_sMultiEntries.m_ppcEntries = entries;
    }
    else lp.m_sMultiEntries.m_bLoadDefault = true;
    *mf = nullptr;
    return A3DAsmModelFileLoadFromFile(opt.input.c_str(), &lp, mf);
}

static bool LoadOk(A3DStatus st) { return st == A3D_SUCCESS || st == A3D_LOAD_MISSING_COMPONENTS || st == A3D_LOAD_MULTI_MODELS_CADFILE; }

// --layout parts: every referenced file resolves, the main file plus exactly the part files written are used, and each
// part file opens on its own (default prim, units, no composition errors).
// --layout layers: the same for the layers (only the geometry layer has a default prim).
static bool CheckPartFiles(const std::string& output, long partFiles, double mpu, bool layered = false)
{
    const char* what = layered ? "layers" : "part files";
    std::vector<SdfLayerRefPtr> layers;
    std::vector<std::string> assets, unresolved;
    if (!UsdUtilsComputeAllDependencies(SdfAssetPath(output), &layers, &assets, &unresolved))
    {
        printf("VERIFY FAILED: cannot compute the dependencies of %s\n", output.c_str());
        return false;
    }
    bool ok = true;
    for (const std::string& u : unresolved) { printf("  unresolved asset path: %s\n", u.c_str()); ok = false; }
    long standalone = 0;
    for (const SdfLayerRefPtr& l : layers)
    {
        if (l == layers.front()) continue;   // the main file (checked as the composed stage)
        UsdStageRefPtr ps = UsdStage::Open(l);
        const bool needsDefaultPrim = !layered || TfStringStartsWith(TfGetBaseName(l->GetRealPath()), "geometry");
        const bool good = ps && (!needsDefaultPrim || ps->GetDefaultPrim()) && ps->GetCompositionErrors().empty() &&
                          std::fabs(UsdGeomGetStageMetersPerUnit(ps) - mpu) <= 1e-12 * mpu && UsdGeomGetStageUpAxis(ps) == UsdGeomTokens->z;
        if (good) ++standalone;
        else { printf("  file does not open on its own: %s\n", l->GetIdentifier().c_str()); ok = false; }
    }
    printf("Layers: %zu (main file + %ld %s), unresolved asset paths %zu, %s opening on their own %ld\n", layers.size(),
           (long)layers.size() - 1, what, unresolved.size(), what, standalone);
    if ((long)layers.size() != partFiles + 1) { printf("VERIFY FAILED: %ld %s written but %zu layers used\n", partFiles, what, layers.size()); ok = false; }
    if (!ok) printf("VERIFY FAILED: %s\n", what);
    return ok;
}

static int Run(const Options& opt)
{
    std::error_code ec;
    const fs::path inPath = fs::u8path(opt.input), outPath = fs::u8path(opt.output);
    const std::string outExt = Lower(outPath.extension().u8string());
    if (outExt != ".usda" && outExt != ".usdc" && outExt != ".usd" && outExt != ".usdz")
    { printf("ERROR: unsupported output format '%s' (expected .usda, .usdc, .usd or .usdz)\n", outExt.c_str()); return kUnsupported; }
    if (!fs::exists(inPath, ec)) { printf("ERROR: input file not found: %s\n", opt.input.c_str()); return kInput; }
    if (fs::exists(outPath, ec) && !opt.force) { printf("ERROR: output exists: %s (use --force to overwrite)\n", opt.output.c_str()); return kUsage; }
    // --layout parts: the part files go to <output stem>_parts next to the output (inside the package for .usdz).
    // --layout layers: the layers go to <output stem>_layers the same way.
    const bool layersLayout = opt.layout == "layers";
    const bool partsLayout = opt.layout == "parts" || layersLayout;   // written with a side folder
    const std::string partsFolder = outPath.stem().u8string() + (layersLayout ? "_layers" : "_parts");
    const fs::path partsFinal = outPath.parent_path() / fs::u8path(partsFolder);
    if (partsLayout && fs::exists(partsFinal, ec) && Lower(outPath.extension().u8string()) != ".usdz")
    {
        if (!opt.force) { printf("ERROR: output exists: %s (use --force to overwrite)\n", partsFinal.u8string().c_str()); return kUsage; }
        // Only replace a folder this program wrote: USD part files and nothing else.
        for (const auto& e : fs::directory_iterator(partsFinal, ec))
        {
            const std::string x = Lower(e.path().extension().u8string());
            if (!e.is_regular_file() || (x != ".usda" && x != ".usdc"))
            {
                printf("ERROR: %s holds other files than USD part files (%s); not replaced\n", partsFinal.u8string().c_str(),
                       e.path().filename().u8string().c_str());
                return kUsage;
            }
        }
    }

    // ---- first load (default entry of a multi-entry file), B-rep tessellated
    A3DAsmModelFile* mf = nullptr;
    A3DStatus st = Load(opt, nullptr, &mf);
    if (!LoadOk(st) || !mf)
    {
        PrintStatus("A3DAsmModelFileLoadFromFile", st);
        if (mf) A3DAsmModelFileDelete(mf);
        return kInput;
    }
    if (st == A3D_LOAD_MISSING_COMPONENTS) printf("WARNING: some referenced files of the assembly were not found; they are not written\n");
    double unitMm = 1.0;
    if (A3DAsmModelFileGetUnit(mf, &unitMm) != A3D_SUCCESS || unitMm <= 0) unitMm = 1.0;
    const double mpu = MetersPerUnitFromPrcUnit(unitMm);
    printf("Input: %s (unit %g mm, tessellation %s)\n", inPath.filename().u8string().c_str(), unitMm, opt.lodName.c_str());

    const A3DAsmProductOccurrence* container = nullptr;
    ConfigInfo ci = FindConfigurations(mf, &container);
    std::vector<Loaded> loads;
    try
    {
        if (ci.names.empty())
        {
            if (!opt.configuration.empty()) { printf("ERROR: --configuration: the file has no configurations\n"); A3DAsmModelFileDelete(mf); return kInput; }
            Loaded l;
            l.an = std::make_unique<Analyzer>(opt, 0, unitMm);
            l.node = l.an->Build(mf, inPath.stem().u8string());
            l.box = PrcBoundingBox(mf);
            loads.push_back(std::move(l));
            A3DAsmModelFileDelete(mf);
        }
        else
        {
            printf("Configurations: %zu (%s), default '%s'\n", ci.names.size(), TfStringJoin(ci.names, ", ").c_str(), ci.names[ci.defaultIndex].c_str());
            std::vector<size_t> wanted;
            if (!opt.configuration.empty())
            {
                auto it = std::find(ci.names.begin(), ci.names.end(), opt.configuration);
                if (it == ci.names.end()) { printf("ERROR: --configuration: no configuration '%s'\n", opt.configuration.c_str()); A3DAsmModelFileDelete(mf); return kInput; }
                wanted.push_back((size_t)(it - ci.names.begin()));
            }
            else if (!opt.allConfigurations) wanted.push_back(ci.defaultIndex);
            else for (size_t k = 0; k < ci.names.size(); ++k) wanted.push_back(k);
            A3DAsmModelFileDelete(mf);
            mf = nullptr;
            for (size_t k : wanted)
            {
                // Each configuration is its own load; the others are loaded as structure only.
                A3DAsmModelFile* cmf = nullptr;
                A3DStatus cs = Load(opt, &ci.names[k], &cmf);
                if (!LoadOk(cs) || !cmf) { PrintStatus(("loading configuration '" + ci.names[k] + "'").c_str(), cs); if (cmf) A3DAsmModelFileDelete(cmf); return kInput; }
                const A3DAsmProductOccurrence* ctr = nullptr;
                FindConfigurations(cmf, &ctr);
                const A3DAsmProductOccurrence* cfg = nullptr;
                A3DAsmProductOccurrenceData rd = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
                if (ctr && A3DAsmProductOccurrenceGet(ctr, &rd) == A3D_SUCCESS)
                {
                    for (A3DUns32 i = 0; i < rd.m_uiPOccurrencesSize && !cfg; ++i)
                        if (Analyzer::EntityNameOf(rd.m_ppPOccurrences[i]) == ci.names[k]) cfg = rd.m_ppPOccurrences[i];
                    A3DAsmProductOccurrenceGet(nullptr, &rd);
                }
                if (!cfg) { printf("ERROR: configuration '%s' not found after loading it\n", ci.names[k].c_str()); A3DAsmModelFileDelete(cmf); return kInput; }
                // Each load has its own model-file unit: build in it, then scale into the stage unit (first load).
                double cu = unitMm;
                if (A3DAsmModelFileGetUnit(cmf, &cu) != A3D_SUCCESS || cu <= 0) cu = unitMm;
                Loaded l;
                l.name = ci.names[k];
                l.an = std::make_unique<Analyzer>(opt, (int)k + 1, cu);
                l.node = l.an->BuildConfiguration(ctr, cfg);
                l.box = PrcBoundingBox(cmf);
                const double f = cu / unitMm;
                if (l.node && std::fabs(f - 1.0) > 1e-12)
                {
                    GfMatrix4d S(1.0);
                    S.SetScale(f);
                    l.node->local = l.node->local * S;
                    l.box.min *= f;
                    l.box.max *= f;
                }
                A3DAsmModelFileDelete(cmf);
                if (l.node) loads.push_back(std::move(l));
            }
        }
    }
    catch (A3DStatus)
    {
        if (mf) A3DAsmModelFileDelete(mf);
        return kInput;
    }

    long totalMeshes = 0;
    Stats total;
    for (Loaded& l : loads)
    {
        Expect(*l.an, l.node, GfMatrix4d(1.0), false, l.exp, l.an->st);
        totalMeshes += l.exp.meshes;
        total.occurrences += l.an->st.occurrences - (l.name.empty() ? 1 : 0);
        total.removed += l.an->st.removed;
        total.badFaces += l.an->st.badFaces;
        total.skippedFaces += l.an->st.skippedFaces;
        total.textFailures += l.an->st.textFailures;
        total.viewsSkipped += l.an->st.viewsSkipped;
    }
    if (totalMeshes == 0) { printf("ERROR: the model contains no tessellated geometry to write\n"); return kInput; }
    const bool multi = !loads.front().name.empty() && loads.size() > 1;

    // ---- write the stage to a temporary file (a .usdc inside a folder for .usdz), then move it into place
    const bool usdz = outExt == ".usdz";
    fs::path stagingDir, stagePath;
    if (partsLayout && !usdz)
    {
        // The main file and its part folder are written side by side in a staging folder (same relative layout as
        // the output, so references resolve while writing), then moved into place.
        stagingDir = outPath;
        stagingDir += ".partial";
        fs::remove_all(stagingDir, ec);
        fs::create_directories(stagingDir, ec);
        stagePath = stagingDir / outPath.filename();
    }
    else if (usdz)
    {
        stagingDir = outPath;
        stagingDir += ".partial";
        fs::remove_all(stagingDir, ec);
        fs::create_directories(stagingDir, ec);
        stagePath = stagingDir / (outPath.stem().u8string() + ".usdc");
    }
    else
    {
        stagePath = outPath;
        stagePath.replace_filename(outPath.stem().u8string() + ".partial" + outPath.extension().u8string());
        fs::remove(stagePath, ec);
    }
    fs::path partsStaging;
    if (partsLayout)
    {
        partsStaging = stagingDir / fs::u8path(partsFolder);
        fs::create_directories(partsStaging, ec);
        if (ec) { printf("ERROR: cannot create %s: %s\n", partsStaging.u8string().c_str(), ec.message().c_str()); return kExport; }
    }
    auto cleanup = [&]() {
        std::error_code e2;
        if (!stagingDir.empty()) fs::remove_all(stagingDir, e2);
        else fs::remove(stagePath, e2);
    };
    Stats es;   // what the emitter wrote
    std::vector<std::string> writtenLayers;   // --layout layers
    std::vector<std::string> variantNames;
    SdfPath rootPath;
    {
        TfErrorMark mark;
        UsdStageRefPtr stage = UsdStage::CreateNew(stagePath.u8string());
        if (!stage)
        {
            for (auto it = mark.GetBegin(); it != mark.GetEnd(); ++it) printf("  %s\n", it->GetCommentary().c_str());
            mark.Clear();
            printf("ERROR: cannot create USD stage %s\n", stagePath.u8string().c_str());
            return kExport;
        }
        UsdEmitter em(opt, stage, es);
        // Part files: usda next to a .usda output, binary usdc otherwise.
        try
        {
        const std::string sideExt = outExt == ".usda" ? ".usda" : ".usdc";
        if (layersLayout) em.SetLayers(partsStaging, partsFolder, sideExt);
        else if (partsLayout) em.SetPartFiles(partsStaging, partsFolder, sideExt);
        em.Begin(inPath.stem().u8string(), mpu, inPath.filename().u8string());
        rootPath = em.RootPath();
        if (loads.front().name.empty()) em.EmitModel(*loads.front().an, loads.front().node);
        else if (!multi)
        {
            // One configuration requested: written as the model itself (configuration occurrence under the root).
            Node wrapper;
            wrapper.kids.push_back(loads.front().node);
            em.EmitModel(*loads.front().an, &wrapper);
        }
        else
        {
            std::vector<std::pair<const Analyzer*, const Node*>> cfgs;
            std::vector<std::string> names;
            size_t def = 0;
            for (size_t k = 0; k < loads.size(); ++k)
            {
                cfgs.push_back({loads[k].an.get(), loads[k].node});
                names.push_back(loads[k].name);
                if (loads[k].name == ci.names[ci.defaultIndex]) def = k;
            }
            variantNames = em.EmitConfigurations(cfgs, names, def);
        }
        }
        catch (const std::exception& e)
        {
            printf("ERROR: %s\n", e.what());
            cleanup();
            return kExport;
        }
        if (!em.SaveLayers() || !stage->GetRootLayer()->Save() || !em.SaveParts())
        {
            printf("ERROR: cannot save %s\n", stagePath.u8string().c_str());
            cleanup();
            return kExport;
        }
        writtenLayers = em.WrittenLayers();
        if (!mark.IsClean())
        {
            for (auto it = mark.GetBegin(); it != mark.GetEnd(); ++it) printf("WARNING: USD: %s\n", it->GetCommentary().c_str());
            mark.Clear();
        }
    }
    if (usdz)
    {
        fs::path tmpZip = outPath;
        tmpZip += ".partial.usdz";
        fs::remove(tmpZip, ec);
        if (!UsdUtilsCreateNewUsdzPackage(SdfAssetPath(stagePath.u8string()), tmpZip.u8string()))
        { printf("ERROR: cannot create the usdz package\n"); fs::remove_all(stagingDir, ec); return kExport; }
        fs::remove_all(stagingDir, ec);
        stagePath = tmpZip;
    }
    if (partsLayout && !usdz && es.partFiles + es.layerFiles > 0)
    {
        fs::remove_all(partsFinal, ec);   // checked above: only part files of an earlier run
        fs::rename(partsStaging, partsFinal, ec);
        if (ec) { printf("ERROR: cannot move %s to %s: %s\n", partsStaging.u8string().c_str(), partsFinal.u8string().c_str(), ec.message().c_str()); cleanup(); return kExport; }
    }
    if (fs::exists(outPath, ec)) fs::remove(outPath, ec);
    fs::rename(stagePath, outPath, ec);
    if (ec) { printf("ERROR: cannot move %s to %s: %s\n", stagePath.u8string().c_str(), opt.output.c_str(), ec.message().c_str()); cleanup(); return kExport; }
    if (partsLayout && !usdz) fs::remove_all(stagingDir, ec);

    long long tris = 0;
    long pmi = 0, views = 0;
    for (const Loaded& l : loads) { tris += l.exp.triangles; pmi += l.exp.pmi; views += l.exp.views; }
    printf("Written: occurrences %ld, meshes %ld (hidden %ld), triangles %lld, prototypes %ld, instances %ld, materials %ld, attributes %ld\n",
           total.occurrences, totalMeshes, es.hiddenMeshes, tris, es.prototypes, es.instances, es.materials, es.attributes);
    if (opt.pmi != "none")
        printf("PMI: markups %ld (curves %ld), views %ld (cameras %ld, view variants %ld), purpose %s\n", pmi, es.pmiCurves, views,
               es.cameras, es.viewVariants, opt.pmi.c_str());
    if (total.viewsSkipped) printf("Views of nested parts without annotations not written: %ld (--all-views writes them)\n", total.viewsSkipped);
    if (multi) printf("Configurations written as variants: %s\n", TfStringJoin(variantNames, ", ").c_str());
    if (layersLayout)
        printf("Layout: layers, %ld layers in %s/ (%s)\n", es.layerFiles, partsFolder.c_str(), TfStringJoin(writtenLayers, ", ").c_str());
    else if (partsLayout)
        printf("Layout: parts, %ld part files in %s/ (referenced %ld times, instanceable %ld, payloads %ld)\n", es.partFiles, partsFolder.c_str(),
               es.partReferences, es.instances, es.payloads);
    if (total.removed) printf("Not written: %ld removed%s objects\n", total.removed, opt.skipHidden ? " or hidden" : "");
    if (total.textFailures) printf("WARNING: %ld PMI texts could not be tessellated (font not available); their strings are kept in hoops:pmi:text\n", total.textFailures);
    if (total.badFaces || total.skippedFaces) printf("WARNING: %lld triangles with invalid indices and %lld faces with an unsupported layout were skipped\n", total.badFaces, total.skippedFaces);
    printf("Wrote %s\n", opt.output.c_str());

    // ---- reopen the written file with OpenUSD and compare each configuration with its HOOPS Exchange load
    UsdStageRefPtr back;
    {
        TfErrorMark mark;
        back = UsdStage::Open(opt.output);
        if (!back)
        {
            for (auto it = mark.GetBegin(); it != mark.GetEnd(); ++it) printf("  %s\n", it->GetCommentary().c_str());
            mark.Clear();
            printf("VERIFY FAILED: OpenUSD cannot open the written file\n");
            return kVerify;
        }
    }
    if (opt.dumpTree) DumpTree(back);
    printf("Re-opened USD: metersPerUnit %g, upAxis %s, prototypes %zu\n", UsdGeomGetStageMetersPerUnit(back),
           UsdGeomGetStageUpAxis(back).GetText(), back->GetPrototypes().size());
    int rc = kOk;
    if (std::fabs(UsdGeomGetStageMetersPerUnit(back) - mpu) > 1e-12 * mpu) { printf("VERIFY FAILED: metersPerUnit\n"); rc = kVerify; }
    const PcpErrorVector compErrors = back->GetCompositionErrors();
    if (!compErrors.empty())
    {
        for (const PcpErrorBasePtr& e : compErrors) printf("  %s\n", e->ToString().c_str());
        printf("VERIFY FAILED: %zu composition errors\n", compErrors.size());
        rc = kVerify;
    }
    if (partsLayout && !CheckPartFiles(opt.output, es.partFiles + es.layerFiles, mpu, layersLayout)) rc = kVerify;
    if (!multi)
    {
        const UsdContent got = MeasureUsd(back);
        printf("Re-opened USD: prims %ld, instances %ld\n", got.prims, got.instancePrims);
        if (!CheckContent("Model", got, loads.front().exp, loads.front().box)) rc = kVerify;
    }
    else
    {
        UsdPrim root = back->GetPrimAtPath(rootPath);
        for (size_t k = 0; k < loads.size(); ++k)
        {
            {
                UsdEditContext session(back, back->GetSessionLayer());
                root.GetVariantSet("configuration").SetVariantSelection(variantNames[k]);
            }
            const UsdContent got = MeasureUsd(back);
            if (!CheckContent(("Configuration '" + loads[k].name + "'").c_str(), got, loads[k].exp, loads[k].box)) rc = kVerify;
        }
    }
    // --layout layers: muting the PMI or views layer removes exactly that data (current configuration).
    if (layersLayout)
    {
        const Loaded* sel = &loads.front();   // with configurations: the first one, selected below
        if (multi)
        {
            UsdEditContext session(back, back->GetSessionLayer());
            back->GetPrimAtPath(rootPath).GetVariantSet("configuration").SetVariantSelection(variantNames[0]);
        }
        for (const std::string& f : writtenLayers)
        {
            const bool pmiLayer = TfStringStartsWith(f, "pmi"), viewsLayer = TfStringStartsWith(f, "views");
            if (!pmiLayer && !viewsLayer) continue;
            // The layer's identifier as the stage knows it (a package path such as "x.usdz[x_layers/pmi.usdc]" in a .usdz).
            std::string id;
            for (const SdfLayerHandle& l : back->GetLayerStack(false))
            {
                const std::string li = l->GetIdentifier();
                if (TfStringEndsWith(li, partsFolder + "/" + f) || TfStringEndsWith(li, partsFolder + "/" + f + "]")) id = li;
            }
            if (id.empty()) { printf("VERIFY FAILED: layer %s not in the layer stack\n", f.c_str()); rc = kVerify; continue; }
            UsdContent m;
            if (!usdz)
            {
                back->MuteLayer(id);
                m = MeasureUsd(back);
                back->UnmuteLayer(id);
            }
            else
            {
                // Muting does not apply to a layer inside a .usdz package: recompose the stage without it instead.
                SdfLayerRefPtr root = SdfLayer::CreateAnonymous("without.usda");
                root->TransferContent(back->GetRootLayer());
                std::vector<std::string> sub;
                for (const SdfLayerHandle& l : back->GetLayerStack(false))
                    if (l != back->GetRootLayer() && l->GetIdentifier() != id) sub.push_back(l->GetIdentifier());
                root->SetSubLayerPaths(sub);
                UsdStageRefPtr without = UsdStage::Open(root);
                if (multi)
                {
                    UsdEditContext session(without, without->GetSessionLayer());
                    without->GetPrimAtPath(rootPath).GetVariantSet("configuration").SetVariantSelection(variantNames[0]);
                }
                m = MeasureUsd(without);
            }
            printf("%s %s: meshes %ld, triangles %lld, PMI %ld, cameras %ld\n", usdz ? "Without" : "Muted", f.c_str(), m.meshes, m.triangles, m.pmi, m.cameras);
            const bool ok = m.meshes == sel->exp.meshes && m.triangles == sel->exp.triangles &&
                            (pmiLayer ? m.pmi == 0 : m.cameras == 0 && m.pmi == sel->exp.pmi);
            if (!ok) { printf("VERIFY FAILED: muting %s\n", f.c_str()); rc = kVerify; }
        }
    }
    // --payloads: opened without loading, the stage holds the assembly but none of the payload geometry.
    if (opt.payloads && es.payloads > 0)
    {
        UsdStageRefPtr unloaded = UsdStage::Open(opt.output, UsdStage::LoadNone);
        long payloadPrims = 0;
        for (UsdPrim p : unloaded->Traverse(UsdTraverseInstanceProxies(UsdPrimAllPrimsPredicate)))
            if (p.HasAuthoredPayloads()) ++payloadPrims;
        const UsdContent u = MeasureUsd(unloaded);
        unloaded->LoadAndUnload(SdfPathSet{SdfPath::AbsoluteRootPath()}, SdfPathSet{});
        const UsdContent l = MeasureUsd(unloaded);
        printf("Payloads: %ld prims; opened unloaded: meshes %ld; after Load(): meshes %ld, triangles %lld\n", payloadPrims, u.meshes, l.meshes, l.triangles);
        if (payloadPrims == 0 || u.meshes >= l.meshes) { printf("VERIFY FAILED: payloads\n"); rc = kVerify; }
    }
    if (rc == kOk) printf("Check: OK (meshes, triangles, PMI, cameras, units and world bounding box match the HOOPS Exchange model)\n");
    return rc;
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
        else if (a == "--dump-tree") opt.dumpTree = true;
        else if (a == "--skip-hidden") opt.skipHidden = true;
        else if (a == "--no-instancing") opt.instancing = false;
        else if (a == "--debug-pmi") g_debugPmi = true;
        else if (a == "--all-views") opt.allViews = true;
        else if (a == "--default-configuration") opt.allConfigurations = false;
        else if (a == "--configuration")
        {
            if (i + 1 >= args.size()) { printf("ERROR: --configuration expects a name\n"); return kUsage; }
            opt.configuration = args[++i];
        }
        else if (a == "--pmi")
        {
            opt.pmi = i + 1 < args.size() ? Lower(args[++i]) : "";
            if (opt.pmi != "guide" && opt.pmi != "default" && opt.pmi != "none") { printf("ERROR: --pmi expects guide, default or none\n"); return kUsage; }
        }
        else if (a == "--layout")
        {
            opt.layout = i + 1 < args.size() ? Lower(args[++i]) : "";
            if (opt.layout != "monolithic" && opt.layout != "parts" && opt.layout != "layers")
            { printf("ERROR: --layout expects monolithic, parts or layers\n"); return kUsage; }
        }
        else if (a == "--payloads") opt.payloads = true;
        else if (a == "--lod")
        {
            const std::string v = i + 1 < args.size() ? Lower(args[++i]) : "";
            static const std::map<std::string, A3DETessellationLevelOfDetail> lods = {
                {"extralow", kA3DTessLODExtraLow}, {"low", kA3DTessLODLow}, {"medium", kA3DTessLODMedium},
                {"high", kA3DTessLODHigh}, {"extrahigh", kA3DTessLODExtraHigh}};
            auto it = lods.find(v);
            if (it == lods.end()) { printf("ERROR: --lod expects extralow, low, medium, high or extrahigh\n"); return kUsage; }
            opt.lod = it->second;
            opt.lodName = v;
        }
        else if (a.size() > 2 && a.compare(0, 2, "--") == 0) { printf("ERROR: unknown option %s\n", a.c_str()); PrintUsage(); return kUsage; }
        else pos.push_back(a);
    }
    if (pos.size() != 2) { PrintUsage(); return kUsage; }
    if (opt.payloads && opt.layout != "parts") { printf("ERROR: --payloads needs --layout parts\n"); return kUsage; }
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
