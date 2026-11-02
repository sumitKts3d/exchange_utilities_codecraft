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

// qif_document.hpp - QIF_Writer: the QIF 3.0 document built from a HOOPS Exchange model file. Included once by
// QIF_Writer.cpp after qif_pmi.hpp.
//
// Geometry and topology: each A3DRiBrepModel instance is adapted to NURBS (A3DCopyAndAdaptBrepModel, same topology)
// and written in model space. PRC conventions -> QIF conventions (as read by ../QIF_Reader):
//   * NURBS surface control points: PRC v fastest -> QIF u fastest;
//   * loops: a loop reversed with its surface is written reversed (co-edges in reverse order, orientations flipped),
//     so every QIF loop is counter-clockwise in UV for an outer loop; the outer loop is written first;
//   * co-edge p-curves follow the co-edge (reversed when PRC's UV curve runs against the loop);
//   * a face reversed in its shell, or placed by a mirroring transformation, is written turned="true";
//   * co-edges of the same edge (PRC neighbour ring) share one QIF Edge; EdgeOriented turned compares directions.
#pragma once

struct SourceProps { bool ok = false; double area = 0, volume = 0; };

static SourceProps SourceProperties(A3DAsmModelFile* mf)
{
    SourceProps sp;
    double unit = 1.0;
    if (A3DAsmModelFileGetUnit(mf, &unit) != A3D_SUCCESS || unit <= 0) unit = 1.0;
    A3DPhysicalPropertiesData pp = A3D_MAKE_DATA(A3DPhysicalPropertiesData);
    pp.m_bUseGeometryOnRiBRep = true;
    if (A3DComputeModelFilePhysicalProperties(mf, &pp) == A3D_SUCCESS)
    {
        sp.ok = true;
        sp.area = pp.m_dSurface * unit * unit;
        sp.volume = pp.m_bVolumeComputed ? pp.m_dVolume * unit * unit * unit : 0.0;
    }
    return sp;
}

struct WriterStats
{
    long sourceBreps = 0, sourceFaces = 0, bodies = 0, faces = 0, edges = 0, vertices = 0, loops = 0, surfaces = 0;
    long curves3d = 0, pcurves = 0, splitBreps = 0, failedBreps = 0, meshesSkipped = 0, hiddenSkipped = 0;
    long coedgesNoUV = 0, edgesNo3d = 0, surfacesFailed = 0;
    long pmiDisplays = 0, characteristics = 0, gdt = 0, dimensions = 0, datums = 0, datumTargets = 0, notes = 0;
    long features = 0, linkedPmi = 0, unresolvedLinks = 0, textFailures = 0, extraRows = 0, emptyDrawings = 0;
    long savedViews = 0, cameras = 0, annotationViews = 0;
    double worstGap = 0, worstPlanarity = 0;
    long checkedCoEdges = 0;
    std::vector<std::pair<double, double>> gaps;   // (p-curve end to vertex gap, edge tolerance) above 1e-9
    long recomputedUV = 0, recomputed3D = 0;
};

class QifDocument
{
public:
    explicit QifDocument(const Options& o) : opt_(o) {}
    std::string uuid, sourceName;
    double unitMm = 1.0, modelSize = 1.0;
    WriterStats st;

    bool Build(const A3DAsmModelFile* mf)
    {
        if (A3DAsmModelFileGetUnit(mf, &unitMm) != A3D_SUCCESS || unitMm <= 0) unitMm = 1.0;
        A3DBoundingBoxData bb = A3D_MAKE_DATA(A3DBoundingBoxData);
        if (A3DMiscGetBoundingBox(mf, &bb) == A3D_SUCCESS)
        {
            const double d = (V(bb.m_sMax) - V(bb.m_sMin)).norm();
            if (d > 0 && std::isfinite(d)) modelSize = d;
        }
        A3DAsmModelFileData md = A3D_MAKE_DATA(A3DAsmModelFileData);
        if (A3DAsmModelFileGet(mf, &md) != A3D_SUCCESS) return false;
        A3DMiscCascadedAttributes* root = nullptr;
        A3DMiscCascadedAttributesCreate(&root);
        for (A3DUns32 i = 0; i < md.m_uiPOccurrencesSize; ++i)
        {
            if (partName_.empty()) partName_ = OccurrenceName(md.m_ppPOccurrences[i]);
            Occurrence(md.m_ppPOccurrences[i], Mat4(), root, true, 0);
        }
        A3DMiscCascadedAttributesDelete(root);
        A3DAsmModelFileGet(nullptr, &md);
        if (haveBox_)
        {
            const double d = (boxMax_ - boxMin_).norm();
            if (d > 0 && std::isfinite(d)) modelSize = d;
        }
        if (partName_.empty()) partName_ = fs::u8path(sourceName).stem().u8string();
        if (opt_.pmi) BuildPmi();
        return true;
    }

    std::string Xml()
    {
        E root("QIFDocument");
        root.a("xmlns", "http://qifstandards.org/xsd/qif3").a("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance");
        root.a("versionQIF", "3.0.0");
        root.c("QPId", uuid);
        {
            char ts[32];
            std::time_t now = std::time(nullptr);
            std::tm tmv{};
#ifdef _WIN32
            gmtime_s(&tmv, &now);
#else
            gmtime_r(&now, &tmv);
#endif
            std::strftime(ts, sizeof ts, "%Y-%m-%dT%H:%M:%SZ", &tmv);
            root.c(E("Version").c("TimeCreated", ts));
        }
        root.c(E("Header").c(E("Application").c("Name", "QIF_Writer (HOOPS Exchange " + std::to_string(A3D_DLL_MAJORVERSION) + "." +
                                                             std::to_string(A3D_DLL_MINORVERSION) + ")")
                                                 .c("Organization", "CodeCraft"))
                              .c("Description", "Converted from " + sourceName));
        root.c(FileUnits());
        if (!datumDefs_.empty()) root.c(Set("DatumDefinitions", datumDefs_));
        if (!datumTargets_.empty()) root.c(Set("DatumTargetDefinitions", datumTargets_));
        if (!drfs_.empty()) root.c(Set("DatumReferenceFrames", drfs_));
        root.c(Product());
        if (!featureNominals_.empty())
        {
            E f("Features");
            f.c(Set("FeatureDefinitions", featureDefs_));
            f.c(Set("FeatureNominals", featureNominals_));
            root.c(std::move(f));
        }
        if (!charNominals_.empty())
        {
            E c("Characteristics");
            c.c(Set("CharacteristicDefinitions", charDefs_));
            c.c(Set("CharacteristicNominals", charNominals_));
            root.c(std::move(c));
        }
        root.a("idMax", lastId_);
        std::string out = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
        out.reserve(1 << 20);
        root.write(out, 0);
        return out;
    }

    void PrintSummary() const
    {
        printf("Written model: bodies %ld, faces %ld, edges %ld, vertices %ld, surfaces %ld, edge curves %ld, p-curves %ld\n",
               st.bodies, st.faces, st.edges, st.vertices, st.surfaces, st.curves3d, st.pcurves);
        if (opt_.pmi)
            printf("Written PMI: displays %ld (%ld linked), GD&T %ld, dimensions %ld, datums %ld, datum targets %ld, notes %ld, "
                   "features %ld, saved views %ld, annotation planes %ld\n",
                   st.pmiDisplays, st.linkedPmi, st.gdt, st.dimensions, st.datums, st.datumTargets, st.notes, st.features, st.savedViews,
                   st.annotationViews);
        if (st.recomputedUV || st.recomputed3D) printf("NOTE: p-curves computed for %ld B-rep(s), 3D edge curves for %ld (missing in the source)\n", st.recomputedUV, st.recomputed3D);
        if (st.splitBreps) printf("NOTE: %ld B-rep(s) needed periodic faces split at their seam (PMI links to them are by body)\n", st.splitBreps);
        if (st.failedBreps) printf("WARNING: %ld B-rep(s) could not be adapted to NURBS and were not written\n", st.failedBreps);
        if (st.meshesSkipped) printf("NOTE: %ld tessellated (mesh) bodies not written: QIF holds exact B-rep\n", st.meshesSkipped);
        if (st.hiddenSkipped) printf("NOTE: %ld hidden bodies skipped (--skip-hidden)\n", st.hiddenSkipped);
        if (st.coedgesNoUV) printf("WARNING: %ld co-edges without a p-curve\n", st.coedgesNoUV);
        if (st.edgesNo3d) printf("NOTE: %ld edges without a 3D curve (degenerate)\n", st.edgesNo3d);
        if (st.surfacesFailed) printf("WARNING: %ld surfaces could not be written as NURBS (their faces are missing)\n", st.surfacesFailed);
        if (st.unresolvedLinks) printf("NOTE: %ld PMI links to topology could not be resolved\n", st.unresolvedLinks);
        if (st.extraRows) printf("NOTE: %ld additional feature control frame rows (composite tolerances) kept in the drawing only\n", st.extraRows);
        if (st.textFailures) printf("NOTE: %ld PMI texts without a drawable font (text kept in the semantic / note data)\n", st.textFailures);
        if (st.emptyDrawings) printf("NOTE: %ld PMI without drawing got a placeholder mark\n", st.emptyDrawings);
        if (st.worstPlanarity > 1e-6 * modelSize) printf("NOTE: non-planar PMI drawing projected on its plane (worst %.3g)\n", st.worstPlanarity);
    }

private:
    const Options& opt_;
    long lastId_ = 0;
    bool haveBox_ = false;
    Vec3 boxMin_, boxMax_;
    void Grow(const Vec3& p)
    {
        if (!haveBox_) { boxMin_ = boxMax_ = p; haveBox_ = true; return; }
        boxMin_ = Vec3(std::min(boxMin_.x, p.x), std::min(boxMin_.y, p.y), std::min(boxMin_.z, p.z));
        boxMax_ = Vec3(std::max(boxMax_.x, p.x), std::max(boxMax_.y, p.y), std::max(boxMax_.z, p.z));
    }
    std::string partName_;
    long id() { return ++lastId_; }

    // geometry / topology sets
    std::vector<E> points_, curves13_, curves12_, surfaces_, vertices_, edges_, loops_, faces_, shells_, bodies_;
    std::vector<long> bodyIds_;

    struct BrepInstance
    {
        const A3DAsmProductOccurrence* po = nullptr;
        bool split = false;
        // keys as in A3DMiscReferenceOnTopology: the face index counts the faces of the whole B-rep
        std::map<std::vector<A3DUns32>, long> faces;   // (face) -> QIF face
        std::map<std::vector<A3DUns32>, long> edges;   // (face, loop, co-edge) -> QIF edge
        std::vector<long> allFaces;
    };
    std::map<const A3DTopoBrepData*, std::vector<BrepInstance>> breps_;

    // PMI
    struct OwnedPmi { PmiItem item; Mat4 W; const A3DAsmProductOccurrence* owner = nullptr; };
    std::vector<OwnedPmi> pmi_;
    std::set<std::pair<const void*, const void*>> pmiSeen_;   // (annotation item, owner occurrence)
    struct ViewRec
    {
        std::string name;
        bool isDefault = false, ortho = true;
        Vec3 loc, target, up;
        double xfov = 0, yfov = 0, znear = 0, zfar = 0, aspect = 1;
        std::vector<const void*> annots;
    };
    std::vector<ViewRec> views_;

    // PMI output
    std::vector<E> datumDefs_, datumTargets_, drfs_, featureDefs_, featureNominals_, charDefs_, charNominals_, notes_;
    std::vector<E> pmiDisplays_, annotationViews_, cameras_, savedViews_;
    std::vector<long> featureIds_, charIds_, noteIds_, datumIds_, drfIds_, viewIds_;
    std::map<std::string, long> datumByLabel_, drfByKey_;
    std::map<std::string, long> annotationViewByKey_;
    std::map<const void*, std::vector<long>> refsByAnnotation_;
    long featureDefId_ = 0;

    static E Set(const char* name, std::vector<E>& items)
    {
        E s(name);
        s.a("n", items.size());
        for (E& e : items) s.c(std::move(e));
        items.clear();
        return s;
    }

    E FileUnits() const
    {
        std::string unitName = "unit";
        if (std::fabs(unitMm - 1.0) < 1e-12) unitName = "mm";
        else if (std::fabs(unitMm - 25.4) < 1e-9) unitName = "inch";
        else if (std::fabs(unitMm - 10.0) < 1e-12) unitName = "cm";
        else if (std::fabs(unitMm - 1000.0) < 1e-9) unitName = "meter";
        else if (std::fabs(unitMm - 304.8) < 1e-9) unitName = "ft";
        E pu("PrimaryUnits");
        pu.c(E("PMIAngularUnit").c("SIUnitName", "radian").c("UnitName", "degree")
                 .c(E("UnitConversion").c(E("Factor").raw(qo::num(qo::kPi / 180.0)))));
        pu.c(E("LinearUnit").c("SIUnitName", "meter").c("UnitName", unitName)
                 .c(E("UnitConversion").c(E("Factor").raw(qo::num(unitMm / 1000.0)))));
        return E("FileUnits").c(std::move(pu));
    }

    E Product()
    {
        E p("Product");
        p.c(E("Header").c(E("File").c("Name", sourceName)));
        E gs("GeometrySet");
        if (!points_.empty()) gs.c(Set("PointSet", points_));
        if (!curves12_.empty()) gs.c(Set("Curve12Set", curves12_));
        if (!curves13_.empty()) gs.c(Set("Curve13Set", curves13_));
        if (!surfaces_.empty()) gs.c(Set("SurfaceSet", surfaces_));
        p.c(std::move(gs));
        E ts("TopologySet");
        ts.c(Set("VertexSet", vertices_));
        ts.c(Set("EdgeSet", edges_));
        ts.c(Set("LoopSet", loops_));
        ts.c(Set("FaceSet", faces_));
        ts.c(Set("ShellSet", shells_));
        ts.c(Set("BodySet", bodies_));
        p.c(std::move(ts));
        if (!notes_.empty()) p.c(Set("NoteSet", notes_));
        if (!cameras_.empty() || !savedViews_.empty() || !annotationViews_.empty())
        {
            E vs("ViewSet");
            if (!cameras_.empty()) vs.c(Set("CameraSet", cameras_));
            if (!savedViews_.empty()) vs.c(Set("SavedViewSet", savedViews_));
            if (!annotationViews_.empty()) vs.c(Set("AnnotationViewSet", annotationViews_));
            p.c(std::move(vs));
        }
        if (!pmiDisplays_.empty()) p.c(E("VisualizationSet").c(Set("PMIDisplaySet", pmiDisplays_)));
        const long partId = id();
        E part("Part");
        part.a("id", partId).a("label", partName_);
        if (!featureIds_.empty()) part.ids("FeatureNominalIds", featureIds_);
        if (!charIds_.empty()) part.ids("CharacteristicNominalIds", charIds_);
        if (!noteIds_.empty()) part.ids("NoteIds", noteIds_);
        if (!datumIds_.empty()) part.ids("DatumDefinitionIds", datumIds_);
        if (!drfIds_.empty()) part.ids("DatumReferenceFrameIds", drfIds_);
        part.ids("BodyIds", bodyIds_);
        if (!viewIds_.empty()) part.ids("ViewIds", viewIds_);
        E partSet("PartSet");
        partSet.a("n", 1).c(std::move(part));
        p.c(std::move(partSet));
        p.c(E("RootPart").c(E("Id").raw(std::to_string(partId))));
        return p;
    }

    // ---- structure ----------------------------------------------------------------------------------------------
    struct Cascade
    {
        A3DMiscCascadedAttributes* attr = nullptr;
        bool shown = true, removed = false;
        Rgb color;
        Cascade(const A3DEntity* e, const A3DMiscCascadedAttributes* father)
        {
            A3DMiscCascadedAttributesCreate(&attr);
            A3DMiscCascadedAttributesPush(attr, e, father);
            A3DMiscCascadedAttributesData cd = A3D_MAKE_DATA(A3DMiscCascadedAttributesData);
            if (A3DMiscCascadedAttributesGet(attr, &cd) == A3D_SUCCESS)
            {
                shown = cd.m_bShow;
                removed = cd.m_bRemoved;
                color = ColorOfStyle(cd.m_sStyle);
                A3DMiscCascadedAttributesGet(nullptr, &cd);
            }
        }
        ~Cascade() { if (attr) A3DMiscCascadedAttributesDelete(attr); }
        Cascade(const Cascade&) = delete;
        Cascade& operator=(const Cascade&) = delete;
    };

    void Occurrence(const A3DAsmProductOccurrence* po, const Mat4& parentW, const A3DMiscCascadedAttributes* father, bool fatherShown, int depth)
    {
        Cascade ca(po, father);
        if (ca.removed) return;
        const bool shown = ca.shown && fatherShown;
        const Mat4 W = FromPrcTransform(EffectiveLocation(po)) * parentW;
        const std::string name = OccurrenceName(po);
        if (A3DAsmPartDefinition* part = EffectivePart(po))
        {
            Cascade pa(part, ca.attr);
            const std::string partName = EntityName(part).empty() ? name : EntityName(part);
            auto attrs = PrcAttributes(po);
            for (auto& a : PrcAttributes(part)) attrs.push_back(a);
            A3DAsmPartDefinitionData pd = A3D_MAKE_DATA(A3DAsmPartDefinitionData);
            if (A3DAsmPartDefinitionGet(part, &pd) == A3D_SUCCESS)
            {
                for (A3DUns32 i = 0; i < pd.m_uiRepItemsSize; ++i) Item(pd.m_ppRepItems[i], W, pa.attr, shown, partName, attrs, po);
                if (opt_.pmi)
                {
                    for (A3DUns32 i = 0; i < pd.m_uiAnnotationsSize; ++i) Annotation(pd.m_ppAnnotations[i], W, po);
                    for (A3DUns32 i = 0; i < pd.m_uiViewsSize; ++i) View(pd.m_ppViews[i], W, depth);
                }
                A3DAsmPartDefinitionGet(nullptr, &pd);
            }
        }
        if (opt_.pmi) OccurrencePmi(po, W, depth);
        for (A3DAsmProductOccurrence* k : EffectiveChildren(po)) Occurrence(k, W, ca.attr, shown, depth + 1);
    }

    void OccurrencePmi(const A3DAsmProductOccurrence* po, const Mat4& W, int depth)
    {
        bool gotA = false, gotV = false;
        for (const A3DAsmProductOccurrence* p = po; p && (!gotA || !gotV);)
        {
            A3DAsmProductOccurrenceData d = A3D_MAKE_DATA(A3DAsmProductOccurrenceData);
            if (A3DAsmProductOccurrenceGet(p, &d) != A3D_SUCCESS) break;
            if (!gotA && d.m_uiAnnotationsSize) { for (A3DUns32 i = 0; i < d.m_uiAnnotationsSize; ++i) Annotation(d.m_ppAnnotations[i], W, po); gotA = true; }
            if (!gotV && d.m_uiViewsSize) { for (A3DUns32 i = 0; i < d.m_uiViewsSize; ++i) View(d.m_ppViews[i], W, depth); gotV = true; }
            const A3DAsmProductOccurrence* next = d.m_pPrototype;
            A3DAsmProductOccurrenceGet(nullptr, &d);
            p = next;
        }
    }

    void Item(const A3DRiRepresentationItem* ri, const Mat4& W, const A3DMiscCascadedAttributes* father, bool fatherShown,
              const std::string& name, const std::vector<std::pair<std::string, std::string>>& attrs, const A3DAsmProductOccurrence* po)
    {
        A3DEEntityType t = kA3DTypeUnknown;
        if (!ri || A3DEntityGetType(ri, &t) != A3D_SUCCESS) return;
        Cascade ca(ri, father);
        if (ca.removed) return;
        const bool shown = ca.shown && fatherShown;
        Mat4 L;
        A3DRiRepresentationItemData rd = A3D_MAKE_DATA(A3DRiRepresentationItemData);
        if (A3DRiRepresentationItemGet(ri, &rd) == A3D_SUCCESS)
        {
            if (rd.m_pCoordinateSystem)
            {
                A3DRiCoordinateSystemData cs = A3D_MAKE_DATA(A3DRiCoordinateSystemData);
                if (A3DRiCoordinateSystemGet(rd.m_pCoordinateSystem, &cs) == A3D_SUCCESS)
                {
                    L = FromPrcTransform(cs.m_pTransformation);
                    A3DRiCoordinateSystemGet(nullptr, &cs);
                }
            }
            A3DRiRepresentationItemGet(nullptr, &rd);
        }
        const Mat4 Wri = L * W;
        if (t == kA3DTypeRiSet)
        {
            A3DRiSetData sd = A3D_MAKE_DATA(A3DRiSetData);
            if (A3DRiSetGet(ri, &sd) != A3D_SUCCESS) return;
            for (A3DUns32 i = 0; i < sd.m_uiRepItemsSize; ++i) Item(sd.m_ppRepItems[i], Wri, ca.attr, shown, name, attrs, po);
            A3DRiSetGet(nullptr, &sd);
        }
        else if (t == kA3DTypeRiBrepModel)
        {
            if (!shown && opt_.skipHidden) { ++st.hiddenSkipped; return; }
            std::string label = EntityName(ri);
            if (label.empty()) label = name;
            Brep(ri, Wri, ca.color, !shown, label, attrs, po);
        }
        else if (t == kA3DTypeRiPolyBrepModel) ++st.meshesSkipped;
    }

    // ---- B-rep ----------------------------------------------------------------------------------------------------
    A3DRiBrepModel* Adapt(const A3DRiBrepModel* ri, double tol, bool split, bool computeUV, bool compute3D)
    {
        A3DUns32 surfs[] = {kA3DTypeSurfNurbs};
        A3DUns32 crvs[] = {kA3DTypeCrvNurbs};
        A3DCopyAndAdaptBrepModelData s = A3D_MAKE_DATA(A3DCopyAndAdaptBrepModelData);
        s.m_bUseSameParam = false;
        s.m_dTol = tol;
        s.m_bDeleteCrossingUV = false;
        s.m_bSplitFaces = split;
        s.m_bSplitClosedFaces = false;
        s.m_bForceComputeUV = computeUV;
        s.m_bAllowUVCrossingSeams = false;
        s.m_bForceCompute3D = compute3D;
        s.m_uiAcceptableSurfacesSize = 1;
        s.m_puiAcceptableSurfaces = surfs;
        s.m_uiAcceptableCurvesSize = 1;
        s.m_puiAcceptableCurves = crvs;
        s.m_bContinueOnError = false;
        s.m_bClampTolerantUVCurvesInsideUVDomain = false;
        s.m_bForceDuplicateGeometries = false;
        A3DEntity* out = nullptr;
        if (A3DCopyAndAdaptBrepModel(ri, &s, &out) != A3D_SUCCESS || !out) return nullptr;
        A3DEEntityType t = kA3DTypeUnknown;
        A3DEntityGetType(out, &t);
        if (t != kA3DTypeRiBrepModel) { A3DEntityDelete(out); return nullptr; }
        return (A3DRiBrepModel*)out;
    }

    // Does the adapted B-rep miss p-curves on co-edges, or 3D curves on non-degenerate edges?
    static void MissingCurves(const A3DRiBrepModel* ri, bool& noUV, bool& no3D)
    {
        noUV = no3D = false;
        A3DRiBrepModelData rd = A3D_MAKE_DATA(A3DRiBrepModelData);
        if (A3DRiBrepModelGet(ri, &rd) != A3D_SUCCESS) return;
        A3DTopoBrepDataData bd = A3D_MAKE_DATA(A3DTopoBrepDataData);
        if (rd.m_pBrepData && A3DTopoBrepDataGet(rd.m_pBrepData, &bd) == A3D_SUCCESS)
        {
            for (A3DUns32 c = 0; c < bd.m_uiConnexSize && !(noUV && no3D); ++c)
            {
                A3DTopoConnexData cd = A3D_MAKE_DATA(A3DTopoConnexData);
                if (A3DTopoConnexGet(bd.m_ppConnexes[c], &cd) != A3D_SUCCESS) continue;
                for (A3DUns32 s = 0; s < cd.m_uiShellSize; ++s)
                {
                    A3DTopoShellData sd = A3D_MAKE_DATA(A3DTopoShellData);
                    if (A3DTopoShellGet(cd.m_ppShells[s], &sd) != A3D_SUCCESS) continue;
                    for (A3DUns32 f = 0; f < sd.m_uiFaceSize; ++f)
                    {
                        A3DTopoFaceData fd = A3D_MAKE_DATA(A3DTopoFaceData);
                        if (A3DTopoFaceGet(sd.m_ppFaces[f], &fd) != A3D_SUCCESS) continue;
                        for (A3DUns32 l = 0; l < fd.m_uiLoopSize; ++l)
                        {
                            A3DTopoLoopData ld = A3D_MAKE_DATA(A3DTopoLoopData);
                            if (A3DTopoLoopGet(fd.m_ppLoops[l], &ld) != A3D_SUCCESS) continue;
                            for (A3DUns32 k = 0; k < ld.m_uiCoEdgeSize; ++k)
                            {
                                A3DTopoCoEdgeData ce = A3D_MAKE_DATA(A3DTopoCoEdgeData);
                                if (A3DTopoCoEdgeGet(ld.m_ppCoEdges[k], &ce) != A3D_SUCCESS) continue;
                                if (!ce.m_pUVCurve) noUV = true;
                                A3DTopoEdgeData ed = A3D_MAKE_DATA(A3DTopoEdgeData);
                                if (ce.m_pEdge && A3DTopoEdgeGet(ce.m_pEdge, &ed) == A3D_SUCCESS)
                                {
                                    if (!ed.m_p3dCurve && ed.m_pStartVertex != ed.m_pEndVertex) no3D = true;
                                    A3DTopoEdgeGet(nullptr, &ed);
                                }
                                A3DTopoCoEdgeGet(nullptr, &ce);
                            }
                            A3DTopoLoopGet(nullptr, &ld);
                        }
                        A3DTopoFaceGet(nullptr, &fd);
                    }
                    A3DTopoShellGet(nullptr, &sd);
                }
                A3DTopoConnexGet(nullptr, &cd);
            }
            A3DTopoBrepDataGet(nullptr, &bd);
        }
        A3DRiBrepModelGet(nullptr, &rd);
    }

    // Original faces by (connex, shell, face), for their colours and names.
    static std::map<std::vector<A3DUns32>, const A3DTopoFace*> FaceIndex(const A3DTopoBrepData* brep, long& count)
    {
        std::map<std::vector<A3DUns32>, const A3DTopoFace*> idx;
        count = 0;
        A3DTopoBrepDataData bd = A3D_MAKE_DATA(A3DTopoBrepDataData);
        if (!brep || A3DTopoBrepDataGet(brep, &bd) != A3D_SUCCESS) return idx;
        for (A3DUns32 c = 0; c < bd.m_uiConnexSize; ++c)
        {
            A3DTopoConnexData cd = A3D_MAKE_DATA(A3DTopoConnexData);
            if (A3DTopoConnexGet(bd.m_ppConnexes[c], &cd) != A3D_SUCCESS) continue;
            for (A3DUns32 s = 0; s < cd.m_uiShellSize; ++s)
            {
                A3DTopoShellData sd = A3D_MAKE_DATA(A3DTopoShellData);
                if (A3DTopoShellGet(cd.m_ppShells[s], &sd) != A3D_SUCCESS) continue;
                for (A3DUns32 f = 0; f < sd.m_uiFaceSize; ++f) { idx[{c, s, f}] = sd.m_ppFaces[f]; ++count; }
                A3DTopoShellGet(nullptr, &sd);
            }
            A3DTopoConnexGet(nullptr, &cd);
        }
        A3DTopoBrepDataGet(nullptr, &bd);
        return idx;
    }

    struct EdgeRec { long id = 0; Vec3 start, end; const A3DTopoEdge* entity = nullptr; double tol = 0; };
    struct SurfRec { long id = 0; SurfNurbs s; };
    struct BrepCtx
    {
        Mat4 M;
        double scale = 1.0;   // length scale of M (tolerances)
        double tol = 1e-6;
        std::map<const A3DTopoVertex*, std::pair<long, Vec3>> vertices;
        std::map<const A3DTopoVertex*, double> vertexTol;
        std::map<std::tuple<long long, long long, long long>, long> vertexAt;
        std::map<const A3DTopoEdge*, EdgeRec> edges;
        std::map<const A3DTopoCoEdge*, EdgeRec> coedgeEdge;
        std::map<const A3DSurfBase*, SurfRec> surfaces;
    };

    std::pair<long, Vec3> Vertex(BrepCtx& b, const A3DTopoVertex* v)
    {
        auto it = b.vertices.find(v);
        if (it != b.vertices.end()) return it->second;
        Vec3 p;
        A3DEEntityType t = kA3DTypeUnknown;
        A3DEntityGetType(v, &t);
        if (t == kA3DTypeTopoUniqueVertex)
        {
            A3DTopoUniqueVertexData d = A3D_MAKE_DATA(A3DTopoUniqueVertexData);
            if (A3DTopoUniqueVertexGet(v, &d) == A3D_SUCCESS) { p = V(d.m_sPoint); b.vertexTol[v] = d.m_dTolerance * b.scale; A3DTopoUniqueVertexGet(nullptr, &d); }
        }
        else if (t == kA3DTypeTopoMultipleVertex)
        {
            A3DTopoMultipleVertexData d = A3D_MAKE_DATA(A3DTopoMultipleVertexData);
            if (A3DTopoMultipleVertexGet(v, &d) == A3D_SUCCESS) { if (d.m_uiSize) p = V(d.m_pPts[0]); A3DTopoMultipleVertexGet(nullptr, &d); }
        }
        p = b.M.point(p);
        Grow(p);
        // Vertices at the same place (several PRC vertex entities) are one QIF vertex.
        const double q = std::max(1e-12, 1e-9 * modelSize);
        const auto key = std::make_tuple(std::llround(p.x / q), std::llround(p.y / q), std::llround(p.z / q));
        auto at = b.vertexAt.find(key);
        long vid;
        if (at != b.vertexAt.end()) vid = at->second;
        else
        {
            const long pid = id();
            points_.push_back(E("Point").a("id", pid).c(E("XYZ").raw(qo::vec(p))));
            vid = id();
            vertices_.push_back(E("Vertex").a("id", vid).ref("Point", pid));
            b.vertexAt[key] = vid;
            ++st.vertices;
        }
        return b.vertices[v] = {vid, p};
    }

    EdgeRec MakeEdge(BrepCtx& b, const A3DTopoEdge* e)
    {
        EdgeRec r;
        r.entity = e;
        A3DTopoEdgeData d = A3D_MAKE_DATA(A3DTopoEdgeData);
        if (A3DTopoEdgeGet(e, &d) != A3D_SUCCESS) return r;
        auto vs = Vertex(b, d.m_pStartVertex);
        auto ve = Vertex(b, d.m_pEndVertex);
        r.start = vs.second;
        r.end = ve.second;
        // tolerance of a tolerant join: the edge's, or its vertices' when larger
        r.tol = std::max({d.m_dTolerance * b.scale, b.vertexTol[d.m_pStartVertex], b.vertexTol[d.m_pEndVertex]});
        r.id = id();
        E edge("Edge");
        edge.a("id", r.id);
        CurveNurbs c = GetCurveNurbs(d.m_p3dCurve, b.tol);
        if (c.ok)
        {
            for (Vec3& p : c.cps) p = b.M.point(p);
            const int n = (int)c.cps.size();
            double d0 = c.knots[c.degree], d1 = c.knots[n];
            if (d.m_bHasTrimDomain) { d0 = d.m_sInterval.m_dMin; d1 = d.m_sInterval.m_dMax; }
            const long cid = id();
            E core("Nurbs13Core");
            core.a("domain", qo::vec2(d0, d1));
            core.c(E("Order").raw(std::to_string(c.degree + 1)));
            core.c(E("Knots").a("count", c.knots.size()).raw(qo::nums(c.knots)));
            std::vector<double> flat;
            for (const Vec3& p : c.cps) { flat.push_back(p.x); flat.push_back(p.y); flat.push_back(p.z); }
            core.c(E("CPs").a("count", c.cps.size()).raw(qo::nums(flat)));
            if (!c.weights.empty()) core.c(E("Weights").a("count", c.weights.size()).raw(qo::nums(c.weights)));
            curves13_.push_back(E("Nurbs13").a("id", cid).c(std::move(core)));
            ++st.curves3d;
            edge.ref("Curve", cid);
        }
        else ++st.edgesNo3d;
        edge.ref("VertexBeg", vs.first).ref("VertexEnd", ve.first);
        edges_.push_back(std::move(edge));
        ++st.edges;
        A3DTopoEdgeGet(nullptr, &d);
        return r;
    }

    // The QIF edge of a co-edge: shared with its PRC neighbours (the other co-edges of the same edge).
    EdgeRec EdgeOf(BrepCtx& b, const A3DTopoCoEdge* ce, const A3DTopoEdge* e, const A3DTopoCoEdge* neighbour)
    {
        if (auto it = b.coedgeEdge.find(ce); it != b.coedgeEdge.end()) return it->second;
        EdgeRec r;
        if (auto it = b.edges.find(e); it != b.edges.end()) r = it->second;
        else
        {
            int guard = 0;
            for (const A3DTopoCoEdge* n = neighbour; n && n != ce && guard < 16; ++guard)
            {
                if (auto it = b.coedgeEdge.find(n); it != b.coedgeEdge.end()) { r = it->second; break; }
                A3DTopoCoEdgeData nd = A3D_MAKE_DATA(A3DTopoCoEdgeData);
                if (A3DTopoCoEdgeGet(n, &nd) != A3D_SUCCESS) break;
                const A3DTopoCoEdge* next = nd.m_pNeighbor;
                A3DTopoCoEdgeGet(nullptr, &nd);
                n = next;
            }
            if (!r.id) r = MakeEdge(b, e);
            b.edges[e] = r;
        }
        b.coedgeEdge[ce] = r;
        return r;
    }

    const SurfRec* Surface(BrepCtx& b, const A3DSurfBase* s)
    {
        if (auto it = b.surfaces.find(s); it != b.surfaces.end()) return it->second.id ? &it->second : nullptr;
        SurfRec r;
        r.s = GetSurfNurbs(s, b.tol, b.M);
        if (!r.s.ok) { ++st.surfacesFailed; b.surfaces[s] = r; return nullptr; }
        r.id = id();
        E core("Nurbs23Core");
        core.c(E("OrderU").raw(std::to_string(r.s.du + 1)));
        core.c(E("OrderV").raw(std::to_string(r.s.dv + 1)));
        core.c(E("KnotsU").a("count", r.s.ku.size()).raw(qo::nums(r.s.ku)));
        core.c(E("KnotsV").a("count", r.s.kv.size()).raw(qo::nums(r.s.kv)));
        std::vector<double> flat;
        flat.reserve(r.s.cps.size() * 3);
        for (const Vec3& p : r.s.cps) { flat.push_back(p.x); flat.push_back(p.y); flat.push_back(p.z); }
        core.c(E("CPs").a("count", r.s.cps.size()).raw(qo::nums(flat)));
        if (!r.s.w.empty()) core.c(E("Weights").a("count", r.s.w.size()).raw(qo::nums(r.s.w)));
        surfaces_.push_back(E("Nurbs23").a("id", r.id).c(std::move(core)));
        ++st.surfaces;
        return &(b.surfaces[s] = std::move(r));
    }

    void Brep(const A3DRiBrepModel* ri, const Mat4& W, const Rgb& bodyColor, bool hidden, const std::string& label,
              const std::vector<std::pair<std::string, std::string>>& attrs, const A3DAsmProductOccurrence* po)
    {
        ++st.sourceBreps;
        A3DRiBrepModelData od = A3D_MAKE_DATA(A3DRiBrepModelData);
        if (A3DRiBrepModelGet(ri, &od) != A3D_SUCCESS) { ++st.failedBreps; return; }
        const A3DTopoBrepData* origBrep = od.m_pBrepData;
        const bool solidFlag = od.m_bSolid;
        A3DRiBrepModelGet(nullptr, &od);
        long origFaceCount = 0;
        const auto origFaces = FaceIndex(origBrep, origFaceCount);
        st.sourceFaces += origFaceCount;

        BrepCtx b;
        b.M = Mat4::scale(BrepScale(origBrep)) * W;
        b.scale = std::cbrt(std::fabs(b.M.det3()));
        if (!(b.scale > 0)) b.scale = 1.0;
        const bool mirror = b.M.det3() < 0;
        double diag = 0;
        {
            A3DTopoBrepDataData bd = A3D_MAKE_DATA(A3DTopoBrepDataData);
            if (A3DTopoBrepDataGet(origBrep, &bd) == A3D_SUCCESS)
            {
                diag = (V(bd.m_sBoundingBox.m_sMax) - V(bd.m_sBoundingBox.m_sMin)).norm();
                A3DTopoBrepDataGet(nullptr, &bd);
            }
        }
        b.tol = std::min(1e-2, std::max(1e-6, (std::isfinite(diag) ? diag : 1.0) * 1e-6));

        // First keep the B-rep's own curves; when p-curves or 3D curves are missing (common in CATIA / NX data, which
        // store only one of them), let HOOPS Exchange compute them.
        bool split = false;
        A3DRiBrepModel* nb = Adapt(ri, b.tol, false, false, false);
        bool noUV = false, no3D = false;
        if (nb) MissingCurves(nb, noUV, no3D);
        if (!nb || noUV || no3D)
        {
            if (nb) A3DEntityDelete(nb);
            nb = Adapt(ri, b.tol, false, true, true);
            if (noUV) ++st.recomputedUV;
            if (no3D) ++st.recomputed3D;
        }
        if (!nb) { nb = Adapt(ri, b.tol, true, true, true); split = nb != nullptr; }
        if (!nb) { ++st.failedBreps; printf("WARNING: B-rep '%s' could not be adapted to NURBS: not written\n", label.c_str()); return; }

        A3DRiBrepModelData nd = A3D_MAKE_DATA(A3DRiBrepModelData);
        A3DRiBrepModelGet(nb, &nd);
        A3DTopoBrepDataData bdd = A3D_MAKE_DATA(A3DTopoBrepDataData);
        if (!nd.m_pBrepData || A3DTopoBrepDataGet(nd.m_pBrepData, &bdd) != A3D_SUCCESS)
        {
            A3DRiBrepModelGet(nullptr, &nd);
            A3DEntityDelete(nb);
            ++st.failedBreps;
            return;
        }
        long newFaceCount = 0;
        FaceIndex(nd.m_pBrepData, newFaceCount);
        if (newFaceCount != origFaceCount) split = true;
        if (split) ++st.splitBreps;

        BrepInstance inst;
        inst.po = po;
        inst.split = split;
        std::vector<long> shellIds, bodyFaces;
        bool allClosed = bdd.m_uiConnexSize > 0;
        A3DUns32 faceIndex = 0;   // running index over the B-rep's faces (reference-on-topology numbering)
        for (A3DUns32 c = 0; c < bdd.m_uiConnexSize; ++c)
        {
            A3DTopoConnexData cd = A3D_MAKE_DATA(A3DTopoConnexData);
            if (A3DTopoConnexGet(bdd.m_ppConnexes[c], &cd) != A3D_SUCCESS) continue;
            for (A3DUns32 s = 0; s < cd.m_uiShellSize; ++s)
            {
                A3DTopoShellData sd = A3D_MAKE_DATA(A3DTopoShellData);
                if (A3DTopoShellGet(cd.m_ppShells[s], &sd) != A3D_SUCCESS) continue;
                std::vector<long> shellFaces;
                for (A3DUns32 f = 0; f < sd.m_uiFaceSize; ++f)
                {
                    const bool reversed = sd.m_pucOrientationWithShell && sd.m_pucOrientationWithShell[f] == 0;
                    const A3DTopoFace* orig = nullptr;
                    if (!split) if (auto it = origFaces.find({c, s, f}); it != origFaces.end()) orig = it->second;
                    const A3DUns32 gi = faceIndex++;
                    const long fid = Face(b, sd.m_ppFaces[f], orig, reversed != mirror, split ? std::vector<A3DUns32>() : std::vector<A3DUns32>{gi}, inst);
                    if (!fid) continue;
                    shellFaces.push_back(fid);
                    bodyFaces.push_back(fid);
                    if (!split) inst.faces[{gi}] = fid;
                }
                if (!shellFaces.empty())
                {
                    const long sid = id();
                    E shell("Shell");
                    shell.a("id", sid);
                    if (sd.m_bClosed) shell.a("closed", "true");
                    shell.ids("FaceIds", shellFaces);
                    shells_.push_back(std::move(shell));
                    shellIds.push_back(sid);
                }
                allClosed = allClosed && sd.m_bClosed;
                A3DTopoShellGet(nullptr, &sd);
            }
            A3DTopoConnexGet(nullptr, &cd);
        }
        A3DTopoBrepDataGet(nullptr, &bdd);
        A3DRiBrepModelGet(nullptr, &nd);
        A3DEntityDelete(nb);
        if (bodyFaces.empty()) { ++st.failedBreps; return; }

        const long bid = id();
        E body("Body");
        body.a("id", bid).a("label", label.empty() ? "Body" + std::to_string(bid) : label);
        body.a("form", (allClosed && solidFlag) || allClosed ? "SOLID" : "SHEET");
        if (bodyColor.set) body.a("color", ColorText(bodyColor));
        if (hidden) body.a("hidden", "true");
        if (!attrs.empty()) body.c(AttributesElement(attrs));
        body.ids("ShellIds", shellIds);
        body.ids("FaceIds", bodyFaces);
        bodies_.push_back(std::move(body));
        bodyIds_.push_back(bid);
        ++st.bodies;
        inst.allFaces = bodyFaces;
        breps_[origBrep].push_back(std::move(inst));
    }

    long Face(BrepCtx& b, const A3DTopoFace* face, const A3DTopoFace* orig, bool turned, const std::vector<A3DUns32>& key, BrepInstance& inst)
    {
        A3DTopoFaceData fd = A3D_MAKE_DATA(A3DTopoFaceData);
        if (A3DTopoFaceGet(face, &fd) != A3D_SUCCESS) return 0;
        const SurfRec* sr = Surface(b, fd.m_pSurface);
        if (!sr) { A3DTopoFaceGet(nullptr, &fd); return 0; }
        // outer loop first
        std::vector<A3DUns32> order;
        const bool hasOuter = fd.m_uiOuterLoopIndex < fd.m_uiLoopSize;
        if (hasOuter) order.push_back(fd.m_uiOuterLoopIndex);
        for (A3DUns32 l = 0; l < fd.m_uiLoopSize; ++l) if (!hasOuter || l != fd.m_uiOuterLoopIndex) order.push_back(l);
        std::vector<long> loopIds;
        for (A3DUns32 li : order)
        {
            A3DTopoLoopData ld = A3D_MAKE_DATA(A3DTopoLoopData);
            if (A3DTopoLoopGet(fd.m_ppLoops[li], &ld) != A3D_SUCCESS) continue;
            const bool loopReversed = ld.m_ucOrientationWithSurface == 0;
            std::vector<A3DUns32> ceOrder;
            for (A3DUns32 k = 0; k < ld.m_uiCoEdgeSize; ++k) ceOrder.push_back(loopReversed ? ld.m_uiCoEdgeSize - 1 - k : k);
            E coedges("CoEdges");
            coedges.a("n", ld.m_uiCoEdgeSize);
            for (A3DUns32 k : ceOrder)
            {
                const A3DTopoCoEdge* ce = ld.m_ppCoEdges[k];
                A3DTopoCoEdgeData cd = A3D_MAKE_DATA(A3DTopoCoEdgeData);
                if (A3DTopoCoEdgeGet(ce, &cd) != A3D_SUCCESS) continue;
                bool withLoop = cd.m_ucOrientationWithLoop != 0, uvWithLoop = cd.m_ucOrientationUVWithLoop != 0;
                if (loopReversed) { withLoop = !withLoop; uvWithLoop = !uvWithLoop; }
                const EdgeRec er = EdgeOf(b, ce, cd.m_pEdge, cd.m_pNeighbor);
                // own start / end of this co-edge's PRC edge
                Vec3 ownStart = er.start, ownEnd = er.end;
                if (er.entity != cd.m_pEdge)
                {
                    A3DTopoEdgeData ed = A3D_MAKE_DATA(A3DTopoEdgeData);
                    if (A3DTopoEdgeGet(cd.m_pEdge, &ed) == A3D_SUCCESS)
                    {
                        ownStart = Vertex(b, ed.m_pStartVertex).second;
                        ownEnd = Vertex(b, ed.m_pEndVertex).second;
                        A3DTopoEdgeGet(nullptr, &ed);
                    }
                }
                const Vec3 ceStart = withLoop ? ownStart : ownEnd, ceEnd = withLoop ? ownEnd : ownStart;
                bool edgeTurned = !withLoop;
                if (er.entity != cd.m_pEdge && (er.start - er.end).norm() > 1e-12)
                    edgeTurned = (ceStart - er.end).norm() < (ceStart - er.start).norm();
                E co("CoEdge");
                E eo("EdgeOriented");
                if (edgeTurned) eo.a("turned", "true");
                eo.c(E("Id").raw(std::to_string(er.id)));
                co.c(std::move(eo));
                CurveNurbs pc = GetCurveNurbs(cd.m_pUVCurve, b.tol);
                if (pc.ok)
                {
                    if (!uvWithLoop) ReverseNurbs(pc);
                    const long pid = id();
                    E core("Nurbs12Core");
                    core.c(E("Order").raw(std::to_string(pc.degree + 1)));
                    core.c(E("Knots").a("count", pc.knots.size()).raw(qo::nums(pc.knots)));
                    std::vector<double> flat;
                    for (const Vec3& p : pc.cps) { flat.push_back(p.x); flat.push_back(p.y); }
                    core.c(E("CPs").a("count", pc.cps.size()).raw(qo::nums(flat)));
                    if (!pc.weights.empty()) core.c(E("Weights").a("count", pc.weights.size()).raw(qo::nums(pc.weights)));
                    curves12_.push_back(E("Nurbs12").a("id", pid).c(std::move(core)));
                    ++st.pcurves;
                    co.ref("Curve12", pid);
                    // self-check: the p-curve ends, evaluated on the surface, are the co-edge's vertices
                    const int n = (int)pc.cps.size();
                    const Vec3 uv0 = qo::evalNurbs(pc.degree, pc.knots, pc.cps, pc.weights, pc.knots[pc.degree]);
                    const Vec3 uv1 = qo::evalNurbs(pc.degree, pc.knots, pc.cps, pc.weights, pc.knots[n]);
                    const Vec3 p0 = qo::evalNurbsSurface(sr->s.du, sr->s.dv, sr->s.ku, sr->s.kv, sr->s.nu, sr->s.nv, sr->s.cps, sr->s.w, uv0.x, uv0.y);
                    const Vec3 p1 = qo::evalNurbsSurface(sr->s.du, sr->s.dv, sr->s.ku, sr->s.kv, sr->s.nu, sr->s.nv, sr->s.cps, sr->s.w, uv1.x, uv1.y);
                    const double gap = std::max((p0 - ceStart).norm(), (p1 - ceEnd).norm());
                    st.worstGap = std::max(st.worstGap, gap);
                    if (gap > 1e-9) st.gaps.push_back({gap, er.tol});
                    ++st.checkedCoEdges;
                }
                else ++st.coedgesNoUV;
                coedges.c(std::move(co));
                if (!key.empty()) inst.edges[{key[0], li, k}] = er.id;
                A3DTopoCoEdgeGet(nullptr, &cd);
            }
            const long lid = id();
            E loop("Loop");
            loop.a("id", lid);
            if (hasOuter) loop.a("form", li == fd.m_uiOuterLoopIndex ? "OUTER" : "INNER");
            loop.c(std::move(coedges));
            loops_.push_back(std::move(loop));
            loopIds.push_back(lid);
            ++st.loops;
            A3DTopoLoopGet(nullptr, &ld);
        }
        A3DTopoFaceGet(nullptr, &fd);
        const long fid = id();
        E f("Face");
        f.a("id", fid);
        const std::string name = EntityName(orig ? (const A3DEntity*)orig : (const A3DEntity*)face);
        if (!name.empty()) f.a("label", name);
        Rgb c = OwnColor(orig ? (const A3DEntity*)orig : (const A3DEntity*)face);
        if (c.set) f.a("color", ColorText(c));
        if (turned) f.a("turned", "true");
        if (!hasOuter) f.a("hasOuter", "false");
        f.ref("Surface", sr->id);
        f.ids("LoopIds", loopIds);
        faces_.push_back(std::move(f));
        ++st.faces;
        return fid;
    }

    // ---- PMI collection -------------------------------------------------------------------------------------------
    void Annotation(const A3DMkpAnnotationEntity* e, const Mat4& W, const A3DAsmProductOccurrence* owner)
    {
        A3DEEntityType t = kA3DTypeUnknown;
        if (!e || A3DEntityGetType(e, &t) != A3D_SUCCESS) return;
        if (t == kA3DTypeMkpAnnotationSet)
        {
            A3DMkpAnnotationSetData sd = A3D_MAKE_DATA(A3DMkpAnnotationSetData);
            if (A3DMkpAnnotationSetGet(e, &sd) != A3D_SUCCESS) return;
            for (A3DUns32 i = 0; i < sd.m_uiAnnotationsSize; ++i) Annotation(sd.m_ppAnnotations[i], W, owner);
            A3DMkpAnnotationSetGet(nullptr, &sd);
            return;
        }
        if (t != kA3DTypeMkpAnnotationItem) return;
        if (!pmiSeen_.insert({e, owner}).second) return;   // listed by the occurrence and its part
        OwnedPmi o;
        if (!ReadAnnotationItem(e, o.item)) return;
        o.W = W;
        o.owner = owner;
        pmi_.push_back(std::move(o));
    }

    static void FlattenItems(const A3DMkpAnnotationEntity* e, std::vector<const void*>& out)
    {
        A3DEEntityType t = kA3DTypeUnknown;
        if (!e || A3DEntityGetType(e, &t) != A3D_SUCCESS) return;
        if (t == kA3DTypeMkpAnnotationItem) { out.push_back(e); return; }
        if (t == kA3DTypeMkpAnnotationReference)
        {
            A3DMkpAnnotationReferenceData rd = A3D_MAKE_DATA(A3DMkpAnnotationReferenceData);
            if (A3DMkpAnnotationReferenceGet(e, &rd) == A3D_SUCCESS)
            {
                for (A3DUns32 i = 0; i < rd.m_uiLinkedItemsSize; ++i)
                {
                    A3DMiscMarkupLinkedItemData ld = A3D_MAKE_DATA(A3DMiscMarkupLinkedItemData);
                    if (A3DMiscMarkupLinkedItemGet(rd.m_ppLinkedItems[i], &ld) == A3D_SUCCESS)
                    {
                        if (ld.m_pReference) out.push_back(ld.m_pReference);
                        A3DMiscMarkupLinkedItemGet(nullptr, &ld);
                    }
                }
                A3DMkpAnnotationReferenceGet(nullptr, &rd);
            }
            return;
        }
        if (t != kA3DTypeMkpAnnotationSet) return;
        A3DMkpAnnotationSetData sd = A3D_MAKE_DATA(A3DMkpAnnotationSetData);
        if (A3DMkpAnnotationSetGet(e, &sd) != A3D_SUCCESS) return;
        for (A3DUns32 i = 0; i < sd.m_uiAnnotationsSize; ++i) FlattenItems(sd.m_ppAnnotations[i], out);
        A3DMkpAnnotationSetGet(nullptr, &sd);
    }

    void View(const A3DMkpView* v, const Mat4& W, int depth)
    {
        A3DMkpViewData vd = A3D_MAKE_DATA(A3DMkpViewData);
        if (!v || A3DMkpViewGet(v, &vd) != A3D_SUCCESS) return;
        ViewRec w;
        w.name = EntityName(v);
        w.isDefault = vd.m_bIsDefaultView;
        for (A3DUns32 i = 0; i < vd.m_uiAnnotationsSize; ++i) FlattenItems(vd.m_ppAnnotations[i], w.annots);
        bool camera = false;
        if (vd.m_pSceneDisplayParameters && !vd.m_bIsAnnotationView && !(depth > 0 && w.annots.empty()))
        {
            A3DGraphSceneDisplayParametersData sd = A3D_MAKE_DATA(A3DGraphSceneDisplayParametersData);
            if (A3DGraphSceneDisplayParametersGet(vd.m_pSceneDisplayParameters, &sd) == A3D_SUCCESS)
            {
                A3DGraphCameraData cd = A3D_MAKE_DATA(A3DGraphCameraData);
                if (sd.m_pCamera && A3DGraphCameraGet(sd.m_pCamera, &cd) == A3D_SUCCESS)
                {
                    camera = true;
                    w.ortho = cd.m_bOrthographic;
                    w.loc = W.point(V(cd.m_sLocation));
                    w.target = W.point(V(cd.m_sLookAt));
                    w.up = W.dir(V(cd.m_sUp)).unit();
                    w.xfov = cd.m_dXFovy;
                    w.yfov = cd.m_dYFovy;
                    w.znear = cd.m_dZNear;
                    w.zfar = cd.m_dZFar;
                    w.aspect = cd.m_dAspectRatio > 0 ? cd.m_dAspectRatio : 1.0;
                    A3DGraphCameraGet(nullptr, &cd);
                }
                A3DGraphSceneDisplayParametersGet(nullptr, &sd);
            }
        }
        if (camera) views_.push_back(std::move(w));
        A3DMkpViewGet(nullptr, &vd);
    }

    // ---- PMI output -----------------------------------------------------------------------------------------------
    std::vector<long> ResolveLinks(const OwnedPmi& o)
    {
        std::vector<long> ids;
        std::set<long> seen;
        for (const PmiLink& l : o.item.links)
        {
            auto it = breps_.find(l.brep);
            if (it == breps_.end() || it->second.empty())
            {
                ++st.unresolvedLinks;
                if (g_verbose) printf("  [pmi] '%s': link to an unknown B-rep (%p, type %d, %zu indices)\n", o.item.name.c_str(), (const void*)l.brep, (int)l.topoType, l.indices.size());
                continue;
            }
            const BrepInstance* inst = &it->second.front();
            for (const BrepInstance& bi : it->second)
                if ((l.target && bi.po == l.target) || (!l.target && bi.po == o.owner)) { inst = &bi; break; }
            std::vector<long> got;
            if (l.topoType == kA3DTypeTopoBrepData || inst->split) got = inst->allFaces;
            else if (l.topoType == kA3DTypeTopoFace && l.indices.size() >= 1)
            {
                if (auto f = inst->faces.find({l.indices[0]}); f != inst->faces.end()) got.push_back(f->second);
            }
            else if ((l.topoType == kA3DTypeTopoEdge || l.topoType == kA3DTypeTopoCoEdge || l.topoType == kA3DTypeTopoUniqueVertex ||
                      l.topoType == kA3DTypeTopoMultipleVertex) && l.indices.size() >= 3)
            {
                // a vertex reference is written as its edge (QIF links PMI to faces and edges)
                if (auto e = inst->edges.find({l.indices[0], l.indices[1], l.indices[2]}); e != inst->edges.end()) got.push_back(e->second);
            }
            if (got.empty())
            {
                ++st.unresolvedLinks;
                if (g_verbose) printf("  [pmi] '%s': link type %d with %zu indices not found\n", o.item.name.c_str(), (int)l.topoType, l.indices.size());
                continue;
            }
            for (long x : got) if (seen.insert(x).second) ids.push_back(x);
        }
        return ids;
    }

    long FeatureFor(const std::string& name, const std::vector<long>& entities)
    {
        if (entities.empty()) return 0;
        if (!featureDefId_)
        {
            featureDefId_ = id();
            featureDefs_.push_back(E("OtherNonShapeFeatureDefinition").a("id", featureDefId_).c("Description", "Topology linked to PMI (QIF_Writer)"));
        }
        const long fid = id();
        featureNominals_.push_back(E("OtherNonShapeFeatureNominal").a("id", fid).c("Name", name.empty() ? "Feature " + std::to_string(fid) : name)
                                       .c(E("FeatureDefinitionId").raw(std::to_string(featureDefId_)))
                                       .ids("EntityInternalIds", entities));
        featureIds_.push_back(fid);
        ++st.features;
        return fid;
    }

    long DatumDefinition(const std::string& label, const std::string& name, long feature)
    {
        auto it = datumByLabel_.find(label);
        if (it != datumByLabel_.end()) return it->second;
        const long did = id();
        E d("DatumDefinition");
        d.a("id", did);
        if (!name.empty() && name != "Datum " + label) d.a("label", name);
        d.c("DatumLabel", label);
        if (feature) d.ids("FeatureNominalIds", {feature});
        datumDefs_.push_back(std::move(d));
        datumIds_.push_back(did);
        return datumByLabel_[label] = did;
    }

    long DatumReferenceFrame(const PmiItem& p)
    {
        std::string key;
        for (int s = 0; s < 3; ++s)
        {
            key += "|";
            for (size_t k = 0; k < p.datums[s].labels.size(); ++k) key += p.datums[s].labels[k] + "/" + std::to_string(p.datums[s].modifiers[k]) + ",";
        }
        if (key == "|||") return 0;
        if (auto it = drfByKey_.find(key); it != drfByKey_.end()) return it->second;
        static const char* prec[3] = {"PRIMARY", "SECONDARY", "TERTIARY"};
        E datums("Datums");
        int n = 0;
        for (int s = 0; s < 3; ++s)
        {
            const PmiDatumRef& r = p.datums[s];
            if (r.labels.empty()) continue;
            E datum("Datum");
            if (r.labels.size() == 1)
                datum.c(E("SimpleDatum").c(E("DatumDefinitionId").raw(std::to_string(DatumDefinition(r.labels[0], "", 0))))
                            .c("MaterialModifier", QifMaterialModifier(r.modifiers[0]))
                            .c("ReferencedComponent", "NOMINAL"));
            else
            {
                E inner("Datums");
                inner.a("n", r.labels.size());
                for (size_t k = 0; k < r.labels.size(); ++k)
                    inner.c(E("Datum").c(E("DatumDefinitionId").raw(std::to_string(DatumDefinition(r.labels[k], "", 0))))
                                .c("MaterialModifier", QifMaterialModifier(r.modifiers[k]))
                                .c("ReferencedComponent", "NOMINAL"));
                datum.c(E("CompoundDatum").c(std::move(inner)));
            }
            datum.c(E("Precedence").c("PrecedenceEnum", prec[s]));
            datums.c(std::move(datum));
            ++n;
        }
        datums.a("n", n);
        const long rid = id();
        drfs_.push_back(E("DatumReferenceFrame").a("id", rid).c(std::move(datums)));
        drfIds_.push_back(rid);
        return drfByKey_[key] = rid;
    }

    static int DecimalPlaces(double acc)
    {
        if (!(acc > 0) || acc >= 1) return 0;
        return std::max(0, std::min(12, (int)std::lround(-std::log10(acc))));
    }

    // QIF semantic entity of one markup; returns the id the PMIDisplay references.
    long Semantic(const OwnedPmi& o, long feature, const std::vector<long>& entities)
    {
        const PmiItem& p = o.item;
        const std::string name = p.name;
        if (p.semType == kA3DTypeMarkupDatum)
        {
            if (p.target)
            {
                const long tid = id();
                E t("DatumTarget");
                t.a("id", tid);
                if (!name.empty() && name != "Datum target " + p.datumLabel) t.a("label", name);
                t.c("DatumTargetLabel", p.datumLabel);
                if (feature) t.c(E("FeatureNominalId").raw(std::to_string(feature)));
                datumTargets_.push_back(std::move(t));
                ++st.datumTargets;
                return tid;
            }
            if (auto it = datumByLabel_.find(p.datumLabel); it != datumByLabel_.end()) return it->second;
            ++st.datums;
            return DatumDefinition(p.datumLabel, name, feature);
        }
        if (p.semType == kA3DTypeMarkupGDT && QifGdtName(p.gdtType))
        {
            const std::string base = QifGdtName(p.gdtType);
            const bool orientationOrLocation = p.gdtType == kA3DGDTTypePosition || p.gdtType == kA3DGDTTypePerpendicularity ||
                                               p.gdtType == kA3DGDTTypeParallelism || p.gdtType == kA3DGDTTypeAngularity ||
                                               p.gdtType == kA3DGDTTypeStraightness;
            const long did = id();
            E def(base + "CharacteristicDefinition");
            def.a("id", did);
            def.c("StatisticalCharacteristic", p.statistical ? "true" : "false");
            def.c(E("ToleranceValue").raw(qo::num(p.tolerance)));
            if (const long drf = DatumReferenceFrame(p)) def.c(E("DatumReferenceFrameId").raw(std::to_string(drf)));
            if (orientationOrLocation || p.modifier) def.c("MaterialCondition", QifMaterialModifier(p.modifier));
            if (orientationOrLocation || p.zone == kA3DGDTValueDiameter || p.zone == kA3DGDTValueSpherical)
                def.c(E("ZoneShape").c(E(p.zone == kA3DGDTValueDiameter ? "DiametricalZone" : p.zone == kA3DGDTValueSpherical ? "SphericalZone" : "NonDiametricalZone")));
            if (p.freeState) def.c("FreeState", "true");
            charDefs_.push_back(std::move(def));
            const long nid = id();
            E nom(base + "CharacteristicNominal");
            nom.a("id", nid);
            nom.c(E("CharacteristicDefinitionId").raw(std::to_string(did)));
            if (feature) nom.ids("FeatureNominalIds", {feature});
            nom.c("Name", name.empty() ? base + "." + std::to_string(nid) : name);
            charNominals_.push_back(std::move(nom));
            charIds_.push_back(nid);
            ++st.gdt;
            ++st.characteristics;
            st.extraRows += p.extraRows;
            return nid;
        }
        if (p.semType == kA3DTypeMarkupDimension)
        {
            const std::string base = QifDimensionName(p);
            const bool angle = base == "Angle";
            const long did = id();
            const int dp = DecimalPlaces(p.accuracy);
            auto value = [&](const char* tag, double v) {
                E x(tag);
                if (p.accuracy > 0) x.a("decimalPlaces", dp);
                return x.raw(qo::num(v));
            };
            E tol("Tolerance");
            tol.c(value("MaxValue", p.sup)).c(value("MinValue", p.inf)).c("DefinedAsLimit", "false");
            charDefs_.push_back(E(base + "CharacteristicDefinition").a("id", did).c(std::move(tol)));
            const long nid = id();
            E nom(base + "CharacteristicNominal");
            nom.a("id", nid);
            if (!p.suffix.empty()) nom.c(AttributesElement({{"_3dv.TextSuffix", p.suffix}}));
            nom.c(E("CharacteristicDefinitionId").raw(std::to_string(did)));
            if (feature) nom.ids("FeatureNominalIds", {feature});
            nom.c("Name", name.empty() ? base + "." + std::to_string(nid) : name);
            nom.c(value("TargetValue", p.value));
            (void)angle;
            charNominals_.push_back(std::move(nom));
            charIds_.push_back(nid);
            ++st.dimensions;
            ++st.characteristics;
            return nid;
        }
        // notes, and every markup without a semantic QIF equivalent (roughness, welding, balloon, ...)
        std::string text;
        for (const auto& l : p.lines) text += (text.empty() ? "" : "\n") + l;
        if (text.empty()) text = p.text;
        const long nid = id();
        E note("Note");
        note.a("id", nid).a("label", name.empty() ? "Note " + std::to_string(nid) : name).a("form", "3D");
        if (!entities.empty()) note.ids("EntityInternalIds", entities);
        note.c("Text", text);
        notes_.push_back(std::move(note));
        noteIds_.push_back(nid);
        ++st.notes;
        return nid;
    }

    long AnnotationView(const Vec3& n, const Vec3& d)
    {
        auto r = [](double v) { return std::to_string(std::llround(v * 1e9)); };
        const std::string key = r(n.x) + "," + r(n.y) + "," + r(n.z) + "|" + r(d.x) + "," + r(d.y) + "," + r(d.z);
        if (auto it = annotationViewByKey_.find(key); it != annotationViewByKey_.end()) return it->second;
        const long aid = id();
        annotationViews_.push_back(E("AnnotationView").a("id", aid).c(E("Normal").raw(qo::vec(n))).c(E("Direction").raw(qo::vec(d))));
        ++st.annotationViews;
        return annotationViewByKey_[key] = aid;
    }

    void Display(const OwnedPmi& o, long refId)
    {
        const PmiItem& p = o.item;
        std::vector<std::vector<Vec3>> lines;
        std::vector<Vec3> tris;
        for (const auto& pl : p.polylines) { std::vector<Vec3> w; for (const Vec3& q : pl) w.push_back(o.W.point(q)); lines.push_back(std::move(w)); }
        for (const Vec3& q : p.triangles) tris.push_back(o.W.point(q));
        std::vector<Vec3> all;
        for (const auto& pl : lines) all.insert(all.end(), pl.begin(), pl.end());
        all.insert(all.end(), tris.begin(), tris.end());
        Vec3 c, n(0, 0, 1), x(1, 0, 0);
        if (all.empty())
        {
            // no drawing: a small mark at the occurrence origin, so the semantic content is still displayed
            ++st.emptyDrawings;
            c = o.W.point(Vec3());
            const double s = 0.01 * modelSize;
            lines.push_back({c - Vec3(s, 0, 0), c + Vec3(s, 0, 0)});
            lines.push_back({c - Vec3(0, s, 0), c + Vec3(0, s, 0)});
        }
        else
        {
            for (const Vec3& q : all) c = c + q;
            c = c * (1.0 / all.size());
            double cov[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
            for (const Vec3& q : all)
            {
                const double d[3] = {q.x - c.x, q.y - c.y, q.z - c.z};
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j) cov[i][j] += d[i] * d[j];
            }
            double vals[3];
            Vec3 vecs[3];
            qo::eigenSym3(cov, vals, vecs);
            if (vals[1] > 1e-20)   // planar or not, the best plane; degenerate (collinear) keeps the default frame
            {
                n = vecs[0].unit();
                x = vecs[2].unit();
            }
            else if (vals[2] > 1e-20)
            {
                x = vecs[2].unit();
                n = std::fabs(x.z) < 0.9 ? x.cross(Vec3(0, 0, 1)).unit() : x.cross(Vec3(1, 0, 0)).unit();
            }
            // prefer a normal pointing to +Z / +Y / +X so that the same plane is found for every markup on it
            if (n.z < -1e-9 || (std::fabs(n.z) <= 1e-9 && (n.y < -1e-9 || (std::fabs(n.y) <= 1e-9 && n.x < 0)))) { n = -n; x = -x; }
            for (const Vec3& q : all) st.worstPlanarity = std::max(st.worstPlanarity, std::fabs((q - c).dot(n)));
        }
        const Vec3 y = n.cross(x);
        auto P2 = [&](const Vec3& q) { const Vec3 d = q - c; return qo::vec2(d.dot(x), d.dot(y)); };
        const long avId = AnnotationView(n, x);
        E disp("PMIDisplay");
        if (p.color.set) disp.c(E("Color").raw(ColorText(p.color)));
        disp.c(E("Plane").ref("AnnotationViewId", avId).c(E("Origin").raw(qo::vec(c))).c(E("Direction").raw(qo::vec(x))));
        E g("Graphics");
        E pls("Polylines");
        size_t npl = 0;
        for (const auto& pl : lines)
        {
            if (pl.size() < 2) continue;
            std::string s;
            for (const Vec3& q : pl) { if (!s.empty()) s += ' '; s += P2(q); }
            pls.c(E("Polyline").c(E("Points").a("count", pl.size()).raw(s)));
            ++npl;
        }
        pls.a("n", npl);
        if (npl) g.c(std::move(pls));
        if (!tris.empty())
        {
            std::string v, ix;
            for (size_t i = 0; i < tris.size(); ++i) { if (i) { v += ' '; ix += ' '; } v += P2(tris[i]); ix += std::to_string(i); }
            g.c(E("Areas").a("n", 1).c(E("Area").c(E("Triangulation").c(E("Vertices").a("count", tris.size()).raw(v))
                                                       .c(E("Triangles").a("count", tris.size() / 3).raw(ix)))));
        }
        disp.c(std::move(g));
        disp.ref("Reference", refId);
        pmiDisplays_.push_back(std::move(disp));
        ++st.pmiDisplays;
    }

    static std::string Quaternion(const Vec3& xc, const Vec3& yc, const Vec3& zc)
    {
        // rotation matrix with columns xc, yc, zc -> (w, x, y, z), the order ../QIF_Reader reads
        const double m00 = xc.x, m01 = yc.x, m02 = zc.x, m10 = xc.y, m11 = yc.y, m12 = zc.y, m20 = xc.z, m21 = yc.z, m22 = zc.z;
        double w, x, y, z;
        const double tr = m00 + m11 + m22;
        if (tr > 0) { double s = std::sqrt(tr + 1.0) * 2; w = 0.25 * s; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s; }
        else if (m00 > m11 && m00 > m22) { double s = std::sqrt(1.0 + m00 - m11 - m22) * 2; w = (m21 - m12) / s; x = 0.25 * s; y = (m01 + m10) / s; z = (m02 + m20) / s; }
        else if (m11 > m22) { double s = std::sqrt(1.0 + m11 - m00 - m22) * 2; w = (m02 - m20) / s; x = (m01 + m10) / s; y = 0.25 * s; z = (m12 + m21) / s; }
        else { double s = std::sqrt(1.0 + m22 - m00 - m11) * 2; w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = 0.25 * s; }
        return qo::num(w) + " " + qo::num(x) + " " + qo::num(y) + " " + qo::num(z);
    }

    void BuildPmi()
    {
        // datums first, so that feature control frames find them by label
        std::vector<size_t> order;
        for (size_t i = 0; i < pmi_.size(); ++i) if (pmi_[i].item.semType == kA3DTypeMarkupDatum && !pmi_[i].item.target) order.push_back(i);
        for (size_t i = 0; i < pmi_.size(); ++i) if (!(pmi_[i].item.semType == kA3DTypeMarkupDatum && !pmi_[i].item.target)) order.push_back(i);
        for (size_t i : order)
        {
            const OwnedPmi& o = pmi_[i];
            const std::vector<long> entities = ResolveLinks(o);
            if (!entities.empty()) ++st.linkedPmi;
            const bool usesFeature = o.item.semType == kA3DTypeMarkupGDT || o.item.semType == kA3DTypeMarkupDimension || o.item.semType == kA3DTypeMarkupDatum;
            const long feature = usesFeature ? FeatureFor(o.item.name, entities) : 0;
            const long ref = Semantic(o, feature, entities);
            st.textFailures += o.item.textFailures;
            Display(o, ref);
            refsByAnnotation_[o.item.entity].push_back(ref);
        }
        for (const ViewRec& v : views_)
        {
            Vec3 zc = (v.loc - v.target).unit();
            if (zc.norm() == 0) continue;
            Vec3 yc = (v.up - zc * v.up.dot(zc)).unit();
            if (yc.norm() == 0) yc = std::fabs(zc.z) < 0.9 ? Vec3(0, 0, 1) : Vec3(0, 1, 0);
            yc = (yc - zc * yc.dot(zc)).unit();
            const Vec3 xc = yc.cross(zc);
            const double dist = (v.loc - v.target).norm();
            double height = v.ortho ? 2 * v.yfov : 2 * dist * std::tan(std::min(1.5, v.yfov));
            if (!(height > 0)) height = modelSize;
            const long cid = id();
            cameras_.push_back(E("Camera").a("id", cid).c(E("ViewPlaneOrigin").raw(qo::vec(v.target)))
                                   .c(E("Orientation").c(E("Value").raw(Quaternion(xc, yc, zc))))
                                   .c(E("Ratio").raw(qo::num(v.aspect)))
                                   .c(E("Near").raw(qo::num(v.znear)))
                                   .c(E("Far").raw(qo::num(v.zfar)))
                                   .c(E("Height").raw(qo::num(height))));
            ++st.cameras;
            std::vector<long> visible;
            std::set<long> seen;
            for (const void* a : v.annots)
                if (auto it = refsByAnnotation_.find(a); it != refsByAnnotation_.end())
                    for (long r : it->second) if (seen.insert(r).second) visible.push_back(r);
            const long sid = id();
            E sv("SavedView");
            sv.a("id", sid);
            if (!v.name.empty()) sv.a("label", v.name);
            sv.c("ActiveView", v.isDefault ? "true" : "false");
            if (!visible.empty()) sv.ids("AnnotationVisibleIds", visible);
            sv.ids("CameraIds", {cid});
            savedViews_.push_back(std::move(sv));
            viewIds_.push_back(sid);
            ++st.savedViews;
        }
    }
};
