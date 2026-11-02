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

// prc_appearance.hpp - layer 2 of QIF_Reader: colours, visibility, layers, names/attributes,
// wire bodies and coordinate systems. Included once by QIF_Reader.cpp (single translation unit).
//
// Grounded in: samples/publishing/PublishPRCCubes/modules/CreateGraphics.cpp (style / graphics creation),
//              docs "Managing Attribute Inheritance" (face colours through entity references on the product occurrence,
//              A3DMiscReferenceOnTopology), A3DSDKMisc.h (reference indices), A3DSDKGraphics.h, A3DSDKRepItems.h.
#pragma once

// ---- styles ---------------------------------------------------------------------------------------------------------
class StyleCache
{
public:
    // Global style (colour + line width) for a QIF colour; A3D_DEFAULT_STYLE_INDEX when no colour is set.
    A3DUns32 get(const qif::Color& c, double width = A3D_DEFAULT_LINE_WIDTH)
    {
        if (!c.set) return A3D_DEFAULT_STYLE_INDEX;
        auto key = std::make_tuple(c.r, c.g, c.b, (int)std::lround(width * 1000));
        auto it = styles_.find(key);
        if (it != styles_.end()) return it->second;
        A3DGraphRgbColorData rgb = A3D_MAKE_DATA(A3DGraphRgbColorData);
        rgb.m_dRed = c.r / 255.0; rgb.m_dGreen = c.g / 255.0; rgb.m_dBlue = c.b / 255.0;
        A3DUns32 colorIndex = A3D_DEFAULT_COLOR_INDEX;
        if (A3DGlobalInsertGraphRgbColor(&rgb, &colorIndex) != A3D_SUCCESS) return styles_[key] = A3D_DEFAULT_STYLE_INDEX;
        A3DGraphStyleData sd = A3D_MAKE_DATA(A3DGraphStyleData);
        sd.m_dWidth = width;
        sd.m_bVPicture = false;
        sd.m_uiLinePatternIndex = A3D_DEFAULT_LINEPATTERN_INDEX;
        sd.m_bMaterial = false;
        sd.m_uiRgbColorIndex = colorIndex;
        sd.m_bIsTransparencyDefined = false;
        sd.m_ucTransparency = 255;
        A3DUns32 styleIndex = A3D_DEFAULT_STYLE_INDEX;
        if (A3DGlobalInsertGraphStyle(&sd, &styleIndex) != A3D_SUCCESS) styleIndex = A3D_DEFAULT_STYLE_INDEX;
        return styles_[key] = styleIndex;
    }
    A3DUns32 colorIndex(const qif::Color& c)
    {
        A3DGraphRgbColorData rgb = A3D_MAKE_DATA(A3DGraphRgbColorData);
        rgb.m_dRed = c.r / 255.0; rgb.m_dGreen = c.g / 255.0; rgb.m_dBlue = c.b / 255.0;
        A3DUns32 idx = A3D_DEFAULT_COLOR_INDEX;
        A3DGlobalInsertGraphRgbColor(&rgb, &idx);
        return idx;
    }

private:
    std::map<std::tuple<int, int, int, int>, A3DUns32> styles_;
};

// Sets colour / visibility / layer on any A3DRootBaseWithGraphics entity.
static void SetGraphics(A3DEntity* e, A3DUns32 style, bool show, A3DUns16 layer = A3D_DEFAULT_LAYER)
{
    if (!e) return;
    A3DGraphicsData gd = A3D_MAKE_DATA(A3DGraphicsData);
    gd.m_uiLayerIndex = layer;
    gd.m_uiStyleIndex = style;
    gd.m_usBehaviour = show ? kA3DGraphicsShow : 0;
    if (style != A3D_DEFAULT_STYLE_INDEX) gd.m_usBehaviour |= kA3DGraphicsSonHeritColor;
    if (!show) gd.m_usBehaviour |= kA3DGraphicsSonHeritShow;
    A3DGraphics* g = nullptr;
    if (A3DGraphicsCreate(&gd, &g) != A3D_SUCCESS) return;
    A3DRootBaseWithGraphicsData rg = A3D_MAKE_DATA(A3DRootBaseWithGraphicsData);
    rg.m_pGraphics = g;
    A3DRootBaseWithGraphicsSet(e, &rg);
    A3DEntityDelete(g);   // Set copies the graphics (pattern of PublishPRCCubes::setEntityColor)
}

// ---- names, persistent ids and attributes ------------------------------------------------------------------------
// One A3DMiscAttribute per QIF attribute; numbers stay numbers where the PRC type allows it.
static void SetNameAndAttributes(A3DEntity* e, const std::string& name, const std::vector<qif::Attribute>& attrs, long qifId = 0)
{
    if (!e) return;
    std::vector<A3DMiscAttribute*> made;
    for (const auto& a : attrs)
    {
        A3DMiscSingleAttributeData single = A3D_MAKE_DATA(A3DMiscSingleAttributeData);
        std::string title = a.name.empty() ? a.type : a.name;
        single.m_bTitleIsInt = false;
        single.m_pcTitle = const_cast<A3DUTF8Char*>(title.c_str());
        single.m_eType = kA3DModellerAttributeTypeString;   // values are kept verbatim ("20", "4 -1", "true", ...)
        single.m_pcData = const_cast<A3DUTF8Char*>(a.value.c_str());
        A3DMiscAttributeData ad = A3D_MAKE_DATA(A3DMiscAttributeData);
        ad.m_bTitleIsInt = false;
        ad.m_pcTitle = single.m_pcTitle;
        ad.m_uiSize = 1;
        ad.m_asSingleAttributesData = &single;
        A3DMiscAttribute* pa = nullptr;
        if (A3DMiscAttributeCreate(&ad, &pa) == A3D_SUCCESS) made.push_back(pa);
    }
    A3DRootBaseData rb = A3D_MAKE_DATA(A3DRootBaseData);
    rb.m_pcName = name.empty() ? nullptr : const_cast<A3DUTF8Char*>(name.c_str());
    rb.m_uiSize = (A3DUns32)made.size();
    rb.m_ppAttributes = made.empty() ? nullptr : made.data();
    rb.m_uiPersistentId = (A3DUns32)qifId;   // QIF id kept as PRC persistent id (traceability)
    A3DRootBaseSet(e, &rb);
    for (auto* pa : made) A3DEntityDelete(pa);
}

// ---- wire bodies (sketch / construction curves) -------------------------------------------------------------------
// One A3DRiCurve per QIF edge, grouped in an A3DRiSet named after the body.
// QIF edge id -> PRC curve of a wire body (PMI linked to wire geometry points at these).
static std::map<long, A3DRiRepresentationItem*> g_wireRefs;

static A3DRiRepresentationItem* MakeWireBody(const qif::Model& m, const qif::Body& b, StyleCache& styles, Report& rep)
{
    std::vector<A3DRiRepresentationItem*> curves;
    for (long eid : b.edges)
    {
        auto e = m.edges.find(eid);
        if (e == m.edges.end()) continue;
        auto c = m.curves3d.find(e->second.curve);
        if (c == m.curves3d.end() || !c->second.ok()) { rep.skip("wire edge skipped: unsupported curve"); continue; }
        std::string why;
        A3DCrvBase* crv = MakeCurve(c->second, false, why);
        if (!crv) { rep.skip("wire edge skipped: " + why); continue; }
        A3DTopoWireEdgeData wd = A3D_MAKE_DATA(A3DTopoWireEdgeData);
        wd.m_p3dCurve = crv;
        wd.m_bHasTrimDomain = true;
        wd.m_sInterval.m_dMin = c->second.d0;
        wd.m_sInterval.m_dMax = c->second.d1;
        A3DTopoWireEdge* we = nullptr;
        if (A3DTopoWireEdgeCreate(&wd, &we) != A3D_SUCCESS) continue;
        A3DTopoSingleWireBodyData sd = A3D_MAKE_DATA(A3DTopoSingleWireBodyData);
        sd.m_pWireEdge = we;
        A3DTopoSingleWireBody* swb = nullptr;
        if (A3DTopoSingleWireBodyCreate(&sd, &swb) != A3D_SUCCESS) continue;
        A3DRiCurveData rd = A3D_MAKE_DATA(A3DRiCurveData);
        rd.m_pBody = swb;
        A3DRiCurve* ri = nullptr;
        if (A3DRiCurveCreate(&rd, &ri) != A3D_SUCCESS) continue;
        qif::Color col = e->second.color.set ? e->second.color : b.color;
        SetNameAndAttributes(ri, e->second.label == " " ? std::string() : e->second.label, {}, eid);
        SetGraphics(ri, styles.get(col), true);
        curves.push_back(ri);
        g_wireRefs[eid] = ri;
        ++rep.created["wire edge " + c->second.type];
    }
    if (curves.empty()) return nullptr;
    A3DRiSetData sd = A3D_MAKE_DATA(A3DRiSetData);
    sd.m_uiRepItemsSize = (A3DUns32)curves.size();
    sd.m_ppRepItems = curves.data();
    A3DRiSet* set = nullptr;
    if (A3DRiSetCreate(&sd, &set) != A3D_SUCCESS) return nullptr;
    return set;
}

// ---- coordinate systems ------------------------------------------------------------------------------------------
static A3DRiRepresentationItem* MakeCoordinateSystem(const qif::CoordSystem& k)
{
    A3DMiscCartesianTransformationData td = A3D_MAKE_DATA(A3DMiscCartesianTransformationData);
    SetTrsf(td, k.origin, k.x.unit(), k.y.unit());
    if (k.x.unit().cross(k.y.unit()).dot(k.z) < 0) td.m_ucBehaviour |= kA3DTransformationMirror;
    A3DMiscCartesianTransformation* t = nullptr;
    if (A3DMiscCartesianTransformationCreate(&td, &t) != A3D_SUCCESS) return nullptr;
    A3DRiCoordinateSystemData cd = A3D_MAKE_DATA(A3DRiCoordinateSystemData);
    cd.m_pTransformation = t;
    A3DRiCoordinateSystem* cs = nullptr;
    if (A3DRiCoordinateSystemCreate(&cd, &cs) != A3D_SUCCESS) return nullptr;
    return cs;
}

// ---- face colours: entity references on the product occurrence -------------------------------------------------------
static A3DMiscEntityReference* MakeFaceColorReference(A3DTopoBrepData* brep, A3DUns32 faceIndex, A3DUns32 style)
{
    A3DMiscReferenceOnTopologyData td = A3D_MAKE_DATA(A3DMiscReferenceOnTopologyData);
    td.m_eTopoItemType = kA3DTypeTopoFace;
    td.m_pBrepData = brep;
    td.m_uiSize = 1;
    td.m_puiAdditionalIndexes = &faceIndex;
    A3DMiscReferenceOnTopology* ref = nullptr;
    if (A3DMiscReferenceOnTopologyCreate(&td, &ref) != A3D_SUCCESS) return nullptr;
    A3DMiscEntityReferenceData ed = A3D_MAKE_DATA(A3DMiscEntityReferenceData);
    ed.m_pEntity = ref;
    ed.m_pCoordinateSystem = nullptr;
    A3DMiscEntityReference* er = nullptr;
    if (A3DMiscEntityReferenceCreate(&ed, &er) != A3D_SUCCESS) return nullptr;
    SetGraphics(er, style, true);
    return er;
}
