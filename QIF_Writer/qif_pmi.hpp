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

// qif_pmi.hpp - QIF_Writer: reads one PRC markup (annotation item) into a PmiItem: its drawn content (markup
// tessellation: polylines, filled triangles, text as glyph outlines), its semantic definition (GD&T feature control
// frame, dimension, datum, note) and the B-rep topology it is linked to. Included once by QIF_Writer.cpp.
//
// Grounded in: samples viewers/Viewer/traverse MarkupTessConnector.cpp (A3DTessMarkup code stream), A3DSDKMarkup.h
// (A3DGlobalFontTextTessellationGet, A3DMiscMarkupLinkedItem), A3DSDKMarkupTolerance.h, A3DSDKMarkupDimension.h,
// A3DSDKMarkupText.h, A3DSDKMisc.h (A3DMiscReferenceOnTopology); ../USD_Writer (same decoding, verified on CATIA,
// SolidWorks and PRC PMI).
#pragma once

struct PmiLink
{
    const A3DTopoBrepData* brep = nullptr;
    const A3DAsmProductOccurrence* target = nullptr;
    A3DEEntityType topoType = kA3DTypeUnknown;   // kA3DTypeTopoFace / Edge / CoEdge / ...
    std::vector<A3DUns32> indices;
};

struct PmiDatumRef { std::vector<std::string> labels; std::vector<int> modifiers; };   // compound: several labels

struct PmiItem
{
    const A3DEntity* entity = nullptr;     // annotation item (views refer to it)
    std::string name, label, text;
    Rgb color;
    A3DEEntityType semType = kA3DTypeUnknown;
    // GD&T
    int gdtType = 0, zone = 0, modifier = 0;
    double tolerance = 0;
    bool freeState = false, statistical = false, hasValue = false;
    PmiDatumRef datums[3];
    int extraRows = 0;
    // dimension
    int dimType = 0, symbol = 0;
    double value = 0, sup = 0, inf = 0, accuracy = 0;
    std::string prefix, suffix;
    // datum / note
    std::string datumLabel;
    bool target = false;
    std::vector<std::string> lines;
    // links and drawing (in the coordinates of the owning occurrence)
    std::vector<PmiLink> links;
    std::vector<std::vector<Vec3>> polylines;
    std::vector<Vec3> triangles;           // 3 points per triangle
    long textFailures = 0;
};

// ---- drawing --------------------------------------------------------------------------------------------------------
static void CollectRiItems(const A3DRiRepresentationItem* ri, std::vector<const A3DRiRepresentationItem*>& out)
{
    A3DEEntityType t = kA3DTypeUnknown;
    if (!ri || A3DEntityGetType(ri, &t) != A3D_SUCCESS) return;
    if (t == kA3DTypeRiSet)
    {
        A3DRiSetData sd = A3D_MAKE_DATA(A3DRiSetData);
        if (A3DRiSetGet(ri, &sd) != A3D_SUCCESS) return;
        for (A3DUns32 i = 0; i < sd.m_uiRepItemsSize; ++i) CollectRiItems(sd.m_ppRepItems[i], out);
        A3DRiSetGet(nullptr, &sd);
    }
    else out.push_back(ri);
}

// Wire tessellation of a glyph (A3DGlobalFontTextTessellationGet returns an RiSet of wire items).
static void GlyphStrokes(const A3DRiRepresentationItem* ri, std::vector<std::vector<Vec3>>& out)
{
    std::vector<const A3DRiRepresentationItem*> items;
    CollectRiItems(ri, items);
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
                auto P = [&](A3DUns32 i3) { return Vec3(bd.m_pdCoords[i3], bd.m_pdCoords[i3 + 1], bd.m_pdCoords[i3 + 2]); };
                if (wd.m_uiSizesWiresSize == 0)
                {
                    std::vector<Vec3> pl;
                    for (A3DUns32 k = 0; k < nPts; ++k) pl.push_back(P(3 * k));
                    if (pl.size() > 1) out.push_back(pl);
                }
                for (A3DUns32 i = 0; i < wd.m_uiSizesWiresSize;)
                {
                    const A3DUns32 head = wd.m_puiSizesWires[i];
                    const A3DUns32 n = head & ~(A3DUns32)(kA3DTess3DWireDataIsClosing | kA3DTess3DWireDataIsContinuous);
                    std::vector<Vec3> pl;
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

// Text drawn as glyph outlines along +X of the current markup matrix, scaled to the width stored in the tessellation.
static void TextStrokes(const A3DFontKeyData& font, const std::string& text, double width, double height, const Mat4& M, PmiItem& p)
{
    struct Glyph { std::vector<std::vector<Vec3>> strokes; double width = 0; bool ok = false; };
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
            if (set) { GlyphStrokes(set, g.strokes); A3DEntityDelete(set); }
        }
        glyphs.push_back(std::move(g));
    }
    double sum = 0;
    int n = 0;
    for (const Glyph& g : glyphs) if (g.ok && g.width > 0) { sum += g.width; ++n; }
    if (n == 0) { ++p.textFailures; return; }
    const double fallback = 0.6 * sum / n;
    std::vector<std::vector<Vec3>> strokes;
    double x = 0;
    for (Glyph& g : glyphs)
    {
        for (auto& pl : g.strokes)
        {
            for (Vec3& q : pl) q.x += x;
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
        for (const auto& pl : strokes) for (const Vec3& q : pl) ymax = std::max(ymax, q.y);
        if (ymax > 0) s = height / ymax;
    }
    for (auto& pl : strokes)
    {
        for (Vec3& q : pl) q = M.point(q * s);
        p.polylines.push_back(std::move(pl));
    }
}

// A3DTessMarkup code stream: polylines, polygons, triangles, matrix blocks, colour, font and text. Camera-dependent
// modes (face view, frame draw, fixed size) are drawn in model space at their anchor.
static void DecodeMarkupTess(const A3DTessMarkup* tess, PmiItem& p)
{
    if (!tess) return;
    A3DTessMarkupData md = A3D_MAKE_DATA(A3DTessMarkupData);
    A3DTessBaseData bd = A3D_MAKE_DATA(A3DTessBaseData);
    if (A3DTessMarkupGet(tess, &md) != A3D_SUCCESS) return;
    if (A3DTessBaseGet(tess, &bd) != A3D_SUCCESS) { A3DTessMarkupGet(nullptr, &md); return; }
    if (md.m_pcLabel && *md.m_pcLabel && p.label.empty()) p.label = md.m_pcLabel;
    for (A3DUns32 t = 0; t < md.m_uiTextsSize; ++t)
        if (md.m_ppcTexts[t] && *md.m_ppcTexts[t]) p.text += (p.text.empty() ? "" : "\n") + std::string(md.m_ppcTexts[t]);

    std::vector<Mat4> stack{Mat4()};
    std::vector<Mat4> modeStack;
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
        const Mat4 M = stack.back();
        auto P = [&](const double* q) { return M.point(Vec3(q[0], q[1], q[2])); };
        if (code & kA3DMarkupIsExtraData)
        {
            const A3DUns32 type = A3D_DECODE_EXTRA_DATA(code);
            if (type == A3D_DECODE_EXTRA_DATA(kA3DMarkupFaceViewMask) || type == A3D_DECODE_EXTRA_DATA(kA3DMarkupFrameDrawMask) ||
                type == A3D_DECODE_EXTRA_DATA(kA3DMarkupFixedSizeMask))
            {
                if (nd >= 3)
                {
                    Mat4 T;
                    T.m[3][0] = d[0]; T.m[3][1] = d[1]; T.m[3][2] = d[2];
                    modeStack.push_back(stack.back());
                    stack.push_back(T * stack.back());
                    d += 3;
                }
                else if (!modeStack.empty() && stack.size() > 1) { stack.pop_back(); modeStack.pop_back(); }
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
                std::vector<Vec3> pl;
                for (A3DUns32 k = 0; k + 3 <= nd; k += 3) pl.push_back(P(d + k));
                if (pl.size() > 2) pl.push_back(pl.front());
                if (pl.size() > 1) p.polylines.push_back(pl);
                break;
            }
            case A3D_DECODE_EXTRA_DATA(kA3DMarkupColorMask):
                if (cnt >= 1 && !p.color.set) p.color = ColorOfIndex(codes[i + 2]);
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
                    TextStrokes(font, md.m_ppcTexts[codes[i + 2]], d[0], d[1], M, p);
                break;
            default: break;   // points, cylinders, symbols, stipple, line width
            }
        }
        else if (code & kA3DMarkupIsMatrix)
        {
            if (nd >= 16)
            {
                Mat4 L;
                for (int r = 0; r < 4; ++r)
                    for (int c = 0; c < 4; ++c) L.m[r][c] = d[r * 4 + c];
                stack.push_back(L * stack.back());
                d += 16;
            }
            else if (stack.size() > 1) stack.pop_back();
            i += 2;
            continue;
        }
        else
        {
            std::vector<Vec3> pl;
            for (A3DUns32 k = 0; k + 3 <= nd; k += 3) pl.push_back(P(d + k));
            if (pl.size() > 1) p.polylines.push_back(pl);
        }
        d += nd;
        i += cnt + 2;
    }
    A3DTessBaseGet(nullptr, &bd);
    A3DTessMarkupGet(nullptr, &md);
}

// ---- links ----------------------------------------------------------------------------------------------------------
static void ReadLink(const A3DMiscMarkupLinkedItem* li, PmiItem& p)
{
    A3DMiscMarkupLinkedItemData ld = A3D_MAKE_DATA(A3DMiscMarkupLinkedItemData);
    if (!li || A3DMiscMarkupLinkedItemGet(li, &ld) != A3D_SUCCESS) return;
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
            PmiLink l;
            l.brep = rt.m_pBrepData;
            l.target = ld.m_pTargetProductOccurrence;
            l.topoType = rt.m_eTopoItemType;
            l.indices.assign(rt.m_puiAdditionalIndexes, rt.m_puiAdditionalIndexes + rt.m_uiSize);
            p.links.push_back(l);
            A3DMiscReferenceOnTopologyGet(nullptr, &rt);
        }
    }
    else if (t == kA3DTypeRiBrepModel)
    {
        A3DRiBrepModelData bd = A3D_MAKE_DATA(A3DRiBrepModelData);
        if (A3DRiBrepModelGet(ref, &bd) == A3D_SUCCESS)
        {
            PmiLink l;
            l.brep = bd.m_pBrepData;
            l.target = ld.m_pTargetProductOccurrence;
            l.topoType = kA3DTypeTopoBrepData;   // the whole body
            p.links.push_back(l);
            A3DRiBrepModelGet(nullptr, &bd);
        }
    }
    A3DMiscMarkupLinkedItemGet(nullptr, &ld);
}

// ---- semantic content -----------------------------------------------------------------------------------------------
static PmiDatumRef RowDatum(const A3DMDFCFRowDatum* rd)
{
    PmiDatumRef r;
    for (const A3DMDFCFRowDatum* x = rd; x;)
    {
        A3DMDFCFRowDatumData d = A3D_MAKE_DATA(A3DMDFCFRowDatumData);
        if (A3DMDFCFRowDatumGet(x, &d) != A3D_SUCCESS) break;
        std::string label = d.m_pcDatum ? d.m_pcDatum : "";
        // "A-B" written as one label is a compound datum too
        size_t start = 0;
        for (size_t k = 0; k <= label.size(); ++k)
            if (k == label.size() || label[k] == '-')
            {
                std::string one = label.substr(start, k - start);
                if (!one.empty()) { r.labels.push_back(one); r.modifiers.push_back((int)d.m_eModifier); }
                start = k + 1;
            }
        const A3DMDFCFRowDatum* next = d.m_pNext;
        A3DMDFCFRowDatumGet(nullptr, &d);
        x = next;
    }
    return r;
}

static double ParseLeadingNumber(const std::string& s, bool& ok)
{
    for (size_t i = 0; i < s.size(); ++i)
        if ((s[i] >= '0' && s[i] <= '9') || s[i] == '.')
        {
            char* end = nullptr;
            double v = std::strtod(s.c_str() + i, &end);
            ok = end != s.c_str() + i;
            return v;
        }
    ok = false;
    return 0;
}

static void ReadSemantic(const A3DMkpMarkup* mk, PmiItem& p)
{
    A3DEEntityType t = kA3DTypeUnknown;
    A3DEntityGetType(mk, &t);
    p.semType = t;
    if (t == kA3DTypeMarkupGDT)
    {
        A3DMarkupGDTData g = A3D_MAKE_DATA(A3DMarkupGDTData);
        if (A3DMarkupGDTGet(mk, &g) != A3D_SUCCESS) { p.semType = kA3DTypeUnknown; return; }
        bool first = true;
        for (A3DUns32 i = 0; i < g.m_uiNumberOfMarkupTolerances; ++i)
        {
            A3DEEntityType tt = kA3DTypeUnknown;
            A3DEntityGetType(g.m_ppsMarkupTolerances[i], &tt);
            if (tt != kA3DTypeMDFeatureControlFrame) continue;
            A3DMDFeatureControlFrameData f = A3D_MAKE_DATA(A3DMDFeatureControlFrameData);
            if (A3DMDFeatureControlFrameGet(g.m_ppsMarkupTolerances[i], &f) != A3D_SUCCESS) continue;
            for (A3DUns32 r = 0; r < f.m_uiNumberOfRows; ++r)
            {
                A3DEEntityType rt = kA3DTypeUnknown;
                A3DEntityGetType(f.m_ppRows[r], &rt);
                if (rt != kA3DTypeMDFCFDrawingRow) continue;
                if (!first) { ++p.extraRows; continue; }
                A3DMDFCFDrawingRowData d = A3D_MAKE_DATA(A3DMDFCFDrawingRowData);
                if (A3DMDFCFDrawingRowGet((const A3DMDFCFDrawingRow*)f.m_ppRows[r], &d) != A3D_SUCCESS) continue;
                first = false;
                p.gdtType = (int)d.m_eType;
                if (d.m_psSemanticValue)
                {
                    A3DMDFCFToleranceValueData v = A3D_MAKE_DATA(A3DMDFCFToleranceValueData);
                    if (A3DMDFCFToleranceValueGet(d.m_psSemanticValue, &v) == A3D_SUCCESS)
                    {
                        p.tolerance = v.m_sValue.m_dValue;
                        p.hasValue = true;
                        if (v.m_sValue.m_bIsValue && v.m_sValue.m_pcValue)
                        {
                            bool ok = false;
                            double x = ParseLeadingNumber(v.m_sValue.m_pcValue, ok);
                            if (ok) p.tolerance = x;
                        }
                        p.zone = (int)v.m_eValueType;
                        p.modifier = (int)v.m_eModifier;
                        p.freeState = v.m_bFreeState;
                        p.statistical = v.m_bStatistical;
                        A3DMDFCFToleranceValueGet(nullptr, &v);
                    }
                }
                if (!p.hasValue && d.m_pcValue)
                {
                    bool ok = false;
                    p.tolerance = ParseLeadingNumber(d.m_pcValue, ok);
                    p.hasValue = ok;
                }
                p.datums[0] = RowDatum(d.m_psPrimaryRef);
                p.datums[1] = RowDatum(d.m_psSecondaryRef);
                p.datums[2] = RowDatum(d.m_psTertiaryRef);
                A3DMDFCFDrawingRowGet(nullptr, &d);
            }
            A3DMDFeatureControlFrameGet(nullptr, &f);
        }
        A3DMarkupGDTGet(nullptr, &g);
        if (first) p.semType = kA3DTypeUnknown;   // no drawing row: written as a note
    }
    else if (t == kA3DTypeMarkupDimension)
    {
        A3DMarkupDimensionData d = A3D_MAKE_DATA(A3DMarkupDimensionData);
        if (A3DMarkupDimensionGet(mk, &d) != A3D_SUCCESS) { p.semType = kA3DTypeUnknown; return; }
        p.dimType = (int)d.m_eType;
        p.symbol = (int)d.m_eSymbol;
        p.value = d.m_dValue;
        if (d.m_pcPrefixe) p.prefix = d.m_pcPrefixe;
        if (d.m_pcSuffixe) p.suffix = d.m_pcSuffixe;
        if (d.m_pMainValue)
        {
            A3DMDDimensionValueData v = A3D_MAKE_DATA(A3DMDDimensionValueData);
            if (A3DMDDimensionValueGet(d.m_pMainValue, &v) == A3D_SUCCESS)
            {
                p.sup = v.m_dTolNumSup;
                p.inf = v.m_dTolNumInf;
                p.accuracy = v.m_dAccuracy;
                A3DMDDimensionValueGet(nullptr, &v);
            }
        }
        A3DMarkupDimensionGet(nullptr, &d);
    }
    else if (t == kA3DTypeMarkupDatum)
    {
        A3DMarkupDatumData d = A3D_MAKE_DATA(A3DMarkupDatumData);
        if (A3DMarkupDatumGet(mk, &d) != A3D_SUCCESS) { p.semType = kA3DTypeUnknown; return; }
        p.datumLabel = d.m_pcUpText ? d.m_pcUpText : "";
        p.target = d.m_bTarget;
        A3DMarkupDatumGet(nullptr, &d);
        if (p.datumLabel.empty()) p.semType = kA3DTypeUnknown;
    }
    else if (t == kA3DTypeMarkupText)
    {
        A3DMarkupTextData d = A3D_MAKE_DATA(A3DMarkupTextData);
        if (A3DMarkupTextGet(mk, &d) == A3D_SUCCESS)
        {
            for (A3DUns32 i = 0; i < d.m_uiLinesSize; ++i) if (d.m_ppLines[i]) p.lines.push_back(d.m_ppLines[i]);
            A3DMarkupTextGet(nullptr, &d);
        }
    }
}

// One annotation item -> PmiItem (false when it has no markup).
static bool ReadAnnotationItem(const A3DMkpAnnotationEntity* e, PmiItem& p)
{
    A3DMkpAnnotationItemData id = A3D_MAKE_DATA(A3DMkpAnnotationItemData);
    if (A3DMkpAnnotationItemGet(e, &id) != A3D_SUCCESS) return false;
    bool ok = false;
    if (id.m_pMarkup)
    {
        A3DMkpMarkupData md = A3D_MAKE_DATA(A3DMkpMarkupData);
        if (A3DMkpMarkupGet(id.m_pMarkup, &md) == A3D_SUCCESS)
        {
            ok = true;
            p.entity = e;
            p.name = EntityName(id.m_pMarkup);
            if (p.name.empty()) p.name = EntityName(e);
            p.color = OwnColor(id.m_pMarkup);
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
            for (A3DUns32 l = 0; l < md.m_uiLinkedItemsSize; ++l) ReadLink(md.m_ppLinkedItems[l], p);
            ReadSemantic(id.m_pMarkup, p);
            if (p.name.empty()) p.name = p.label;
            A3DMkpMarkupGet(nullptr, &md);
        }
    }
    A3DMkpAnnotationItemGet(nullptr, &id);
    return ok;
}

// ---- QIF names ------------------------------------------------------------------------------------------------------
static const char* QifGdtName(int t)
{
    switch (t)
    {
    case kA3DGDTTypeStraightness: return "Straightness";
    case kA3DGDTTypeFlatness: return "Flatness";
    case kA3DGDTTypeCircularity: return "Circularity";
    case kA3DGDTTypeCylindricity: return "Cylindricity";
    case kA3DGDTTypeLineProfile: return "LineProfile";
    case kA3DGDTTypeSurfaceProfile: return "SurfaceProfile";
    case kA3DGDTTypeAngularity: return "Angularity";
    case kA3DGDTTypePerpendicularity: return "Perpendicularity";
    case kA3DGDTTypeParallelism: return "Parallelism";
    case kA3DGDTTypePosition: return "Position";
    case kA3DGDTTypeConcentricity: return "Concentricity";
    case kA3DGDTTypeSymmetry: return "Symmetry";
    case kA3DGDTTypeCircularRunout: return "CircularRunout";
    case kA3DGDTTypeTotalRunout: return "TotalRunout";
    default: return nullptr;
    }
}

static std::string QifDimensionName(const PmiItem& p)
{
    const bool spherical = !p.prefix.empty() && (p.prefix[0] == 'S' || p.prefix[0] == 's');
    switch (p.dimType)
    {
    case kA3DMDDimensionTypeDiameter: case kA3DMDDimensionTypeDiameterTangent: case kA3DMDDimensionTypeDiameterCylinder:
    case kA3DMDDimensionTypeDiameterEdge: case kA3DMDDimensionTypeDiameterCone:
        return spherical ? "SphericalDiameter" : "Diameter";
    case kA3DMDDimensionTypeRadius: case kA3DMDDimensionTypeRadiusTangent: case kA3DMDDimensionTypeRadiusCylinder:
    case kA3DMDDimensionTypeRadiusEdge:
        return spherical ? "SphericalRadius" : "Radius";
    case kA3DMDDimensionTypeAngle: case kA3DMDDimensionTypeSlope: return "Angle";
    case kA3DMDDimensionTypeLength: case kA3DMDDimensionTypeLengthCurvilinear: return "Length";
    case kA3DMDDimensionTypeChamfer: return "Chamfer";
    default: return "DistanceBetween";   // distance, distance offset
    }
}

static const char* QifMaterialModifier(int m)
{
    switch (m)
    {
    case kA3DGDTModifierMax: return "MAXIMUM";
    case kA3DGDTModifierMin: return "LEAST";
    case kA3DGDTModifierRfs: return "REGARDLESS";
    default: return "NONE";
    }
}
