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

// prc_pmi.hpp - layers 3 and 4 of QIF_Reader: graphical PMI (QIF VisualizationSet/PMIDisplaySet) as tessellated
// PRC markups linked to their faces/edges, and views (saved views with camera, annotation planes).
// Included once by QIF_Reader.cpp (single translation unit).
//
// Grounded in: samples/publishing/PublishPRCCubes (tessellation_wrapper/HXWMarkupTessellation.cpp: code stream for
//              colour, font, text, polyline, triangles, matrix; modules/CreateMarkups.cpp: markup + annotation item;
//              modules/CreateLinkedItem.cpp: linked item on topology; modules/CreateViews.cpp: views + camera),
//              A3DSDKEnums.h (kA3DMarkup*Mask, A3D_ENCODE_EXTRA_DATA), A3DSDKMisc.h (A3DMiscReferenceOnTopology indices).
#pragma once

// ---- markup tessellation code stream ------------------------------------------------------------------------------
class MarkupTess
{
public:
    void color(A3DUns32 colorIndex)
    {
        if (colorIndex == A3D_DEFAULT_COLOR_INDEX) return;
        codes_.push_back(A3D_ENCODE_EXTRA_DATA(kA3DMarkupColorMask, 1));
        codes_.push_back(0);
        codes_.push_back(colorIndex);
    }
    bool font(const char* family, A3DUns32 size)
    {
        A3DFontData fd = A3D_MAKE_DATA(A3DFontData);
        fd.m_pcFamilyName = const_cast<A3DUTF8Char*>(family);
        fd.m_eCharset = kA3DCharsetRoman;
        fd.m_uiSize = size;
        fd.m_cAttributes = 0;
        A3DFontKeyData fk = A3D_MAKE_DATA(A3DFontKeyData);
        if (A3DGlobalFontKeyCreate(&fd, &fk) != A3D_SUCCESS) return false;
        fontKey_ = fk;
        hasFont_ = true;
        A3DUns32 attrib = (A3DUns32)fk.m_cAttributes + ((A3DUns32)fk.m_iFontSizeIndex << 12) + ((A3DUns32)fk.m_iFontStyleIndex << 24);
        codes_.push_back(A3D_ENCODE_EXTRA_DATA(kA3DMarkupFontMask, 2));
        codes_.push_back(0);
        codes_.push_back((A3DUns32)fk.m_iFontFamilyIndex);
        codes_.push_back(attrib);
        return true;
    }
    void polyline(const std::vector<Vec3>& p)
    {
        if (p.size() < 2) return;
        codes_.push_back(0);
        codes_.push_back((A3DUns32)p.size() * 3);
        for (const auto& v : p) push(v);
    }
    void triangles(const std::vector<Vec3>& t)   // 3 points per triangle
    {
        if (t.size() < 3) return;
        size_t n = t.size() / 3 * 3;
        codes_.push_back(A3D_ENCODE_EXTRA_DATA(kA3DMarkupTrianglesMask, 0));
        codes_.push_back((A3DUns32)n * 3);
        for (size_t i = 0; i < n; ++i) push(t[i]);
    }
    // Text drawn in the plane (x, normal), baseline-left at 'pos', scaled to 'height' model units.
    // Returns the advance width in model units.
    double text(const std::string& s, const Vec3& pos, const Vec3& normal, const Vec3& x, double height)
    {
        if (s.empty() || !hasFont_) return 0;
        Vec3 y = normal.cross(x);
        double m[16] = {x.x, x.y, x.z, 0, y.x, y.y, y.z, 0, normal.x, normal.y, normal.z, 0, pos.x, pos.y, pos.z, 1};
        codes_.push_back(kA3DMarkupIsMatrix);
        codes_.push_back(16);
        size_t matrixAt = coords_.size();
        coords_.insert(coords_.end(), m, m + 16);
        texts_.push_back(s);
        codes_.push_back(A3D_ENCODE_EXTRA_DATA(kA3DMarkupTextMask, 1));
        codes_.push_back(2);                            // text width + text height follow in the coordinates
        codes_.push_back((A3DUns32)texts_.size() - 1);  // index in the text array
        double w = 0, h = 0;
        std::string tmp = s;
        if (A3DGlobalFontTextBoxGet(&fontKey_, &tmp[0], &w, &h) != A3D_SUCCESS || w <= 0 || h <= 0)
        { h = 1.0; w = 0.6 * (double)s.size(); }
        double k = height / h;   // uniform scale keeps the font's aspect ratio
        for (int i = 0; i < 3; ++i) { coords_[matrixAt + i] *= k; coords_[matrixAt + 4 + i] *= k; }
        coords_.push_back(w);
        coords_.push_back(h);
        codes_.push_back(kA3DMarkupIsMatrix);   // end of matrix mode
        codes_.push_back(0);
        return w * k;
    }
    bool empty() const { return codes_.empty(); }
    A3DTessMarkup* build(const std::string& label)
    {
        std::vector<A3DUTF8Char*> tp;
        for (auto& t : texts_) tp.push_back(const_cast<A3DUTF8Char*>(t.c_str()));
        A3DTessMarkupData d = A3D_MAKE_DATA(A3DTessMarkupData);
        d.m_uiCodesSize = (A3DUns32)codes_.size();
        d.m_puiCodes = codes_.data();
        d.m_uiTextsSize = (A3DUns32)tp.size();
        d.m_ppcTexts = tp.empty() ? nullptr : tp.data();
        d.m_pcLabel = const_cast<A3DUTF8Char*>(label.c_str());
        d.m_cBehaviour = 0;
        A3DTessMarkup* t = nullptr;
        if (A3DTessMarkupCreate(&d, &t) != A3D_SUCCESS) return nullptr;
        A3DTessBaseData bd = A3D_MAKE_DATA(A3DTessBaseData);
        bd.m_bIsCalculated = false;
        bd.m_uiCoordSize = (A3DUns32)coords_.size();
        bd.m_pdCoords = coords_.data();
        A3DTessBaseSet(t, &bd);
        return t;
    }

private:
    std::vector<A3DUns32> codes_;
    std::vector<double> coords_;
    std::vector<std::string> texts_;
    A3DFontKeyData fontKey_{};
    bool hasFont_ = false;
    void push(const Vec3& v) { coords_.push_back(v.x); coords_.push_back(v.y); coords_.push_back(v.z); }
};

// ---- QIF text tokens -> Unicode (ASME Y14.5 symbols) -------------------------------------------------------------------
static std::vector<std::string> QifTextLines(const std::string& in)
{
    static const std::map<std::string, std::string> sym = {
        {"DIAMETER", "\xE2\x8C\x80"}, {"DIAMETER_SPHERICAL", "S\xE2\x8C\x80"}, {"RADIUS_SPHERICAL", "SR"},
        {"POSITION", "\xE2\x8C\x96"}, {"PROFILE_SURFACE", "\xE2\x8C\x93"}, {"PROFILE_LINE", "\xE2\x8C\x92"},
        {"FLATNESS", "\xE2\x8F\xA5"}, {"STRAIGHTNESS", "\xE2\x8F\xA4"}, {"CIRCULARITY", "\xE2\x97\x8B"},
        {"CYLINDRICITY", "\xE2\x8C\xAD"}, {"PERPENDICULARITY", "\xE2\x8A\xA5"}, {"PARALLELISM", "\xE2\x88\xA5"},
        {"ANGULARITY", "\xE2\x88\xA0"}, {"CONCENTRICITY", "\xE2\x97\x8E"}, {"SYMMETRY", "\xE2\x8C\xAF"},
        {"RUNOUT_CIRCULAR", "\xE2\x86\x97"}, {"RUNOUT_TOTAL", "\xE2\x8C\xB0"}, {"PLUS_MINUS", "\xC2\xB1"},
        {"DEGREE", "\xC2\xB0"}, {"M", "\xE2\x93\x82"}, {"L", "\xE2\x93\x81"}, {"F", "\xE2\x93\x95"},
        {"P", "\xE2\x93\x85"}, {"T", "\xE2\x93\x89"}, {"U", "\xE2\x93\x8A"}, {"S", "\xE2\x93\x88"},
        {"ST", "\xE2\x9F\xA8ST\xE2\x9F\xA9"}, {"CF", "CF"}, {"COUNTERBORE", "\xE2\x8C\xB4"},
        {"COUNTERSINK", "\xE2\x8C\xB5"}, {"DEPTH", "\xE2\x86\xA7"}, {"SQUARE", "\xE2\x96\xA1"}};
    std::vector<std::string> lines(1);
    for (size_t i = 0; i < in.size();)
    {
        if (in[i] == '{')
        {
            size_t e = in.find('}', i);
            if (e != std::string::npos)
            {
                std::string tok = in.substr(i + 1, e - i - 1);
                if (tok == "BR") lines.emplace_back();
                else { auto it = sym.find(tok); lines.back() += it != sym.end() ? it->second : tok; }
                i = e + 1;
                continue;
            }
        }
        lines.back() += in[i++];
    }
    return lines;
}

// ---- QIF text tokens -> runs of plain text and vector-drawn Y14.5 symbols --------------------------------------------
// Viewer fonts (Arial etc.) have no glyphs for most GD&T characters (U+23E5, U+2225, U+2220, U+2316, ...): they show as
// empty boxes. Symbols are therefore drawn as polylines in the markup tessellation; plain text stays text.
struct TextRun { bool symbol = false; std::string s; };

static std::vector<std::vector<TextRun>> QifTextRuns(const std::string& in)
{
    static const std::map<std::string, std::string> asText = {{"PLUS_MINUS", "\xC2\xB1"}, {"DEGREE", "\xC2\xB0"},
                                                              {"RADIUS_SPHERICAL", "SR"}, {"CF", "CF"}};
    std::vector<std::vector<TextRun>> lines(1);
    auto addText = [&](const std::string& t) {
        auto& l = lines.back();
        if (!l.empty() && !l.back().symbol) l.back().s += t; else l.push_back({false, t});
    };
    for (size_t i = 0; i < in.size();)
    {
        if (in[i] == '{')
        {
            size_t e = in.find('}', i);
            if (e != std::string::npos)
            {
                std::string tok = in.substr(i + 1, e - i - 1);
                i = e + 1;
                if (tok == "BR") { lines.emplace_back(); continue; }
                if (auto a = asText.find(tok); a != asText.end()) { addText(a->second); continue; }
                if (tok == "DIAMETER_SPHERICAL") { addText("S"); lines.back().push_back({true, "DIAMETER"}); continue; }
                lines.back().push_back({true, tok});   // unknown tokens are drawn as text by DrawSymbol
                continue;
            }
        }
        addText(std::string(1, in[i++]));
    }
    return lines;
}

// Draws symbol 'tok' with its baseline-left corner at (x0, y0) of the PMI plane, cap height 'h'.
// Returns the advance width, or -1 when the token is not a known symbol.
template <class Frame>
static double DrawSymbol(MarkupTess& t, const Frame& f, const std::string& tok, double x0, double y0, double h)
{
    auto P = [&](double u, double v) { return f.at(x0 + u * h, y0 + v * h); };
    auto Ln = [&](std::initializer_list<std::pair<double, double>> pts) {
        std::vector<Vec3> p;
        for (auto& q : pts) p.push_back(P(q.first, q.second));
        t.polyline(p);
    };
    auto Arc = [&](double cu, double cv, double r, double a0, double a1, bool close = false) {
        std::vector<Vec3> p;
        const int N = 40;
        for (int i = 0; i <= N; ++i) { double a = a0 + (a1 - a0) * i / N; p.push_back(P(cu + r * std::cos(a), cv + r * std::sin(a))); }
        if (close) p.push_back(p.front());
        t.polyline(p);
    };
    auto ArrowHead = [&](double tu, double tv, double du, double dv) {   // filled head at (tu,tv), direction (du,dv)
        double n = std::sqrt(du * du + dv * dv); du /= n; dv /= n;
        double L = 0.3, W = 0.11;
        t.triangles({P(tu, tv), P(tu - du * L - dv * W, tv - dv * L + du * W), P(tu - du * L + dv * W, tv - dv * L - du * W)});
    };
    const double pi = qif::kPi;
    if (tok == "FLATNESS") { Ln({{0, 0.15}, {0.72, 0.15}, {1, 0.85}, {0.28, 0.85}, {0, 0.15}}); return 1.0; }
    if (tok == "STRAIGHTNESS") { Ln({{0, 0.5}, {1, 0.5}}); return 1.0; }
    if (tok == "CIRCULARITY") { Arc(0.5, 0.5, 0.48, 0, 2 * pi); return 1.0; }
    if (tok == "CYLINDRICITY") { Arc(0.5, 0.5, 0.36, 0, 2 * pi); Ln({{0.02, 0}, {0.42, 1}}); Ln({{0.58, 0}, {0.98, 1}}); return 1.0; }
    if (tok == "PERPENDICULARITY") { Ln({{0, 0}, {1, 0}}); Ln({{0.5, 0}, {0.5, 1}}); return 1.0; }
    if (tok == "PARALLELISM") { Ln({{0.1, 0}, {0.55, 1}}); Ln({{0.45, 0}, {0.9, 1}}); return 1.0; }
    if (tok == "ANGULARITY") { Ln({{1, 0}, {0, 0}, {0.9, 0.6}}); return 1.0; }
    if (tok == "POSITION") { Arc(0.5, 0.5, 0.33, 0, 2 * pi); Ln({{0.5, 0}, {0.5, 1}}); Ln({{0, 0.5}, {1, 0.5}}); return 1.0; }
    if (tok == "CONCENTRICITY") { Arc(0.5, 0.5, 0.48, 0, 2 * pi); Arc(0.5, 0.5, 0.24, 0, 2 * pi); return 1.0; }
    if (tok == "SYMMETRY") { Ln({{0, 0.5}, {1, 0.5}}); Ln({{0.2, 0.82}, {0.8, 0.82}}); Ln({{0.2, 0.18}, {0.8, 0.18}}); return 1.0; }
    if (tok == "PROFILE_LINE") { Arc(0.5, 0.15, 0.5, 0, pi); return 1.0; }
    if (tok == "PROFILE_SURFACE") { Arc(0.5, 0.15, 0.5, 0, pi); Ln({{0, 0.15}, {1, 0.15}}); return 1.0; }
    if (tok == "RUNOUT_CIRCULAR") { Ln({{0.15, 0}, {0.8, 0.9}}); ArrowHead(0.85, 0.97, 0.65, 0.9); return 1.0; }
    if (tok == "RUNOUT_TOTAL")
    {
        Ln({{0.05, 0}, {0.45, 0.85}}); ArrowHead(0.5, 0.95, 0.4, 0.85);
        Ln({{0.5, 0}, {0.9, 0.85}}); ArrowHead(0.95, 0.95, 0.4, 0.85);
        Ln({{0.05, 0}, {0.5, 0}});
        return 1.05;
    }
    if (tok == "DIAMETER") { Arc(0.45, 0.5, 0.38, 0, 2 * pi); Ln({{0.0, -0.05}, {0.9, 1.05}}); return 0.95; }
    if (tok == "SQUARE") { Ln({{0.1, 0}, {0.9, 0}, {0.9, 0.8}, {0.1, 0.8}, {0.1, 0}}); return 1.0; }
    if (tok == "COUNTERBORE") { Ln({{0.05, 0.9}, {0.05, 0.05}, {0.95, 0.05}, {0.95, 0.9}}); return 1.0; }
    if (tok == "COUNTERSINK") { Ln({{0, 0.9}, {0.5, 0.05}, {1, 0.9}}); return 1.0; }
    if (tok == "DEPTH") { Ln({{0.5, 1}, {0.5, 0.2}}); ArrowHead(0.5, 0.05, 0, -1); Ln({{0.15, 0}, {0.85, 0}}); return 1.0; }
    if (tok.size() == 1 && std::string("MLFPTUS").find(tok) != std::string::npos)
    {   // circled modifier letter
        Arc(0.55, 0.5, 0.55, 0, 2 * pi);
        double w = t.text(tok, P(0.3, 0.22), f.n, f.x, 0.62 * h);
        (void)w;
        return 1.1;
    }
    if (tok == "ST")
    {   // statistical tolerance: "ST" in an elongated hexagon
        Ln({{0, 0.5}, {0.25, 1.05}, {1.35, 1.05}, {1.6, 0.5}, {1.35, -0.05}, {0.25, -0.05}, {0, 0.5}});
        t.text("ST", P(0.3, 0.15), f.n, f.x, 0.7 * h);
        return 1.7;
    }
    return -1;
}

// Draws one line of runs; returns the end x.
template <class Frame>
static double DrawRuns(MarkupTess& t, const Frame& f, const std::vector<TextRun>& runs, double x, double y, double lh)
{
    const double capH = 0.9 * lh, gap = 0.12 * lh;
    for (size_t i = 0; i < runs.size(); ++i)
    {
        const TextRun& r = runs[i];
        if (r.symbol)
        {
            if (i > 0) x += gap;
            double adv = DrawSymbol(t, f, r.s, x, y, capH);
            if (adv < 0) { x += t.text(r.s, f.at(x, y), f.n, f.x, lh); continue; }
            x += adv * capH + gap;
        }
        else x += t.text(r.s, f.at(x, y), f.n, f.x, lh);
    }
    return x;
}

// ---- links from a QIF PMI entity to the converted topology ------------------------------------------------------------
// Characteristic -> FeatureNominalIds -> EntityInternalIds; Note / DatumTarget -> EntityInternalIds directly.
static void CollectTopology(const qif::Model& m, long id, std::set<long>& out, int depth = 0)
{
    if (depth > 6) return;
    auto it = m.byId.find(id);
    if (it == m.byId.end()) return;
    const qifxml::Node* n = it->second;
    if (n->name == "Face" || n->name == "Edge" || n->name == "Vertex") { out.insert(id); return; }
    std::function<void(const qifxml::Node*)> walk = [&](const qifxml::Node* c) {
        for (const auto& ch : c->children)
        {
            if (ch->name == "EntityInternalIds" || ch->name == "FeatureNominalIds" || ch->name == "FeatureNominalId" ||
                ch->name == "CharacteristicNominalIds" || ch->name == "DatumDefinitionId" || ch->name == "DatumTargetIds")
            {
                if (ch->child("Id")) { for (long x : qif::idList(ch.get())) CollectTopology(m, x, out, depth + 1); }
                else if (!ch->text.empty()) CollectTopology(m, std::atol(ch->text.c_str()), out, depth + 1);
            }
            else if (ch->name != "Attributes") walk(ch.get());
        }
    };
    walk(n);
}

static std::vector<A3DMiscMarkupLinkedItem*> MakeLinkedItems(const qif::Model& m, long refId, const BrepBuilder& b, long& unresolved)
{
    std::set<long> topo;
    CollectTopology(m, refId, topo);
    std::vector<A3DMiscMarkupLinkedItem*> items;
    for (long t : topo)
    {
        const TopoRef* r = nullptr;
        A3DEEntityType type = kA3DTypeTopoFace;
        if (auto f = b.faceRefs.find(t); f != b.faceRefs.end()) r = &f->second;
        else if (auto e = b.edgeRefs.find(t); e != b.edgeRefs.end()) { r = &e->second; type = kA3DTypeTopoEdge; }
        if (!r || !r->brep)
        {
            // wire-body edge (e.g. datum target lines): link to its PRC curve
            auto w = g_wireRefs.find(t);
            if (w == g_wireRefs.end()) { ++unresolved; continue; }
            A3DMiscMarkupLinkedItemData ld = A3D_MAKE_DATA(A3DMiscMarkupLinkedItemData);
            ld.m_pReference = w->second;
            ld.m_pTargetProductOccurrence = nullptr;
            A3DMiscMarkupLinkedItem* li = nullptr;
            if (A3DMiscMarkupLinkedItemCreate(&ld, &li) == A3D_SUCCESS) items.push_back(li); else ++unresolved;
            continue;
        }
        std::vector<A3DUns32> idx = r->idx;
        A3DMiscReferenceOnTopologyData td = A3D_MAKE_DATA(A3DMiscReferenceOnTopologyData);
        td.m_eTopoItemType = type;
        td.m_pBrepData = r->brep;
        td.m_uiSize = (A3DUns32)idx.size();
        td.m_puiAdditionalIndexes = idx.data();
        A3DMiscReferenceOnTopology* ref = nullptr;
        if (A3DMiscReferenceOnTopologyCreate(&td, &ref) != A3D_SUCCESS) { ++unresolved; continue; }
        A3DMiscMarkupLinkedItemData ld = A3D_MAKE_DATA(A3DMiscMarkupLinkedItemData);
        ld.m_pReference = ref;
        ld.m_pTargetProductOccurrence = nullptr;   // same product occurrence as the markup
        A3DMiscMarkupLinkedItem* li = nullptr;
        if (A3DMiscMarkupLinkedItemCreate(&ld, &li) == A3D_SUCCESS) items.push_back(li);
    }
    return items;
}

// QIF referenced entity -> PRC markup type / sub-type.
static void MarkupKind(const std::string& qifType, A3DEMarkupType& t, A3DEMarkupSubType& st)
{
    t = kA3DMarkupTypeOther; st = kA3DMarkupSubTypeUnknown;
    auto ends = [&](const char* s) { return qifType.find(s) != std::string::npos; };
    if (qifType == "Note") { t = kA3DMarkupTypeText; return; }
    if (qifType == "DatumDefinition") { t = kA3DMarkupTypeDatum; st = kA3DMarkupSubTypeDatumIdent; return; }
    if (qifType == "DatumTarget") { t = kA3DMarkupTypeDatum; st = kA3DMarkupSubTypeDatumTarget; return; }
    if (ends("SphericalDiameter") || ends("Diameter")) { t = kA3DMarkupTypeDimension; st = kA3DMarkupSubTypeDimensionDiameter; return; }
    if (ends("Radius")) { t = kA3DMarkupTypeDimension; st = kA3DMarkupSubTypeDimensionRadius; return; }
    if (ends("Angle") && !ends("Angularity")) { t = kA3DMarkupTypeDimension; st = kA3DMarkupSubTypeDimensionAngle; return; }
    if (ends("Length")) { t = kA3DMarkupTypeDimension; st = kA3DMarkupSubTypeDimensionLength; return; }
    if (ends("DistanceBetween") || ends("DistanceFrom") || ends("Width") || ends("Height") || ends("Depth") || ends("Thickness") ||
        ends("Chamfer"))
    { t = kA3DMarkupTypeDimension; st = kA3DMarkupSubTypeDimensionDistance; return; }
    if (ends("Position") || ends("Profile") || ends("Flatness") || ends("Straightness") || ends("Circularity") ||
        ends("Cylindricity") || ends("Perpendicularity") || ends("Parallelism") || ends("Angularity") || ends("Runout") ||
        ends("Concentricity") || ends("Symmetry"))
    { t = kA3DMarkupTypeGdt; st = kA3DMarkupSubTypeGdtFcf; return; }
}

// ---- one PMIDisplay -> one annotation item --------------------------------------------------------------------------
struct PmiFrame { Vec3 o, x, y, n; Vec3 at(double u, double v) const { return o + x * u + y * v; } };

static std::vector<double> D(const qifxml::Node* n) { return n ? qif::parseDoubles(n->text) : std::vector<double>(); }

static void DrawHead(MarkupTess& t, const PmiFrame& f, const std::string& form, double h, Vec3 tip2, Vec3 from2)
{
    Vec3 d = (tip2 - from2);
    if (d.norm() <= 0 || h <= 0) return;
    d = d.unit();
    Vec3 p(-d.y, d.x, 0);
    if (form == "ARROW_FILLED" || form == "ARROW_OPEN" || form == "ARROW")
    {
        Vec3 b1 = tip2 - d * h + p * (h / 3), b2 = tip2 - d * h - p * (h / 3);
        if (form == "ARROW_FILLED") t.triangles({f.at(tip2.x, tip2.y), f.at(b1.x, b1.y), f.at(b2.x, b2.y)});
        else { t.polyline({f.at(b1.x, b1.y), f.at(tip2.x, tip2.y), f.at(b2.x, b2.y)}); }
    }
    else if (form == "DOT_FILLED" || form == "DOT")
    {
        double r = h / 3;
        std::vector<Vec3> tri;
        const int N = 16;
        for (int i = 0; i < N; ++i)
        {
            double a0 = 2 * qif::kPi * i / N, a1 = 2 * qif::kPi * (i + 1) / N;
            tri.push_back(f.at(tip2.x, tip2.y));
            tri.push_back(f.at(tip2.x + r * std::cos(a0), tip2.y + r * std::sin(a0)));
            tri.push_back(f.at(tip2.x + r * std::cos(a1), tip2.y + r * std::sin(a1)));
        }
        t.triangles(tri);
    }
    else if (form == "TRIANGLE_FILLED" || form == "TRIANGLE")
    {
        // datum feature triangle: base on the feature at the tip, apex towards the leader
        Vec3 apex = tip2 - d * h, b1 = tip2 + p * (h / 1.732), b2 = tip2 - p * (h / 1.732);
        if (form == "TRIANGLE_FILLED") t.triangles({f.at(apex.x, apex.y), f.at(b1.x, b1.y), f.at(b2.x, b2.y)});
        else t.polyline({f.at(apex.x, apex.y), f.at(b1.x, b1.y), f.at(b2.x, b2.y), f.at(apex.x, apex.y)});
    }
}

struct PmiStats { long markups = 0, linked = 0, unresolved = 0, texts = 0, noFont = 0; };

static A3DMkpAnnotationItem* MakePmiDisplay(const qif::Model& m, const qifxml::Node* pd, const BrepBuilder& b, StyleCache& styles,
                                           long& refId, long& viewId, PmiStats& ps)
{
    const qifxml::Node* plane = pd->child("Plane");
    if (!plane) return nullptr;
    PmiFrame f;
    qif::parseVec(plane->child("Origin"), f.o);
    viewId = qif::refId(plane->child("AnnotationViewId"));
    Vec3 dir(1, 0, 0), nrm(0, 0, 1);
    if (auto v = m.byId.find(viewId); v != m.byId.end())
    { qif::parseVec(v->second->child("Normal"), nrm); qif::parseVec(v->second->child("Direction"), dir); }
    qif::parseVec(plane->child("Direction"), dir);   // optional per-display override
    f.n = nrm.unit();
    f.x = (dir - f.n * dir.dot(f.n)).unit();
    f.y = f.n.cross(f.x);

    MarkupTess t;
    qif::Color col;
    { auto c = D(pd->child("Color")); if (c.size() >= 3) { col.set = true; col.r = (int)c[0]; col.g = (int)c[1]; col.b = (int)c[2]; } }
    if (col.set) t.color(styles.colorIndex(col));
    auto P2 = [&](const qifxml::Node* n) { auto d = D(n); return d.size() >= 2 ? Vec3(d[0], d[1], 0) : Vec3(); };
    auto L = [&](Vec3 a, Vec3 c) { t.polyline({f.at(a.x, a.y), f.at(c.x, c.y)}); };

    // frames
    if (const auto* fr = pd->child("Frames"))
        for (const auto& c : fr->children)
        {
            Vec3 xy = P2(c->child("XY"));
            if (c->name == "FrameRectangular")
            {
                double w = std::strtod(c->childText("Width").c_str(), nullptr), h = std::strtod(c->childText("Height").c_str(), nullptr);
                t.polyline({f.at(xy.x, xy.y), f.at(xy.x + w, xy.y), f.at(xy.x + w, xy.y + h), f.at(xy.x, xy.y + h), f.at(xy.x, xy.y)});
            }
            else if (c->name == "FrameCircular")
            {
                double r = std::strtod(c->childText("Radius").c_str(), nullptr);
                std::vector<Vec3> circ;
                for (int i = 0; i <= 48; ++i) circ.push_back(f.at(xy.x + r * std::cos(2 * qif::kPi * i / 48), xy.y + r * std::sin(2 * qif::kPi * i / 48)));
                t.polyline(circ);
                if (c->attrBool("crossed")) { double k = r * 0.7071; L({xy.x - k, xy.y - k, 0}, {xy.x + k, xy.y + k, 0}); L({xy.x - k, xy.y + k, 0}, {xy.x + k, xy.y - k, 0}); }
            }
        }
    // leaders
    for (const auto& c : pd->children)
    {
        const std::string& n = c->name;
        if (n.compare(0, 6, "Leader") != 0) continue;
        Vec3 s = P2(c->child("StartPoint")), e = P2(c->child("EndPoint"));
        double hh = std::strtod(c->childText("HeadHeight").c_str(), nullptr);
        if (n == "LeaderDoubleHeadCircular")
        {
            Vec3 ctr = P2(c->child("Center"));
            double r = (s - ctr).norm(), a0 = std::atan2(s.y - ctr.y, s.x - ctr.x), a1 = std::atan2(e.y - ctr.y, e.x - ctr.x);
            double da = a1 - a0;
            while (da > qif::kPi) da -= 2 * qif::kPi;
            while (da < -qif::kPi) da += 2 * qif::kPi;   // minor arc
            std::vector<Vec3> arc;
            for (int i = 0; i <= 32; ++i) { double a = a0 + da * i / 32; arc.push_back(f.at(ctr.x + r * std::cos(a), ctr.y + r * std::sin(a))); }
            t.polyline(arc);
            Vec3 ts(-std::sin(a0), std::cos(a0), 0), te(-std::sin(a1), std::cos(a1), 0);
            double sg = da > 0 ? 1 : -1;
            DrawHead(t, f, c->childText("HeadForm"), hh, s, s + ts * sg);
            DrawHead(t, f, c->childText("HeadForm2"), hh, e, e - te * sg);
            continue;
        }
        L(s, e);
        if (c->child("PointExtension")) L(e, P2(c->child("PointExtension")));
        DrawHead(t, f, c->childText("HeadForm"), hh, s, e);
        if (n.find("DoubleHead") != std::string::npos) DrawHead(t, f, c->childText("HeadForm2"), hh, e, s);
    }
    // witness lines
    if (const auto* w = pd->child("WitnessLines"))
        for (const auto& sgm : w->children) L(P2(sgm->child("StartPoint")), P2(sgm->child("EndPoint")));
    // explicit graphics
    if (const auto* g = pd->child("Graphics"))
    {
        if (const auto* pls = g->child("Polylines"))
            for (const auto& pl : pls->children)
            {
                auto d = D(pl->child("Points"));
                std::vector<Vec3> pts;
                for (size_t i = 0; i + 1 < d.size(); i += 2) pts.push_back(f.at(d[i], d[i + 1]));
                t.polyline(pts);
            }
        if (const auto* as = g->child("Areas"))
            for (const auto& a : as->children)
                if (const auto* tr = a->child("Triangulation"))
                {
                    auto v = D(tr->child("Vertices"));
                    auto ix = D(tr->child("Triangles"));
                    std::vector<Vec3> tri;
                    for (size_t i = 0; i + 2 < ix.size(); i += 3)
                    {
                        size_t a0 = (size_t)ix[i], a1 = (size_t)ix[i + 1], a2 = (size_t)ix[i + 2];
                        if (a0 == a1 || a1 == a2 || a0 == a2 || 2 * std::max({a0, a1, a2}) + 1 >= v.size()) continue;
                        for (size_t k : {a0, a1, a2}) tri.push_back(f.at(v[2 * k], v[2 * k + 1]));
                    }
                    t.triangles(tri);
                }
    }
    // texts (after the lines so that they stay readable on top)
    std::vector<std::string> allLines;
    if (const auto* ts = pd->child("Texts"))
    {
        double lh = qif::attrDouble(ts, "lineHeight", 3.5);
        bool fontOk = t.font("Arial", 10);
        if (!fontOk) ++ps.noFont;
        for (const auto& tx : ts->children)
        {
            Vec3 xy = P2(tx->child("XY"));
            auto lines = QifTextLines(tx->childText("Data"));   // Unicode text for semantic PMI / attributes
            allLines.insert(allLines.end(), lines.begin(), lines.end());
            auto runs = QifTextRuns(tx->childText("Data"));       // drawn: text + vector symbols
            for (size_t i = 0; i < runs.size(); ++i)
            {
                if (runs[i].empty()) continue;
                DrawRuns(t, f, runs[i], xy.x, xy.y - lh * (double)i, lh);
                ++ps.texts;
            }
        }
    }
    if (t.empty()) return nullptr;

    refId = qif::refId(pd->child("Reference"));
    std::string qifType, label;
    if (auto r = m.byId.find(refId); r != m.byId.end())
    {
        qifType = r->second->name;
        if (r->second->attr("label")) label = r->second->attr("label");
        if (label.empty()) label = r->second->childText("Name");
        if (label.empty() && qifType == "DatumDefinition") label = "Datum " + r->second->childText("DatumLabel");
        if (label.empty() && qifType == "DatumTarget") label = "Datum target " + r->second->childText("DatumTargetLabel");
    }
    if (label.empty()) label = qifType + " " + std::to_string(refId);
    A3DTessMarkup* tess = t.build(label);
    if (!tess) return nullptr;

    long unresolved = 0;
    std::vector<A3DMiscMarkupLinkedItem*> links = MakeLinkedItems(m, refId, b, unresolved);
    ps.linked += (long)links.size();
    ps.unresolved += unresolved;

    A3DMkpMarkupData md = A3D_MAKE_DATA(A3DMkpMarkupData);
    MarkupKind(qifType, md.m_eType, md.m_eSubType);
    md.m_uiLinkedItemsSize = (A3DUns32)links.size();
    md.m_ppLinkedItems = links.empty() ? nullptr : links.data();
    md.m_pTessellation = tess;
    // Semantic definition first (layer 5); the tessellation, type and links are then set on it.
    std::string kind;
    NoteGeom ng;
    ng.base = f.x; ng.up = f.y; ng.pos = f.o;
    if (const auto* ts = pd->child("Texts"))
    {
        ng.height = qif::attrDouble(ts, "lineHeight", 3.5);
        if (!ts->children.empty()) { Vec3 xy = P2(ts->children[0]->child("XY")); ng.pos = f.at(xy.x, xy.y); }
    }
    A3DMkpMarkup* mk = g_semantic ? MakeSemanticMarkup(m, refId, allLines, ng, g_semStats, kind) : nullptr;
    if (mk && A3DMkpMarkupSet(mk, &md) != A3D_SUCCESS) { ++g_semStats.failed["A3DMkpMarkupSet on " + kind]; mk = nullptr; }
    if (!mk && A3DMkpMarkupCreate(&md, &mk) != A3D_SUCCESS) return nullptr;
    std::string joined;
    for (const auto& l : allLines) joined += (joined.empty() ? "" : "\n") + l;
    std::vector<qif::Attribute> mattrs = {{"QIF", "QIF type", qifType}};
    if (!joined.empty()) mattrs.push_back({"QIF", "QIF text", joined});
    SetNameAndAttributes(mk, label, mattrs, refId);
    if (col.set) SetGraphics(mk, styles.get(col), true);
    A3DMkpAnnotationItemData ad = A3D_MAKE_DATA(A3DMkpAnnotationItemData);
    ad.m_pMarkup = mk;
    A3DMkpAnnotationItem* ai = nullptr;
    if (A3DMkpAnnotationItemCreate(&ad, &ai) != A3D_SUCCESS) return nullptr;
    SetNameAndAttributes(ai, label, {}, refId);
    ++ps.markups;
    return ai;
}

// ---- views -----------------------------------------------------------------------------------------------------------
// QIF camera orientation quaternion is read as (w, x, y, z): with that order the NIST saved views come out as the expected
// isometric-style directions with +Z up. Camera = looking along -Zcam at ViewPlaneOrigin, Height = visible height.
static A3DGraphCamera* MakeCamera(const qifxml::Node* cam)
{
    Vec3 target;
    qif::parseVec(cam->child("ViewPlaneOrigin"), target);
    std::vector<double> q;
    if (const auto* o = cam->child("Orientation")) q = qif::parseDoubles(o->childText("Value"));
    if (q.size() < 4) return nullptr;
    double w = q[0], x = q[1], y = q[2], z = q[3];
    auto rot = [&](const Vec3& v) {
        return Vec3((1 - 2 * (y * y + z * z)) * v.x + 2 * (x * y - z * w) * v.y + 2 * (x * z + y * w) * v.z,
                    2 * (x * y + z * w) * v.x + (1 - 2 * (x * x + z * z)) * v.y + 2 * (y * z - x * w) * v.z,
                    2 * (x * z - y * w) * v.x + 2 * (y * z + x * w) * v.y + (1 - 2 * (x * x + y * y)) * v.z);
    };
    Vec3 zc = rot(Vec3(0, 0, 1)), up = rot(Vec3(0, 1, 0));
    double height = std::strtod(cam->childText("Height").c_str(), nullptr);
    double ratio = std::strtod(cam->childText("Ratio").c_str(), nullptr);
    if (height <= 0) height = 100;
    A3DGraphCameraData cd = A3D_MAKE_DATA(A3DGraphCameraData);
    cd.m_bOrthographic = true;
    cd.m_sLookAt = V3(target);
    cd.m_sLocation = V3(target + zc * (2 * height));
    cd.m_sUp = V3(up);
    cd.m_dYFovy = height / 2;   // orthographic: half of the visible height
    cd.m_dXFovy = height / 2 * (ratio > 0 ? ratio : 1);
    cd.m_dAspectRatio = ratio > 0 ? ratio : 1;
    cd.m_dZNear = 0;
    cd.m_dZFar = 4 * height;
    cd.m_dZoomFactor = 1;
    A3DGraphCamera* c = nullptr;
    return A3DGraphCameraCreate(&cd, &c) == A3D_SUCCESS ? c : nullptr;
}

static A3DMkpView* MakeView(const std::string& name, std::vector<A3DMkpAnnotationEntity*> annots, A3DGraphCamera* cam,
                            A3DSurfPlane* plane, bool isAnnotationView, bool isDefault)
{
    A3DMkpViewData vd = A3D_MAKE_DATA(A3DMkpViewData);
    vd.m_uiAnnotationsSize = (A3DUns32)annots.size();
    vd.m_ppAnnotations = annots.empty() ? nullptr : annots.data();
    vd.m_pPlane = plane;
    vd.m_bIsAnnotationView = isAnnotationView;
    vd.m_bIsDefaultView = isDefault;
    if (cam)
    {
        A3DGraphSceneDisplayParametersData sd = A3D_MAKE_DATA(A3DGraphSceneDisplayParametersData);
        sd.m_bIsActive = true;
        sd.m_pCamera = cam;
        sd.m_uiBackgroundStyleIndex = A3D_DEFAULT_STYLE_INDEX;
        sd.m_uiDefaultStyleIndex = A3D_DEFAULT_STYLE_INDEX;
        A3DGraphSceneDisplayParametersCreate(&sd, &vd.m_pSceneDisplayParameters);
    }
    A3DMkpView* v = nullptr;
    if (A3DMkpViewCreate(&vd, &v) != A3D_SUCCESS) return nullptr;
    SetNameAndAttributes(v, name, {});
    return v;
}

static A3DSurfPlane* MakeAnnotationPlane(const Vec3& n, const Vec3& dir)
{
    Vec3 z = n.unit(), x = (dir - z * dir.dot(z)).unit(), y = z.cross(x);
    A3DSurfPlaneData d = A3D_MAKE_DATA(A3DSurfPlaneData);
    SetTrsf(d.m_sTrsf, Vec3(0, 0, 0), x, y);
    d.m_sParam.m_dUCoeffA = 1; d.m_sParam.m_dVCoeffA = 1;
    d.m_sParam.m_sUVDomain.m_sMin.m_dX = -1e3; d.m_sParam.m_sUVDomain.m_sMax.m_dX = 1e3;
    d.m_sParam.m_sUVDomain.m_sMin.m_dY = -1e3; d.m_sParam.m_sUVDomain.m_sMax.m_dY = 1e3;
    A3DSurfPlane* p = nullptr;
    return A3DSurfPlaneCreate(&d, &p) == A3D_SUCCESS ? p : nullptr;
}

struct PmiResult { std::vector<A3DMkpAnnotationEntity*> annotations; std::vector<A3DMkpView*> views; PmiStats stats; long savedViews = 0, annotationViews = 0; };

static PmiResult BuildPmiAndViews(const qif::Model& m, const BrepBuilder& b, StyleCache& styles)
{
    PmiResult res;
    const qifxml::Node* prod = m.doc ? m.doc->child("Product") : nullptr;
    if (!prod) return res;
    std::map<long, std::vector<A3DMkpAnnotationEntity*>> byRef, byPlane;   // QIF reference id / annotation view id
    if (const auto* vs = prod->child("VisualizationSet"))
        if (const auto* set = vs->child("PMIDisplaySet"))
            for (const auto& pd : set->children)
            {
                if (pd->name != "PMIDisplay") continue;
                long refId = 0, viewId = 0;
                if (A3DMkpAnnotationItem* ai = MakePmiDisplay(m, pd.get(), b, styles, refId, viewId, res.stats))
                {
                    res.annotations.push_back(ai);
                    byRef[refId].push_back(ai);
                    byPlane[viewId].push_back(ai);
                }
            }
    const qifxml::Node* viewSet = prod->child("ViewSet");
    // annotation planes
    if (viewSet)
        if (const auto* avs = viewSet->child("AnnotationViewSet"))
            for (const auto& av : avs->children)
            {
                long id = qif::idOf(av.get());
                Vec3 n(0, 0, 1), d(1, 0, 0);
                qif::parseVec(av->child("Normal"), n);
                qif::parseVec(av->child("Direction"), d);
                auto annots = byPlane[id];
                if (annots.empty()) continue;
                std::string name = av->attr("label") ? av->attr("label") : "Annotation plane " + std::to_string(id);
                if (A3DMkpView* v = MakeView(name, annots, nullptr, MakeAnnotationPlane(n, d), true, false))
                { res.views.push_back(v); ++res.annotationViews; }
            }
    // saved views with camera and the annotations they show
    if (viewSet)
        if (const auto* svs = viewSet->child("SavedViewSet"))
            for (const auto& sv : svs->children)
            {
                std::vector<A3DMkpAnnotationEntity*> annots;
                for (long id : qif::idList(sv->child("AnnotationVisibleIds")))
                    for (auto* a : byRef[id]) annots.push_back(a);
                A3DGraphCamera* cam = nullptr;
                for (long cid : qif::idList(sv->child("CameraIds")))
                    if (auto c = m.byId.find(cid); c != m.byId.end()) { cam = MakeCamera(c->second); break; }
                bool active = sv->childText("ActiveView") == "true";
                std::string name = sv->attr("label") ? sv->attr("label") : "View " + std::to_string(qif::idOf(sv.get()));
                if (A3DMkpView* v = MakeView(name, annots, cam, nullptr, false, active)) { res.views.push_back(v); ++res.savedViews; }
            }
    return res;
}
