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

// prc_semantic.hpp - layer 5 of QIF_Reader: semantic PMI. The QIF characteristic / datum / note behind a PMIDisplay
// becomes a semantic PRC markup (A3DMarkupGDT with a feature control frame, A3DMarkupDimension, A3DMarkupDatum,
// A3DMarkupText); the tessellation, type and links of layer 4 are then set on it with A3DMkpMarkupSet.
// Included once by QIF_Reader.cpp (single translation unit).
//
// Grounded in: include/A3DSDKSemanticMarkupCreation.h (A3DMarkupGDTCreate, A3DMDFeatureControlFrameCreate,
//              A3DMDFCFDrawingRowCreate, A3DMDFCFRowDatumCreate, A3DMDFCFToleranceValueCreate, A3DMarkupDatumCreate,
//              A3DMarkupDimensionCreate, A3DMDDimensionValueCreate, A3DMarkupTextCreate, A3DMkpMarkupSet),
//              A3DSDKMarkupDefinition.h (semantic markups are A3DMkpMarkup entities of type kA3DTypeMarkup*),
//              docs guide/markup "Semantic PMI", QIF 3.0 Characteristics.xsd.
#pragma once

struct SemanticStats { std::map<std::string, long> created; std::map<std::string, long> failed; };
static bool g_semantic = true;          // --no-semantic turns it off
static bool g_debug = false;            // --debug
static SemanticStats g_semStats;

static double ChildDouble(const qifxml::Node* n, const char* c, double def = 0, bool* found = nullptr)
{
    const qifxml::Node* x = n ? n->child(c) : nullptr;
    if (found) *found = x != nullptr;
    return x ? std::strtod(x->text.c_str(), nullptr) : def;
}
static std::string Trim(const std::string& s)
{
    size_t b = s.find_first_not_of(" \t\r\n"), e = s.find_last_not_of(" \t\r\n");
    return b == std::string::npos ? std::string() : s.substr(b, e - b + 1);
}

static A3DEGDTType GdtType(const std::string& t)
{
    struct { const char* q; A3DEGDTType p; } map[] = {
        {"Straightness", kA3DGDTTypeStraightness}, {"Flatness", kA3DGDTTypeFlatness}, {"Circularity", kA3DGDTTypeCircularity},
        {"Cylindricity", kA3DGDTTypeCylindricity}, {"LineProfile", kA3DGDTTypeLineProfile}, {"SurfaceProfile", kA3DGDTTypeSurfaceProfile},
        {"Angularity", kA3DGDTTypeAngularity}, {"Perpendicularity", kA3DGDTTypePerpendicularity}, {"Parallelism", kA3DGDTTypeParallelism},
        {"Position", kA3DGDTTypePosition}, {"Concentricity", kA3DGDTTypeConcentricity}, {"Symmetry", kA3DGDTTypeSymmetry},
        {"CircularRunout", kA3DGDTTypeCircularRunout}, {"TotalRunout", kA3DGDTTypeTotalRunout}};
    for (auto& m : map) if (t.compare(0, strlen(m.q), m.q) == 0 && t.find("CharacteristicNominal") != std::string::npos) return m.p;
    return kA3DGDTTypeNone;
}

static A3DEMDGDTModifierType MaterialModifier(const std::string& mc)
{
    if (mc == "MAXIMUM") return kA3DGDTModifierMax;
    if (mc == "LEAST") return kA3DGDTModifierMin;
    if (mc == "REGARDLESS") return kA3DGDTModifierRfs;
    return kA3DGDTModifierNone;
}

// Datum label of a DatumDefinition id ("A"); compound datums are joined with '-'.
static std::string DatumLabel(const qif::Model& m, const qifxml::Node* datum)
{
    std::string label;
    std::function<void(const qifxml::Node*)> walk = [&](const qifxml::Node* n) {
        for (const auto& c : n->children)
        {
            if (c->name == "DatumDefinitionId")
            {
                auto d = m.byId.find(std::atol(c->text.c_str()));
                if (d != m.byId.end()) { if (!label.empty()) label += "-"; label += Trim(d->second->childText("DatumLabel")); }
            }
            else walk(c.get());
        }
    };
    walk(datum);
    return label;
}

// One FCF row datum per precedence (primary, secondary, tertiary) of a QIF DatumReferenceFrame.
static void DatumRefs(const qif::Model& m, long drfId, A3DMDFCFRowDatum* refs[3], std::string& text)
{
    refs[0] = refs[1] = refs[2] = nullptr;
    auto it = m.byId.find(drfId);
    if (it == m.byId.end()) return;
    const qifxml::Node* datums = it->second->child("Datums");
    if (!datums) return;
    int seq = 0;
    for (const auto& d : datums->children)
    {
        if (d->name != "Datum") continue;
        int slot = seq++;
        if (const auto* p = d->child("Precedence"))
        {
            std::string e = Trim(p->childText("PrecedenceEnum"));
            slot = e == "PRIMARY" ? 0 : e == "SECONDARY" ? 1 : e == "TERTIARY" ? 2 : slot;
        }
        if (const auto* s = d->child("SequenceNumber")) slot = std::atoi(s->text.c_str()) - 1;
        if (slot < 0 || slot > 2) continue;
        std::string label = DatumLabel(m, d.get());
        std::string mm;
        std::function<void(const qifxml::Node*)> findMM = [&](const qifxml::Node* n) {
            for (const auto& c : n->children) { if (c->name == "MaterialModifier") mm = Trim(c->text); else findMM(c.get()); }
        };
        findMM(d.get());
        A3DMDFCFRowDatumData rd = A3D_MAKE_DATA(A3DMDFCFRowDatumData);
        rd.m_pcDatum = const_cast<A3DUTF8Char*>(label.c_str());
        rd.m_eModifier = MaterialModifier(mm);
        rd.m_pNext = nullptr;
        rd.m_pLinkedItem = nullptr;
        A3DMDFCFRowDatum* r = nullptr;
        if (A3DMDFCFRowDatumCreate(&rd, &r) == A3D_SUCCESS) refs[slot] = r;
        text += " | " + label + (mm == "MAXIMUM" ? " (M)" : mm == "LEAST" ? " (L)" : "");
    }
}

// Characteristic nominal -> A3DMarkupGDT with one feature control frame row.
static A3DMkpMarkup* MakeGdt(const qif::Model& m, const qifxml::Node* nominal, A3DEGDTType type)
{
    auto di = m.byId.find(std::atol(nominal->childText("CharacteristicDefinitionId").c_str()));
    const qifxml::Node* def = di == m.byId.end() ? nullptr : di->second;
    if (!def) return nullptr;
    double tol = ChildDouble(def, "ToleranceValue");
    std::string mc = Trim(def->childText("MaterialCondition"));
    A3DMDFCFToleranceValueData tv = A3D_MAKE_DATA(A3DMDFCFToleranceValueData);
    std::string tolText = Trim(def->childText("ToleranceValue"));
    tv.m_sValue.m_dValue = tol;
    tv.m_sValue.m_pcValue = const_cast<A3DUTF8Char*>(tolText.c_str());
    tv.m_sValue.m_bIsValue = false;   // false: m_dValue is the value (true would mean a user text value)
    tv.m_eValueType = kA3DGDTValueNone;
    if (const auto* z = def->child("ZoneShape"))
    {
        if (z->child("DiametricalZone")) tv.m_eValueType = kA3DGDTValueDiameter;
        else if (z->child("SphericalZone")) tv.m_eValueType = kA3DGDTValueSpherical;
    }
    tv.m_eModifier = MaterialModifier(mc);
    tv.m_bFreeState = Trim(def->childText("FreeState")) == "true";
    tv.m_bStatistical = Trim(def->childText("StatisticalCharacteristic")) == "true";
    A3DMDFCFToleranceValue* ptv = nullptr;
    if (A3DMDFCFToleranceValueCreate(&tv, &ptv) != A3D_SUCCESS) return nullptr;

    std::string text = (tv.m_eValueType == kA3DGDTValueDiameter ? "\xE2\x8C\x80" : tv.m_eValueType == kA3DGDTValueSpherical ? "S\xE2\x8C\x80" : "") +
                       tolText + (mc == "MAXIMUM" ? " (M)" : mc == "LEAST" ? " (L)" : "");
    A3DMDFCFRowDatum* refs[3];
    std::string datumText;
    DatumRefs(m, std::atol(def->childText("DatumReferenceFrameId").c_str()), refs, datumText);

    A3DMDFCFDrawingRowData rd = A3D_MAKE_DATA(A3DMDFCFDrawingRowData);
    rd.m_eType = type;
    rd.m_pcValue = const_cast<A3DUTF8Char*>(text.c_str());
    rd.m_psSemanticValue = ptv;
    rd.m_psPrimaryRef = refs[0];
    rd.m_psSecondaryRef = refs[1];
    rd.m_psTertiaryRef = refs[2];
    A3DMDFCFDrawingRow* row = nullptr;
    if (A3DMDFCFDrawingRowCreate(&rd, &row) != A3D_SUCCESS) return nullptr;
    A3DMDFeatureControlFrameData fd = A3D_MAKE_DATA(A3DMDFeatureControlFrameData);
    A3DMDFCFRow* rows[1] = {(A3DMDFCFRow*)row};
    fd.m_uiNumberOfRows = 1;
    fd.m_ppRows = rows;
    fd.m_uiNumberOfIndicators = 0;
    fd.m_ppIndicators = nullptr;
    A3DMDFeatureControlFrame* fcf = nullptr;
    if (A3DMDFeatureControlFrameCreate(&fd, &fcf) != A3D_SUCCESS) return nullptr;
    A3DMarkupGDTData gd = A3D_MAKE_DATA(A3DMarkupGDTData);
    A3DMDTolerance* tols[1] = {(A3DMDTolerance*)fcf};
    gd.m_uiNumberOfMarkupTolerances = 1;
    gd.m_ppsMarkupTolerances = tols;
    gd.m_pTextProperties = nullptr;
    A3DMarkupGDT* gdt = nullptr;
    if (A3DMarkupGDTCreate(&gd, &gdt) != A3D_SUCCESS) return nullptr;
    return gdt;
}

// Characteristic nominal -> A3DMarkupDimension (nominal value + plus/minus tolerance).
static A3DMkpMarkup* MakeDimension(const qif::Model& m, const qifxml::Node* nominal, const std::string& t)
{
    auto di = m.byId.find(std::atol(nominal->childText("CharacteristicDefinitionId").c_str()));
    const qifxml::Node* def = di == m.byId.end() ? nullptr : di->second;
    bool hasValue = false;
    double value = ChildDouble(nominal, "TargetValue", 0, &hasValue);
    double up = 0, low = 0;
    bool hasTol = false;
    int decimals = -1;
    if (const auto* tv = nominal->child("TargetValue")) if (tv->attr("decimalPlaces")) decimals = std::atoi(tv->attr("decimalPlaces"));
    if (def)
        if (const auto* tol = def->child("Tolerance"))
        {
            up = ChildDouble(tol, "MaxValue", 0, &hasTol);
            low = ChildDouble(tol, "MinValue");
            if (Trim(tol->childText("DefinedAsLimit")) == "true") { up -= value; low -= value; }
        }
    A3DMDDimensionValueData vd = A3D_MAKE_DATA(A3DMDDimensionValueData);
    vd.m_bFormat = false;
    vd.m_dAccuracy = decimals >= 0 ? std::pow(10.0, -decimals) : 0.001;
    vd.m_dToleranceAccuracy = vd.m_dAccuracy;
    vd.m_dTolNumSup = up;
    vd.m_dTolNumInf = low;
    A3DMDDimensionValue* mv = nullptr;
    if (A3DMDDimensionValueCreate(&vd, &mv) != A3D_SUCCESS) return nullptr;

    A3DMarkupDimensionData dd = A3D_MAKE_DATA(A3DMarkupDimensionData);
    dd.m_eSymbol = kA3DDimensionSymbolTypeNONE;
    dd.m_eAdditionnalSymbol = kA3DDimensionSymbolTypeNONE;
    if (t.find("SphericalDiameter") == 0) { dd.m_eType = kA3DMDDimensionTypeDiameter; dd.m_eSymbol = kA3DDimensionSymbolTypeDIAMETER; }
    else if (t.find("Diameter") == 0) { dd.m_eType = kA3DMDDimensionTypeDiameter; dd.m_eSymbol = kA3DDimensionSymbolTypeDIAMETER; }
    else if (t.find("SphericalRadius") == 0 || t.find("Radius") == 0) dd.m_eType = kA3DMDDimensionTypeRadius;
    else if (t.find("Angle") == 0) dd.m_eType = kA3DMDDimensionTypeAngle;
    else if (t.find("Length") == 0) dd.m_eType = kA3DMDDimensionTypeLength;
    else if (t.find("Chamfer") == 0) dd.m_eType = kA3DMDDimensionTypeChamfer;
    else dd.m_eType = kA3DMDDimensionTypeDistance;   // DistanceBetween, DistanceFrom, Width, Height, Depth, Thickness
    dd.m_pMainValue = mv;
    dd.m_pDualValue = nullptr;
    dd.m_dValue = hasValue ? value : 0;
    std::string prefix = t.find("Spherical") == 0 ? "S" : "", suffix;
    for (const auto& a : qif::parseAttributes(nominal)) if (a.name == "_3dv.TextSuffix") suffix = a.value;
    dd.m_pcPrefixe = prefix.empty() ? nullptr : const_cast<A3DUTF8Char*>(prefix.c_str());
    dd.m_pcSuffixe = suffix.empty() ? nullptr : const_cast<A3DUTF8Char*>(suffix.c_str());
    dd.m_pExtentionLine = nullptr;
    dd.m_pLine = nullptr;
    dd.m_pTextProperties = nullptr;
    A3DMarkupDimension* dim = nullptr;
    if (A3DMarkupDimensionCreate(&dd, &dim) != A3D_SUCCESS) return nullptr;
    return dim;
}

static A3DMkpMarkup* MakeDatum(const qifxml::Node* n, bool target)
{
    std::string label = Trim(n->childText(target ? "DatumTargetLabel" : "DatumLabel"));
    if (label.empty()) return nullptr;
    A3DMarkupDatumData dd = A3D_MAKE_DATA(A3DMarkupDatumData);
    dd.m_pcUpText = const_cast<A3DUTF8Char*>(label.c_str());
    dd.m_pcDownText = nullptr;
    dd.m_bTarget = target;
    dd.m_bDiameter = false;
    dd.m_bIndividual = false;
    dd.m_pTextProperties = nullptr;
    A3DMarkupDatum* d = nullptr;
    if (A3DMarkupDatumCreate(&dd, &d) != A3D_SUCCESS) return nullptr;
    return d;
}

static A3DMkpMarkup* MakeNoteText(const std::vector<std::string>& lines, double charHeight, const Vec3& pos, const Vec3& base, const Vec3& up)
{
    if (lines.empty()) return nullptr;
    A3DMDTextPropertiesData tp = A3D_MAKE_DATA(A3DMDTextPropertiesData);
    tp.m_pcFont = const_cast<A3DUTF8Char*>("Arial");
    tp.m_dCharHeight = charHeight;
    tp.m_dCharWidth = charHeight * 0.6;
    tp.m_dCharSpacing = 0;
    tp.m_dAngle = 0;
    A3DMDTextProperties* props = nullptr;
    A3DMDTextPropertiesCreate(&tp, &props);
    A3DMDTextPositionData pd = A3D_MAKE_DATA(A3DMDTextPositionData);
    pd.m_sPosition = V3(pos);
    pd.m_sBaseVector = V3(base);
    pd.m_sUpVector = V3(up);
    A3DMDTextPosition* tpos = nullptr;
    A3DMDTextPositionCreate(&pd, &tpos);
    std::vector<A3DUTF8Char*> pl;
    for (const auto& l : lines) pl.push_back(const_cast<A3DUTF8Char*>(l.c_str()));
    A3DMarkupTextData td = A3D_MAKE_DATA(A3DMarkupTextData);
    td.m_uiLinesSize = (A3DUns32)pl.size();
    td.m_ppLines = pl.data();
    td.m_uiFileLinksSize = 0;
    td.m_ppFileLinks = nullptr;
    td.m_eFrameType = kA3DMarkupFrameNone;
    td.m_eLeaderAlignmentType = kA3DLeaderAlignment_Simple;
    td.m_pTextProperties = props;
    td.m_psTextPosition = tpos;
    A3DMarkupText* t = nullptr;
    if (A3DMarkupTextCreate(&td, &t) != A3D_SUCCESS) return nullptr;
    // HOOPS Exchange 2026.8 accepts the lines but A3DMarkupTextGet returns none afterwards, also after
    // A3DMarkupTextSet: the note text is therefore also stored as the markup attribute "QIF text" (and drawn in the
    // tessellation). Reported once per run.
    A3DMarkupTextData chk = A3D_MAKE_DATA(A3DMarkupTextData);
    A3DUns32 kept = 0;
    if (A3DMarkupTextGet(t, &chk) == A3D_SUCCESS) { kept = chk.m_uiLinesSize; A3DMarkupTextGet(nullptr, &chk); }
    if (kept != td.m_uiLinesSize) { A3DMarkupTextSet(&td, t); ++g_semStats.failed["note lines not kept by A3DMarkupTextCreate (text kept as attribute)"]; }
    return t;
}

// Semantic markup for the QIF entity behind a PMIDisplay, or nullptr (the caller then creates a plain tessellated markup).
struct NoteGeom { double height = 3.5; Vec3 pos, base{1, 0, 0}, up{0, 1, 0}; };

static A3DMkpMarkup* MakeSemanticMarkup(const qif::Model& m, long refId, const std::vector<std::string>& displayLines,
                                        const NoteGeom& ng, SemanticStats& ss, std::string& kind)
{
    auto it = m.byId.find(refId);
    if (it == m.byId.end()) return nullptr;
    const qifxml::Node* n = it->second;
    const std::string& t = n->name;
    A3DMkpMarkup* mk = nullptr;
    if (t == "Note") { kind = "note text"; mk = MakeNoteText(displayLines, ng.height, ng.pos, ng.base, ng.up); }
    else if (t == "DatumDefinition") { kind = "datum"; mk = MakeDatum(n, false); }
    else if (t == "DatumTarget") { kind = "datum target"; mk = MakeDatum(n, true); }
    else if (t.find("CharacteristicNominal") != std::string::npos)
    {
        A3DEGDTType g = GdtType(t);
        if (g != kA3DGDTTypeNone) { kind = "GD&T feature control frame"; mk = MakeGdt(m, n, g); }
        else { kind = "dimension"; mk = MakeDimension(m, n, t); }
    }
    else return nullptr;
    if (mk) ++ss.created[kind]; else ++ss.failed[kind + " (" + t + ")"];
    return mk;
}
