# Independent expectation of the semantic PMI, computed directly from the QIF XML (not from the converter code):
# one line per semantic markup, in the format QIF_Reader --dump-pmi prints after re-importing the PRC.
import xml.etree.ElementTree as ET, sys, os, glob
GDT = {'Straightness':1,'Flatness':2,'Circularity':3,'Cylindricity':4,'LineProfile':5,'SurfaceProfile':6,'Angularity':7,
       'Perpendicularity':8,'Parallelism':9,'Position':10,'Concentricity':11,'Symmetry':12,'CircularRunout':13,'TotalRunout':14}
def g(x): return '%g' % x
for f in sys.argv[1:]:
    r = ET.parse(f).getroot(); ns = r.tag.split('}')[0] + '}'
    T = lambda e: e.tag.replace(ns, '')
    ids = {e.attrib['id']: e for e in r.iter() if 'id' in e.attrib}
    txt = lambda e, c: (e.find(ns + c).text or '').strip() if e is not None and e.find(ns + c) is not None else ''
    out = []
    for pd in r.iter(ns + 'PMIDisplay'):
        n = ids.get(pd.find(ns + 'Reference/' + ns + 'Id').text)
        if n is None: continue
        t = T(n)
        if t == 'DatumDefinition':
            out.append('PMI-DATUM Datum %s: %s target=0' % (txt(n, 'DatumLabel'), txt(n, 'DatumLabel')))
        elif t == 'DatumTarget':
            out.append('PMI-DATUM Datum target %s: %s target=1' % (txt(n, 'DatumTargetLabel'), txt(n, 'DatumTargetLabel')))
        elif t.endswith('CharacteristicNominal'):
            base = t[:-len('CharacteristicNominal')]
            d = ids.get(txt(n, 'CharacteristicDefinitionId'))
            name = txt(n, 'Name')
            if base in GDT:
                zone = 0
                zs = d.find(ns + 'ZoneShape')
                if zs is not None:
                    if zs.find(ns + 'DiametricalZone') is not None: zone = 1
                    elif zs.find(ns + 'SphericalZone') is not None: zone = 3
                mc = txt(d, 'MaterialCondition'); mod = {'MAXIMUM': 1, 'LEAST': 2, 'REGARDLESS': 5}.get(mc, 0)
                slots = {}
                drf = ids.get(txt(d, 'DatumReferenceFrameId'))
                if drf is not None and drf.find(ns + 'Datums') is not None:
                    for i, dt in enumerate(drf.find(ns + 'Datums').findall(ns + 'Datum')):
                        s = i
                        pe = dt.find(ns + 'Precedence/' + ns + 'PrecedenceEnum')
                        if pe is not None: s = {'PRIMARY': 0, 'SECONDARY': 1, 'TERTIARY': 2}.get(pe.text.strip(), s)
                        sq = dt.find(ns + 'SequenceNumber')
                        if sq is not None: s = int(sq.text) - 1
                        labels = [ids[x.text.strip()].find(ns + 'DatumLabel').text.strip() for x in dt.iter(ns + 'DatumDefinitionId')]
                        mm = [x.text.strip() for x in dt.iter(ns + 'MaterialModifier')]
                        m = {'MAXIMUM': 1, 'LEAST': 2, 'REGARDLESS': 5}.get(mm[-1] if mm else '', 0)
                        slots[s] = '-'.join(labels) + ('/%d' % m if m else '')
                datums = ','.join(slots[k] for k in sorted(slots) if 0 <= k <= 2)
                out.append('PMI-GDT %s: type=%d tol=%s zone=%d mod=%d datums=%s' % (name, GDT[base], g(float(txt(d, 'ToleranceValue') or 0)), zone, mod, datums))
            else:
                val = float(txt(n, 'TargetValue') or 0)
                up = lo = 0.0
                tol = d.find(ns + 'Tolerance') if d is not None else None
                if tol is not None:
                    up = float(txt(tol, 'MaxValue') or 0); lo = float(txt(tol, 'MinValue') or 0)
                    if txt(tol, 'DefinedAsLimit') == 'true': up -= val; lo -= val
                if base.startswith('SphericalDiameter') or base.startswith('Diameter'): ty, sym = 9, 27
                elif base.startswith('SphericalRadius') or base.startswith('Radius'): ty, sym = 5, 0
                elif base.startswith('Angle'): ty, sym = 4, 0
                elif base.startswith('Length'): ty, sym = 2, 0
                elif base.startswith('Chamfer'): ty, sym = 14, 0
                else: ty, sym = 0, 0
                out.append('PMI-DIM %s: type=%d value=%s tol=%s/%s symbol=%d' % (name, ty, g(val), g(up), g(lo), sym))
    name = os.path.splitext(os.path.basename(f))[0]
    open(os.path.join(os.path.dirname(__file__), 'testdata', 'expected_pmi', name + '.txt'), 'w').write('\n'.join(sorted(out)) + '\n')
    print(name, len(out))
