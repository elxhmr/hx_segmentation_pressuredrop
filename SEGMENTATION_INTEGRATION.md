# Segmentierung der Wärmeübertrager - Integrationsleitfaden

## Was wurde hinzugefügt?

### 1. Neue Datei: `vclibpy/components/segmentation.py`
Enthält die Segmentierungslogik für Wärmeübertrager-Phasen:
- **`HeatExchangerSegmentation`**: Hauptklasse für die Verwaltung von Phasensegmentierung
- **`PhaseSegment`**: Dataclass, die einzelne Segmente darstellt

**Standard-Konfiguration:**
- SC (Subcooling): 3 Segmente
- LAT (Latent/Two-Phase): 5 Segmente
- SH (Superheat): 3 Segmente

Alle Werte sind konfigurierbar!

### 2. Erweiterung: `vclibpy/components/heat_exchangers/moving_boundary_ntu.py`
Die `MovingBoundaryNTU` Klasse wurde erweitert um:
- Optional Segmentierung aktivieren
- Neue Methoden zur Verwaltung der Segmentierung
- Vollständige **Rückwärts-Kompatibilität**

### 3. Test und Beispiele
- **`tests/test_segmentation.py`**: Umfassende Tests
- **`examples/e10_segmentation_example.py`**: Praktische Beispiele
- **`docs/source/SEGMENTATION.md`**: Ausführliche Dokumentation

## Wichtig: Rückwärts-Kompatibilität ✓

Die Segmentierung ist **standardmäßig DEAKTIVIERT**. Das bedeutet:
- ✓ Bestehender Code funktioniert ohne Änderungen
- ✓ Keine Breaking Changes
- ✓ Segmentierung muss explizit aktiviert werden

## Verwendung

### Standard (kein Segmentierung)
```python
# Alles funktioniert wie vorher
hx = MovingBoundaryNTUCondenser(A=50, ...)
# use_segmentation ist standardmäßig False
```

### Mit Segmentierung
```python
# Aktiviere Segmentierung mit Standard-Parametern
hx = MovingBoundaryNTUCondenser(
    A=50,
    ...,
    use_segmentation=True  # ← Aktivieren
)

# Oder mit benutzerdefinierten Segmentzahlen
hx = MovingBoundaryNTUCondenser(
    A=50,
    ...,
    use_segmentation=True,
    n_segments_sc=2,      # SC in 2 Segmente
    n_segments_lat=8,     # LAT in 8 Segmente
    n_segments_sh=4       # SH in 4 Segmente
)
```

### Laufzeit-Anpassung
```python
# Parameter später ändern
hx.set_segmentation_params(
    n_segments_sc=3,
    n_segments_lat=10,
    n_segments_sh=3
)
```

## Struktur

```
vclibpy/
├── components/
│   ├── __init__.py                    (aktualisiert)
│   ├── segmentation.py                (NEU)
│   ├── heat_exchangers/
│   │   └── moving_boundary_ntu.py     (erweitert)
│   └── ...
└── ...

examples/
└── e10_segmentation_example.py        (NEU)

tests/
└── test_segmentation.py               (NEU)

docs/source/
└── SEGMENTATION.md                    (NEU)
```

## Zukunftspläne

Die Segmentierungsfunktionalität ist so konzipiert, dass sie später auch auf andere Komponenten übertragen werden kann:
- Rohrleitungen (Pipes)
- Andere Komponenten mit phasenwechsel

Die Basis-Struktur erlaubt einfache Erweiterung.

## Tests durchführen

```bash
# Segmentierungs-Tests
python tests/test_segmentation.py

# Beispiele
python examples/e10_segmentation_example.py
```

## Fragen?

Weitere Details siehe:
- `docs/source/SEGMENTATION.md` - Ausführliche Dokumentation
- `examples/e10_segmentation_example.py` - Code-Beispiele
- `vclibpy/components/segmentation.py` - Implementierung
