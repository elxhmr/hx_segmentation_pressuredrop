# Heat Exchanger Segmentation Module

## Übersicht

Das `segmentation.py` Modul in `vclibpy/components/` ermöglicht die Aufteilung von Wärmeübertrager-Phasen in mehrere Rechensegmente für verbesserte Wärmeübergangsberechnungen.

## Funktionalität

### Standard-Segmentierung
- **SC (Subcooling)**: 3 Segmente (konfigurierbar)
- **LAT (Latent/Two-Phase)**: 5 Segmente (konfigurierbar)
- **SH (Superheat)**: 3 Segmente (konfigurierbar)

### Klassen

#### `PhaseSegment`
Dataclass, die ein einzelnes Segment innerhalb einer Phase darstellt:

```python
@dataclass
class PhaseSegment:
    phase_type: str          # 'sc', 'lat', 'sh'
    segment_index: int       # Index innerhalb der Phase (0-based)
    total_segments: int      # Gesamtanzahl Segmente dieser Phase
    Q: float                 # Wärmestrom in diesem Segment
    dT_max: float            # Maximale Temperaturdifferenz
    state_inlet: object = None
    state_outlet: object = None
```

#### `HeatExchangerSegmentation`
Verwaltet die Segmentierung von Wärmeübertrager-Phasen:

```python
# Instanziierung mit Standard-Werten
seg = HeatExchangerSegmentation()

# Oder mit benutzerdefinierten Segmentzahlen
seg = HeatExchangerSegmentation(n_segments_sc=2, n_segments_lat=8, n_segments_sh=4)

# Alle Phasen gleichzeitig segmentieren
segments_dict = seg.segment_all_phases(
    Q_sc=10000, Q_lat=50000, Q_sh=15000,
    dT_max_sc=20, dT_max_lat=10, dT_max_sh=25
)

# Ergebnis: {'sc': [PhaseSegment, ...], 'lat': [...], 'sh': [...]}
```

## Integration in MovingBoundaryNTU

### Aktivierung der Segmentierung

```python
from vclibpy.components.heat_exchangers.moving_boundary_ntu import MovingBoundaryNTUCondenser

# Wärmeübertrager mit Segmentierung erstellen
hx = MovingBoundaryNTUCondenser(
    A=50,
    ...,
    use_segmentation=True,           # Segmentierung aktivieren
    n_segments_sc=3,                 # SC in 3 Segmente
    n_segments_lat=5,                # LAT in 5 Segmente
    n_segments_sh=3                  # SH in 3 Segmente
)
```

### Laufzeit-Anpassung

```python
# Segmentierungsparameter zur Laufzeit aktualisieren
hx.set_segmentation_params(n_segments_sc=2, n_segments_lat=10, n_segments_sh=4)
```

### Rückwärts-Kompatibilität

**Wichtig**: Die Segmentierung ist standardmäßig DEAKTIVIERT (`use_segmentation=False`), um vollständige Rückwärts-Kompatibilität zu gewährleisten. Bestehender Code funktioniert ohne Änderungen!

## Beispiele

### Beispiel 1: Basis-Segmentierung

```python
from vclibpy.components.segmentation import HeatExchangerSegmentation

seg = HeatExchangerSegmentation(n_segments_sc=3, n_segments_lat=5, n_segments_sh=3)

segments = seg.segment_all_phases(
    Q_sc=10000, Q_lat=50000, Q_sh=15000,
    dT_max_sc=20, dT_max_lat=10, dT_max_sh=25
)

# Ausgabe:
# SC: 3 Segmente à 3333.3 W
# LAT: 5 Segmente à 10000 W
# SH: 3 Segmente à 5000 W
```

### Beispiel 2: Segmentverteilung analysieren

```python
distribution = seg.get_segment_distribution(segments)

for phase_type, info in distribution.items():
    print(f"{phase_type}: {info['total_Q']:.0f} W ({info['fraction']*100:.1f}%), {info['n_segments']} Segmente")
```

### Beispiel 3: Heatmaps für verschiedene Strategien

```python
# Feine Segmentierung (bessere Auflösung, mehr Rechenzeit)
seg_fine = HeatExchangerSegmentation(5, 10, 5)

# Grobe Segmentierung (schneller, weniger Auflösung)
seg_coarse = HeatExchangerSegmentation(2, 3, 2)

# Keine Segmentierung (Standard)
seg_none = HeatExchangerSegmentation(1, 1, 1)
```

## Vorteile der Segmentierung

1. **Bessere Genauigkeit**: Mehrere Segmente ermöglichen bessere Auflösung von nichtlinearen Effekten
2. **Flexible Kalibrierung**: Segment-Anzahl kann problemabhängig angepasst werden
3. **Modulare Architektur**: Segmentierung ist optional und unabhängig
4. **Rückwärts-kompatibel**: Bestehender Code funktioniert unverändert

## Architektur-Notizen

### Warum separate Datei?
- Die `segmentation.py` ist unabhängig und kann später auch auf andere Komponenten (z.B. Rohre) ausgeweitet werden
- Klare Trennung von Verantwortlichkeiten
- Leichte Testbarkeit

### Erweiterbarkeit für Rohre
In Zukunft kann das Modul erweitert werden:

```python
# Geplant für Zukunft
class PipeSegmentation(HeatExchangerSegmentation):
    """Segmentierung für Rohrleitungen."""
    pass
```

## Hinweise zur Verwendung

1. **Standardverhalten**: Ohne Aktivierung verhält sich alles wie vorher
2. **Schrittweise Einführung**: Segmentierung kann schrittweise für verschiedene HX-Typen aktiviert werden
3. **Performance**: Mit vielen Segmenten (z.B. 10+) kann die Rechenzeit ansteigen
4. **Debugging**: Der FlowsheetState kann zur Speicherung von Segment-Informationen erweitert werden

## Testing

Siehe `examples/e10_segmentation_example.py` für vollständige Beispiele.

```bash
python examples/e10_segmentation_example.py
```
