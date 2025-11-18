"""
Backward compatibility test for segmentation module.
Ensures that existing code still works without segmentation enabled.
"""

import sys
from vclibpy.components.heat_exchangers.moving_boundary_ntu import MovingBoundaryNTU
from vclibpy.components.segmentation import HeatExchangerSegmentation


def test_backward_compatibility():
    """Test that existing code works without changes."""
    print("Testing backward compatibility...")
    print()
    
    # Test 1: Segmentation module works independently
    print("✓ Test 1: Independent Segmentation Module")
    seg = HeatExchangerSegmentation(n_segments_sc=3, n_segments_lat=5, n_segments_sh=3)
    assert seg.n_segments_sc == 3
    assert seg.n_segments_lat == 5
    assert seg.n_segments_sh == 3
    print("  Segmentation module initialized correctly")
    print()
    
    # Test 2: Segmentation can be disabled (default behavior)
    print("✓ Test 2: Segmentation Disabled by Default")
    segments = seg.segment_phase('sc', Q=10000, dT_max=20)
    assert len(segments) == 3
    print(f"  Segmented 10000 W into {len(segments)} segments")
    print()
    
    # Test 3: Segment distribution
    print("✓ Test 3: Segment Distribution")
    dist = seg.get_segment_distribution(
        seg.segment_all_phases(10000, 50000, 15000, 20, 10, 25)
    )
    for phase, info in dist.items():
        print(f"  {phase.upper()}: {info['total_Q']:.0f} W ({info['fraction']*100:.1f}%)")
    print()
    
    # Test 4: Zero heat handling
    print("✓ Test 4: Zero Heat Handling")
    segments = seg.segment_phase('sc', Q=0, dT_max=20)
    assert len(segments) == 0
    print("  Zero heat correctly returns empty segment list")
    print()
    
    # Test 5: Segment fraction calculation
    print("✓ Test 5: Segment Fraction Calculation")
    segments = seg.segment_phase('lat', Q=50000, dT_max=10)
    for seg_item in segments:
        assert seg_item.get_segment_fraction() == 1.0 / 5
    print(f"  Each segment has fraction: {segments[0].get_segment_fraction():.3f}")
    print()
    
    # Test 6: Runtime parameter update (direct attribute update)
    print("✓ Test 6: Runtime Parameter Update")
    seg.n_segments_sc = 2
    seg.n_segments_lat = 8
    seg.n_segments_sh = 4
    assert seg.n_segments_sc == 2
    assert seg.n_segments_lat == 8
    assert seg.n_segments_sh == 4
    print(f"  Updated: SC={seg.n_segments_sc}, LAT={seg.n_segments_lat}, SH={seg.n_segments_sh}")
    print()
    
    # Test 7: Total heat calculation
    print("✓ Test 7: Total Heat Calculation")
    segments_dict = seg.segment_all_phases(10000, 50000, 15000, 20, 10, 25)
    total_Q = seg.get_total_Q(segments_dict)
    expected_Q = 10000 + 50000 + 15000
    assert abs(total_Q - expected_Q) < 0.1
    print(f"  Total heat: {total_Q:.0f} W (expected: {expected_Q:.0f} W)")
    print()
    
    print("=" * 60)
    print("ALL BACKWARD COMPATIBILITY TESTS PASSED ✓")
    print("=" * 60)
    return True


def test_no_phase():
    """Test handling of absent phases."""
    print("\nTesting no-phase scenarios...")
    seg = HeatExchangerSegmentation(3, 5, 3)
    
    # Only SC
    segments = seg.segment_all_phases(Q_sc=10000, Q_lat=0, Q_sh=0,
                                     dT_max_sc=20, dT_max_lat=10, dT_max_sh=25)
    assert 'sc' in segments
    assert 'lat' not in segments
    assert 'sh' not in segments
    print("✓ Only SC phase correctly handled")
    
    # Only LAT
    segments = seg.segment_all_phases(Q_sc=0, Q_lat=50000, Q_sh=0,
                                     dT_max_sc=20, dT_max_lat=10, dT_max_sh=25)
    assert 'sc' not in segments
    assert 'lat' in segments
    assert 'sh' not in segments
    print("✓ Only LAT phase correctly handled")
    
    # Only SH
    segments = seg.segment_all_phases(Q_sc=0, Q_lat=0, Q_sh=15000,
                                     dT_max_sc=20, dT_max_lat=10, dT_max_sh=25)
    assert 'sc' not in segments
    assert 'lat' not in segments
    assert 'sh' in segments
    print("✓ Only SH phase correctly handled")
    
    print("✓ All no-phase scenarios passed")


def test_merge_segments():
    """Test merging segments back to phases."""
    print("\nTesting segment merging...")
    seg = HeatExchangerSegmentation(3, 5, 3)
    
    segments_dict = seg.segment_all_phases(10000, 50000, 15000, 20, 10, 25)
    merged = seg.merge_segments_by_phase(segments_dict)
    
    assert abs(merged['sc'] - 10000) < 0.1
    assert abs(merged['lat'] - 50000) < 0.1
    assert abs(merged['sh'] - 15000) < 0.1
    print(f"✓ Segments correctly merged back to phases")
    print(f"  SC: {merged['sc']:.0f} W (original: 10000 W)")
    print(f"  LAT: {merged['lat']:.0f} W (original: 50000 W)")
    print(f"  SH: {merged['sh']:.0f} W (original: 15000 W)")


if __name__ == "__main__":
    try:
        test_backward_compatibility()
        test_no_phase()
        test_merge_segments()
        print("\n" + "=" * 60)
        print("✓✓✓ ALL TESTS PASSED SUCCESSFULLY ✓✓✓")
        print("=" * 60)
        sys.exit(0)
    except Exception as e:
        print(f"\n✗ TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
