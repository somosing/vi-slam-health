from types import SimpleNamespace

from vif_fault_injection.image_faults import occluded_copy


def test_occlusion_preserves_metadata_and_zeros_pixels():
    source = SimpleNamespace(
        header=SimpleNamespace(stamp=123, frame_id="camera_left_optical_frame"),
        height=2,
        width=3,
        encoding="mono8",
        step=3,
        data=bytes([1, 2, 3, 4, 5, 6]),
    )

    result = occluded_copy(source)

    assert result is not source
    assert result.header is not source.header
    assert result.header.stamp == source.header.stamp
    assert result.header.frame_id == source.header.frame_id
    assert result.height == source.height
    assert result.width == source.width
    assert result.encoding == source.encoding
    assert result.step == source.step
    assert result.data == bytes(6)
    assert source.data == bytes([1, 2, 3, 4, 5, 6])
