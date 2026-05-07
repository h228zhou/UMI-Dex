"""Tests for umi_dex.controllers.usart_decode."""

import pytest

from umi_dex.controllers.usart_decode import NUM_JOINTS, DecodedSample, UsartDecoder


def test_happy_path_returns_decoded_sample():
    decoder = UsartDecoder()
    raw = [100, 200, 300, 400, 500, 600]
    t = 1_500_000_000

    sample = decoder.feed_usart_frame(t, raw, valid_mask=0x3F)

    assert isinstance(sample, DecodedSample)
    assert sample.t_ros_ns == t
    assert sample.raw_counts == [100.0, 200.0, 300.0, 400.0, 500.0, 600.0]
    assert sample.valid_mask == 0x3F


def test_valid_mask_passthrough():
    decoder = UsartDecoder()
    raw = [0, 0, 0, 0, 0, 0]
    for mask in (0x00, 0x01, 0x15, 0x3F):
        sample = decoder.feed_usart_frame(0, raw, valid_mask=mask)
        assert sample.valid_mask == mask


def test_raw_counts_are_cast_to_float():
    decoder = UsartDecoder()
    sample = decoder.feed_usart_frame(0, [1, 2, 3, 4, 5, 6], 0x3F)
    assert all(isinstance(v, float) for v in sample.raw_counts)


def test_wrong_channel_count_raises():
    decoder = UsartDecoder()
    short = [1, 2, 3, 4, 5]
    long = [1, 2, 3, 4, 5, 6, 7]
    for bad in (short, long, []):
        with pytest.raises(ValueError):
            decoder.feed_usart_frame(0, bad, valid_mask=0x3F)


def test_num_joints_contract():
    assert NUM_JOINTS == 6
