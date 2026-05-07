#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Decode raw USART encoder frames into 6-channel count vectors.

Operates on deserialized bag messages (``UsartFrame``).  Unlike the CAN
path, USART frames are emitted pre-assembled by firmware — a single
message already carries all 6 raw counts and the validity mask — so this
decoder is a trivial passthrough that shares the :class:`DecodedSample`
contract with :mod:`.can_decode` for a uniform downstream API.
"""

from __future__ import annotations

from typing import Iterable

from .can_decode import JOINT_NAMES, NUM_JOINTS, DecodedSample

__all__ = ["UsartDecoder", "DecodedSample", "JOINT_NAMES", "NUM_JOINTS"]


class UsartDecoder:
    """Stateless assembler for ``UsartFrame`` messages.

    Feed individual messages via :meth:`feed_usart_frame`.  Every call
    returns a :class:`DecodedSample` (the frame is already complete on
    arrival) — in contrast to :class:`.can_decode.CanDecoder`, which
    buffers 3-part groups.
    """

    def feed_usart_frame(
        self,
        t_ros_ns: int,
        raw: Iterable[int],
        valid_mask: int,
    ) -> DecodedSample:
        counts = [float(x) for x in raw]
        if len(counts) != NUM_JOINTS:
            raise ValueError(
                f"UsartFrame expected {NUM_JOINTS} channels, got {len(counts)}"
            )
        return DecodedSample(
            t_ros_ns=int(t_ros_ns),
            raw_counts=counts,
            valid_mask=int(valid_mask),
        )
