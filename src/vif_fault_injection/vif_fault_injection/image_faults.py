"""Pure image-message transformations used by deterministic fault injection."""

from __future__ import annotations

import copy


def occluded_copy(message):
    """Return a black image without changing its timestamp or metadata."""
    occluded = copy.deepcopy(message)
    occluded.data = bytes(len(message.data))
    return occluded
