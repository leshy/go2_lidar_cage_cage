"""uv run --python 3.13 --with git+https://github.com/cyberchitta/cad-khana --with pytest pytest"""
import math
import struct
from pathlib import Path

import pytest
from build123d import Rot
from cad_khana.mechanism.hold import hold

import assembly as A

HERE = Path(__file__).parent


@pytest.fixture(scope="module")
def cage():
    return A.cage()


def test_khana_assertions():
    # one solid, clear of the spinning cage, >= 1.5 mm from it (see assembly.build_fit)
    failed = [f"{a.name}: {a.detail}" for a in hold(A.build_fit()).assertions if a.passed is False]
    assert not failed


def test_clear_of_floor(cage):
    # robot lying down: nothing reaches past the floor plane (rotated onto +Z)
    top = (Rot(0, -A.FLOOR_DEG, 0) * cage).bounding_box().max.Z
    assert top <= A.FLOOR_S - A.FLOOR_MARGIN + 0.1


def stl_volume(path):  # binary STL, signed tetrahedra
    data = path.read_bytes()
    n = struct.unpack_from("<I", data, 80)[0]
    vol = 0.0
    for i in range(n):
        a, b, c = (struct.unpack_from("<3f", data, 84 + 50 * i + 12 + 12 * k) for k in range(3))
        vol += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6
    return abs(vol)


def test_stl_matches_model(cage):
    # the committed print file was exported from the current model
    assert math.isclose(stl_volume(HERE / "go2-lidar-cage.stl"), cage.volume, rel_tol=0.005)
