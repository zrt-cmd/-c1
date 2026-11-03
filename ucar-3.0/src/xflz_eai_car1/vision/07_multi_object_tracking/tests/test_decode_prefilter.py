"""decode_outputs_prefilter 与 decode_outputs 逐帧一致（等价性）+ 性能。"""
import numpy as np
from detect import decode_outputs, decode_outputs_prefilter, dfl_decode, box_process

def _synth_outputs(h=8, w=8, nc=80, seed=0):
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(3):
        box = rng.normal(0, 1, (1, 64, h, w)).astype(np.float32)
        cls = rng.uniform(0, 0.02, (1, nc, h, w)).astype(np.float32)
        cls[0, 2, 2, 2] = 0.9
        cls[0, 5, 4, 4] = 0.85
        sm = rng.uniform(0.5, 1.0, (1, 1, h, w)).astype(np.float32)
        out += [box, cls, sm]
    return out

def _eq(a, b):
    return (round(a.x1,2), round(a.y1,2), round(a.x2,2), round(a.y2,2),
            round(a.score,5), a.class_id) == \
           (round(b.x1,2), round(b.y1,2), round(b.x2,2), round(b.y2,2),
            round(b.score,5), b.class_id)

def test_prefilter_matches_reference():
    out = _synth_outputs()
    old = decode_outputs(out, 0.25, 0.45, 640)
    new = decode_outputs_prefilter(out, 0.25, 0.45, 640)
    assert len(old) == len(new)
    for a, b in zip(old, new):
        assert _eq(a, b)

def test_prefilter_empty_when_all_low():
    rng = np.random.default_rng(1)
    out = []
    for _ in range(3):
        box = rng.normal(0, 1, (1, 64, 8, 8)).astype(np.float32)
        cls = rng.uniform(0, 0.01, (1, 8, 8, 8)).astype(np.float32)
        sm = rng.uniform(0.1, 0.5, (1, 1, 8, 8)).astype(np.float32)
        out += [box, cls, sm]
    assert decode_outputs(out, 0.25, 0.45, 640) == []
    assert decode_outputs_prefilter(out, 0.25, 0.45, 640) == []
