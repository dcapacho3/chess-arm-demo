"""Read model poses from Gazebo (ground truth, used only to verify moves)."""
import re
import subprocess

_BLOCK = re.compile(r'pose \{\s*name: "([^"]+)"\s*id: \d+\s*position \{\s*x: ([-\d.e+]+)\s*y: ([-\d.e+]+)\s*z: ([-\d.e+]+)\s*\}'
                    r'\s*orientation \{\s*x: ([-\d.e+]+)\s*y: ([-\d.e+]+)\s*z: ([-\d.e+]+)\s*w: ([-\d.e+]+)')


def piece_poses(world='chess'):
    """{model name: (x, y, z, qx, qy, qz, qw)} for every piece_* model."""
    out = subprocess.check_output(['gz', 'topic', '-e', '-t', f'/world/{world}/pose/info', '-n', '1'],
                                  timeout=15).decode()
    return {m[0]: tuple(float(v) for v in m[1:]) for m in _BLOCK.findall(out) if m[0].startswith('piece_')}


def is_upright(p, tol=0.3):
    """True if the piece's z axis is within tol (rad-ish) of vertical."""
    qx, qy = p[3], p[4]
    return 2 * (qx * qx + qy * qy) < tol * tol / 2
