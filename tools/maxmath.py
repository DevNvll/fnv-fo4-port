"""Matrix and quaternion helpers with the conventions of the 3ds Max SDK.

A matrix is a numpy array of shape (4, 3): rows 0 to 2 are the axes and row 3 is
the translation. A point is a row vector: p' = p @ m[:3] + m[3]. The product
mul(a, b) is "a * b" of the SDK: a is applied first, then b. A child world
matrix is mul(local, parent).

A quaternion is (x, y, z, w) as in the SDK class Quat. The SDK uses the
left-hand rule for a Quat and an AngAxis: quat_to_mat(q) is the matrix of
Quat::MakeMatrix(mat, false). The functions rotx, roty and rotz are the SDK
functions RotateXMatrix, RotateYMatrix and RotateZMatrix (right-hand rule).
"""
import math
import numpy as np

TICKS_PER_SEC = 4800


def ident():
    m = np.zeros((4, 3))
    m[0, 0] = m[1, 1] = m[2, 2] = 1.0
    return m


def mul(a, b):
    r = np.empty((4, 3))
    r[:3] = a[:3] @ b[:3]
    r[3] = a[3] @ b[:3] + b[3]
    return r


def inv(m):
    ri = np.linalg.inv(m[:3])
    r = np.empty((4, 3))
    r[:3] = ri
    r[3] = -(m[3] @ ri)
    return r


def ptrans(p, m):
    """Point3 * Matrix3."""
    return np.asarray(p, float) @ m[:3] + m[3]


def vtrans(v, m):
    """Matrix3::VectorTransform."""
    return np.asarray(v, float) @ m[:3]


def trans_mat(p):
    m = ident()
    m[3] = p
    return m


def scale_mat(s):
    m = ident()
    m[0, 0], m[1, 1], m[2, 2] = s
    return m


def rotx(a):
    c, s = math.cos(a), math.sin(a)
    m = ident()
    m[1] = (0, c, s)
    m[2] = (0, -s, c)
    return m


def roty(a):
    c, s = math.cos(a), math.sin(a)
    m = ident()
    m[0] = (c, 0, -s)
    m[2] = (s, 0, c)
    return m


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    m = ident()
    m[0] = (c, s, 0)
    m[1] = (-s, c, 0)
    return m


def pre_translate(m, p):
    """Matrix3::PreTranslate: the translation is in the space of m."""
    r = m.copy()
    r[3] = r[3] + np.asarray(p, float) @ m[:3]
    return r


def translate(m, p):
    """Matrix3::Translate: the translation is in the parent space."""
    r = m.copy()
    r[3] = r[3] + np.asarray(p, float)
    return r


def pre_scale(m, s):
    """Matrix3::PreScale."""
    r = m.copy()
    r[0] *= s[0]
    r[1] *= s[1]
    r[2] *= s[2]
    return r


def no_trans(m):
    r = m.copy()
    r[3] = 0.0
    return r


def no_rot(m):
    """Matrix3::NoRot: keep the translation and the scale of each row."""
    r = m.copy()
    for i in range(3):
        ln = np.linalg.norm(m[i])
        r[i] = 0.0
        r[i, i] = ln
    return r


def no_scale(m):
    r = m.copy()
    for i in range(3):
        ln = np.linalg.norm(r[i])
        if ln > 0:
            r[i] /= ln
    return r


def quat_norm(q):
    q = np.asarray(q, float)
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([0.0, 0.0, 0.0, 1.0])


def quat_to_mat(q):
    """Quat::MakeMatrix(mat, false)."""
    x, y, z, w = quat_norm(q)
    m = ident()
    m[0] = (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w))
    m[1] = (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w))
    m[2] = (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))
    return m


def mat_to_quat(m):
    """Quat(Matrix3): the inverse of quat_to_mat. The rows must have no scale."""
    r = no_scale(m)[:3]
    # r is the standard column-convention rotation matrix of q
    t = r[0, 0] + r[1, 1] + r[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return quat_norm((x, y, z, w))


def quat_hamilton(p, q):
    """Hamilton product p (x) q. quat_to_mat(p (x) q) == mul(quat_to_mat(p), quat_to_mat(q))."""
    px, py, pz, pw = p
    qx, qy, qz, qw = q
    return np.array([
        pw * qx + px * qw + py * qz - pz * qy,
        pw * qy - px * qz + py * qw + pz * qx,
        pw * qz + px * qy - py * qx + pz * qw,
        pw * qw - px * qx - py * qy - pz * qz,
    ])


def quat_inv(q):
    x, y, z, w = q
    return np.array([-x, -y, -z, w]) / (x * x + y * y + z * z + w * w)


def quat_slerp(p, q, t):
    """Slerp along the shorter arc."""
    p = quat_norm(p)
    q = quat_norm(q)
    d = float(np.dot(p, q))
    if d < 0:
        q = -q
        d = -d
    if d > 0.999999:
        return quat_norm(p + (q - p) * t)
    th = math.acos(d)
    return (p * math.sin((1 - t) * th) + q * math.sin(t * th)) / math.sin(th)


def pre_rotate(m, q):
    """PreRotateMatrix(mat, q): mat = q.mat * mat. The translation stays."""
    r = m.copy()
    r[:3] = quat_to_mat(q)[:3] @ m[:3]
    return r


def euler_to_mat(ang, order=(0, 1, 2)):
    """EulerToMatrix: mat.RotateX(ang[0]), then RotateY, then RotateZ for EULERTYPE_XYZ."""
    f = (rotx, roty, rotz)
    m = ident()
    for i in range(3):
        m = mul(m, f[order[i]](ang[i]))
    return m


def ticks(frame, fps=30):
    return int(round(frame * TICKS_PER_SEC / fps))
