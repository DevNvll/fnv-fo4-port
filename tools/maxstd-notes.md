# Notes for maxstd.py

These notes give the chunk formats that `maxstd.py` reads and the proof for each rule.
The data is from 47 scene files of 3ds Max 2014 (`ole/index.txt`) and from `ole/fo4rig`.
`python3 tools/maxstd_check.py` runs each check again (67 seconds).

All numbers are little-endian. A time is in ticks (4800 for each second). A float has 32 bits.

## Time configuration (Config stream)

Chunk `0x20b0` is a container.

| Chunk | Type | Content |
| --- | --- | --- |
| `0x0010` | int | Frames for each second (30 in each scene) |
| `0x0050` | int | Start of the animation range, ticks (0 in each scene) |
| `0x0060` | int | End of the animation range, ticks |
| `0x0070` | int | Time of the time slider at the save, ticks |
| `0x0020`, `0x0030`, `0x0040`, `0x0110`, `0x0120` | int | Not decoded (0, 1, 1, 0, 1 in each scene) |

Proof for `0x0070`: each animated Bezier Float keeps its last computed value with a validity
interval (t, t). That t is equal to `0x0070` in each scene.
`0x0050` and `0x0060` have no second source in the file. The names are from the values (0 and
the last frame of the animation).

## Keyframe controllers

Bezier Float, Bezier Point3 and Bezier Color have their data in the container `0x7127`.
Bezier Position, Bezier Scale and Linear Rotation have the same chunks directly in the object.

| Chunk | Size | Content |
| --- | --- | --- |
| `0x2501` | 4 or 12 | Cached value of a float or Point3 controller (`curval`) |
| `0x2503` | 12 | Cached value of Bezier Position |
| `0x2504` | 16 | Cached value of Linear Rotation, Quat (x, y, z, w) |
| `0x2505` | 28 | Cached value of Bezier Scale, ScaleValue (Point3 s, Quat q) |
| `0x2500` | 8 | Validity interval of the cached value (start, end) |
| `0x3002` | 4 | Track flags (`TFLAG_*` of istdplug.h). The scenes have 0, 1, 4 and 5 |
| `0x3003` | 8 | Time range of the keys (first key, last key). (0x80000000, 0x80000000) with no key |
| `0x2525` | 28 for each key | Keys of Bezier Float |
| `0x2528` | 148 for each key | Keys of Bezier Scale |
| `0x2532`, `0x2533`, `0x2534` | container | Each has `0x2700`: a bit count (int) and one bit for each key. Not used |
| `0x3005` | 4 | 0x186 in each controller. Not decoded |
| `0x2535` | 4 | Lock flag of the track |

A controller with no key has the validity (0x80000000, 0x7fffffff) and the cached value is its
value. A controller with keys has the validity (t, t), and the cached value is the value at t.

A Bezier Float key has these fields: time (int), flags (uint), value, in tangent, out tangent,
in length, out length (5 floats).
A Bezier Scale key has: time (int), flags (uint), then 5 ScaleValue records of 28 bytes
(value, in tangent, out tangent, in length, out length). A ScaleValue is a Point3 and a Quat.

Key flags (istdplug.h): bits 7 to 9 are the in tangent type, bits 10 to 12 are the out tangent
type (0 smooth, 1 linear, 2 step, 3 fast, 4 slow, 5 custom, 6 auto). Bits 0 to 3, 30 and 31 are
selection flags. Each of the 40041 keys of the 47 scenes has the type custom for the two
tangents (flags 0x1680 and selection bits).

Bezier Point3, Bezier Color, Bezier Position and Linear Rotation have no keys in the 47 scenes.
The module raises NotImplementedError if it finds a key chunk (id `0x2510` to `0x252f`) in them.

### Interpolation of a Bezier key segment

For key 0 (t0, v0, out tangent ot, out length ol) and key 1 (t1, v1, in tangent it, in length il),
with dt = t1 - t0, the curve is a cubic Bezier in (time, value) with the control points
(t0, v0), (t0 + ol dt, v0 + ot ol dt), (t1 - il dt, v1 + it il dt), (t1, v1).
A tangent is a slope for each tick. The in tangent points back in time, thus a smooth key has
it = -ot.
A length of 0.3333 in the file is the default and behaves as 1/3 exactly. With two default
lengths the time is linear in the parameter, u = (t - t0) / dt. With a different length the
module solves time(u) = t.
A length of -1 is in the file for the in length of the first key and the out length of the
last key, and for some keys of constant tracks. The module uses the default for a length <= 0.

Proof: 648 Bezier Float controllers have a cached value between two keys. The module gives the
cached value with a maximum relative error of 1.2e-7 (float precision). 7 of them have a
changed tangent length: relative error 4.0e-7. With the literal 0.3333 the error is 1.5e-4,
and with "value handle = tangent dt / 3" for a changed length the value is wrong by
approximately 50 %. 4933 cached values at a key time agree with the key value (error 9.4e-9).

### Out-of-range types

`Control::Save` writes the container `0x8499` with `0x3000` (type before the range), `0x3001`
(type after the range) and `0x3003` (flags) only if a value is not the default. No keyframe
controller of the scenes has this chunk, thus each one has the type constant.
The module has the other types (cycle, loop, ping pong, linear, identity, relative repeat) from
`CycleTime`, `NumCycles` and the Extrapolate example of the SDK (control.h, simpwave.cpp).
No scene uses them.

## Position XYZ, Euler XYZ, ScaleXYZ

The references 0, 1 and 2 are the float controllers for X, Y and Z.
Euler XYZ has chunk `0x1003` (int): the axis order. A value >= 100 is "order + 100" and the
references are the X, Y and Z angle. A value from 1 to 5 is a file of an old version, where
reference i is angle i. Each of the 7606 Euler XYZ controllers of the scenes has 0 (XYZ).
Value: `tm = RotateX(x) * RotateY(y) * RotateZ(z)` for XYZ, and `mat = tm * mat` for
CTRL_RELATIVE (eulrctrl.cpp).

## Position/Rotation/Scale

| Chunk | Size | Content |
| --- | --- | --- |
| `0x7230` | 4 | Inheritance flags. A set bit means "do not inherit". Bits 0 to 2 position, 3 to 5 rotation, 6 to 8 scale |
| `0x7231` | 4 | Equal to `0x7230` in each of the 9404 controllers. Not decoded |
| `0x7232` | container | In one controller of each scene. `0x03e8` is a Matrix3 (12 floats), identity in each scene. `0x03f2` is an int (4). It is probably the "inheritOffsetTM" that CATRigPresets.cpp names |
| `0x2535` | 4 | Lock flag |

The references 0, 1 and 2 are the position, rotation and scale controller.
The scenes have the flags 0 (6443) and 0x1c0 (2961, on the PRS controllers below the CAT classes
and below one Link Constraint).
The module applies position, rotation and scale with CTRL_RELATIVE in this sequence. For 0x1c0
it removes the scale from the parent matrix (polar decomposition) before that. It raises
NotImplementedError for other flag values and for an offset matrix that is not identity.

## List controllers

| Chunk | Content |
| --- | --- |
| `0x1010` | Count of the sub-controllers (int) |
| `0x1020` | Index of the active sub-controller (int) |
| `0x1030` | Name of an entry (UTF-16). `0x1040` is an entry with no name |

References: the sub-controllers, then one empty slot ("Available"), then the clipboard
controller, then the ParamBlock2 with the weights. In the ParamBlock2, parameter 0 is the weight
Tab (float, 1.0 = 100 %) and parameter 1 is "average" (bool, 0 in each scene).
The evaluation is a direct port of listctrl.cpp. The lists that the scenes use are
Position List [Position XYZ, Position Constraint] and Rotation List [Euler XYZ or Linear
Rotation, Orientation Constraint, Noise Rotation]. 17 Rotation Lists have a Bezier Float
controller on a weight.

## Position Constraint and Orientation Constraint

Reference 0 is the ParamBlock2. Parameter 0 is the weight Tab, 1 the target node Tab, 2
"relative" (keep the initial offset), 3 "local_world" (Orientation Constraint only).

| Chunk | Position Constraint | Orientation Constraint |
| --- | --- | --- |
| `0x1001` | basePointLocal (Point3) | baseRotQuatLocal (Quat) |
| `0x1002` | basePointWorld (Point3) | baseRotQuatWorld (Quat) |
| `0x1003` | InitialPosition (Point3) | InitialOrientQuat (Quat) |
| `0x1004` | oldTargetNumber (int) | oldTargetNumber (int) |

Each constraint of the 47 scenes has 1 target and the world mode.

## Link Constraint and LinkTimeControl

Link Constraint: chunk `0x0111` is the version (450). References: 0 the PRS controller, 1 not
used, 2 the ParamBlock2, 3 the LinkTimeControl. In the ParamBlock2, parameter 0 is the target
node Tab (a null node is the world), parameter 2 is the start time Tab (ticks), parameter 5 is
the key selection Tab.
LinkTimeControl has no data. Its value is the index of the active link.
The evaluation is a direct port of `GetParentTM` and `CompTM` of link_cnstrnt.cpp. The
Link Constraint does not use the matrix of the scene parent.

## ParamBlock2

| Chunk | Size | Content |
| --- | --- | --- |
| `0x0009` | 16 | Header: class directory index of the owner (int), block id (uint16), 0x2a00 (uint16), 16000 (uint16, the release), parameter count (uint16), scene index of the owner (int) |
| `0x000b` | 24 | Header with the Class_ID (2 uint) and the superclass (uint) of the owner in place of the class index |
| `0x100e` | variable | One parameter |
| `0x0003` | container | Data of a bitmap parameter (after its `0x100e` chunk). Not decoded |
| `0x1005`, `0x000c`, `0x0011`, `0x0007` | | Not decoded. `0x1005` has parameter names of a Tab |

No scene has a `0x000e` chunk.

A `0x100e` chunk has: parameter id (int16), type (int32, `ParamType2`, `0x800` = Tab), flags
(int64, `P_*` of iparamb2.h), then the value. A Tab has one byte (0x80 if animatable), the count
(int32) and then the entries. Each value or entry starts with one flag byte:

| Flag byte | Meaning |
| --- | --- |
| `0x40` | A constant follows |
| `0xc0` | A constant follows, the parameter is animatable and has no controller |
| `0x80` | No constant. The value is in a reference slot (a controller) |

Size of the constant: float, angle, percent, world, colour channel 4 (float). int, bool, time,
radio index, index 4 (int). Point3, colour 12. Point4 16. Matrix3 52 (12 floats and the 4-byte
identity flags). String and file name: byte count (int32, -1 for null) and UTF-16 text with the
end zero. Bitmap 1 byte.
A reference type (material, map, node, reference target) with `P_NO_REF`, `P_OWNERS_REF` or
`P_SUBTEX` stores a scene index (int32, -1 for null). With no such flag it has no constant and
uses a reference slot.

Reference slots: the references of the block (chunk `0x2034`) are in the sequence of the
parameters and of the Tab entries. Each owned reference and each entry with the flag byte 0x80
uses the next slot.
Proof: the count of slots is equal to the count of references in 43697 blocks (all blocks of
the 48 scene folders but 48). Each of the 4113 node slots points to a Node, and each of the 4209
controller slots of a class that is not from CAT points to a controller.
The 48 other blocks have the sparse reference chunk `0x2035` with a count of 9 and no entry:
`maxscene.py` gives an empty list for them, and the module gives -1 for each of their slots.
The CAT classes use the slot of an animatable float parameter for objects that are not
controllers (2056 cases, for example DigitData and CATSpineData2).

## ParamBlock (old)

`0x0001` parameter count (int), `0x0005` version (uint16), then one container `0x0002` for each
parameter: `0x0003` index (int), `0x0004` (empty, animatable), and one of `0x0100` float,
`0x0101` int, `0x0102` Point3 or colour, `0x0104` bool, `0x0200` (empty: a controller in the
next reference slot). One block of each scene has a controller. No covered class uses a
ParamBlock.

## Noise controllers

Class_ID 0x87a6df24 to 0x87a6df28 (float, position, Point3, rotation, scale). The scenes have 4
Noise Rotation controllers (inspect_17, unfolded_inspect_17, jam_19, unfolded_jam_19), in the
Rotation List of the CAT bone "Base HumanCambone".

| Chunk | Content |
| --- | --- |
| `0x8499` | Container of `Control::Save`: `0x3000`, `0x3001` out-of-range types, `0x3003` flags (1 in each noise controller: time range off, `A_ORT_DISABLED`) |
| `0x0110`, `0x0111`, `0x0112` | The ">0" flag of X, Y, Z (int) |
| `0x0103` | Frequency (float) |
| `0x0104` | Roughness (float) |
| `0x0105` | Seed (int) |
| `0x0106` | Fractal (int) |
| `0x0107` | Time range (start, end) |
| `0x0108`, `0x0109` | Ramp in, ramp out (ticks) |

References 0 and 1 are empty (ease and multiplier curves). Reference 2 is the strength
controller (Bezier Point3, radians for a rotation).
The port follows noizctrl.cpp and perlin.cpp with 32-bit float arithmetic. The table of
perlin.cpp uses `srand(0)` and `rand()`: the module uses the generator of the Microsoft C
library (`x = x * 214013 + 2531011`, result `(x >> 16) & 0x7fff`).
Values of the scenes: frequency 0.02 and 0.01, fractal off, seed 0, strength 3, 1, 2 degrees
(inspect) and 0.1, 0.2, 0.25 degrees (jam). The largest noise angle is approximately 0.5 degrees.

## Proof of the conventions

| Rule | Check | Result |
| --- | --- | --- |
| Euler XYZ is `mul(mul(rotx(x), roty(y)), rotz(z))` and applies as `E * tm` | For a bone with a Rotation List [Euler XYZ, Orientation Constraint], the stored `baseRotQuatWorld` is `Quat(E * M(baseRotQuatWorld of the parent bone))` | 1175 of 1175 bone pairs, error < 1.1e-4. Order ZYX fits 658, `tm * E`, negative angles and the other Quat hand fit 0 |
| `quat_to_mat` is `Quat::MakeMatrix` for row vectors (left-hand rule) | Same check, and `basePointWorld = basePointLocal * M(parent)` in fo4rig | 59 of 59 point pairs, error < 8.2e-6. The transposed matrix fits 2 |
| `Quat::operator*` is `quat_hamilton(a, b)`, and `M(a * b) = mul(M(a), M(b))` | `baseRotQuatWorld = baseRotQuatLocal * parent rotation` in fo4rig (orientation_cnstrnt.cpp, GetValue) | 59 of 59 pairs with a local offset > 2 degrees, error < 3.6e-7. The other order fits 2 |
| Inheritance flags: a set bit means "do not inherit" | 6443 ordinary nodes have 0 and their rotations compose with the parent (first check). CATRigPresets.cpp says the same in a comment | |
| ParamBlock2 reference slots | Slot count and class of each target | 43697 blocks |
| Bezier interpolation | Cached values | 648 values |

`maxmath.py` has no error. Its doc text agrees with each check.

## Rules with no proof in the files

See the report of the module author for the full list. In short:
the value of a noise controller (time factor 0.005, `EulerToQuat`, ramp with the range off),
`Quat::operator/` (from the SDK doc text), the parent filter of a PRS controller for flags
0x1c0 if the parent has a scale, `ApplyScaling` with an axis quaternion, tangent types other
than custom, out-of-range types other than constant, Euler orders other than XYZ, the local
mode of the Orientation Constraint, constraints with more than one target, list weights with
"average", the old ParamBlock controller slot.
`StdEval.unverified` gets a name when one of these paths runs. `StdEval(scene, node_tm,
strict=True)` raises NotImplementedError on them.
