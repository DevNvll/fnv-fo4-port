# Mesh, skin and material data in a 3ds Max 2014 scene file

These notes give the chunk format that `tools/maxmesh.py` reads. The data comes from the
47 scene files of the MP7 animation set (3ds Max 2014, version 16.00, not compressed) and
from the scene `fo4rig`. A line says so when a value is a guess.

## Commands

```
python3 tools/maxmesh.py ole/reload_48 -o out/mesh/reload_48
```

The command writes `out/mesh/reload_48/meshes.json`. The option `--node NAME` reads one node
(repeat it for more nodes). The option `--rest-node NAME` sets the node for the rest transform
(default `MP7`).

```
U=/tmp/mp7-port/blender-user
BLENDER_USER_CONFIG=$U/config BLENDER_USER_SCRIPTS=$U/scripts BLENDER_USER_DATAFILES=$U/datafiles \
  blender --background --factory-startup --python tools/blender_meshcheck.py -- \
  out/mesh/reload_48/meshes.json out/mesh/reload_48/pictures
```

The command writes 24 PNG files into `out/mesh/reload_48/pictures`. The docstring of
`tools/blender_meshcheck.py` lists them.

## Conventions

* A matrix has 4 rows of 3 numbers. Rows 0 to 2 are the axes and row 3 is the translation.
  A point is a row vector: `p2 = p @ m[:3] + m[3]`. This is the convention of `tools/maxmath.py`.
* A quaternion is `(x, y, z, w)` as the file stores it. `maxmath.quat_to_mat` gives its matrix.
* All integers and floats are little-endian. A float has 32 bits.
* Text is UTF-16 with no byte count, unless the notes say a count.
* Weapon space (the object space of each weapon mesh): +X is the muzzle direction, +Y is up,
  +Z is the right side of the gun (the side of the ejection port). The origin is in the pistol
  grip, at the height of the magazine release. The axis of the barrel is at Y = 4.10.
* World space of the body in the bind pose: +Z is up, the body looks to +Y, the left arm is at -X.
* The values are the numbers of the file. The system unit setting of the file is not decoded.
  The models come from Fallout: New Vegas NIF files, where 1 unit is 1.4288 cm. The receiver
  with the muzzle is 28.71 units long, which is 41.0 cm. The real MP7 is 41.5 cm long with the
  stock in.

## Scene objects

The stream `Scene` has one root chunk. Each child of the root is one scene object. The chunk id
of an object is its index in `ClassDirectory3`. The id `0x2032` is not in the class directory:
it is a derived object (an object with modifiers).

| Chunk | Content |
| --- | --- |
| `0x2034` | References: a list of int32 scene object indices, -1 for none. |
| `0x2035` | References as pairs: one int32, then pairs of (slot, object index). |
| `0x204B` | 1 byte, `0x2E` in each object. Not decoded. |

## Node (class id `(1, 0)`, superclass `0x1`)

| Chunk | Size | Content |
| --- | --- | --- |
| `0x0960` | 8 | int32 index of the parent node, then 4 bytes of flags. |
| `0x0962` | | Node name, UTF-16. |
| `0x096A` | 12 | Object offset position: 3 floats. |
| `0x096B` | 16 | Object offset rotation: quaternion `x y z w`. |
| `0x096C` | 28 | Object offset scale: 3 floats, then the quaternion of the scale axes. |
| `0x0975` | 0 | Empty chunk that only some nodes have. Not decoded. |

References of a node: 0 is the transform controller, 1 is the object, 3 is the material.

The object transform is `offset * nodeTM`, with `offset = scale * rotation * translation`
(the order of `INode::GetObjectTM`). Each of the 23 mesh nodes has the offset
position `(0, 0, 0)`, rotation `(0, 0, 0, 1)` and scale `(1, 1, 1)`. The scale path of
`offset_matrix` had no data with a scale to test it.

## Derived object (chunk id `0x2032`)

The references are the modifiers, the top of the stack first, and then the base object.

| Chunk | Content |
| --- | --- |
| `0x2500` | Container, one for each modifier, in the order of the references. |
| `0x2500 / 0x2511` | 24 bytes: 6 floats, the bounding box of the modifier context (min, max). |
| `0x2500 / 0x2512` | Container: the local data of the modifier for this object. |
| `0x2500 / 0x2513` | 4 bytes, `00 01 04 00` in each case. Not decoded. |
| `0x2501` | Empty, after the last `0x2500`. |

The arm nodes have `BSDismemberSkin Modifier`, then `Skin`, then `Editable Mesh`. The hand nodes
have `Skin`, then `Editable Mesh`.

## Editable Mesh (class id `(0xE44F10B3, 0)`, superclass `0x10`)

Chunks of the object. The names are from `maxsdk/samples/mesh/editablemesh/triobjed.cpp`.

| Chunk | Content |
| --- | --- |
| `0x4020` to `0x403B` | Soft selection and flags of the Editable Mesh (`AR_CHUNK`, `FALLOFF_CHUNK` and others). Not used. |
| `0x0900` | 44 bytes: 5 intervals (int32 start, int32 end) and one int32. Not used. |
| `0x08FE` | Container: the mesh. |
| `0x0901` | Container with 16 small chunks (`0x300D` to `0x301E`). Not decoded. |
| `0x0902`, `0x0903`, `0x0904` | 4 bytes each. Not decoded. |

### Mesh container `0x08FE`

The chunks are in this order.

| Chunk | Content |
| --- | --- |
| `0x0906` | 4 bytes (`02 00 08 00`, `02 10 08 00` or `02 10 0A 00`). Not decoded. |
| `0x0908` | 1 float (10.0 or 20.0). Not decoded. |
| `0x0914` | Vertices: uint32 count, then count * 3 floats. Object space. |
| `0x0912` | Faces: uint32 count, then count records of 20 bytes. |
| `0x0924` | 4 bytes, different in each mesh. Not decoded. |
| `0x0928` | 4 bytes, 0. Not decoded. |
| `0x092A` | 4 bytes, 1. Not decoded. |
| `0x0959` | int32 map channel number of the next `0x2394` and `0x2396`. |
| `0x2398` | 4 bytes, 1. Not decoded. |
| `0x2394` | Map vertices: uint32 count, then count * 3 floats (u, v, w). |
| `0x2396` | Map faces: uint32 count, then count * 3 uint32 (one map vertex index for each face corner). |
| `0x23A0` | Container: the stored normals (MeshNormalSpec). |

A face record is the SDK class `Face`: `uint32 v[3]`, `uint32 smGroup`, `uint32 flags`. In the
flags, bits 0 to 2 are the visible edges, bit 3 is `FACE_HIDDEN` and bits 16 to 31 are the
material ID. In the MP7 scenes each face has the flags `0x00000007`: the material ID is 0 for
each face.

The group `0x0959`, `0x2398`, `0x2394`, `0x2396` comes one time for each map channel. The MP7
meshes have only channel 1. The meshes of `fo4rig` have the channels -2 (alpha), 0 (vertex
colour) and 1, which shows that `0x0959` is the channel number.

In a map vertex, `(0, 0)` is the bottom left corner of the texture and v goes up. A NIF file
needs `v2 = 1 - v`. In each MP7 mesh the number of map vertices is the number of vertices, and
the map faces are equal to the faces.

The front of a face is the side where its 3 vertices go counter-clockwise.

### Stored normals `0x23A0`

| Chunk | Content |
| --- | --- |
| `0x0130` | 4 bytes, 1. Not decoded. |
| `0x0100` | uint32 flags of `MeshNormalSpec` (`0x0B`: built, computed, face angles). |
| `0x0110` | Normals: uint32 count, then count * 3 floats. Object space, length 1. |
| `0x0114 / 0x2700` | Bit array: 1 for a normal that is explicit. All bits are 1 in the MP7 scenes. |
| `0x0120` | uint32 number of faces. |
| `0x0124` | uint32 face index of the next `0x0128`. |
| `0x0128 / 0x0200` | 3 int32: the normal index of each corner of the face. |
| `0x0128 / 0x0210` | 4 bytes. Byte 0 has bits 0 to 2: the corner uses a specified normal. Bytes 1 to 3 are not initialized. |

A bit array (`0x2700`) is a uint32 bit count and then the bits, the least significant bit of
each byte first.

In each MP7 mesh the number of normals is the number of vertices, the normal index of a corner
is its vertex index, and each corner is specified. These are the normals of the NIF file that
the NifTools importer read.

### Smoothing groups and stored normals

The smoothing groups of the MP7 meshes do not give the stored normals. `maxmesh.py` computes
`mesh.normals` from the smoothing groups as requested: at a vertex, faces that share a group
bit (directly or through a chain of faces at that vertex) share one normal, and the weight of
a face is its angle at the vertex. For `Receiver:0` the angle between this normal and the
stored normal has a mean of 14.1 degrees and is larger than 49 degrees for 5 % of the corners.
For `Muzzle:0` the mean is 32 degrees, which is near flat shading.

The stored normals agree with one normal for each vertex index, computed from all faces at
that vertex without smoothing groups (mean 1.0 degree for `Receiver:0`, 0.7 for `Muzzle:0`).
For the body meshes they agree with one normal for each position, across the texture seams
(mean 0.03 degree for `RightHand:0`). Use `mesh.spec_normals` for an export.

## Skin modifier (class id `(0x0095C723, 0x00015666)`)

The source is `maxsdk/samples/modifiers/bonesdef` (`BonesDefMod::Save` in `ClassDescStuff.cpp`,
`BonesDefMod::SaveLocalData` in `bonesdef.cpp`).

### Chunks of the modifier

| Chunk | Content |
| --- | --- |
| `0x0020` | int32 number of bones. |
| `0x0025` | One for each bone: Matrix3 `BoneData.tm`, the inverse of the object transform of the bone at the bind. |
| `0x0480` | One for each bone: Matrix3 `InitNodeTM`, the node transform of the bone at the bind. |
| `0x0520` | One for each bone: Matrix3 `InitStretchTM`. Identity for each bone in the MP7 scenes. |
| `0x0030` | Bone data, see below. |
| `0x0160` | Table of bone names. The names are empty in the MP7 scenes; `maxmesh.py` takes the name of the node. |
| `0x0230` | int32 version, 5. |
| `0x0400`, `0x0410`, `0x0490`, `0x0530`, `0x0550` | End point data, weight table window and weight tool. Not used. |

A Matrix3 is a container with `0x03E8` (12 floats: 4 rows of 3) and `0x03F2` (uint32 flags:
1 position is identity, 2 rotation is identity, 4 scale is identity).

The bone data `0x0030` has one record for each bone: int32 number of cross sections, then for
each cross section (float u, int32 inner reference, int32 outer reference), then uint8 flags,
uint8 falloff type, int32 `BoneRefID`, int32 end point 1 reference, int32 end point 2
reference. `BoneRefID` is an index into the references of the modifier. That reference is the
node of the bone. The references 0 to 6 of the modifier are parameter blocks and one
controller, and the bone references start at 10.

### Local data (`0x2512` of the derived object)

| Chunk | Content |
| --- | --- |
| `0x0010` | Matrix3 `BaseTM`: the object transform of the mesh node at the bind. |
| `0x0500` | Matrix3 `BaseNodeTM`: the node transform of the mesh node at the bind. |
| `0x0040` | int32 number of vertices. |
| `0x0490` | Weights, see below. |
| `0x0540` | int32 for each vertex: the closest bone (-1 in the MP7 scenes). |
| `0x0510` | uint32 reference id of the mesh node. Not used. |
| `0x0220`, `0x0240`, `0x0250`, `0x0460` | Exclusion lists, gizmos and named selections. Empty in the MP7 scenes. |

The weights `0x0490` have one record for each vertex: int32 number of influences, uint32 vertex
flags, then for each influence 44 bytes: int32 bone index, float weight, int32 curve id,
int32 segment id, float curve u, 3 floats tangent, 3 floats point. The bone index is the index
of the bone in the modifier. The vertex flags are 2 (`VERTEXFLAG_MODIFIED`) for each vertex of
the MP7 scenes.

### Spaces of the matrices

| JSON key | Chunk | Space |
| --- | --- | --- |
| `skin.bones[].init_node_tm` | `0x0480` | Bone space to world space, at the bind. |
| `skin.bones[].inv_init_object_tm` | `0x0025` | World space to bone object space, at the bind. It is the inverse of `init_node_tm` in the MP7 scenes (difference below 0.00001), because the bones have no object offset. |
| `skin.bones[].init_stretch_tm` | `0x0520` | Stretch transform of the bone at the bind. |
| `skin.mesh_init_object_tm` | `0x0010` | Mesh object space to world space, at the bind. |
| `skin.mesh_init_node_tm` | `0x0500` | Mesh node space to world space, at the bind. Equal to `mesh_init_object_tm` in the MP7 scenes. |

World space is the world space of the scene at the time of the bind. With row vectors:

```
p_world      = p_object * mesh_init_object_tm                    (bind pose)
skin_to_bone = mesh_init_object_tm * inverse(init_node_tm)       (mesh object space to bone space)
p_deformed   = sum over the influences of  w * (p_world * inv_init_object_tm * objectTM_bone(t))
```

`p_deformed` is in world space. The modifier then goes back to the object space of the mesh
(`temptm = BaseTM * tm * ntm * InverseBaseTM` in `bonesdef.cpp`).

Use the relative transform `skin_to_bone` and not the absolute matrices. The bind of `Arms:0`
and `Arms:1` has a shift of 0.0773 units on X relative to the bind of `limbcaps` and
`meatneck01`: the mesh matrix and the bone matrices have the same shift.

The mesh matrix is a translation of 0.0773 on X for `Arms:0`, `Arms:1` and `bodycaps`. It is a
rotation of 180 degrees about X for `RightHand:0` and `LeftHand:0`, and the same rotation with
the translation of 0.0773 on X for `limbcaps`: the vertices of these 3 meshes are in a space
where Y and Z have the opposite sign. It is a translation of `(0.0731, 0.2903, 112.8435)` for
`meatneck01` and `meathead01`.

## BSDismemberSkin Modifier (NifTools, class id `(0xE9A0A68E, 0xB091BD48)`)

Local data (`0x2512` of the derived object):

| Chunk | Content |
| --- | --- |
| `0x2846` | Container with one group for each partition: `0x2870` (container with a bit array `0x2700`: the faces of the partition), `0x2860` (empty), `0x2850` (4 bytes, 0). |
| `0x2848` | 4 bytes, 0. Not decoded. |
| `0x2849` | uint32 count, then for each partition 12 bytes: uint32 flags, int32 body part, uint32 (3 in each case). |

The flags are those of a NIF partition (1 editor visible, 0x100 start of a new bone set). The
body part is the Fallout 3 and New Vegas number (0 torso, 3 left arm, 5 right arm, 7 left leg,
10 right leg, 3000 and 5000 the arm sections of the torso). Each face is in one partition.
The partitions of `Arms:1` with the body parts 3, 3000, 5 and 5000 are the two forearms.

## Materials

The reference 3 of a node is its material. The MP7 scenes have only Standard materials
(class id `(2, 0)`, superclass `0xC00`). They have no Multi/Sub-Object material: the code for
it lists the references that are materials, and no data was available to test it.

| Object | Data |
| --- | --- |
| Material or texture map | Name: chunk `0x4000 / 0x4001`, UTF-16. |
| Standard | Reference 1 is the `Texmaps` object, reference 2 is the shader (`Niftools Shader`). |
| Texmaps | Two references for each slot: `2 * slot` is the amount controller, `2 * slot + 1` is the texture map. |
| Bitmap (class id `(0x240, 0)`) | A reference to a ParamBlock2 with the chunk `0x0003 / 0x1260 / 0x0002`: 16 bytes, the asset id of the file. |
| Normal Bump | Reference 1 is the normal map (a Bitmap). |
| Niftools Shader | Its ParamBlock2 has the NIF shader type as a string parameter (`BSShaderPPLightingProperty`). |

A parameter record of a ParamBlock2 is the chunk `0x100E`: uint16 parameter id, uint16 type,
11 bytes, then the value. For the type 8 the value is a uint32 byte count and UTF-16 text.

The stream `FileAssetMetaData3` is a list of records: 16 bytes asset id, then 3 texts. A text
is a uint32 number of characters, the UTF-16 characters and a 2 byte zero. Text 1 is the asset
type (`Bitmap`), text 2 is the file path, text 3 is empty.

The NifTools importer also gives the file path as the name of the Bitmap map.

The scene does not store the slot names of the Niftools Shader. From the file names: slot 0 is
the diffuse texture, slot 5 is the normal map, slot 11 is the skin map of the hands (`_sk`),
slot 12 is `mp7_s.dds` and slot 13 is the cube map. All paths are relative
(`textures\...`), and the texture files are not in `/tmp/mp7-port`.

## Static controller values

`maxmesh.py` reads only the values at the time of the save. It reads no key.

| Object | Data |
| --- | --- |
| Position/Rotation/Scale | References: position, rotation, scale controller. |
| Position XYZ, Euler XYZ | References: 3 Bezier Float controllers (X, Y, Z). Euler XYZ has the chunk `0x1003` (int32 axis order, 0 is XYZ). The angles are in radians. |
| Bezier Float | Chunk `0x7127 / 0x2501`: the float value. |
| Bezier Scale | Chunk `0x2505`: 3 floats scale, then the quaternion of the scale axes. |
| Link Constraint | Reference 0 is a Position/Rotation/Scale controller, reference 2 is a ParamBlock2 whose references are the target nodes. |

The local matrix of a Position/Rotation/Scale controller is `scale * rotation * translation`.
With one target, the node transform of a Link Constraint is `prs * nodeTM(target)`
(`LinkConstTransform::GetValue` in `maxsdk/samples/controllers/link_cnstrnt.cpp`).

Results for `reload_48`:

* `MP7` has the local position `(0, 0, -0.00004)` and no rotation relative to `Weapon`.
* `MP7 Parts` has a Link Constraint to `Weapon` with the position `(0.00004, -0.00024, -0.00073)`
  and no rotation.
* Each of the 15 weapon mesh nodes has a local position below 0.00002 and no rotation
  relative to its parent.
* Thus `Receiver:0`, `Stock:0`, `Grip:0`, `IronSights:0`, `Muzzle:0`, `FireSelector:0`,
  `BoltCatch:0` and `MagRelease:0` are at the identity relative to `MP7`, within 0.001 units.
* The bones `##nmBar`, `##MP7ChargingHandel`, `##nmClip`, `##nmBullet`, `##Trigger` and
  `##nmClip2` have a Link Constraint to a CAT bone (`Base HumanCatWeaponBoltBone` and others).
  Their rest transform relative to `MP7` is not in the static data.
* In `fp_base_idle_unfolded_4_6` these bones have a Position List and a Rotation List with a
  constraint to the same CAT bones. `##nmBullet` has a static identity relative to `##nmClip`.
* The vertices of these 7 meshes are in weapon space: the magazine is in the grip, the bolt is
  in the ejection port and the cartridge is at the top of the magazine when each mesh has the
  identity. The picture `weapon_views.png` shows this.
* `##SIGHTANIM` is at the identity relative to `MP7` (position below 0.001). `##SightingNode`
  has the position `(-18.0, 8.609, -0.023)` and the Euler angles `(2.36444, -1.57079, 2.36444)`
  relative to `##SIGHTANIM`.
* `ProjectileNode` and `ShellCasingNode` are children of the scene root, but their static
  positions are weapon space values: `(18.5, 4.2, 0.0)` is in front of the muzzle (the muzzle
  ends at X = 18.20, the barrel axis is at Y = 4.10) and `(-2.4417, 4.0, 1.0)` is at the
  ejection port on the +Z side.

## The file meshes.json

```
format, scene, notes, assets[]
nodes[]:
  name, index, parent, parent_index, object_index, object_class, modifiers[]
  object_offset: pos, rot, scale, scale_axis, matrix
  node_static:   controller, static, pos, euler_xyz_rad, scale, matrix, relative_to, link_targets
  parent_static: the same keys for the parent node
  rest:          relative_to, matrix, steps[]
  material:      class, name, shader, nif_shader_type, maps[]: slot, slot_name, class, name, file, maps[]
  mesh:          num_verts, num_faces, bbox_min, bbox_max, verts, faces, smoothing_groups,
                 material_ids, face_flags, map_channels, uv, uv_faces, normals,
                 spec_normals, spec_normal_faces, spec_normal_specified, spec_normal_flags
  skin:          bones[]: name, node_index, parent, ancestors[], flags, init_node_tm,
                          inv_init_object_tm, init_stretch_tm
                 mesh_init_object_tm, mesh_init_node_tm, weights, vertex_flags,
                 weight_sum_min, weight_sum_max, weight_sum_not_1, max_influences
  dismember:     partitions[]: body_part, part_flags, faces[]
```

The key `notes` of the file says what each array is and in which space. `mesh.normals` has one
normal for each face corner (faces x 3 x 3). The floats of the file are the shortest decimal
text that gives the same 32 bit float again.

## Checks

* Each face index, map face index and normal index is in its range. Each chunk is read to its
  last byte for the bone data and the weights.
* The mesh data (vertices, faces, map channel 1) of each of the 23 mesh nodes is the same in
  all 47 MP7 scenes (MD5 of the chunks). The JSON files of `reload_48` and
  `fp_base_idle_unfolded_4_6` differ only in the rest matrix (0.001 units) and its steps.
* The face normals point out: the signed volume of each closed mesh is positive, and the face
  normal agrees with the stored normals (6 of 21054 corners of `Receiver:0` have a negative dot
  product).
* Map channel 1: 95.7 % of the area of `Receiver:0` and 100 % of the area of 11 other meshes
  have triangles with a counter-clockwise order in the texture, which is the usual result of an
  unwrap with v up. `RightHand:0` is 100 % clockwise: it uses the texture of the left hand as a
  mirror image. `##nmClip:2`, `Stock:0` and `FireSelector:0` have mirror image parts (94 %,
  31 % and 45 % of the area).
* Map channel 1: the triangles of the 13 different weapon meshes cover 79.7 % of the texture
  square, and no pixel of a 2048 x 2048 grid is in the triangles of two meshes
  (`uv_weapon.png`). All weapon meshes use `mp7_d.dds`.
* Skin: the weights of a vertex add up to 1 (limits 0.9999999 and 1.0000001), except 3 vertices
  of `LeftHand:0` (538, 554 and 577: 0.9476, 0.9476 and 0.9499). No weight is negative. A
  vertex has 4 influences at most.
* Skin: `inverse(inv_init_object_tm)` is equal to `init_node_tm` for each bone. Each bone
  matrix has no scale (determinant 1).
* Skin: in the bind pose in world space the 11 wrist vertices of each hand mesh are at the same
  place as 11 vertices of `Arms:1` (distance below 0.001), and 42 vertices of `Arms:1` are at
  the same place as vertices of `Arms:0`. At these places the weights of the two meshes agree
  for each bone name (largest difference 0.0000 for the right hand, 0.0031 for the left hand).
  The two meshes have different bone lists, so this also checks the bone index of the weights.
* Skin: the picture `arms_bind.png` shows the 4 meshes as one body with the colour of the bone
  with the largest weight. The picture `arms_bend.png` shows a test pose (forearms 70 degrees,
  3 joints of each finger) computed with the formula above: the elbows bend, the hands stay on
  the wrists and the fingers close.
* Dismember partitions: each face is in one partition, and the partition with the body part 3
  (left arm) is at -X.

## Not decoded and not verified

* The system unit setting of the scene file.
* The keys of the controllers and each controller that is not in the table above.
* The chunks `0x0906`, `0x0908`, `0x0924`, `0x0928`, `0x092A` and `0x2398` of the mesh.
* The slot names of the Niftools Shader, and the Multi/Sub-Object material.
* The direction of v was not checked against a texture file, because no texture file is in
  the scratch folder.
* The MP7 scenes have no mesh with a material ID other than 0 and no mesh node with an object
  offset other than the identity. The code for these two cases had no data to test it.
