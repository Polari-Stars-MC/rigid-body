# Default Collider Policy (Java/JNI and C/FFM)

The policy selects caller-provided geometry for future calls to
`worldInsertDefaultCollider`. It does not disable collision detection globally,
replace existing shapes, or change explicit collider insertion APIs.

Values: `0` = None, `1` = Simple (default), `2` = Compound, `3` = Adaptive.

Adaptive prefers the compound builder when both builders are supplied, and falls back to the simple builder when only that builder is available. It never replaces existing colliders.
Simple accepts non-compound shapes, including caller-supplied meshes; the name
does not guarantee cheap collision detection. Compound requires a real compound
builder, for example one created by `colliderBuilderCreateCompoundBoxes`.

Add these declarations to the downstream Java class
`org.polaris2023.mps.rapier.RapierNative` (Java sources are maintained outside
this repository):

```java
public static native long worldCreateWithCollisionMode(double gx, double gy, double gz, int mode);
public static native boolean worldSetDefaultCollisionMode(long world, int mode);
public static native int worldGetDefaultCollisionMode(long world);
public static native long worldInsertDefaultCollider(long world, long body, long simpleBuilder, long compoundBuilder);
```

Create body handles using the existing rigid-body API, then insert the selected
collider:

```java
long collider = RapierNative.worldInsertDefaultCollider(world, body, simpleBuilder, compoundBuilder);
```

Only the selected builder is read. Unselected builders may be `0`. Builders
are borrowed and reusable, unlike `colliderBuilderBuild`, which consumes its
builder. Destroy each borrowed builder once after its last use. Each insertion
adds a collider and returns its handle; it does not replace previous insertions.
None returns `0` with native error `ERR_OK`. Failures return `0` with a nonzero
native error. A failed getter returns `-1`; invalid modes fail without changing
the policy. In None mode, configure body mass explicitly if needed because no
collider contributes mass or inertia.

All mutation calls require exclusive world access, including relative to
`worldStep`. Getters also require no concurrent world mutation.

`worldGetPipelineTimings(world, outValues, 7)` exposes the latest Rapier
counter values in milliseconds, in this order: update, broad phase, narrow
phase, island construction, solver, CCD, total. Counters are updated by the
last completed `worldStep`; before the first step they are zero. This is the
production profiling path and is preferable to parsing ETL symbols.

C/FFM equivalents are declared in generated `rigid_body.h`:
`world_create_with_collision_mode`, `world_set_default_collision_mode`,
`world_get_default_collision_mode`, `world_insert_default_collider`.
Modes use `uint32_t`; builder arguments are opaque addresses and the result of
insertion is a packed `uint64_t` collider handle.

The ignored `collider_mode_performance_benchmark` in mps-test exercises these
production C ABI entry points. It does not measure JNI transition overhead or
represent a dense-contact workload.

Standalone JNI smoke test (PowerShell, GNU build):

```powershell
$env:Path = "C:\msys64\mingw64\bin;$env:Path"
cargo +stable-x86_64-pc-windows-gnu build -p mps-jni
javac -d target/jni-collision-mode crates/mps-test/java/collision-mode/org/polaris2023/mps/rapier/RapierNative.java
java -cp target/jni-collision-mode org.polaris2023.mps.rapier.RapierNative "$((Resolve-Path target/debug/mps_rigid_body.dll).Path)"
```
