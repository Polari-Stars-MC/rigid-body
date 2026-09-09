# World Step Benchmark Matrix

Run alone on the same machine with its power configuration unchanged:

```powershell
$env:Path = "C:\msys64\mingw64\bin;$env:Path"
cargo +stable-x86_64-pc-windows-gnu test -p mps-test --release world_step_performance_matrix -- --ignored --nocapture --test-threads=1
```

The generated `target/world-step-matrix.csv` records mean, P50, P95 and
process CPU usage normalized to available logical processors. Windows CPU time
comes from GetProcessTimes (kernel + user, all process threads). Other platforms
currently report NaN for CPU usage. No system-wide utilization is inferred.
P50 is the median; P95 uses nearest rank (the maximum at 10 samples).

`target/world-step-matrix-raw.csv` preserves all ten samples, body count,
process CPU seconds and seven pipeline timings. Enable `--features profiler`
to populate the stage columns; otherwise those columns are zero. Stage timings
include callbacks invoked within each stage and do not isolate event handler
cost. Measure absolute acceptance timings without the profiler enabled.

Set `MPS_MATRIX_BODIES` to change the shared body count (default 10000), and
`MPS_MATRIX_CASES` to a comma-separated subset, for example `none,sparse` for
100000 or 1000000 bodies. Each invocation overwrites both CSVs; archive them
before the next run. The runner prints logical CPU and Rayon thread counts;
record CPU model, power mode and `rustc -Vv` alongside archived results.
Set `MPS_MATRIX_STEPS` to run multiple consecutive steps per sample. This is
useful for sleeping and warm-start convergence; the default remains one step so
the baseline stays comparable.
Run without other builds or tests competing for CPU. Debug builds are rejected.

All cases use 10,000 bodies, equal mass/inertia, identical initial velocities,
and dt=1/60. Each case is warmed up, then recreated for 10 independent first-step
samples. Case order rotates between repetitions. Setup, validation and teardown
are excluded. None/sparse/dense/dense-all form the four principal groups.
Dense-all enables CCD, collision/contact-force events and real hook dispatch.
The hook implementation is the existing core handler, without a custom Coulomb
law or Java callback; arbitrary callback workload is not measured.

Solver 1/2/4/8, events, hooks and sleeping compare against dense_baseline.
CCD 0/1/2/4 all keep body CCD enabled; compare those rows to ccd_1, not to a
CCD-disabled baseline. Disabling collision/contact events must produce no events;
enabled dense cases must produce events. All body states must remain finite.

These first-step data do not establish resting-stack stability, long-term energy
error, tunneling prevention, or sleeping convergence. A faster solver setting
must not be described as equivalent precision. The old dt=0.5, single-sample
benchmarks are different workloads and cannot establish a 20% improvement here.

## Java Runtime Configuration

Add this declaration to the downstream RapierNative class:

```java
public static native boolean worldApplyRuntimeSettings(long world, int solverIterations,
    int ccdSubsteps, int enableCollisionEvents, int enableContactForceEvents,
    int enableCcd, int enableSleeping);
```

Binary values must be 0/1. Use 4,1,0,0,0,1 as a conservative low-overhead
starting configuration. Settings affect current bodies/colliders and are also
applied automatically to objects inserted through the standard world APIs.
These world-level flags intentionally override conflicting builder flags.
No hidden O(n) reconfiguration is added to world_step. Calls require exclusive
world access. Existing queued events are retained, and collider force-event
thresholds are preserved. Disabling sleeping wakes current bodies. Re-enabling
sleep restores default thresholds only for bodies with negative thresholds.
Existing world defaults and physical solver accuracy are not silently changed.
`enableCcd=0` also sets effective CCD substeps to zero: this Rapier fork otherwise
automatically sweeps fast dynamic bodies against fixed colliders even when their
body CCD flags are false. Re-enable with the requested substep budget. Directly
setting integration parameters afterwards can independently re-enable that tier.

The stage-four performance targets are goals, not assertions derived from a
single run. Only apply algorithm changes after repeatable same-workload profiles
and physics regression tests support them.

## Recorded Baseline (2026-09-09)

Windows GNU, rustc 1.98.1, Release without profiler, 12 available logical CPUs
and 12 Rayon workers. Ten independent first steps with 10000 bodies:

| Case | Mean ms | P50 ms | P95 ms | Process CPU / machine capacity |
| --- | ---: | ---: | ---: | ---: |
| No collider | 11.583 | 10.815 | 14.953 | 21.36% |
| Sparse collider | 20.944 | 20.410 | 24.675 | 27.98% |
| Dense contacts | 84.617 | 82.588 | 91.415 | 49.40% |
| Dense + CCD/events/hooks | 85.725 | 84.204 | 94.586 | 43.14% |

Archived local outputs: `target/world-step-matrix-release-10000.csv` and
`target/world-step-matrix-release-10000-raw.csv`. Windows process CPU accounting
has coarse resolution, so short samples can report zero CPU time; percentages
aggregate CPU and wall time across all ten samples.

Solver 1/2/4/8 averaged 69.182/72.369/83.542/99.706 ms. One iteration was
17.2% faster than four in this run, with different solver accuracy. Contact
force events averaged 90.594 ms versus 84.617 ms for the baseline. Other small
differences, including the apparently faster hooks case, require further runs
before attributing them to a feature. Sleeping cannot converge in one step.

These measurements establish a baseline, not an achieved 20% optimization.
The 100000-body non-regression and million-body sub-2-second targets remain
unverified against an equivalent historical workload. Defaults retain four
solver iterations; disabling colliders is not recommended for colliding scenes.

A separate profiler-enabled run averaged the following dense-baseline stages:
update 1.748 ms, broad phase 11.822 ms, narrow phase 22.336 ms, island construction
7.228 ms, solver 28.693 ms, CCD 0 ms, pipeline total 80.302 ms. These counters are
not a disjoint exhaustive accounting of total time. The profiler CSVs are archived
as `target/world-step-matrix-profiler-10000.csv` and
`target/world-step-matrix-profiler-10000-raw.csv`.

Prioritize solver and contact generation in subsequent algorithm experiments.
The fixture already uses balls, so replacing mesh geometry cannot explain an
improvement here. Its dense layout is a single plane of overlapping balls with
alternating horizontal velocities, not a settled production stack. A resting
stack workload and a representative Java scene are needed before recommending
sleeping or solver accuracy changes for production.
