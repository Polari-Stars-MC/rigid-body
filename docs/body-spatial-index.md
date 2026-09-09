# Body Spatial Index

Region count, immediate activation and scheduled region policies share one
body-center BVH owned by `PhysicsWorld`. It indexes dynamic bodies independently
of colliders. Multiple or offset colliders cannot duplicate or hide a body.
An AABB traversal finds candidates, followed by an exact sphere test.

The index is built lazily at the first region query. This initial O(n) build is
not part of the steady-state query cost. Subsequent queries synchronize only
the optional `RigidBodySet` spatial-change journal. This journal records insert,
remove, mutable indexing, mutable iteration, get_mut and paired mutable access;
it is separate from Rapier's modified-body list and does not consume that list.
Deletion/reinsertion resolves the current arena generation before updating a leaf.
Unchanged positions do not update the BVH.

After `world_step`, the existing force-reset loop updates indexed positions,
including internal solver writes, then clears the journal. No additional full
body traversal is introduced. A mutable iteration over every body still produces
O(n) synchronization candidates; moving every body requires updating every leaf.
Idle repeated queries neither scan the body set nor rebuild the tree.

This uses Parry's dynamic `Bvh`, not `GenericAabbIndex`: the latter performs linear
entry lookups and rebuilds its tree after mutations. Rapier `QueryPipeline` is a
cheap borrowed view of a collider BVH, not an owned index to cache in the world.

Calls retain the existing exclusive-world-access contract for mutations and
step. The index mutex only protects the query cache, not concurrent body writes.
Replacing the public `bodies` set or clearing its spatial journal externally is
not supported; use body insertion/removal/mutation APIs.

Region intervals still control policy application frequency, not independent
Rapier solver timesteps. Sleeping and wake behavior are unchanged by this index.

## Verification

```powershell
cargo +stable-x86_64-pc-windows-gnu test -p mps-test body_index
cargo +stable-x86_64-pc-windows-gnu test -p mps-test --release body_index_query_benchmark -- --ignored --nocapture --test-threads=1
```

The benchmark compares ten indexed queries and ten scans at 100000 and 1000000
bodies, reporting initial build cost separately. It measures sparse region
queries, not a claim about whole-world step acceleration.

Recorded on 2026-09-09, Windows GNU Release, ten repeated queries per method,
90 matched bodies in a sparse region:

| Bodies | First Query Including Build | Indexed Mean | Scan Mean |
| ---: | ---: | ---: | ---: |
| 100000 | 63.894 ms | 3.39 us | 2.764 ms |
| 1000000 | 739.238 ms | 3.69 us | 26.398 ms |

These are unchanged-world queries. Large-scale motion and large query coverage
have different maintenance and result-processing costs and are not measured here.
