#!/usr/bin/env python3
"""engine-compare CSV 汇总：把 mps 侧与 Jolt 侧的采集 CSV 合并成
engine-compare.md 对照表的一行 Markdown。

用法：
  python3 crates/mps-test/scripts/summarize_engine_compare.py \
      target/engine-compare-mps.csv target/engine-compare-jolt.csv

CSV 契约（engine_benchmark_vs_jolt_protocol 与 Jolt 采集脚本共同遵守）：
  engine,scene,bodies,steps,mean_ms,p50_ms,p95_ms,logical_cpus
（logical_cpus 列是采集环境的逻辑核数记录，不是 solver 迭代数；同迭代数
的可比性由采集流程保证，本脚本不校验它。）
无 Jolt CSV 时只打印 mps 行（比值列 —），退出码 0；两侧都有时打印合并行
并把比值算出来。不做任何跨机器推断——CSV 里没有机器指纹列，同机性由
采集流程保证，本脚本不猜。
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path}: no data rows")
    for row in rows:
        for key in ("mean_ms", "p50_ms", "p95_ms"):
            row[key] = float(row[key])
        row["bodies"] = int(row["bodies"])
        row["steps"] = int(row["steps"])
        row["logical_cpus"] = int(row["logical_cpus"])
    return rows


def pick(rows: list[dict], engine: str, bodies: int, logical_cpus: int) -> dict:
    """选 bodies/logical_cpus 匹配的行；多轮采集（同参数多次跑）取 p50
    中位数——用中位数而不是均值，对 boost 波动稳健，与计时口径一致。"""
    matches = [r for r in rows if r.get("engine") == engine
               and r["bodies"] == bodies and r["logical_cpus"] == logical_cpus]
    if not matches:
        raise SystemExit(f"no {engine} rows for bodies={bodies} logical_cpus={logical_cpus}")
    matches.sort(key=lambda r: r["p50_ms"])
    return matches[len(matches) // 2]


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__)
        return 2
    mps_rows = read_rows(Path(argv[1]))
    jolt_rows = read_rows(Path(argv[2])) if len(argv) > 2 and Path(argv[2]).exists() else []

    # 汇总目标：最大 bodies（最重的负载行）。
    bodies = max(r["bodies"] for r in mps_rows)
    cpus = {r["logical_cpus"] for r in mps_rows if r["bodies"] == bodies}
    if len(cpus) != 1:
        raise SystemExit(f"mps rows for bodies={bodies} have mixed logical_cpus: {sorted(cpus)}")
    logical_cpus = cpus.pop()

    m = pick(mps_rows, "mps", bodies, logical_cpus)
    if not jolt_rows:
        print(f"| _待填_ | | | {bodies} cubes, sleeping "
              f"| {m['p50_ms']:.3f} | _待测_ | — |")
        return 0
    j = pick(jolt_rows, "jolt", bodies, logical_cpus)
    ratio = m["p50_ms"] / j["p50_ms"]
    print(f"| _日期_ | | | {bodies} cubes, sleeping "
          f"| {m['p50_ms']:.3f} | {j['p50_ms']:.3f} | {ratio:.2f}x |")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
