#!/usr/bin/env python3
"""生成 Stage A 证据矩阵文档骨架（evidence-first-freeze-contract skill 调用）。

用法：python .claude/skills/evidence-first-freeze-contract/scripts/gen_stage_a_doc.py <topic> [--date YYYY-MM-DD]

行为：
1、创建 docs/review/<topic>_stage_a_<date>.md 骨架，自动填 Date 与
   Review-Baseline（当前分支 @ HEAD）。
2、topic 必须是 kebab-case（如 rir-dual-product）。
3、同名文件已存在时拒绝覆盖，退出码 1。

skill 规定 Stage A 文档必须由本脚本初始化、禁止手写骨架，避免格式漂移；
脚本自身是骨架的唯一权威来源。
"""

import argparse
import datetime
import re
import subprocess
import sys
from pathlib import Path


def get_repo_root() -> Path:
    """取 git 仓库根；不在 git 仓库内时退回脚本所在 .claude/ 的上一级。

    脚本位于 <仓库根>/.claude/skills/evidence-first-freeze-contract/scripts/，
    parents 依次为 scripts → evidence-first-freeze-contract → skills → .claude → 仓库根。
    """
    try:
        res = subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            check=True, capture_output=True, text=True,
        )
        return Path(res.stdout.strip())
    except Exception:
        return Path(__file__).resolve().parents[4]


REPO_ROOT = get_repo_root()
TOPIC_RE = re.compile(r"^[a-z0-9]+(-[a-z0-9]+)*$")
DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")

SKELETON = """\
---
Status: draft
Date: {date}
Review-Baseline: `{branch}` @ `{commit}`
Authority: 过程脚手架记录（非耐久）；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准；
  权威回写完成、合并进 main 前移除本文档。
---

# {topic}：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

<!-- 1、触发来源一句话，并用证据形式锁定。
2、每个待裁定项一行，回答四问：冻结什么、什么证据能证明、什么证据能否定、通过后最小改动范围。
3、待裁定项必须是"需求/风险是否成立"的决定，不是实现可行性问题。
-->

## §1 证据矩阵

<!-- 1、探针/测试列写已执行的动作与结果。
2、建议判定只能取 pass / reject / narrow / defer，最终以用户裁定为准。
-->

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|

## §2 判定汇总与待裁定问题

<!-- 1、每项一行：建议判定 + 一句话理由。
2、defer 项给出下一步需要的探针。
3、需要用户拍板的问题逐条列出。
-->

## §3 冻结契约（用户讨论结束后填写）

<!-- 一行一项：
1、允许范围：模块/目录、类/函数、测试与文档。
2、明确禁止范围：公开头文件、跨模块类型、schema/回放、测试阈值、兼容层。
3、行为边界：输入、输出、错误回退、生命周期。
4、爆炸半径与回滚：下游消费方影响、回退难度（无损/破坏性/回滚注意点）。
5、验收门：构建、聚焦测试、契约测试、特征化测试、探针转正（有回归价值的探针转正式测试）。
6、非目标。
-->

## 修订记录

<!-- 编号条目（修订 1、修订 2……）：日期、裁定内容、来源（用户指令/新证据）；不静默改写既有条目。 -->

## §4 运行记录（Stage C 后填写）

<!-- 对照强制回写清单勾项，全部完成才允许拆脚手架（见 SKILL.md 收尾规则）：
1、实现范围：改动文件与接口。
2、验证命令与结果：命令: pass/fail（含转正的探针测试）。
3、权威回写去向：
   1、正向边界：docs/<module>/design.md（boundaries/data-flow/algorithms 按归属选）。
   2、否决记录：docs/<module>/design.md 的"架构裁定与否决记录"专节（体裁见 docs-governance-standard）。
   3、开放议题：docs/common/open_questions.md 登记（编号）。
   4、证据锁：新增/修订规则后附 - **证据**：[evidence: 路径]。
4、残留风险。
5、后续冻结项。
-->
"""


def git(*args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(REPO_ROOT), *args],
        check=True, capture_output=True, text=True,
    )
    return result.stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser(description="生成 Stage A 证据矩阵文档骨架")
    parser.add_argument("topic", help="主题，kebab-case，如 rir-dual-product")
    parser.add_argument("--date", default=datetime.date.today().isoformat(),
                        help="日期（YYYY-MM-DD），默认今天")
    args = parser.parse_args()

    if not TOPIC_RE.match(args.topic):
        print(f"错误：topic 须为 kebab-case（小写字母/数字/连字符），收到：{args.topic}",
              file=sys.stderr)
        return 1
    if not DATE_RE.match(args.date):
        print(f"错误：--date 须为 YYYY-MM-DD，收到：{args.date}", file=sys.stderr)
        return 1

    path = REPO_ROOT / "docs" / "review" / f"{args.topic}_stage_a_{args.date}.md"
    if path.exists():
        print(f"错误：文件已存在，拒绝覆盖：{path.relative_to(REPO_ROOT)}", file=sys.stderr)
        return 1

    try:
        branch = git("rev-parse", "--abbrev-ref", "HEAD")
        commit = git("rev-parse", "--short", "HEAD")
    except subprocess.CalledProcessError as exc:
        print(f"错误：无法读取 git 基线（{exc}）", file=sys.stderr)
        return 1

    path.write_text(
        SKELETON.format(date=args.date, branch=branch, commit=commit, topic=args.topic),
        encoding="utf-8",
    )
    print(path.relative_to(REPO_ROOT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
