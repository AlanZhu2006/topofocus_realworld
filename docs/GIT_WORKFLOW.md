# Git 管理与发布规则

## 远端与分支

- 主远端：`git@github.com:AlanZhu2006/topofocus_realworld.git`
- 默认分支：`main`
- 普通工作分支：`agent/<short-description>` 或 `feature/<short-description>`
- 不在 WSJ 的生产 TinyNav checkout 里长期保留未提交修改；先创建独立 worktree/branch。

本次目标仓库创建时为空，因此首个可复现基线直接建立 `main`。后续变化走工作分支和 Pull Request。

当前集成分支为 `agent/live-map-recovery-20260722`，目标为 `main`，草稿
PR 为 GitHub `#1`。截至 2026-07-24，作者身份应为：

```text
user.name  = AlanZhu2006
user.email = yz11502@nyu.edu
```

发布前必须再次用 `gh auth status` 确认实际账号为 `AlanZhu2006`，不能
只依赖 SSH host 名称。

本机同时配置多个 GitHub 身份时，先确认仓库所有者对应的 SSH host alias：

```bash
ssh -T github-alan
git remote set-url origin git@github.com:AlanZhu2006/topofocus_realworld.git
git remote set-url --push origin git@github-alan:AlanZhu2006/topofocus_realworld.git
git remote -v
```

这里的 `github-alan` 是本机 `~/.ssh/config` 中的身份别名，不应原样复制到新机器。新 Go2 只需读取公开仓库时，可直接使用 README 中的 HTTPS clone 地址，无需 GitHub 凭据。不要把私钥、token 或 `~/.ssh/config` 提交到仓库。

## 每次提交前

```bash
git status --short
git diff --check
bash hub/scripts/verify_repository.sh --tests
```

只显式暂存本次范围，例如：

```bash
git add README.md docs/ hub/robot_overlay/ manifests/
git diff --cached --stat
git diff --cached
```

不要使用会把运行目录一起带入的无审阅 `git add -A`。`artifacts/`、`data/`、`logs/`、`hub/runtime/`、虚拟环境、机器人 token 和本机配置永不提交。

大规模现场集成可以形成一个完整的阶段性提交，但仍要先生成显式文件
清单，逐项检查 `git check-ignore`、文件大小、秘密扫描和 staged diff。
“所有当前进度”不等于把 runtime 或操作者凭据加入 Git。

## 提交信息

推荐 Conventional Commit：

```text
feat(robot): add fail-closed camera preflight
fix(mapping): reject mismatched native map frames
docs(repro): record WSJ JetPack baseline
test(hub): cover occupancy adapter metadata
```

一个提交只表达一个可回滚意图。物理机器人行为变化必须在正文写明安全边界、验证证据和未验证项。

## 更新 WSJ TinyNav 快照

1. 在 WSJ 的独立 branch/worktree 中完成并测试修改。
2. 确认起点是可公开获取的上游 commit。
3. 检查所有提交和最终 tree 是否含 token、口令、内网凭据或大模型。
4. 从公开基线到最终 tree 生成一个 flattened `--binary --full-index` patch。
5. 如果历史提交含敏感值，只发布最终净状态 patch，不发布 bundle/mail history。
6. 更新 `tinynav_snapshot/README.md`、`manifest.sha256`、环境文档和 audit。
7. 在一个临时 clone 中实际运行 `bootstrap_go2.sh`，校验结果 tree 和测试。

原始 WSJ 证据必须标注：远端路径、字节数、SHA-256，以及 observed/source-derived/unverified。临时导出目录或传输服务验证后立即移除。

## 版本与发布

在满足以下条件后再建立 tag：

- `main` 干净且 CI/本地验证通过；
- 外部资产 manifest 与文档同步；
- 新 Go2 的代码预检通过；
- 对应门禁结论没有被夸大。

推荐 tag：`v0.x.y`；若只是实验现场冻结，可用 `wsj-YYYYMMDD`，但不能把实验 tag 描述为生产 release。

机器人部署快照和 Git 提交是两个不同证据：

- 机器人归档 hash 证明某些字节已复制到设备；
- Git commit 证明另一台机器能取得相同源码；
- 只有设备重启并核对进程命令/hash，才能证明运行时已加载该版本；
- 只有现场观察和 episode 报告才能证明真机行为。

四层证据必须分别写成 `implemented`、`synchronized`,
`loaded/observed` 或 `physically verified`，不得合并描述。

## 回滚

部署始终创建新 checkout/branch，不覆盖已知可运行目录。回滚通过切换到上一 Git commit 和对应 transform version；地图、标定和代码版本必须一起回滚，禁止只换代码却继续复用不兼容的 world frame。
