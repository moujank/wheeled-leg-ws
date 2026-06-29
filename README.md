# wheeled-leg-ws

这是当前工作空间的 Git 仓库说明文档，用于记录仓库中的主要文件、常用操作方式，以及几个辅助脚本的用途说明。

## 仓库内容

- `thinking.txt`

  当前工作空间中的主要文本内容文件。

- `GIT_HISTORY.md`

  由 Git 历史自动生成的日志文件，用于快速查看历史提交记录、作者、提交时间以及每次提交对应的文件变更统计。

- `sync_to_github.sh`

  用于将当前工作空间中的全部改动快速同步到 GitHub。

  中文说明：
  该脚本会自动执行 `git add -A`、`git commit` 和 `git push`。如果你没有传入提交说明，它会自动使用当前时间生成一条提交信息。适合在你完成一轮修改后，快速将本地内容同步到远程仓库。

- `update_git_history.sh`

  用于根据当前仓库的 Git 提交历史重新生成 `GIT_HISTORY.md` 文件。

  中文说明：
  该脚本会读取当前仓库的提交记录，并把每次提交的哈希值、提交说明、作者、时间以及文件变更统计写入 `GIT_HISTORY.md`。适合在你提交新内容后刷新历史日志，方便后续查看和回顾。

## 常用命令

### 1. 同步当前修改到 GitHub

```bash
./sync_to_github.sh
```

如果你想自定义本次提交说明，可以这样执行：

```bash
./sync_to_github.sh "update: 这里写本次修改说明"
```

### 2. 更新 Git 历史日志文件

```bash
./update_git_history.sh
```

执行后会重新生成 `GIT_HISTORY.md`。

## 使用建议

推荐使用顺序如下：

1. 完成当前文件修改。
2. 运行 `./update_git_history.sh` 更新历史日志。
3. 运行 `./sync_to_github.sh` 将最新内容提交并推送到 GitHub。

这样可以保证仓库中的历史日志文件和远程仓库内容保持同步。
