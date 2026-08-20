# GIT_ADVICE.md — working on this repo, two people

Practical git for this project. Not a tutorial — just the parts you will use.

---

## The 6 commands that cover 95% of daily work

```bash
git pull                          # 1. ALWAYS do this first, every session
git status                        # 2. what did I change?
git add -A                        # 3. stage everything
git commit -m "what I did"        # 4. save locally
git push                          # 5. send to GitHub
git log --oneline -10             # 6. what happened recently
```

**The rhythm:** `pull` → work → `add` → `commit` → `push`. Every time.

## The one rule that prevents most pain

> **`git pull` before you start. `git push` when you stop.**

Conflicts happen when two people edit the same file *without pulling first*.
Pulling often keeps conflicts small and rare.

---

## Two people: use branches, don't both push to `main`

```bash
git checkout -b armin/ros2-port      # start a piece of work
# ...edit files...
git add -A
git commit -m "port scene_observer to rclpy"
git push -u origin armin/ros2-port   # -u only needed on the first push of a branch
```

Then open a **Pull Request** on GitHub. The other person reviews and merges.

**Why bother:** `main` always stays working. If a branch breaks something, `main`
is untouched and the other person is not blocked.

**Branch naming:** `harish/...` and `armin/...` — instantly obvious whose it is.

### Switching between branches

```bash
git branch                    # list local branches, * marks current
git checkout main             # switch back to main
git pull                      # get whatever was merged while you were away
git checkout -b harish/next   # start the next piece
```

### After your PR is merged, clean up

```bash
git checkout main
git pull
git branch -d armin/ros2-port          # delete local copy
git push origin --delete armin/ros2-port   # delete on GitHub
```

---

## Split the work so you rarely touch the same files

Branches *manage* conflicts. Not editing the same file *avoids* them. Based on
where this project currently stands:

| Harish | Armin |
|---|---|
| ROS 2 port (`pragmabot_node.py`, `scene_observer.py`) | Memory experiments (`config.yaml`, LTM CSVs, results) |
| `ros2_ws/franka_ros2/pragmabot_bridge/` | `pragmabot/calibration/` |

Agree on who owns what *before* starting, not after a conflict.

---

## When something goes wrong

| Situation | Command |
|---|---|
| Undo changes to a file (not yet committed) | `git restore path/to/file` |
| Unstage a file but keep the edits | `git restore --staged path/to/file` |
| Fix the last commit message | `git commit --amend -m "better message"` |
| See exactly what you changed | `git diff` |
| See what you changed vs a branch | `git diff main` |
| Push rejected, "behind remote" | `git pull` then `git push` |
| Accidentally on the wrong branch | `git stash`, `git checkout right-branch`, `git stash pop` |
| Want to throw away ALL local changes | `git reset --hard` ⚠️ destroys work, no undo |

### Merge conflicts

Git marks the file like this:

```
<<<<<<< HEAD
your version
=======
their version
>>>>>>> armin/ros2-port
```

Edit the file so it reads correctly, **delete all three marker lines**, then:

```bash
git add path/to/file
git commit
```

That's it. A conflict is not an error — it's git asking you to decide.

---

## Things not to do

- **Never `git push --force` on `main`.** It can erase the other person's work.
  If you think you need it, ask first.
- **Never commit generated or huge files.** `.gitignore` already blocks
  `.venv/`, `build/`, `install/`, `log/`, `*.pth`, `*.pt`, `*.db3`. Keep it that
  way — one committed 866 MB checkpoint stays in history forever, even after
  deletion.
- **Don't commit `.env`.** It holds your personal `USER_UID`/`USER_GID`.
  `.env.example` is the tracked template.

---

## Repo-specific notes

**Line endings.** The root `.gitattributes` forces `eol=lf` on all shell
scripts. Without it, Windows checkouts produce CRLF and bash fails with
`$'\r': command not found`. Don't remove it.

**The vendored third-party trees** (`GraspGen/`, `groundedsam/`,
`zed_ros2_ws/src/zed-ros2-wrapper/`, `ros2_ws/franka_ros2/`) were brought in
with `git subtree`. Edit them normally — they are just files now. To pull
upstream updates later:

```bash
git subtree pull --prefix=GraspGen https://github.com/NVlabs/GraspGen.git main --squash
```

**Upstream pragmabot.** This repo descends from the official release commit
`ee68710`. To see everything added on top:

```bash
git remote add upstream https://github.com/leggedrobotics/pragmabot.git
git fetch upstream
git diff --stat upstream/main..HEAD
```

---

## Two habits worth building

1. **Commit small and often.** Ten small commits beat one giant one — easier to
   review, easier to undo, easier to find where a bug came from.
2. **Write messages that say *what*, not *how*.** `"fix grasp frame transform"`
   beats `"update file"`. Future-you reading `git log` will thank you.

---

## If you remember one thing

**`git pull` before you start. `git push` before you stop.**
