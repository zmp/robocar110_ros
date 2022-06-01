# Update
## Source Code

If you have some source code in `~/ros/src/robocar110_ros/`, try to do the following to update it.

```
cd ~/ros/src/robocar110_ros/
git fetch
```

First, if `git status` command shows changes, you can either remove them:
```
git reset --hard origin/master
```
, or save them:
```
git checkout -b new_branch
git add -A
git commit -m "commit message"

git checkout master
```

Then you can update the code with:
```
git pull
```

That's basic git usage. For more details, you can check [the book](https://git-scm.com/book/en/v2) for example, which is available on multiple languages.

## Binaries
If you have problems with `rc110_robot_*.run` file or you want another version installed, it's possible to do it with the following commands:
```
make deps
make install
```
