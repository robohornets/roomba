# 1209 Public Robot Code 2026
[![Build Robot Code](https://github.com/robohornets/roomba/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/robohornets/roomba/actions/workflows/main.yml)

Our robot, codenamed roomba, has its full code publicly available. It is licensed under MIT, so fee free to use or modify the code in any way you want.

## Contributing
You cannot commit directly to the main branch. Instead, you must make a new branch and then create a pull request to merge with the main branch.

Additionally, to merge your pull request, your code must compile. This is managed by GitHub Actions which will test running the code before merging.

### Clear extra branches in VSCode
This will delete the local branches without remotes from VSCode as well as updating with the current remote list.

```
git fetch --prune && git branch -vv | grep ': gone]' | awk '{print $1}' | xargs git branch -d
```