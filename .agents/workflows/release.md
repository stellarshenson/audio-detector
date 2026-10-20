---
description: Release a new version, update the CHANGELOG, commit, tag and push
---

Release audio-detector.

1. Read `VERSION` and report the current version
2. Run `make increment_version`; the target prints `Version bumped: <old> -> <new>`
3. Move the `Unreleased` entries in `CHANGELOG.md` under a new `## [<version>] - <date>` heading,
   and leave `## [Unreleased]` in place
4. Commit `VERSION` and `CHANGELOG.md` with the message `chore: bump version to <version>`
5. Tag the commit `v<version>`
6. Push the commit and the tag

Ask for approval before the commit, the tag and the push. Stop at the first failure and report the
exact error text. Never delete an old tag.
