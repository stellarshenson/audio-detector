# Releasing audio-detector

The version lives in `VERSION`; `host/pyproject.toml` keeps its own version. A release increments the
patch version, moves the changelog entries under the new heading, commits both files, tags the
commit and pushes it.

## Release process

1. Run `make increment_version`
2. Move the `Unreleased` entries in `CHANGELOG.md` under the new version heading with the date
3. Commit `VERSION` and `CHANGELOG.md` with the message `chore: bump version to <version>`
4. Tag the commit `v<version>`
5. Push the commit and the tag

## Rules

- Every tag carries the `v` prefix, for example `v0.1.1`
- Never delete an old tag; `copier update` resolves a template version from the tags
- Tag only after `make test` passes
