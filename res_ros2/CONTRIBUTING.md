# Contribution Guidelines

## Style Guide

All checks are enforced by [`pre-commit`](https://pre-commit.com/).

### Commit Style

`commitlint` is enabled for this project. This is checked whenever a commit is made.
The style for the commit is configured to be **Conventional Commits**.

More on the conventional commit format can be found at <https://www.conventionalcommits.org/en/v1.0.0>

### Code Linter

This project uses [`pre-commit`](https://pre-commit.com/) and [`ruff`](https://docs.astral.sh/ruff/) as the linters.
To run them

```bash
uv run pre-commit run --all-files
```

`pre-commit` downloads `ruff` and other package within the first run.
It will take some time.

## Install Pre-commit hooks

This will enable the run of pre-commit whenever you try to make a commit
```bash
uv run pre-commit install
```
