# Contributing to City Traffic Simulation

## Setup

1. **Godot 4.7** — install from [godotengine.org](https://godotengine.org)
2. **gdtoolkit** (formatting + linting):
   ```bash
   pip3 install gdtoolkit
   ```
3. **GUT** (unit test framework) — already included in `addons/gut/`
4. **Pre-commit hook** (enforces quality gate):
   ```bash
   make install-hooks
   ```

## Workflow

For any code change:

```bash
make format     # format scripts
make lint       # lint scripts
make test       # run unit tests
make check      # all three at once
```

The git pre-commit hook runs `make check` automatically and **blocks
commits that fail**. If a commit is blocked, fix the issues and re-stage.

## Quality standards

- **Formatting**: `gdformat` is authoritative — run `make format`
- **Linting**: `gdlint` must produce zero warnings
- **Tests**: all GUT tests must pass (`make test`)
- **DRY**: no duplicated logic — extract shared classes
- **RefCounted** for data classes, **Node** only for SceneTree participants
- **Signals** for child→parent communication, not hardcoded string paths
- **snake_case** files/folders, **PascalCase** node names

## Commit conventions

- Prefix: `feat:`, `fix:`, `refactor:`, `test:`, `docs:`, `chore:`
- Present tense, imperative mood
- Subject ≤ 72 characters
- Body explains *why*

## Running the simulation

```bash
make run
```

Controls: F1 = debug overlay, F5 = regenerate grid.