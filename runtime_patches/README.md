# Runtime artifacts and patches

The repository does not include a copied virtual environment. Virtual environments contain absolute paths, caches, and platform-specific launchers and are not portable.

Instead, this branch archives only the runtime pieces that differ from a normal package installation:

- `Carla_module/carla-0.9.15-cp310-cp310-linux_x86_64.whl`: the CARLA Python API compiled for CPython 3.10 on x86_64 Linux.
- `carla-0.9.15/linuxmint-setup.patch`: the one-line CARLA build-source compatibility change for Linux Mint.
- `commonroad-scenario-designer-0.8.4/`: present on variants whose shared environment contained the modified Scenario Designer converter.

The CARLA wheel is platform-specific. Rebuild it when using another Python ABI, CPU architecture, operating system, C++ runtime, or materially different CARLA server revision.

The CommonRoad backup contains the complete modified file and a minimal patch. Install `commonroad-scenario-designer==0.8.4` first, then copy the backed-up `crdesigner/` subtree into the active environment as described in `docs/SETUP.md`.
