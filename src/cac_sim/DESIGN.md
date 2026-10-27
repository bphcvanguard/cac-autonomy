# cac_sim arena generator: design contract

Everything about the arena is generated from two inputs:

- `config/arena.yaml`: geometry and rule limits (one source of truth).
- a **layout**: where the boulders and craters are and where the robot starts. A layout is either a fixed
  YAML file (`layouts/*.yaml`) or produced by the seeded randomiser.

Output for one layout `<name>` is a self-contained directory:

```
<out>/<name>/
  arena.sdf                 Gazebo Harmonic world (SDFormat 1.9+), no network/Fuel includes
  meshes/terrain.obj        visual sand surface, craters carved in, tilled roughness, UVs
  meshes/terrain_collision.obj   coarser copy for physics
  meshes/rock_<i>.obj       one mesh per boulder
  materials/sand.png, materials/rock.png   generated textures
  truth.yaml                ground truth: obstacles, zones, start pose (for test harness/scoring ONLY;
                            never read by the robot stack: "obstacle locations must not be uploaded")
  map.png                   top-down picture of the layout, for briefings
```

## Layout data (python dict == YAML)

```yaml
name: str
seed: int | null
boulders: [{position: [x, y], diameter: m, height: m}]
craters:  [{position: [x, y], diameter: m, depth: m}]     # optional per crater: rim: m
start_pose: {x: m, y: m, yaw_deg: deg}                    # robot base centre, in the start zone
```

The column comes from `arena.yaml`, never from the layout (it does not move between attempts).

## Modules (`cac_sim_gen/`, numpy + PyYAML only, Python 3.10+)

| Module | Owner | Public API |
|---|---|---|
| `config.py` | integrator | `load_config(path=None) -> dict`, `load_layout(path) -> dict`, `DEFAULT_CONFIG` path |
| `geometry.py` | geometry | `surface_height(cfg, layout, x, y, seed) -> ndarray` (vectorised) · `build_terrain(cfg, layout, seed, res, fine_res) -> Mesh` · `rock_mesh(diameter, height, seed) -> Mesh` · `write_obj(path, mesh, mtl_name=None)` · `sand_texture(path, seed, size)` · `rock_texture(path, seed, size)` |
| `layout.py` | randomiser | `random_layout(cfg, seed) -> layout` · `check_layout(cfg, layout, strict=True) -> list[str]` (empty = valid) · `path_exists(cfg, layout) -> (bool, list[(x, y)])` · `random_start_pose(cfg, rng) -> dict` · `truth(cfg, layout) -> dict` · `render_map(cfg, layout, png_path, path=None)` |
| `world.py` | world | `write_world(cfg, layout, out_dir, markers=False, sitcams=False, overlay=False, low_gfx=False) -> Path` |
| `scripts/generate_arena` | world | CLI (below) |

`Mesh` is a small dataclass: `vertices (N,3) float`, `faces (M,3) int`, optional `uvs (N,2)`, `normals (N,3)`.

### CLI

```
generate_arena --layout schematic|empty|random|<file.yaml> [--seed N] [--count K] --out DIR
               [--markers] [--sitcams] [--overlay] [--low-gfx]
```

## Conventions

- Arena frame: origin at the start-zone corner, x along 9 m, y along 5 m, z = 0 at the undisturbed sand
  surface. Gazebo world frame == arena frame.
- Craters are holes in the sand surface (negative obstacles): the terrain mesh dips, nothing is added.
- Boulders sit on the surface, sunk 2-4 cm so there is no visible gap.
- Walls are outside the 9 x 5 m interior (inner faces at x = 0, x = 9, y = 0, y = 5).
- Zones and the target box are NOT painted in the real arena (unknown), so they are only drawn with
  `--overlay` (visual-only, no collision, for debugging).
- Everything is deterministic for a given seed. Each subsystem uses its own RNG stream
  (`numpy.random.default_rng([seed, STREAM_ID])`) so adding a feature never reshuffles existing layouts.
- Budgets so CPU-only laptops stay real-time: terrain visual <= 80k triangles, collision <= 20k,
  each rock <= 400.
- No network access at generation or load time. No Fuel URIs.
