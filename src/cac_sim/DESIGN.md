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
  meshes/*.mtl              minimal material for every OBJ (else gz logs 'Missing material')
  materials/sand.png, sand_normal.png, rock_<k>.png, rock_normal.png   generated textures
  truth/truth.yaml          ground truth: obstacles, zones, start pose (for test harness/scoring ONLY;
                            never read by the robot stack: "obstacle locations must not be uploaded")
  truth/map.png             top-down picture of the layout, for briefings
```

## Layout data (python dict == YAML)

```yaml
name: str
seed: int | null
boulders: [{position: [x, y], diameter: m, height: m}]
craters:  [{position: [x, y], diameter: m}]               # optional per crater: depth: m (absolute),
                                                          # depth_ratio: depth / diameter, rim: m
start_pose: {x: m, y: m, yaw_deg: deg}                    # robot base centre, in the start zone
```

The column comes from `arena.yaml`, never from the layout (it does not move between attempts). A box
column is treated as its circumscribed circle in layout checks (conservative).

## Modules (`cac_sim_gen/`, numpy + PyYAML only, Python 3.10+)

| Module | Owner | Public API |
|---|---|---|
| `config.py` | integrator | `load_config(path=None) -> dict`, `resolve(cfg, marker_preset=None) -> dict`, `load_layout(path) -> dict`, `empty_layout(cfg)`, `DEFAULT_CONFIG` path |
| `markers.py` | world | `marker_footprints(cfg)`, `post_circles(cfg) -> [(id, x, y, r)]`, `marker_placement`, `check_markers(cfg)`; shared by `layout.py` and `world.py` (no import cycle) |
| `geometry.py` | geometry | `surface_height(cfg, layout, x, y, seed) -> ndarray` (vectorised) · `build_terrain(cfg, layout, seed, res, fine_res) -> Mesh` · `rock_mesh(diameter, height, seed, sink=0.04, index=0) -> Mesh` · `crater_params(cfg, layout, i, seed) -> dict` (depth, rim, outer_radius, effective depth after the slope cap) · `write_obj(path, mesh, mtl_name=None)` (with `mtl_name` also writes the sibling `.mtl`) · `write_png(path, img)` · `sand_texture(path, seed, size, albedo)` · `sand_normal_map(...)` · `rock_texture_variants(out_dir, seed, n)` · `rock_normal_map(...)` |
| `layout.py` | randomiser | `random_layout(cfg, seed) -> layout` · `check_layout(cfg, layout, strict=True) -> list[str]` (empty = valid) · `path_exists(cfg, layout) -> (bool, list[(x, y)])` (disc) · `se2_path_exists(cfg, layout) -> (bool, list[(x, y, yaw_deg)])` (real footprint, must turn) · `random_start_pose(cfg, rng) -> dict` · `truth(cfg, layout, markers=False) -> dict` · `render_map(cfg, layout, png_path, path=None, markers=False)` |
| `world.py` | world | `write_world(cfg, layout, out_dir, markers=False, sitcams=False, overlay=False, low_gfx=False) -> Path` · `check_marker_support(cfg)` (raises `MarkerError`) · `check_markers(cfg)` (rule check) |
| `scripts/generate_arena` | world | CLI (below) |

`Mesh` is a small dataclass: `vertices (N,3) float`, `faces (M,3) int`, optional `uvs (N,2)`, `normals (N,3)`.

### CLI

```
generate_arena --layout schematic|empty|random|<file.yaml> [--seed N] [--count K] --out DIR
               [--config arena.yaml] [--markers] [--marker-preset frame_inner|perimeter_walls]
               [--sitcams] [--overlay] [--low-gfx]
```

Output on stderr: `warning [name]:` = hard problem (overlap, outside the arena, no path);
`info [name]:` = rulebook deviation only (strict check, e.g. the schematic's known ones).
`--markers` without a usable `cv2.aruco`, or with an unknown dictionary name, is an `error:` and
exit code 2; no world is written. Truth is generated only with `--out`: it is not installed.

## Conventions

- Arena frame: origin at the start-zone corner, x along 9 m, y along 5 m, z = 0 at the undisturbed sand
  surface. Gazebo world frame == arena frame.
- Craters are holes in the sand surface (negative obstacles): the terrain mesh dips, nothing is added.
- Boulders sit on the surface: the origin is sunk `boulders.sink` (1 cm) and the mesh underside is 4 cm
  below its origin, so the rock is ~5 cm into the sand and shows no gap.
- Marker collision posts are obstacles for every path check and for truth, whether or not
  `--markers` is given: the randomiser always reserves room for the configured preset's posts.
- Obstacle extent (rim included) keeps `randomiser.start_zone_margin` / `construction_zone_margin`
  from those zones; the randomiser and the strict `check_layout` use the same numbers.
- Walls are outside the 9 x 5 m interior (inner faces at x = 0, x = 9, y = 0, y = 5).
- Zones and the target box are NOT painted in the real arena (unknown), so they are only drawn with
  `--overlay` (visual-only, no collision, for debugging).
- Everything is deterministic for a given seed. Each subsystem uses its own RNG stream
  (`numpy.random.default_rng([seed, STREAM_ID, ...])`) so adding a feature never reshuffles existing layouts.
  Stream ids (all distinct; `test_rng_streams_distinct` checks seeds 0..20): layout obstacles `[s, 2]`,
  start pose `[s, 3]`, boulder yaw in `world.py` `[s, 7]`; geometry: textures 20-26 (sand etc.), crater
  rim `[s, 21, i]`, crater depth `[s, 22, i]`, rock shape `[s, 30, i]`, rock texture variants `[s, 50+k]`.
- Budgets so CPU-only laptops stay real-time: terrain visual <= 80k triangles, collision <= 20k,
  each rock <= 400 triangles (measured max ~216).
- No network access at generation or load time. No Fuel URIs.
