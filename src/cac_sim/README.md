# cac_sim
**Owner:** Mission/Sim/Ops

Gazebo Harmonic model of the 9 x 5 m CAC 2027 sand arena (tilled sand, boulders, craters, central column, walls) and a seeded randomiser for obstacles and the start pose. Design contract: `DESIGN.md`.

## Words used here

- **Arena frame / origin:** the coordinate system of the arena. Origin (0, 0) is the inner corner where the west and south walls meet, at the start-zone corner. x runs along the 9 m side, y along the 5 m side.
- **Zones:** `start` (2 x 2 m, no obstacles), `obstacle` (the first 5 m), `excavation` (the last 4 m, where sand may be dug), `construction` (where the berm is built).
- **Truth:** the file (`truth/truth.yaml`) with the true obstacle positions. For scoring and setup only. The robot never reads it.
- **ASSUMED:** a value the rulebook does not give. Our best guess; change it in `config/arena.yaml`.
- **Slalom:** the judges must force a route that weaves between obstacles.
- **SE(2) path check:** a check that a 1.5 x 0.75 m rover can actually drive AND turn through the field.
- **Seed:** a number that fixes the random layout. Same seed, same arena.

| Robot view from the start zone | Overview | Layout (`truth/map.png`) |
|---|---|---|
| ![robot view](docs/arena_robot_view.png) | ![overview](docs/arena_overview.png) | ![map](docs/arena_map.png) |

## Run a world

`colcon build` generates and installs these worlds: `schematic` (organiser's example layout), `empty`, `seed_0001` to `seed_0005`.

Installed worlds live at `$(ros2 pkg prefix cac_sim)/share/cac_sim/worlds/<name>/arena.sdf`. `seed_0001` to `seed_0005` have the same `arena.sdf` as `generate_arena --layout random --seed 1..5` output (checked for seed 1; the installed copy has no `truth/`).

```bash
source install/setup.bash
gz sim -r $(ros2 pkg prefix cac_sim)/share/cac_sim/worlds/schematic/arena.sdf
gz sim -s -r <world>      # headless server, no GUI
gz sim -s -r --headless-rendering <world>   # headless, but cameras/LiDAR still render (slow PCs)
```

**Give every Gazebo its own partition.** Gazebo finds other Gazebo instances on the same network
automatically, so two teammates on the same Wi-Fi (or two sims on one PC) silently connect: your
window shows someone else's world, or your robot gets their commands. Put this in your `~/.bashrc`
(inside the container too) with your own name:

```bash
export GZ_PARTITION=$USER-$(hostname)
```

Arena frame: origin at the start-zone corner, x along the 9 m side, y along the 5 m side, z = 0 at the undisturbed sand. The Gazebo world frame is the same frame.

## Generate new arenas

```bash
ros2 run cac_sim generate_arena --layout random --seed 42 --out /tmp/arenas
ros2 run cac_sim generate_arena --layout random --seed 100 --count 20 --out /tmp/arenas   # seeds 100..119
ros2 run cac_sim generate_arena --layout my_layout.yaml --out /tmp/arenas
```

From the source tree, `src/cac_sim/scripts/generate_arena` works the same. It prints the path and the `gz sim` command for each world.

| Flag | Effect |
|---|---|
| `--layout` | `schematic`, `empty`, `random` or a layout YAML file |
| `--seed N`, `--count K` | random layouts for seeds N .. N+K-1 (same seed = same arena) |
| `--out DIR` | one sub-directory per world; the SDF uses relative paths, so it can be moved |
| `--markers` | ArUco boards from `arena.yaml` (needs `python3-opencv`; without it, or with a wrong dictionary name, the CLI prints an error to stderr and exits non-zero) |
| `--config FILE` | `arena.yaml` override (default: the packaged one) |
| `--marker-preset NAME` | `frame_inner` (start zone's inner edges, default) or `perimeter_walls` (the west and south walls). The rule text allows both readings; see `arena.yaml` |
| `--setup-sheet` | also writes `truth/setup_sheet.md` and `truth/setup_plan.png`, so a physical mock arena can copy the layout (see below). Nothing else changes. Reveals obstacle positions, so it stays with the truth |
| `--sitcams` | the two situational cameras, 1280x720 at 5 Hz, `/sitcam/<name>/image` |
| `--overlay` | zones, target box and start footprint drawn on the sand (visual only, for debugging) |
| `--low-gfx` | no shadows at all, for slow laptops (lighting is otherwise unchanged) |
| `--vary-appearance` | off by default. Seeded change of the look only: light level, key direction, ambient, colour temperature, sand tint/brightness/contrast, rock palette, wall colour. Geometry, collision, layout and truth stay byte-identical. Ranges (all ASSUMED) in `arena.yaml` `appearance:` |
| `--appearance-seed K` | seed for the look only (implies `--vary-appearance`); default is the layout seed, so "seed 7" stays one fixed scene |

Output folders are named `seed_0007` (seed 7, four digits) or `schematic` / `empty` / the layout file's name. Each world directory holds `arena.sdf`, `meshes/`, `materials/` and a `truth/` sub-directory (`truth.yaml`, `map.png`). A smaller mock arena should use a custom `--config` (rocks and craters do not scale down).

### Building a field in the mock arena

`setup_sheet.md` is a tape-measure checklist: every obstacle sorted by x, in cm from the west-south inner corner, plus the start pose, column, markers (with `--markers`) and pre-run checks.
`setup_plan.png` is the labelled plan (50 cm grid, each obstacle marked with its id).
Peg each centre, place the obstacle, tick the row, then remove the pegs before the robot runs.

Marker boards are visual only; free-standing ones get a thin collision post and are moved just clear of the start zone. The CLI warns if the marker list breaks the rules (max 4, below 0.60 m). The posts are real obstacles: the randomiser and all path checks (and the truth paths) always keep room for the configured preset's posts, with or without `--markers`.

CLI messages go to stderr: `warning [name]` = a hard problem (overlapping obstacles, an obstacle centre outside the arena, no path), `info [name]` = a deviation from the rulebook or our margins only, such as a crater rim reaching past a wall (the schematic prints a few of these: it is the organiser's drawing). Obstacles keep `randomiser.start_zone_margin` / `construction_zone_margin` (in `arena.yaml`) from those zones.

Lighting imitates the indoor hall of the Artemis Arena: a dim directional key plus four overhead lamps 4.5 m above the sand, a high ambient so shadowed sides are not black, a neutral grey background and a visual-only grey hall floor starting 0.3 m outside the walls (no collision). Only the key casts shadows (one shadow pass, cheap enough for CPU laptops); `--low-gfx` turns even that off. Walls are beige, the column concrete grey. Rocks use three texture variants plus normal maps (sand and rock).

Use `--vary-appearance` to test perception against venue conditions you cannot know in advance (dim or warm hall, greyer sand, other wall paint). The same layout with `--appearance-seed 1..N` gives N looks of one fixed obstacle field.

## Ground truth rule

`truth/truth.yaml` lists every obstacle, zone and the start pose, and `truth/map.png` draws them. They are for the test harness and scoring **only**. The robot stack must never read them: the rules forbid uploading obstacle locations, and the robot has to find obstacles with its own sensors. The CLI prints a warning to that effect on every run.

The installed worlds (`share/cac_sim/worlds/<name>/`) contain no truth, and truth is not installed anywhere (any node can read `share/`). Truth is generated only with `--out`: the harness generates the arena it tests on and keeps the truth itself.

## Assumptions

Everything the rulebook does not give us (wall height, column position and size, rock heights, crater depth, marker size, camera positions, and so on) is marked `ASSUMED` in `config/arena.yaml`, with where the guess comes from. The CAC arena text is copied from the NASA Lunabotics guidebook, so the Lunabotics Artemis Arena is the reference for the guesses: a square ~0.5 m column near the centre, low walls ~0.5 m, and an optional PVC duct along the walls (`wall.inner_pipe_diameter`, off by default). Geometry, rule limits, randomiser margins, sand friction, rock sink and the situational-camera sensor (fov, resolution, rate, clip) are all in that file; change it there and rebuild. Only the lighting rig (`KEY_DIFFUSE`, `LAMP_DIFFUSE`, `AMBIENT`, `BACKGROUND`, lamp height) and the colours of walls, column, floors and hall are named constants at the top of `cac_sim_gen/world.py`; `--vary-appearance` scales those same constants. Where the rulebook table and the schematic disagree (berm and target size), the smaller pair is the default.

## Tests

`colcon test --packages-select cac_sim` (pytest in `test/`). The randomiser tests use 60 seeds so CI
stays quick. After changing the randomiser or the limits in `arena.yaml`, run the full sweep:
`CAC_SIM_SEEDS=500 colcon test --packages-select cac_sim` (about 3 minutes).

## What the sensors see (measured in this sim)

Craters are shallow bowls: 3-12 cm deep for 30-50 cm wide (see `arena.yaml` for why). That makes them
**much harder to see than boulders**. Measured with a static sensor 0.5 m above the sand:

| Crater | Depth camera (640x480, 87 deg, pitched 20 deg) | 16-beam LiDAR (tilted 15 deg) |
|---|---|---|
| 30 cm wide, 3-6 cm deep | seen from 1.0-2.0 m | seen from 1.0 m |
| 40 cm wide, 4-8 cm deep | seen from 2.0-3.0 m | seen from 1.0 m |
| 50 cm wide, 5-10 cm deep | seen from 3.0 m | seen from 1.5-2.0 m |

- A 30 cm boulder (12 cm tall) shows up at 3.5 m on the depth camera. The LiDAR loses it at 3.5 m,
  because it falls between beam rings.
- The tilled sand is about 4 mm RMS. No flat-sand point fell below -3 cm in any test.
- These numbers are from the sim, with sim noise. They show the shape of the problem, not the real
  sensor's performance. Re-measure on real sand.
