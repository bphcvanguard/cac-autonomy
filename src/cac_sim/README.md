# cac_sim
**Owner:** Mission/Sim/Ops

Gazebo Harmonic model of the 9 x 5 m CAC 2027 sand arena (tilled sand, boulders, craters, central column, walls) and a seeded randomiser for obstacles and the start pose. Design contract: `DESIGN.md`.

| Robot view from the start zone | Overview | Layout (`truth/map.png`) |
|---|---|---|
| ![robot view](docs/arena_robot_view.png) | ![overview](docs/arena_overview.png) | ![map](docs/arena_map.png) |

## Run a world

`colcon build` generates and installs these worlds: `schematic` (organiser's example layout), `empty`, `seed_0001` to `seed_0005`.

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
| `--markers` | ArUco boards from `arena.yaml` (needs `python3-opencv`, else skipped with a warning) |
| `--marker-preset NAME` | `frame_inner` (start zone's inner edges, default) or `perimeter_walls` (the west and south walls). The rule text allows both readings; see `arena.yaml` |
| `--sitcams` | the two situational cameras, 1280x720 at 5 Hz, `/sitcam/<name>/image` |
| `--overlay` | zones, target box and start footprint drawn on the sand (visual only, for debugging) |
| `--low-gfx` | no shadows at all, for slow laptops (lighting is otherwise unchanged) |

Each world directory holds `arena.sdf`, `meshes/`, `materials/` and a `truth/` sub-directory (`truth.yaml`, `map.png`). Marker boards are visual only; free-standing ones get a thin collision post and are moved just clear of the start zone. The CLI warns if the marker list breaks the rules (max 4, below 0.60 m).

Lighting imitates the indoor hall of the Artemis Arena: a dim directional key plus four overhead lamps 4.5 m above the sand, a high ambient so shadowed sides are not black, a neutral grey background and a visual-only grey hall floor starting 0.3 m outside the walls (no collision). Only the key casts shadows (one shadow pass, cheap enough for CPU laptops); `--low-gfx` turns even that off. Walls are beige, the column concrete grey. Rocks use three texture variants plus normal maps (sand and rock).

## Ground truth rule

`truth/truth.yaml` lists every obstacle, zone and the start pose, and `truth/map.png` draws them. They are for the test harness and scoring **only**. The robot stack must never read them: the rules forbid uploading obstacle locations, and the robot has to find obstacles with its own sensors. The CLI prints a warning to that effect on every run.

The installed worlds (`share/cac_sim/worlds/<name>/`) contain no truth. The truth for the default worlds is installed separately as `share/cac_sim/truth/<name>.yaml` and `<name>.png`.

## Assumptions

Everything the rulebook does not give us (wall height, column position and size, rock heights, crater depth, marker size, camera positions, and so on) is marked `ASSUMED` in `config/arena.yaml`, with where the guess comes from. The CAC arena text is copied from the NASA Lunabotics guidebook, so the Lunabotics Artemis Arena is the reference for the guesses: a square ~0.5 m column near the centre, low walls ~0.5 m, and an optional PVC duct along the walls (`wall.inner_pipe_diameter`, off by default). Change it there and rebuild; nothing is hard-coded elsewhere. Where the rulebook table and the schematic disagree (berm and target size), the smaller pair is the default.

## Tests

`colcon test --packages-select cac_sim` (pytest in `test/`).

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
