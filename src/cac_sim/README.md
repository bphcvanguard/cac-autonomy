# cac_sim
**Owner:** Mission/Sim/Ops

Gazebo Harmonic model of the 9 x 5 m CAC 2027 sand arena (tilled sand, boulders, craters, central column, walls) and a seeded randomiser for obstacles and the start pose. Design contract: `DESIGN.md`.

## Run a world

`colcon build` generates and installs these worlds: `schematic` (organiser's example layout), `empty`, `seed_0001` to `seed_0005`.

```bash
source install/setup.bash
gz sim -r $(ros2 pkg prefix cac_sim)/share/cac_sim/worlds/schematic/arena.sdf
gz sim -s -r <world>      # headless server, no GUI
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
| `--sitcams` | the two situational cameras, 1280x720 at 5 Hz, `/sitcam/<name>/image` |
| `--overlay` | zones, target box and start footprint drawn on the sand (visual only, for debugging) |
| `--low-gfx` | no shadows, for slow laptops |

Each world directory holds `arena.sdf`, `meshes/`, `materials/`, `truth.yaml` and `map.png`.

## truth.yaml

`truth.yaml` lists every obstacle, zone and the start pose. It is for the test harness and scoring **only**. The robot stack must never read it: the rules say obstacle locations must not be uploaded. The robot has to find obstacles with its own sensors.

## Assumptions

Everything the rulebook does not give us (wall height, column position and size, rock heights, marker size, camera positions, and so on) is marked `ASSUMED` in `config/arena.yaml`. Change it there and rebuild; nothing is hard-coded elsewhere. Where the rulebook table and the schematic disagree (berm and target size), the smaller pair is the default.

## Tests

`colcon test --packages-select cac_sim` (pytest in `test/`).
