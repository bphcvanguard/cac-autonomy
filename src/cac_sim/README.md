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

`colcon build` generates and installs these worlds: `schematic` (organiser's example layout), `empty`, the five stress layouts (`gate`, `column_pinch`, `crater_field`, `busy_dig`, `start_trap`; see below), `seed_0001` to `seed_0005`.

Installed worlds live at `$(ros2 pkg prefix cac_sim)/share/cac_sim/worlds/<name>/arena.sdf`. `seed_0001` to `seed_0005` have the same `arena.sdf` as `generate_arena --layout random --seed 1..5` output (checked for seed 1; the installed copy has no `truth/`).

```bash
source install/setup.bash
gz sim -r $(ros2 pkg prefix cac_sim)/share/cac_sim/worlds/schematic/arena.sdf
gz sim -s -r <world>      # headless server, no GUI
gz sim -s -r --headless-rendering <world>   # headless, but cameras/LiDAR still render (slow PCs)
```

The worlds select bullet-featherstone themselves (no `--physics-engine` needed); see "Physics engine".

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
| `--layout` | `schematic`, `empty`, `random`, a stress layout name (below) or a layout YAML file |
| `--seed N`, `--count K` | random layouts for seeds N .. N+K-1 (same seed = same arena) |
| `--out DIR` | one sub-directory per world; the SDF uses relative paths, so it can be moved |
| `--markers` | ArUco boards from `arena.yaml` (needs `python3-opencv`; without it, or with a wrong dictionary name, the CLI prints an error to stderr and exits non-zero) |
| `--config FILE` | `arena.yaml` override (default: the packaged one) |
| `--marker-preset NAME` | `frame_inner` (start zone's inner edges, default) or `perimeter_walls` (the west and south walls). The rule text allows both readings; see `arena.yaml` |
| `--setup-sheet` | also writes `truth/setup_sheet.md` and `truth/setup_plan.png`, so a physical mock arena can copy the layout (see below). Nothing else changes. Reveals obstacle positions, so it stays with the truth |
| `--sitcams` | the two situational cameras, 1280x720 at 5 Hz, `/sitcam/<name>/image` |
| `--overlay` | zones, target box and start footprint drawn on the sand (visual only, for debugging) |
| `--low-gfx` | no shadows at all, for slow laptops (lighting is otherwise unchanged) |
| `--sand-collision mesh\|tiles` | override `sand.collision` (default `tiles`: flat sand is boxes, craters are mesh patches). `mesh`: one collision mesh with the tilled relief; on bullet-featherstone ~40 % slower physics, on dart friction-inert. See "How the sand drives" |
| `--physics-engine bullet-featherstone\|dart` | override `physics.engine` (default `bullet-featherstone`: our rover's mimic joints need it). Sets `<physics type>` and the Physics system's `<engine>` plugin. See "Physics engine" |
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

## Stress-test layouts

Hand-made, rule-legal layouts in `layouts/` that each push one hard case. Every one passes the hard
checks and leaves a route for the 1.5 x 0.75 m footprint; the comment block at the top of each file
says what it stresses, why, and what a good robot does. Run one by name (installed worlds are in
`share/cac_sim/worlds/<name>/`; from the source tree use `src/cac_sim/scripts/generate_arena`):

```bash
ros2 run cac_sim generate_arena --layout gate --out /tmp/arenas
gz sim -r /tmp/arenas/gate/arena.sdf
```

| Name | What it stresses | Start pose (x, y, heading) |
|---|---|---|
| `gate` | A barrier across the obstacle zone with ONE gap, 5 cm wider than the SE(2) minimum for the rover: precise planning, costmap inflation | (1.0, 1.4), east |
| `column_pinch` | The only route is the lane between the column and a shallow crater: negative-obstacle detection next to a tall obstacle | (1.0, 1.4), east |
| `crater_field` | 7 shallow craters (depth ratio 0.10-0.12) on the travel route, few boulders: crater detection, the hardest perception case | (1.0, 1.0), east |
| `busy_dig` | Normal obstacle zone, but the excavation zone holds the maximum boulders and craters around the dig spot and haul route: dig spot choice, dig to berm haul | (1.0, 1.0), east |
| `start_trap` | Boulders right outside the start zone, one exit to the north, start facing the west wall: turning in place, leaving the start zone | (1.0, 1.0), west |

`gate` is the only one that breaks a strict rule (its gap is a straight corridor, so the 'slalom' rule
reports it, on purpose). `test/test_stress_layouts.py` checks every one, and that the gate stays
within 10 cm of the SE(2) minimum if the limits in `arena.yaml` change.

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

## Physics engine

The worlds run on **bullet-featherstone** by default (`physics.engine` in `arena.yaml`, or
`generate_arena --physics-engine dart`). It is a team decision driven by our rover: the
`cac_description` rocker-bogie differential is URDF mimic joints, and gz-sim 8's default dartsim
does not support mimic constraints ("the chosen physics engine does not support mimic
constraints"). On dart the rover tips over (measured here: pitch -34.4 deg in the arena);
bullet-featherstone enforces the mimics and it stands level.

**`<physics type>` alone does not pick the engine in gz-sim 8.** Measured with `gz sim -v 4`:
a world with `type="bullet-featherstone"` and a bare Physics plugin still loaded
`gz::physics::dartsim::Plugin`. So the generated world also names the engine plugin in the
Physics system:

```xml
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
  <engine><filename>gz-physics-bullet-featherstone-plugin</filename></engine>
</plugin>
```

With that, plain `gz sim -r arena.sdf` loads bullet-featherstone; you do **not** need
`--physics-engine` on the command line. `gz sim --physics-engine gz-physics-dartsim-plugin ...`
still overrides the world file (measured), for a one-off dart run. All 12 installed worlds load
on bullet-featherstone with 0 `[Err]` / 0 `[Wrn]` (`-v 4`; it prints one `[Dbg]` that
`CollisionPairMaxContacts` is unsupported).

Speed, measured with the `cac_description` rover spawned in the start zone and shuttling at
0.2 m/s (gz `DiffDrive`), headless, 2 pinned cores (`--cpuset-cpus 0,1`), CPU only
(`LIBGL_ALWAYS_SOFTWARE=1`), RTF over 20 s:

| World | bullet, `tiles` (default) | bullet, `mesh` | capped (`tiles`) |
|---|---|---|---|
| schematic | 4.82 / 4.93 / 5.07 | 3.04 | 1.00 |
| dense seed 16 | 5.15 / 5.22 / 5.40 | 3.07 | 1.00 |

On bullet-featherstone the whole-arena mesh is ~40 % slower than `tiles`, so `tiles` stays the
default for speed as well as friction. (dart with the same rover ran 3.7-6.0 uncapped, but the
rover had tipped over, so that is not a like-for-like number.)

## How the sand drives (measured in this sim)

**The engine decides how sand friction works.** Measured by tilting gravity to tan 0.5 (a block
slides when its effective mu < 0.5) with 0.2 m blocks on the generated schematic arena:

| | bullet-featherstone (default) | dart |
|---|---|---|
| Tag it reads | `<bullet><friction>` / `<friction2>` only (`<ode><mu>` alone: ignored, stays at 1.0) | `<ode><mu>` / `<mu2>` only |
| Sand on the tiles' boxes | honoured | honoured |
| Sand on a mesh (crater patches, `--sand-collision mesh`) | honoured (sand mu 0.3 on the mesh: block slides) | INERT: DART default 1.0 (`AttachMeshShape` drops `<surface>`) |
| Combining sand and wheel/block mu | ~0.7 x sand x block (product): block mu 1.0 holds at sand 0.75 and 0.8, sticks after 0.18 m at 0.70, slides at 0.68 and below; sand 1.0 x block 0.55 or 0.6 slides; symmetric in the two, same at 45 deg | min(sand, block): sand 0.5 holds, 0.45 slides; 0.6 x 0.6 holds |
| A collision with no friction tag (e.g. the `cac_description` wheels) | counts as 1.0 | counts as 1.0 |
| Anisotropic `fdir1` + `mu2`/`friction2` (1.0 along x, 0.3 across) | honoured (holds along, slides across) | honoured |

So every sand collision carries the friction twice, as `<ode>` and as `<bullet>`, and the
tilt test still passes on both engines: box mu 1.0 on `tiles` slides at sand mu 0.3 and holds at
sand mu 0.9. On mesh mode it slides at 0.3 on bullet and holds at both on dart.

**What it means:** with the default bullet-featherstone and the rover's untagged wheels (1.0),
the effective mu on sand is about 0.7 x `terrain.friction` = ~0.35 at the shipped 0.5, not 0.5.
If you set wheel friction on the robot, it multiplies (wheel 0.5 on sand 0.5 gives ~0.18), it does
not take the minimum as on dart.

**Craters** (`tiles`: each crater is a mesh patch out to its outer radius + 5 cm; the tilled relief
(+/- 1.2 cm) is dropped from the collision and stays in the picture only). A 5 cm sphere dropped
on each crater centre rests 0.6-1.3 mm above `-depth_eff + r` on both engines (schematic: 3
craters, `crater_field`: 8). **Rocks are solid** on bullet-featherstone: blocks slid into the 9
schematic rocks either stopped flush against them (6, front face 0.11-0.17 m short of the rock
centre) or were deflected round them (3); none passed through.

On dart a cylinder wheel gets 2 contact points on a box and 1 on the mesh. `<max_contacts>` is
not honoured by either engine.

### Older drive measurements (dart, scratch test rover)

Everything below was measured on **dart** before the switch, with a scratch test rover, and has
**not** been re-run on bullet-featherstone. The friction rules differ (table above), so treat the
numbers as the shape of the problem, not as current values. Dart, test rover shuttling, 3 runs:
uncapped RTF schematic 7.2 (tiles) vs 7.8 (mesh); dense seed 16 6.5 vs 8.2.

Test rover (scratch only, not in this package): 0.6 x 0.3 m, 18.4 kg, 4 wheels r 0.08 m,
wheelbase 0.40 m, track 0.36 m, gz `DiffDrive` skid-steer, 5 N m per wheel.

Mesh collision (`--sand-collision mesh`), sand mu inert, so the effective mu is the wheel mu:

| Run | Odometry vs truth |
|---|---|
| Straight 4 m at 0.2 m/s, flat, any sand mu or wheel mu 0.1-1.0 | 0.01 % distance, < 0.1 deg heading |
| Straight 4 m across an 11 cm deep, 50 cm crater | 0.2 % distance |
| 360 deg in place, wheel mu 1.0 (the default) | rover does not turn (0 deg); odometry says 160 deg |
| 360 deg in place, wheel mu 0.5 | turns 150 deg; odometry says 360 deg |
| 360 deg in place, wheel mu 0.4 sideways / 0.6 rolling (`fdir1` = axle) | turns 250-290 deg; odometry says 360 deg |
| Robot-side `WheelSlip` system, compliance 1 / 1 (not tuned) | 1.3 % distance, but 0.4 m sideways and 10 deg heading drift over 4 m, invisible to odometry |

With wheel mu 0.5, or with 0.4 / 0.6, the rover drove out of the 11 cm crater from its centre and
turned in place inside it.

`tiles` collision, sand mu 0.5 (now effective on flat sand):

| Run | Odometry vs truth |
|---|---|
| Straight 4 m at 0.2 m/s, flat, wheel mu 0.5 or 1.0 | 0.00 % distance, 0.0 deg heading |
| Straight 4 m across the 50 cm crater | 0.3 % distance, 0.5 deg heading (the in-place turn after it: 61 deg, marginal) |
| 360 deg in place, wheel mu 1.0 or 0.5, or sand mu 0.3 / 0.9 | rover does not turn (0 deg); odometry says 360 deg (wheels spin) |
| 360 deg in place, wheel mu 0.4 sideways / 0.6 rolling (`fdir1` = axle) | turns 316 deg; odometry says 360 deg |
| 360 deg in place, wheelbase 0.30 < track 0.40, wheel mu 1.0 | turns 247 deg (on the mesh: 29 deg) |

So no: with isotropic friction the test rover still does not turn in place, at any mu. That is plain
Coulomb physics, not a sim bug. Its wheelbase (0.40 m) is longer than its track (0.36 m), so the
sideways scrub of the wheels has a longer lever than the drive force, at any shared mu. The
wide-track rover does turn. The old "turns 150 deg at wheel mu 0.5" came from the mesh's single
contact point per wheel.

Seams (tiles): a 7.6 m diagonal drive (13.7 deg) across 3 box-box seams, 4 box-patch seams and two
craters, at 0.2 and 0.5 m/s. On flat sand the body height held at 0.0800 m (spread < 0.1 mm), speed
0.200 / 0.500 m/s, pitch 0.00 deg: no seam bumps or snags. Through the craters: body dip 4.7-5.5 mm,
pitch up to 5.4 deg, speed 0.18-0.23 m/s (at 0.2 m/s), the same as the mesh (dip 4.7-7.2 mm, 5.4 deg,
0.18-0.22 m/s). The rover crossed the full 7.6 m in both modes.

What this means for the robot team:
- **Straight-line wheel odometry is near perfect in sim (< 0.3 %), with either collision.** Real dry
  sand gives ~5-10 % of distance. Coulomb friction alone never makes a flat-ground slip like that.
  `tiles` makes the friction coefficient real on flat sand, but there is still no rolling
  resistance, sinkage, bulldozing or tilled relief under the wheels in the sim. So localisation
  tested here will look better than it will be on real sand. Add the error on purpose: a scale error plus heading drift in the odometry
  path, or tune the robot-side `gz::sim::systems::WheelSlip` plugin.
- **Turning in place is where the error shows up.** With the geometric track, the odometry
  overstates in-place yaw by 70-210 deg per turn. Calibrate an effective track, or take yaw from
  the IMU. Real skid-steer robots have the same problem.
- **Set wheel friction on the robot.** A skid-steer longer than it is wide does not turn in place
  with isotropic friction (on `tiles`, at any mu; on the mesh, at wheel mu 1.0). Use anisotropic
  friction with `<fdir1>` along the axle: lower `<mu>` sideways than `<mu2>` rolling.
- **Whether it turns depends on the friction pyramid's orientation and the contact count.** On the
  mesh at wheel mu 0.5 with `<fdir1>` along the axle (isotropic), the rover did not turn at all.
  Without `<fdir1>` it turned 150 deg. On `tiles` it did not turn either way.
  Re-measure on the real robot on real sand. These numbers show the shape of the problem only.
