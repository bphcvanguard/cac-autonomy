# cac_description
**Owners:** Platform (body, wheels) · Perception (sensor mounting)

URDF of the **old Vanguard rover**: a six-wheel rocker-bogie with skid-steer and no steering joints.
It is generated from the team's CAD (`NEW JOINTS KESHAV ASSEMBLY.step`, 28 Oct 2026) by `tools/prep_keshav.py` and `tools/build_urdf.py`.
In that file **only the robot's right side has the new joints**, so the left side is rebuilt as an exact mirror of the right (`--mirror-from`). The rods, joints, wheels and wheel-side parts are mirrored. The body and the diff bar are untouched, and so are the pivot bolts and screws (they are body parts), so those stay one-sided.
Frames follow REP-103/105: `base_footprint → base_link → …`, with x forward, y left, z up. Units are metres and kg.

```bash
ros2 launch cac_description display.launch.py      # RViz: robot model + TF (needs a display)
```

## What's in the model

| Link | What | Joint to parent |
|---|---|---|
| `base_link` | Body (frame box). Origin is on the rocker-pivot axis, midway between the two pivots | fixed to `base_footprint` |
| `left_rocker`, `right_rocker` | Rocker arms. Each carries its **rear** wheel and its bogie | revolute about y at the body pivot |
| `left_bogie`, `right_bogie` | Bogies. Each carries the **front** and **middle** wheels | revolute about y at the bogie pin |
| `diff_bar` | Differential bar on top of the body | revolute about z, mimics the right rocker × 0.412 |
| `*_wheel` (6) | Wheels: 220 mm diameter, 60 mm wide | continuous about y |

- **Differential:** `left_rocker_joint` mimics `right_rocker_joint` × −1. Relative to the body, the rockers always turn by equal and opposite amounts, as the real differential bar forces them to. The bar's ratio, 0.412, comes from the linkage geometry: the ball-joint height above the rocker pivot (126 mm) divided by the bar's half-length (305.5 mm).
- **Zero pose = standing on flat ground:**
  - The rockers sit at their CAD angle, where the differential is neutral.
  - Each bogie is rotated -0.75° from the CAD until its wheels line up.
  - All six wheels then touch the ground (`test_urdf.py` checks this to within 2 mm).
  - The body sits 0.5° from level on flat ground: essentially level. The old CAD gave 6°; the new joints fixed it.
- **Size standing:** 1.11 m long × 0.7505 m wide × 0.68 m tall. The rocker pivots are 0.49 m above the ground. The wheelbase is 0.89 m and the track 0.65 m.
  - **The width is 0.5 mm over the rules' 0.75 m.** This comes from the mirrored left side, so measure the real rover once both sides have the new joints.
- **Front:** the CAD's −x end (rod/wheel "1" end, the bogies lead). Rebuild with `--front=+x` to flip it.

## Looks

Each link is drawn per material, in colours that follow the team's renders:
- PLA joints, brackets and plates: white;
- carbon-fibre rods: black;
- wheels: dark rubber;
- body frame: aluminium grey;
- differential bar: wood;
- pins and screws: steel;
- drive motors: dark grey.

The colours come from `MAT_COLOUR` in `tools/build_urdf.py`. Each link still has one combined mesh as well (`meshes/<link>.stl`), used by the tools and tests.

## Putting the rover in the arena

```bash
ros2 run cac_sim generate_arena --layout schematic \
  --robot $(ros2 pkg prefix cac_description)/share/cac_description/urdf/rover_sim.urdf --out /workspace/arenas
gz sim -r /workspace/arenas/schematic/arena.sdf
```

The rover is placed at the layout's start pose in the start zone. Its meshes are copied into the world folder.

## Camera: RealSense D455 on a printed bracket

**Pose.** The camera's centre is at x 0.35, y 0, z 0.68 m (in `base_footprint`), pitched 22.5° down.
The top of the camera is 0.698 m above the ground, so it is inside the 0.75 m stowed height.
The pose comes from a study (`cad/camera_mount_study.md`, figures `cad/camera_fig*.png`). It ray-cast the D455's field of view against this rover's real geometry over a sweep of positions and pitches. Results at this pose:

- **Self-occlusion:** none. 0 % of the depth rays hit the rover, even with the rover pitched 5° nose-down.
- **Blind zone:** the first ground the camera sees is 0.24 m past the front wheels.
- **Ground coverage:** 99 % of the band 0.6–4 m ahead, ±1 m to the sides.
- **Craters:** the study estimated a 40 cm × 6 cm crater is visible from about 2.4 m. The measured numbers are below.
- **Markers:** every ArUco board (centre 0.35 m) stays in the colour image from 0.5 to 8 m. The margin at 8 m is 11°, or 6° with the rover pitched nose-down. That 8 m marker is what limits the pitch to about 23°.

**Measured in the arena with this mounted camera.**

The test put the rover at set distances, took one depth frame, and back-projected it to the world. A crater counts as detected when at least 10 depth points fall more than 3 cm below the sand.

Maximum detection distance (crater straight ahead, distance from the camera's ground point, 0.5 m steps):

| Crater width \ depth | 4 cm | 6 cm | ~7–10 cm (slope-capped) |
|---|---|---|---|
| 30 cm | 2.0 m | 2.0 m | 2.5 m |
| 40 cm | 2.0 m | 2.5 m | 3.0 m |
| 50 cm | 2.5 m | 3.0 m | ≥ 3.5 m |

- With 7 mm Gaussian depth noise, the 40 × 6 cm crater is still detected at 2.5 m (32 points) and not at 3.0 m (4 points). The 50 × 6 cm crater is detected at 3.0 m (21 points).
- **Flat sand:** zero points below −3 cm from 1 to 4 m, with or without noise. The lowest is −1.9 cm and the spread about 5 mm.
- **A 30 cm × 15 cm boulder** is seen at 2, 3, 4 and 5 m (114 points at 5 m).
- **Plan the speed around it:** a small, shallow crater is only reliably seen about 2 m ahead. The crater's own rim hides its near half, so only the far floor gets sampled.

Real D455 noise grows with range and is worse on bright sand than this constant 7 mm, so expect these distances to be a little shorter on the real rover.

**Bracket.** `cad/d455_bracket.step` and `.stl` are one PLA part, about 130 cm³, in `base_link` coordinates. Its parametric FreeCAD source is `tools/d455_bracket.py`. It has three parts:

- a **saddle** over the body's front top cross member (30 × 30 mm), clamped by **2 × M5 bolts** that run fore-aft under the member;
- a **web** up to the camera plate;
- a **plate** tilted 23°, with a 1/4-20 clearance hole and a head counterbore for the D455's tripod thread, and a back lip so the camera can't turn.

**Printing and fitting:**
- Print with the saddle's open side down.
- Use a 1/4-20 screw no longer than plate + 5 mm. The plate is 6 mm thick; check the D455's thread depth.

**URDF.**
- `camera_mount` (the bracket) and `camera_link` (the D455 body) are fixed to the body.
- `camera_color_optical_frame` and `camera_depth_optical_frame` are standard optical frames: z forward, x right, y down.
- For Gazebo there are **two sensors**:
  - a colour `camera` at 90°, topic `camera/image`;
  - a `depth_camera` at 87°, 0.52–6 m, topic `camera/depth_image`.

  The combined `rgbd_camera` in gz-sim 8 renders its colour image unlit (black geometry; measured), so it is not used.
- `cad/camera_sim_rgb.png` is the simulated colour frame from the start zone: sand, column, rocks, a crater and a marker.

**To verify on the real camera:**
- The D455's size (124 × 26 × 29 mm assumed) and mass (0.39 kg assumed).
- The optical-frame offsets. On the real robot, use `realsense2_description` for the exact extrinsics; here the optical frames sit at the body centre.
- Re-run the study once the digging tool is modelled: a bucket in front could block the lower view.

**Later:** a raising and rotating mast is allowed after the start (up to +1.75 m). The bracket's plate and tripod interface can move onto it unchanged.

## Masses: from the materials, until the rover is weighed

| Material | Parts | Density used (per CAD volume) |
|---|---|---|
| PLA (team) | joints, brackets, plates, motor caps, diff bar | 744 kg/m³ = 60 % of solid PLA (walls + infill, ASSUMED) |
| Carbon fibre (team) | rods | 558 kg/m³: the CAD draws solid 20 mm bars, so this assumes a 20 × 16 mm tube |
| Steel | pivot screws and pins | 7850 kg/m³ |
| ASSUMED | drive motors (about 0.6 kg each), wheels (1.0 kg each), body (4.6 kg) | see `MATERIAL_DENSITY` in `tools/build_urdf.py` |

The total comes out at **20.8 kg**.

Once the rover has been weighed, rebuild with `--total-mass <KG>`. That keeps each part's share of the mass and scales every inertia to match.
Update `MATERIAL_DENSITY` when the real tube size, PLA infill, wheel material and motor weights are known.

## Two URDFs: which one to use

| File | Use it for | Differential | Wheels |
|---|---|---|---|
| `urdf/rover.urdf` | RViz, TF, and suspension studies on Bullet | real: mimic joints | no friction tags |
| `urdf/rover_sim.urdf` | **Driving in Gazebo** (the `cac_sim` arena runs on DART) | rockers and diff bar **locked**; bogies still move | no tags: the sand's friction sets the grip |

Both files have the same links, frames, meshes, inertias and collisions; a test checks this.
They are generated together.

**Why two files.** Gazebo Harmonic has no physics engine that does both things this rover needs (all measured):

- **DART ignores mimic joints.** The full rover's free differential lets the body tip over: 34° backwards on flat ground (old CAD). With the rockers locked (`rover_sim.urdf`), DART drives well. Measured with the new-joints rover in the arena, sand friction 0.5:

  | Test | Result |
  |---|---|
  | Turn in place, 360° commanded | 360.3–360.5°, with any wheel friction; odometry exact at the 0.647 m track |
  | Arc, 180° commanded | 180.6° |
  | Straight, 4 m | 4.001 m |
  | 11 cm crater | crosses; max roll 2.4° |
  | 15 cm steep boulder, head-on | stalls (avoid rocks anyway: touching one is a penalty) |
- **Bullet-Featherstone enforces the mimics,** so the real differential works. Checked with a 10 cm block under one wheel: rockers ∓0.0299 rad, diff bar 0.0123 rad. **But it cannot skid-steer:**
  - turn in place: 0.2° of 360° (old CAD);
  - Bullet-Featherstone fixes friction directions to the world axes.

  Use it only to study the suspension.
- **DART cannot model the real linkage as a closed loop either** (gz-physics drops loop joints).

**Geometry matters for turning.** The old CAD (wheelbase 1.16 m on a 0.57 m track) could only turn in place with anisotropic wheels (sideways 0.2, rolling 1.0, `fdir1` = axle). The new joints make the rover squarer (0.89 m on 0.65 m), and it turns with ordinary friction. If a design change makes it long and narrow again, set `SIM_WHEEL_FRICTION` in `tools/build_urdf.py`.

To run the full model on Bullet, start Gazebo with `--physics-engine gz-physics-bullet-featherstone-plugin`, or name the plugin in the world's Physics system. `<physics type>` alone still loads DART.

- **Collision shapes:**
  - Wheels are exact cylinders.
  - Every solid part of the body, rockers, bogies and diff bar over 15 cm³ is a box (62 boxes in all). Each part gets the smaller of its oriented (PCA) box and its axis-aligned box, and large hollow parts such as the body frame are split into several boxes, so the collision envelope stays inside the visual one (0.733 m wide, no lower than the visual bottom).
  - Pins and bolts have no collision.
  - The full meshes are for visuals only: using them as collisions ran the arena at 0.05× real time.
- **No drive plugin is included.** The skid-steer controller belongs to the Platform pod.
- **Drive setup measured in the arena:** wheel separation 0.647 m and wheel radius 0.11 m. The skid-steer controller belongs to the Platform pod; see `cac_sim`'s README, "How the sand drives".

## Regenerating from the CAD

The pipeline needs FreeCAD (flatpak `org.freecad.FreeCAD`) for the first two steps only:

1. `tools/inspect_step.py`: run in FreeCADCmd. Lists every solid part with its placement, size and volume.
2. `tools/export_parts.py`: run in FreeCADCmd. Writes one STL per part, in metres.
3. `tools/prep_keshav.py`: rotates this file's frame into the generator's convention and drops duplicates and hardware under 1 cm³. It also assigns each part a link (from a name table, with the side taken from position) and a material.
4. `tools/build_urdf.py --model cad/model_keshav.json --mirror-from left --front=-x`: mirrors the updated side and rotates the bogies into stance. It then builds each link's mesh, mass, centre of mass and inertia, and writes the oriented-box collisions, the joints, `rover.urdf`, `rover_sim.urdf` and `urdf/build_summary.json`.
5. `tools/fk_check.py`: renders the zero pose and prints how far each wheel's bottom is from the ground.

Example run for step 1:

```bash
flatpak run --command=FreeCADCmd --filesystem=home org.freecad.FreeCAD tools/inspect_step.py \
  "NEW JOINTS KESHAV ASSEMBLY.step" cad/parts_keshav_raw.json
```

## Open items

- **Total mass:** to be measured. Rebuild with `--total-mass`.
- **Real stance pitch:** the model sits 0.5° nose-down on flat ground. Check that against the real rover.
- **Sensors:** none are mounted yet. Perception adds the camera/LiDAR links and their extrinsics here.
- **Envelope:** 0.68 m tall standing, so it is under the 0.75 m stowed height. It is 0.7505 m wide with the mirrored side, which is 0.5 mm over 0.75 m. Measure the real rover.
- **Left side:** it is a mirror of the right until the CAD's left side has the new joints too. Once it has, rebuild without `--mirror-from`.
