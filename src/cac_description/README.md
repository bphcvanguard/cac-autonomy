# cac_description
**Owners:** Platform (body, wheels) · Perception (sensor mounting)

URDF of the **old Vanguard rover**: a six-wheel rocker-bogie with skid-steer and no steering joints.
It is generated from the team's CAD (`01Design_25.step`, a Fusion 360 export) by `tools/build_urdf.py`.
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
  - Each bogie is rotated until its wheels line up: +2.36° left and +2.38° right from the CAD.
  - All six wheels then touch the ground (`test_urdf.py` checks this to within 2 mm).
  - **The body sits pitched 6.0° nose-down** on flat ground (`base_footprint → base_link`). That is what the CAD geometry gives. Check it against the real rover; if the real body is level, the CAD's rocker or bogie angles are off.
- **Size standing:** 1.38 m long × 0.67 m wide × 0.92 m tall. The rocker pivots are 0.72 m above the ground.
- **Front:** the CAD's −x end (rod/wheel "1" end, the bogies lead). Rebuild with `--front=+x` to flip it.

## Masses: ASSUMED until measured

The STEP has no materials, so the 27.9 kg total comes from assumed densities (`DENSITY` in `tools/build_urdf.py`).

When the rover has been weighed, regenerate with the measured total:

```bash
python3 tools/build_urdf.py cad/parts.json <parts_m dir> . --front=-x --total-mass <KG>
```

That keeps each part's share of the mass and scales every inertia to match.

## Two URDFs: which one to use

| File | Use it for | Differential | Wheels |
|---|---|---|---|
| `urdf/rover.urdf` | RViz, TF, and suspension studies on Bullet | real: mimic joints | no friction tags |
| `urdf/rover_sim.urdf` | **Driving in Gazebo** (the `cac_sim` arena runs on DART) | rockers and diff bar **locked**; bogies still move | lateral 0.2 / rolling 1.0, `fdir1` = axle |

Both files have the same links, frames, meshes, inertias and collisions; a test checks this.
They are generated together.

**Why two files.** Gazebo Harmonic has no physics engine that does both things this rover needs (all measured):

- **DART ignores mimic joints.** The full rover's free differential lets the body tip over: 34° backwards on flat ground. With the rockers locked (`rover_sim.urdf`), DART drives well:
  - turn in place: 360.4° of 360°, and odometry matches exactly at the real 0.570 m track;
  - 1 m arc: 1.036 m radius;
  - 11 cm crater: crosses 6 out of 6 times;
  - off-centre boulder: climbs it with under 10° of roll.
- **Bullet-Featherstone enforces the mimics,** so the real differential works. Checked with a 10 cm block under the left front wheel: rockers −0.0299 / +0.0299 rad, diff bar 0.0123 rad (= 0.412 × 0.0299). **But it cannot skid-steer:**
  - turn in place: 0.2° of 360°;
  - even with spinning friction off, only 53–93°, because Bullet-Featherstone fixes friction directions to the world axes;
  - it got stuck in an 11 cm crater.

  Use it only to study the suspension.
- **DART cannot model the real linkage as a closed loop either** (gz-physics drops loop joints).

**Wheel friction must be anisotropic.** The wheels sit +0.87 to −0.29 m fore-aft on a 0.57 m track. With equal friction in every direction, the rover cannot turn in place on either engine. Lateral friction above about 0.3 turned 0°; 0.2 / 1.0 turns cleanly. This is ordinary friction physics, so the real rover on real sand has the same problem in a milder form.

To run the full model on Bullet, start Gazebo with `--physics-engine gz-physics-bullet-featherstone-plugin`, or name the plugin in the world's Physics system. `<physics type>` alone still loads DART.

- **Collision shapes:**
  - Wheels are exact cylinders.
  - Every solid part of the body, rockers, bogies and diff bar over 15 cm³ is an oriented bounding box (39 boxes).
  - Pins and bolts have no collision.
  - The full meshes are for visuals only: using them as collisions ran the arena at 0.05× real time.
- **No drive plugin is included.** The skid-steer controller belongs to the Platform pod.
- **Drive setup measured in the arena:** wheel separation 0.57 m and wheel radius 0.11 m. The skid-steer controller belongs to the Platform pod; see `cac_sim`'s README, "How the sand drives".

## Regenerating from the CAD

The pipeline needs FreeCAD (flatpak `org.freecad.FreeCAD`) for the first two steps only:

1. `tools/inspect_step.py`: run in FreeCADCmd. Lists every solid part with its placement, size and volume (`cad/parts.json`).
2. `tools/export_parts.py`: run in FreeCADCmd. Writes one STL per part, in metres, in CAD coordinates.
3. `tools/classify.py`: assigns each part to a link **by position**. The CAD's left/right names are not consistent (for example `Rod 1L` and `Plate 2L` are on opposite sides).
4. `tools/build_urdf.py`: rotates the bogies into stance, builds each link's mesh, mass, centre of mass and inertia (exact from the mesh volumes), and writes the oriented-box collisions, the joints and `urdf/build_summary.json`.
5. `tools/fk_check.py`: renders the zero pose and prints how far each wheel's bottom is from the ground.

Example run for step 1:

```bash
flatpak run --command=FreeCADCmd --filesystem=home org.freecad.FreeCAD tools/inspect_step.py \
  01Design_25.step cad/parts.json
```

## Open items

- **Total mass:** to be measured. Rebuild with `--total-mass`.
- **Real stance pitch:** does the real body sit nose-down 6° on flat ground?
- **Sensors:** none are mounted yet. Perception adds the camera/LiDAR links and their extrinsics here.
- **Height:** 0.92 m tall standing. That breaks the 0.75 m stowed height on Unstop and the schematic, but is within the rulebook's 1.0 m. Organiser question RULES.md §8 Q5.
