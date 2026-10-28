# D455 fixed mount on the CAC 2027 rover: design study

Scratch study, 28 Oct 2026. Rover: `cac_description` `rover.urdf` visual meshes at zero pose (standing on flat ground). Frames: REP-103, `base_footprint` with ground at z=0, front = +x.
Code: `common.py` (FK + meshes), `rt.py` (numpy Moller-Trumbore), `model.py` (metrics), `sweep.py`, `robust.py`, `figs.py`, `pose_json.py`. Data: `sweep.json` (375 poses).

## Recommendation

| | x (m) | y | z (m) | pitch down | camera top |
|---|---|---|---|---|---|
| **Recommended** | **0.35** | 0 | **0.68** | **22.5 deg** | 0.698 m (52 mm under the 0.75 m cap) |
| Runner-up | 0.30 | 0 | 0.68 | 20.0 deg | 0.697 m |

Pose = centre of the camera body in `base_footprint`. In `base_link`: (0.3517, 0, 0.1903), pitch 23.01 deg (base_link itself sits 0.51 deg nose-up). Full poses and quaternions: `camera_pose.json`.

Recommended pose, measured:
- **No self-occlusion**: 0 % of depth rays hit the rover, and still 0 % with the rover pitched 5 deg nose-down.
- **Near blind edge at x = 0.895 m**, 0.24 m ahead of the front wheels' leading edge (x = 0.651 m). This limit comes from the bottom of the FOV, not from the rover. It is 0.80 m nose-down and 1.00 m nose-up.
- **Marker visibility**: the full 30 cm board (0.20-0.50 m high) at **0.5, 1, 2, 5 and 8 m** is inside the RGB VFOV and unoccluded.
  - At 8 m: 11.2 deg margin, and 6.2 deg with the rover 5 deg nose-down.
  - At 0.5 m: 19 deg margin with the rover 5 deg nose-up.
- **Ground coverage**: 99 % of the 0.6-4 m x +-1 m band is seen. The wheel corridor (|y| <= 0.45 m, out to 2.5 m) is 100 % seen. Visible ground at Z <= 6 m: 36 m2.
- **Crater detection range**, from the camera (proxy model below):
  - 30x4.5 cm crater: 1.8 m;
  - 40x6 cm crater: 2.4 m;
  - 50x7.5 cm crater: 2.8 m.
- **Grazing angle**: 18.8 deg at 2 m and 12.8 deg at 3 m.

**Attachment**: the **front top cross member** of the `base_link` frame box.
- Section: 30 x 30 mm (the CAD draws it solid).
- In `base_link`: x 0.2462-0.2762, z 0.0699-0.0999, y +-0.275. The clear span between the corner uprights is |y| < 0.245.
- In `base_footprint`: top face z = 0.589, front face x = 0.276.
- Nothing else on the body sits in front of the diff bar, so the bracket volume is clear (checked: 0 mesh vertices inside the camera box or the bracket envelope).
- Bracket: an L-riser bolted to the top and front faces at y = 0, lifting the camera about 70 mm. The camera's rear face sits about 60 mm ahead of the member, on a 22.5 deg wedge. The camera fixes with its 1/4-20 bottom thread.
- Runner-up (x = 0.30): the camera sits nearly flush on the member's front face. The bracket is simpler and gives 2.5 deg more marker margin. The cost: 2.2 % of rays clip the bogie PLA in the lower corners when the rover is nose-down, so mask those pixels.

## Why this pose: the trade-off (fig3_sweep.png)

1. **The marker sets the pitch ceiling.**
   - A marker at 8 m, 0.2-0.5 m high, lies 1-4 deg below the camera's horizon.
   - The RGB half-VFOV is 32.5 deg. So the board leaves the frame at a pitch of about 29 deg, independent of height and x.
   - Requiring a 5 deg margin with the rover pitched 5 deg nose-down (sand slope, dig-zone tilt) caps the pitch at **about 23.5 deg**.
   - The near markers (0.5 m, inside the start zone) only need a pitch of at least 5 deg.
2. **Pitch decides the blind distance.**
   - The blind edge moves about 0.05 m per 2.5 deg of pitch.
   - The highest pitch the marker allows (22.5 deg) gives the shortest blind distance.
3. **Height barely matters** between 0.62 and 0.73 m.
   - Grazing at 2 m runs from 17.2 to 20.1 deg, and the crater ranges change by less than 0.1 m.
   - Lower is slightly better for the blind edge, but a camera at z <= 0.65 m with x <= 0.30 m sees the bogies once pitched past about 20 deg.
   - Above 0.71 m the 0.75 m cap leaves no tolerance.
   - 0.68 m is the robust middle.
4. **x**: moving 0.05 m forward removes the bogie occlusion at steep pitch, and moves the blind edge 0.05 m further out.

**Constraints used to select:**
- camera top at least 20 mm under the 0.75 m cap;
- 8 m and 5 m boards in view with at least 5 deg margin when the rover is 5 deg nose-down;
- 0.5 m board in view when the rover is 5 deg nose-up;
- rover occlusion at most 1 % nominal and at most 3 % nose-down.

The selection then minimised the blind edge in the wheel track. Of 375 poses, 146 passed. The top of the list sits within 0.03 m of the best blind edge: the 0.30/0.71/22.5 pose gives 0.865 m, at the cost of 2.6 % occlusion and a 22 mm cap margin. The recommended pose was preferred for zero occlusion and cap margin.

**Look-back case**: to re-sight from the dig zone or berm, the rover turns to face the start corner, so the markers are straight ahead at 5-8 m with this same mount.
- Board margin: 11-12 deg nominal, at least 6.2 deg with the rover 5 deg nose-down.
- Marker size at 8 m: 0.35 m / 8 m is about 36 px on the 1280 px RGB (90 deg HFOV). Enough for ArUco/AprilTag.
- Larger body pitch, for example on the berm face, needs re-sighting from flat sand.

## Pose sweep (excerpt; all 375 in sweep.json)

x, z = camera centre in base_footprint (m). "max" = cap-limited height. "+5" = rover pitched 5 deg nose-down. "-5" = nose-up.

| x | z | pitch | top z | rover-blocked % (nom / nose-down 5) | blind edge x (wheel track) | ground m2 (Z<=6 m) | band cov / q-cov | 40x6 crater range from cam (m) | graze @2 m (deg) | 8 m board margin deg (nom / +5) | 0.5 m board margin at -5 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.35 | 0.680 | 0.0 | 0.693 | 0.0 / 0.0 | 1.58 | 33 | 0.81 / 0.566 | 2.41 | 18.8 | 29.2 / 28.7 | -3.4 |
| 0.35 | 0.680 | 10.0 | 0.695 | 0.0 / 0.0 | 1.19 | 34 | 0.93 / 0.674 | 2.38 | 18.8 | 23.7 / 18.7 | 6.6 |
| 0.35 | 0.680 | 15.0 | 0.696 | 0.0 / 0.0 | 1.06 | 34 | 0.96 / 0.707 | 2.37 | 18.8 | 18.7 / 13.7 | 11.6 |
| 0.35 | 0.680 | 17.5 | 0.697 | 0.0 / 0.0 | 1.00 | 35 | 0.98 / 0.726 | 2.37 | 18.8 | 16.2 / 11.2 | 14.1 |
| 0.35 | 0.680 | 20.0 | 0.697 | 0.0 / 0.0 | 0.95 | 35 | 0.99 / 0.739 | 2.38 | 18.8 | 13.7 / 8.7 | 16.6 |
| 0.35 | 0.680 | 22.5 | 0.698 | 0.0 / 0.0 | 0.90 | 36 | 0.99 / 0.742 | 2.38 | 18.8 | 11.2 / 6.2 | 19.1 |
| 0.35 | 0.680 | 25.0 | 0.698 | 0.0 / 1.4 | 0.85 | 37 | 0.99 / 0.745 | 2.39 | 18.8 | 8.7 / 3.7 | 21.6 |
| 0.35 | 0.680 | 27.5 | 0.698 | 0.0 / 4.2 | 0.81 | 38 | 0.99 / 0.748 | 2.39 | 18.8 | 6.2 / 1.2 | 22.7 |
| 0.35 | 0.680 | 30.0 | 0.699 | 0.9 / 7.2 | 0.76 | 39 | 0.99 / 0.752 | 2.40 | 18.8 | 3.7 / -1.3 | 20.2 |
| 0.35 | 0.680 | 35.0 | 0.699 | 7.2 / 13.8 | 0.76 | 37 | 0.99 / 0.761 | 2.43 | 18.8 | -1.3 / -6.3 | 15.2 |
| 0.35 | 0.620 | 22.5 | 0.638 | 0.0 / 3.6 | 0.85 | 36 | 0.99 / 0.741 | 2.42 | 17.2 | 10.8 / 5.8 | 22.3 |
| 0.35 | 0.650 | 22.5 | 0.668 | 0.0 / 1.2 | 0.87 | 36 | 0.99 / 0.742 | 2.40 | 18.0 | 11.0 / 6.0 | 20.7 |
| 0.35 | 0.710 | 22.5 | 0.728 | 0.0 / 0.0 | 0.92 | 36 | 0.99 / 0.741 | 2.35 | 19.5 | 11.4 / 6.5 | 17.5 |
| 0.35 | 0.730 | 22.5 | 0.748 | 0.0 / 0.0 | 0.94 | 36 | 0.99 / 0.740 | 2.33 | 20.1 | 11.6 / 6.6 | 16.5 |
| 0.26 | 0.680 | 22.5 | 0.698 | 3.8 / 10.6 | 0.84 | 36 | 0.99 / 0.739 | 2.38 | 18.8 | 11.2 / 6.2 | 21.7 |
| 0.30 | 0.680 | 22.5 | 0.698 | 0.0 / 5.0 | 0.85 | 36 | 0.99 / 0.742 | 2.38 | 18.8 | 11.2 / 6.2 | 20.6 |
| 0.40 | 0.680 | 22.5 | 0.698 | 0.0 / 0.0 | 0.95 | 36 | 0.99 / 0.742 | 2.38 | 18.8 | 11.2 / 6.2 | 17.4 |
| 0.45 | 0.680 | 22.5 | 0.698 | 0.0 / 0.0 | 1.00 | 36 | 0.99 / 0.742 | 2.38 | 18.8 | 11.3 / 6.3 | 15.6 |
| 0.30 | 0.680 | 20.0 | 0.697 | 0.0 / 2.2 | 0.90 | 35 | 0.99 / 0.739 | 2.38 | 18.8 | 13.7 / 8.7 | 18.1 |
| 0.26 | 0.620 | 30.0 | 0.639 | 18.9 / 24.8 | 0.68 | 39 | 0.98 / 0.740 | 2.44 | 17.2 | 3.3 / -1.7 | 15.2 |
| 0.30 | 0.620 | 25.0 | 0.638 | 7.9 / 14.2 | 0.83 | 37 | 0.99 / 0.744 | 2.42 | 17.2 | 8.3 / 3.3 | 20.5 |

## Method and assumptions

- **Rays**: 80 x 50 rays across the depth FOV (87 x 58 deg, pinhole) for the self-occlusion fraction. Coverage uses ground grids (2.5 cm in the band, 10 cm out to 7 m). Every ground point is checked for FOV, depth Z of 0.52-6 m, and occlusion by a ray-cast to it.
- **Mesh**: all visual meshes, vertex-clustered at 3 mm (40 k triangles), with a conservative frustum cull.
- **Crater proxy**:
  - Reference crater: a parabolic bowl.
  - Visible depth fraction: f(grazing) = 1 if tan(g) >= d/a, else 1-(1-tan(g)a/d)^2.
  - Range noise: sigma_Z = 0.004 Z^2 m (about 2x Intel's theory, as on sand).
  - Pixels on the crater counted at 848x480.
  - Detection SNR = f d sqrt(N) / (sigma_r sin g).
  - The threshold is calibrated so a 30x4.5 cm crater is seen to 2.0 m from 0.5 m height (the cac_sim result). The same threshold then reproduces the sim's 40 cm (2.4 m vs 2-3 m) and 50 cm (2.8 m vs 3 m) results.
  - It is a ranking proxy, not a prediction. Re-measure on real sand.
- **Markers**: tested at the front wheel's leading edge + D, straight ahead, using the RGB VFOV of 65 deg. The 8 m board also fits inside the depth VFOV (58 deg).
- **D455 facts**: the network was down, so Intel's datasheet could **not** be fetched; nothing here is verified against it. Values used: body 124 x 26 x 29 mm, depth 87 x 58 deg, RGB 90 x 65 deg, min-Z 0.52 m, baseline 95 mm, 1/4-20 bottom thread (the 2 x M4 on the back are not verified).
- **Not modelled**:
  - the excavation tool or bucket, which could block the lower FOV: re-run with its mesh;
  - dust;
  - suspension articulation (the +-5 deg body-pitch cases approximate it);
  - the depth origin's 47.5 mm lateral offset (left imager).

## Files

- `fig1_footprint.png`: top-down coverage and crater quality, recommended pose.
- `fig2_side.png` and `fig2b_side_zoom.png`: FOV wedges, rover, ground, marker sight lines.
- `fig3_sweep.png`: metrics vs pitch and height.
- `camera_pose.json`: the chosen pose and the attachment surface.
