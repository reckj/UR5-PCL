# Gripper v2 (side-mount) — print files

Exported from Fusion "Gripper v3" by `fusion_gripper_v2_sidemount.py` (2026-11-02),
mid-pose (phi = 0), units mm, high mesh refinement. Meshes are in the gripper
frame (Z = servo axis, Y = forward, X = jaw axis) — reorient in the slicer:

| file            | qty | material / notes                         | print orientation (face on bed)                     |
| --------------- | --- | ---------------------------------------- | --------------------------------------------------- |
| 01_Body         | 1   | PETG/PLA, 4 walls, 30 %                  | flange face (-Y) down; bay opens upward, no supports |
| 02_Upper        | 1   | PETG/PLA, 4 walls                        | tower floor (-Z) down; lid plate stands vertical     |
| 03_Palm         | 1   | PETG/PLA, 100 % or 5 walls               | plate underside (-Z) down, upstand rises             |
| 04_Hub_R        | 1   | PETG, 100 % (gear + horn pocket)         | gear face (-Z) down; horn pocket faces the bed       |
| 05_Hub_L        | 1   | PETG, 100 %                              | gear face (-Z) down                                  |
| 06/07_Follower  | 1+1 | PETG, 100 %                              | flat                                                 |
| 08/09_Carrier   | 1+1 | PETG, 100 % (jaw pad)                    | carrier top (+Z) down -> jaw stands upright          |
| 10_Cover        | 1   | PLA, 4 walls                             | flat                                                 |
| 11_Sleeve_1..3  | 3   | PETG, 100 % (or brass tube 6x3.4x20)     | upright                                              |
| 12_Bushing_1..4 | 4   | PETG, 100 % (or brass tube 5x3.4x6.6)    | upright                                              |

Tolerances built in: 0.5 mm layer gaps, 6.4 bores on 6.0 sleeves, 5.4 holes on
5.0 bushings, 0.3 mm servo pocket clearance, 0.15 mm gear backlash, M3 insert
holes 4.0 mm, 2-56 holes 2.3 mm. Verify horn height / T-CAN485 holes first
(Documentation/15). Heat-set inserts: see BOM in doc 15.
