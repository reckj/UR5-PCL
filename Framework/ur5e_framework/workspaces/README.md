# workspaces/ — shared workspace files

Ready-made workspaces everybody can use, next to the personal `workspace.json`
you teach yourself (which stays git-ignored in the repo folder).

| file | use it with | what it is |
|---|---|---|
| `cart_360_real.json` | `--target real` | the **whole lab cell** `cart_360` as one workspace |
| `cart_360_sim.json` | the simulator, together with `--cell cart_360` | the same, for rehearsing |

```bash
python students/<name>/my_script.py --workspace workspaces/cart_360_sim.json --cell cart_360
python students/<name>/my_script.py --target real --workspace workspaces/cart_360_real.json --payload KG --tcp X Y Z RX RY RZ
```

- The box is the complete cell: above the cart top, around the cart, down to
  50 mm above the floor. The cart, its handle and the floor are still checked
  on every move (`docs/SAFETY_SETUP.md`).
- `ws.home_joints` is a common home above the cart top, tool pointing down.
- **Use absolute poses** (`[x, y, z, rx, ry, rz]`, Base frame) with these
  files. `ws.pose(u, v, w)`, `ws.grid` and `ws.line` spread over a box of
  about 1.9 × 1.9 × 1.75 m, most of which the arm cannot reach — for work in
  fractions of an area, teach your own workspace (`00_teach_workspace.py`).
- A file is tied to where it is used: the `real` file is refused in the
  simulator and the `sim` file on the real arm.

Generated from the cell file by the instructor; do not edit.
