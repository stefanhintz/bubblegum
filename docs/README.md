# Bubblegum Sticky Pick

This extension adds one OmniGraph node for Isaac Sim:

- `Bubblegum Sticky Pick`

The node implements a simple sticky-gripper behavior:

- set `helperPrimPath` to a helper prim that defines the sticky volume
- pulse `stick` to attach the first overlapping prim
- keep evaluating the node to make the attached prim follow the helper
- pulse `release` to drop it

Inputs:

- `execIn`
- `enabled`
- `helperPrimPath`
- `stick`
- `release`
- `candidatePrimPaths`

Outputs:

- `execOut`
- `isAttached`
- `attachedPrimPath`
- `didAttach`
- `didRelease`

Notes:

- overlap uses world-space AABB intersection
- there is no padding, filtering, or physics joint yet
- invalid prim paths fail fast

To enable this extension, go to `Window > Extensions` and enable `bubblegum`.
