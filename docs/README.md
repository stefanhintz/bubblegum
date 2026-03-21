# Bubblegum Sticky Pick

This extension adds one OmniGraph node for Isaac Sim:

- `Bubblegum Sticky Pick`

The node implements a simple sticky-gripper behavior:

- set `helperPrim` to a helper prim that defines the sticky volume
- set `stick = true` to attach and keep holding the first overlapping prim
- keep evaluating the node to make the attached prim follow the helper
- set `stick = false` to release it
- when the object attaches, it keeps the relative contact offset instead of jumping to the helper center

Inputs:

- `execIn`
- `enabled`
- `helperPrim`
- `stick`
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
- the held object preserves its initial relative offset to the helper
- invalid prim paths fail fast

To enable this extension, go to `Window > Extensions` and enable `bubblegum`.
