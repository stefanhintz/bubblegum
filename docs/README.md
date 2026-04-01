# Bubblegum Sticky Pick

This extension adds one OmniGraph node for Isaac Sim:

- `Bubblegum Sticky Pick`

The node implements a simple sticky-gripper behavior:

- set `helperPrim` to a helper prim that defines the sticky volume
- set `stick = true` to attach and keep holding the first overlapping prim
- keep evaluating the node to make the attached prim follow the helper
- set `stick = false` to release it
- when the object attaches, it keeps the relative contact offset instead of jumping to the helper center
- `candidatePrimPaths` accepts exact prim paths and wildcard filters such as `/World/pallet-with-boxes/*` or `pallet-with-boxes/*`
- `excludePrimPaths` accepts exact prim paths and wildcard filters such as `/World/Robot/*` or `gripper/*` and always wins over candidate filters

Inputs:

- `execIn`
- `enabled`
- `helperPrim`
- `stick`
- `candidatePrimPaths`
- `excludePrimPaths`

Outputs:

- `execOut`
- `isAttached`
- `attachedPrimPath`
- `didAttach`
- `didRelease`

Notes:

- overlap uses world-space AABB intersection
- candidate filters support simple wildcard matching with `*`
- exclude filters support the same wildcard matching and take precedence
- there is no padding, filtering, or physics joint yet
- the held object preserves its initial relative offset to the helper
- stopping simulation clears the current attachment state and restores all objects moved by the node to their pre-attach transforms
- invalid prim paths fail fast

To enable this extension, go to `Window > Extensions` and enable `bubblegum`.
