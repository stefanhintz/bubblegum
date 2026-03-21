# Bubblegum Sticky Node

This extension adds one minimal OmniGraph node for Isaac Sim:

- `Bubblegum Sticky Pick`

The node is meant for a first-pass "sticky gripper" behavior:

- give it a helper prim path that marks the sticky volume
- pulse `stick`
- if a candidate prim overlaps the helper volume, the node attaches it
- while attached, the node teleports the object to follow the helper
- pulse `release` to drop it

## Node Contract

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

## Usage

1. Add `/Users/stefanhintz/bubblegum` as an extension search path in Isaac Sim.
2. Enable `bubblegum` in the Extension Manager.
3. In an Action Graph, add `Bubblegum Sticky Pick`.
4. Drive `execIn` from `On Playback Tick` or `On Physics Step`.
5. Set `helperPrimPath` to a prim whose world-space bounds define the sticky zone.
6. Optionally set `candidatePrimPaths` to limit attachment to a known set of prims.
7. Pulse `stick` to attach and pulse `release` to detach.

## Limitations

- This is intentionally simple and fast to iterate on.
- Overlap detection uses world-space AABB intersection.
- The attached object is moved by authoring a single matrix xform op every tick.
- The node preserves the attached prim's scale, but not its original xform op stack.
- There is no physics joint, force limit, filtering, debounce, or safety padding yet.

## Development Notes

- For this extension, keep OmniGraph node source files under `python/nodes/`.
- If you change `.ogn` files, disable and re-enable the extension or restart Isaac Sim.
