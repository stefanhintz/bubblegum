# Bubblegum Sticky Pick

This extension adds one OmniGraph node for Isaac Sim:

- `Bubblegum Sticky Pick`
- `AGV Waypoint Driver`
- `Prim Reset`
- `OBS Scene Switcher`

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
- `helperPrim`
- `stick`
- `candidatePrimPaths`
- `excludePrimPaths`

Outputs:

- `execAttached`
- `execReleased`
- `isAttached`
- `attachedPrimPath`

Notes:

- overlap uses world-space AABB intersection
- candidate filters support simple wildcard matching with `*`
- exclude filters support the same wildcard matching and take precedence
- there is no padding, filtering, or physics joint yet
- the held object preserves its initial relative offset to the helper
- stopping simulation clears the current attachment state and restores all objects moved by the node to their pre-attach transforms
- invalid prim paths fail fast

To enable this extension, go to `Window > Extensions` and enable `bubblegum`.

## Prim Reset

The prim reset node captures a whole prim subtree's initial state and restores it on demand during simulation.

Typical wiring:

- connect any tick or control execution to `execIn`
- set `prim`
- trigger `execReset` whenever you want to restore that prim and all descendants underneath it

Behavior:

- the first time the node sees a valid `prim`, it captures the whole subtree rooted there
- for every xformable prim in the subtree, it captures the local transform and xform op stack
- for every rigid-body prim in the subtree, it also captures the initial rigid-body enabled and kinematic-enabled flags
- on `execReset`, it restores the captured subtree state and attempts to zero rigid-body linear and angular velocities

Inputs:

- `execIn`
- `prim`
- `execReset`

Outputs:

- `execResetDone`
- `isCaptured`
- `trackedPrimPath`

## AGV Waypoint Driver

The AGV node turns the script-editor waypoint follower into a stateful OmniGraph node.

Scene expectations:

- an AGV Xform prim to move
- a path root prim whose child Xforms are waypoints
- waypoint children are traversed in lexicographic order, so use names like `wp_001`, `wp_002`, ...

Waypoint custom data:

- `waypoint.waitMs`: wait at the waypoint in milliseconds
- `waypoint.bendRadiusCm`: blend a corner with a tangent arc of the given radius in centimeters when the geometry allows it
- `waypoint.reverse`: mark an endpoint as a reverse point
- `waypoint.dock`: mark a dock point that is approached straight in and backed out in reverse
- `waypoint.yawDeg`: optional yaw override in degrees for the waypoint pose, useful for the start/end orientation

Example:

```json
{
  "waypoint": {
    "waitMs": 1000,
    "bendRadiusCm": 25,
    "reverse": true,
    "dock": true,
    "yawDeg": 180
  }
}
```

Typical wiring:

- connect `On Physics Step.execOut` to `AGV Waypoint Driver.execIn`
- connect physics delta time to `deltaTime`
- set `agvPrim`
- set one or more `pathRoots`
- choose the route with `activePathIndex`
- set `run = true` to move
- trigger `execReset` to snap the AGV to the first waypoint of the active route and clear route state

Main inputs:

- `agvPrim`
- `pathRoots`
- `activePathIndex`
- `run`
- `execReset`
- `deltaTime`
- `targetSpeedMps`
- `maxAccelMps2`
- `maxYawRateRps`
- `maxYawAccelRps2`
- `positionToleranceM`
- `yawToleranceDeg`
- `lookaheadM`

Main outputs:

- `isRouteValid`
- `isStopped`
- `execFinished`

Notes:

- `waypoint.reverse` controls endpoint reversal
- if both endpoints have `waypoint.reverse = true`, the AGV reverses at both ends
- if only one endpoint has `waypoint.reverse = true`, the AGV turns around there once and stops at the opposite end
- `waypoint.yawDeg` overrides the inferred heading at that waypoint; if it is not set, the first and last waypoint use the waypoint prim rotation as their stop/start orientation
- if the first or last waypoint is a dock, the AGV departs that endpoint in reverse from the dock pose instead of turning in place first
- `waypoint.bendRadiusCm` is a true path blend radius: the AGV drives to the tangent entry point, then follows an arc into the outgoing segment
- bend arcs use radius-based speed limits, and the AGV only slows on the incoming line when the arc is too short to absorb that deceleration by itself
- `waypoint.dock` makes the AGV enter the point straight; if there is a following waypoint after the dock it backs out and resumes there, otherwise it stays parked at the dock unless `waypoint.reverse = true`, in which case it backs out and continues in the opposite direction
- this first pass expects fixed waypoint Xforms and no live obstacle handling
- the AGV is moved kinematically by updating its translate/orient ops each evaluation
- when reverse mode is disabled, the node stays stopped at the endpoint until the timeline is restarted or the route changes

## OBS Scene Switcher

The OBS node connects to an OBS WebSocket and switches the current program scene once per distinct scene name.

Typical wiring:

- connect any execution trigger to `execIn`
- set `sceneName`
- set `wsUrl` if your OBS instance does not use the default local websocket port
- set `wsPassword`

Inputs:

- `execIn`
- `sceneName`
- `wsUrl`
- `wsPassword`

Outputs:

- `execDone`

Notes:

- the node uses a small built-in WebSocket client and does not require `websocket-client`
- the node caches one websocket connection and reuses it while the scene name stays the same
- if the websocket fails, the node logs an error and retries on the next compute
- the password input is visible and should be set when OBS authentication is enabled
