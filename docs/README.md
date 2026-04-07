# Bubblegum Sticky Pick

This extension adds one OmniGraph node for Isaac Sim:

- `Bubblegum Sticky Pick`
- `AGV Waypoint Driver`

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

## AGV Waypoint Driver

The AGV node turns the script-editor waypoint follower into a stateful OmniGraph node.

Scene expectations:

- an AGV Xform prim to move
- a path root prim whose child Xforms are waypoints
- waypoint children are traversed in lexicographic order, so use names like `wp_001`, `wp_002`, ...

Waypoint name tokens:

- `_waitMS####`: wait at the waypoint in milliseconds
- `_bendingCM###`: round a corner with the given radius in centimeters when the corner geometry allows it
- `_reverse`: enable reverse-at-endpoints mode only when present on both the first and last waypoint

Typical wiring:

- connect `On Physics Step.execOut` to `AGV Waypoint Driver.execIn`
- connect physics delta time to `deltaTime`
- set `agvPrim`
- set `pathRootPrim`

Main inputs:

- `agvPrim`
- `pathRootPrim`
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
- `isWaiting`
- `isStopped`
- `reverseMode`
- `currentWaypointIndex`
- `currentWaypointName`
- `waypointCount`

Notes:

- this first pass expects fixed waypoint Xforms and no live obstacle handling
- the AGV is moved kinematically by updating its translate/orient ops each evaluation
- when reverse mode is disabled, the node stays stopped at the endpoint until the timeline is restarted or the route changes
