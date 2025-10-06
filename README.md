# jointstate2tf

Minimal TypeScript library to compute TF (TransformStamped array) from a URDF
and a JointState. It does not depend on ROS; you can wrap inputs/outputs in any
middleware (ROS 1/2 or others).

- Single class API: parse URDF once, run at high frequency.
- ESM output with bundled type declarations.
- No runtime dependencies – only core FK to TF.

## Install

```bash
npm i jointstate2tf
```

## Usage

Use a real Panda URDF (Franka Emika Panda) from:
`https://raw.githubusercontent.com/StanfordASL/PandaRobot.jl/refs/heads/master/deps/Panda/panda.urdf`

```ts
import JointState2TF, { type JointState } from 'jointstate2tf';

// 1) Load URDF (Browser or Node 18+ with global fetch)
const engine = await JointState2TF.fromUrl({
  url: 'https://raw.githubusercontent.com/StanfordASL/PandaRobot.jl/refs/heads/master/deps/Panda/panda.urdf',
});

// 2) Build JointState (example: 7 Panda joints)
const jointState: JointState = {
  header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
  name: [
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7',
  ],
  position: [0.0, -0.3, 0.2, -1.2, 0.5, 1.0, -0.4], // unit: radians
};

// 3) Compute TF (optional: include timestamp in nanoseconds)
const tf = engine.computeFromJointState(jointState, { publishTimeNs: Date.now() * 1e6 });

console.log('TFMessage:', JSON.stringify(tf, null, 2));
```

Input JointState structure (example):
```json
{
  "header": { "stamp": { "sec": 0, "nanosec": 0 }, "frame_id": "" },
  "name": ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"],
  "position": [0.0, -0.3, 0.2, -1.2, 0.5, 1.0, -0.4]
}
```

Output TFMessage structure (snippet, showing the first two transforms):
```json
{
  "transforms": [
    {
      "header": { "stamp": { "sec": 1720000000, "nanosec": 123456789 }, "frame_id": "panda_link0" },
      "child_frame_id": "panda_link1",
      "transform": {
        "translation": { "x": 0.0, "y": 0.0, "z": 0.333 },
        "rotation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
      }
    },
    {
      "header": { "stamp": { "sec": 1720000000, "nanosec": 123456789 }, "frame_id": "panda_link1" },
      "child_frame_id": "panda_link2",
      "transform": {
        "translation": { "x": 0.0, "y": 0.0, "z": 0.0 },
        "rotation": { "x": 0.0, "y": 0.0, "z": 0.3826834, "w": 0.9238795 }
      }
    }
  ]
}
```
Note: the numbers above are illustrative; actual poses depend on the URDF and joint positions.

### Node example: use fromXml

```ts
import { readFileSync } from 'node:fs';
import JointState2TF, { type JointState } from 'jointstate2tf';

// Read URDF content from a local file
const xml = readFileSync('panda.urdf', 'utf8');
const engine = JointState2TF.fromXml({ xml });

// Same JointState format as above
const jointState: JointState = {
  header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
  name: [
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7',
  ],
  position: [0.0, -0.3, 0.2, -1.2, 0.5, 1.0, -0.4],
};

const tf = engine.computeFromJointState(jointState);
console.log(JSON.stringify(tf, null, 2));
```

## API

- `class JointState2TF`
  - `static fromUrl({ url })`: Load URDF from URL, returns instance.
  - `static fromXml({ xml })`: Parse from URDF XML content, returns instance.
  - `setJointState(jointState)`: Set joint values on the internal model.
  - `compute({ publishTimeNs? })`: Compute TF for all links.
  - `computeFromJointState(jointState, { publishTimeNs? })`: Convenience wrapper.

## Types

The package exports: `JointState`, `TFMessage`, `TransformStamped`, `Transform`,
`Header`, `Time`.

## Notes

- Mesh/rendering is not used in computations; the library performs kinematic FK only.
