import JointState2TF from '../dist/index.js';

// Minimal URDF: base_link ->(rev z, origin xyz=1,0,0)-> link1 ->(prismatic x, origin xyz=0,1,0)-> link2
const urdf = `
<robot name="mini">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="joint2" type="prismatic">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 1 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
  </joint>
 </robot>`;

function assertNear(actual, expected, eps = 1e-6, label = '') {
  if (Math.abs(actual - expected) > eps) {
    throw new Error(`${label} expected ${expected}, got ${actual}`);
  }
}

function assertVec3Near(v, e, eps = 1e-6, label = 'vec3') {
  assertNear(v.x, e.x, eps, `${label}.x`);
  assertNear(v.y, e.y, eps, `${label}.y`);
  assertNear(v.z, e.z, eps, `${label}.z`);
}

function assertQuatNear(q, e, eps = 1e-6, label = 'quat') {
  // Normalize sign (q and -q represent the same rotation)
  const s = q.w < 0 ? -1 : 1;
  assertNear(s * q.x, e.x, eps, `${label}.x`);
  assertNear(s * q.y, e.y, eps, `${label}.y`);
  assertNear(s * q.z, e.z, eps, `${label}.z`);
  assertNear(s * q.w, e.w, eps, `${label}.w`);
}

async function main() {
  const engine = JointState2TF.fromXml({ xml: urdf });

  // joint1 = 90deg around Z, joint2 = 0.5m along X
  engine.setJointState({
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
    name: ['joint1', 'joint2'],
    position: [Math.PI / 2, 0.5],
  });

  const tf = engine.compute();

  // Find transforms
  const t1 = tf.transforms.find(t => t.child_frame_id === 'link1');
  const t2 = tf.transforms.find(t => t.child_frame_id === 'link2');

  if (!t1 || !t2) throw new Error('Missing expected transforms');

  // joint1: translation (1,0,0), rotation around Z by 90deg
  assertVec3Near(t1.transform.translation, { x: 1, y: 0, z: 0 }, 1e-9, 'joint1.t');
  const s2 = Math.SQRT1_2; // sqrt(1/2)
  assertQuatNear(t1.transform.rotation, { x: 0, y: 0, z: s2, w: s2 }, 1e-6, 'joint1.q');

  // joint2: origin (0,1,0) then prismatic along X by 0.5 -> (0.5, 1, 0), no rotation
  assertVec3Near(t2.transform.translation, { x: 0.5, y: 1, z: 0 }, 1e-9, 'joint2.t');
  assertQuatNear(t2.transform.rotation, { x: 0, y: 0, z: 0, w: 1 }, 1e-9, 'joint2.q');

  console.log('All assertions passed. Transforms:');
  console.log(JSON.stringify(tf, null, 2));
}

main().catch(err => {
  console.error(err);
  process.exit(1);
});


