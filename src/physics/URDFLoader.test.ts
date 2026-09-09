import { describe, it, expect, afterEach } from 'vitest'
import {
  parseURDF,
  extractMeshPaths,
  normalizeMeshPath,
  validateURDF,
  getMeshFilenames,
  stripUnsupportedURDFElements,
} from './URDFLoader'

// ---------------------------------------------------------------------------
// Test URDF strings
// ---------------------------------------------------------------------------

const MINIMAL_URDF = `
<robot name="test_robot">
  <link name="base_link"/>
</robot>`

const TWO_LINK_URDF = `
<robot name="two_link">
  <link name="base_link"/>
  <link name="arm_link"/>
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
  </joint>
</robot>`

const COMPLEX_URDF = `
<robot name="arm_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://arm/meshes/base.stl"/>
      </geometry>
    </visual>
  </link>
  <link name="shoulder">
    <visual>
      <geometry>
        <mesh filename="package://arm/meshes/shoulder.dae"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://arm/meshes/shoulder_col.stl"/>
      </geometry>
    </collision>
  </link>
  <link name="elbow"/>
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder"/>
  </joint>
  <joint name="elbow_joint" type="revolute">
    <parent link="shoulder"/>
    <child link="elbow"/>
  </joint>
</robot>`

const URDF_WITH_DUPLICATE_MESH = `
<robot name="dup_mesh">
  <link name="link1">
    <visual>
      <geometry>
        <mesh filename="meshes/part.stl"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh filename="meshes/part.stl"/>
      </geometry>
    </collision>
  </link>
</robot>`

const URDF_NO_LINKS = `
<robot name="empty">
</robot>`

const URDF_MISSING_NAME = `
<robot>
  <link name="base"/>
</robot>`

const URDF_MULTIPLE_ROOTS = `
<robot name="multi_root">
  <link name="root1"/>
  <link name="root2"/>
  <link name="child"/>
  <joint name="j1" type="fixed">
    <parent link="root1"/>
    <child link="child"/>
  </joint>
</robot>`

const URDF_BAD_JOINT_REFS = `
<robot name="bad_refs">
  <link name="base"/>
  <link name="arm"/>
  <joint name="j1" type="revolute">
    <parent link="base"/>
    <child link="nonexistent"/>
  </joint>
</robot>`

const URDF_WITH_TRANSMISSIONS = `
<robot name="arm_with_trans">
  <link name="base"/>
  <link name="shoulder"/>
  <joint name="shoulder_pan" type="revolute">
    <parent link="base"/>
    <child link="shoulder"/>
  </joint>
  <transmission name="shoulder_pan_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="shoulder_pan">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor1">
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
</robot>`

const URDF_VARIOUS_MESH_PREFIXES = `
<robot name="prefixes">
  <link name="link1">
    <visual>
      <geometry><mesh filename="package://pkg/mesh1.stl"/></geometry>
    </visual>
  </link>
  <link name="link2">
    <visual>
      <geometry><mesh filename="file:///absolute/mesh2.obj"/></geometry>
    </visual>
  </link>
  <link name="link3">
    <visual>
      <geometry><mesh filename="relative/mesh3.dae"/></geometry>
    </visual>
  </link>
</robot>`

// ---------------------------------------------------------------------------
// parseURDF
// ---------------------------------------------------------------------------

describe('parseURDF', () => {
  it('parses a minimal valid URDF', () => {
    const result = parseURDF(MINIMAL_URDF)
    expect(result.ok).toBe(true)
    if (!result.ok) return
    expect(result.value.name).toBe('test_robot')
    expect(result.value.links).toEqual(['base_link'])
    expect(result.value.joints).toHaveLength(0)
    expect(result.value.meshPaths).toHaveLength(0)
  })

  it('extracts links and joints from a two-link robot', () => {
    const result = parseURDF(TWO_LINK_URDF)
    expect(result.ok).toBe(true)
    if (!result.ok) return
    expect(result.value.links).toEqual(['base_link', 'arm_link'])
    expect(result.value.joints).toHaveLength(1)
    expect(result.value.joints[0]).toEqual({
      name: 'arm_joint',
      type: 'revolute',
      parent: 'base_link',
      child: 'arm_link',
    })
  })

  it('extracts mesh paths from visual and collision geometry', () => {
    const result = parseURDF(COMPLEX_URDF)
    expect(result.ok).toBe(true)
    if (!result.ok) return
    expect(result.value.meshPaths).toContain('arm/meshes/base.stl')
    expect(result.value.meshPaths).toContain('arm/meshes/shoulder.dae')
    expect(result.value.meshPaths).toContain('arm/meshes/shoulder_col.stl')
  })

  it('deduplicates mesh paths', () => {
    const result = parseURDF(URDF_WITH_DUPLICATE_MESH)
    expect(result.ok).toBe(true)
    if (!result.ok) return
    expect(result.value.meshPaths).toEqual(['meshes/part.stl'])
  })

  it('returns error for invalid XML', () => {
    const result = parseURDF('<not-closed')
    expect(result.ok).toBe(false)
    if (result.ok) return
    expect(result.error).toContain('Invalid XML')
  })

  it('returns error for non-robot root element', () => {
    const result = parseURDF('<mujoco><worldbody/></mujoco>')
    expect(result.ok).toBe(false)
    if (result.ok) return
    expect(result.error).toContain('<mujoco>')
  })

  it('returns error when no links are found', () => {
    const result = parseURDF(URDF_NO_LINKS)
    expect(result.ok).toBe(false)
    if (result.ok) return
    expect(result.error).toContain('no <link> elements')
  })

  it('warns when robot name is missing', () => {
    const result = parseURDF(URDF_MISSING_NAME)
    expect(result.ok).toBe(true)
    if (!result.ok) return
    expect(result.value.name).toBe('unnamed_robot')
    expect(result.value.warnings).toContain('Robot element is missing a "name" attribute.')
  })

  it('warns when joints reference unknown links', () => {
    const result = parseURDF(URDF_BAD_JOINT_REFS)
    expect(result.ok).toBe(true)
    if (!result.ok) return
    expect(result.value.warnings.some(w => w.includes('nonexistent'))).toBe(true)
  })

  it('extracts multiple joints from complex URDF', () => {
    const result = parseURDF(COMPLEX_URDF)
    expect(result.ok).toBe(true)
    if (!result.ok) return
    expect(result.value.joints).toHaveLength(2)
    expect(result.value.joints[0].name).toBe('shoulder_joint')
    expect(result.value.joints[1].name).toBe('elbow_joint')
  })

  it('handles URDF with duplicate link names', () => {
    const xml = `
    <robot name="dup">
      <link name="base"/>
      <link name="base"/>
    </robot>`
    const result = parseURDF(xml)
    expect(result.ok).toBe(true)
    if (!result.ok) return
    expect(result.value.warnings.some(w => w.includes('Duplicate link name'))).toBe(true)
  })

  it('handles joints without type attribute', () => {
    const xml = `
    <robot name="no_type">
      <link name="a"/>
      <link name="b"/>
      <joint name="j1">
        <parent link="a"/>
        <child link="b"/>
      </joint>
    </robot>`
    const result = parseURDF(xml)
    expect(result.ok).toBe(true)
    if (!result.ok) return
    expect(result.value.joints[0].type).toBe('fixed')
    expect(result.value.warnings.some(w => w.includes('missing a "type"'))).toBe(true)
  })

  it('ignores joint elements nested inside transmission elements', () => {
    const result = parseURDF(URDF_WITH_TRANSMISSIONS)
    expect(result.ok).toBe(true)
    if (!result.ok) return
    // Should find exactly 1 joint (the real one), not the transmission reference
    expect(result.value.joints).toHaveLength(1)
    expect(result.value.joints[0].name).toBe('shoulder_pan')
    expect(result.value.joints[0].type).toBe('revolute')
    // No warnings about duplicates or missing attributes
    expect(result.value.warnings).toHaveLength(0)
  })
})

// ---------------------------------------------------------------------------
// extractMeshPaths
// ---------------------------------------------------------------------------

describe('extractMeshPaths', () => {
  it('returns empty array for links without meshes', () => {
    const doc = new DOMParser().parseFromString(MINIMAL_URDF, 'application/xml')
    expect(extractMeshPaths(doc.documentElement)).toEqual([])
  })

  it('extracts paths from visual and collision mesh elements', () => {
    const doc = new DOMParser().parseFromString(COMPLEX_URDF, 'application/xml')
    const paths = extractMeshPaths(doc.documentElement)
    expect(paths).toHaveLength(3)
  })
})

// ---------------------------------------------------------------------------
// normalizeMeshPath
// ---------------------------------------------------------------------------

describe('normalizeMeshPath', () => {
  it('strips package:// prefix', () => {
    expect(normalizeMeshPath('package://my_pkg/meshes/part.stl')).toBe('my_pkg/meshes/part.stl')
  })

  it('strips file:// prefix and leading slashes', () => {
    expect(normalizeMeshPath('file:///home/user/mesh.obj')).toBe('home/user/mesh.obj')
  })

  it('leaves relative paths unchanged', () => {
    expect(normalizeMeshPath('meshes/part.stl')).toBe('meshes/part.stl')
  })

  it('strips leading slashes', () => {
    expect(normalizeMeshPath('/meshes/part.stl')).toBe('meshes/part.stl')
  })

  it('trims whitespace', () => {
    expect(normalizeMeshPath('  meshes/part.stl  ')).toBe('meshes/part.stl')
  })
})

// ---------------------------------------------------------------------------
// validateURDF
// ---------------------------------------------------------------------------

describe('validateURDF', () => {
  it('returns empty array for valid URDF', () => {
    expect(validateURDF(TWO_LINK_URDF)).toEqual([])
  })

  it('returns error for invalid XML', () => {
    const errors = validateURDF('<broken')
    expect(errors.length).toBeGreaterThan(0)
    expect(errors[0].message).toContain('Invalid XML')
  })

  it('returns error for non-robot root', () => {
    const errors = validateURDF('<mujoco/>')
    expect(errors.length).toBeGreaterThan(0)
  })

  it('returns error for no links', () => {
    const errors = validateURDF(URDF_NO_LINKS)
    expect(errors.length).toBeGreaterThan(0)
  })

  it('returns error for multiple root links', () => {
    const errors = validateURDF(URDF_MULTIPLE_ROOTS)
    expect(errors.some(e => e.message.includes('Multiple root links'))).toBe(true)
  })

  it('returns no errors for a single root link', () => {
    expect(validateURDF(MINIMAL_URDF)).toEqual([])
  })
})

// ---------------------------------------------------------------------------
// getMeshFilenames
// ---------------------------------------------------------------------------

describe('getMeshFilenames', () => {
  it('returns basenames of mesh files', () => {
    const filenames = getMeshFilenames(COMPLEX_URDF)
    expect(filenames).toContain('base.stl')
    expect(filenames).toContain('shoulder.dae')
    expect(filenames).toContain('shoulder_col.stl')
  })

  it('returns empty array for URDF without meshes', () => {
    expect(getMeshFilenames(MINIMAL_URDF)).toEqual([])
  })

  it('returns empty array for invalid URDF', () => {
    expect(getMeshFilenames('<broken')).toEqual([])
  })

  it('handles various path prefixes', () => {
    const filenames = getMeshFilenames(URDF_VARIOUS_MESH_PREFIXES)
    expect(filenames).toContain('mesh1.stl')
    expect(filenames).toContain('mesh2.obj')
    expect(filenames).toContain('mesh3.dae')
  })
})

// ---------------------------------------------------------------------------
// stripUnsupportedURDFElements
// ---------------------------------------------------------------------------

const URDF_WITH_ROOT_MATERIALS = `
<robot name="so_arm100">
  <material name="3d_printed">
    <color rgba="1.0 0.82 0.12 1.0"/>
  </material>
  <material name="sts3215">
    <color rgba="0.1 0.1 0.1 1.0"/>
  </material>
  <link name="base_link">
    <visual>
      <geometry><mesh filename="assets/Base.stl"/></geometry>
      <material name="3d_printed"/>
    </visual>
  </link>
  <link name="shoulder_link"/>
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
  </joint>
</robot>`

const URDF_WITH_ALL_UNSUPPORTED = `
<robot name="full_test">
  <material name="mat1">
    <color rgba="1 0 0 1"/>
  </material>
  <link name="base"/>
  <link name="arm"/>
  <joint name="j1" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
  </joint>
  <transmission name="trans1">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="j1"/>
    <actuator name="motor1"/>
  </transmission>
  <gazebo reference="base">
    <material>Gazebo/Orange</material>
  </gazebo>
</robot>`

describe('stripUnsupportedURDFElements', () => {
  it('strips top-level <material> elements', () => {
    const result = stripUnsupportedURDFElements(URDF_WITH_ROOT_MATERIALS)
    // Top-level <material> with children should be gone — check both
    // the open tag and the content won't appear at the root level.
    // Use a parsed check: re-parse and confirm no direct material children.
    const doc = new DOMParser().parseFromString(result, 'application/xml')
    const root = doc.documentElement
    const rootMaterials = Array.from(root.children).filter(c => c.tagName === 'material')
    expect(rootMaterials).toHaveLength(0)
    expect(root.getAttribute('name')).toBe('so_arm100')
  })

  it('preserves <material> elements nested inside <visual> tags', () => {
    const result = stripUnsupportedURDFElements(URDF_WITH_ROOT_MATERIALS)
    // The <material name="3d_printed"/> ref inside <visual> must remain
    const doc = new DOMParser().parseFromString(result, 'application/xml')
    const nestedMaterials = doc.querySelectorAll('visual material')
    expect(nestedMaterials.length).toBeGreaterThan(0)
    expect(nestedMaterials[0].getAttribute('name')).toBe('3d_printed')
  })

  it('strips <transmission> and <gazebo> elements', () => {
    const result = stripUnsupportedURDFElements(URDF_WITH_ALL_UNSUPPORTED)
    expect(result).not.toContain('<transmission')
    expect(result).not.toContain('<gazebo')
  })

  it('preserves links, joints, and robot name', () => {
    const result = stripUnsupportedURDFElements(URDF_WITH_ALL_UNSUPPORTED)
    const doc = new DOMParser().parseFromString(result, 'application/xml')
    const root = doc.documentElement
    expect(root.getAttribute('name')).toBe('full_test')
    const links = root.querySelectorAll(':scope > link')
    expect(links).toHaveLength(2)
    const joints = root.querySelectorAll(':scope > joint')
    expect(joints).toHaveLength(1)
  })

  it('leaves URDFs without unsupported elements unchanged', () => {
    const clean = `<robot name="clean"><link name="base"/></robot>`
    const result = stripUnsupportedURDFElements(clean)
    // DOMParser/XMLSerializer round-trip may add XML declaration or
    // normalize whitespace, so verify structure rather than exact string.
    const doc = new DOMParser().parseFromString(result, 'application/xml')
    expect(doc.documentElement.getAttribute('name')).toBe('clean')
    expect(doc.documentElement.children).toHaveLength(1)
    expect(doc.documentElement.children[0].getAttribute('name')).toBe('base')
  })

  it('returns original XML for input without <robot>', () => {
    const broken = '<not-xml-at-all'
    const result = stripUnsupportedURDFElements(broken)
    expect(result).toBe(broken)
  })

  it('returns original XML for non-robot root element', () => {
    const mjcf = '<mujoco><worldbody/></mujoco>'
    const result = stripUnsupportedURDFElements(mjcf)
    // DOMParser parses this successfully but root is <mujoco>, not <robot>,
    // so the function returns unchanged via XMLSerializer round-trip.
    const doc = new DOMParser().parseFromString(result, 'application/xml')
    expect(doc.documentElement.tagName).toBe('mujoco')
  })

  it('handles XML comments containing the word "material" without breaking', () => {
    const xml = `<robot name="commented">
  <!-- material database reference -->
  <material name="top_level_mat">
    <color rgba="1 0 0 1"/>
  </material>
  <link name="base"/>
</robot>`
    const result = stripUnsupportedURDFElements(xml)
    // The top-level <material> should be stripped
    const doc = new DOMParser().parseFromString(result, 'application/xml')
    const rootMaterials = Array.from(doc.documentElement.children).filter(c => c.tagName === 'material')
    expect(rootMaterials).toHaveLength(0)
    // The link should still be there
    expect(doc.documentElement.querySelector('link')).not.toBeNull()
  })

  it('preserves link order and joint order after stripping', () => {
    const xml = `<robot name="ordered">
  <material name="mat1"><color rgba="1 0 0 1"/></material>
  <link name="a"/>
  <link name="b"/>
  <link name="c"/>
  <joint name="j1" type="fixed"><parent link="a"/><child link="b"/></joint>
  <joint name="j2" type="fixed"><parent link="b"/><child link="c"/></joint>
  <transmission name="t1"><joint name="j1"/></transmission>
</robot>`
    const result = stripUnsupportedURDFElements(xml)
    const doc = new DOMParser().parseFromString(result, 'application/xml')
    const links = Array.from(doc.documentElement.querySelectorAll(':scope > link'))
    expect(links.map(l => l.getAttribute('name'))).toEqual(['a', 'b', 'c'])
    const joints = Array.from(doc.documentElement.querySelectorAll(':scope > joint'))
    expect(joints.map(j => j.getAttribute('name'))).toEqual(['j1', 'j2'])
  })
})

// ---------------------------------------------------------------------------
// stripUnsupportedURDFElements — regex fallback (Web Worker contexts)
// ---------------------------------------------------------------------------

describe('stripUnsupportedURDFElements (regex fallback)', () => {
  // Snapshot the real DOMParser so we can null it out per-test and restore
  // afterwards. Mirrors the runtime feature-detect in stripUnsupportedURDFElements.
  const realDOMParser = (globalThis as { DOMParser?: typeof DOMParser }).DOMParser

  afterEach(() => {
    Object.defineProperty(globalThis, 'DOMParser', {
      configurable: true,
      writable: true,
      value: realDOMParser,
    })
  })

  function withoutDOMParser<T>(fn: () => T): T {
    Object.defineProperty(globalThis, 'DOMParser', {
      configurable: true,
      writable: true,
      value: undefined,
    })
    try {
      return fn()
    } finally {
      Object.defineProperty(globalThis, 'DOMParser', {
        configurable: true,
        writable: true,
        value: realDOMParser,
      })
    }
  }

  it('strips root <material>, <transmission>, and <gazebo> via regex', () => {
    const result = withoutDOMParser(() => stripUnsupportedURDFElements(URDF_WITH_ALL_UNSUPPORTED))
    expect(result).not.toContain('<transmission')
    expect(result).not.toContain('<gazebo')
    expect(result).not.toMatch(/<material\s/)
    // Real-DOMParser path is restored, so we can use it for structural assertions.
    const doc = new DOMParser().parseFromString(result, 'application/xml')
    expect(doc.documentElement.getAttribute('name')).toBe('full_test')
    expect(doc.documentElement.querySelectorAll(':scope > link')).toHaveLength(2)
    expect(doc.documentElement.querySelectorAll(':scope > joint')).toHaveLength(1)
  })

  it('preserves <material> nested inside <visual> when DOMParser is unavailable', () => {
    const result = withoutDOMParser(() =>
      stripUnsupportedURDFElements(URDF_WITH_ROOT_MATERIALS),
    )
    // Top-level <material name="3d_printed">…</material> with children gone.
    expect(result).not.toMatch(/<material name="3d_printed">/)
    // Nested <material name="3d_printed"/> reference inside <visual> survives.
    expect(result).toMatch(/<visual>[\s\S]*<material name="3d_printed"\/>[\s\S]*<\/visual>/)
  })
})
