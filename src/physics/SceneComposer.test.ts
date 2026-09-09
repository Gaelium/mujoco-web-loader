import { describe, it, expect } from 'vitest'
import {
  composeScene,
  composeStandaloneRobot,
  generateOptionXml,
  generateGroundXml,
  generatePrimitiveXml,
  generateRobotIncludeXml,
  primitiveDiagInertia,
  escapeXmlAttr,
  vec3Attr,
  quatAttr,
  rgbaAttr,
} from './SceneComposer'
import { DEFAULT_ENVIRONMENT as EMPTY_PRESET } from '../environments/defaultEnvironment'
import type { EnvironmentConfig, ScenePrimitive } from '../types/environment'
import type { RobotPlacement } from './SceneComposer'

// Local fixtures — literal copies of the host app's TABLETOP/WAREHOUSE
// presets (the app's preset catalog stays app-side per the OPEN-SOURCE-SPLIT
// cut; SPLIT-NOTES D-SP-4). Data identical, import severed.
const TABLETOP_PRESET: EnvironmentConfig = {
  id: 'tabletop',
  name: 'Tabletop',
  description: 'Raised table surface for manipulation and pick-and-place tasks.',
  physics: EMPTY_PRESET.physics,
  ground: {
    type: 'solid',
    size: 10,
    color1: [0.3, 0.3, 0.3, 1],
    color2: [0.3, 0.3, 0.3, 1],
  },
  lighting: {
    ambientIntensity: 0.5,
    directionalIntensity: 0.7,
    directionalPosition: [0, -1, 2],
    shadows: true,
  },
  primitives: [
    {
      id: 'table',
      name: 'Table',
      shape: 'box',
      size: [0.4, 0.3, 0.02],
      position: [0.5, 0, 0.4],
      quaternion: [1, 0, 0, 0],
      rgba: [0.6, 0.4, 0.2, 1],
      mass: 0,
    },
    {
      id: 'table-leg-fl',
      name: 'Table Leg FL',
      shape: 'cylinder',
      size: [0.02, 0.4, 0],
      position: [0.15, -0.25, 0.2],
      quaternion: [1, 0, 0, 0],
      rgba: [0.5, 0.35, 0.15, 1],
      mass: 0,
    },
    {
      id: 'table-leg-fr',
      name: 'Table Leg FR',
      shape: 'cylinder',
      size: [0.02, 0.4, 0],
      position: [0.85, -0.25, 0.2],
      quaternion: [1, 0, 0, 0],
      rgba: [0.5, 0.35, 0.15, 1],
      mass: 0,
    },
    {
      id: 'table-leg-bl',
      name: 'Table Leg BL',
      shape: 'cylinder',
      size: [0.02, 0.4, 0],
      position: [0.15, 0.25, 0.2],
      quaternion: [1, 0, 0, 0],
      rgba: [0.5, 0.35, 0.15, 1],
      mass: 0,
    },
    {
      id: 'table-leg-br',
      name: 'Table Leg BR',
      shape: 'cylinder',
      size: [0.02, 0.4, 0],
      position: [0.85, 0.25, 0.2],
      quaternion: [1, 0, 0, 0],
      rgba: [0.5, 0.35, 0.15, 1],
      mass: 0,
    },
  ],
}

const WAREHOUSE_PRESET: EnvironmentConfig = {
  id: 'warehouse',
  name: 'Warehouse',
  description: 'Enclosed warehouse with shelving and obstacles.',
  physics: EMPTY_PRESET.physics,
  ground: {
    type: 'solid',
    size: 20,
    color1: [0.55, 0.55, 0.55, 1],
    color2: [0.55, 0.55, 0.55, 1],
  },
  lighting: {
    ambientIntensity: 0.3,
    directionalIntensity: 0.6,
    directionalPosition: [0, 0, 5],
    shadows: false,
  },
  primitives: [
    {
      id: 'shelf-1',
      name: 'Shelf 1',
      shape: 'box',
      size: [0.6, 0.15, 0.8],
      position: [2, -2, 0.8],
      quaternion: [1, 0, 0, 0],
      rgba: [0.4, 0.4, 0.45, 1],
      mass: 0,
    },
    {
      id: 'shelf-2',
      name: 'Shelf 2',
      shape: 'box',
      size: [0.6, 0.15, 0.8],
      position: [2, 2, 0.8],
      quaternion: [1, 0, 0, 0],
      rgba: [0.4, 0.4, 0.45, 1],
      mass: 0,
    },
    {
      id: 'crate',
      name: 'Crate',
      shape: 'box',
      size: [0.15, 0.15, 0.15],
      position: [1, 0, 0.15],
      quaternion: [1, 0, 0, 0],
      rgba: [0.6, 0.5, 0.3, 1],
      mass: 5,
    },
  ],
}

// ---------------------------------------------------------------------------
// XML helpers
// ---------------------------------------------------------------------------

describe('escapeXmlAttr', () => {
  it('escapes ampersands', () => {
    expect(escapeXmlAttr('a&b')).toBe('a&amp;b')
  })

  it('escapes angle brackets', () => {
    expect(escapeXmlAttr('<tag>')).toBe('&lt;tag&gt;')
  })

  it('escapes quotes', () => {
    expect(escapeXmlAttr('"hello"')).toBe('&quot;hello&quot;')
  })

  it('escapes apostrophes', () => {
    expect(escapeXmlAttr("it's")).toBe('it&apos;s')
  })

  it('handles clean strings unchanged', () => {
    expect(escapeXmlAttr('clean')).toBe('clean')
  })
})

describe('vec3Attr', () => {
  it('formats a Vec3', () => {
    expect(vec3Attr([1, 2, 3])).toBe('1 2 3')
  })

  it('handles decimals', () => {
    expect(vec3Attr([0.1, -0.2, 9.81])).toBe('0.1 -0.2 9.81')
  })
})

describe('quatAttr', () => {
  it('formats a quaternion', () => {
    expect(quatAttr([1, 0, 0, 0])).toBe('1 0 0 0')
  })
})

describe('rgbaAttr', () => {
  it('formats RGBA', () => {
    expect(rgbaAttr([0.5, 0.5, 0.5, 1])).toBe('0.5 0.5 0.5 1')
  })
})

// ---------------------------------------------------------------------------
// Fragment generators
// ---------------------------------------------------------------------------

describe('generateOptionXml', () => {
  it('includes timestep', () => {
    const xml = generateOptionXml(EMPTY_PRESET)
    expect(xml).toContain('timestep="0.002"')
  })

  it('includes iterations', () => {
    const xml = generateOptionXml(EMPTY_PRESET)
    expect(xml).toContain('iterations="50"')
  })

  it('includes gravity', () => {
    const xml = generateOptionXml(EMPTY_PRESET)
    expect(xml).toContain('gravity="0 0 -9.81"')
  })
})

describe('generateGroundXml', () => {
  it('generates a plane geom for grid ground', () => {
    const xml = generateGroundXml(EMPTY_PRESET)
    expect(xml).toContain('type="plane"')
    expect(xml).toContain('name="ground"')
    expect(xml).toContain('size="10 10 0.01"')
  })

  it('includes friction from physics params', () => {
    const xml = generateGroundXml(EMPTY_PRESET)
    expect(xml).toContain(`friction="1 0.005 0.0001"`)
  })

  it('returns empty string for "none" ground type', () => {
    const config: EnvironmentConfig = {
      ...EMPTY_PRESET,
      ground: { ...EMPTY_PRESET.ground, type: 'none' },
    }
    expect(generateGroundXml(config)).toBe('')
  })
})

describe('generatePrimitiveXml', () => {
  const staticPrim: ScenePrimitive = {
    id: 'box1',
    name: 'Test Box',
    shape: 'box',
    size: [0.1, 0.2, 0.3],
    position: [1, 2, 3],
    quaternion: [1, 0, 0, 0],
    rgba: [1, 0, 0, 1],
    mass: 0,
  }

  const dynamicPrim: ScenePrimitive = {
    ...staticPrim,
    id: 'dyn-box',
    name: 'Dynamic Box',
    mass: 5,
  }

  it('wraps the geom in a body element', () => {
    const xml = generatePrimitiveXml(staticPrim)
    expect(xml).toContain('<body name="Test Box"')
    expect(xml).toContain('</body>')
  })

  it('includes position and quaternion on the body', () => {
    const xml = generatePrimitiveXml(staticPrim)
    expect(xml).toContain('pos="1 2 3"')
    expect(xml).toContain('quat="1 0 0 0"')
  })

  it('sets the geom shape and size', () => {
    const xml = generatePrimitiveXml(staticPrim)
    expect(xml).toContain('type="box"')
    expect(xml).toContain('size="0.1 0.2 0.3"')
  })

  it('includes RGBA', () => {
    const xml = generatePrimitiveXml(staticPrim)
    expect(xml).toContain('rgba="1 0 0 1"')
  })

  it('does not add freejoint for static objects (mass=0)', () => {
    const xml = generatePrimitiveXml(staticPrim)
    expect(xml).not.toContain('freejoint')
    expect(xml).not.toContain('inertial')
  })

  it('adds freejoint and inertial for dynamic objects (mass>0)', () => {
    const xml = generatePrimitiveXml(dynamicPrim)
    expect(xml).toContain('<freejoint/>')
    expect(xml).toContain('mass="5"')
  })

  it('emits shape-derived diagonal inertia for dynamic objects (T3-F1)', () => {
    const xml = generatePrimitiveXml(dynamicPrim)
    const [ixx, iyy, izz] = primitiveDiagInertia(dynamicPrim)
    expect(xml).toContain(`diaginertia="${ixx} ${iyy} ${izz}"`)
    expect(xml).not.toContain('diaginertia="0.01 0.01 0.01"')
  })

  it('computes uniform-density inertia per shape', () => {
    const box = primitiveDiagInertia(dynamicPrim)
    expect(box[0]).toBeCloseTo((5 / 3) * (0.2 * 0.2 + 0.3 * 0.3), 10)
    expect(box[1]).toBeCloseTo((5 / 3) * (0.1 * 0.1 + 0.3 * 0.3), 10)
    expect(box[2]).toBeCloseTo((5 / 3) * (0.1 * 0.1 + 0.2 * 0.2), 10)

    const sphere = primitiveDiagInertia({
      ...dynamicPrim,
      shape: 'sphere',
      size: [0.1, 0, 0],
      mass: 2,
    })
    expect(sphere[0]).toBeCloseTo(0.008, 10)
    expect(sphere[1]).toBeCloseTo(0.008, 10)
    expect(sphere[2]).toBeCloseTo(0.008, 10)

    const cyl = primitiveDiagInertia({
      ...dynamicPrim,
      shape: 'cylinder',
      size: [0.1, 0.2, 0],
      mass: 3,
    })
    expect(cyl[0]).toBeCloseTo((3 * (3 * 0.01 + 4 * 0.04)) / 12, 10)
    expect(cyl[1]).toBeCloseTo(cyl[0], 10)
    expect(cyl[2]).toBeCloseTo(0.015, 10)
  })

  it('escapes special characters in names', () => {
    const prim: ScenePrimitive = {
      ...staticPrim,
      name: 'Box "A" & <B>',
      id: 'special&id',
    }
    const xml = generatePrimitiveXml(prim)
    expect(xml).toContain('name="Box &quot;A&quot; &amp; &lt;B&gt;"')
    expect(xml).toContain('name="special&amp;id"')
  })

  it('uses box type as collision approximation for mesh shapes', () => {
    const meshPrim: ScenePrimitive = {
      id: 'mesh1',
      name: 'Custom Mesh',
      shape: 'mesh',
      size: [0.2, 0.3, 0.4],
      position: [0, 0, 0],
      quaternion: [1, 0, 0, 0],
      rgba: [0.5, 0.5, 0.5, 1],
      mass: 0,
      meshId: 'some-mesh-id',
    }
    const xml = generatePrimitiveXml(meshPrim)
    expect(xml).toContain('type="box"')
    expect(xml).not.toContain('type="mesh"')
    expect(xml).toContain('size="0.2 0.3 0.4"')
  })
})

describe('generateRobotIncludeXml', () => {
  it('generates an include directive', () => {
    const robot: RobotPlacement = {
      id: 'arm1',
      vfsPath: '/robots/arm.urdf',
      position: [0, 0, 0],
      quaternion: [1, 0, 0, 0],
    }
    const xml = generateRobotIncludeXml(robot)
    expect(xml).toContain('<include file="/robots/arm.urdf"/>')
  })

  it('escapes path characters', () => {
    const robot: RobotPlacement = {
      id: 'r1',
      vfsPath: '/robots/my "robot".urdf',
      position: [0, 0, 0],
      quaternion: [1, 0, 0, 0],
    }
    const xml = generateRobotIncludeXml(robot)
    expect(xml).toContain('file="/robots/my &quot;robot&quot;.urdf"')
  })
})

// ---------------------------------------------------------------------------
// Full scene composition
// ---------------------------------------------------------------------------

describe('composeScene', () => {
  it('produces valid-looking MJCF with root element', () => {
    const xml = composeScene(EMPTY_PRESET)
    expect(xml).toContain('<mujoco model="robot-sim-scene">')
    expect(xml).toContain('</mujoco>')
  })

  it('includes compiler element', () => {
    const xml = composeScene(EMPTY_PRESET)
    expect(xml).toContain('<compiler angle="radian"')
    expect(xml).toContain('meshdir="/working"')
  })

  it('includes option element with physics params', () => {
    const xml = composeScene(EMPTY_PRESET)
    expect(xml).toContain('timestep="0.002"')
    expect(xml).toContain('gravity="0 0 -9.81"')
  })

  it('includes worldbody with ground', () => {
    const xml = composeScene(EMPTY_PRESET)
    expect(xml).toContain('<worldbody>')
    expect(xml).toContain('</worldbody>')
    expect(xml).toContain('name="ground"')
    expect(xml).toContain('type="plane"')
  })

  it('includes a light element', () => {
    const xml = composeScene(EMPTY_PRESET)
    expect(xml).toContain('name="main_light"')
    expect(xml).toContain('castshadow="true"')
  })

  it('includes default friction settings', () => {
    const xml = composeScene(EMPTY_PRESET)
    expect(xml).toContain('<default>')
    expect(xml).toContain('friction="1 0.005 0.0001"')
    expect(xml).toContain('</default>')
  })

  it('includes primitives from the environment', () => {
    const xml = composeScene(TABLETOP_PRESET)
    expect(xml).toContain('name="Table"')
    expect(xml).toContain('type="box"')
  })

  it('includes dynamic primitives with freejoint', () => {
    const xml = composeScene(WAREHOUSE_PRESET)
    expect(xml).toContain('<freejoint/>')
    expect(xml).toContain('mass="5"')
  })

  it('omits ground when type is "none"', () => {
    const config: EnvironmentConfig = {
      ...EMPTY_PRESET,
      ground: { ...EMPTY_PRESET.ground, type: 'none' },
    }
    const xml = composeScene(config)
    expect(xml).not.toContain('name="ground"')
  })

  it('includes robot includes when provided', () => {
    const robots: RobotPlacement[] = [
      { id: 'r1', vfsPath: '/robot.urdf', position: [0, 0, 0], quaternion: [1, 0, 0, 0] },
    ]
    const xml = composeScene(EMPTY_PRESET, robots)
    expect(xml).toContain('<include file="/robot.urdf"/>')
  })

  it('places robot include at <mujoco> level, outside <worldbody>', () => {
    const robots: RobotPlacement[] = [
      { id: 'r1', vfsPath: '/robot.urdf', position: [0, 0, 0], quaternion: [1, 0, 0, 0] },
    ]
    const xml = composeScene(EMPTY_PRESET, robots)
    // The include must appear AFTER </worldbody> for MuJoCo to properly
    // merge the URDF's converted worldbody with the scene's worldbody.
    const worldbodyEnd = xml.indexOf('</worldbody>')
    const includePos = xml.indexOf('<include file="/robot.urdf"/>')
    expect(includePos).toBeGreaterThan(worldbodyEnd)
  })

  it('includes multiple robots', () => {
    const robots: RobotPlacement[] = [
      { id: 'r1', vfsPath: '/arm.urdf', position: [0, 0, 0], quaternion: [1, 0, 0, 0] },
      { id: 'r2', vfsPath: '/leg.urdf', position: [1, 0, 0], quaternion: [1, 0, 0, 0] },
    ]
    const xml = composeScene(EMPTY_PRESET, robots)
    expect(xml).toContain('<include file="/arm.urdf"/>')
    expect(xml).toContain('<include file="/leg.urdf"/>')
  })

  it('uses outdoor friction when specified', () => {
    const xml = composeScene({
      ...EMPTY_PRESET,
      physics: { ...EMPTY_PRESET.physics, friction: 0.5 },
    })
    expect(xml).toContain('friction="0.5 0.005 0.0001"')
  })
})

// ---------------------------------------------------------------------------
// Bug reproduction: environment change drops robot
// ---------------------------------------------------------------------------

describe('composeScene — environment-only recomposition', () => {
  /**
   * When a robot is loaded, SimWorker updates physics params in-place
   * rather than recomposing (because MuJoCo can't nest URDF inside MJCF
   * worldbody via <include>). These tests verify that the environment-only
   * scene (used when no robot is loaded) composes correctly for each preset.
   */

  it('should compose environment-only scene without robot includes', () => {
    const xml = composeScene(TABLETOP_PRESET)
    expect(xml).not.toContain('<include')
    expect(xml).toContain('name="Table"')
  })

  it('should compose different environments with correct primitives', () => {
    const emptyXml = composeScene(EMPTY_PRESET)
    expect(emptyXml).not.toContain('name="Table"')
    expect(emptyXml).not.toContain('name="Shelf 1"')

    const warehouseXml = composeScene(WAREHOUSE_PRESET)
    expect(warehouseXml).toContain('name="Shelf 1"')
    expect(warehouseXml).toContain('name="Crate"')
  })

  it('should include physics params from the environment config', () => {
    const customConfig: EnvironmentConfig = {
      ...EMPTY_PRESET,
      physics: { ...EMPTY_PRESET.physics, timestep: 0.001, gravity: [0, 0, -3.71] },
    }
    const xml = composeScene(customConfig)
    expect(xml).toContain('timestep="0.001"')
    expect(xml).toContain('gravity="0 0 -3.71"')
  })
})

describe('composeStandaloneRobot', () => {
  it('produces valid MJCF', () => {
    const xml = composeStandaloneRobot('/my-robot.urdf')
    expect(xml).toContain('<mujoco model="standalone-robot">')
    expect(xml).toContain('</mujoco>')
  })

  it('includes the URDF via include directive', () => {
    const xml = composeStandaloneRobot('/my-robot.urdf')
    expect(xml).toContain('<include file="/my-robot.urdf"/>')
  })

  it('wraps the robot in a positioned body', () => {
    const xml = composeStandaloneRobot('/r.urdf', [1, 2, 3], [0.5, 0.5, 0.5, 0.5])
    expect(xml).toContain('pos="1 2 3"')
    expect(xml).toContain('quat="0.5 0.5 0.5 0.5"')
  })

  it('uses default position and orientation', () => {
    const xml = composeStandaloneRobot('/r.urdf')
    expect(xml).toContain('pos="0 0 0"')
    expect(xml).toContain('quat="1 0 0 0"')
  })

  it('includes a ground plane', () => {
    const xml = composeStandaloneRobot('/r.urdf')
    expect(xml).toContain('type="plane"')
  })

  it('includes default physics options', () => {
    const xml = composeStandaloneRobot('/r.urdf')
    expect(xml).toContain('timestep="0.002"')
    expect(xml).toContain('gravity="0 0 -9.81"')
  })
})
