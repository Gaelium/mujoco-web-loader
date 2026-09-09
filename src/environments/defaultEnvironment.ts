/**
 * The package's built-in default environment: a bare grid ground plane with
 * default physics and no primitives.
 *
 * Value-identical to the host app's `EMPTY_PRESET` (D-SP-4, SPLIT-NOTES.md):
 * `preprocessURDF` reads `ground.size`/`ground.type`, so this object's values
 * are behavior-relevant — both consumers must see the same ground.
 */

import type { EnvironmentConfig } from '../types/environment'
import { DEFAULT_PHYSICS_PARAMS } from '../types/environment'

/** Empty environment: just a ground plane and default physics. */
export const DEFAULT_ENVIRONMENT: EnvironmentConfig = {
  id: 'empty',
  name: 'Empty',
  description: 'Flat ground plane with default physics settings.',
  physics: DEFAULT_PHYSICS_PARAMS,
  ground: {
    type: 'grid',
    size: 10,
    color1: [0.4, 0.4, 0.4, 1],
    color2: [0.6, 0.6, 0.6, 1],
  },
  lighting: {
    ambientIntensity: 0.4,
    directionalIntensity: 0.8,
    directionalPosition: [1, 1, 2],
    shadows: true,
  },
  primitives: [],
}
