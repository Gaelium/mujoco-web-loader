/**
 * URDF validation and mesh path extraction.
 *
 * Parses URDF XML using DOMParser, validates its structure, and extracts
 * mesh file paths referenced in <visual> and <collision> elements. This
 * information is needed to write mesh assets to the VFS before MuJoCo
 * can compile the model.
 *
 * Runs exclusively inside the physics Web Worker.
 */

import type { SimResult } from '../types/simulation'

// ---------------------------------------------------------------------------
// Result types
// ---------------------------------------------------------------------------

/** Information extracted from a parsed URDF. */
export interface URDFParseResult {
  /** Robot name from <robot name="…">. */
  readonly name: string
  /** Link names found in the URDF. */
  readonly links: readonly string[]
  /** Joint definitions. */
  readonly joints: readonly URDFJointInfo[]
  /** All mesh file paths referenced in <visual> and <collision> geometry. */
  readonly meshPaths: readonly string[]
  /** Validation warnings (non-fatal issues). */
  readonly warnings: readonly string[]
}

/** Joint info extracted from URDF <joint> elements. */
export interface URDFJointInfo {
  readonly name: string
  readonly type: string
  readonly parent: string
  readonly child: string
}

/** A validation error found in the URDF. */
export interface URDFValidationError {
  readonly message: string
  readonly element?: string
}

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------

/**
 * Parse and validate a URDF XML string.
 * @param xml - Raw URDF XML content
 * @returns SimResult containing the parse result or an error message
 */
export function parseURDF(xml: string): SimResult<URDFParseResult> {
  const parser = new DOMParser()
  const doc = parser.parseFromString(xml, 'application/xml')

  // Check for XML parse errors
  const parseError = doc.querySelector('parsererror')
  if (parseError) {
    return {
      ok: false,
      error: 'Invalid XML',
      detail: parseError.textContent ?? 'Unknown parse error',
    }
  }

  // Validate root element
  const root = doc.documentElement
  if (root.tagName !== 'robot') {
    return {
      ok: false,
      error: `Expected <robot> root element, found <${root.tagName}>`,
    }
  }

  const robotName = root.getAttribute('name') ?? 'unnamed_robot'
  const warnings: string[] = []

  if (!root.getAttribute('name')) {
    warnings.push('Robot element is missing a "name" attribute.')
  }

  // Extract links (only direct children of <robot>, not nested in <transmission> etc.)
  const linkElements = root.querySelectorAll(':scope > link')
  const links: string[] = []
  const linkNames = new Set<string>()
  for (const linkEl of linkElements) {
    const name = linkEl.getAttribute('name')
    if (!name) {
      warnings.push('Found <link> element without a "name" attribute.')
      continue
    }
    if (linkNames.has(name)) {
      warnings.push(`Duplicate link name: "${name}".`)
    }
    linkNames.add(name)
    links.push(name)
  }

  if (links.length === 0) {
    return {
      ok: false,
      error: 'URDF contains no <link> elements.',
    }
  }

  // Extract joints (only direct children of <robot>, not nested in <transmission>)
  const jointElements = root.querySelectorAll(':scope > joint')
  const joints: URDFJointInfo[] = []
  const jointNames = new Set<string>()
  for (const jointEl of jointElements) {
    const name = jointEl.getAttribute('name')
    const type = jointEl.getAttribute('type')
    const parentEl = jointEl.querySelector('parent')
    const childEl = jointEl.querySelector('child')

    if (!name) {
      warnings.push('Found <joint> element without a "name" attribute.')
      continue
    }
    if (jointNames.has(name)) {
      warnings.push(`Duplicate joint name: "${name}".`)
    }
    jointNames.add(name)

    if (!type) {
      warnings.push(`Joint "${name}" is missing a "type" attribute.`)
    }

    const parent = parentEl?.getAttribute('link') ?? ''
    const child = childEl?.getAttribute('link') ?? ''

    if (!parent) {
      warnings.push(`Joint "${name}" is missing a <parent link="…"/> element.`)
    }
    if (!child) {
      warnings.push(`Joint "${name}" is missing a <child link="…"/> element.`)
    }

    if (parent && !linkNames.has(parent)) {
      warnings.push(`Joint "${name}" references unknown parent link "${parent}".`)
    }
    if (child && !linkNames.has(child)) {
      warnings.push(`Joint "${name}" references unknown child link "${child}".`)
    }

    joints.push({ name, type: type ?? 'fixed', parent, child })
  }

  // Extract mesh paths from <visual> and <collision> geometry
  const meshPaths = extractMeshPaths(root)

  return {
    ok: true,
    value: {
      name: robotName,
      links,
      joints,
      meshPaths,
      warnings,
    },
  }
}

// ---------------------------------------------------------------------------
// URDF preprocessing for MuJoCo compatibility
// ---------------------------------------------------------------------------

/**
 * URDF elements that MuJoCo's URDF compiler rejects.
 *
 * MuJoCo supports only a subset of URDF and raises schema violation errors
 * for unsupported top-level elements. Only DIRECT children of `<robot>` are
 * stripped — nested occurrences (e.g. `<material>` inside `<visual>`) are
 * valid URDF and MuJoCo accepts them.
 *
 * To add a new element to this list:
 * 1. Confirm MuJoCo rejects it by attempting to load a URDF that contains
 *    it and checking the error from `printErr` in SimWorker.worker.ts
 * 2. Verify the element only appears as a direct child of `<robot>` in URDF
 *    schemas — if it can be nested, this stripping approach won't work
 * 3. Add the tag name (lowercase) to this array
 * 4. Add a test case to URDFLoader.test.ts demonstrating the element is
 *    stripped from the root but preserved when nested
 *
 * Stripping is visually lossless because the Three.js rendering layer
 * (URDFVisualLoader) reads materials and other metadata directly from
 * the original URDF, not from the version passed to MuJoCo.
 */
const UNSUPPORTED_ROOT_ELEMENTS = ['material', 'transmission', 'gazebo'] as const

/**
 * Strip URDF elements that MuJoCo's URDF compiler rejects.
 *
 * Uses DOMParser to parse the XML and iterates only the direct children
 * of `<robot>`, removing those whose tagName is in the unsupported list.
 * This correctly handles XML comments, CDATA sections, and nested elements
 * with the same name (e.g. `<material>` inside `<visual>` is untouched).
 *
 * Falls back to a simple regex approach in Web Worker contexts where
 * DOMParser may not be available (depends on browser/bundler).
 *
 * @param xml - Raw URDF XML string
 * @returns Cleaned URDF XML string with unsupported root elements removed
 */
export function stripUnsupportedURDFElements(xml: string): string {
  // DOMParser is available on the main thread and in most modern Web
  // Worker environments, but some worker/bundler configurations lack it.
  if (typeof DOMParser !== 'undefined') {
    return stripViaDOM(xml)
  }
  return stripViaRegex(xml)
}

/** Primary implementation using DOMParser — handles all XML edge cases. */
function stripViaDOM(xml: string): string {
  const parser = new DOMParser()
  const doc = parser.parseFromString(xml, 'application/xml')

  if (doc.querySelector('parsererror')) return xml

  const root = doc.documentElement
  if (root.tagName !== 'robot') return xml

  const removed: string[] = []
  // Snapshot children before mutation — removing during iteration of a
  // live HTMLCollection skips elements.
  const children = Array.from(root.children)
  for (const child of children) {
    if ((UNSUPPORTED_ROOT_ELEMENTS as readonly string[]).includes(child.tagName)) {
      removed.push(child.getAttribute('name') ?? child.tagName)
      root.removeChild(child)
    }
  }

  if (removed.length > 0) {
    console.log(
      `[URDFLoader] Stripped ${removed.length} unsupported root element(s): ${removed.join(', ')}`,
    )
  }

  return new XMLSerializer().serializeToString(doc)
}

/**
 * Fallback for Web Worker contexts without DOMParser.
 *
 * Removes unsupported elements that are direct children of `<robot>` by
 * iterating the XML string and tracking nesting depth. Only elements at
 * depth 1 (direct children of the root) are candidates for removal.
 * Nested occurrences (e.g. `<material>` inside `<visual>`) are untouched
 * because they appear at depth >= 2.
 */
function stripViaRegex(xml: string): string {
  if (!xml.includes('<robot')) return xml

  const robotOpenMatch = xml.match(/<robot[^>]*>/)
  const robotCloseIdx = xml.lastIndexOf('</robot>')
  if (!robotOpenMatch || robotCloseIdx < 0) return xml

  const bodyStart = robotOpenMatch.index! + robotOpenMatch[0].length
  let body = xml.slice(bodyStart, robotCloseIdx)

  let strippedCount = 0
  const strippedNames: string[] = []

  for (const tagName of UNSUPPORTED_ROOT_ELEMENTS) {
    // Remove paired elements at depth 0 within the <robot> body.
    // We track depth: depth increments on every open tag, decrements on
    // every close tag. Only elements opening at depth 0 are direct
    // children of <robot>.
    body = removeDirectChildren(body, tagName, strippedNames)
  }

  strippedCount = strippedNames.length
  if (strippedCount > 0) {
    console.log(
      `[URDFLoader] Stripped ${strippedCount} unsupported root element(s): ${strippedNames.join(', ')}`,
    )
  }

  return xml.slice(0, bodyStart) + body + xml.slice(robotCloseIdx)
}

/** Remove all direct-child occurrences of `tagName` from an XML body string. */
function removeDirectChildren(body: string, tagName: string, removed: string[]): string {
  // Match both paired (<tag ...>...</tag>) and self-closing (<tag .../>)
  // elements. The regex matches any occurrence; we filter by checking
  // that the match is at nesting depth 0 (direct child of root).
  const tagRe = new RegExp(
    `[ \\t]*<${tagName}(\\s[^>]*)?>([\\s\\S]*?)<\\/${tagName}>[ \\t]*\\n?` +
    `|[ \\t]*<${tagName}(\\s[^>]*)?\\/>[\\t ]*\\n?`,
    'g',
  )

  // Pre-compute the nesting depth at each character position.
  // A match at depth 0 is a direct child; anything deeper is nested.
  const depths = computeDepths(body, tagName)

  return body.replace(tagRe, (match, _a1, _a2, _a3, offset: number) => {
    // The regex captures leading whitespace, so the recorded depth lives at
    // the `<` position, not at the match start. Look up depth at the actual
    // tag-open position.
    const tagStart = offset + match.indexOf('<')
    if (depths[tagStart] !== 0) return match // nested — keep
    const nameAttr = match.match(/name=["']([^"']*)["']/)
    removed.push(nameAttr ? nameAttr[1] : tagName)
    return ''
  })
}

/**
 * Compute the nesting depth of generic XML tags at each position in `body`.
 * Depth 0 = direct child of root. Skips the target `tagName` from depth
 * tracking so its own open/close tags don't interfere with depth calculation.
 */
function computeDepths(body: string, skipTag: string): Uint16Array {
  const depths = new Uint16Array(body.length)
  let depth = 0
  // Match open tags, close tags, and self-closing tags
  const tagRe = /<(\/?)([\w-]+)([^>]*?)(\/?)>/g
  let m: RegExpExecArray | null
  while ((m = tagRe.exec(body)) !== null) {
    const isClose = m[1] === '/'
    const name = m[2]
    const isSelfClose = m[4] === '/'
    // Record the depth at this position
    if (isClose) {
      if (name !== skipTag) depth = Math.max(0, depth - 1)
      depths[m.index] = depth
    } else {
      depths[m.index] = depth
      if (!isSelfClose && name !== skipTag) depth++
    }
  }
  return depths
}

/**
 * Extract all mesh file paths from a URDF document.
 * Searches both <visual><geometry><mesh> and <collision><geometry><mesh>.
 * @param root - The <robot> root element
 * @returns Deduplicated array of mesh file paths
 */
export function extractMeshPaths(root: Element): readonly string[] {
  const paths = new Set<string>()

  const meshElements = root.querySelectorAll('mesh')
  for (const meshEl of meshElements) {
    const filename = meshEl.getAttribute('filename')
    if (filename) {
      paths.add(normalizeMeshPath(filename))
    }
  }

  return [...paths]
}

/**
 * Normalize a mesh file path from a URDF.
 * Strips common prefixes like "package://" and "file://".
 * @param raw - Raw path from the URDF mesh filename attribute
 * @returns Normalized relative path
 */
export function normalizeMeshPath(raw: string): string {
  let path = raw.trim()
  if (path.startsWith('package://')) {
    path = path.slice('package://'.length)
  } else if (path.startsWith('file://')) {
    path = path.slice('file://'.length)
  }
  // Remove leading slashes for consistent relative paths
  while (path.startsWith('/')) {
    path = path.slice(1)
  }
  return path
}

/**
 * Validate a URDF string and return structured errors.
 * This is a stricter check than parseURDF — it returns errors for
 * conditions that parseURDF only warns about.
 * @param xml - Raw URDF XML content
 * @returns Array of validation errors (empty = valid)
 */
export function validateURDF(xml: string): readonly URDFValidationError[] {
  const errors: URDFValidationError[] = []

  const result = parseURDF(xml)
  if (!result.ok) {
    errors.push({ message: result.error })
    return errors
  }

  const { value } = result

  if (value.links.length === 0) {
    errors.push({ message: 'URDF must contain at least one <link> element.' })
  }

  // Check for kinematic tree validity: each non-root link should be a child of some joint
  const childLinks = new Set(value.joints.map(j => j.child))
  const rootLinks = value.links.filter(l => !childLinks.has(l))

  if (rootLinks.length === 0 && value.links.length > 0) {
    errors.push({ message: 'No root link found — every link is a child of some joint.' })
  }
  if (rootLinks.length > 1) {
    errors.push({
      message: `Multiple root links found: ${rootLinks.join(', ')}. URDF should have exactly one root.`,
    })
  }

  return errors
}

/**
 * Extract just the mesh filenames (basenames) from a URDF.
 * Useful for checking which assets need to be uploaded.
 * @param xml - Raw URDF XML content
 * @returns Array of mesh filenames, or empty array if parsing fails
 */
export function getMeshFilenames(xml: string): readonly string[] {
  const result = parseURDF(xml)
  if (!result.ok) return []
  return result.value.meshPaths.map(p => {
    const parts = p.split('/')
    return parts[parts.length - 1]
  })
}
