#!/usr/bin/env node
const fs = require('fs');
const path = require('path');

const root = process.cwd();
const uaDir = path.join(root, '.understand-anything');
const tmpDir = path.join(uaDir, 'tmp');
const graphPath = path.join(uaDir, 'knowledge-graph.json');
const metaPath = path.join(uaDir, 'meta.json');
const parentScanPath = path.join(uaDir, 'intermediate', 'scan-result.json');
const prefix = 'src/FAST_LIO_LOCALIZATION_HUMANOID/';

const subScan = readJson(path.join(tmpDir, 'fastlio-humanoid-scan-files.json'));
const subImports = readJson(path.join(tmpDir, 'fastlio-humanoid-import-map.json')).importMap || {};
const subStructure = readJson(path.join(tmpDir, 'fastlio-humanoid-structure.json'));
const graph = readJson(graphPath);

fs.copyFileSync(graphPath, path.join(tmpDir, 'knowledge-graph.before-fastlio-humanoid.json'));

const fileMeta = new Map(subScan.files.map((file) => [file.path, file]));
const fileIdBySubPath = new Map();
const newNodes = [];
const newEdges = [];

for (const result of subStructure.results) {
  const meta = fileMeta.get(result.path);
  if (!meta) continue;

  const fileType = fileNodeType(meta);
  const prefixedPath = prefix + result.path;
  const fileId = `${fileType}:${prefixedPath}`;
  fileIdBySubPath.set(result.path, fileId);

  newNodes.push({
    id: fileId,
    type: fileType,
    name: path.basename(result.path),
    filePath: prefixedPath,
    summary: summarizeFile(result, meta),
    complexity: complexityForLines(result.nonEmptyLines || meta.sizeLines || 0),
    tags: tagsForFile(result.path, meta),
  });

  const functionNameCounts = countNames(result.functions || []);
  for (const fn of result.functions || []) {
    const lines = Math.max(0, (fn.endLine || 0) - (fn.startLine || 0) + 1);
    if (lines < 10 && fn.name !== 'main') continue;
    const idName = functionNameCounts.get(fn.name) > 1 ? `${fn.name}:${fn.startLine}` : fn.name;
    const fnId = `function:${prefixedPath}:${idName}`;
    newNodes.push({
      id: fnId,
      type: 'function',
      name: fn.name,
      filePath: prefixedPath,
      summary: `Implements ${fn.name} in ${result.path}, contributing to FAST-LIO localization or Livox driver behavior.`,
      complexity: complexityForLines(lines),
      tags: uniq(['function', ...domainTags(result.path)]),
    });
    newEdges.push(edge(fileId, fnId, 'contains', 1.0));
  }

  const classNameCounts = countNames(result.classes || []);
  for (const cls of result.classes || []) {
    const lines = Math.max(0, (cls.endLine || 0) - (cls.startLine || 0) + 1);
    const methodCount = Array.isArray(cls.methods) ? cls.methods.length : 0;
    if (lines < 20 && methodCount < 2) continue;
    const idName = classNameCounts.get(cls.name) > 1 ? `${cls.name}:${cls.startLine}` : cls.name;
    const clsId = `class:${prefixedPath}:${idName}`;
    newNodes.push({
      id: clsId,
      type: 'class',
      name: cls.name,
      filePath: prefixedPath,
      summary: `Defines ${cls.name} in ${result.path}, used by the FAST-LIO localization stack.`,
      complexity: complexityForLines(lines),
      tags: uniq(['class', ...domainTags(result.path)]),
    });
    newEdges.push(edge(fileId, clsId, 'contains', 1.0));
  }
}

for (const [source, targets] of Object.entries(subImports)) {
  const sourceId = fileIdBySubPath.get(source);
  if (!sourceId) continue;
  for (const target of targets || []) {
    const targetId = fileIdBySubPath.get(target);
    if (targetId) newEdges.push(edge(sourceId, targetId, 'imports', 0.7));
  }
}

const removedIds = new Set(
  graph.nodes
    .filter((node) => (node.filePath && node.filePath.startsWith(prefix)) || node.id.includes(prefix))
    .map((node) => node.id),
);

graph.nodes = graph.nodes.filter((node) => !removedIds.has(node.id));
graph.edges = graph.edges.filter(
  (e) => !removedIds.has(e.source) && !removedIds.has(e.target) && !e.source.includes(prefix) && !e.target.includes(prefix),
);

graph.nodes = dedupeById([...graph.nodes, ...newNodes]);
const nodeIds = new Set(graph.nodes.map((node) => node.id));
graph.edges = dedupeEdges([...graph.edges, ...newEdges]).filter(
  (e) => nodeIds.has(e.source) && nodeIds.has(e.target),
);

const fileLevelIds = newNodes
  .filter((node) => ['file', 'config', 'document', 'service', 'pipeline', 'table', 'schema', 'resource', 'endpoint'].includes(node.type))
  .map((node) => node.id);

const layer = {
  id: 'layer:fast-lio-localization-humanoid',
  name: 'FAST-LIO Localization Humanoid',
  description: 'FAST-LIO based ROS2 localization stack with Open3D global localization, Livox input support, and humanoid-oriented launch/configuration files.',
  nodeIds: fileLevelIds,
};
graph.layers = (graph.layers || []).filter((item) => item.id !== layer.id);
graph.layers.push(layer);

const tourNodeIds = [
  'document:src/FAST_LIO_LOCALIZATION_HUMANOID/README.md',
  'file:src/FAST_LIO_LOCALIZATION_HUMANOID/open3d_loc/src/global_localization.cpp',
  'file:src/FAST_LIO_LOCALIZATION_HUMANOID/FAST_LIO/src/laserMapping.cpp',
  'config:src/FAST_LIO_LOCALIZATION_HUMANOID/open3d_loc/package.xml',
  'config:src/FAST_LIO_LOCALIZATION_HUMANOID/livox_ros_driver2/package.xml',
].filter((id) => nodeIds.has(id));
const maxOrder = Math.max(0, ...(graph.tour || []).map((step) => Number(step.order) || 0));
graph.tour = (graph.tour || []).filter((step) => step.title !== 'FAST-LIO Localization Humanoid');
graph.tour.push({
  order: maxOrder + 1,
  title: 'FAST-LIO Localization Humanoid',
  description: 'Introduces the ROS2 relocalization repository: README overview, Open3D global localization, FAST-LIO odometry, and Livox MID360 input integration.',
  nodeIds: tourNodeIds,
});

graph.project = graph.project || {};
graph.project.analyzedAt = new Date().toISOString();
graph.project.gitCommitHash = gitHead();
graph.project.languages = uniq([...(graph.project.languages || []), ...Object.keys(subScan.stats?.byLanguage || {})]).sort();
graph.project.frameworks = uniq([...(graph.project.frameworks || []), 'FAST-LIO', 'Open3D', 'Livox SDK']).sort();

writeJson(graphPath, graph);
updateParentScan();
updateMeta();
writeFingerprintInput();

console.log(JSON.stringify({
  addedNodes: newNodes.length,
  addedEdges: newEdges.length,
  fileNodes: fileLevelIds.length,
  functions: newNodes.filter((node) => node.type === 'function').length,
  classes: newNodes.filter((node) => node.type === 'class').length,
  totalGraphNodes: graph.nodes.length,
  totalGraphEdges: graph.edges.length,
  totalFilesInScan: readJson(parentScanPath).totalFiles,
}, null, 2));

function updateParentScan() {
  if (!fs.existsSync(parentScanPath)) return;
  const parentScan = readJson(parentScanPath);
  const oldFiles = (parentScan.files || []).filter((file) => !file.path.startsWith(prefix));
  const addedFiles = subScan.files.map((file) => ({ ...file, path: prefix + file.path }));
  parentScan.files = [...oldFiles, ...addedFiles].sort((a, b) => a.path.localeCompare(b.path));
  parentScan.totalFiles = parentScan.files.length;

  parentScan.importMap = parentScan.importMap || {};
  for (const key of Object.keys(parentScan.importMap)) {
    if (key.startsWith(prefix)) delete parentScan.importMap[key];
  }
  for (const [source, targets] of Object.entries(subImports)) {
    parentScan.importMap[prefix + source] = (targets || []).map((target) => prefix + target);
  }
  for (const file of parentScan.files) {
    if (!parentScan.importMap[file.path]) parentScan.importMap[file.path] = [];
  }

  const byCategory = {};
  const byLanguage = {};
  for (const file of parentScan.files) {
    byCategory[file.fileCategory] = (byCategory[file.fileCategory] || 0) + 1;
    byLanguage[file.language] = (byLanguage[file.language] || 0) + 1;
  }
  parentScan.stats = {
    ...(parentScan.stats || {}),
    filesScanned: parentScan.totalFiles,
    byCategory,
    byLanguage,
  };
  parentScan.languages = Object.keys(byLanguage).sort();
  parentScan.estimatedComplexity = 'very-large';
  writeJson(parentScanPath, parentScan);
}

function updateMeta() {
  const scanTotal = fs.existsSync(parentScanPath) ? readJson(parentScanPath).totalFiles : undefined;
  const meta = fs.existsSync(metaPath) ? readJson(metaPath) : {};
  meta.lastAnalyzedAt = new Date().toISOString();
  meta.gitCommitHash = gitHead();
  meta.version = meta.version || '1.0.0';
  if (scanTotal) meta.analyzedFiles = scanTotal;
  writeJson(metaPath, meta);
}

function writeFingerprintInput() {
  const scan = fs.existsSync(parentScanPath) ? readJson(parentScanPath) : null;
  if (!scan) return;
  writeJson(path.join(tmpDir, 'fastlio-humanoid-fingerprint-input.json'), {
    projectRoot: root,
    sourceFilePaths: scan.files.map((file) => file.path),
    gitCommitHash: gitHead(),
  });
}

function summarizeFile(result, meta) {
  const rel = result.path;
  const pieces = [];
  if (rel === 'README.md') return 'Top-level documentation for the FAST_LIO_LOCALIZATION_HUMANOID relocalization stack, including pipeline overview, dependencies, build steps, and launch usage.';
  if (rel.includes('open3d_loc')) pieces.push('Open3D global localization');
  if (rel.includes('FAST_LIO')) pieces.push('FAST-LIO odometry and mapping');
  if (rel.includes('livox_ros_driver2')) pieces.push('Livox ROS driver integration');
  if (rel.includes('launch')) pieces.push('launch configuration');
  if (rel.includes('config')) pieces.push('runtime configuration');
  const scope = pieces.length ? pieces.join(', ') : 'FAST_LIO_LOCALIZATION_HUMANOID';
  if (meta.fileCategory === 'docs') return `Documents ${scope} behavior and usage.`;
  if (meta.fileCategory === 'config') return `Configures ${scope} for ROS/CMake/runtime operation.`;
  const fnCount = (result.functions || []).length;
  const clsCount = (result.classes || []).length;
  return `Implements ${scope} behavior with ${fnCount} detected functions and ${clsCount} detected classes.`;
}

function tagsForFile(rel, meta) {
  return uniq([meta.fileCategory, meta.language, ...domainTags(rel)]);
}

function domainTags(rel) {
  const tags = ['fast-lio-localization-humanoid'];
  if (rel.includes('FAST_LIO')) tags.push('fast-lio', 'lidar-odometry');
  if (rel.includes('open3d_loc')) tags.push('open3d-localization', 'global-localization');
  if (rel.includes('livox_ros_driver2')) tags.push('livox-driver', 'mid360');
  if (rel.includes('launch')) tags.push('ros-launch');
  if (rel.includes('msg/')) tags.push('ros-message');
  if (rel.includes('CMakeLists') || rel.includes('package.xml')) tags.push('build-system');
  return tags;
}

function fileNodeType(file) {
  if (file.fileCategory === 'docs') return 'document';
  if (file.fileCategory === 'config') return 'config';
  return 'file';
}

function complexityForLines(lines) {
  if (lines > 200) return 'complex';
  if (lines > 50) return 'moderate';
  return 'simple';
}

function countNames(items) {
  const counts = new Map();
  for (const item of items) counts.set(item.name, (counts.get(item.name) || 0) + 1);
  return counts;
}

function edge(source, target, type, weight) {
  return { source, target, type, direction: 'forward', weight };
}

function dedupeById(nodes) {
  return [...new Map(nodes.map((node) => [node.id, node])).values()];
}

function dedupeEdges(edges) {
  const map = new Map();
  for (const e of edges) map.set(`${e.source}\u0000${e.target}\u0000${e.type}`, e);
  return [...map.values()];
}

function uniq(values) {
  return [...new Set(values.filter(Boolean))];
}

function readJson(file) {
  return JSON.parse(fs.readFileSync(file, 'utf8'));
}

function writeJson(file, value) {
  fs.mkdirSync(path.dirname(file), { recursive: true });
  fs.writeFileSync(file, JSON.stringify(value, null, 2) + '\n');
}

function gitHead() {
  return process.env.GIT_COMMIT_HASH || '';
}
