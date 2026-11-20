#!/usr/bin/env python3
"""Migrate the Gazebo series_legged terrain into portable MuJoCo resources.

Partition each connected solid by its source face planes, retain interior
convex cells, then merge neighbors whose union is convex. This preserves
the source slopes/steps instead of replacing the entire field by its hull.
"""
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull

from stl_to_hfield import read_stl

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE.parent / 'rm_control/rm_gazebo/worlds'
DESTINATION = PACKAGE / 'worlds/series_legged'
TOLERANCE = 1e-8


def components(triangles):
    _, indices = np.unique(np.round(triangles.reshape(-1, 3), 7), axis=0, return_inverse=True)
    parent = list(range(len(triangles)))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    owners = {}
    for i, face in enumerate(indices.reshape(-1, 3)):
        for a, b in zip(face, np.roll(face, -1)):
            edge = tuple(sorted((a, b)))
            if edge in owners:
                parent[root(i)] = root(owners[edge])
            else:
                owners[edge] = i
    groups = {}
    for i in range(len(triangles)):
        groups.setdefault(root(i), []).append(i)
    return [triangles[indices] for indices in groups.values()]


def compact(vertices):
    vertices = np.unique(vertices, axis=0)
    return vertices[ConvexHull(vertices).vertices]


def split(vertices, normal, offset):
    distance = vertices @ normal - offset
    if distance.max() < TOLERANCE or distance.min() > -TOLERANCE:
        return [vertices]
    edges = set()
    for face in ConvexHull(vertices).simplices:
        edges.update(tuple(sorted((a, b))) for a, b in zip(face, np.roll(face, -1)))
    intersections = []
    for a, b in sorted(edges):
        if distance[a] * distance[b] < 0:
            intersections.append(vertices[a] + (vertices[b] - vertices[a]) *
                                 distance[a] / (distance[a] - distance[b]))
    intersections = np.asarray(intersections).reshape(-1, 3)
    return [compact(np.vstack((vertices[distance <= 0], intersections))),
            compact(np.vstack((vertices[distance >= 0], intersections)))]


def inside(point, triangles):
    a, b, c = (triangles[:, i] - point for i in range(3))
    la, lb, lc = (np.linalg.norm(v, axis=1) for v in (a, b, c))
    dot = lambda x, y: np.einsum('ij,ij->i', x, y)
    solid_angle = 2 * np.arctan2(dot(a, np.cross(b, c)),
        la * lb * lc + dot(a, b) * lc + dot(b, c) * la + dot(c, a) * lb)
    return abs(solid_angle.sum()) > 2 * np.pi


def decompose(triangles):
    planes = []
    for a, b, c in triangles:
        normal = np.cross(b - a, c - a)
        if np.linalg.norm(normal) < 1e-12:
            continue
        normal /= np.linalg.norm(normal)
        offset = normal @ a
        if normal[np.argmax(abs(normal))] < 0:
            normal, offset = -normal, -offset
        if not any(np.max(abs(normal - n)) < 1e-6 and abs(offset - d) < 1e-6 for n, d in planes):
            planes.append((normal, offset))
    cells = [compact(triangles.reshape(-1, 3))]
    for normal, offset in planes:
        cells = [part for cell in cells for part in split(cell, normal, offset)]
    cells = [cell for cell in cells if inside(cell.mean(axis=0), triangles)]
    merged = True
    while merged:
        merged = False
        for i, a in enumerate(cells):
            volume_a = ConvexHull(a).volume
            for j in range(i + 1, len(cells)):
                b = cells[j]
                if np.any(np.minimum(a.max(0), b.max(0)) - np.maximum(a.min(0), b.min(0)) < -TOLERANCE):
                    continue
                vertices = np.vstack((a, b))
                hull = ConvexHull(vertices)
                volume = volume_a + ConvexHull(b).volume
                if abs(hull.volume - volume) < max(1e-10, 1e-7 * volume):
                    cells[i] = vertices[hull.vertices]
                    cells.pop(j)
                    merged = True
                    break
            if merged:
                break
    return cells


def numbers(values):
    return ' '.join(format(float(x), '.12g') for x in np.asarray(values).ravel())


def main():
    world = ET.parse(SOURCE / 'series_legged.world')
    link = world.find('.//model/link')
    pose = np.fromstring(link.findtext('pose'), sep=' ')
    assert np.all(pose[3:] == 0), 'This source terrain has translation only.'
    assert link.findtext('./collision/geometry/mesh/scale') == '1 1 1'
    source_stl = SOURCE / 'place/series_legged_world.stl'
    triangles = np.asarray(read_stl(source_stl))
    solids = components(triangles)
    parts = [cell for solid in solids for cell in decompose(solid)]
    source_volume = sum(abs(np.einsum('ij,ij->i', t[:, 0], np.cross(t[:, 1], t[:, 2])).sum() / 6)
                        for t in solids)
    collision_volume = sum(ConvexHull(cell).volume for cell in parts)
    assert abs(source_volume - collision_volume) < 1e-6

    (DESTINATION / 'meshes').mkdir(parents=True, exist_ok=True)
    (DESTINATION / 'source').mkdir(exist_ok=True)
    shutil.copyfile(source_stl, DESTINATION / 'meshes/series_legged_world.stl')
    shutil.copyfile(SOURCE / 'series_legged.world', DESTINATION / 'source/series_legged.world')
    vertices, faces = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    # Inline mesh data make the include independent of a robot's compiler.meshdir.
    lines = ['<mujoco model="series_legged_terrain">', '  <asset>',
             '    <mesh name="series_legged_world_visual_mesh" vertex="%s" face="%s"/>' %
             (numbers(vertices), ' '.join(str(i) for i in faces))]
    for i, cell in enumerate(parts):
        lines.append('    <mesh name="series_legged_world_convex_%02d" vertex="%s"/>' % (i, numbers(cell)))
    lines += ['  </asset>', '  <worldbody>',
              '    <body name="series_legged_world" pos="%s">' % numbers(pose[:3]),
              '      <geom name="series_legged_world_visual" type="mesh" mesh="series_legged_world_visual_mesh"'
              ' group="1" contype="0" conaffinity="0" rgba=".55 .64 .72 1" density="0"/>']
    for i in range(len(parts)):
        lines.append('      <geom name="series_legged_world_collision_%02d" type="mesh"'
                     ' mesh="series_legged_world_convex_%02d" group="3" contype="2" conaffinity="1"'
                     ' condim="3" friction="1 .005 .0001" solref=".004 1"'
                     ' solimp=".95 .99 .001" density="0" rgba=".2 .5 .8 .25"/>' % (i, i))
    lines += ['    </body>', '  </worldbody>', '</mujoco>']
    (DESTINATION / 'terrain.xml').write_text('\n'.join(lines) + '\n')
    manifest = dict(source_world='src/rm_control/rm_gazebo/worlds/series_legged.world',
        source_mesh='src/rm_control/rm_gazebo/worlds/place/series_legged_world.stl',
        translation_m=pose[:3].tolist(), scale=[1, 1, 1], triangles=len(triangles),
        connected_solids=len(solids), convex_collision_geoms=len(parts),
        world_bounds_m=[(triangles.reshape(-1, 3).min(0) + pose[:3]).tolist(),
                        (triangles.reshape(-1, 3).max(0) + pose[:3]).tolist()],
        source_volume_m3=source_volume, collision_volume_m3=collision_volume,
        algorithm='source face-plane partition, winding classification, convex neighbor merge',
        coplanar_plane_tolerance=1e-6)
    (DESTINATION / 'conversion.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print('Migrated %d source triangles into %d convex collision geoms; volume difference %.3g m^3.' %
          (len(triangles), len(parts), collision_volume - source_volume))


if __name__ == '__main__':
    main()
