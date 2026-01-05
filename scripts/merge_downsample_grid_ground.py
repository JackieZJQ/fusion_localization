#!/usr/bin/env python3
"""
merge_downsample_grid_ground.py

- 加载目录下所有 PCD/PLY 点云块，合并
- 栅格最低高度去地面（适合曲面/缓坡）
- 可选：在去地面后再做一轮平面去除精修
- 体素下采样
- 保存输出（PCD/PLY）

依赖：open3d (pip install open3d)
例子：

python3 ./scripts/merge_downsample_grid_ground.py \
  --input_dir=/home/jackie/robobus_localization/map_data \
  --grid=0.3 \
  --height_thresh=0.35 \
  --voxel=0.4 \
  --refine_plane \
  --plane_dist=0.06 \
  --output=/home/jackie/robobus_localization/merged_map_grid_refine.pcd

"""

import argparse
import glob
import os
import numpy as np
import open3d as o3d

def merge_pointclouds(input_dir):
    files = glob.glob(os.path.join(input_dir, "*.pcd")) + glob.glob(os.path.join(input_dir, "*.ply"))
    if not files:
        raise FileNotFoundError(f"No PCD/PLY files found in: {input_dir}")
    merged = o3d.geometry.PointCloud()
    for i, f in enumerate(sorted(files)):
        pc = o3d.io.read_point_cloud(f)
        if pc.is_empty():
            print(f"[WARN] Empty cloud: {f}")
            continue
        merged += pc
        if (i + 1) % 10 == 0:
            print(f"[INFO] Merged {i+1}/{len(files)} files, current points: {len(merged.points)}")
    print(f"[INFO] Total points before filtering: {len(merged.points)}")
    return merged

def grid_min_z_filter(cloud, grid_size=0.5, height_thresh=0.25):
    pts = np.asarray(cloud.points)
    if pts.shape[0] == 0:
        return cloud
    # 栅格索引
    gx = np.floor(pts[:, 0] / grid_size).astype(np.int64)
    gy = np.floor(pts[:, 1] / grid_size).astype(np.int64)
    keys = np.stack([gx, gy], axis=1)
    # 统计每格 min_z
    from collections import defaultdict
    minz = defaultdict(lambda: np.inf)
    for k, z in zip(map(tuple, keys), pts[:, 2]):
        if z < minz[k]:
            minz[k] = z
    # 标记保留点：高于 min_z + 阈值
    mask = []
    for k, z in zip(map(tuple, keys), pts[:, 2]):
        if z > minz[k] + height_thresh:
            mask.append(True)   # 非地面保留
        else:
            mask.append(False)  # 地面丢弃
    mask = np.array(mask, dtype=bool)
    kept = cloud.select_by_index(np.nonzero(mask)[0])
    print(f"[INFO] Grid filter keep {len(kept.points)} / {len(pts)} "
          f"(grid={grid_size} m, thresh={height_thresh} m)")
    return kept

def optional_plane_refine(cloud, dist_thresh=0.1, ransac_n=3, num_iter=200):
    if len(cloud.points) == 0:
        return cloud
    plane_model, inliers = cloud.segment_plane(
        distance_threshold=dist_thresh,
        ransac_n=ransac_n,
        num_iterations=num_iter
    )
    print(f"[INFO] Plane refine: model={plane_model}, remove inliers={len(inliers)}")
    return cloud.select_by_index(inliers, invert=True)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input_dir", required=True, help="Dir with PCD/PLY tiles")
    ap.add_argument("--output", required=True, help="Output .pcd or .ply")
    ap.add_argument("--grid", type=float, default=0.5, help="Grid size (m)")
    ap.add_argument("--height_thresh", type=float, default=0.25, help="Keep points higher than min_z+thresh")
    ap.add_argument("--voxel", type=float, default=0.3, help="Voxel size (m), 0 to skip")
    ap.add_argument("--refine_plane", action="store_true", help="Optional plane removal after grid filter")
    ap.add_argument("--plane_dist", type=float, default=0.1, help="Plane distance threshold (m)")
    args = ap.parse_args()

    cloud = merge_pointclouds(args.input_dir)
    cloud = grid_min_z_filter(cloud, grid_size=args.grid, height_thresh=args.height_thresh)

    if args.refine_plane:
        cloud = optional_plane_refine(cloud, dist_thresh=args.plane_dist)

    if args.voxel > 0:
        cloud = cloud.voxel_down_sample(voxel_size=args.voxel)
        print(f"[INFO] After voxel({args.voxel} m): {len(cloud.points)}")

    ok = o3d.io.write_point_cloud(args.output, cloud, write_ascii=False, compressed=True)
    if not ok:
        raise RuntimeError(f"Failed to write point cloud to {args.output}")
    print(f"[INFO] Saved to: {args.output}")

if __name__ == "__main__":
    main()