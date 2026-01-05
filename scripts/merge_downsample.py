#!/usr/bin/env python3
"""
merge_and_downsample.py

加载目录中的所有点云块（PCD/PLY），合并后做体素下采样，保存输出。
依赖：open3d (pip install open3d)
"""

import argparse
import glob
import os
import open3d as o3d

def merge_and_downsample(input_dir: str,
                         voxel_size: float,
                         output_path: str):
    files = []
    files.extend(glob.glob(os.path.join(input_dir, "*.pcd")))
    files.extend(glob.glob(os.path.join(input_dir, "*.ply")))
    if not files:
        raise FileNotFoundError(f"No PCD/PLY files found in: {input_dir}")

    print(f"[INFO] Found {len(files)} point cloud blocks")
    merged = o3d.geometry.PointCloud()

    for i, f in enumerate(sorted(files)):
        pc = o3d.io.read_point_cloud(f)
        if pc.is_empty():
            print(f"[WARN] Empty cloud: {f}")
            continue
        merged += pc
        if (i + 1) % 10 == 0:
            print(f"[INFO] Merged {i+1}/{len(files)} files, current points: {len(merged.points)}")

    print(f"[INFO] Total points before downsample: {len(merged.points)}")
    if voxel_size > 0:
        merged = merged.voxel_down_sample(voxel_size=voxel_size)
        print(f"[INFO] Points after voxel({voxel_size} m): {len(merged.points)}")

    # 自动根据扩展名保存
    ok = o3d.io.write_point_cloud(output_path, merged, write_ascii=False, compressed=True)
    if not ok:
        raise RuntimeError(f"Failed to write point cloud to {output_path}")
    print(f"[INFO] Saved merged cloud to: {output_path}")

def main():
    parser = argparse.ArgumentParser(description="Merge and downsample tiled point clouds.")
    parser.add_argument("--input_dir", required=True, help="Directory containing PCD/PLY tiles.")
    parser.add_argument("--voxel", type=float, default=0.2, help="Voxel size in meters (0 to skip).")
    parser.add_argument("--output", required=True, help="Output file path (.pcd or .ply).")
    args = parser.parse_args()

    merge_and_downsample(args.input_dir, args.voxel, args.output)

if __name__ == "__main__":
    main()