# tools/colored_map — カメラ着色点群マップとエクスポート

カメラ画像を LiDAR 点群へ投影して実色の SLAM マップを作り、GIS / mesh /
LAS / CAD-BIM へ書き出すためのツール群。**3D Gaussian Splatting には依存しない**
(古典的な投影 + z バッファ遮蔽 + medoid 色決定。レンダも `--device cpu` の
numpy ラスタライザで CUDA / torch 不要)。歴史的経緯で
`tools/gaussian_splatting/` に同居していたが 2026-07 に分離した。旧パスは
互換 shim で import / 実行とも動き続ける。

## 主なエントリポイント

| ツール | 役割 |
|---|---|
| `colored_map_pipeline.py` | bag + TUM 軌跡 → posed images → 着色マップ → 品質ゲートまでの一括実行 |
| `extract_posed_images.py` | bag から姿勢付きカメラ画像 (`transforms.json`) を抽出 |
| `attach_dynamic_image_masks.py` | 外部の動的物体PNG maskを検証し、hash/coverage付きmanifestへ接続 |
| `build_lidar_init.py` | スキャン蓄積 + robust着色（overlap RGB balance / view confidence対応） |
| `recolor_pointcloud.py` | 既存PLYのXYZを保持してcamera画像から再着色し、coverage JSONを出力 |
| `render_map_flythrough.py` | 着色マップ動画（cinematic path / surface splat / 描画指標対応） |
| `colorize_from_bag.py` | SLAM なし・静的 extrinsic での単発着色 (マルチカメラ融合対応) |
| `map_export.py` / `las_export.py` / `mesh_export.py` | GIS delimited text / LAS 1.2 / 色付き mesh 出力 |
| `bim_export.py` / `bim_pipeline.py` | 平面抽出 → IfcSlab/IfcWall/IfcDoor/IfcWindow/IfcSpace (IFC4) |

品質評価は `scripts/evaluate_heldout_point_colors.py --image-margin`(忠実度)、
`scripts/evaluate_colored_map_appearance.py`(彩度保持・胡椒ノイズ・coverage)、
`scripts/check_colored_map_quality.py`(ゲート統合)の3点セットで行う。
RTK-SLAM construction_seq1 の較正済み report-only 閾値は
`configs/colored_map_quality_profiles/rtkslam_seq1_report_only.yaml` にある。
品質ゲートのレポート引数はプロファイルが参照する領域だけ指定すればよい。
`appearance_planar_roughness_*` 閾値を持つプロファイルでは平面限定roughnessも
パイプラインが自動計算する。
realtime nodeの出力確認には`scripts/evaluate_realtime_colored_map.py`を使い、
confirmed coverageとchromaをJSON保存できる。
CPU rendererの`--soft-edge-px 1`は不透明surfaceを変えず黒い隙間だけをfadeで
埋める。mesh exportは`--thin-voxel`でmulti-million-point入力を事前に間引ける。
RTK-SLAMで検証済みのビネット補正は `--color-image-margin 120
--color-vignette-gain-limit 2.5`。補正はgain limitが1のとき無効で、従来出力を
維持する。
K3構成ではさらに `--color-overlap-balance --color-view-confidence
--color-normal-voxel 0.12 --color-view-score-power 1` を使う。前者は同じ3D点を
見る画像間のRGB差から露出・white balanceを安定化し、後者はsurface normalの
入射角と投影解像度で観測を順位付けする。いずれもdefault-off。

地図geometry自体の動的障害物は、任意依存の
[`dynamic-object-removal`](https://github.com/rsasaki0109/dynamic-3d-object-removal)
0.5以降を導入し、`--dynamic-map-cleaner fusion`で除去できる。各LiDAR scanを
trajectoryでworld座標へ変換した点と同じ時刻のsensor originをcleanerへ渡すため、
単純な完成地図の点数削減ではない。大規模bagでは
`--dynamic-map-cleaner-evidence-stride N`で判定用scanを間引ける。除去点数、比率、
使用scan数、実装versionは`dynamic_map_cleaning.json`へ保存する。この機能も
default-offで、静的構造の保持と既存quality profileをpaired評価してから有効化する。

```bash
pip install 'dynamic-object-removal>=0.5'
python3 tools/colored_map/colored_map_pipeline.py BAG TRAJECTORY OUT \
  --dynamic-map-cleaner fusion --dynamic-map-cleaner-workers 4 \
  --dynamic-map-cleaner-evidence-stride 5
```

物体境界の色滲みを抑えるgeometry-aware fusionもdefault-offで利用できる。
`--color-geometry-aware`は1 pixel z-buffer近傍で、手前silhouetteの隣に投影された
背景点と、深度不連続の両側をRGB候補から除外する。外部segmentationのPNGを
`--dynamic-mask-dir`で接続し`--color-dynamic-exclusion`を指定すると動的領域も
除外する。`--refine-spatiotemporal-calibration`と
`--color-calibration-sigma-multiplier`を組み合わせると、較正の7DoF不確実性と
camera速度をpixel半径へ伝播し、各guardを観測ごとに拡張する。棄却数は
`fusion_diagnostics`としてmap/recolor reportへ残る。

```bash
python3 tools/colored_map/colored_map_pipeline.py BAG TRAJECTORY OUT \
  --extrinsic BODY_CAMERA.json --refine-spatiotemporal-calibration \
  --color-geometry-aware \
  --dynamic-mask-dir dynamic_masks --color-dynamic-exclusion \
  --color-dynamic-mask-margin-px 2 \
  --color-calibration-sigma-multiplier 1.0
```

maskは各posed imageと同じstemのPNGで、非zero pixelを除外領域とする。動的除外を
有効にする場合は全frameのmaskが必須。詳しい設計と安全条件は
[`colored-map-geometry-aware-fusion-2026-07.md`](../../docs/research/colored-map-geometry-aware-fusion-2026-07.md)
を参照。
silhouette/depth-edge marginはConstruction Seq1の全量候補が既存planar quality
gateを通らなかったため既定0。dataset固有のpaired A/Bと既存profileを通すまで
明示的に有効化しないこと。

edge-aware samplingは4 cornerの巨大な一時stackを作らず、同じcornerからRGBの
min/maxをin-place更新する。旧式との完全一致testに加え、Construction Seq1の
paired screenでPLY SHA-256とreportが一致し、wall timeを25.0%短縮した。詳細は
[`colored-map-fusion-performance-2026-07.md`](../../docs/research/colored-map-fusion-performance-2026-07.md)。

README動画の再現設定は `render_map_flythrough.py --device cpu
--soft-edge-px 1 --surface-splat --surface-aspect-limit 2.5
--surface-normal-voxel 0.12 --camera-preset cinematic --render-voxel 0.03
--render-workers 4 --metrics-out metrics.json`。legacy camera、円形splat、直列描画は
既定値のままで互換性を維持する。

## 引き継ぎ文書

- [`COLORING_HANDOFF.md`](COLORING_HANDOFF.md) — 着色品質枝 (2026-07-18)
- [`BIM_HANDOFF.md`](BIM_HANDOFF.md) — Scan-to-BIM 枝 (2026-07-11)
- [`colored-map-release-readiness-2026-07.md`](../../docs/research/colored-map-release-readiness-2026-07.md) — 公開判定と制約

チュートリアル: [`docs/3dgs-map-tutorial.md`](../../docs/3dgs-map-tutorial.md)
(フライスルー生成)、[`docs/workflows.md`](../../docs/workflows.md)。
