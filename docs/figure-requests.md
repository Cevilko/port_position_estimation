# Figure and data requests for the pipeline machine

Written 2026-09-20 for whoever (human or Claude) works on the machine that runs
`port_position_estimation`. It is self-contained: you do not need the thesis
repo or the conversation that produced this list to act on it.

The thesis draft is complete and compiles. Eighteen image files and four data
exports are all that is outstanding. Every one of them currently prints as a
framed placeholder carrying the file name, so **the file names below are not
suggestions — they are what the document already asks for.** A different name
means the placeholder stays.

Deliver everything into a single flat directory and hand it back; it gets copied
into `figures/` (images) and `figures/data/` (CSV) on the thesis side.

---

## Rule zero: one run, not a fresh one

The thesis already states these numbers in tables and prose:

| Quantity | Value in the text |
|---|---|
| Ports localised | 1809 of 1982 (91%) |
| Three-view fits / two-view fits | 1244 / 565 |
| Position error, median / p90 / max | 0.43 / 1.47 / 40.7 mm |
| Mahalanobis distance, median | 0.18 |
| Reprojection residual, median | 0.11 px |

Every export must come from **the same run that produced those numbers** —
the 991-frame series behind `runs/sfp_yolo26s_p2/`. Re-running the pipeline and
exporting fresh numbers will silently disagree with tables that are already
written, and the disagreement will not be obvious in a plot.

If the original run is gone and you must re-run: say so explicitly in your
handback, and list the new values for all the rows above, so the tables get
corrected rather than quietly contradicted.

---

## Tier A — turnkey, no new code

### A1. Four training artefacts: copy and rename

They already exist in `runs/sfp_yolo26s_p2/`. Only the names change.

| Source | Deliver as |
|---|---|
| `results.png` | `training-curves.png` |
| `BoxPR_curve.png` | `pr-curve.png` |
| `confusion_matrix.png` | `confusion-matrix.png` |
| one of `val_batch*_pred.jpg` | `val-predictions.jpg` |

For `val-predictions.jpg`, prefer a batch that contains at least one side-camera
frame — the thesis uses it to discuss per-camera recall, not just to show hits.

### A2. `labels-raw-vs-projected.png`

The `--annotate` flag already does this (`scripts/project_port_bboxes.py:228`):

```
./run.sh bbox rosbag_samples/<bag>/frames/<NNNNN> --annotate annotated.jpg
```

Deliver the raw `<camera>_image.jpg` and the annotated copy composed side by
side into one image. Pick a centre-camera frame where both ports are visible and
one is turned slightly away — the caption claims the box is tight on the
face-on port and slightly generous on the turned one, so the picture has to show
both cases.

### A3. `box-sizes.csv`

Derivable from the exported YOLO labels alone; nothing needs to run. Labels are
`class cx cy w h`, normalised, at 1152x1024:

```
find <dataset>/labels -name '*.txt' | xargs awk '{printf "%.3f,%.3f\n", $4*1152, $5*1024}' \
  | sed '1i width_px,height_px' > box-sizes.csv
```

Expect 5074 rows plus the header. If the count differs, the dataset is not the
one the thesis describes — flag it rather than shipping it.

### A4. `results.csv`

Copy `runs/sfp_yolo26s_p2/results.csv` as-is. No processing.

---

## Tier B — screenshots and frame selection, human judgement

These cannot be scripted; they need someone to look. Native resolution, no
upscaling, no JPEG artefacts on anything with text in it.

| File | What it must show |
|---|---|
| `scene-overview.png` | Isaac Sim viewport: UR5e, the three cameras on the gripper, the NIC card with two SFP ports on the task board. The three cameras must be individually distinguishable — this is the reader's first sight of the rig. |
| `dataset-montage.png` | Six thumbnails spanning the randomisation range, arm pose and fixture yaw. Pick the extremes, not six similar frames; the figure exists to prove diversity. |
| `three-cameras-one-instant.png` | One instant from all three cameras, left/centre/right, labelled. Should make it visually obvious why the side views are the harder ones. |
| `rviz-detections.png` | RViz with the three annotated camera streams side by side, system running live. Detections visible in all three panes. |
| `side-camera-failure.png` | A side-camera label that is geometrically correct but visually unlearnable — the port occluded by the gripper while the projected box sits over it. This single frame carries the thesis's main explanation for the 0.84–0.89 side-camera recall, so it must be unambiguous. |
| `port-identity-swap.png` | A fixture yaw where positional port ids stop matching true identity. Ideally two frames before/after the swap, with ids drawn. |
| `outlier-case.png` | The 40.7 mm worst case and the geometry that caused it. **Blocked on C1** — you cannot pick this frame until the per-sample CSV tells you which one it is. Do this last. |

---

## Tier C — needs new code on your side

This is the part that is not a matter of running an existing command. The
quantities all exist in memory at some point, but nothing in the repo writes any
of them to disk — there is no CSV path anywhere in `ros_ws/` or `scripts/`.

### C1. `position-errors.csv`

Wanted schema, one row per scored port:

```
stamp,error_m,n_views,distance_m,mahalanobis
```

What stands in the way, precisely:

- `port_error_node.py:85-92` receives `PoseWithCovarianceStamped` and keeps only
  `position`. The covariance arrives and is discarded. Mahalanobis needs it, so
  this callback has to retain the 3x3 block.
- `errors_seen` (`:133`) accumulates scalars, and `_report()` (`:140`) reduces
  them to a log line. The per-sample values are never written out.
- `n_views` is not in the message at all. The triangulator knows it — it comes
  from `port["cameras"]` (`port_triangulator_node.py:181`) — but
  `_to_message()` does not carry it. Either publish it or log it with the stamp
  and join on the stamp afterwards.
- `distance_m` is port-to-camera distance. Define it as distance from the
  **centre** camera's optical origin to the triangulated point, and say so in
  your handback if you choose otherwise — the plot's caption depends on it.

The thesis reports median Mahalanobis 0.18 against a calibrated ~1.5. If your
computed column lands far from 0.18, stop and report it rather than shipping —
that number is load-bearing in two chapters.

### C2. `reprojection-residuals.csv`

```
stamp,rms_error_px,n_views
```

`rms_error` is computed inside `match_and_triangulate` and reaches the node —
`self.last_errors` at `port_triangulator_node.py:186` — and then goes nowhere.
It is never published and never persisted. Expose it.

Median should come out at 0.11 px.

---

## Where the plots go

Do not render the Tier C data as images. Deliver CSV only. The thesis draws
these six in `pgfplots` from the CSVs, so the fonts match the body text and the
result stays vector:

`box-size-histogram`, `error-cdf`, `error-by-views`, `error-by-range`,
`mahalanobis-histogram`, `reprojection-residuals`.

A PNG of a matplotlib chart cannot be used for any of them.

---

## Complete delivery checklist

Images (12 files, closing 12 of the 18 placeholders):

```
scene-overview.png              labels-raw-vs-projected.png
dataset-montage.png             three-cameras-one-instant.png
rviz-detections.png             side-camera-failure.png
port-identity-swap.png          outlier-case.png
training-curves.png             pr-curve.png
confusion-matrix.png            val-predictions.jpg
```

That is 12 image files; the remaining 6 placeholders are the pgfplots figures
above, which need no image from you — only the CSVs.

Data (4), into `figures/data/`:

```
box-sizes.csv    position-errors.csv    reprojection-residuals.csv    results.csv
```

Suggested order: A1, A3, A4 (minutes) -> A2 and Tier B except `outlier-case`
-> C1, C2 -> `outlier-case.png` once C1 identifies the frame.
