# Training the SFP port detector

What the 2026-09-18 training run did, what it scored, and what those numbers
mean. The dataset it consumed is described in `docs/dataset.md`; read that
first, because most of the caveats here are really caveats about the data.

## The run

```bash
./run.sh train pretrained=yolo26s.pt epochs=100 name=sfp_yolo26s_p2
```

100 epochs in **43.8 minutes** (~26 s/epoch) on the RTX 5090. Everything not
named above came from `run.sh`'s defaults, which are tuned for ~17 px objects
and explained in CLAUDE.md:

| Setting | Value | Source |
|---|---|---|
| `model` | `yolo26s-p2.yaml` | default -- adds a stride-4 detection level |
| `imgsz` | 1152 | default -- native width |
| `batch` | -1 -> **5** | default -- AutoBatch |
| `scale` / `mosaic` | 0.25 / 0.4 | defaults -- stock 0.5/1.0 shrink tiny objects |
| `optimizer` | auto -> AdamW, lr=0.002 | ultralytics |
| `pretrained` | `yolo26s.pt` | passed explicitly |

Model: 329 layers, 9.66M parameters, 26.6 GFLOPs, `nc=80 -> nc=1`.
Dataset scan: 2207 train (172 backgrounds), 550 val (38 backgrounds), 0 corrupt.

### Two things the log says that look alarming and are not

**AutoBatch prints `CUDA out of memory` three times before settling.** That is
the probe searching upward, catching the failures, and landing on
`batch-size 5` at 19.22G/31.36G (61%). It is not a crash. It does show the
stock `batch=16` would have failed outright: a P2 head at 1152 carries a
288x288 stride-4 feature map, which is where the memory goes.

**`val/cls_loss` is `nan` in every row of `results.csv`.** mAP is computed
normally throughout, so this is a loss-reporting artifact of the single-class
NMS-free head, not divergence. If it were real, mAP would be zero.

### Pretrained transfer is partial, and that is structural

```
Transferred 360/902 items from pretrained weights
```

Only ~40% of tensors matched. The P2 config inserts a stride-4 branch that
renumbers layers, so the early backbone transfers while much of the neck and
head stay random. **A P2 model cannot be fully initialised from stock
`yolo26s.pt` weights** -- the architectures genuinely differ. Passing
`pretrained=` is still worth it, but do not read it as full COCO
initialisation, and do not compare it to the smoke tests, which loaded
`yolo26n.pt` whole into a matching architecture.

## Results

Best epoch **99**, saved to `runs/sfp_yolo26s_p2/weights/best.pt` (20.4 MB).

| Metric | Value |
|---|---|
| Precision | 0.997 |
| Recall | 0.913 |
| mAP50 | 0.919 |
| **mAP50-95** | **0.869** |

Inference: **3.0-3.8 ms per 1152x1152 image**, plus ~1 ms pre/post. That is
~260-330 FPS, so a live ROS 2 node would be bound by the camera, not the model.

Training was well-behaved: mAP50-95 rose 0.358 -> 0.680 -> 0.712 -> 0.825 ->
0.845 -> 0.866 -> 0.869 at epochs 1/5/10/40/60/90/99, and val box loss fell
monotonically 1.75 -> 0.52. No overfitting signature; the curve was still
creeping up at epoch 100, so more epochs would likely buy a little more.

### One number in the log is wrong

The final in-training validation prints **mAP50-95 = 0.828**. That figure is
not reproducible. Running `yolo detect val` against the same `best.pt` returns
**0.869**, matching what epoch 99 reported, and it returns 0.869 at both
`batch=5` and `batch=16`, so batch composition does not explain it. **Quote
0.869.** The 0.828 line appears to be an artifact of the end-of-training
validation pass and has not been explained further.

## What the score actually measures

**Val is drawn from the same 1000-pose run as train** -- same room, same card,
same lighting, same intrinsics, split by frame index with verified zero
leakage. So 0.919 mAP50 is an *in-distribution* number. It says the detector
learned this fixture under this randomization. It says nothing about a
different room, different lighting, or a real camera.

### Recall is capped by the two cameras nobody vets

Overall recall of 0.913 is not a small-object problem. Broken down:

| Camera | val boxes | recall @IoU0.5 |
|---|---|---|
| **center** | 368 | **0.997** |
| left | 329 | 0.888 |
| right | 324 | 0.843 |

The sampler's acceptance test -- frustum, facing angle, depth occlusion -- runs
against `center_camera` only. The side cameras ride along and are labelled by
projection regardless of whether the port is occluded by the gripper, edge-on,
or otherwise invisible. Those labels are geometrically correct and visually
unlearnable, and they are where essentially all of the missed recall lives.

**On the camera the pipeline actually validates, the detector is at 0.997.**

This also killed the obvious hypothesis. Recall barely varies with box size:

| GT box (px) | n | recall |
|---|---|---|
| 10-12 | 22 | 0.818 |
| 12-14 | 77 | 0.935 |
| 14-17 | 212 | 0.849 |
| 17-20 | 236 | 0.949 |
| 20-25 | 240 | 0.887 |
| 25-35 | 147 | 0.973 |
| 35+ | 87 | 0.943 |

Missed boxes have a median size of 17.4 px against 19.6 px for detected ones --
essentially the same. The single missed box on the centre camera was **116 px**,
a close-up with both ports plainly visible, which is the opposite of a
small-object failure. Mean IoU on hits is 0.94-0.96 across every bucket, so
when it fires, it localises tightly.

If you want the headline number to go up, fix the labels rather than the model:
extend the acceptance test to all three cameras, or export only
`center_camera`, or drop side-camera boxes that fail an occlusion check.

## Where the outputs are

`runs/sfp_yolo26s_p2/` (42 MB, gitignored):

| Path | What |
|---|---|
| `weights/best.pt` | **the model** -- epoch 99 |
| `weights/last.pt` | epoch 100; `resume=True` continues from it |
| `results.csv` | per-epoch metrics, the table above |
| `args.yaml` | every setting, including the defaults |
| `BoxPR_curve.png`, `results.png` | precision/recall and training curves |
| `val_batch*_pred.jpg` | predictions vs labels -- quickest sanity check |

See CLAUDE.md for how to run val / predict / export from `best.pt`. The one
thing that matters: **always pass `imgsz=1152`**, or a caller silently gets 640
and a 17 px port becomes 9 px.
