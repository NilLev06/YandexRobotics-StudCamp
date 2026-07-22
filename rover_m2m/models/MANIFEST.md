# Local object detector models

Source folder: `https://drive.google.com/drive/folders/1Q_2Yfu6m8u0lwaOBki3LKjDMecIZuRAL`

The rover uses `yolo11n-ball-cube-int8.onnx` by default. Both models accept a
`1x3x512x512` FP32 tensor around an INT8-quantized graph and detect `ball`,
`cube`, and `robot-claw`.

| Local file | Google Drive ID | SHA-256 | Runtime output |
| --- | --- | --- | --- |
| `yolo11n-ball-cube-int8.onnx` | `1eK2ElBBXWdb1pRVYnRjsr1oKrTfesLk1` | `26c475c61e1a5b1547ba54b85fc60a234c66b160d930f96c81acdd4fccac2148` | `1x7x5376`, external NMS |
| `yolo26n-ball-cube-int8.onnx` | `1a1eu9YWknLWkuPtQAPNi67VrY6peEJ-g` | `0e8baf30da0f988a54dde2a8ea089112bb1e2cd2ccde5155a5368521961242ca` | `1x300x6`, end-to-end |

The supplied metadata declares Ultralytics AGPL-3.0 licensing. ONNX Runtime is
installed in `/src/vendor-python`; this path and the model directory survive
container recreation because `/src` is bind-mounted from the robot host.
