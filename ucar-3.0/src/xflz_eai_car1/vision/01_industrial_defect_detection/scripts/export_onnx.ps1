<# export_onnx.ps1 - exp01 RKNN-optimized ONNX export (pure ASCII) #>
$WS = Resolve-Path "$PSScriptRoot/../../.."
$FORK = "$WS/ultralytics_yolov8-main"
$WEIGHTS = if($args[0]){$args[0]}else{"E:\AIoT\AIoT_B2\runs\detect\train\weights\best.pt"}

Write-Host "DEBUG RAW WEIGHTS: $WEIGHTS"
# backslashes -> forward slashes for python
$WEIGHTS_PY = $WEIGHTS -replace '\\','/'
Write-Host "DEBUG PY WEIGHTS: $WEIGHTS_PY"

if(-not (Test-Path $WEIGHTS)){
    Write-Host "ERROR: Weight file not found $WEIGHTS"
    exit 1
}

$env:PYTHONPATH = "$FORK"

Write-Host "==> Export RKNN optimized ONNX (default format=rknn, output 3xnl raw tensors, DFL kept on CPU side)"
Write-Host "==> NOTE: fork exporter.py __main__ ignores command-line args (model=xxx is discarded)."
Write-Host "    DEFAULT_CFG.model defaults to yolov8n.pt, which triggers the download."
Write-Host "    Correct way: get_cfg(DEFAULT_CFG, overrides={model: <your .pt>}) (same as yolo CLI)."
Push-Location $FORK
python -c "import sys; from ultralytics.cfg import get_cfg, DEFAULT_CFG; from ultralytics.engine.exporter import export; sys.argv=['export']; export(get_cfg(DEFAULT_CFG, overrides={'model': r'$WEIGHTS_PY', 'format': 'rknn', 'imgsz': 640}))"
Pop-Location

Write-Host ""
Write-Host "==> If imgsz differs from training, edit 640 in overrides above (train default: 640)"

$onnxOutput = $WEIGHTS -replace '\.pt$','.onnx'
$onnxOutputPy = $onnxOutput -replace '\\','/'
Write-Host "==> Expected artifact: $onnxOutput"
Write-Host "==> Check onnx output count = 9 (detect, 3 layers x (box, cls, cls_sum)):"
python -c "import onnx; m=onnx.load(r'$onnxOutputPy'); print('outputs =', len(m.graph.output))"
