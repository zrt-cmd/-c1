<# train.ps1 #>
$WS = Resolve-Path "$PSScriptRoot/../../.."
$FORK = "$WS/ultralytics_yolov8-main"
$DATA = "$WS/NEU-DET.v1i.yolov8/data.yaml"
$DEVICE = if($args[0]){$args[0]}else{"0"}

Write-Host "==> Install RKNN optimized ultralytics fork"
Push-Location $FORK
pip install -e . -q -i https://pypi.tuna.tsinghua.edu.cn/simple
Pop-Location

Write-Host "==> Start train yolov8n imgsz=640"
yolo train `
  data="$DATA" `
  model=yolov8n.pt `
  epochs=100 imgsz=640 batch=8 workers=4`
  device="$DEVICE" `
  project="$WS/runs/train_neu_course" `
  name=yolov8n_img640_e100

Write-Host "==> Output artifact: $WS/runs/train_neu_course/yolov8n_img640_e100/weights/best.pt"