param(
    [string]$IsaacRoot = 'D:\isaacsim',
    [switch]$Gui
)
$ErrorActionPreference = 'Stop'
$Workspace = Split-Path (Split-Path $PSScriptRoot)
$PythonBat = Join-Path $IsaacRoot 'python.bat'
$Scene = Join-Path $Workspace 'research\scene_audit\code_as_room\scene.usda'
$Visual = Join-Path $Workspace 'research\scene_audit\code_as_room\robot_visual.usdc'
$Geometry = Join-Path $Workspace 'research\scene_audit\code_as_room_geometry\code_as_room_colliders.json'
$Output = Join-Path $Workspace 'modules\wheeled_navigation\artifacts\code_as_room_demo'
foreach ($Required in @($PythonBat, $Scene, $Geometry)) {
    if (-not (Test-Path -LiteralPath $Required)) { throw "Missing required local asset: $Required. See CODE_AS_ROOM_DEMO.zh.md." }
}
if (-not (Test-Path -LiteralPath $Visual)) {
    & $PythonBat (Join-Path $Workspace 'research\prepare_robot_visual.py')
    if ($LASTEXITCODE -ne 0) { throw 'Could not prepare visual-only robot asset.' }
}
$Runner = Join-Path $PSScriptRoot 'scripts\run_code_room_mapping.py'
if ($Gui) { & $PythonBat $Runner --gui } else { & $PythonBat $Runner }
if ($LASTEXITCODE -ne 0) { throw 'Isaac scene run failed.' }
$Metrics = Get-Content -LiteralPath (Join-Path $Output 'metrics.json') -Raw -Encoding UTF8 | ConvertFrom-Json
if ($Metrics.status -ne 'passed') { throw 'Isaac run did not produce a passed validation record.' }
& $PythonBat (Join-Path $Workspace 'modules\wheeled_navigation\scripts\validate_code_room_recording.py') $Output --geometry $Geometry
if ($LASTEXITCODE -ne 0) { throw 'Offline scan reconstruction failed.' }
Write-Host "Preview and report: $Output"
