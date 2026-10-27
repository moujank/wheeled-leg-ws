param([string]$IsaacRoot='D:\isaacsim', [switch]$Preview)
$ErrorActionPreference='Stop'
$Workspace=Split-Path (Split-Path $PSScriptRoot)
$PythonBat=Join-Path $IsaacRoot 'python.bat'
$Asset=Join-Path $Workspace 'research\scene_audit\code_as_room\wheeled_arm_v2_bent_visual.usdc'
if (-not (Test-Path -LiteralPath $PythonBat)) { throw "Isaac Python not found: $PythonBat" }
if (-not (Test-Path -LiteralPath $Asset)) {
    & $PythonBat (Join-Path $Workspace 'research\prepare_wheeled_arm_visual.py')
    if ($LASTEXITCODE -ne 0) { throw 'Robot pose preparation failed.' }
}
$Renderer=Join-Path $PSScriptRoot 'scripts\render_code_room_reveal.py'
if ($Preview) {
    & $PythonBat $Renderer --preview
    $Folder='code_as_room_video_preview'
} else {
    & $PythonBat $Renderer
    $Folder='code_as_room_video'
}
$Output=Join-Path $Workspace "modules\wheeled_navigation\artifacts\$Folder"
$Status=Get-Content -LiteralPath (Join-Path $Output 'render_status.json') -Raw -Encoding UTF8 | ConvertFrom-Json
if ($Status.status -ne 'passed') { throw 'Rendering failed. Inspect render_status.json.' }
& $PythonBat (Join-Path $Workspace 'modules\wheeled_navigation\scripts\validate_reveal_video.py') $Output
if ($LASTEXITCODE -ne 0) { throw 'Encoded video validation failed.' }
Write-Host "Video, GIF and preview page: $Output"
