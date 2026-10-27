param(
    [string]$IsaacRoot = 'D:\isaacsim',
    [switch]$Gui,
    [switch]$Hold,
    [switch]$Ros2,
    [switch]$ExternalSlam,
    [double[]]$Start = @(-5.5, -3.5),
    [double[]]$Goal = @(5.4, 3.5),
    [string]$Output = ''
)
$ErrorActionPreference = 'Stop'
if ($Start.Count -ne 2 -or $Goal.Count -ne 2) { throw 'Start and Goal each require x,y.' }
$python = Join-Path $IsaacRoot 'python.bat'
if (-not (Test-Path -LiteralPath $python)) { throw "Isaac Python not found: $python" }
$script = Join-Path $PSScriptRoot 'scripts\run_isaac_mapping.py'
$runArgs = @($script, '--start', $Start[0].ToString([cultureinfo]::InvariantCulture), $Start[1].ToString([cultureinfo]::InvariantCulture),
             '--goal', $Goal[0].ToString([cultureinfo]::InvariantCulture), $Goal[1].ToString([cultureinfo]::InvariantCulture))
if ($Gui) { $runArgs += '--gui' }
if ($Hold) { $runArgs += '--hold' }
if ($Ros2) { $runArgs += '--ros2' }
if ($ExternalSlam) { $runArgs += '--external-slam' }
if ($Output) { $runArgs += @('--output', $Output) }
& $python @runArgs
exit $LASTEXITCODE
