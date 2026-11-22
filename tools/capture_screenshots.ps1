# Regenerates the README screenshots in docs/images/ by driving the GUI from the command line.
#
# Usage (PowerShell, from the repository root, after building the GUI):
#   .\tools\capture_screenshots.ps1                                   # uses build\local-qt-mingw
#   .\tools\capture_screenshots.ps1 -Exe path\to\arm-motion-planner.exe
#
# Each shot loads a scenario, plans (and optionally executes), waits, saves a PNG of the main
# window via --screenshot, and quits. Planning is deterministic, so the images are reproducible.
param(
    [string]$Exe = "build\local-qt-mingw\bin\arm-motion-planner.exe"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$out = Join-Path $root "docs\images"
New-Item -ItemType Directory -Force $out | Out-Null
$exePath = Resolve-Path (Join-Path $root $Exe)
$scenarios = Join-Path $root "scenarios"

$shots = @(
    @{ File = "narrow_passage_rrtstar.png"; Args = @("--scenario", "$scenarios\02_two_link_narrow_passage.json", "--plan", "rrtstar"); Delay = 5000 },
    @{ File = "narrow_passage_astar.png";   Args = @("--scenario", "$scenarios\02_two_link_narrow_passage.json", "--plan", "astar");   Delay = 3000 },
    @{ File = "execution_tracking.png";     Args = @("--scenario", "$scenarios\02_two_link_narrow_passage.json", "--plan", "rrt", "--execute", "--pause-at", "3.5"); Delay = 6500 },
    @{ File = "execution_feedback_only.png"; Args = @("--scenario", "$scenarios\02_two_link_narrow_passage.json", "--plan", "rrt", "--execute", "--pause-at", "3.5", "--no-feedforward"); Delay = 6500 },
    @{ File = "bone_cut_corridor.png";      Args = @("--scenario", "$scenarios\05_bone_cut_corridor.json", "--plan", "rrt", "--execute", "--pause-at", "6"); Delay = 9000 },
    @{ File = "redundant_5dof.png";         Args = @("--scenario", "$scenarios\06_redundant_reach_5dof.json", "--plan", "rrt"); Delay = 4000 }
)

foreach ($shot in $shots) {
    $target = Join-Path $out $shot.File
    $arguments = $shot.Args + @("--screenshot", $target, "--delay", $shot.Delay)
    Write-Host "Capturing $($shot.File)..."
    $p = Start-Process -FilePath $exePath -ArgumentList $arguments -PassThru -Wait
    if (-not (Test-Path $target)) {
        throw "Screenshot was not written: $target (exit code $($p.ExitCode))"
    }
}
Write-Host "Done. Images are in $out"
