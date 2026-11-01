param(
    [Parameter(Mandatory=$true)][string]$Title,
    [Parameter(Mandatory=$true)][string]$Bash,
    [Parameter(Mandatory=$true)][string]$Out,
    [int]$WaitSeconds = 26,
    [int]$Width = 1500,
    [int]$Height = 860
)
$ErrorActionPreference = 'Stop'
$base = 'C:\Users\liyut\Documents\deepseek-harness\default-workspace\week2_color_target'

# Close any terminal windows left over from earlier attempts so the capture is unambiguous.
Get-Process WindowsTerminal -ErrorAction SilentlyContinue | ForEach-Object { Stop-Process -Id $_.Id -Force }
Start-Sleep -Seconds 3

$runner = Join-Path $base 'tools\shots\run_shot.ps1'
Start-Process -FilePath 'powershell.exe' -ArgumentList '-NoExit','-ExecutionPolicy','Bypass','-File',"`"$runner`"",'-Title',$Title,'-Bash',"`"$Bash`"" -WindowStyle Normal
Start-Sleep -Seconds $WaitSeconds

& (Join-Path $base 'tools\shots\capture_window.ps1') -TitleLike $Title -Out $Out -Width $Width -Height $Height
