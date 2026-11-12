param(
    [Parameter(Mandatory=$true)][string]$TitleLike,
    [Parameter(Mandatory=$true)][string]$Out,
    [int]$Width = 1400,
    [int]$Height = 820
)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Text;
using System.Runtime.InteropServices;
public class ShotWin {
  public delegate bool EnumProc(IntPtr h, IntPtr l);
  [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left, Top, Right, Bottom; }
  [DllImport("user32.dll")] public static extern bool EnumWindows(EnumProc cb, IntPtr l);
  [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetWindowTextW(IntPtr h, StringBuilder s, int n);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
  [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr h, IntPtr hdc, uint flags);
  [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
  [DllImport("user32.dll")] public static extern bool MoveWindow(IntPtr h, int x, int y, int w, int ht, bool repaint);
  public static IntPtr Find(string needle) {
    IntPtr best = IntPtr.Zero;
    long bestArea = 0;
    EnumWindows((h, l) => {
      if (!IsWindowVisible(h)) return true;
      var t = new StringBuilder(512);
      GetWindowTextW(h, t, 512);
      if (t.ToString().IndexOf(needle, StringComparison.OrdinalIgnoreCase) >= 0) {
        RECT r;
        if (GetWindowRect(h, out r)) {
          long area = (long)(r.Right - r.Left) * (r.Bottom - r.Top);
          if (area > bestArea) { bestArea = area; best = h; }
        }
      }
      return true;
    }, IntPtr.Zero);
    return best;
  }
}
'@
$h = [ShotWin]::Find($TitleLike)
if ($h -eq [IntPtr]::Zero) { Write-Output 'WINDOW_NOT_FOUND'; exit 2 }
[ShotWin]::MoveWindow($h, 60, 60, $Width, $Height, $true) | Out-Null
Start-Sleep -Milliseconds 900
$r = New-Object ShotWin+RECT
[ShotWin]::GetWindowRect($h, [ref]$r) | Out-Null
$w = $r.Right - $r.Left
$ht = $r.Bottom - $r.Top
if ($w -le 0 -or $ht -le 0) { Write-Output 'BAD_RECT'; exit 3 }
$bmp = New-Object System.Drawing.Bitmap $w, $ht
$g = [System.Drawing.Graphics]::FromImage($bmp)
$hdc = $g.GetHdc()
$rendered = [ShotWin]::PrintWindow($h, $hdc, 2)
$g.ReleaseHdc($hdc)
$g.Dispose()
if (-not $rendered) { Write-Output 'PRINTWINDOW_FAILED'; exit 4 }
$dir = Split-Path -Parent $Out
if (-not (Test-Path -LiteralPath $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
$bmp.Save($Out, [System.Drawing.Imaging.ImageFormat]::Png)
$bmp.Dispose()
Write-Output "SAVED $Out ${w}x${ht}"
