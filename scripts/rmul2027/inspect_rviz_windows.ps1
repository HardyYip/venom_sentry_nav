param([string]$CaptureDirectory, [switch]$Restore)
Add-Type @'
using System;
using System.Text;
using System.Runtime.InteropServices;
public static class RvizWindowProbe {
  [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
  public delegate bool Callback(IntPtr h, IntPtr p);
  [StructLayout(LayoutKind.Sequential)] public struct Rect { public int Left, Top, Right, Bottom; }
  [DllImport("user32.dll")] public static extern bool EnumWindows(Callback c, IntPtr p);
  [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
  [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
  [DllImport("user32.dll")] public static extern bool IsIconic(IntPtr h);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out Rect r);
  [DllImport("user32.dll")] public static extern uint GetWindowThreadProcessId(IntPtr h, out uint p);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int n);
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
  [DllImport("dwmapi.dll")] public static extern int DwmGetWindowAttribute(IntPtr h, int a, out int v, int n);
}
'@
[void][RvizWindowProbe]::SetProcessDPIAware()
$TaskWindows = [System.Collections.Generic.List[object]]::new()
$TaskCallback = [RvizWindowProbe+Callback]{
    param($TaskHandle, $TaskUnused)
    $TaskTitle = [System.Text.StringBuilder]::new(2048)
    [void][RvizWindowProbe]::GetWindowText($TaskHandle, $TaskTitle, $TaskTitle.Capacity)
    if ($TaskTitle.ToString() -match 'interactive-light.*RViz') {
        $TaskRect = [RvizWindowProbe+Rect]::new()
        [void][RvizWindowProbe]::GetWindowRect($TaskHandle, [ref]$TaskRect)
        $TaskWindowProcessId = [uint32]0
        [void][RvizWindowProbe]::GetWindowThreadProcessId($TaskHandle, [ref]$TaskWindowProcessId)
        $TaskCloaked = 0
        [void][RvizWindowProbe]::DwmGetWindowAttribute($TaskHandle, 14, [ref]$TaskCloaked, 4)
        $TaskWindows.Add([pscustomobject]@{
            handle = $TaskHandle.ToInt64(); title = $TaskTitle.ToString()
            process_id = $TaskWindowProcessId; visible = [RvizWindowProbe]::IsWindowVisible($TaskHandle)
            minimized = [RvizWindowProbe]::IsIconic($TaskHandle); cloaked = $TaskCloaked
            rect = @($TaskRect.Left, $TaskRect.Top, $TaskRect.Right, $TaskRect.Bottom)
        })
    }
    return $true
}
[void][RvizWindowProbe]::EnumWindows($TaskCallback, [IntPtr]::Zero)
if ($CaptureDirectory -and $TaskWindows.Count -eq 1) {
    [void][System.IO.Directory]::CreateDirectory($CaptureDirectory)
    $TaskWindow = $TaskWindows[0]
    $TaskHandle = [IntPtr]$TaskWindow.handle
    if ($Restore) { [void][RvizWindowProbe]::ShowWindow($TaskHandle, 9) }
    if (-not [RvizWindowProbe]::IsIconic($TaskHandle)) { [void][RvizWindowProbe]::SetForegroundWindow($TaskHandle) }
    Start-Sleep -Milliseconds 700
    $TaskRect = [RvizWindowProbe+Rect]::new()
    [void][RvizWindowProbe]::GetWindowRect($TaskHandle, [ref]$TaskRect)
    $TaskWidth = $TaskRect.Right - $TaskRect.Left
    $TaskHeight = $TaskRect.Bottom - $TaskRect.Top
    if ($TaskWidth -gt 0 -and $TaskHeight -gt 0 -and -not [RvizWindowProbe]::IsIconic($TaskHandle)) {
        Add-Type -AssemblyName System.Drawing
        $TaskBitmap = [System.Drawing.Bitmap]::new($TaskWidth, $TaskHeight)
        $TaskGraphics = [System.Drawing.Graphics]::FromImage($TaskBitmap)
        $TaskGraphics.CopyFromScreen($TaskRect.Left, $TaskRect.Top, 0, 0, [System.Drawing.Size]::new($TaskWidth, $TaskHeight))
        $TaskBitmap.Save((Join-Path $CaptureDirectory 'rviz-screen.png'), [System.Drawing.Imaging.ImageFormat]::Png)
        $TaskGraphics.Dispose()
        $TaskBitmap.Dispose()
    }
}
ConvertTo-Json -InputObject @($TaskWindows.ToArray()) -Depth 5
