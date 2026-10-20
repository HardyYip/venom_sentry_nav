param([Parameter(Mandatory=$true)][string]$ConfigTitle)
Add-Type @'
using System;
using System.Text;
using System.Runtime.InteropServices;
public static class ProjectRvizRestore {
  public delegate bool Callback(IntPtr h, IntPtr p);
  [StructLayout(LayoutKind.Sequential)] public struct Rect { public int Left,Top,Right,Bottom; }
  [DllImport("user32.dll")] public static extern bool EnumWindows(Callback c, IntPtr p);
  [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
  [DllImport("user32.dll")] public static extern bool IsIconic(IntPtr h);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int n);
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out Rect r);
  [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr h,IntPtr after,int x,int y,int cx,int cy,uint flags);
  [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
}
'@
[void][ProjectRvizRestore]::SetProcessDPIAware()
$TaskDeadline = [DateTime]::UtcNow.AddSeconds(30)
while ([DateTime]::UtcNow -lt $TaskDeadline) {
    $TaskMatches = [System.Collections.Generic.List[object]]::new()
    $TaskCallback = [ProjectRvizRestore+Callback]{
        param($TaskHandle, $TaskUnused)
        $TaskTitle = [System.Text.StringBuilder]::new(2048)
        [void][ProjectRvizRestore]::GetWindowText($TaskHandle, $TaskTitle, 2048)
        if ($TaskTitle.ToString().Contains($ConfigTitle) -and $TaskTitle.ToString() -match 'RViz') {
            $TaskMatches.Add([pscustomobject]@{ handle=$TaskHandle; title=$TaskTitle.ToString() })
        }
        return $true
    }
    [void][ProjectRvizRestore]::EnumWindows($TaskCallback, [IntPtr]::Zero)
    if ($TaskMatches.Count -gt 1) { Write-Error 'Ambiguous project RViz windows; no window was changed.'; exit 2 }
    if ($TaskMatches.Count -eq 1) {
        $TaskWindow = $TaskMatches[0]
        $TaskWasMinimized = [ProjectRvizRestore]::IsIconic($TaskWindow.handle)
        [void][ProjectRvizRestore]::ShowWindow($TaskWindow.handle, 9)
        Start-Sleep -Milliseconds 200
        $TaskRect = [ProjectRvizRestore+Rect]::new()
        [void][ProjectRvizRestore]::GetWindowRect($TaskWindow.handle, [ref]$TaskRect)
        Add-Type -AssemblyName System.Windows.Forms
        $TaskIntersects = $false
        foreach ($TaskScreen in [System.Windows.Forms.Screen]::AllScreens) {
            $TaskArea = $TaskScreen.WorkingArea
            if ($TaskRect.Right -gt $TaskArea.Left -and $TaskRect.Left -lt $TaskArea.Right -and $TaskRect.Bottom -gt $TaskArea.Top -and $TaskRect.Top -lt $TaskArea.Bottom) { $TaskIntersects = $true }
        }
        if (-not $TaskIntersects) {
            $TaskArea = [System.Windows.Forms.Screen]::PrimaryScreen.WorkingArea
            [void][ProjectRvizRestore]::SetWindowPos($TaskWindow.handle, [IntPtr]::Zero, $TaskArea.Left+40, $TaskArea.Top+40, [Math]::Min(1400,$TaskArea.Width-80), [Math]::Min(1000,$TaskArea.Height-80), 4)
        }
        $TaskFocused = [ProjectRvizRestore]::SetForegroundWindow($TaskWindow.handle)
        ConvertTo-Json -Compress @{ project_rviz_found=$true; was_minimized=$TaskWasMinimized; offscreen_relocated=(-not $TaskIntersects); foreground_requested=$TaskFocused; copy_mode=($TaskWindow.title -match 'COPY MODE') }
        exit 0
    }
    Start-Sleep -Milliseconds 300
}
Write-Error 'No exported project RViz window within 30s; inspect WSLg. Navigation was not restarted.'
exit 1
