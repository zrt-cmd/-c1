Add-Type @"
using System;
using System.Text;
using System.Runtime.InteropServices;
public class Win {
    public delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);
    [DllImport("user32.dll")] public static extern bool EnumWindows(EnumWindowsProc lpEnumFunc, IntPtr lParam);
    [DllImport("user32.dll")] public static extern int GetWindowText(IntPtr hWnd, StringBuilder lpString, int nMaxCount);
    [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr hWnd);
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr hWnd);
    [DllImport("user32.dll")] public static extern bool MoveWindow(IntPtr hWnd, int X, int Y, int nWidth, int nHeight, bool bRepaint);
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr hWnd, out RECT lpRect);
    [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr hWnd, IntPtr hWndInsertAfter, int X, int Y, int cx, int cy, uint uFlags);
    [DllImport("user32.dll")] public static extern IntPtr GetAncestor(IntPtr hWnd, uint gaFlags);
    public struct RECT { public int Left, Top, Right, Bottom; }
    public static string ListTitles() {
        StringBuilder all = new StringBuilder();
        EnumWindows((h, l) => {
            StringBuilder sb = new StringBuilder(512);
            GetWindowText(h, sb, 512);
            string t = sb.ToString();
            if (t.Length > 0) {
                RECT r; GetWindowRect(h, out r);
                all.AppendLine(h + " | vis=" + IsWindowVisible(h) + " | " + r.Left + "," + r.Top + " " + (r.Right-r.Left) + "x" + (r.Bottom-r.Top) + " | " + t);
            }
            return true;
        }, IntPtr.Zero);
        return all.ToString();
    }
    public static IntPtr FindByTitle(string needle) {
        IntPtr result = IntPtr.Zero;
        EnumWindows((h, l) => {
            StringBuilder sb = new StringBuilder(512);
            GetWindowText(h, sb, 512);
            if (sb.ToString().IndexOf(needle, StringComparison.OrdinalIgnoreCase) >= 0) {
                result = h;
                return false;
            }
            return true;
        }, IntPtr.Zero);
        return result;
    }
}
"@

Write-Output "=== WINDOWS ==="
Write-Output ([Win]::ListTitles())

$h = [Win]::FindByTitle("Gazebo")
if ($h -eq [IntPtr]::Zero) {
    Write-Output "GAZEBO_WINDOW_NOT_FOUND"
} else {
    $root = [Win]::GetAncestor($h, 2)   # GA_ROOT
    Write-Output ("ROOT handle={0}" -f $root)
    [void][Win]::ShowWindow($root, 9)   # SW_RESTORE
    # HWND_TOPMOST(-1), move to primary monitor + size + show
    [void][Win]::SetWindowPos($root, [IntPtr](-1), 20, 20, 1200, 760, 0x0040)
    [void][Win]::SetForegroundWindow($root)
    $r = New-Object -TypeName 'Win+RECT'
    [void][Win]::GetWindowRect($h, [ref]$r)
    Write-Output ("MOVED handle={0} rect={1},{2} {3}x{4}" -f $h, $r.Left, $r.Top, ($r.Right-$r.Left), ($r.Bottom-$r.Top))
}
