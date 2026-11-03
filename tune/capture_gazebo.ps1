param([Parameter(Mandatory=$true)][string]$OutputPath)

Add-Type -AssemblyName System.Drawing
Add-Type @'
using System.Runtime.InteropServices;
public class CaptureDpi {
    [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
}
'@
[void][CaptureDpi]::SetProcessDPIAware()
. "$PSScriptRoot\..\fix_gazebo_window.ps1" | Out-Null
$gazeboHandle = [Win]::FindByTitle("Gazebo")
if ($gazeboHandle -eq [IntPtr]::Zero) {
    throw "Gazebo window was not found"
}
Start-Sleep -Milliseconds 700
$rectangle = New-Object -TypeName 'Win+RECT'
[void][Win]::GetWindowRect($gazeboHandle, [ref]$rectangle)
$width = $rectangle.Right - $rectangle.Left
$height = $rectangle.Bottom - $rectangle.Top
$bitmap = New-Object System.Drawing.Bitmap($width, $height)
$graphics = [System.Drawing.Graphics]::FromImage($bitmap)
try {
    $graphics.CopyFromScreen($rectangle.Left, $rectangle.Top, 0, 0, $bitmap.Size)
    $bitmap.Save($OutputPath, [System.Drawing.Imaging.ImageFormat]::Png)
    Write-Output $OutputPath
} finally {
    $graphics.Dispose()
    $bitmap.Dispose()
}
