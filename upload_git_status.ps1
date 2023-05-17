# ====
# Uploads a snapshot of the current git state to a remote ADB device.
# ====

# verify a compatible version of PowerShell is installed
if ($PSVersionTable.PSVersion.Major -lt 5) {
    throw "This script requires PowerShell 5.0 or later."
}

function New-TemporaryDirectory {
    $parent = [System.IO.Path]::GetTempPath()
    [string] $name = [System.Guid]::NewGuid()
    New-Item -ItemType Directory -Path $parent -Name $name -ErrorAction Stop
}

Write-Output "Preparing..."
$tempdir = New-TemporaryDirectory

$version = 1

Write-Output "Collecting information..."

# get the current git repo's name and other information
$reponame = (git rev-list --max-parents=0 HEAD)
$head_at = (git rev-parse HEAD)

if ($return_to -eq "HEAD") {
    $return_to = $head_at
}
# do we need to add files?
if ((git status --porcelain=v1 | Measure-Object).Count -gt 0)
{
    Write-Output "  * adding files..."
    git add .
    $diff = git --no-pager diff --cached --no-color
    $has_diff = $true
} else {
    $has_diff = $false
}

$owner = git config user.name
$now = Get-Date -Format "yyyy-MM-dd HH:mm:ss"

Write-Output "Staging information..."

# write current state to output directory...
$container = New-Item -Type Directory -Path $tempdir -Name "$($reponame)_state" -Force
$gitstatusfile = New-Item -Type File -Path $container -Name "git_status.txt" -Force  # empty file if already exists
Add-Content $gitstatusfile "UploadGitStatus`n[version] $version"
Add-Content $gitstatusfile "[commit] $head_at"
Add-Content $gitstatusfile "[deployed_by] $owner"
Add-Content $gitstatusfile "[deployed_at] $now"
if ($has_diff) {
    Add-Content $gitstatusfile "[staged] staged.patch"
    $stageditemsfile = New-Item -Type File -Path $container -Name "staged.patch" -Force
    Set-Content $stageditemsfile $diff
}

Write-Output "Deploying..."

# check if a device is connected
$device = (adb devices -l | Select-String -Pattern "device product:").Line
if ($null -eq $device) {
    throw "No device connected."
}

# push the directory to the device
adb push $container /sdcard/

Write-Output "Cleaning up..."
# cleanup
Remove-Item -Path $tempdir -Recurse -Force
if ($has_diff)
{
    Write-Output "All done: deployed commit $($head_at.Substring(0, 7)) + staged changes to connected device"
} else {
    Write-Output "All done: deployed commit $($head_at.Substring(0, 7)) to connected device"
}