param(
    [string]$HostAddress,
    [int]$Port = 8899,
    [Parameter(Mandatory = $true)][string]$CalibrationFile,
    [System.Management.Automation.PSCredential]$Credential,
    [string]$Distro = 'Ubuntu-22.04-F',
    [string]$ConnectionFile = (Join-Path ([Environment]::GetFolderPath('LocalApplicationData')) 'AuboTraction\connection.clixml')
)
$ErrorActionPreference = 'Stop'
if (-not $HostAddress) { $HostAddress = $env:AUBO_V4_HOST }
if (-not $HostAddress -or $Port -lt 1 -or $Port -gt 65535) { throw 'Specify the controller host and a valid TCP port.' }
if ($CalibrationFile -notmatch '^/[A-Za-z0-9_./-]+$') { throw 'CalibrationFile must be an absolute WSL path.' }
if (-not $Credential) {
    if ($env:AUBO_V4_USERNAME -and $env:AUBO_V4_PASSWORD) {
        $secret = ConvertTo-SecureString $env:AUBO_V4_PASSWORD -AsPlainText -Force
        $Credential = [pscredential]::new($env:AUBO_V4_USERNAME, $secret)
    } else {
        $Credential = Get-Credential -Message 'AUBO SDK controller login (saved for this Windows account only)'
    }
}
if (-not $Credential) { throw 'No SDK credential supplied.' }
$directory = Split-Path -Parent $ConnectionFile
New-Item -ItemType Directory -Path $directory -Force | Out-Null
# Windows Export-Clixml protects PSCredential's SecureString with DPAPI.
# The file stays outside Git and can only be decrypted by this user on this PC.
[pscustomobject]@{
    HostAddress = $HostAddress
    Port = $Port
    CalibrationFile = $CalibrationFile
    Credential = $Credential
} | Export-Clixml -LiteralPath $ConnectionFile
if ($CalibrationFile -notmatch '^(/home/[^/]+)/') { throw 'Calibration must be inside the WSL user home.' }
$sharedDirectory = $Matches[1] + '/.local/share/aubo-traction'
& wsl.exe -d $Distro --exec /usr/bin/install -d -m 700 $sharedDirectory
if ($LASTEXITCODE -ne 0) { throw 'Could not create the WSL user connection directory.' }
$sharedFile = "\\wsl.localhost\$Distro" + (($sharedDirectory + '/connection.clixml') -replace '/', '\')
Copy-Item -LiteralPath $ConnectionFile -Destination $sharedFile -Force
& wsl.exe -d $Distro --exec /bin/chmod 600 ($sharedDirectory + '/connection.clixml')
if ($LASTEXITCODE -ne 0) { throw 'Could not restrict the WSL user connection file permissions.' }
$localProject = Split-Path $PSScriptRoot -Parent
$calibrationReference = Join-Path $localProject 'config/live_calibration.local.path'
Set-Content -LiteralPath $calibrationReference -Value $CalibrationFile -Encoding Ascii
Write-Output 'Local AUBO connection saved. Start from WSL with: bash run_workstation.sh --live'
