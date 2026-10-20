param(
    [string]$HostAddress,
    [int]$Port = 8899,
    [Parameter(Mandatory = $true)][string]$CalibrationFile,
    [System.Management.Automation.PSCredential]$Credential,
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
Write-Output 'Local AUBO connection saved. Start from WSL with: bash run_workstation.sh --live'
