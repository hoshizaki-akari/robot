param(
    [string]$Distro = 'Ubuntu-22.04-F',
    [string]$ExistingWslFile = '/home/zhj/projects/aubo_i5_traction_ws/aubo_i5_traction_workstation/config/commissioning.local.yaml'
)
# Offline resolver regression: dummy credentials, CheckLocalConfig only, no SDK/serial.
$ErrorActionPreference = 'Stop'
$testDirectory = Join-Path ([IO.Path]::GetTempPath()) ('aubo_config_test_' + [guid]::NewGuid())
New-Item -ItemType Directory -Path (Join-Path $testDirectory 'scripts') | Out-Null
New-Item -ItemType Directory -Path (Join-Path $testDirectory 'config') | Out-Null
$source = Join-Path (Split-Path $PSScriptRoot -Parent) 'scripts/start_live_workstation.ps1'
$launcher = Join-Path $testDirectory 'scripts/start_live_workstation.ps1'
Copy-Item -LiteralPath $source -Destination $launcher
$fixturePath = Join-Path $testDirectory 'connection.clixml'
$referencePath = Join-Path $testDirectory 'config/live_calibration.local.path'
$fixture = [pscustomobject]@{
    HostAddress = '127.0.0.1'
    Port = 1
    CalibrationFile = ''
    Credential = [pscredential]::new('offline-test', (ConvertTo-SecureString 'offline-only' -AsPlainText -Force))
}
$shellExecutable = (Get-Process -Id $PID).Path
try {
    # Reproduce the user's missing calibration field without using real secrets.
    $fixture | Export-Clixml -LiteralPath $fixturePath
    Set-Content -LiteralPath $referencePath -Value $ExistingWslFile -Encoding Ascii
    $result = & $shellExecutable -NoProfile -File $launcher -Distro $Distro -ConnectionFile $fixturePath -CheckLocalConfig
    if ($LASTEXITCODE -ne 0) { throw 'Project reference did not recover the empty Windows calibration field' }
    $data = $result | ConvertFrom-Json
    if ($data.CalibrationSource -ne 'project local reference' -or $data.CalibrationFile -ne $ExistingWslFile) {
        throw 'Project calibration selection was incorrect'
    }
    $result = & $shellExecutable -NoProfile -File $launcher -Distro $Distro -ConnectionFile $fixturePath -CalibrationFile $ExistingWslFile -CheckLocalConfig
    if ($LASTEXITCODE -ne 0 -or ($result | ConvertFrom-Json).CalibrationSource -ne 'command line') {
        throw 'Explicit calibration override was not preserved'
    }
    Remove-Item -LiteralPath $referencePath
    $previousPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        & $shellExecutable -NoProfile -File $launcher -Distro $Distro -ConnectionFile $fixturePath -CheckLocalConfig 2>$null | Out-Null
        $missingStatus = $LASTEXITCODE
    } finally { $ErrorActionPreference = $previousPreference }
    if ($missingStatus -eq 0) { throw 'Missing calibration was incorrectly accepted' }
    $fixture.CalibrationFile = $ExistingWslFile
    $fixture | Export-Clixml -LiteralPath $fixturePath
    $result = & $shellExecutable -NoProfile -File $launcher -Distro $Distro -ConnectionFile $fixturePath -CheckLocalConfig
    if ($LASTEXITCODE -ne 0 -or ($result | ConvertFrom-Json).CalibrationSource -ne 'Windows connection file') {
        throw 'Existing Windows calibration fallback was not preserved'
    }
    # Resolve a shared credential source even when Windows LocalAppData is a
    # different/missing profile view. This remains a protected dummy credential.
    $result = & $shellExecutable -NoProfile -File $launcher -Distro $Distro -SharedConnectionFile $fixturePath -CheckLocalConfig
    if ($LASTEXITCODE -ne 0 -or ($result | ConvertFrom-Json).ConnectionFile -ne $fixturePath) {
        throw 'Shared WSL connection source was not selected'
    }
    Write-Output 'PASS: project recovery, explicit override, missing-config failure, Windows fallback, shared credential source; no hardware opened.'
}
finally {
    $resolvedTest = [IO.Path]::GetFullPath($testDirectory)
    $resolvedTemp = [IO.Path]::GetFullPath([IO.Path]::GetTempPath())
    if ($resolvedTest.StartsWith($resolvedTemp, [StringComparison]::OrdinalIgnoreCase) -and
        (Split-Path $resolvedTest -Leaf) -like 'aubo_config_test_*') {
        Remove-Item -LiteralPath $resolvedTest -Recurse -Force
    }
}
