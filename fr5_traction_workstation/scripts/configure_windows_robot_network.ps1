param(
    [string]$InterfaceGuid,
    [string]$Address = '192.168.58.10',
    [switch]$Apply,
    [switch]$Elevate,
    [string]$RestoreBackup
)

# Local helper for this computer's verified, dedicated Realtek Ethernet port.
# Default: read-only. This is not a general network backup/rollback utility.
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()
$resultPath = $null
trap {
    $message = $_ | Out-String
    if ($resultPath) {
        try {
            [pscustomobject]@{ Status = 'failed'; Error = $message; Backup = $backupPath } |
                ConvertTo-Json | Set-Content -LiteralPath $resultPath -Encoding UTF8
        } catch { Write-Warning "Could not save result log: $($_.Exception.Message)" }
    }
    Write-Error $message -ErrorAction Continue
    exit 1
}
$dedicatedGuid = [guid]'6364643c-8787-4077-83a6-9042b445f162'
if ($Apply -and $RestoreBackup) { throw 'Choose either Apply or RestoreBackup.' }
if ($RestoreBackup) {
    $saved = Get-Content -LiteralPath $RestoreBackup -Raw | ConvertFrom-Json
    $InterfaceGuid = $saved.InterfaceGuid
}
if (-not $InterfaceGuid) {
    Get-NetAdapter -Physical | Select-Object Name, InterfaceGuid, InterfaceDescription, Status
    throw 'Pass the verified dedicated Ethernet InterfaceGuid explicitly.'
}
$guid = [guid]$InterfaceGuid
if ($guid -ne $dedicatedGuid) { throw 'This helper is restricted to the verified dedicated Realtek Ethernet GUID.' }
$adapter = @(Get-NetAdapter | Where-Object { [guid]$_.InterfaceGuid -eq $guid })
if ($adapter.Count -ne 1 -or -not $adapter[0].HardwareInterface -or
    $adapter[0].InterfaceDescription -notmatch 'Realtek' -or
    $adapter[0].InterfaceDescription -match 'Wireless|Wi-Fi|WiFi|802\.11|Bluetooth') {
    throw 'The verified physical Realtek Ethernet adapter was not found.'
}
$adapter = $adapter[0]
$index = $adapter.ifIndex
$registrySubkey = "SYSTEM\CurrentControlSet\Services\Tcpip\Parameters\Interfaces\{$guid}"
$registryPath = "HKLM:\$registrySubkey"
$valueNames = @('EnableDHCP', 'IPAddress', 'SubnetMask', 'DefaultGateway', 'DefaultGatewayMetric', 'NameServer')
function Read-PersistentValues {
    $key = Get-Item -LiteralPath $registryPath
    foreach ($name in $valueNames) {
        $exists = $key.GetValueNames() -contains $name
        $entry = [ordered]@{ Name = $name; Exists = $exists; Kind = $null; Value = $null }
        if ($exists) {
            $entry.Kind = $key.GetValueKind($name).ToString()
            $entry.Value = $key.GetValue($name, $null, 'DoNotExpandEnvironmentNames')
        }
        [pscustomobject]$entry
    }
}
function Test-PersistentValues($Expected) {
    $actual = @(Read-PersistentValues)
    foreach ($entry in $Expected) {
        $found = @($actual | Where-Object Name -eq $entry.Name)
        if ($found.Count -ne 1 -or $found[0].Exists -ne $entry.Exists) { return $false }
        if ($entry.Exists -and ($found[0].Kind -ne $entry.Kind -or
            (ConvertTo-Json -InputObject @($found[0].Value) -Compress) -cne
            (ConvertTo-Json -InputObject @($entry.Value) -Compress))) { return $false }
    }
    return $true
}
function Write-PersistentValues($Values) {
    $key = [Microsoft.Win32.Registry]::LocalMachine.OpenSubKey($registrySubkey, $true)
    if (-not $key) { throw 'Cannot open the selected adapter registry key for writing.' }
    try {
        foreach ($entry in $Values) {
            if ($valueNames -notcontains $entry.Name) { throw 'Unexpected registry value in backup.' }
            if (-not $entry.Exists) { $key.DeleteValue($entry.Name, $false); continue }
            $value = $entry.Value
            switch ($entry.Kind) {
                'MultiString' { $value = [string[]]@($value) }
                'DWord' { $value = [int]$value }
                'QWord' { $value = [long]$value }
                'Binary' { $value = [byte[]]@($value) }
            }
            $key.SetValue($entry.Name, $value, [Microsoft.Win32.RegistryValueKind]$entry.Kind)
        }
    } finally { $key.Close() }
}
function Preparation-Result([bool]$Changed, [string]$Backup) {
    $liveAdapter = Get-NetAdapter | Where-Object { [guid]$_.InterfaceGuid -eq $guid }
    $liveInterface = Get-NetIPInterface -InterfaceIndex $index -AddressFamily IPv4
    $addresses = @(Get-NetIPAddress -InterfaceIndex $index -AddressFamily IPv4 -ErrorAction SilentlyContinue)
    $gateways = @(Get-NetRoute -InterfaceIndex $index -AddressFamily IPv4 -DestinationPrefix '0.0.0.0/0' -ErrorAction SilentlyContinue)
    $dns = @(Get-DnsClientServerAddress -InterfaceIndex $index -AddressFamily IPv4 | ForEach-Object ServerAddresses)
    $active = "$($liveAdapter.Status)" -eq 'Up' -and "$($liveInterface.Dhcp)" -eq 'Disabled' -and
        @($addresses | Where-Object { $_.IPAddress -eq $Address -and $_.PrefixLength -eq 24 }).Count -eq 1 -and
        @($addresses | Where-Object { $_.IPAddress -notlike '169.254.*' }).Count -eq 1 -and
        $gateways.Count -eq 0 -and $dns.Count -eq 0
    [pscustomobject]@{
        Status = $(if ($active) { 'configured_active' } elseif ("$($liveAdapter.Status)" -ne 'Up') { 'prepared_offline' } else { 'prepared_pending_activation' })
        InterfaceGuid = "$guid"; InterfaceAlias = $liveAdapter.Name; Address = $Address; PrefixLength = 24
        Changed = $Changed; Backup = $Backup; VerifiedPersistent = $true; VerifiedActive = $active
        AdapterStatus = "$($liveAdapter.Status)"; ActiveDhcp = "$($liveInterface.Dhcp)"
    }
}

if (-not $RestoreBackup -and ($Address -notmatch '^192\.168\.58\.(?:[1-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-4])$' -or $Address -eq '192.168.58.2')) {
    throw 'Use an available address in 192.168.58.0/24, excluding robot 192.168.58.2.'
}
$desired = @(
    [pscustomobject]@{ Name = 'EnableDHCP'; Exists = $true; Kind = 'DWord'; Value = 0 }
    [pscustomobject]@{ Name = 'IPAddress'; Exists = $true; Kind = 'MultiString'; Value = @($Address) }
    [pscustomobject]@{ Name = 'SubnetMask'; Exists = $true; Kind = 'MultiString'; Value = @('255.255.255.0') }
    [pscustomobject]@{ Name = 'DefaultGateway'; Exists = $true; Kind = 'MultiString'; Value = @() }
    [pscustomobject]@{ Name = 'DefaultGatewayMetric'; Exists = $true; Kind = 'MultiString'; Value = @() }
    [pscustomobject]@{ Name = 'NameServer'; Exists = $true; Kind = 'String'; Value = '' }
)
if (-not $Apply -and -not $RestoreBackup) {
    [pscustomobject]@{ Name = $adapter.Name; InterfaceGuid = "$guid"; Status = "$($adapter.Status)"
        IntendedAddress = "$Address/24"; PersistentValues = @(Read-PersistentValues) } | ConvertTo-Json -Depth 6
    exit 0
}
if ($Apply -and (Test-PersistentValues $desired)) {
    Preparation-Result $false $null | ConvertTo-Json
    exit 0 # Truly read-only when already prepared; preserve the original backup.
}
$isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole(
    [Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    if (-not $Elevate) { throw 'Administrator rights required. Use -Elevate to request Windows UAC.' }
    $arguments = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', ('"{0}"' -f $PSCommandPath))
    if ($RestoreBackup) { $arguments += @('-RestoreBackup', ('"{0}"' -f $RestoreBackup)) }
    else { $arguments += @('-InterfaceGuid', "$guid", '-Address', $Address, '-Apply') }
    $powershellPath = Join-Path $PSHOME 'powershell.exe'
    try { $child = Start-Process -FilePath $powershellPath -Verb RunAs -ArgumentList $arguments -PassThru -Wait }
    catch { throw "Windows UAC was declined or elevation failed: $($_.Exception.Message)" }
    exit $child.ExitCode
}
$stateDirectory = Join-Path $env:LOCALAPPDATA 'fr5-traction'
New-Item -ItemType Directory -Path $stateDirectory -Force | Out-Null
$resultPath = Join-Path $stateDirectory 'network_last_result.json'
if ($RestoreBackup) {
    if ($saved.SchemaVersion -eq 2) {
        $restore = @($saved.RegistryValues)
        if ($restore.Count -ne $valueNames.Count -or
            @($restore.Name | Sort-Object -Unique).Count -ne $valueNames.Count -or
            @($restore | Where-Object { $valueNames -notcontains $_.Name }).Count) { throw 'Incomplete or unexpected registry backup.' }
        $restoreScope = 'exact_saved_registry_values'
    } elseif ($saved.Dhcp -eq 'Enabled' -and @($saved.StaticAddresses).Count -eq 0 -and @($saved.DefaultRoutes).Count -eq 0) {
        # Compatibility with the original 20261026_161718_500 DHCP-only backup.
        # That format did not record original registry types or value existence.
        $restore = @(
            [pscustomobject]@{ Name = 'EnableDHCP'; Exists = $true; Kind = 'DWord'; Value = 1 }
            [pscustomobject]@{ Name = 'IPAddress'; Exists = $false }
            [pscustomobject]@{ Name = 'SubnetMask'; Exists = $false }
            [pscustomobject]@{ Name = 'DefaultGateway'; Exists = $true; Kind = 'MultiString'; Value = @() }
            [pscustomobject]@{ Name = 'DefaultGatewayMetric'; Exists = $true; Kind = 'MultiString'; Value = @() }
            [pscustomobject]@{ Name = 'NameServer'; Exists = $true; Kind = 'String'; Value = [string]$saved.StaticDns }
        )
        $restoreScope = 'legacy_dhcp_only'
    } else { throw 'Unsupported legacy backup; restore it manually without guessing routes or metrics.' }
    Write-PersistentValues $restore
    Restart-NetAdapter -Name $adapter.Name -Confirm:$false
    if (-not (Test-PersistentValues $restore)) { throw 'Restored persistent values failed verification.' }
    [pscustomobject]@{ Status = 'restored_persistent'; Backup = $RestoreBackup; InterfaceGuid = "$guid"
        RestoreScope = $restoreScope; VerifiedPersistent = $true; VerifiedActive = $false } |
        ConvertTo-Json | Set-Content -LiteralPath $resultPath -Encoding UTF8
} else {
    $ipv4 = @(Get-NetIPAddress -InterfaceIndex $index -AddressFamily IPv4 -ErrorAction SilentlyContinue)
    if (@($ipv4 | Where-Object { $_.IPAddress -notlike '169.254.*' -and $_.IPAddress -ne $Address }).Count) {
        throw 'The adapter has another IPv4 address; inspect it before replacing its dedicated configuration.'
    }
    $backupPath = Join-Path $stateDirectory ('network_backup_{0}.json' -f (Get-Date -Format 'yyyyMMdd_HHmmss_fff'))
    [pscustomobject]@{ SchemaVersion = 2; InterfaceGuid = "$guid"; InterfaceAlias = $adapter.Name
        RegistryValues = @(Read-PersistentValues) } | ConvertTo-Json -Depth 6 |
        Out-File -FilePath $backupPath -Encoding UTF8 -NoClobber
    # No WMI EnableStatic or interface/route metric changes; Wi-Fi is untouched.
    Write-PersistentValues $desired
    Restart-NetAdapter -Name $adapter.Name -Confirm:$false
    if (-not (Test-PersistentValues $desired)) { throw "Persistent verification failed. Backup: $backupPath" }
    Preparation-Result $true $backupPath | ConvertTo-Json |
        Set-Content -LiteralPath $resultPath -Encoding UTF8
}
Get-Content -LiteralPath $resultPath
