param(
    [string]$PortName = 'COM4',
    [int]$TcpPort = 18755
)

$ErrorActionPreference = 'Stop'
$serial = [System.IO.Ports.SerialPort]::new(
    $PortName, 460800, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.ReadTimeout = 20
$serial.WriteTimeout = 200
$listener = [System.Net.Sockets.TcpListener]::new(
    [System.Net.IPAddress]::Loopback, $TcpPort)
$client = $null
$stream = $null
try {
    $serial.Open()
    $listener.Start()
    Write-Host "KWR75D bridge listening on 127.0.0.1:$TcpPort ($PortName, 460800 8N1)"
    Write-Host 'Waiting for one local WSL client. Ctrl+C stops the bridge.'
    $client = $listener.AcceptTcpClient()
    $client.NoDelay = $true
    $stream = $client.GetStream()
    $serialBuffer = New-Object byte[] 4096
    $networkBuffer = New-Object byte[] 256
    while ($client.Connected) {
        if ($client.Client.Poll(0, [System.Net.Sockets.SelectMode]::SelectRead) -and
            $client.Client.Available -eq 0) { break }
        $availableSerial = $serial.BytesToRead
        if ($availableSerial -gt 0) {
            $count = $serial.Read($serialBuffer, 0, [Math]::Min($availableSerial, $serialBuffer.Length))
            if ($count -gt 0) { $stream.Write($serialBuffer, 0, $count) }
        }
        if ($stream.DataAvailable) {
            $count = $stream.Read($networkBuffer, 0, $networkBuffer.Length)
            if ($count -eq 0) { break }
            $serial.Write($networkBuffer, 0, $count)
        }
        Start-Sleep -Milliseconds 1
    }
}
finally {
    if ($serial.IsOpen) {
        try { $serial.Write([byte[]](0x43, 0xAA, 0x0D, 0x0A), 0, 4) } catch { }
    }
    if ($stream) { $stream.Dispose() }
    if ($client) { $client.Dispose() }
    $listener.Stop()
    $serial.Dispose()
}
