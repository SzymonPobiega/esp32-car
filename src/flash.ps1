<#
.SYNOPSIS
Builds a sketch for the right vehicle and flashes it. The board is identified by its MAC,
looked up in boards.json, which gives the sketch, the vehicle and the board settings (FQBN).

.EXAMPLE
./flash.ps1 -Port COM7
Identifies the board on COM7, builds its sketch for its vehicle and flashes it.

.EXAMPLE
./flash.ps1 -Port COM7 -Sketch BmsConnector -Vehicle Volvo -Fqbn esp32:esp32:esp32
Flashes a board that is not in boards.json yet.

.EXAMPLE
./flash.ps1 -Sketch TpmsConnector -Vehicle Toyota -BuildOnly
Only builds, no board needed.

.EXAMPLE
./flash.ps1 -Ota -Sketch ObdConnector
Over the air: the connector must be in update mode (hold OBD on the dashboard's diagnostics
page) and the laptop on the maintenance Wi-Fi from Shared/Secrets.hpp. The board is found by
its host name, <vehicle>-<sketch>-<last 3 MAC bytes>.local, derived from boards.json.

.EXAMPLE
./flash.ps1 -Ota -Sketch HeaterConnector -HostAddress 192.168.137.42
Over the air to an IP address, when the .local name doesn't resolve.
#>
param(
    [string]$Port,
    [string]$Sketch,
    [ValidateSet('Toyota', 'Volvo')][string]$Vehicle,
    [string]$Fqbn,
    [switch]$BuildOnly,
    # Allow flashing a board with a vehicle other than the one boards.json assigns to it
    [switch]$Force,
    # Upload over Wi-Fi to a connector in maintenance mode instead of over USB
    [switch]$Ota,
    # With -Ota: the board's IP address, instead of resolving its .local name
    [string]$HostAddress
)

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot

$cli = Join-Path $env:LOCALAPPDATA 'Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
if (-not (Test-Path $cli)) {
    $cli = (Get-Command arduino-cli -ErrorAction SilentlyContinue)?.Source
}
if (-not $cli) { throw 'arduino-cli not found (install Arduino IDE 2 or arduino-cli)' }

$boards = (Get-Content (Join-Path $root 'boards.json') -Raw | ConvertFrom-Json -AsHashtable).boards

# ---- Identify the board ----
$board = $null
if ($Port) {
    $esptool = Get-ChildItem (Join-Path $env:LOCALAPPDATA 'Arduino15\packages\esp32\tools\esptool_py') -Filter esptool.exe -Recurse |
        Sort-Object FullName -Descending | Select-Object -First 1 -ExpandProperty FullName
    if (-not $esptool) { throw 'esptool not found (install the esp32 core in Arduino IDE)' }

    $out = & $esptool --port $Port read-mac 2>&1 | Out-String
    if ($LASTEXITCODE -ne 0) { throw "esptool could not read the board on ${Port}:`n$out" }

    $mac = [regex]::Match($out, 'MAC:\s+([0-9a-fA-F:]{17})').Groups[1].Value.ToLower()
    $board = $boards[$mac]
    if ($board) {
        Write-Host "Board ${mac}: $($board.name)"
    } else {
        Write-Host "Board $mac is not in boards.json"
    }
}

# ---- OTA: the board is the one in boards.json running this sketch for this vehicle ----
if ($Ota) {
    if ($Port) { throw '-Ota and -Port exclude each other' }
    if (-not $Sketch) { throw '-Ota needs -Sketch' }
    $found = @($boards.GetEnumerator() | Where-Object {
        $_.Value.sketch -eq $Sketch -and (-not $Vehicle -or $_.Value.vehicle -eq $Vehicle)
    })
    if ($found.Count -eq 0) { throw "No board in boards.json runs $Sketch$(if ($Vehicle) { " for the $Vehicle" })" }
    if ($found.Count -gt 1) { throw "Several boards run ${Sketch}: pass -Vehicle" }
    $mac = $found[0].Key
    $board = $found[0].Value
    Write-Host "Board ${mac}: $($board.name) (over the air)"
}

# ---- Resolve sketch, vehicle and board settings ----
if ($board) {
    if ($Vehicle -and $Vehicle -ne $board.vehicle -and -not $Force) {
        throw "This board belongs to the $($board.vehicle). Use -Force to flash it as $Vehicle."
    }
    if (-not $Sketch) { $Sketch = $board.sketch }
    if (-not $Vehicle) { $Vehicle = $board.vehicle }
    if (-not $Fqbn) { $Fqbn = $board.fqbn }
}

# Build-only or unknown board: borrow the settings of a known board running the same sketch
if (-not $Fqbn -and $Sketch) {
    $match = $boards.Values | Where-Object { $_.sketch -eq $Sketch -and $_.fqbn } | Select-Object -First 1
    if ($match) { $Fqbn = $match.fqbn }
}

if (-not $Sketch) { throw 'No sketch: pass -Sketch or fill in "sketch" for this board in boards.json' }
if (-not $Vehicle) { throw 'No vehicle: pass -Vehicle or add this board to boards.json' }
if (-not $Fqbn) { throw 'No board settings: pass -Fqbn or fill in "fqbn" for this board in boards.json' }

$sketchDir = Join-Path $root $Sketch
if (-not (Test-Path $sketchDir)) { throw "Sketch folder $sketchDir not found" }

# ---- Build ----
$define = "-DVEHICLE_$($Vehicle.ToUpper())"
$buildPath = Join-Path ([IO.Path]::GetTempPath()) "toyotatpms-build\$Sketch-$Vehicle"

Write-Host "Building $Sketch for the $Vehicle ($Fqbn)"
& $cli compile --fqbn $Fqbn --build-path $buildPath `
    --build-property "compiler.cpp.extra_flags=$define" `
    --build-property "compiler.c.extra_flags=$define" `
    $sketchDir
if ($LASTEXITCODE -ne 0) { throw 'Build failed' }

if ($BuildOnly) { return }

# ---- Flash over the air ----
if ($Ota) {
    $secrets = Join-Path $root 'Shared\Secrets.hpp'
    if (-not (Test-Path $secrets)) { throw 'Shared/Secrets.hpp not found (copy Secrets.example.hpp and fill it in)' }
    $password = [regex]::Match((Get-Content $secrets -Raw), 'OTA_PASSWORD\s*=\s*"([^"]*)"').Groups[1].Value
    if (-not $password) { throw 'OTA_PASSWORD is empty in Shared/Secrets.hpp' }

    $espota = Get-ChildItem (Join-Path $env:LOCALAPPDATA 'Arduino15\packages\esp32\hardware\esp32') -Filter espota.exe -Recurse |
        Sort-Object FullName -Descending | Select-Object -First 1 -ExpandProperty FullName
    if (-not $espota) { throw 'espota.exe not found (install the esp32 core in Arduino IDE)' }

    $target = $HostAddress
    if (-not $target) {
        $hostName = "$Vehicle-$Sketch-$($mac.Replace(':', '').Substring(6))".ToLower() + '.local'
        Write-Host "Looking up $hostName"
        try {
            $target = ([System.Net.Dns]::GetHostAddresses($hostName) |
                Where-Object AddressFamily -eq 'InterNetwork' | Select-Object -First 1).IPAddressToString
        } catch { }
        if (-not $target) {
            throw "$hostName not found. Is the connector in update mode and the laptop on the maintenance Wi-Fi? (or pass -HostAddress)"
        }
    }

    $bin = Join-Path $buildPath "$Sketch.ino.bin"
    Write-Host "Flashing $Sketch to $target over the air"
    & $espota -i $target -p 3232 -a $password -f $bin -r
    if ($LASTEXITCODE -ne 0) { throw 'OTA upload failed (the first time, allow espota through the Windows firewall: the board connects back to the laptop)' }
    Write-Host 'Done; the connector restarts into normal operation'
    return
}

if (-not $Port) { throw '-Port is required to flash (or pass -BuildOnly or -Ota)' }

# ---- Flash ----
Write-Host "Flashing $Sketch to $Port"
& $cli upload --fqbn $Fqbn --build-path $buildPath -p $Port $sketchDir
if ($LASTEXITCODE -ne 0) { throw 'Upload failed' }
