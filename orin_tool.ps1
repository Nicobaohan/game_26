[CmdletBinding()]
param(
    [ValidateSet('Sync', 'Deploy', 'StartSafe', 'StartOn', 'StartFire', 'Status', 'FetchCalibration')]
    [string]$Action = 'Deploy',
    [switch]$OpenCode
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
trap {
    Write-Host ("ERROR: {0}" -f $_.Exception.Message) -ForegroundColor Red
    exit 1
}

$workspace = $PSScriptRoot
$targetConfigPath = Join-Path $workspace 'orin_target.psd1'
if (-not (Test-Path -LiteralPath $targetConfigPath)) {
    throw "Orin target configuration not found: $targetConfigPath"
}

$targetConfig = Import-PowerShellDataFile -LiteralPath $targetConfigPath
$sshTarget = [string]$targetConfig.HostAlias
$remoteHome = [string]$targetConfig.RemoteHome
if ([string]::IsNullOrWhiteSpace($sshTarget)) {
    throw 'HostAlias is empty in orin_target.psd1.'
}
if ($remoteHome -notmatch '^/home/[A-Za-z0-9._-]+$') {
    throw 'RemoteHome must be a direct child of /home.'
}

$remoteProject = "$remoteHome/game_26_current"
$remoteAimScript = "$remoteProject/scripts/aim"

function Invoke-NativeChecked {
    param(
        [Parameter(Mandatory = $true)]
        [scriptblock]$Operation,
        [Parameter(Mandatory = $true)]
        [string]$FailureMessage
    )

    & $Operation
    if ($LASTEXITCODE -ne 0) {
        throw "$FailureMessage (exit code $LASTEXITCODE)"
    }
}

function Invoke-Sync {
    $syncScript = Join-Path $workspace 'sync_to_orin.ps1'
    & $syncScript `
        -HostAlias $sshTarget `
        -RemoteHome $remoteHome `
        -OpenCode:$OpenCode
}

function Invoke-RemoteAim {
    param([Parameter(Mandatory = $true)][string]$AimCommand)

    Invoke-NativeChecked -FailureMessage "Remote aim command failed: $AimCommand" -Operation {
        ssh -tt -o BatchMode=yes $sshTarget "bash '$remoteAimScript' '$AimCommand'"
    }
}

switch ($Action) {
    'Sync' {
        Invoke-Sync
    }
    'Deploy' {
        Invoke-Sync
        Invoke-RemoteAim -AimCommand 'build'
    }
    'StartSafe' {
        Invoke-RemoteAim -AimCommand 'safe'
    }
    'StartOn' {
        Invoke-RemoteAim -AimCommand 'on'
    }
    'StartFire' {
        Invoke-RemoteAim -AimCommand 'fire'
    }
    'Status' {
        Invoke-RemoteAim -AimCommand 'status'
    }
    'FetchCalibration' {
        $latestOutput = & ssh -o BatchMode=yes $sshTarget `
            "cat '$remoteHome/autoaim_calibration/latest_path.txt'"
        if ($LASTEXITCODE -ne 0) {
            throw 'No saved calibration was found. Run calibration and click SAVE first.'
        }
        $latestPath = ([string]$latestOutput).Trim()
        if ([string]::IsNullOrWhiteSpace($latestPath)) {
            throw 'The saved calibration path is empty.'
        }
        if ($latestPath -notmatch "^$([regex]::Escape($remoteHome))/autoaim_calibration/[0-9_]+/ost\.yaml$") {
            throw "Unexpected remote calibration path: $latestPath"
        }

        $destination = Join-Path $workspace `
            'src/ros2-hik-camera/config/camera_info_vehicle.yaml'
        $backupDirectory = Join-Path $workspace '.calibration_backups'
        New-Item -ItemType Directory -Path $backupDirectory -Force | Out-Null
        if (Test-Path -LiteralPath $destination) {
            $backupName = 'camera_info_vehicle_{0}.yaml' -f (Get-Date -Format 'yyyyMMdd_HHmmss')
            Copy-Item -LiteralPath $destination `
                -Destination (Join-Path $backupDirectory $backupName)
        }

        $temporaryFile = Join-Path ([System.IO.Path]::GetTempPath()) `
            ("camera_info_vehicle_{0}.yaml" -f [guid]::NewGuid().ToString('N'))
        try {
            $scpSource = "${sshTarget}:$latestPath"
            Invoke-NativeChecked -FailureMessage 'Could not download ost.yaml.' -Operation {
                scp -q $scpSource $temporaryFile
            }
            Move-Item -LiteralPath $temporaryFile -Destination $destination -Force
        }
        finally {
            if (Test-Path -LiteralPath $temporaryFile) {
                Remove-Item -LiteralPath $temporaryFile -Force
            }
        }

        Write-Host "Calibration installed locally: $destination" -ForegroundColor Green
        Write-Host 'Run deploy_to_orin.cmd once more to upload and build it.' -ForegroundColor Cyan
    }
}
