[CmdletBinding()]
param(
    [string]$HostAlias = "192.168.0.2",
    [string]$RemoteHome = "/home/nvidia",
    [switch]$OpenCode
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$workspace = $PSScriptRoot
$sshTarget = $HostAlias
if ($RemoteHome -notmatch '^/home/[A-Za-z0-9._-]+$') {
    throw "RemoteHome must be a direct child of /home (for example /home/nvidia)."
}
$remoteRepo = "$RemoteHome/.deploy/game_26.git"
$remoteCurrent = "$RemoteHome/game_26_current"
$temporaryIndex = Join-Path ([System.IO.Path]::GetTempPath()) (
    "game_26_deploy_{0}.index" -f [guid]::NewGuid().ToString("N")
)

function Invoke-Checked {
    param(
        [Parameter(Mandatory = $true)]
        [scriptblock]$Command,
        [Parameter(Mandatory = $true)]
        [string]$FailureMessage
    )

    & $Command
    if ($LASTEXITCODE -ne 0) {
        throw $FailureMessage
    }
}

function Invoke-RemoteBash {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Script,
        [Parameter(Mandatory = $true)]
        [string]$FailureMessage
    )

    $encodedScript = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($Script))
    Invoke-Checked -FailureMessage $FailureMessage -Command {
        ssh -o BatchMode=yes $sshTarget "printf '%s' '$encodedScript' | base64 -d | bash"
    }
}

if (-not (Test-Path -LiteralPath (Join-Path $workspace ".git"))) {
    throw "Run this script from the game_26 Git repository root."
}

Write-Host "[1/5] Checking the Orin SSH connection..." -ForegroundColor Cyan
Invoke-Checked -FailureMessage "Could not connect to the Orin with an SSH key." -Command {
    ssh -o BatchMode=yes -o ConnectTimeout=8 $sshTarget "printf SSH_OK"
}

$head = (git -C $workspace rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or -not $head) {
    throw "Could not read the local Git HEAD."
}

$savedIndexFile = $env:GIT_INDEX_FILE
$savedAuthorName = $env:GIT_AUTHOR_NAME
$savedAuthorEmail = $env:GIT_AUTHOR_EMAIL
$savedCommitterName = $env:GIT_COMMITTER_NAME
$savedCommitterEmail = $env:GIT_COMMITTER_EMAIL

try {
    Write-Host "[2/5] Capturing the current worktree without changing the branch or index..." -ForegroundColor Cyan
    $env:GIT_INDEX_FILE = $temporaryIndex
    $env:GIT_AUTHOR_NAME = "RoboMaster Deploy"
    $env:GIT_AUTHOR_EMAIL = "deploy@local"
    $env:GIT_COMMITTER_NAME = "RoboMaster Deploy"
    $env:GIT_COMMITTER_EMAIL = "deploy@local"

    Invoke-Checked -FailureMessage "Could not create the temporary Git index." -Command {
        git -C $workspace read-tree $head
    }
    Invoke-Checked -FailureMessage "Could not collect the worktree files." -Command {
        git -C $workspace add -A
    }

    $tree = (git -C $workspace write-tree).Trim()
    if ($LASTEXITCODE -ne 0 -or -not $tree) {
        throw "Could not create the deployment tree."
    }

    $deployMessage = "deploy: {0}" -f (Get-Date -Format "yyyy-MM-dd HH:mm:ss")
    $commit = (git -C $workspace commit-tree $tree -p $head -m $deployMessage).Trim()
    if ($LASTEXITCODE -ne 0 -or -not $commit) {
        throw "Could not create the deployment snapshot."
    }
}
finally {
    $env:GIT_INDEX_FILE = $savedIndexFile
    $env:GIT_AUTHOR_NAME = $savedAuthorName
    $env:GIT_AUTHOR_EMAIL = $savedAuthorEmail
    $env:GIT_COMMITTER_NAME = $savedCommitterName
    $env:GIT_COMMITTER_EMAIL = $savedCommitterEmail

    if (Test-Path -LiteralPath $temporaryIndex) {
        Remove-Item -LiteralPath $temporaryIndex -Force
    }
}

Write-Host "[3/5] Initializing the deployment repository on the Orin..." -ForegroundColor Cyan
$initializeRemote = @'
set -eu
mkdir -p __REMOTE_HOME__/.deploy
if [ ! -d __REMOTE_HOME__/.deploy/game_26.git ]; then
    git init --bare __REMOTE_HOME__/.deploy/game_26.git
fi
if git --git-dir=__REMOTE_HOME__/.deploy/game_26.git show-ref --verify --quiet refs/heads/deploy; then
    git --git-dir=__REMOTE_HOME__/.deploy/game_26.git update-ref \
        refs/heads/previous refs/heads/deploy
fi
'@
$initializeRemote = $initializeRemote.Replace('__REMOTE_HOME__', $RemoteHome)
Invoke-RemoteBash -Script $initializeRemote `
    -FailureMessage "Could not initialize the Orin deployment repository."

Write-Host "[4/5] Uploading the snapshot incrementally..." -ForegroundColor Cyan
$deployRef = "{0}:refs/heads/deploy" -f $commit
$deployUrl = "{0}:{1}" -f $sshTarget, $remoteRepo
Invoke-Checked -FailureMessage "Could not upload the deployment snapshot." -Command {
    git -C $workspace push --force $deployUrl $deployRef
}

Write-Host "[5/5] Publishing the remote release..." -ForegroundColor Cyan
$publishRemote = @'
set -eu
repo=__REMOTE_HOME__/.deploy/game_26.git
current=__REMOTE_HOME__/game_26_current
marker="$current/.codex_deploy_managed"

# Older versions of this script used a symlink to immutable release folders.
# Removing the link does not remove its target, so the previous deployment
# remains recoverable under /home/nvidia/game_26_releases.
if [ -L "$current" ]; then
    unlink "$current"
fi

if [ -e "$current" ] && [ ! -d "$current" ]; then
    echo "Refusing to replace a non-directory path: $current" >&2
    exit 4
fi

if [ -d "$current" ] && [ ! -f "$marker" ]; then
    if find "$current" -mindepth 1 -maxdepth 1 -print -quit | grep -q .; then
        echo "Refusing to overwrite an unmanaged directory: $current" >&2
        exit 5
    fi
fi

mkdir -p "$current"
GIT_DIR="$repo" GIT_WORK_TREE="$current" \
    git read-tree --reset -u refs/heads/deploy
touch "$marker"

# A deployment snapshot created from a Windows worktree does not reliably
# preserve Unix executable bits. Restore them for scripts that are meant to be
# launched directly on the Orin.
chmod +x "$current/aim_on.sh" "$current/scripts/aim"
find "$current/scripts" -maxdepth 1 -type f \
    \( -name '*.sh' -o -name '*.py' \) -exec chmod +x {} +

tree=$(git --git-dir="$repo" rev-parse 'refs/heads/deploy^{tree}')
echo "REMOTE_CURRENT=$current"
echo "DEPLOY_TREE=$tree"
'@
$publishRemote = $publishRemote.Replace('__REMOTE_HOME__', $RemoteHome)
Invoke-RemoteBash -Script $publishRemote `
    -FailureMessage "Could not publish the Orin worktree."

Write-Host "Sync complete: $remoteCurrent" -ForegroundColor Green
Write-Host "Existing projects outside $remoteCurrent were not modified." -ForegroundColor DarkGray

if ($OpenCode) {
    $code = Get-Command code -ErrorAction SilentlyContinue
    if (-not $code) {
        Write-Warning "The VS Code CLI was not found; the remote folder was not opened."
    }
    else {
        & $code.Source --new-window --remote ("ssh-remote+{0}" -f $HostAlias) $remoteCurrent
    }
}
