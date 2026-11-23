# Read-only collection of current Drone project evidence plus full reachable Git refs.
# No drone/network commands are issued. Original project files are never modified.
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path -LiteralPath 'C:\python\Drone\Drone').Path.TrimEnd('\')
if (-not (Test-Path -LiteralPath (Join-Path $root 'hover'))) { throw "Unexpected Drone folder: $root" }
$out = Join-Path (Split-Path $root -Parent) ('DroneEvidence_' + (Get-Date -Format 'yyyyMMdd_HHmmss'))
New-Item -ItemType Directory -Path $out | Out-Null
$skipDir = '(?i)[\/](\.git|\.venv|venv|env|__pycache__|node_modules|\.pytest_cache|\.mypy_cache|dist|build)[\/]'
$skipSecret = '(?i)(^|[\/])(\.env(\.[^\/]*)?|credentials?(\.[^\/]*)?|secrets?(\.[^\/]*)?|id_rsa(\.[^\/]*)?|id_ed25519(\.[^\/]*)?|[^\/]+\.(pem|key|pfx|p12))$'
$textExt = @('.py','.pyi','.ps1','.psm1','.bat','.cmd','.sh','.yml','.yaml','.json','.toml','.ini','.cfg','.md','.txt','.csv','.tsv','.log','.ipynb','.xml','.html','.css','.js','.ts')
$binaryExt = @('.png','.jpg','.jpeg','.bmp','.webp','.tif','.tiff','.mp4','.avi','.mov','.mkv','.zip','.npy','.npz','.pkl','.db','.sqlite','.sqlite3')
$dataDirs = '(?i)[\/](captures|capture|data|datasets|results|recordings|logs|calibration|calibrations|images|frames|video|videos|diagnostics|evidence)[\/]'
$relevantName = '(?i)(flight|drift|drone|tello|aruco|marker|hover|vision|path|axis|streaming|motor|calibrat|junction|v[1-6])'
$files = @(Get-ChildItem -LiteralPath $root -Recurse -File -Force -ErrorAction SilentlyContinue | Where-Object {
    $ext = $_.Extension.ToLowerInvariant()
    $_.FullName -notmatch $skipDir -and $_.FullName -notmatch $skipSecret -and
    (($textExt -contains $ext) -or ($_.FullName -match $dataDirs) -or
     (($binaryExt -contains $ext) -and $_.Name -match $relevantName) -or
     $_.Name -in @('.gitignore','.gitattributes','Dockerfile'))
})
$inventory = foreach ($f in $files) {
    [pscustomobject]@{RelativePath=$f.FullName.Substring($root.Length+1); Bytes=$f.Length; ModifiedUTC=$f.LastWriteTimeUtc.ToString('o'); Type=$(if($binaryExt -contains $f.Extension.ToLowerInvariant()){'binary'}else{'text/data'})}
}
$inventoryPath = Join-Path $out 'file_inventory.csv'
$inventory | Export-Csv -LiteralPath $inventoryPath -NoTypeInformation -Encoding UTF8
$gitInfo = @()
if (Get-Command git -ErrorAction SilentlyContinue) {
    & git -C $root rev-parse --is-inside-work-tree 2>$null | Out-Null
    if ($LASTEXITCODE -eq 0) {
        $status = Join-Path $out 'git_status.txt'; & git -C $root status --short --branch | Out-File -LiteralPath $status -Encoding utf8; $gitInfo += $status
        $branches = Join-Path $out 'git_branches.txt'; & git -C $root branch -a -vv | Out-File -LiteralPath $branches -Encoding utf8; $gitInfo += $branches
        $log = Join-Path $out 'git_all_history_files.txt'; & git -C $root log --all --date=iso --pretty=format:'%h %ad %d %s' --name-status | Out-File -LiteralPath $log -Encoding utf8; $gitInfo += $log
        & git -C $root bundle create (Join-Path $out 'git_all_refs.bundle') --all | Out-Null
        if ($LASTEXITCODE -ne 0) { 'Git bundle failed; see Git metadata.' | Set-Content (Join-Path $out 'git_bundle_WARNING.txt') }
    }
}
Add-Type -AssemblyName System.IO.Compression
Add-Type -AssemblyName System.IO.Compression.FileSystem
$errorsFile = Join-Path $out 'archive_errors.txt'
function Write-EvidenceZip([string]$destination, [object[]]$items, [string[]]$extras) {
    $zip = [System.IO.Compression.ZipFile]::Open($destination,[System.IO.Compression.ZipArchiveMode]::Create)
    try {
        foreach ($f in $items) {
            $name = $f.FullName.Substring($root.Length+1).Replace('\','/')
            try { [System.IO.Compression.ZipFileExtensions]::CreateEntryFromFile($zip,$f.FullName,$name,[System.IO.Compression.CompressionLevel]::Optimal) | Out-Null }
            catch { Add-Content -LiteralPath $errorsFile -Value ("FAILED: $name :: $_") }
        }
        foreach ($p in $extras) {
            try { [System.IO.Compression.ZipFileExtensions]::CreateEntryFromFile($zip,$p,('metadata/'+[IO.Path]::GetFileName($p)),[System.IO.Compression.CompressionLevel]::Optimal) | Out-Null }
            catch { Add-Content -LiteralPath $errorsFile -Value ("FAILED metadata: $p :: $_") }
        }
    } finally { $zip.Dispose() }
}
$binary = @($files | Where-Object { $binaryExt -contains $_.Extension.ToLowerInvariant() })
$text = @($files | Where-Object { $binaryExt -notcontains $_.Extension.ToLowerInvariant() })
Write-EvidenceZip (Join-Path $out 'flight_images_archives.zip') $binary @()
Write-EvidenceZip (Join-Path $out 'code_csv_logs.zip') $text (@($inventoryPath) + $gitInfo)
Write-Host "DONE: $out"
Write-Host "SOURCE/CSV/LOG FILES: $($text.Count) | IMAGE/BINARY/OLD ZIP FILES: $($binary.Count)"
Get-ChildItem -LiteralPath $out -File | Select-Object Name,@{N='MB';E={[math]::Round($_.Length/1MB,1)}} | Format-Table -AutoSize
Write-Host 'No flight, motor, Wi-Fi or drone control was attempted.'

