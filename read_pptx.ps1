Add-Type -AssemblyName System.IO.Compression.FileSystem

$path = "d:\CTU\trafficDigital\gama folder\nq\Báo cáo lần 2.pptx"
$outDir = "d:\CTU\trafficDigital\gama folder\nq\pptx_images"

if (-not (Test-Path $outDir)) { New-Item -ItemType Directory -Path $outDir | Out-Null }

$zip = [System.IO.Compression.ZipFile]::OpenRead($path)

foreach ($e in $zip.Entries) {
    if ($e.FullName -match '^ppt/media/') {
        $outPath = Join-Path $outDir $e.Name
        $stream = $e.Open()
        $fileStream = [System.IO.File]::Create($outPath)
        $stream.CopyTo($fileStream)
        $fileStream.Close()
        $stream.Close()
        Write-Host ("Extracted: " + $e.Name)
    }
}

$zip.Dispose()
Write-Host "Done. Images saved to: $outDir"
