# Script to append summary statistics to the bottom of GAMA simulation result CSV files.
# It checks if a summary already exists to prevent duplicate insertion.

$files = Get-ChildItem -Path "d:\CTU\trafficDigital\gama folder\nq\models\KPI_Result_*.csv"

foreach ($file in $files) {
    $lines = Get-Content -Path $file.FullName
    if ($lines.Count -le 2) {
        Write-Output "File $($file.Name) is empty or has no data rows. Skipping."
        continue
    }

    # Check if the file already contains a summary row to avoid duplicates
    $hasSummary = $false
    for ($i = [math]::Max(0, $lines.Count - 5); $i -lt $lines.Count; $i++) {
        if ($lines[$i] -match "Overall Result" -or $lines[$i] -match "Summary") {
            $hasSummary = $true
            break
        }
    }

    if ($hasSummary) {
        Write-Output "File $($file.Name) already contains summary data. Skipping."
        continue
    }

    $total_tp = 0
    $total_q = 0
    $weighted_delay = 0.0
    $valid_rows = 0

    foreach ($line in $lines) {
        $trimmed = $line.Trim().Replace("'", "").Replace('"', "")
        if ($trimmed -ne "" -and -not $trimmed.StartsWith("Intersection_Name")) {
            $parts = $trimmed.Split(",")
            if ($parts.Count -ge 6) {
                $tp_str = $parts[4].Trim()
                $q_str = $parts[3].Trim()
                $delay_str = $parts[5].Trim()

                if ($tp_str -match '^[0-9]+$' -and $q_str -match '^[0-9]+$') {
                    $tp = [int]$tp_str
                    $q = [int]$q_str
                    $delay = 0.0
                    if ($delay_str -match '^[0-9]+(\.[0-9]+)?$') {
                        $delay = [double]$delay_str
                    }

                    $total_tp += $tp
                    $total_q += $q
                    $weighted_delay += ($delay * $tp)
                    $valid_rows++
                }
            }
        }
    }

    if ($valid_rows -gt 0) {
        $avg_q = [math]::Round(($total_q / $valid_rows), 2)
        $avg_delay = if ($total_tp -gt 0) { [math]::Round(($weighted_delay / $total_tp), 2) } else { 0.0 }

        # Append two empty rows, a header row, and the summary data row
        $summaryHeader = ",,,,,"
        $labelsRow = "Summary,,,Avg Queue,Total Throughput,Weighted Avg Delay"
        $valuesRow = "Overall Result,,,Format: Average,Format: Sum,Format: Weighted Average"
        $dataRow = "Overall Result,,,$avg_q,$total_tp,$avg_delay"

        Add-Content -Path $file.FullName -Value ""
        Add-Content -Path $file.FullName -Value ""
        Add-Content -Path $file.FullName -Value $labelsRow
        Add-Content -Path $file.FullName -Value $valuesRow
        Add-Content -Path $file.FullName -Value $dataRow

        Write-Output "Successfully appended summary to $($file.Name): AvgQueue=$avg_q, TotalThroughput=$total_tp, AvgDelay=$avg_delay s"
    } else {
        Write-Output "No valid data rows found in $($file.Name)."
    }
}
