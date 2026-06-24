$files = Get-ChildItem -Path "d:\CTU\trafficDigital\gama folder\nq\models\KPI_Result_*.csv"
foreach ($file in $files) {
    $lines = Get-Content -Path $file.FullName
    if ($lines.Count -gt 2) {
        $total_tp = 0
        $total_q = 0
        $weighted_delay = 0
        $valid_rows = 0
        
        # Start from index 1 (or 2) and scan all lines. We skip header lines by checking column formats.
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
        $avg_q = if ($valid_rows -gt 0) { [math]::Round(($total_q / $valid_rows), 2) } else { 0 }
        $avg_delay = if ($total_tp -gt 0) { [math]::Round(($weighted_delay / $total_tp), 2) } else { 0 }
        Write-Output "$($file.Name): Throughput=$total_tp, AvgQueue=$avg_q, AvgDelay=$avg_delay s"
    }
}
