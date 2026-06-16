$files = Get-ChildItem -Path "d:\CTU\trafficDigital\gama folder\nq\models\KPI_Result_*.csv"
foreach ($file in $files) {
    $data = Import-Csv -Path $file.FullName
    if ($data.Count -gt 0) {
        $total_tp = 0
        $total_q = 0
        $weighted_delay = 0
        foreach ($row in $data) {
            $tp = [int]$row.Throughput_per_Cycle
            $q = [int]$row.Queue_Length
            $delay = [double]$row.Average_Delay
            
            $total_tp += $tp
            $total_q += $q
            $weighted_delay += ($delay * $tp)
        }
        $avg_q = [math]::Round(($total_q / $data.Count), 2)
        $avg_delay = if ($total_tp -gt 0) { [math]::Round(($weighted_delay / $total_tp), 2) } else { 0 }
        Write-Output "$($file.Name): Throughput=$total_tp, AvgQueue=$avg_q, AvgDelay=$avg_delay s"
    }
}
