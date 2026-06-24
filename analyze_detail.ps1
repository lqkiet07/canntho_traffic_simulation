$fixedFile = "d:\CTU\trafficDigital\gama folder\nq\models\KPI_Result_FixedTime_High_1400.csv"
$paperFile = "d:\CTU\trafficDigital\gama folder\nq\models\KPI_Result_CBMP_Paper_High_1400.csv"

Write-Output "=== DETAILED ANALYSIS FOR HIGH (1400 VPH) ==="

function Analyze-File($filePath, $label) {
    Write-Output "--- $label ---"
    $data = Import-Csv -Path $filePath
    $groups = $data | Group-Object -Property Intersection_Name
    foreach ($g in $groups) {
        $total_tp = 0
        $total_q = 0
        $weighted_delay = 0
        foreach ($row in $g.Group) {
            $tp = [int]$row.Throughput_per_Cycle
            $q = [int]$row.Queue_Length
            $delay = [double]$row.Average_Delay
            $total_tp += $tp
            $total_q += $q
            $weighted_delay += ($delay * $tp)
        }
        $avg_q = [math]::Round(($total_q / $g.Group.Count), 2)
        $avg_delay = if ($total_tp -gt 0) { [math]::Round(($weighted_delay / $total_tp), 2) } else { 0 }
        Write-Output "  Node: $($g.Name) -> Throughput=$total_tp, AvgQueue=$avg_q, AvgDelay=$avg_delay s"
    }
}

Analyze-File -filePath $fixedFile -label "FIXED TIME"
Analyze-File -filePath $paperFile -label "CBMP PAPER"
