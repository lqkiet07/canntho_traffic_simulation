"""
analyze_results.py
------------------
Đọc các file KPI_Summary_*.csv từ GAMA batch (n replicate),
tính mean ± std, so sánh 3 thuật toán: Fixed-Time, CBMP Paper (Đếm xe), và CBMP Area (Diện tích),
xuất bảng so sánh chi tiết ra file Excel + CSV.
"""

import os
import sys
import numpy as np
import pandas as pd
from scipy.stats import wilcoxon, ttest_rel

# Đảm bảo stdout sử dụng encoding utf-8 trên Windows
if sys.stdout.encoding != 'utf-8':
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass

# ── CẤU HÌNH ──────────────────────────────────────────────────────────────────
BASE_DIR   = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(BASE_DIR, "outputs")
RESULT_DIR = os.path.join(BASE_DIR, "outputs", "analysis")

# Số lượng replicate tối đa muốn quét (mặc định GAMA chạy bao nhiêu quét bấy nhiêu)
N_REPLICATES = 10

SCENARIOS = [
    ("Low_400",       "Low (400 vph)"),
    ("Medium_900",    "Medium (900 vph)"),
    ("High_1400",     "High (1400 vph)"),
]

METRICS = [
    ("Avg_Queue (Veh)",        "Avg Queue (xe)",  "giảm"),
    ("Total_Throughput (Veh)", "Throughput (xe)", "tăng"),
    ("Avg_Delay (s)",          "Avg Delay (s)",   "giảm"),
]

def list_available_files():
    if not os.path.exists(OUTPUT_DIR):
        print(f"\n❌ Thư mục outputs không tồn tại. Tạo mới tại: {OUTPUT_DIR}")
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        return []
    
    files = [f for f in os.listdir(OUTPUT_DIR) if f.startswith("KPI_Result_") and f.endswith(".csv")]
    if not files:
        print(f"\n❌ Không tìm thấy file KPI_Result_*.csv nào trong:\n   {OUTPUT_DIR}")
    else:
        print(f"\n📂 Tìm thấy {len(files)} file KPI_Result trong {OUTPUT_DIR}:")
        for f in sorted(files)[:5]:
            print(f"   {f}")
        if len(files) > 5:
            print(f"   ... và {len(files) - 5} file khác.")
    return files

def read_summary(filepath):
    try:
        # Đọc file KPI_Result trực tiếp và tự tổng hợp dữ liệu (vì GAMA không ghi file summary cho Fixed-time/Paper)
        df = pd.read_csv(filepath, skipinitialspace=True)
        
        # Dọn sạch dấu nháy đơn, nháy kép và khoảng trắng thừa trong tên cột
        df.columns = df.columns.str.replace("'", "").str.replace('"', "").str.strip()
        
        # Bỏ các dòng rác hoặc dòng trống nếu có
        df = df.dropna(subset=['Queue_Length', 'Throughput_per_Cycle', 'Average_Delay'])
        
        # Ép kiểu dữ liệu sang dạng số
        df['Queue_Length'] = pd.to_numeric(df['Queue_Length'], errors='coerce')
        df['Throughput_per_Cycle'] = pd.to_numeric(df['Throughput_per_Cycle'], errors='coerce')
        df['Average_Delay'] = pd.to_numeric(df['Average_Delay'], errors='coerce')
        df = df.dropna(subset=['Queue_Length', 'Throughput_per_Cycle', 'Average_Delay'])
        
        if df.empty:
            return None
            
        # 1. Tính hàng chờ trung bình toàn mạng:
        # Gom nhóm theo Cycle để tính tổng hàng chờ toàn mạng tại mỗi chu kỳ, sau đó lấy trung bình của các tổng này
        queue_per_cycle = df.groupby('Cycle')['Queue_Length'].sum()
        avg_q = queue_per_cycle.mean()
        
        # 2. Tổng Throughput của toàn bộ mô phỏng
        total_tp = df['Throughput_per_Cycle'].sum()
        
        # 3. Tính Delay trung bình có trọng số toàn mạng
        weighted_delay_sum = (df['Average_Delay'] * df['Throughput_per_Cycle']).sum()
        avg_delay = weighted_delay_sum / total_tp if total_tp > 0 else 0.0
        
        return {
            "Avg_Queue (Veh)": float(avg_q),
            "Total_Throughput (Veh)": float(total_tp),
            "Avg_Delay (s)": float(avg_delay)
        }
    except Exception as e:
        print(f"  ⚠️  Không đọc được {os.path.basename(filepath)}: {e}")
        return None

def collect_replicates(algo, scenario_key, available_files):
    data = {label: [] for _, label, _ in METRICS}
    found = []
    
    # Quét rep1..rep10
    for rep in range(1, N_REPLICATES + 1):
        fname = f"KPI_Result_{algo}_{scenario_key}_rep{rep}.csv"
        if fname in available_files:
            found.append(fname)

    # Fallback quét tất cả các file có pattern
    if len(found) < N_REPLICATES:
        pattern = f"KPI_Result_{algo}_{scenario_key}_rep"
        extras = sorted([f for f in available_files if f.startswith(pattern) and f not in found])
        found += extras
        found = found[:N_REPLICATES]

    if not found:
        # Thử tìm file không có đuôi rep (dành cho kịch bản chạy đơn lẻ)
        fname_single = f"KPI_Result_{algo}_{scenario_key}.csv"
        if fname_single in available_files:
            found.append(fname_single)

    if not found:
        return data

    print(f"  📄 {algo:12s} + {scenario_key:15s}: dùng {len(found)} file")

    for fname in found:
        fpath = os.path.join(OUTPUT_DIR, fname)
        vals = read_summary(fpath)
        if vals is None:
            continue
        for raw_col, label, _ in METRICS:
            matched = next((v for k, v in vals.items() if raw_col.lower() in k.lower()), None)
            if matched is not None:
                data[label].append(matched)

    return data

def run_stats(vals1, vals2, direction):
    n = min(len(vals1), len(vals2))
    if n < 1:
        return np.nan, np.nan, 0.0
    
    arr1 = np.array(vals1[:n])
    arr2 = np.array(vals2[:n])
    
    mean1 = arr1.mean()
    mean2 = arr2.mean()
    
    improvement = 0.0
    if mean1 != 0:
        improvement = ((mean1 - mean2) / mean1 * 100 if direction == "giảm" else (mean2 - mean1) / mean1 * 100)
        
    try:
        diff = arr2 - arr1
        if np.all(diff == 0) or n < 3:
            p_val = np.nan
        else:
            if n == 5:
                _, p_val = ttest_rel(arr1, arr2)
            else:
                _, p_val = wilcoxon(arr1, arr2, alternative="two-sided")
    except Exception:
        p_val = np.nan
        
    return mean2, arr2.std(ddof=1) if n > 1 else 0.0, improvement, p_val

def analyze():
    os.makedirs(RESULT_DIR, exist_ok=True)
    available_files = list_available_files()
    if not available_files:
        return

    rows = []

    print("\n" + "="*85)
    print("  PHÂN TÍCH SO SÁNH: CBMP Paper vs CBMP Area")
    print("="*85)

    for scenario_key, scenario_label in SCENARIOS:
        print(f"\n📊 Scenario: {scenario_label}")

        paper_data = collect_replicates("CBMP_Paper", scenario_key, available_files)
        area_data  = collect_replicates("CBMP_Area",  scenario_key, available_files)

        for raw_col, metric_label, direction in METRICS:
            paper_vals = paper_data.get(metric_label, [])
            area_vals  = area_data.get(metric_label, [])

            n_paper = len(paper_vals)
            n_area  = len(area_vals)
            
            if n_paper < 1 and n_area < 1:
                continue

            # 1. Tính Paper mean & std
            paper_mean, paper_std = np.nan, 0.0
            if n_paper > 0:
                paper_mean = np.array(paper_vals).mean()
                paper_std  = np.array(paper_vals).std(ddof=1) if n_paper > 1 else 0.0

            # 2. Tính Area mean & std
            area_mean, area_std = np.nan, 0.0
            if n_area > 0:
                area_mean = np.array(area_vals).mean()
                area_std  = np.array(area_vals).std(ddof=1) if n_area > 1 else 0.0

            # 3. So sánh Area vs Paper (So sánh trực tiếp 2 CBMP)
            imp_area_vs_paper = 0.0
            p_area_vs_paper = np.nan
            if n_paper > 0 and n_area > 0:
                _, _, imp_area_vs_paper, p_area_vs_paper = run_stats(paper_vals, area_vals, direction)

            # In ra màn hình log nhanh
            arrow = "↓" if direction == "giảm" else "↑"
            paper_str = f"Paper={paper_mean:.2f}" if not np.isnan(paper_mean) else "Paper=N/A"
            area_str  = f"Area={area_mean:.2f} ({arrow}{abs(imp_area_vs_paper):.1f}% vs Paper)" if not np.isnan(area_mean) and not np.isnan(paper_mean) else (f"Area={area_mean:.2f}" if not np.isnan(area_mean) else "Area=N/A")
            print(f"  {metric_label:25s} | {paper_str} | {area_str}")

            rows.append({
                "Scenario":                     scenario_label,
                "Chỉ số":                       metric_label,
                "CBMP Paper (mean ± std)":      f"{paper_mean:.2f} ± {paper_std:.2f}" if not np.isnan(paper_mean) else "N/A",
                "CBMP Area (mean ± std)":       f"{area_mean:.2f} ± {area_std:.2f}" if not np.isnan(area_mean) else "N/A",
                "Cải thiện Area vs Paper (%)":  round(imp_area_vs_paper, 1) if (not np.isnan(paper_mean) and not np.isnan(area_mean)) else "N/A",
                "p-value (Area vs Paper)":      round(p_area_vs_paper, 4) if not np.isnan(p_area_vs_paper) else "N/A",
                "Ý nghĩa (p<0.05)":             "Có ✓" if (not np.isnan(p_area_vs_paper) and p_area_vs_paper < 0.05) else "Không",
            })

    if not rows:
        print("\n❌ Không có dữ liệu để xuất.")
        return

    df_result = pd.DataFrame(rows)

    # Ghi file CSV
    csv_path = os.path.join(RESULT_DIR, "comparison_table.csv")
    df_result.to_csv(csv_path, index=False, encoding="utf-8-sig")
    print(f"\n✅ Đã lưu CSV: {csv_path}")

    # Ghi file Excel đẹp
    try:
        from openpyxl.styles import PatternFill, Font, Alignment
        excel_path = os.path.join(RESULT_DIR, "comparison_table.xlsx")
        
        with pd.ExcelWriter(excel_path, engine="openpyxl") as writer:
            df_result.to_excel(writer, index=False, sheet_name="So sánh Paper vs Area")
            ws = writer.sheets["So sánh Paper vs Area"]

            # Autofit cột
            for col in ws.columns:
                max_len = max(len(str(cell.value or "")) for cell in col) + 3
                ws.column_dimensions[col[0].column_letter].width = min(max_len, 30)

            # Thiết lập header
            header_fill = PatternFill("solid", fgColor="1E293B") # Dark blue grey
            for cell in ws[1]:
                cell.fill = header_fill
                cell.font = Font(color="FFFFFF", bold=True)
                cell.alignment = Alignment(horizontal="center", vertical="center")

            # Kẻ sọc xen kẽ theo Scenario
            scenario_order = [s[1] for s in SCENARIOS]
            colors = ["F1F5F9", "FFFFFF"]
            for row in ws.iter_rows(min_row=2):
                sv = str(row[0].value or "")
                idx = scenario_order.index(sv) % 2 if sv in scenario_order else 0
                fill = PatternFill("solid", fgColor=colors[idx])
                for cell in row:
                    cell.fill = fill
                    cell.alignment = Alignment(horizontal="center")

            # Tô màu xanh đỏ cho cột cải thiện Area vs Paper (Cột index 5, tức là cột E)
            for col_idx in [5]: # 1-based index tương ứng
                for row in range(2, ws.max_row + 1):
                    cell = ws.cell(row=row, column=col_idx)
                    try:
                        val = float(cell.value)
                        if val > 0:
                            cell.font = Font(color="15803D", bold=True) # Xanh lá đậm
                        elif val < 0:
                            cell.font = Font(color="B91C1C", bold=True) # Đỏ đậm
                    except (TypeError, ValueError):
                        pass

        print(f"✅ Đã lưu Excel: {excel_path}")
    except ImportError:
        print("⚠️ Chưa cài openpyxl. Hãy chạy: pip install openpyxl để xuất file Excel định dạng đẹp.")

    print("\n" + "="*85)
    print("  BẢNG KẾT QUẢ TỔNG HỢP SO SÁNH CBMP PAPER VS CBMP AREA")
    print("="*85)
    print(df_result.to_string(index=False))

if __name__ == "__main__":
    analyze()
