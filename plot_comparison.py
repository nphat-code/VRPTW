import matplotlib.pyplot as plt
import numpy as np

def plot_comparison():
    # --- DỮ LIỆU ĐẦU VÀO (Người dùng cập nhật tại đây) ---
    # Thời gian chạy thực tế (giây) cho từng quy mô
    # Dựa trên các lần chạy trước đó:
    # 25 khách hàng: ~1.5 giây
    # 50 khách hàng: ~120 giây (giới hạn thời gian)
    # 100 khách hàng: ~301 giây
    scenarios = ['25 Khách hàng', '50 Khách hàng', '100 Khách hàng']
    times = [1.5, 120, 301.56] 
    
    # --- VẼ BIỂU ĐỒ ---
    plt.figure(figsize=(10, 6))
    
    # Tạo màu sắc khác nhau cho các cột
    colors = ['#4CAF50', '#FFC107', '#F44336'] # Xanh, Vàng, Đỏ
    
    # Vẽ biểu đồ cột
    bars = plt.bar(scenarios, times, color=colors, width=0.6)
    
    # Thêm nhãn giá trị trên đầu mỗi cột
    for bar in bars:
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., height,
                 f'{height:.2f}s',
                 ha='center', va='bottom', fontsize=11, fontweight='bold')

    # Trang trí biểu đồ
    plt.title('So sánh Thời gian Giải thuật toán Branch and Cut (MIP)', fontsize=14, fontweight='bold', pad=20)
    plt.xlabel('Quy mô Bài toán', fontsize=12)
    plt.ylabel('Thời gian chạy (giây)', fontsize=12)
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    
    # Thêm chú thích về độ phức tạp
    plt.figtext(0.5, 0.01, "Lưu ý: Thời gian tăng theo hàm mũ khi số lượng khách hàng tăng", 
                ha="center", fontsize=10, style='italic', color='gray')

    plt.tight_layout()
    
    # Lưu và hiển thị
    output_file = 'solving_time_comparison.png'
    plt.savefig(output_file, dpi=300)
    print(f"Đã lưu biểu đồ so sánh tại: {output_file}")
    plt.show()

if __name__ == "__main__":
    plot_comparison()
