import matplotlib.pyplot as plt
import numpy as np

# --- 1. DỮ LIỆU THỰC NGHIỆM CỦA BẠN ---
# Thay thế các con số dưới đây bằng kết quả thực tế từ máy của bạn
data_results = {
    'Quy mô': ['25 KH', '50 KH', '100 KH'],
    'Thời gian (s)': [0.21, 2.98, 20.25],
    'Quãng đường (km)': [191.81, 362.25, 828.94],
    'Số lượng xe': [3, 5, 10]
}

def draw_performance_charts():
    labels = data_results['Quy mô']
    times = data_results['Thời gian (s)']
    distances = data_results['Quãng đường (km)']
    
    x = np.arange(len(labels))
    width = 0.35

    # --- BIỂU ĐỒ 1: THỜI GIAN GIẢI (BAR CHART) ---
    fig, ax1 = plt.subplots(figsize=(10, 6))
    
    color_time = 'tab:red'
    ax1.set_xlabel('Quy mô bài toán (Số lượng khách hàng)')
    ax1.set_ylabel('Thời gian giải (giây)', color=color_time, fontsize=12, fontweight='bold')
    bars = ax1.bar(x, times, width, color=color_time, alpha=0.7, label='Thời gian (s)')
    ax1.tick_params(axis='y', labelcolor=color_time)
    
    # Thêm số liệu trên đầu cột
    for bar in bars:
        height = bar.get_height()
        ax1.annotate(f'{height}s',
                    xy=(bar.get_x() + bar.get_width() / 2, height),
                    xytext=(0, 3), textcoords="offset points",
                    ha='center', va='bottom', fontweight='bold')

    plt.xticks(x, labels)
    plt.grid(axis='y', linestyle='--', alpha=0.6)
    
    plt.tight_layout()
    plt.savefig('route_images/chart_time_complexity.png', dpi=300)
    print("Đã lưu biểu đồ thời gian tại: route_images/chart_time_complexity.png")

    # --- BIỂU ĐỒ 2: TỔNG QUÃNG ĐƯỜNG (LINE CHART) ---
    plt.figure(figsize=(10, 6))
    plt.plot(labels, distances, marker='o', linestyle='-', color='tab:blue', linewidth=3, markersize=10)
    
    for i, txt in enumerate(distances):
        plt.annotate(f'{txt}km', (labels[i], distances[i]), xytext=(0, 10), 
                     textcoords='offset points', ha='center', fontweight='bold', color='tab:blue')
    plt.xlabel('Quy mô bài toán')
    plt.ylabel('Quãng đường (km)', fontsize=12, fontweight='bold')
    plt.grid(True, linestyle=':', alpha=0.7)
    
    plt.tight_layout()
    plt.savefig('route_images/chart_distance.png', dpi=300)
    print("Đã lưu biểu đồ quãng đường tại: route_images/chart_distance.png")
    
    plt.show()

if __name__ == "__main__":
    import os
    if not os.path.exists('route_images'): os.makedirs('route_images')
    draw_performance_charts()