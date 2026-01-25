from mip import Model, xsum, BINARY, MINIMIZE
import math

def solve_vrptw_mip(data_rows, sys_capacity):
    cnt = len(data_rows)
    # Khởi tạo mô hình với bộ giải CBC (mã nguồn mở)
    model = Model(solver_name="CBC")

    # --- 1. Biến quyết định (Chương 3.2) ---
    # x[i][j] = 1 nếu xe đi từ i đến j
    x = [[model.add_var(var_type=BINARY, name=f"x_{i}_{j}") for j in range(cnt)] for i in range(cnt)]
    # t[i] là thời gian xe đến khách hàng i (Time Window)
    t = [model.add_var(name=f"t_{i}") for i in range(cnt)]
    # u[i] là tải trọng lũy kế trên xe tại điểm i (Capacity)
    u = [model.add_var(name=f"u_{i}") for i in range(cnt)]

    # --- 2. Hàm mục tiêu (Chương 2.2) ---
    def dist(i, j):
        return math.sqrt((data_rows[i]['x'] - data_rows[j]['x'])**2 + 
                         (data_rows[i]['y'] - data_rows[j]['y'])**2)

    model.objective = xsum(dist(i, j) * x[i][j] for i in range(cnt) for j in range(cnt) if i != j)
    model.sense = MINIMIZE

    # --- 3. Các ràng buộc cơ bản ---
    # Mỗi khách hàng i phải có đúng 1 luồng vào và 1 luồng ra
    for i in range(1, cnt):
        model.add_constr(xsum(x[i][j] for j in range(cnt) if i != j) == 1)
        model.add_constr(xsum(x[j][i] for j in range(cnt) if i != j) == 1)

    # Ràng buộc tại Depot (Nút 0): Số xe rời kho = Số xe về kho
    # Giới hạn số xe tối đa (ví dụ: 25 xe)
    model.add_constr(xsum(x[0][j] for j in range(1, cnt)) <= 25)
    model.add_constr(xsum(x[0][j] for j in range(1, cnt)) == xsum(x[j][0] for j in range(1, cnt)))

    # --- 4. Ràng buộc MTZ cải tiến cho VRPTW (Chương 3.3) ---
    # Ràng buộc này vừa loại bỏ chu trình con (Subtour), vừa đảm bảo thời gian và sức tải
    M = 1e5 # Big-M
    for i in range(cnt):
        for j in range(cnt):
            if i != j and j != 0:
                # Phụ thuộc thời gian: thời gian tại j >= thời gian tại i + phục vụ + di chuyển
                model.add_constr(t[j] >= t[i] + data_rows[i]['service'] + dist(i, j) - M * (1 - x[i][j]))
                # Phụ thuộc sức tải: tải trọng tại j >= tải trọng tại i + nhu cầu j
                model.add_constr(u[j] >= u[i] + data_rows[j]['demand'] - M * (1 - x[i][j]))

    # Giới hạn cửa sổ thời gian và sức tải xe cho từng khách hàng
    for i in range(cnt):
        # Time Window [Ready, Due]
        model.add_constr(t[i] >= data_rows[i]['ready'])
        model.add_constr(t[i] <= data_rows[i]['due'])
        # Capacity [Demand, Max_Capacity]
        model.add_constr(u[i] >= data_rows[i]['demand'])
        model.add_constr(u[i] <= sys_capacity)

    # --- 5. Thực hiện giải toán ---
    model.max_gap = 0.05 # Chấp nhận sai số 5% để chạy nhanh hơn
    status = model.optimize(max_seconds=60) # Giới hạn 60 giây

    if status.value == 0 or status.value == 1: # Optimal hoặc Feasible
        print(f"Thành công! Tổng quãng đường: {model.objective_value:.2f}")
        # In lộ trình
        for k in range(1, cnt): # Xe xuất phát từ 0 đến k
            if x[0][k].x >= 0.99:
                route = [0, k]
                curr = k
                while curr != 0:
                    for j in range(cnt):
                        if curr != j and x[curr][j].x >= 0.99:
                            route.append(j)
                            curr = j
                            break
                print(f"Xe: {' -> '.join(map(str, route))}")
    else:
        print("Không tìm thấy lời giải khả thi.")

def read_solomon(file_path, n_customers=25):
    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
        return None, None

    with open(file_path, 'r') as f:
        lines = f.readlines()

    # Capacity
    if len(lines) < 5: return None, None
    try:
        sys_capacity = int(lines[4].strip().split()[1])
    except:
        sys_capacity = 200 # Fallback

    data_rows = []
    # Data starts at line 9 (0-indexed)
    start_line = 9
    # Read depot + n_customers
    count = 0
    for i in range(start_line, len(lines)):
        parts = lines[i].strip().split()
        if len(parts) < 7: continue
        
        data_rows.append({
            'id': int(parts[0]),
            'x': float(parts[1]),
            'y': float(parts[2]),
            'demand': float(parts[3]),
            'ready': float(parts[4]),
            'due': float(parts[5]),
            'service': float(parts[6])
        })
        count += 1
        if count > n_customers: # 1 depot + n customers
            break
            
    return data_rows, sys_capacity

import os

if __name__ == "__main__":
    # Point to a file relative to the script or project root
    # Assuming script is in branch_and_cut/, and solomon-25 is in parent dir
    input_file = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "solomon-100", "C101.txt")
    
    print(f"Solving {input_file}...")
    data, cap = read_solomon(input_file, n_customers=100)
    
    if data:
        solve_vrptw_mip(data, cap)
    else:
        print("Could not read data.")