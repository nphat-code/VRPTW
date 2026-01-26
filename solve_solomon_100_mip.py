import math
import os
import matplotlib.pyplot as plt
from mip import Model, xsum, BINARY, MINIMIZE, ConstrsGenerator, OptimizationStatus

# --- 1. LỚP TẠO NHÁT CẮT (LAZY CONSTRAINTS) ---
class SubtourElimination(ConstrsGenerator):
    def __init__(self, n):
        self.n = n

    def generate_constrs(self, model, nodes_ix):
        adj = [[] for _ in range(self.n)]
        for (i, j) in [(i, j) for i in range(self.n) for j in range(self.n) if i != j]:
            var = model.vars[f"x_{i}_{j}"]
            val = model.translate(var).x
            if val is not None and val >= 0.99:
                adj[i].append(j)

        unvisited = set(range(1, self.n))
        while unvisited:
            stack = [next(iter(unvisited))]
            component = set()
            while stack:
                u = stack.pop()
                if u in unvisited:
                    unvisited.remove(u)
                    component.add(u)
                    stack.extend(adj[u])
            
            if len(component) >= 2:
                # Nhát cắt loại bỏ chu trình con - Rất quan trọng cho 100 khách hàng
                model.add_constr(xsum(model.vars[f"x_{i}_{j}"] for i in component for j in component if i != j) <= len(component) - 1)

# --- 2. HÀM ĐỌC FILE SOLOMON (100 CUSTOMERS) ---
def read_solomon_100(file_path, n_customers=100):
    if not os.path.exists(file_path):
        return None, None
    with open(file_path, 'r') as f:
        lines = f.readlines()
    
    capacity = int(lines[4].strip().split()[1])
    data = []
    # Đọc Depot + 100 dòng khách hàng
    for i in range(9, 9 + n_customers + 1):
        if i >= len(lines): break
        p = lines[i].strip().split()
        if len(p) < 7: continue
        data.append({
            'id': int(p[0]), 'x': float(p[1]), 'y': float(p[2]),
            'demand': float(p[3]), 'ready': float(p[4]), 'due': float(p[5]), 'service': float(p[6])
        })
    return data, capacity

# --- 3. THUẬT TOÁN BRANCH AND CUT (OPTIMIZED) ---
def solve_vrptw_100(data, capacity):
    n = len(data)
    model = Model(solver_name="CBC")
    
    # Tính ma trận khoảng cách Euclidean
    dist = [[math.sqrt((data[i]['x']-data[j]['x'])**2 + (data[i]['y']-data[j]['y'])**2) for j in range(n)] for i in range(n)]

    # Biến quyết định
    x = [[model.add_var(var_type=BINARY, name=f"x_{i}_{j}") for j in range(n)] for i in range(n)]
    t = [model.add_var(name=f"t_{i}") for i in range(n)]
    u = [model.add_var(name=f"u_{i}") for i in range(n)]

    model.objective = xsum(dist[i][j] * x[i][j] for i in range(n) for j in range(n) if i != j)
    model.sense = MINIMIZE

    # Ràng buộc luồng (Degree constraints)
    for i in range(1, n):
        model.add_constr(xsum(x[i][j] for j in range(n) if i != j) == 1)
        model.add_constr(xsum(x[j][i] for j in range(n) if i != j) == 1)
    
    model.add_constr(xsum(x[0][j] for j in range(1, n)) <= 25) 
    model.add_constr(xsum(x[0][j] for j in range(1, n)) == xsum(x[j][0] for j in range(1, n)))

    # Ràng buộc MTZ cải tiến cho Time Windows & Capacity
    M = 1e5
    for i in range(n):
        for j in range(1, n):
            if i != j:
                model.add_constr(t[j] >= t[i] + data[i]['service'] + dist[i][j] - M * (1 - x[i][j]))
                model.add_constr(u[j] >= u[i] + data[j]['demand'] - M * (1 - x[i][j]))

    for i in range(n):
        model.add_constr(t[i] >= data[i]['ready'])
        model.add_constr(t[i] <= data[i]['due'])
        model.add_constr(u[i] >= data[i]['demand'])
        model.add_constr(u[i] <= capacity)

    # Kích hoạt tạo nhát cắt tự động (Lazy Constraints)
    model.constrs_generator = SubtourElimination(n)
    
    # Cài đặt giới hạn (100 khách hàng cần kiên nhẫn hơn)
    model.max_gap = 0.10 # Cho phép sai số 10% để nhanh có kết quả
    status = model.optimize(max_seconds=300) # Giới hạn 5 phút

    if status == OptimizationStatus.OPTIMAL or status == OptimizationStatus.FEASIBLE:
        total_dist = model.objective_value
        print(f"\n[HOÀN THÀNH] TỔNG QUÃNG ĐƯỜNG: {total_dist:.2f}")
        
        routes = []
        for j in range(1, n):
            if x[0][j].x is not None and x[0][j].x >= 0.99:
                route = [0, j]
                curr = j
                while curr != 0:
                    for k in range(n):
                        if curr != k and x[curr][k].x is not None and x[curr][k].x >= 0.99:
                            route.append(k)
                            curr = k
                            break
                routes.append(route)
                print(f"Xe {len(routes)}: {' -> '.join(map(str, route))}")
        return routes, total_dist
    return None, None

# --- 4. VẼ BIỂU ĐỒ KẾT QUẢ ---
def plot_solution(data_rows, routes, total_dist=None, save_path=None):
    plt.figure(figsize=(12, 8))
    
    # Plot depot
    depot_x = data_rows[0]['x']
    depot_y = data_rows[0]['y']
    plt.scatter(depot_x, depot_y, c='red', marker='s', s=100, label='Depot', zorder=10)
    plt.annotate('Depot', (depot_x, depot_y), textcoords="offset points", xytext=(0,10), ha='center', fontsize=9, weight='bold')
    
    # Annotate customer IDs
    for d in data_rows[1:]:
         plt.scatter(d['x'], d['y'], c='blue', s=30, zorder=5)
         # Dù 100 điểm có thể rối, nhưng để "giống cách vẽ của file kia", ta vẫn vẽ
         plt.annotate(str(d['id']), (d['x'], d['y']), textcoords="offset points", xytext=(0,5), ha='center', fontsize=8)
    
    # Create a dummy scatter for legend
    plt.scatter([], [], c='blue', s=30, label='Khách hàng')
    
    # Colors for routes
    cmap = plt.get_cmap('tab10')
    
    for i, route in enumerate(routes):
        color = cmap(i % 10)
        
        # Get coordinates for the route
        route_x = [data_rows[node]['x'] for node in route]
        route_y = [data_rows[node]['y'] for node in route]
        
        # Plot lines
        plt.plot(route_x, route_y, c=color, linewidth=2, label=f'Xe {i+1}', alpha=0.7)
        
        # Add arrows direction
        for j in range(len(route)-1):
            p1 = (data_rows[route[j]]['x'], data_rows[route[j]]['y'])
            p2 = (data_rows[route[j+1]]['x'], data_rows[route[j+1]]['y'])
            
            # Simple midpoint for arrow
            mid_x = (p1[0] + p2[0]) / 2
            mid_y = (p1[1] + p2[1]) / 2
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            
            plt.arrow(mid_x - dx*0.1, mid_y - dy*0.1, dx*0.2, dy*0.2, 
                      head_width=1.5, head_length=2, fc=color, ec=color)

    title = 'Minh họa Lộ trình (VRPTW)'
    if total_dist:
        title += f" - Tổng quãng đường: {total_dist:.2f}"
    plt.title(title)
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path)
        print(f"Đã lưu hình ảnh lộ trình tại: {save_path}")
        
    plt.show()

# --- CHƯƠNG TRÌNH CHÍNH ---
if __name__ == "__main__":
    FOLDER = "solomon-100"
    FILE_NAME = "RC201.txt"
    PATH = os.path.join(FOLDER, FILE_NAME)
    
    print(f"--- Bắt đầu giải bài toán 100 khách hàng: {PATH} ---")
    data, cap = read_solomon_100(PATH, n_customers=100)
    
    if data:
        routes, total_dist = solve_vrptw_100(data, cap)
        if routes:
            if not os.path.exists('route_images'):
                os.makedirs('route_images')
            base_name = os.path.splitext(os.path.basename(PATH))[0]
            save_path = f'route_images/solution_100_mip_{base_name}.png'
            plot_solution(data, routes, total_dist, save_path=save_path)
        else:
            print("Không tìm thấy lời giải khả thi trong thời gian giới hạn.")
    else:
        print(f"Lỗi: Không tìm thấy file {PATH}. Hãy kiểm tra thư mục 'solomon-100'.")