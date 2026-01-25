import math
import os
import matplotlib.pyplot as plt
from mip import Model, xsum, BINARY, MINIMIZE, ConstrsGenerator, OptimizationStatus

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
                model.add_constr(xsum(model.vars[f"x_{i}_{j}"] for i in component for j in component if i != j) <= len(component) - 1)

def read_solomon(file_path, n_customers=25):
    if not os.path.exists(file_path):
        return None, None
    with open(file_path, 'r') as f:
        lines = f.readlines()
    capacity = int(lines[4].strip().split()[1])
    data = []
    for i in range(9, 9 + n_customers + 1):
        if i >= len(lines): break
        p = lines[i].strip().split()
        if len(p) < 7: continue
        data.append({
            'id': int(p[0]), 'x': float(p[1]), 'y': float(p[2]),
            'demand': float(p[3]), 'ready': float(p[4]), 'due': float(p[5]), 'service': float(p[6])
        })
    return data, capacity

def solve_vrptw_branch_and_cut(data, capacity):
    n = len(data)
    # DÒNG QUAN TRỌNG: Khởi tạo mô hình
    model = Model(solver_name="CBC") 
    
    # Tính ma trận khoảng cách
    dist = [[math.sqrt((data[i]['x']-data[j]['x'])**2 + (data[i]['y']-data[j]['y'])**2) for j in range(n)] for i in range(n)]

    # 1. Biến quyết định
    x = [[model.add_var(var_type=BINARY, name=f"x_{i}_{j}") for j in range(n)] for i in range(n)]
    t = [model.add_var(name=f"t_{i}") for i in range(n)]
    u = [model.add_var(name=f"u_{i}") for i in range(n)]

    # 2. Hàm mục tiêu
    model.objective = xsum(dist[i][j] * x[i][j] for i in range(n) for j in range(n) if i != j)
    model.sense = MINIMIZE

    # 3. Ràng buộc luồng (Flow)
    for i in range(1, n):
        model.add_constr(xsum(x[i][j] for j in range(n) if i != j) == 1)
        model.add_constr(xsum(x[j][i] for j in range(n) if i != j) == 1)
    
    model.add_constr(xsum(x[0][j] for j in range(1, n)) <= 25)
    model.add_constr(xsum(x[0][j] for j in range(1, n)) == xsum(x[j][0] for j in range(1, n)))

    # 4. Ràng buộc Khung thời gian và Sức tải (MTZ)
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

    # 5. Kích hoạt Lazy Constraints
    model.constrs_generator = SubtourElimination(n)
    
    # 6. Giải toán
    model.max_gap = 0.05
    status = model.optimize(max_seconds=60)

    # --- HẬU XỬ LÝ KẾT QUẢ ---
    if status == OptimizationStatus.OPTIMAL or status == OptimizationStatus.FEASIBLE:
        # Lấy tổng quãng đường
        total_dist = model.objective_value
        print("\n" + "="*40)
        print(f"THÀNH CÔNG! TỔNG QUÃNG ĐƯỜNG: {total_dist:.2f}")
        print("="*40)
        
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
    else:
        print("Không tìm thấy lời giải trong thời gian quy định.")
        return None, None

def plot_routes(data, routes, total_dist):
    if not routes: return
    plt.figure(figsize=(12, 8))
    
    # Vẽ Depot và Khách hàng
    plt.scatter(data[0]['x'], data[0]['y'], c='red', marker='s', s=150, label='Depot', zorder=10)
    for d in data[1:]:
        plt.scatter(d['x'], d['y'], c='blue', s=40, zorder=5)
        plt.annotate(str(d['id']), (d['x'], d['y']), xytext=(0, 5), textcoords='offset points', ha='center', fontsize=9)
    
    # Vẽ lộ trình
    colors = plt.cm.get_cmap('tab20', len(routes))
    for idx, r in enumerate(routes):
        rx = [data[i]['x'] for i in r]
        ry = [data[i]['y'] for i in r]
        plt.plot(rx, ry, color=colors(idx), linewidth=2.5, alpha=0.7, label=f'Xe {idx+1}')

    # Hiển thị Tổng quãng đường ngay trên tiêu đề ảnh
    plt.title(f"Kết quả VRPTW - Branch and Cut\nTổng quãng đường: {total_dist:.2f}", fontsize=14, fontweight='bold', color='darkgreen')
    plt.legend(loc='best', fontsize='small', ncol=2)
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.show()

if __name__ == "__main__":
    file_path = "solomon-25/C101.txt"
    data, capacity = read_solomon(file_path, n_customers=25)
    if data:
        print(f"--- Đang giải {file_path} ---")
        # Hàm trả về 2 giá trị: routes và total_dist
        routes, total_dist = solve_vrptw_branch_and_cut(data, capacity)
        if routes:
            plot_routes(data, routes, total_dist)
    else:
        print(f"Lỗi: Không tìm thấy file {file_path}")