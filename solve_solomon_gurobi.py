import gurobipy as gp
from gurobipy import GRB
import math

def solve_vrptw_c101(file_path):
    print(f"Đang giải file: {file_path}")
    
    # --- 1. Đọc dữ liệu ---
    with open(file_path, 'r') as f:
        lines = f.readlines()

    sys_capacity = int(lines[4].strip().split()[1])

    data = []
    # Dữ liệu khách hàng từ dòng 10 (index 9)
    # Lấy 25 khách hàng + 1 depot (dòng đầu tiên là depot)
    # Tổng cộng 26 dòng
    start_line = 9
    for i in range(26):
        parts = lines[start_line + i].strip().split()
        data.append({
            'id': int(parts[0]),
            'x': float(parts[1]),
            'y': float(parts[2]),
            'demand': float(parts[3]),
            'ready': float(parts[4]),
            'due': float(parts[5]),
            'service': float(parts[6])
        })
    
    # --- 2. Xử lý Nodes ---
    # Node 0: Depot bắt đầu
    # Node 1..25: Khách hàng
    # Node 26: Depot kết thúc (bản sao của Node 0)
    
    n_customers = 25
    clients = [i for i in range(1, n_customers + 1)] # 1..25
    nodes = [i for i in range(n_customers + 2)]      # 0..26
    
    # Mapping coordinates & properties
    coords = {i: (d['x'], d['y']) for i, d in enumerate(data)}
    coords[26] = coords[0] # Depot end
    
    demand = {i: d['demand'] for i, d in enumerate(data)}
    demand[26] = 0
    
    ready = {i: d['ready'] for i, d in enumerate(data)}
    ready[26] = ready[0]
    
    due = {i: d['due'] for i, d in enumerate(data)}
    due[26] = due[0]
    
    service = {i: d['service'] for i, d in enumerate(data)}
    service[26] = 0
    
    def get_dist(i, j):
        return math.sqrt((coords[i][0]-coords[j][0])**2 + (coords[i][1]-coords[j][1])**2)
        
    # --- 3. Tạo Cung (Arcs) ---
    arcs = []
    # Từ Depot Start -> Khách hàng
    for j in clients:
        arcs.append((0, j))
        
    # Giữa các Khách hàng
    for i in clients:
        for j in clients:
            if i != j:
                arcs.append((i, j))
                
    # Từ Khách hàng -> Depot End
    for i in clients:
        arcs.append((i, 26))
        
    dist = {(i, j): get_dist(i, j) for i, j in arcs}
    
    # --- 4. Mô hình Gurobi ---
    # Branch and Cut được Gurobi áp dụng tự động (Cuts, Presolve, Heuristics)
    m = gp.Model("VRPTW_C101_Corrected")
    
    # Biến quyết định
    # x[i,j] = 1 nếu đi từ i đến j
    x = m.addVars(arcs, vtype=GRB.BINARY, name="x")
    
    # s[i] = Thời gian bắt đầu phục vụ tại i
    s = m.addVars(nodes, vtype=GRB.CONTINUOUS, name="s")
    for i in nodes:
        s[i].LB = ready[i]
        s[i].UB = due[i]
    
    # q[i] = Tải trọng tích lũy khi đến i
    q = m.addVars(nodes, vtype=GRB.CONTINUOUS, name="q")
    for i in nodes:
        q[i].LB = 0
        q[i].UB = sys_capacity
        
    # Hàm mục tiêu: Tối thiểu tổng quãng đường
    m.setObjective(gp.quicksum(dist[i,j] * x[i,j] for i,j in arcs), GRB.MINIMIZE)
    
    # --- 5. Ràng buộc ---
    
    # 5.1 Mỗi khách hàng được thăm đúng 1 lần
    for i in clients:
        # Tổng dòng vào i = 1
        m.addConstr(gp.quicksum(x[j, i] for j in nodes if (j, i) in arcs) == 1, name=f"flow_in_{i}")
        # Tổng dòng ra i = 1
        m.addConstr(gp.quicksum(x[i, j] for j in nodes if (i, j) in arcs) == 1, name=f"flow_out_{i}")

    # 5.2 Ràng buộc Thời gian (Time Windows)
    # Nếu đi từ i -> j, thì s[i] + service[i] + dist[i,j] <= s[j]
    # Sử dụng Big-M formulation
    M_time = max(due.values()) + 1000 # Đủ lớn
    for i, j in arcs:
        m.addConstr(s[i] + service[i] + dist[i,j] <= s[j] + M_time * (1 - x[i, j]), name=f"time_{i}_{j}")
        
    # 5.3 Ràng buộc Tải trọng (Capacity)
    # Nếu đi từ i -> j, thì q[i] + demand[j] <= q[j]
    # q[i] ở đây hiểu là tải trọng ĐÃ CÓ trên xe khi đến i (accumulated load)
    # Lưu ý: Demand tại Depot (0 và 26) là 0.
    # q[0] = 0.
    M_cap = sys_capacity + 100
    m.addConstr(q[0] == 0, name="depot_load_start")
    
    for i, j in arcs:
        m.addConstr(q[i] + demand[j] <= q[j] + M_cap * (1 - x[i, j]), name=f"cap_{i}_{j}")
        
    # Đảm bảo q[i] >= demand[i] cho các khách hàng (thực ra đã được cover bởi q[0]=0 và dòng trên)
    for i in clients:
        m.addConstr(q[i] >= demand[i], name=f"min_load_{i}")

    # --- 6. Tối ưu hóa ---
    m.Params.TimeLimit = 60 # Giới hạn thời gian 60s
    m.optimize()
    
    if m.status == GRB.OPTIMAL:
        print(f"\nGIẢI THÀNH CÔNG!")
        result_str = f"Tổng quãng đường: {m.objVal:.2f}\n"
        print(result_str.strip())
        
        # Truy vết lộ trình
        x_vals = m.getAttr('x', x)
        active_arcs = [a for a in arcs if x_vals[a] > 0.5]
        
        adj = {i: j for i, j in active_arcs}
        
        # Tìm các xe bắt đầu từ 0
        vehicle_count = 0
        start_nodes = [j for j in clients if (0, j) in active_arcs]
        
        solution_lines = [result_str]
        
        for start_node in start_nodes:
            vehicle_count += 1
            route = [0]
            curr = start_node
            route.append(curr)
            
            # Load check
            load = 0
            
            while curr != 26:
                curr = adj[curr]
                route.append(curr if curr != 26 else 0) # In ra 0 cho dễ hiểu thay vì 26
            
            route_str = f"Xe {vehicle_count}: {' -> '.join(map(str, route))}"
            print(route_str)
            solution_lines.append(route_str + "\n")
            
        with open('solution.txt', 'w', encoding='utf-8') as f_out:
            f_out.writelines(solution_lines)
            
    elif m.status == GRB.INFEASIBLE:
        print("Mô hình không khả thi (Infeasible). Đang tìm nguyên nhân...")
        m.computeIIS()
        m.write("model.ilp")
        print("Đã ghi file model.ilp chứa các ràng buộc mâu thuẫn.")
    else:
        print(f"Không tìm thấy lời giải tối ưu. Status: {m.status}")

if __name__ == "__main__":
    solve_vrptw_c101('C101.txt')