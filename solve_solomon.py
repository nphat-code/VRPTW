import gurobipy as gp
from gurobipy import GRB
import math

def solve_vrptw_c101(file_path):
    # --- 1. Đọc và xử lý dữ liệu từ file C101.txt ---
    with open(file_path, 'r') as f:
        lines = f.readlines()

    # Sức tải xe (Capacity) ở dòng thứ 5
    capacity = int(lines[4].strip().split()[1])

    # Dữ liệu khách hàng bắt đầu từ dòng thứ 10
    # Định dạng: [ID, X, Y, Demand, ReadyTime, DueDate, ServiceTime]
    nodes = []
    for line in lines[9:]:
        parts = line.strip().split()
        if len(parts) >= 7:
            nodes.append([
                int(parts[0]),     # ID
                float(parts[1]),   # X
                float(parts[2]),   # Y
                float(parts[3]),   # Demand
                float(parts[4]),   # Ready Time
                float(parts[5]),   # Due Date
                float(parts[6])    # Service Time
            ])
        if len(nodes) == 26:  # Lấy Depot + 25 khách hàng đầu tiên
            break

    n = len(nodes)
    clients = range(1, n)
    all_nodes = range(n)

    # Tính ma trận khoảng cách Euclidean
    dist = {(i, j): math.sqrt((nodes[i][1]-nodes[j][1])**2 + (nodes[i][2]-nodes[j][2])**2)
            for i in all_nodes for j in all_nodes if i != j}

    # --- 2. Khởi tạo Mô hình Gurobi ---
    model = gp.Model("VRPTW_C101")
    model.Params.OutputFlag = 1  # Hiện quá trình giải

    # --- 3. Khai báo Biến quyết định ---
    # x[i,j] = 1 nếu xe đi trực tiếp từ i đến j
    x = model.addVars(dist.keys(), vtype=GRB.BINARY, name="x")
    # s[i] = thời điểm bắt đầu phục vụ tại khách hàng i
    s = model.addVars(all_nodes, lb=[nodes[i][4] for i in all_nodes], 
                      ub=[nodes[i][5] for i in all_nodes], name="s")
    # q[i] = lượng hàng tích lũy trên xe sau khi rời khách hàng i
    q = model.addVars(all_nodes, lb=0, ub=capacity, name="q")

    # --- 4. Hàm mục tiêu: Tối thiểu hóa tổng quãng đường ---
    model.setObjective(gp.quicksum(dist[i, j] * x[i, j] for i, j in dist.keys()), GRB.MINIMIZE)

    # --- 5. Các ràng buộc ---
    # Mỗi khách hàng phải được ghé thăm đúng 1 lần
    model.addConstrs(gp.quicksum(x[i, j] for j in all_nodes if i != j) == 1 for i in clients)
    model.addConstrs(gp.quicksum(x[j, i] for j in all_nodes if i != j) == 1 for i in clients)

    # Bảo toàn luồng tại kho (Depot)
    model.addConstr(gp.quicksum(x[0, j] for j in clients) == gp.quicksum(x[j, 0] for j in clients))

    # Ràng buộc thời gian (Time Windows) & Tránh chu trình con
    M = 10000 
    for i, j in dist.keys():
        model.addConstr(s[i] + nodes[i][6] + dist[i, j] <= s[j] + M * (1 - x[i, j]))

    # Ràng buộc sức tải (Capacity)
    for i, j in dist.keys():
        if j != 0:
            model.addConstr(q[i] + nodes[j][3] <= q[j] + M * (1 - x[i, j]))
    
    # Nhu cầu tối thiểu tại mỗi điểm
    for i in clients:
        model.addConstr(q[i] >= nodes[i][3])

    # --- 6. Giải thuật và In kết quả ---
    model.optimize()

    if model.status == GRB.OPTIMAL:
        print(f"\nGIẢI THÀNH CÔNG!")
        print(f"Tổng quãng đường tối ưu: {model.objVal:.2f}")
        
        # Tìm và in lộ trình từng xe
        used_arcs = [arc for arc in dist.keys() if x[arc].X > 0.5]
        start_nodes = [j for i, j in used_arcs if i == 0]
        
        for idx, start_node in enumerate(start_nodes):
            route = [0, start_node]
            curr = start_node
            while curr != 0:
                for i, j in used_arcs:
                    if i == curr:
                        route.append(j)
                        curr = j
                        break
            print(f"Xe {idx+1}: {' -> '.join(map(str, route))}")
    else:
        print("Không tìm thấy lời giải tối ưu.")

if __name__ == "__main__":
    # Đảm bảo file C101.txt nằm cùng thư mục với file python này
    solve_vrptw_c101('C101.txt')