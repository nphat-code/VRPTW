import math
import os
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import matplotlib.pyplot as plt

def plot_solution(data_rows, routes):
    plt.figure(figsize=(12, 8))
    
    # Plot depot
    depot_x = data_rows[0]['x']
    depot_y = data_rows[0]['y']
    plt.scatter(depot_x, depot_y, c='red', marker='s', s=100, label='Depot', zorder=10)
    plt.annotate('Depot', (depot_x, depot_y), textcoords="offset points", xytext=(0,10), ha='center', fontsize=9, weight='bold')
    
    # Plot customers
    # cust_x = [d['x'] for d in data_rows[1:]]
    # cust_y = [d['y'] for d in data_rows[1:]]
    # plt.scatter(cust_x, cust_y, c='blue', s=30, label='Khách hàng', zorder=5)
    
    # Annotate customer IDs
    for d in data_rows[1:]:
         plt.scatter(d['x'], d['y'], c='blue', s=30, zorder=5)
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

    plt.title('Minh họa Lộ trình (VRPTW)')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def solve_vrptw_25(file_path):
    print(f"Đang giải file: {file_path} bằng OR-Tools")
    
    # --- 1. Đọc dữ liệu ---
    if not os.path.exists(file_path):
        print(f"Lỗi: Không tìm thấy file {file_path}")
        return

    with open(file_path, 'r') as f:
        lines = f.readlines()

    # Sức tải xe
    sys_capacity = int(lines[4].strip().split()[1])
    
    # Số lượng xe (giả định đủ lớn để giải)
    max_vehicles = 25 

    data_rows = []
    # Dữ liệu khách hàng từ dòng 10 (index 9)
    # Lấy 25 khách hàng + 1 depot (dòng đầu tiên là depot)
    # Tổng cộng 26 dòng
    start_line = 9
    n_customers = 25 
    lines_to_read = n_customers + 1 # 25 khách + 1 depot
    
    for i in range(lines_to_read):
        if start_line + i >= len(lines):
            break
        parts = lines[start_line + i].strip().split()
        if len(parts) < 7:
            continue
        data_rows.append({
            'id': int(parts[0]),
            'x': float(parts[1]),
            'y': float(parts[2]),
            'demand': float(parts[3]),
            'ready': float(parts[4]),
            'due': float(parts[5]),
            'service': float(parts[6])
        })

    if len(data_rows) < lines_to_read:
        print(f"Cảnh báo: Chỉ đọc được {len(data_rows)} dòng dữ liệu.")
    
    # Chuẩn bị dữ liệu cho OR-Tools
    # Depot là phần tử đầu tiên (index 0)
    
    # Ma trận khoảng cách & Thời gian (Scaling x100)
    scale_factor = 100
    cnt = len(data_rows)
    dist_matrix = [[0] * cnt for _ in range(cnt)]
    time_matrix = [[0] * cnt for _ in range(cnt)]

    for i in range(cnt):
        for j in range(cnt):
            d = math.sqrt((data_rows[i]['x'] - data_rows[j]['x'])**2 + 
                          (data_rows[i]['y'] - data_rows[j]['y'])**2)
            # Scale và chuyển về int
            dist_int = int(round(d * scale_factor))
            service_int = int(round(data_rows[i]['service'] * scale_factor))
            
            dist_matrix[i][j] = dist_int
            time_matrix[i][j] = dist_int + service_int

    data = {}
    data['time_matrix'] = time_matrix
    data['dist_matrix'] = dist_matrix
    data['time_windows'] = [(int(d['ready'] * scale_factor), int(d['due'] * scale_factor)) for d in data_rows]
    data['demands'] = [int(d['demand']) for d in data_rows]
    data['vehicle_capacities'] = [sys_capacity] * max_vehicles
    data['num_vehicles'] = max_vehicles
    data['depot'] = 0

    # --- 2. Khởi tạo Routing Model ---
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']),
                                           data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)

    # --- 3. Đăng ký Transit Callback ---
    
    # 3.1 Khoảng cách (cho Hàm mục tiêu)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['dist_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    # --- 4. Dimensions ---
    
    # 4.1 Capacity
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  
        data['vehicle_capacities'], 
        True, 
        'Capacity')

    # 4.2 Time Window
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    time_callback_index = routing.RegisterTransitCallback(time_callback)
    
    # Max time horizon
    max_time = 100000 * scale_factor
    
    routing.AddDimension(
        time_callback_index,
        max_time, 
        max_time,  
        False,  
        'Time')
    
    time_dimension = routing.GetDimensionOrDie('Time')
    
    # Add Time Window constraints
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.End(i)))

    # --- 5. Tối ưu hóa ---
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 30 # Giây

    print("Đang tìm lời giải...")
    solution = routing.SolveWithParameters(search_parameters)

    # --- 6. In kết quả ---
    if solution:
        print(f"\nGIẢI THÀNH CÔNG!")
        print(f"Tổng quãng đường (Objective): {solution.ObjectiveValue() / scale_factor:.2f}")
        
        solution_lines = [f"Tổng quãng đường: {solution.ObjectiveValue() / scale_factor:.2f}\n"]
        
        vehicle_count = 0
        all_routes = []
        
        for vehicle_id in range(data['num_vehicles']):
            index = routing.Start(vehicle_id)
            
            # Kiểm tra xe có dùng không
            if routing.IsEnd(solution.Value(routing.NextVar(index))):
                continue
                
            vehicle_count += 1
            route = []
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                route.append(node_index)
                index = solution.Value(routing.NextVar(index))
                
            route.append(manager.IndexToNode(index)) # End node
            all_routes.append(route)
            
            route_str = f"Xe {vehicle_count}: {' -> '.join(map(str, route))}"
            print(route_str)
            solution_lines.append(route_str + "\n")
            
        with open('solution.txt', 'w', encoding='utf-8') as f_out:
            f_out.writelines(solution_lines)
            
        # Vẽ biểu đồ
        plot_solution(data_rows, all_routes)
    else:
        print("Không tìm thấy lời giải.")

if __name__ == "__main__":
    solve_vrptw_25('C101.txt')