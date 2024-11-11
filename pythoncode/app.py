from flask import Flask, request, jsonify, render_template
import heapq

app = Flask(__name__)

class Node:
    def __init__(self, position, cost=0, heuristic=0, parent=None):
        self.position = position  
        self.cost = cost
        self.heuristic = heuristic
        self.parent = parent

    def total_cost(self):
        return self.cost + self.heuristic

    def __lt__(self, other):
        return self.total_cost() < other.total_cost()

class AStarPathfinding:
    def __init__(self, start, goal, blocked_cells, grid_size):
        self.start = start  
        self.goal = goal    
        self.blocked_cells = set(blocked_cells)
        self.grid_size = grid_size

    def is_valid(self, position):
        y, x = position
        if not (0 <= y < self.grid_size[1] and 0 <= x < self.grid_size[0]):
            return False
        if position in self.blocked_cells:
            return False
        return True

    def get_neighbors(self, node):
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1),
                      (1, 1), (1, -1), (-1, 1), (-1, -1)]
        neighbors = []
        for direction in directions:
            new_position = (node.position[0] + direction[0], node.position[1] + direction[1])
            if self.is_valid(new_position):
                neighbors.append(new_position)
        return neighbors

    def heuristic(self, current_position, goal_position):
        return abs(current_position[0] - goal_position[0]) + abs(current_position[1] - goal_position[1])

    def a_star_search(self):
        start_node = Node(self.start, cost=0, heuristic=self.heuristic(self.start, self.goal))
        open_list = []
        heapq.heappush(open_list, start_node)
        closed_set = set()
        cost_so_far = {self.start: 0}

        while open_list:
            current_node = heapq.heappop(open_list)

            if current_node.position == self.goal:
                path = []
                while current_node:
                    path.append(current_node.position)
                    current_node = current_node.parent
                return path[::-1]

            closed_set.add(current_node.position)

            for neighbor in self.get_neighbors(current_node):
                new_cost = current_node.cost + 1

                if neighbor in closed_set and new_cost >= cost_so_far.get(neighbor, float('inf')):
                    continue

                if new_cost < cost_so_far.get(neighbor, float('inf')):
                    cost_so_far[neighbor] = new_cost
                    neighbor_node = Node(position=neighbor, cost=new_cost,
                                         heuristic=self.heuristic(neighbor, self.goal), parent=current_node)
                    heapq.heappush(open_list, neighbor_node)

        return None

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/find_path', methods=['POST'])
def find_path():
    grid_width = int(request.form['grid_width'])
    grid_height = int(request.form['grid_height'])
    grid_size = (grid_height, grid_width)

    start_y = int(request.form['start_y'])
    start_x = int(request.form['start_x'])
    goal_y = int(request.form['goal_y'])
    goal_x = int(request.form['goal_x'])

    start = (start_y, start_x)
    goal = (goal_y, goal_x)

    blocked_weather = request.form['blocked_weather'].strip().split(';')
    blocked_traffic = request.form['blocked_traffic'].strip().split(';')
    no_fly_zone = request.form['no_fly_zone'].strip().split(';')

    blocked_cells = set()
    for node_list in (blocked_weather, blocked_traffic, no_fly_zone):
        for node in node_list:
            if node:
                y, x = map(int, node.split(','))
                blocked_cells.add((y, x))

    pathfinder = AStarPathfinding(start, goal, blocked_cells, grid_size)
    path = pathfinder.a_star_search()

    if path:
        return jsonify({'path': path})
    else:
        return jsonify({'error': 'No path found.'})

if __name__ == '__main__':
    app.run(debug=True)
    
