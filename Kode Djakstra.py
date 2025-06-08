import heapq
from itertools import permutations

# ==============================
# Struktur Data Graph dengan Class
# ==============================

class Graph:
    def __init__(self):
        self.edges = {}  # Adjacency list: kota -> list tetangga (tuple)

    def add_edge(self, u, v, weight):
        if u not in self.edges:
            self.edges[u] = []
        if v not in self.edges:
            self.edges[v] = []
        self.edges[u].append((v, weight))
        self.edges[v].append((u, weight))  # Graph tidak berarah

    def to_dict(self):
        """Mengonversi edges ke dictionary biasa untuk digunakan oleh algoritma"""
        result = {}
        for city, neighbors in self.edges.items():
            result[city] = {n: w for n, w in neighbors}
        return result

# ==============================
# Buat Graph Transportasi Korea Selatan (Jalur Darat)
# ==============================

g = Graph()

# Tambahkan jalur (30 edge)
g.add_edge('Seoul', 'Incheon', 27)
g.add_edge('Seoul', 'Suwon', 31)
g.add_edge('Seoul', 'Anseong', 65)
g.add_edge('Seoul', 'Daejeon', 140)
g.add_edge('Incheon', 'Suwon', 38)
g.add_edge('Incheon', 'Jeonju', 190)
g.add_edge('Suwon', 'Anseong', 40)
g.add_edge('Anseong', 'Daejeon', 105)
g.add_edge('Anseong', 'Daegu', 180)
g.add_edge('Daejeon', 'Jeonju', 75)
g.add_edge('Daejeon', 'Daegu', 150)
g.add_edge('Jeonju', 'Gwangju', 95)
g.add_edge('Gwangju', 'Busan', 270)
g.add_edge('Busan', 'Daegu', 90)
g.add_edge('Busan', 'Ulsan', 70)
g.add_edge('Ulsan', 'Daegu', 100)

# Ambil dalam format dictionary (untuk algoritma)
graph_korea = g.to_dict()

# ==============================
# Algoritma Dijkstra
# ==============================

def dijkstra(graph, start, end):
    queue = [(0, start, [])]
    visited = set()

    while queue:
        (cost, node, path) = heapq.heappop(queue)
        if node in visited:
            continue
        visited.add(node)
        path = path + [node]
        if node == end:
            return cost, path
        for neighbor, weight in graph[node].items():
            if neighbor not in visited:
                heapq.heappush(queue, (cost + weight, neighbor, path))
    return float('inf'), []

# ==============================
# Traveling Salesman Problem (TSP)
# ==============================

    
def tsp_brute_force_complete(graph, start):
    cities = list(graph.keys())
    cities.remove(start)
    min_cost = float('inf')
    min_path = None

    for perm in permutations(cities):
        current_cost = 0
        current_node = start
        valid = True

        for next_node in perm:
            if next_node in graph[current_node]:
                current_cost += graph[current_node][next_node]
            else:
                current_cost += 9999  # Penalti besar
            current_node = next_node

        # Kembali ke awal
        if start in graph[current_node]:
            current_cost += graph[current_node][start]
        else:
            current_cost += 9999

        if current_cost < min_cost:
            min_cost = current_cost
            min_path = (start,) + perm + (start,)

    return min_cost, min_path

valid_routes = 0  # Tambahkan di atas for perm in permutations




# ==============================
# Menu Interaktif
# ==============================

def main():
    print("Kota-Kota Yang Ada Di Korea Selatan:")
    print("Seoul, Busan, Incheon, Daegu, Daejeon, Gwangju, Ulsan, Suwon, Jeonju, Anseong")
    print("=" * 50)

    start = input("Masukkan kota asal: ").title()
    end = input("Masukkan kota tujuan: ").title()

    if start in graph_korea and end in graph_korea:
        cost, path = dijkstra(graph_korea, start, end)
        print(f"\n📍 Jalur tercepat dari {start} ke {end}: {' → '.join(path)}")
        print(f"🛣️  Total jarak: {cost} km")
    else:
        print("❌ Nama kota tidak valid!")

    print("\n🔁 Menjalankan TSP dari kota Seoul...")
    tsp_cost, tsp_path = tsp_brute_force_complete (graph_korea, 'Seoul')
    if tsp_path:
        print(f"🧭 Rute TSP terbaik: {' → '.join(tsp_path)}")
        print(f"🕒 Total jarak tempuh: {tsp_cost} km")
    else:
        print("❌ Tidak ada rute TSP yang valid.")

# ==============================
# Jalankan Program
# ==============================

if __name__ == "__main__":
    main()
