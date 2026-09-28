"""
Enemy AI Dungeon Simulation
---------------------------
Algoritma yang diimplementasikan:
1. Finite State Machine (FSM): IDLE -> CHASE -> ATTACK
2. Raycasting (Bresenham's Line Algorithm) untuk Line of Sight (LoS)
3. A* (A-Star) Pathfinding untuk mencari rute terpendek di dungeon
4. Euclidean & Manhattan Distance untuk deteksi jarak
"""

import math
import heapq
import time


class Vector2:
    """Kelas pendukung untuk posisi 2D (Grid)"""
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y

    def euclidean_distance(self, other: 'Vector2') -> float:
        """Menghitung jarak Euclidean (Garis lurus)"""
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def manhattan_distance(self, other: 'Vector2') -> int:
        """Menghitung jarak Manhattan (Langkah Grid)"""
        return abs(self.x - other.x) + abs(self.y - other.y)

    def __eq__(self, other):
        return isinstance(other, Vector2) and self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __repr__(self):
        return f"({self.x}, {self.y})"


class DungeonMap:
    """Representasi Peta Dungeon Grid"""
    def __init__(self, grid_data):
        # 0 = Jalan (Empty), 1 = Tembok (Wall)
        self.grid = grid_data
        self.rows = len(grid_data)
        self.cols = len(grid_data[0])

    def is_valid_tile(self, pos: Vector2) -> bool:
        """Mengecek apakah koordinat ada di dalam map dan bukan tembok"""
        return 0 <= pos.x < self.rows and 0 <= pos.y < self.cols and self.grid[pos.x][pos.y] == 0

    def render(self, enemy_pos: Vector2, player_pos: Vector2, path=None):
        """Menampilkan peta dungeon dalam bentuk ASCII Art di terminal"""
        if path is None:
            path = []
        
        path_set = set((p.x, p.y) for p in path)

        print("+" + "---" * self.cols + "+")
        for r in range(self.rows):
            row_str = "|"
            for c in range(self.cols):
                curr = Vector2(r, c)
                if curr == enemy_pos and curr == player_pos:
                    row_str += " X "  # Pertemuan/Attack
                elif curr == enemy_pos:
                    row_str += " E "  # Enemy
                elif curr == player_pos:
                    row_str += " P "  # Player
                elif (r, c) in path_set:
                    row_str += " * "  # Jalur A* Path
                elif self.grid[r][c] == 1:
                    row_str += "###"  # Tembok
                else:
                    row_str += " . "  # Jalan Kosong
            row_str += "|"
            print(row_str)
        print("+" + "---" * self.cols + "+")


def bresenham_line_of_sight(dungeon: DungeonMap, start: Vector2, end: Vector2) -> bool:
    """
    Algoritma Raycasting (Bresenham's Line Algorithm)
    Mengecek apakah ada tembok yang menghalangi pandangan lurus dari start ke end.
    """
    x1, y1 = start.x, start.y
    x2, y2 = end.x, end.y

    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1

    err = dx - dy

    curr_x, curr_y = x1, y1

    while True:
        # Jika titik saat ini adalah tembok (selain start dan end), pandangan terhalang
        if not (curr_x == start.x and curr_y == start.y) and not (curr_x == end.x and curr_y == end.y):
            if dungeon.grid[curr_x][curr_y] == 1:
                return False  # Terhalang tembok!

        if curr_x == x2 and curr_y == y2:
            break

        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            curr_x += sx
        if e2 < dx:
            err += dx
            curr_y += sy

    return True  # Pandangan bersih (Line of Sight Clear)


def a_star_search(dungeon: DungeonMap, start: Vector2, goal: Vector2) -> list[Vector2]:
    """
    Algoritma Pathfinding A* (A-Star)
    Mencari jalur terpendek dari start (Enemy) menuju goal (Player).
    """
    def heuristic(a: Vector2, b: Vector2) -> float:
        return a.manhattan_distance(b)

    open_set = []
    heapq.heappush(open_set, (0, (start.x, start.y)))

    came_from = {}
    g_score = {(start.x, start.y): 0}

    neighbors_offset = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4 arah gerakan

    while open_set:
        _, current_tuple = heapq.heappop(open_set)
        current = Vector2(current_tuple[0], current_tuple[1])

        if current == goal:
            # Rekonstruksi rute dari goal kembali ke start
            path = []
            curr_key = (goal.x, goal.y)
            while curr_key in came_from:
                path.append(Vector2(curr_key[0], curr_key[1]))
                curr_key = came_from[curr_key]
            path.reverse()
            return path

        for dx, dy in neighbors_offset:
            neighbor = Vector2(current.x + dx, current.y + dy)
            if dungeon.is_valid_tile(neighbor):
                tentative_g = g_score[(current.x, current.y)] + 1

                neighbor_key = (neighbor.x, neighbor.y)
                if neighbor_key not in g_score or tentative_g < g_score[neighbor_key]:
                    g_score[neighbor_key] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    came_from[neighbor_key] = (current.x, current.y)
                    heapq.heappush(open_set, (f_score, neighbor_key))

    return []  # Jalur tidak ditemukan


class Enemy:
    """Kelas AI Musuh dengan Finite State Machine (FSM)"""
    def __init__(self, position: Vector2, detection_radius: float = 6.0, attack_radius: float = 1.5):
        self.position = position
        self.detection_radius = detection_radius  # Radius pendeteksian (Euclidean)
        self.attack_radius = attack_radius        # Radius serangan
        self.state = "IDLE"                        # Initial State
        self.current_path = []

    def update(self, player_pos: Vector2, dungeon: DungeonMap):
        distance = self.position.euclidean_distance(player_pos)
        has_los = bresenham_line_of_sight(dungeon, self.position, player_pos)

        print(f"\n--- [Status Musuh] ---")
        print(f"Posisi Enemy     : {self.position}")
        print(f"Posisi Player    : {player_pos}")
        print(f"Jarak ke Player  : {distance:.2f} (Max Radius: {self.detection_radius})")
        print(f"Line of Sight    : {'TERLIHAT (Clear)' if has_los else 'TERHALANG TEMBOK'}")

        # --- TRANSISI FINITE STATE MACHINE (FSM) ---
        if distance <= self.detection_radius and has_los:
            if distance <= self.attack_radius:
                self.state = "ATTACK"
                self.current_path = []
            else:
                self.state = "CHASE"
                # Hitung ulang rute A* menuju player
                self.current_path = a_star_search(dungeon, self.position, player_pos)
        else:
            self.state = "IDLE"
            self.current_path = []

        print(f"Current State    : >> {self.state} <<")

        # --- EKSEKUSI BEHAVIOR SESUAI STATE ---
        if self.state == "CHASE" and self.current_path:
            # Bergerak ke langkah berikutnya dalam jalur
            next_step = self.current_path[0]
            print(f"Aksi             : Musuh melangkah dari {self.position} -> {next_step}")
            self.position = next_step
        elif self.state == "ATTACK":
            print(f"Aksi             : MUSUH MENYERANG PLAYER! (Dealing Damage)")
        elif self.state == "IDLE":
            print(f"Aksi             : Musuh diam / berpatroli (Player belum terdeteksi)")


# --- MAIN SIMULATION ---
def main():
    # Grid Dungeon (6x8)
    # 0 = Jalan, 1 = Tembok (###)
    raw_map = [
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0]
    ]

    dungeon = DungeonMap(raw_map)
    enemy = Enemy(position=Vector2(0, 0), detection_radius=6.0, attack_radius=1.5)
    
    # Player memulai di area tersembunyi (dibalik tembok), lalu bergerak ke lorong utama
    player_positions = [
        Vector2(3, 3), # Step 1: Dibalik tembok (IDLE)
        Vector2(0, 5), # Step 2: Masuk lorong (CHASE dimulai!)
        Vector2(0, 5), # Step 3...
        Vector2(0, 5),
        Vector2(0, 5),
        Vector2(0, 5),
        Vector2(0, 5)
    ]

    print("==================================================")
    print("   SIMULASI ALGORITMA AI ENEMY DUNGEON PATHFINDING")
    print("==================================================")
    print("Keterangan Simbol:")
    print("  E  = Enemy (Musuh)")
    print("  P  = Player")
    print("  *  = Jalur A* Pathfinding")
    print(" ### = Tembok Dungeon")
    print("  .  = Jalan Kosong")
    print("==================================================")

    step = 1
    while step <= len(player_positions):
        player = player_positions[step - 1]
        print(f"\n================ STEP {step} ================")
        
        # Render Peta sebelum pergerakan
        dungeon.render(enemy.position, player, enemy.current_path)

        # Update Logika Musuh
        enemy.update(player, dungeon)

        # Berhenti jika musuh sudah berhasil menyerang player
        if enemy.state == "ATTACK":
            print("\n================ HASIL AKHIR ================")
            dungeon.render(enemy.position, player)
            print("Musuh berhasil mendeteksi, mengejar, dan menyerang Player!")
            break

        step += 1
        time.sleep(0.3)


if __name__ == "__main__":
    main()
