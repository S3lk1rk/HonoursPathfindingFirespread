import pygame
import math
from queue import PriorityQueue
import time
import random

WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm")

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)


class Spot:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows

    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        return self.color == RED

    def is_open(self):
        return self.color == GREEN

    def is_barrier(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == ORANGE

    def is_end(self):
        return self.color == TURQUOISE

    def reset(self):
        self.color = WHITE

    def make_start(self):
        self.color = ORANGE

    def make_closed(self):
        self.color = RED

    def make_open(self):
        self.color = GREEN

    def make_barrier(self):
        self.color = BLACK

    def make_end(self):
        self.color = TURQUOISE

    def make_path(self):
        self.color = PURPLE

    def is_fire(self):
        return self.color == ORANGE  # Change from RED to ORANGE

    def make_fire(self):
        self.color = ORANGE  # Change from RED to ORANGE

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):
        self.neighbors = []
        # DOWN
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier() and not grid[self.row + 1][self.col].is_fire():
            self.neighbors.append(grid[self.row + 1][self.col])

        # UP
        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier() and not grid[self.row - 1][self.col].is_fire():
            self.neighbors.append(grid[self.row - 1][self.col])

        # RIGHT
        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier() and not grid[self.row][self.col + 1].is_fire():
            self.neighbors.append(grid[self.row][self.col + 1])

        # LEFT
        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier() and not grid[self.row][self.col - 1].is_fire():
            self.neighbors.append(grid[self.row][self.col - 1])

    def __lt__(self, other):
        return False


def h(p1, p2):
	x1, y1 = p1
	x2, y2 = p2
	return abs(x1 - x2) + abs(y1 - y2)


def reconstruct_path(came_from, current, draw):
	while current in came_from:
		current = came_from[current]
		current.make_path()
		draw()


def algorithm(draw, grid, start, end):
	count = 0
	open_set = PriorityQueue()
	open_set.put((0, count, start))
	came_from = {}
	g_score = {spot: float("inf") for row in grid for spot in row}
	g_score[start] = 0
	f_score = {spot: float("inf") for row in grid for spot in row}
	f_score[start] = h(start.get_pos(), end.get_pos())

	open_set_hash = {start}

	while not open_set.empty():
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()

		current = open_set.get()[2]
		open_set_hash.remove(current)

		if current == end:
			reconstruct_path(came_from, end, draw)
			end.make_end()
			return True

		for neighbor in current.neighbors:
			temp_g_score = g_score[current] + 1

			if temp_g_score < g_score[neighbor]:
				came_from[neighbor] = current
				g_score[neighbor] = temp_g_score
				f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
				if neighbor not in open_set_hash:
					count += 1
					open_set.put((f_score[neighbor], count, neighbor))
					open_set_hash.add(neighbor)
					neighbor.make_open()

		draw()

		if current != start:
			current.make_closed()

	return False


def make_grid(rows, width):
	grid = []
	gap = width // rows
	for i in range(rows):
		grid.append([])
		for j in range(rows):
			spot = Spot(i, j, gap, rows)
			grid[i].append(spot)

	return grid


def draw_grid(win, rows, width):
	gap = width // rows
	for i in range(rows):
		pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
		for j in range(rows):
			pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))


def draw(win, grid, rows, width):
	win.fill(WHITE)

	for row in grid:
		for spot in row:
			spot.draw(win)

	draw_grid(win, rows, width)
	pygame.display.update()


def get_clicked_pos(pos, rows, width):
	gap = width // rows
	y, x = pos

	row = y // gap
	col = x // gap

	return row, col

fire_spots = []

def spread_fire(grid, width, total_rows, start, end, initial_fire):
    global fire_spots
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    if not fire_spots:
        fire_spots.append(initial_fire)

    new_fire_spots = []
    for fire_spot in fire_spots:
        random.shuffle(directions)
        for dx, dy in directions:
            row, col = fire_spot.row + dx, fire_spot.col + dy
            if 0 <= row < total_rows and 0 <= col < total_rows:
                spot = grid[row][col]

                if spot != start and spot != end and not spot.is_barrier() and not spot.is_fire():
                    spot.make_fire()
                    spot.update_neighbors(grid)
                    new_fire_spots.append(spot)
                    break

    fire_spots.extend(new_fire_spots)

def create_floorplan(grid, start_row, start_col, end_row, end_col, fire_row, fire_col):
    # Vertical walls
    for i in range(10, 40):
        grid[i][20].make_barrier()
        grid[i][40].make_barrier()

    # Horizontal walls
    for i in range(20, 41):
        grid[10][i].make_barrier()
        grid[40][i].make_barrier()

    # Gap in the walls
    grid[25][20].reset()
    grid[25][40].reset()

    # Start and end points
    grid[start_row][start_col].make_start()
    grid[end_row][end_col].make_end()

    # Fire point
    grid[fire_row][fire_col].make_fire()

def main(win, width):
    ROWS = 50
    grid = make_grid(ROWS, width)
    start = grid[15][15]
    end = grid[45][45]
    fire_start = grid[30][30]

    create_floorplan(grid, 5, 5, 45, 45, 30, 30)


    run = True

    REEVALUATE_ROUTE_EVENT = pygame.USEREVENT + 1
    pygame.time.set_timer(REEVALUATE_ROUTE_EVENT, 10000)

    SPREAD_FIRE_EVENT = pygame.USEREVENT + 2
    pygame.time.set_timer(SPREAD_FIRE_EVENT, 1000)  # Set fire to spread every 3 seconds

    has_started = False

    while run:
        draw(win, grid, ROWS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if pygame.mouse.get_pressed()[0]:  # LEFT
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]
                if not start and spot != end:
                    start = spot
                    start.make_start()

                elif not end and spot != start:
                    end = spot
                    end.make_end()

                elif spot != end and spot != start:
                    spot.make_barrier()

            elif pygame.mouse.get_pressed()[2]:  # RIGHT
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]
                spot.reset()
                if spot == start:
                    start = None
                elif spot == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)

                    algorithm(lambda: draw(win, grid, ROWS, width), grid, start, end)
                    has_started = True

                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
                    has_started = False

            if has_started and event.type == REEVALUATE_ROUTE_EVENT and start and end:
                for row in grid:
                    for spot in row:
                        spot.update_neighbors(grid)

                success = algorithm(lambda: draw(win, grid, ROWS, width),
                                    grid, start, end)
                if not success:  # If the path is blocked, reset the path and start/end points
                    start, end = None, None
                    for row in grid:
                        for spot in row:
                            if spot.is_path() or spot.is_start() or spot.is_end():
                                spot.reset()

            if event.type == SPREAD_FIRE_EVENT:
                spread_fire(grid, width, ROWS, start, end, fire_start)

        time.sleep(0.001)

    pygame.quit()
main(WIN, WIDTH)
