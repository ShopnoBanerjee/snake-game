import pygame
import random
import time
import sys
from search_algorithms import *

# Parse command-line arguments
if len(sys.argv) != 3:
    print("Usage: python snake.py <level> <search_algorithm>")
    sys.exit(1)

level = sys.argv[1].lower()  # Convert to lowercase
search_algorithm = sys.argv[2].lower()

# Validate level
LEVELS = {"level0": 0, "level1": 5, "level2": 10, "level3": 15}
if level not in LEVELS:
    print("Invalid level! Choose from: level0, level1, level2, level3")
    sys.exit(1)

# Validate search algorithm
ALGORITHMS = {"bfs": bfs, "dfs": dfs, "ucs": ucs, "ids": ids, "a*": astar, "random": random_move, "greedy_bfs": greedy_bfs}
if search_algorithm not in ALGORITHMS:
    print("Invalid search algorithm! Choose from: bfs, dfs, ucs, ids, a*, random, greedy_bfs")
    sys.exit(1)


# Initialize pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 500, 500
CELL_SIZE = 20
WHITE, BLACK, GREEN, RED, GRAY = (255, 255, 255), (0, 0, 0), (0, 255, 0), (255, 0, 0), (128, 128, 128)
FONT = pygame.font.Font(None, 36)

# Grid settings
ROWS, COLS = WIDTH // CELL_SIZE, HEIGHT // CELL_SIZE

# AI Snake starting position
snake_pos = [ROWS // 2, COLS // 2]

# Food position
food_pos = [random.randint(0, ROWS - 1), random.randint(0, COLS - 1)]

# Timer settings
TIME_LIMIT = 30
start_time = time.time()

# Setup Pygame window
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption(f"AI Snake Game ({search_algorithm.upper()} - {level.upper()})")

clock = pygame.time.Clock()

# Score
score = 0  

# Generate obstacles based on level
obstacles = set()
OBSTACLE_COUNT = (ROWS * COLS * LEVELS[level]) // 100  # % of total grid size
while len(obstacles) < OBSTACLE_COUNT:
    obstacle = (random.randint(0, ROWS - 1), random.randint(0, COLS - 1))
    if obstacle != tuple(snake_pos) and obstacle != tuple(food_pos):
        obstacles.add(obstacle)

# Directions (Up, Down, Left, Right)
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]


# Game Over function
def game_over():
  
    # creating font object my_font
    my_font = pygame.font.SysFont('times new roman', 50)
    
    # creating a text surface on which text will be drawn
    game_over_surface = my_font.render(
        'Your Score is : ' + str(score), True, RED)
    
    # create a rectangular object for the text surface object
    game_over_rect = game_over_surface.get_rect()
    
    # setting position of the text
    game_over_rect.midtop = (WIDTH/2, HEIGHT/4)
    
    # blit will draw the text on screen
    screen.blit(game_over_surface, game_over_rect)
    pygame.display.flip()

    print("Score: ", score)

    # after 2 seconds we will quit the program
    time.sleep(1)
    
    # deactivating pygame library
    pygame.quit()
    
    # quit the program
    quit()

# Search Algorithm to find the solution
# def search(start, end, obstacles, ROWS, COLS):
#     path = ALGORITHMS[search_algorithm](tuple(snake_pos), tuple(food_pos), obstacles, ROWS, COLS)
#     return path


running = True
path = []

while running:
    screen.fill(BLACK)

    # Timer logic
    elapsed_time = time.time() - start_time
    time_left = max(0, TIME_LIMIT - int(elapsed_time))

    # If time runs out, end game
    if time_left == 0:
        running = False
        game_over()

    # Find path if no current path
    if not path:
        path = ALGORITHMS[search_algorithm](tuple(snake_pos), tuple(food_pos), obstacles, ROWS, COLS)

    # Move AI Snake
    if path:
        move = path.pop(0)  # Take next move from BFS path
        snake_pos[0] += move[0]
        snake_pos[1] += move[1]

    # Check if AI hits wall or obstacle (Game Over)
    if snake_pos[0] < 0 or snake_pos[0] >= ROWS or snake_pos[1] < 0 or snake_pos[1] >= COLS or tuple(snake_pos) in obstacles:
        running = False
        game_over()

    # Check if AI reaches food (Increase Score, Relocate Food)
    if snake_pos == food_pos:
        score += 1  # Increase score
        food_pos = [random.randint(0, ROWS - 1), random.randint(0, COLS - 1)]
        path = []  # Reset path to calculate new one

    # Draw Snake
    pygame.draw.rect(screen, GREEN, (snake_pos[1] * CELL_SIZE, snake_pos[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    # Draw Food
    pygame.draw.rect(screen, RED, (food_pos[1] * CELL_SIZE, food_pos[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    # Draw Obstacles
    for obs in obstacles:
        pygame.draw.rect(screen, GRAY, (obs[1] * CELL_SIZE, obs[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    # Display Timer
    timer_text = FONT.render(f"Time Left: {time_left}s", True, WHITE)
    screen.blit(timer_text, (20, 20))

    # Display Score
    score_text = FONT.render(f"Score: {score}", True, WHITE)
    screen.blit(score_text, (20, 50))

    pygame.display.update()
    clock.tick(5)  # Control AI speed

    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            game_over()

pygame.quit()

print("Score: ", score)
