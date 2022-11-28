from sokoPuzzle import SokoPuzzle
from node import Node
from collections import deque
import itertools
from copy import deepcopy
import numpy as np
import pygame
import time
import os
import sys

class Search:

    """ Uninformed/Blind Search """
    @staticmethod
    def breadthFirst(initial_node):
        
        # Check if the start element is the goal
        if initial_node.state.isGoal(Node.wall_space_obstacle):
            return initial_node, 0
        elif initial_node.state.isDead(Node.deadlock_map):
                 return None, -1

        # Create the OPEN FIFO queue and the CLOSED list
        open = deque([initial_node]) # A FIFO queue
        closed = list()
       
        step = 0
        while True:

            #print (f'*** Step {step} ***')
            step +=1
            
            # Check if the OPEN queue is empty => goal not found 
            if len(open) == 0:
                return None, -1
            
            # Get the first element of the OPEN queue
            current = open.popleft()
            
            # Put the current node in the CLOSED list
            closed.append(current)

            # Generate the successors of the current node
            succ = current.succ()
            while len(succ) != 0:
                child = succ.popleft()

                # Check if the child is not in the OPEN queue and the CLOSED list
                if (child.state.robot_block not in [n.state.robot_block for n in closed] and \
                    child.state.robot_block not in [n.state.robot_block for n in open]):
                       

                    # Check if the child is the goal
                    if child.state.isGoal(Node.wall_space_obstacle):
                        print (f'*** Step {step} ***')
                        print ("Goal reached")
                        return child, step   
                    elif child.state.isDead(Node.deadlock_map):
                         continue

                    # Put the child in the OPEN queue 
                    open.append(child) 
    """ Informed Search """                       
    @staticmethod
    def A(initial_node, heuristic=1):
        
        # Check if the start element is the goal
        if initial_node.state.isGoal(Node.wall_space_obstacle):
            return initial_node, 0
        elif initial_node.state.isDead(Node.deadlock_map):
            return None, -1

        # Evaluate the cost f for the initial node
        initial_node.F_Evaluation(heuristic)

        # Create the OPEN queue with the initial node as the first element
        open = deque([initial_node])

        # Create the CLOSED list
        closed = list()
        
        step = 0
        heur=[]
        while True:

            step +=1
            print (f'*** Step {step} ***')            
            
            # Check if the OPEN queue is empty => goal not found 
            if len(open) == 0:
                return None, -1
            
            # Get the first element of the OPEN queue after sorting
            open = deque(sorted(list(open), key=lambda node: node.f))
            current = open.popleft()
            
            # Put the current node in the CLOSED list
            closed.append(current)
            
            # Check if the current state is a goal
            if current.state.isGoal(Node.wall_space_obstacle):
                print ("Goal reached")
                print(current.f) 
                return current, step ,heur
            elif current.state.isDead(Node.deadlock_map):
                    closed.remove(current)
                    continue
            heur.append(current.f)
            # Generate the successors of the current node
            succ = current.succ()
            while len(succ) != 0:
                # Pop a child node from the list of successors 
                child = succ.popleft()
                # Evaluate the cost f for this child node
                child.F_Evaluation(heuristic)
                
                # If the child is in the OPEN queue
                if child.state.robot_block in [n.state.robot_block for n in open]:
                    # Get the index of the child in the OPEN queue
                    index = [n.state.robot_block for n in open].index(child.state.robot_block)
                    # Replace the node in the OPEN queue by the new one if its cost f is less than the old one
                    if open[index].f > child.f:
                        # Remove the old node from the OPEN queue
                        open.remove(open[index])
                        # Put the new node with the minimal cost f in the OPEN queue 
                        open.append(child) 

                # If the child is not in the OPEN queue    
                else:
                    # If the child is not in the CLOSED list
                    if child.state.robot_block not in [n.state.robot_block for n in closed]:
                        # Put the child in the OPEN queue 
                        open.append(child)  

                    # If the child is in the CLOSED list
                    else:
                        # Get the index of the child in the CLOSED list
                        index = [n.state.robot_block for n in closed].index(child.state.robot_block)
                        # Remove the node from the CLOSED list and add the new one in the OPEN queue if its cost f is less than the old one
                        if closed[index].f > child.f:
                            # Remove the child from the CLOSED list
                            closed.remove(closed[index])
                            # Put the child in the OPEN queue 
                            open.append(child)      

"""@staticmethod
    def A(initial_node, heuristic=1):
        
        # Check if the initial node is the goal
        if initial_node.state.isGoal(Node.wall_space_obstacle):
            return initial_node, 0
        elif initial_node.state.isDead(Node.deadlock_map):
            return None, -1
        # Evaluate the cost f for the initial node
        initial_node.F_Evaluation(heuristic)

        # Create the OPEN list with the initial node as the first element
        open = [initial_node]

        # Create the CLOSED list
        closed = list()
        
        step = 0
        while True:

            step +=1
            #print (f'*** Step {step} ***')            
            
            # Check if the OPEN list is empty => goal not found 
            if len(open) == 0:
                return None, -1
            
            # Get the index of the node with least f in the OPEN list 
            min_index, _ = min(enumerate(open), key=lambda element: element[1].f)            
            current = open[min_index]

            # Remove the current node (i.e. the node with least f) from the OPEN list
            open.remove(current)
            
            # Put the current node in the CLOSED list
            closed.append(current)
            
            # Check if the current state is a goal
            if current.state.isGoal(Node.wall_space_obstacle):
                print ("Goal reached") 
                return current, step 
            elif current.state.isDead(Node.deadlock_map):
                    closed.remove(current)
                    continue
            # Generate the successors of the current node
            succ = current.succ()
            while len(succ) != 0:
                # Pop a child node from the list of successors 
                child = succ.popleft()
                # Evaluate the cost f for this child node
                child.F_Evaluation(heuristic)
                
                # If the child is in the OPEN list
                if child.state.robot_block in [n.state.robot_block for n in open]:
                    # Get the index of the child in the OPEN list
                    index = [n.state.robot_block for n in open].index(child.state.robot_block)
                    # Replace the node in the OPEN list by the new one if its cost f is less than the old one
                    if open[index].f > child.f: 
                        # Remove the old node from the OPEN list
                        open.remove(open[index])
                        # Put the new node with the minimal cost f in the OPEN list 
                        open.append(child) 

                # If the child is not in the OPEN list    
                else:
                    # If the child is not in the CLOSED list
                    if child.state.robot_block not in [n.state.robot_block for n in closed]:
                        # Put the child in the OPEN list 
                        open.append(child)  

                    # If the child is in the CLOSED list
                    else:
                        # Get the index of the child in the CLOSED list
                        index = [n.state.robot_block for n in closed].index(child.state.robot_block)
                        # Remove the node from the CLOSED list and add the new one in the OPEN list if its cost f is less than the old one
                        if closed[index].f > child.f:
                            # Remove the child from the CLOSED list
                            closed.remove(closed[index])
                            # Put the child in the OPEN list 
                            open.append(child)""" 

""" ***************************************************** Main function **************************************************** """

board1 = [['O', 'O', 'O', 'O', 'O', 'O'],
        ['O', 'S', ' ', 'B', ' ', 'O'],
        ['O', ' ', 'O', 'R', ' ', 'O'],
        ['O', ' ', ' ', ' ', ' ', 'O'],
        ['O', ' ', ' ', ' ', ' ', 'O'],
        ['O', 'O', 'O', 'O', 'O', 'O']]

board2 = [['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O', 'O'],
        ['O', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'O'],
        ['O', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'O'],
        ['O', ' ', ' ', 'O', 'O', 'O', ' ', ' ', 'O'],
        ['O', ' ', ' ', ' ', ' ', 'O', '.', ' ', 'O'],
        ['O', ' ', ' ', ' ', ' ', ' ', 'O', ' ', 'O'],
        ['O', ' ', ' ', 'B', ' ', ' ', 'O', ' ', 'O'],
        ['O', ' ', ' ', ' ', ' ', ' ', 'O', ' ', 'O'],
        ['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O', 'O']]

board3 = [['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O'],
        ['O', ' ', ' ', ' ', 'O', ' ', ' ', 'O'],
        ['O', ' ', ' ', 'B', 'R', ' ', ' ', 'O'],
        ['O', ' ', ' ', ' ', 'O', 'B', ' ', 'O'],
        ['O', 'O', 'O', 'O', 'O', ' ', 'S', 'O'],
        ['O', 'O', 'O', 'O', 'O', ' ', 'S', 'O'],
        ['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O']]

board4 = [['O', 'O', 'O', 'O', 'O', 'O', 'O'],
        ['O', 'O', ' ', ' ', 'O', 'O', 'O'],
        ['O', 'O', ' ', ' ', 'O', 'O', 'O'],
        ['O', 'O', ' ', '*', ' ', ' ', 'O'],
        ['O', 'O', 'B', 'O', 'B', ' ', 'O'],
        ['O', ' ', 'S', 'R', 'S', ' ', 'O'],
        ['O', ' ', ' ', ' ', ' ', 'O', 'O'],
        ['O', 'O', 'O', ' ', ' ', 'O', 'O'],
        ['O', 'O', 'O', 'O', 'O', 'O', 'O']]

board5 = [['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O', 'O'],
        ['O', 'O', 'O', 'S', 'O', ' ', ' ', 'O', 'O'],
        ['O', ' ', ' ', ' ', ' ', 'B', ' ', 'O', 'O'],
        ['O', ' ', 'B', ' ', 'R', ' ', ' ', 'S', 'O'],
        ['O', 'O', 'O', ' ', 'O', ' ', 'O', 'O', 'O'],
        ['O', 'O', 'O', 'B', 'O', ' ', 'O', 'O', 'O'],
        ['O', 'O', 'O', ' ', ' ', 'S', 'O', 'O', 'O'],
        ['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O', 'O']]

board6 = [['O', 'O', 'O', 'O', 'O', 'O', 'O'],
        ['O', 'S', ' ', 'O', ' ', 'R', 'O'],
        ['O', ' ', ' ', 'O', 'B', ' ', 'O'],
        ['O', 'S', ' ', ' ', 'B', ' ', 'O'],
        ['O', ' ', ' ', 'O', 'B', ' ', 'O'],
        ['O', 'S', ' ', 'O', ' ', ' ', 'O'],
        ['O', 'O', 'O', 'O', 'O', 'O', 'O']]

board7 = [['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O'],
        ['O', 'S', 'S', 'S', ' ', 'O', 'O', 'O'],
        ['O', ' ', 'S', ' ', 'B', ' ', ' ', 'O'],
        ['O', ' ', ' ', 'B', 'B', 'B', ' ', 'O'],
        ['O', 'O', 'O', 'O', ' ', ' ', 'R', 'O'],
        ['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O']]

board8 = [['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O'],
        ['O', ' ', ' ', ' ', ' ', 'O', 'O', 'O'],
        ['O', ' ', ' ', ' ', 'B', ' ', ' ', 'O'],
        ['O', 'S', 'S', 'S', '*', 'B', 'R', 'O'],
        ['O', ' ', ' ', ' ', 'B', ' ', ' ', 'O'],
        ['O', ' ', ' ', ' ', 'O', 'O', 'O', 'O'],
        ['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O']]

board9 = [['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O', 'O'],
        ['O', 'O', 'O', 'S', ' ', 'O', 'O', 'O', 'O'],
        ['O', 'O', 'O', ' ', ' ', 'O', 'O', 'O', 'O'],
        ['O', 'S', ' ', 'S', ' ', 'O', 'O', 'O', 'O'],
        ['O', ' ', 'B', ' ', 'B', 'B', ' ', ' ', 'O'],
        ['O', 'O', 'O', 'S', ' ', ' ', 'B', 'R', 'O'],
        ['O', 'O', 'O', ' ', ' ', 'O', 'O', 'O', 'O'],
        ['O', 'O', 'O', 'O', 'O', 'O', 'O', 'O', 'O']]

""" This function will create from a board (a level): a static board (wall_space_obstacle) and a dynamic board (robot_block) 
    The static board will be the same in the whole search process (we will use it just for comparison), 
    so it's better to declare it as a static variable in the class Node 
    This function will also create the initial node"""

def create_initial_node(board=None):        
        
        height = len(board)
        width = len(board[0])
                
        # Separate walls, spaces and obstacles board from robot and boxes board and create a deadlock board
        robot_block = [['']*width for _ in range(height)]
        wall_space_obstacle = [['']*width for _ in range(height)]
        deadlock_map = [['']*width for _ in range(height)]


        for i, j in itertools.product(range(height), range(width)):
            if board[i][j] == 'R':
                robot_position = (i, j) 
                robot_block[i][j] = 'R'
                wall_space_obstacle[i][j] = ' '
            elif board[i][j] == 'B':
                robot_block[i][j] = 'B'
                wall_space_obstacle[i][j] = ' '
            elif board[i][j] == 'S' or board[i][j] == 'O' or board[i][j] == ' ':   
                robot_block[i][j] = ' '   
                wall_space_obstacle[i][j] = board[i][j]         
            elif board[i][j] == '*':
                robot_block[i][j] = 'B'
                wall_space_obstacle[i][j] = 'S'
            else: # self.board[i][j] == '.'
                robot_position = (i, j) 
                robot_block[i][j] = 'R'
                wall_space_obstacle[i][j] = 'S'

        for i, j in itertools.product(range(height), range(width)):
            #verification et creation de la matrice deadlock pour les coins
                if(wall_space_obstacle[i][j] == ' ' and wall_space_obstacle[i][j-1] == 'O' and wall_space_obstacle[i-1][j] == 'O') or (wall_space_obstacle[i][j] == ' ' and wall_space_obstacle[i][j+1] == 'O' and wall_space_obstacle[i-1][j] == 'O') or (wall_space_obstacle[i][j] == ' ' and wall_space_obstacle[i][j-1] == 'O' and wall_space_obstacle[i+1][j] == 'O') or (wall_space_obstacle[i][j] == ' ' and wall_space_obstacle[i][j+1] == 'O' and wall_space_obstacle[i+1][j] == 'O'):
                    deadlock_map[i][j] = 'D'
                else:
                    deadlock_map[i][j] = ' '

        for i, j in itertools.product(range(height), range(width)):
            if deadlock_map[i][j] == 'D':
                if board[i][j-1] == 'O' and board[i-1][j] == 'O': #UpLeft
                    line = False
                    for p in range(j+1, width):
                        if deadlock_map[i][p] == 'D':
                            line = True
                            break
                        elif wall_space_obstacle[i][p] == 'S' or wall_space_obstacle[i-1][p] != 'O':
                            line = False
                            break
                    
                    if line:
                        for k in range(j+1, p+1):
                            deadlock_map[i][k] = 'D'
                    
                    line = False 

                    for p in range(i+1, height):
                        if deadlock_map[p][j] == 'D':
                            line = True
                            break
                        elif wall_space_obstacle[p][j] == 'S' or wall_space_obstacle[p][j-1] != 'O':
                            line = False
                            break
                    if line:
                        for k in range(i+1, p+1):
                            deadlock_map[k][j] = 'D'

                elif board[i][j-1] == 'O' and board[i+1][j] == 'O': #DownLeft
                    line = False
                    for p in range(j+1, width):
                        if deadlock_map[i][p] == 'D':
                            line = True
                            
                            break
                        elif wall_space_obstacle[i][p] == 'S' or wall_space_obstacle[i+1][p] != 'O':
                            line = False
                            break
                    
                    if line:
                        for k in range(j+1, p+1):
                            deadlock_map[i][k] = 'D'
                    
                    line = False 
                    for p in range(i-1, 0, -1):
                        if deadlock_map[p][j] == 'D':
                            line = True
                            break
                        elif wall_space_obstacle[p][j] == 'S' or wall_space_obstacle[p][j-1] != 'O':
                            line = False
                            break
                    
                    if line:
                        for k in range(i-1, p-1, -1):
                            deadlock_map[k][j] = 'D'

                elif board[i][j+1] == 'O' and board[i+1][j] == 'O': #DownRight
                    line = False
                    for p in range(j-1, 0, -1):
                        if deadlock_map[i][p] == 'D':
                            line = True
                            break
                        elif wall_space_obstacle[i][p] == 'S' or wall_space_obstacle[i+1][p] != 'O':
                            line = False
                            break

                    if line:
                        for k in range(j-1, p-1, -1):
                            deadlock_map[i][k] = 'D'
                    
                    line = False
                    for p in range(i-1, 0, -1):
                        if deadlock_map[p][j] == 'D':
                            line = True
                            break
                        elif wall_space_obstacle[p][j] == 'S' or wall_space_obstacle[p][j+1] != 'O':
                            line = False
                            break
                    
                    if line:
                        for k in range(i-1, p-1, -1):
                            deadlock_map[k][j] = 'D'
                
                elif board[i][j+1] == 'O' and board[i-1][j] == 'O': #UpRight
                    line = False
                    for p in range(j-1, 0, -1):
                        if deadlock_map[i][p] == 'D':
                            line = True
                            break
                        elif wall_space_obstacle[i][p] == 'S' or wall_space_obstacle[i-1][p] != 'O':
                            line = False
                            break

                    if line:
                        for k in range(j-1, p-1, -1):
                            deadlock_map[i][k] = 'D'
                    
                    line = False 

                    for p in range(i+1, height):
                        if deadlock_map[p][j] == 'D':
                            line = True
                            break
                        elif wall_space_obstacle[p][j] == 'S' or wall_space_obstacle[p][j+1] != 'O':
                            line = False
                            break
                    if line:
                        for k in range(i+1, p+1):
                            deadlock_map[k][j] = 'D'


        Node.deadlock_map = deadlock_map
        Node.wall_space_obstacle = wall_space_obstacle        
        initial_node = Node(SokoPuzzle(robot_block, robot_position))

        return initial_node

# level = board9
# initial_node = create_initial_node(board=level)
# deadlock_map=initial_node.deadlock_map


#! -------------------------------------------------------  pygame
pygame.init()
pygame.font.init()


WIDTH, HEIGHT = 1000, 800
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("SOKOBAN")

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)


FPS = 1

STEPS_FONT = pygame.font.SysFont('comicsans', 30)

CASE_WIDTH, CASE_HEIGHT = 100, 80


ROBOT_IMAGE = pygame.image.load(
    os.path.join('Assets', 'player1.png'))
ROBOT = pygame.transform.scale(
    ROBOT_IMAGE, (CASE_WIDTH, CASE_HEIGHT))

ROBOT_ON_STORAGE_IMAGE = pygame.image.load(
    os.path.join('Assets', 'player_on_storage.png'))
ROBOT_ON_STORAGE = pygame.transform.scale(
    ROBOT_ON_STORAGE_IMAGE, (CASE_WIDTH, CASE_HEIGHT))

BOX_IMAGE = pygame.image.load(
    os.path.join('Assets', 'box.png'))
BOX = pygame.transform.scale(
    BOX_IMAGE, (CASE_WIDTH, CASE_HEIGHT))

STORED_BOX_IMAGE = pygame.image.load(
    os.path.join('Assets', 'box_in_storage.png'))
STORED_BOX = pygame.transform.scale(
    STORED_BOX_IMAGE, (CASE_WIDTH, CASE_HEIGHT))

STORAGE_IMAGE = pygame.image.load(
    os.path.join('Assets', 'storage1.png'))
STORAGE = pygame.transform.scale(
    STORAGE_IMAGE, (CASE_WIDTH, CASE_HEIGHT))

GROUND_IMAGE = pygame.image.load(
    os.path.join('Assets', 'ground.png'))
GROUND = pygame.transform.scale(
    GROUND_IMAGE, (CASE_WIDTH, CASE_HEIGHT))

GROUND_ALL_IMAGE = pygame.image.load(
    os.path.join('Assets', 'ground.png'))
GROUND_ALL = pygame.transform.scale(
    GROUND_ALL_IMAGE, (WIDTH, HEIGHT))

OBSTACLE_IMAGE = pygame.image.load(
    os.path.join('Assets', 'obstacle.png'))
OBSTACLE = pygame.transform.scale(
    OBSTACLE_IMAGE, (CASE_WIDTH, CASE_HEIGHT))

# PLAY_IMAGE = pygame.image.load(
#     os.path.join('Assets', 'play.png'))
# PLAY = pygame.transform.scale(
#     PLAY_IMAGE, (CASE_WIDTH - 20, CASE_HEIGHT - 20))

# AFTER_IMAGE = pygame.image.load(
#     os.path.join('Assets', 'after.png'))
# AFTER = pygame.transform.scale(
#     AFTER_IMAGE, (CASE_WIDTH - 20, CASE_HEIGHT - 20))

# BEFORE = pygame.transform.rotate(pygame.transform.scale(
#     AFTER_IMAGE, (CASE_WIDTH - 20, CASE_HEIGHT - 20)), 180)

# MENU_IMAGE = pygame.image.load(
#     os.path.join('Assets', 'menu.png'))
# MENU = pygame.transform.scale(
#     MENU_IMAGE, (WIDTH, HEIGHT))


def draw_board(board):

    WIN.blit(GROUND_ALL, (0, 0))

    for i in (range(board.shape[0])):
        for j in range(board.shape[1]):
            if board[i][j] == 'R':
                WIN.blit(ROBOT, (i*CASE_WIDTH, j * CASE_HEIGHT))
            elif board[i][j] == 'B':
                WIN.blit(BOX, (i*CASE_WIDTH, j * CASE_HEIGHT))
            elif board[i][j] == 'S':
                WIN.blit(STORAGE, (i*CASE_WIDTH, j * CASE_HEIGHT))
            elif board[i][j] == 'O':
                WIN.blit(OBSTACLE, (i*CASE_WIDTH, j * CASE_HEIGHT))
            elif board[i][j] == '.':
                WIN.blit(ROBOT_ON_STORAGE, (i*CASE_WIDTH, j * CASE_HEIGHT))
            elif board[i][j] == '*':
                WIN.blit(STORED_BOX, (i*CASE_WIDTH, j * CASE_HEIGHT))
            elif board[i][j] == ' ':
                WIN.blit(GROUND, (i*CASE_WIDTH, j * CASE_HEIGHT))

    # WIN.blit(PLAY, (WIDTH//2 - 200, HEIGHT - 70))
    # WIN.blit(BEFORE, (WIDTH//2 - 300, HEIGHT - 70))
    # WIN.blit(AFTER, (WIDTH//2 -100 , HEIGHT - 70))

    pygame.display.update()


def draw_steps(text):
    # title_text = STEPS_FONT.render(, 1, WHITE)
    steps_text = STEPS_FONT.render("steps :" + text, 1, WHITE)
    WIN.blit(steps_text,  (WIDTH/2 - steps_text.get_width() /
                         2, HEIGHT - 50 - steps_text.get_height()/2))
    # WIN.blit(title_text, (WIDTH - title_text.get_width() - 10/
    #                      2, HEIGHT/2 - title_text.get_height()/2 -50))
    
    pygame.display.update()


def main():

    level = board5  # TODO : add a level selection
    level = np.array(level)
    # while select == False:

    #     mx, my = pygame.mouse.get_pos()
    #     button_1 = pygame.Rect(50, 100, 200, 50)
    #     button_2 = pygame.Rect(50, 200, 200, 50)
    #     if button_1.collidepoint((mx, my)):
    #         if click:
    #             select = True
    #     if button_2.collidepoint((mx, my)):
    #         if click:
    #             select = True
    #     pygame.draw.rect(WIN, (255, 0, 0), button_1)
    #     pygame.draw.rect(WIN, (255, 0, 0), button_2)

    #     click = False
    #     for event in pygame.event.get():
    #         if event.type == pygame.QUIT:
    #             pygame.quit()
    #             sys.exit()
    #         if event.type == pygame.KEYDOWN:
    #             if event.key == pygame.K_ESCAPE:
    #                 pygame.quit()
    #                 sys.exit()
    #         if event.type == pygame.MOUSEBUTTONDOWN:
    #             if event.button == 1:
    #                 click = True

    draw_board(level)

    initial_node = create_initial_node(board=level)

    goalNode, num_steps, heur = Search.A(initial_node, heuristic=3)
    if goalNode:
        print(f"Optimal Solution found after {num_steps} steps")
        solution = goalNode.getSolution()
    else:
        print("Optimal solution not found")

    clock = pygame.time.Clock()
    run = True
    while run:
        clock.tick(FPS)

    
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                pygame.quit()
                sys.exit()

        for i in range(len(solution)): 
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    run = False
                    pygame.quit()
                    sys.exit()
                # if event.type == pygame.K_p:
                #     paused = True
                #     while paused:
                #         if event.type == pygame.K_s:
                #             paused = False
            
            
            draw_board(np.array(solution[i]))
            number_steps_text = str(num_steps)
            draw_steps(number_steps_text)
            time.sleep(1) 

    main()


if __name__ == "__main__":
    main()

