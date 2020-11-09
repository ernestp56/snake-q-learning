from gameobjects import GameObject
from move import Move, Direction
import heapq
import random

class SquareGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id):
        return id not in self.walls
    
    def neighbors(self, id):
        (x, y) = id
        results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

class GridWithWeights(SquareGrid):
    def __init__(self, width, height):
        super().__init__(width, height)
        self.weights = {}
    
    def cost(self, from_node, to_node):
        return self.weights.get(to_node, 1)

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]


class Agent:



    def __init__(self):
        self.totalScore = 0
        self.numberGames = 0
        self.path = []
        """" Constructor of the Agent, can be used to set up variables """
    
    def update (self, board, head_position, body_parts, obstacles, goal):
        self.b = GridWithWeights(len(board[0]), len(board[1]))
        path = self.pathfinder(self.b, head_position, goal, body_parts, obstacles)
        return path

    def pathfinder (self, b, head_position, goal, body_parts, obstacles):
        try:
            came_from, cost_so_far = self.a_star_search(b, head_position, goal, body_parts, obstacles)
            path = self.reconstruct_path(came_from, head_position, goal)
        except Exception as e:
            print(e)
            neighbors = b.neighbors(head_position)
            filtered = []
            for i in neighbors:
                if i not in obstacles and i not in body_parts and i not in head_position:
                    filtered.append(i)
            print("EMA HERE")
            print(filtered)
            try:
                smth = random.choice(filtered)
                i = 0
                while i < 5:
                    if smth != GameObject.WALL:
                        return smth
                    else:
                        smth = random.choice(filtered)
                    i = i + 1
                return list(smth)
            except Exception as e:
                return Move.STRAIGHT
            #TODO RANDOM MOVE
        return path

    def heuristic(a, b):
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)


    def a_star_search(self, graph, start, goal, body_parts, obstacles):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        while not frontier.empty():
            current = frontier.get()
            if current == goal:
                break

            for next in graph.neighbors(current):
                if next not in body_parts and next not in obstacles:
                    new_cost = cost_so_far[current] + graph.cost(current, next)
                    if next not in cost_so_far or new_cost < cost_so_far[next]:
                        cost_so_far[next] = new_cost
                        dx1 = current[0] - goal[0]
                        dy1 = current[1] - goal[1]
                        dx2 = start[0] - goal[0]
                        dy2 = start[1] - goal[1]
                        cross = abs(dx1*dy2 - dx2*dy1)
                        priority = new_cost + Agent.heuristic(goal, next) + cross*0.001
                        frontier.put(next, priority)
                        came_from[next] = current
        
        return came_from, cost_so_far

    def reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        # path.append(start) # optional
        path.reverse() # optional
        return path

    def get_move(self, board, score, turns_alive, turns_to_starve, direction, head_position, body_parts):
        """This function behaves as the 'brain' of the snake. You only need to change the code in this function for
        the project. Every turn the agent needs to return a move. This move will be executed by the snake. If this
        functions fails to return a valid return (see return), the snake will die (as this confuses its tiny brain
        that much that it will explode). The starting direction of the snake will be North.

        :param board: A two dimensional array representing the current state of the board. The upper left most
        coordinate is equal to (0,0) and each coordinate (x,y) can be accessed by executing board[x][y]. At each
        coordinate a GameObject is present. This can be either GameObject.EMPTY (meaning there is nothing at the
        given coordinate), GameObject.FOOD (meaning there is food at the given coordinate), GameObject.WALL (meaning
        there is a wall at the given coordinate. TIP: do not run into them), GameObject.SNAKE_HEAD (meaning the head
        of the snake is located there) and GameObject.SNAKE_BODY (meaning there is a body part of the snake there.
        TIP: also, do not run into these). The snake will also die when it tries to escape the board (moving out of
        the boundaries of the array)

        :param score: The current score as an integer. Whenever the snake eats, the score will be increased by one.
        When the snake tragically dies (i.e. by running its head into a wall) the score will be reset. In ohter
        words, the score describes the score of the current (alive) worm.

        :param turns_alive: The number of turns (as integer) the current snake is alive.

        :param turns_to_starve: The number of turns left alive (as integer) if the snake does not eat. If this number
        reaches 1 and there is not eaten the next turn, the snake dies. If the value is equal to -1, then the option
        is not enabled and the snake can not starve.

        :param direction: The direction the snake is currently facing. This can be either Direction.NORTH,
        Direction.SOUTH, Direction.WEST, Direction.EAST. For instance, when the snake is facing east and a move
        straight is returned, the snake wil move one cell to the right.

        :param head_position: (x,y) of the head of the snake. The following should always hold: board[head_position[
        0]][head_position[1]] == GameObject.SNAKE_HEAD.

        :param body_parts: the array of the locations of the body parts of the snake. The last element of this array
        represents the tail and the first element represents the body part directly following the head of the snake.

        :return: The move of the snake. This can be either Move.LEFT (meaning going left), Move.STRAIGHT (meaning
        going straight ahead) and Move.RIGHT (meaning going right). The moves are made from the viewpoint of the
        snake. This means the snake keeps track of the direction it is facing (North, South, West and East).
        Move.LEFT and Move.RIGHT changes the direction of the snake. In example, if the snake is facing north and the
        move left is made, the snake will go one block to the left and change its direction to west.
        """
        obstacles = self.findObstacles(board)
        goal = self.findFood(board)
        print(self.path)
        if len(self.path) == 0:
            self.path = self.update(board, head_position, body_parts, obstacles, goal)
            print("Path:")
            print(self.path)
        try:
            if isinstance(self.path, tuple):
                self.path = list(self.path)
            if self.path[0] not in obstacles and self.path[0] not in body_parts:
                if len(self.path) > 1 or self.path[0] == goal:
                    next = self.path[0]
                    self.path.remove(self.path[0])
                    print("cokolwiek")
                elif self.path != goal:
                    print("tttt")
                    print(self.path)
                    next = self.path
            else:
                if head_position[1] > next[1]:
                    return Move.STRAIGHT
                elif board[head_position[0]+1][head_position[1]] != GameObject.WALL and board[head_position[0]+1][head_position[1]] != GameObject.SNAKE_BODY:
                    return Move.RIGHT
                elif board[head_position[0]-1][head_position[1]] != GameObject.WALL and board[head_position[0]-1][head_position[1]] != GameObject.SNAKE_BODY:
                    return Move.LEFT
                if head_position[0] < next[0]:
                    return Move.RIGHT
            if direction == Direction.NORTH:
                if head_position[0] > next[0]:
                    return Move.LEFT
                if head_position[0] == next[0]:
                    if head_position[1] > next[1]:
                        return Move.STRAIGHT
                    elif board[head_position[0]+1][head_position[1]] != GameObject.WALL \
                            and board[head_position[0]+1][head_position[1]] != GameObject.SNAKE_BODY:
                        return Move.RIGHT
                    elif board[head_position[0]-1][head_position[1]] != GameObject.WALL \
                            and board[head_position[0]-1][head_position[1]] != GameObject.SNAKE_BODY:
                        return Move.LEFT
                    elif next not in obstacles:
                        return Move.STRAIGHT
                if head_position[0] < next[0]:
                    return Move.RIGHT

            if direction == Direction.EAST:
                if head_position[0] > next[0]: #and next not in obstacles:
                    return Move.STRAIGHT
                if head_position[0] == next[0]:
                    if head_position[1] < next[1]:
                        return Move.RIGHT
                    elif head_position[1] > next[1]:
                        return Move.LEFT

            if direction == Direction.SOUTH:
                if head_position[0] > next[0]:
                    return Move.RIGHT
                if head_position[0] == next[0]:
                    if head_position[1] < next[1]: #and next not in obstacles:
                        return Move.STRAIGHT
                    elif board[head_position[0]+1][head_position[1]] != GameObject.WALL \
                            and board[head_position[0]+1][head_position[1]] != GameObject.SNAKE_BODY:
                        return Move.LEFT
                    elif board[head_position[0]-1][head_position[1]] != GameObject.WALL \
                            and board[head_position[0]-1][head_position[1]]!= GameObject.SNAKE_BODY:
                        return Move.RIGHT
                    elif next not in obstacles:
                        return Move.STRAIGHT
                if head_position[0] < next[0]:
                    return Move.LEFT

            if direction == Direction.WEST:
                if head_position[0] < next[0]: #and next not in obstacles:
                    return Move.STRAIGHT
                if head_position[0] == next[0]:
                    if head_position[1] > next[1]:
                        return Move.RIGHT
                    elif head_position[1] < next[1]:
                        return Move.LEFT
            


            if len(self.path) < 1:
                goal = self.findFood(board)
                path = self.update(board, head_position, body_parts, obstacles, goal)
            return Move.STRAIGHT
        except Exception as e:
            # print(e)
            neighbors = self.b.neighbors(head_position)
            filtered = []
            for i in neighbors:
                if i not in obstacles and i not in body_parts and i not in head_position:
                    filtered.append(i)
            try:
                smth = random.choice(filtered)
                print(smth)
                i = 0
                while i < 5:
                    if smth != GameObject.WALL:
                        return smth
                    else:
                        smth = random.choice(filtered)
                    i = i + 1
                return list(smth)
            except Exception as e:
                print("tylko nie to")
                return Move.LEFT
        print("tylko nie to v2")
        return Move.STRAIGHT
        
        

    
    def findFood (self, board):
        for i in range(len(board[0])):
            for j in range(len(board[1])):
                if board[i][j] == GameObject.FOOD:
                    return (i,j)

    def findObstacles (self, board):
        obstacles = []
        for i in range(len(board[0])):
            for j in range(len(board[1])):
                if board[i][j] == GameObject.WALL:
                    obstacles.append(tuple([i,j]))
        return obstacles


    def should_redraw_board(self):
        """
        This function indicates whether the board should be redrawn. Not drawing to the board increases the number of
        games that can be played in a given time. This is especially useful if you want to train you agent. The
        function is called before the get_move function.

        :return: True if the board should be redrawn, False if the board should not be redrawn.
        """
        return True

    def should_grow_on_food_collision(self):
        """
        This function indicates whether the snake should grow when colliding with a food object. This function is
        called whenever the snake collides with a food block.

        :return: True if the snake should grow, False if the snake should not grow
        """
        return True

    def on_die(self, head_position, board, score, body_parts):
        self.totalScore += score
        self.numberGames += 1
        avg = self.totalScore/self.numberGames
        print("Score this time:" + str(score))
        print("Number of Games:" + str(self.numberGames))
        print("Avg so far:" + str(avg))

        """This function will be called whenever the snake dies. After its dead the snake will be reincarnated into a
        new snake and its life will start over. This means that the next time the get_move function is called,
        it will be called for a fresh snake. Use this function to clean up variables specific to the life of a single
        snake or to host a funeral.

        :param head_position: (x, y) position of the head at the moment of dying.

        :param board: two dimensional array representing the board of the game at the moment of dying. The board
        given does not include information about the snake, only the food position(s) and wall(s) are listed.

        :param score: score at the moment of dying.

        :param body_parts: the array of the locations of the body parts of the snake. The last element of this array
        represents the tail and the first element represents the body part directly following the head of the snake.
        When the snake runs in its own body the following holds: head_position in body_parts.
        """
