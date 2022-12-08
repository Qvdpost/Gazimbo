import math

class Callback:
    def __init__(self, parent):
        self.parent = parent
        pass

    def debug(self):
        print('Operation debug has been called by the state machine: pos ({0},{1}): front {2}, right {3}, left {4} ,back {5}'.format(str(self.parent.sm.grid.row), str(self.parent.sm.grid.column), str(self.parent.sm.grid.wall_front), str(self.parent.sm.grid.wall_right), str(self.parent.sm.grid.wall_left), str(self.parent.sm.grid.wall_back)))

    def debug_real(self, val):
        print('Operation debug has been called by the state machine: ' + str(val))

    def abs(self, val):
        return abs(val)

    def compute_path(self):
        targetX = 3
        targetY = 2
        curX = parent.sm.grid.column
        curY = parent.sm.grid.row
        current_node = parent.maze.grid[curX, curY]
        while True:
            wall_front = current_node.walls[parent.sm.grid.orientation]
            wall_right = current_node.walls[(parent.sm.grid.orientation + 1) % 4]
            wall_left  = current_node.walls[(parent.sm.grid.orientation - 1) % 4]
            wall_back  = current_node.walls[(parent.sm.grid.orientation - 2) % 4]




        return math.sqrt(val)