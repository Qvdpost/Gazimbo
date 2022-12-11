import math

class Callback:
    path = {}
    def __init__(self, parent):
        self.parent = parent
        pass

    def debug(self):
        print('Operation debug has been called by the state machine: pos ({0},{1}): front {2}, right {3}, left {4} ,back {5}'.format(str(self.parent.sm.grid.row), str(self.parent.sm.grid.column), str(self.parent.sm.grid.wall_front), str(self.parent.sm.grid.wall_right), str(self.parent.sm.grid.wall_left), str(self.parent.sm.grid.wall_back)))

    def debug_real(self, val):
        print('Operation debug has been called by the state machine: ' + str(val))

    def abs(self, val):
        return abs(val)

    # 0 = north, row-
    def getNorth(self, pos):
        return [pos[0], pos[1]-1]

    # 1 = east,  col+
    def getEast(self, pos):
        return [pos[0]+1, pos[1]]

    # 2 = south, row+
    def getSouth(self, pos):
        return [pos[0], pos[1]+1]

    # 3 = west,  col-
    def getWest(self, pos):
        return [pos[0]-1, pos[1]]

    def search(self, cur, target, visited):
        visited.append(cur)
        if cur == target:
            return visited

        cell = self.parent.maze.grid[cur[1]][cur[0]]
        northWall = cell.walls[0]
        eastWall  = cell.walls[1]
        southWall = cell.walls[2]
        westWall  = cell.walls[3]

        possible_paths = []
        for w, f in [(northWall, self.getNorth), (eastWall, self.getEast), (southWall, self.getSouth), (westWall, self.getWest)]:
            nxt = f(cur)
            if (w == 0 and nxt not in visited and ((0 <= nxt[0] <= 3) and (0 <= nxt[1] <= 3))):
                possible_paths.append(self.search(nxt, target, visited.copy()))

        possible_paths = list (filter(lambda x: x != None, possible_paths))

        if len(possible_paths) > 0:
            return sorted(possible_paths, key=lambda x: len(x))[0]
        return None

    def compute_path(self, targetX, targetY):
        curX = self.parent.sm.grid.column
        curY = self.parent.sm.grid.row

        path = self.search([curX, curY], [targetX, targetY], [])
        if path == None:
            self.path = None
            self.parent.sm.solve.calculated = True
            return

        path_length = len(path)
        path.append([-1, -1])
        for i in range(0,path_length):
            self.path[(path[i][0], path[i][1])] = path[i+1]

        self.parent.sm.solve.calculated = True
        return

    def get_next(self):
        if self.path == None:
            self.parent.sm.solve.target_row = -1
            self.parent.sm.solve.target_col = -1

        curX = self.parent.sm.grid.column
        curY = self.parent.sm.grid.row
        try:
            next_pos = self.path[(curX, curY)]
            self.parent.sm.solve.target_row = next_pos[1]
            self.parent.sm.solve.target_col = next_pos[0]
        except:
            self.parent.sm.solve.recalculate = True
            return

        return
