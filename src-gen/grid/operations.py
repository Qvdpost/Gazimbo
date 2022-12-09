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

        cell = self.parent.maze.grid[cur[0]][cur[1]]
        northWall = cell.walls[0]
        eastWall  = cell.walls[1]
        southWall = cell.walls[2]
        westWall  = cell.walls[3]

        # print('[', cur[0], cur[1], '] [', northWall, eastWall, southWall, westWall,']')

        possible_paths = []
        for w, f in [(northWall, self.getNorth), (eastWall, self.getEast), (southWall, self.getSouth), (westWall, self.getWest)]:
            nxt = f(cur)
            if (w == 0 and nxt not in visited and ((0 <= nxt[0] <= 3) and (0 <= nxt[1] <= 3))):
                if nxt == target:
                    return visited + [nxt]
                possible_paths.append(self.search(nxt, target, visited))

        possible_paths = list (filter(lambda x: x != None, possible_paths))
        # print(possible_paths)

        if len(possible_paths) > 0:
            return sorted(possible_paths, key=lambda x: len(x))[0]
        return None

    def compute_path(self, targetX, targetY):
        curX = self.parent.sm.grid.column
        curY = self.parent.sm.grid.row

        # front = cell.walls[parent.sm.grid.orientation]
        # right = cell.walls[(parent.sm.grid.orientation + 1) % 4]
        # left  = cell.walls[(parent.sm.grid.orientation - 1) % 4]
        # back  = cell.walls[(parent.sm.grid.orientation - 2) % 4]

        return self.search([curX, curY], [targetX, targetY], [])
