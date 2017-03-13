#  @author Zachary Glassner

from PIL import Image       #   Image Library
import re                   #   Regular Expressions
from math import sqrt       #   Square Root function
import sys                  #   Commandline args

#   Color constants
OPEN = (248, 148, 18)
ROUGH = (255, 192, 0)
EASY_FOREST = (255,255,255)
RUN_FOREST = (2, 208, 60)
WALK_FOREST = (2, 136, 40)
IMPASSIBLE = (5, 73, 24)
WATER = (0, 0, 255)
ROAD = (71, 51, 3)
FOOTPATH = (0, 0, 0)
OOB = (205, 0, 101)

class Terrain(object):
    """Object to represent different terrain type """
    def __init__(self, color, speed, name):
        self.color = color  #   Color of the terrain in the map image
        self.speed = speed  #   Speed multiplier of traveling this terrain
        self.name = name    #   Name of the terrain type

#   Map of colors to terrain info
terrains = {
    OPEN: Terrain(OPEN, 2.25, 'Open land'),                            #F89412
    ROUGH: Terrain(ROUGH, 0.8, 'Rough meadow'),                        #FFC000
    EASY_FOREST: Terrain(EASY_FOREST, 1.8, 'Easy movement forest'),    #FFFFFF
    RUN_FOREST: Terrain(RUN_FOREST, 1.5, 'Slow run forest'),           #02D03C
    WALK_FOREST: Terrain(WALK_FOREST, 1.3, 'Walk forest'),             #028828
    IMPASSIBLE: Terrain(IMPASSIBLE, 1/100, 'Impassible vegetation'),   #054918
    WATER: Terrain(WATER, 1/1000, 'Lake/Swamp/Marsh'),                   #0000FF
    ROAD: Terrain(ROAD, 3.0, 'Paved road'),                            #473303
    FOOTPATH: Terrain(FOOTPATH, 2.5, 'Footpath'),                     #000000
    OOB: Terrain(OOB, 1/100000, 'Out of bounds')                       #CD0065
    }

'''
  Get the terrain type based on color
  @arg  rgb:    color tuple (r, g, b)
  @return terrain type
'''
def terrain_lookup(rgb):
    #  rgb is a tuple (r, g, b)
    return terrains.get(rgb)


class MapLoc(object):
    """Class to represent location on map"""
    def __init__(self, terrain, location, height = 0):
        self.terrain = terrain      #  Some terrain object
        self.location = location      #  Some (X,Y) tuple
        self.height = height      #  Some height value

    def __repr__(self):
        return str(self.location)

    @classmethod
    def set_height(self, height):
        self.height = height;

class TerrainMap(object):
    """Class to represent the map"""
    def __init__(self, image):
        pixels = image.convert('RGB').load()
        self.width, self.height = image.size
        self.location = []
        for x in range(self.width):
            for y in range(self.height):
                self.location.append(
                    MapLoc(terrain_lookup(pixels[x,y]), (x, y))
                    )
'''
  Converts an (x,y) coordinate into an array index
  @return index
'''
def m2i(h, x, y):
    return h * x + y

'''
  Checks if an (x, y) are in the bounds of a given map
  @retun boolean
'''
def in_bounds(m, x, y):
    w, h = m.width, m.height
    return x >= 0 and x < w and y >= 0 and y < h

'''
  Gets all the neighbors of a given point, (x, y) in m
  @returns a dictionary of neighbors position -> MapLoc
'''
def get_neighbors(m, point):
    x = point[0]
    y = point[1]
    w = m.width
    h = m.height
    index = m2i(w, x, y)
    neighbors = {}
    nx, ny = x - 1, y - 1
    if in_bounds(m, nx, ny):
        neighbors['topL'] = m.location[m2i(h, nx, ny)],
    nx, ny = x, y - 1
    if in_bounds(m, nx, ny):
        neighbors['topC'] = m.location[m2i(h, nx, ny)],
    nx, ny = x + 1, y - 1
    if in_bounds(m, nx, ny):
        neighbors['topR'] = m.location[m2i(h, nx, ny)],
    nx, ny = x - 1, y
    if in_bounds(m, nx, ny):
        neighbors['midL'] = m.location[m2i(h, nx, ny)],
    nx, ny = x, y
    if in_bounds(m, nx, ny):
        neighbors['midC'] = m.location[m2i(h, nx, ny)],
    nx, ny = x + 1, y
    if in_bounds(m, nx, ny):
        neighbors['midR'] = m.location[m2i(h, nx, ny)],
    nx, ny = x - 1, y + 1
    if in_bounds(m, nx, ny):
        neighbors['botL'] = m.location[m2i(h, nx, ny)],
    nx, ny = x, y + 1
    if in_bounds(m, nx, ny):
        neighbors['botC'] = m.location[m2i(h, nx, ny)],
    nx, ny = x + 1, y + 1
    if in_bounds(m, nx, ny):
        neighbors['botR'] = m.location[m2i(h, nx, ny)]
    return neighbors

'''
  Gets the time it takes to travel from one neighboring pixel to another in the map
  @arg  m:       map to use
  @arg  loc1:    location1 (x, y)
  @arg  loc2:    location2 (x, y)
'''
def get_time(m, loc1, loc2):
    # loc1 and loc2 are of type MapLoc
    #   if we are traveling up or down a pixel
    distance = 0
    if loc1[0] == loc2[0]:
        distance = 10.29
    #   if we are traveling left or right a pixel
    elif loc1[1] == loc2[1]:
        distance = 7.55
    #   if we are traveling diagonally a pixel
    else:
        distance = sqrt( (10.29 ** 2) + (7.55 ** 2) )
    terrainType1 = m.location[m2i(m.height, loc1[0], loc1[1])].terrain
    terrainType2 = m.location[m2i(m.height, loc2[0], loc2[1])].terrain
    height1 = m.location[m2i(m.height, loc2[0], loc2[1])].height
    height2 = m.location[m2i(m.height, loc1[0], loc1[1])].height
    heightMult = ((height1 - height2) / 200 ) + 1
    time = heightMult * ( (distance/2) / terrainType1.speed ) + ( (distance/2) / terrainType2.speed )
    # print(loc1, " -> ", loc2, ": ", time)
    return time

'''
  Builds the a searchable map off of the given input files
  @arg  mapImage:       file with map image
  @arg  mapElevation:   file containing all heights
'''
def build_map(mapImage, mapElevation):
    image = Image.open(mapImage)
    terrainMap = TerrainMap(image)
    heights = open(mapElevation, 'r')
    y = 0
    for line in heights:
        elevations = re.findall('\S+', line)
        for i in range(len(elevations) - 6):
                x = terrainMap.height * (i + 6) + y
                terrainMap.location[x].height = float(elevations[i])
        y+=1
    return terrainMap

'''
  Reads a orienteering course file and creates the suitable output
  @arg  m:            map to use
  @arg  course:       course file name
  @arg  output:       filename to output to
  @arg  mapImage:     file name of mapImage
'''
def run_course(m, course, output, mapImage):
    instructions = open(course, 'r')
    i = 0
    cType = ''
    points = []
    timeout = 0
    for line in instructions:
        if i == 0:
            cType = str(line[0:-1])
        elif i == 1 and cType == 'ScoreO':
            timeout = int(line)
        else:
            point = re.findall('\d+', line)
            point = (int(point[0]), int(point[1]))
            points.append(point)
        i+=1

    if cType == 'Classic':
        print('Solving Classic Course')
        path, time = classic_path_finder(m, points)
        print('Time:\t', time)
        plot_path(mapImage, output+'.png', path, points)
    elif cType == 'ScoreO':
        print('Solving ScoreO')
        path, time, points = scoreO_path_finder(m, points, timeout)
        print('Time:\t', time)
        plot_path(mapImage, output+'.png', path, points)
    else:
        print('ERROR:\tInvalid file format.')

'''
  Adds p into the correct position of the priority queue pq
  @arg  pq:       priority queue
  @arg  p:       point to position
  @return the modified pq
'''
def add_in_order(pq, p):
    i = 0
    while i < len(pq) and i != -1:
        if pq[i][0] == p[0]:
            pq.pop(i)
        i+=1
    i = 0
    while i < len(pq) and p[1] >= pq[i][1]:
        i+=1
    pq.insert(i,p)
    return pq

'''
  Performs a A* search from init to goal
  @returns parent states of all visited point
'''
def per_point_a_star(m, init, goal):
    parents = {}
    pq = [(init,0)]
    while len(pq):
        state = pq.pop(0)
        if state[0] == goal:
            return parents, state[1]
        neighbors = get_neighbors(m, state[0])
        for p in neighbors:
            if type(neighbors[p]) == type((1,1)):
                p = neighbors[p][0].location
            else:
                p = neighbors[p].location
            point = (p, get_time(m, state[0], p) + state[1])
            if p not in parents:
                pq = add_in_order(pq, point)
                parents[p] = state
            if parents[p][1] > point[1]:
                pq = add_in_order(pq, point)
                parents[p] = state

'''
  Backtracks through the pathDict to build a path of pixels
  @arg  pathDict:   Map of points to previous state
  @arg  p           starting point
  @return list of points that make path from some initial point to p
'''
def back_track(pathDict, p):
    path = [p]
    while pathDict[p][0] != p:
        path.append(pathDict[p][0])
        p = pathDict[p][0]
    return path

'''
  Finds a path for a classic orienteering course
  @arg  m:       map to find path on
  @arg  points:  list of (x, y) to visit
  @return   list of (x,y) making the path to all points in order of appearance
'''
def classic_path_finder(m, points):
    path = []
    time = 0
    unvisited = points[:]
    init = unvisited.pop()
    while len(unvisited):
        goal = unvisited.pop()
        parents, t = per_point_a_star(m, init, goal)
        time += t
        # print(parents)
        for p in back_track(parents, goal):
                path.append(p)
        init = goal
    return path, time

'''
  Bresenham's algorithm to rasterize a line of pixels
  @arg  p1:     position1
  @arg  p2:     position2
  @return list of pixels making up the line
'''
def line_rasterize(p1, p2):
    pointList = []
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    swap = False

    if dx == 0:
        for y in range(p1[1], p2[1] + 1):
            pointList.append((p1[0], y))
        return pointList

    if dy == 0:
        for x in range(p1[0], p2[0] + 1):
            pointList.append((x, p1[1]))
        return pointList

    if abs(dx) < abs(dy):
        p1 = (p1[1], p1[0])
        p2 = (p2[1], p2[0])
        swap = True

    if p1[0] > p2[0]:
        temp = p1[:]
        p1 = p2[:]
        p2 = temp[:]

    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    er = int(dx/2)
    yStep = 1 if p1[1] < p2[1] else -1
    y = p1[1]

    for x in range(p1[0], p2[0] + 1):
        point = (x, y) if not swap else (y, x)
        pointList.append(point)
        er -= abs(dy)
        if er < 0:
            y += yStep
            er += dx
    return pointList

'''
  Calculates a point's successors based of the number of pixels in the line from originating point to a scucessor and goal
  @arg  m:      map to use
  @arg  s:      state we are starting in
  @arg  goal:   goal position of course
  @arg  points: list of possible points to go to
  @return list of pixels making up the line
'''
def get_time_heuristic(m, s, goal, points):
    succ = []
    p1 = s[0]
    for p2 in points:
        # time = s[1] + sqrt( (10.29 * (p1[0]+ p2[0])** 2) + (7.55 * (p1[1] + p2[1])** 2) )/3
        time = s[1]
        pListS = line_rasterize(p1, p2)
        for pi in range(len(pListS) - 1):
            time += 1
            # time += get_time(m, pListS[pi], pListS[pi + 1])
        if p2 != goal or p1 != goal:
            pListG = line_rasterize(p2, goal)
            for pi in range(len(pListG) - 1):
                time -= 1/len(points)
                # time -= (get_time(m, pListG[pi], pListG[pi + 1])/len(points))
        toVisit = list(points)
        toVisit.pop(toVisit.index(p2))
        succ.append((p2, time, tuple(toVisit)))
    return succ

'''
  Adds p into the correct position of the priority queue pq
  @arg  pq:       priority queue
  @arg  p:       point to position
  @return the modified pq
'''
def add_in_order_scoreO(pq, p):
    i = 0
    while i < len(pq):
        if pq[i][2] == p[2]:
            pq.pop(i)
        i+=1
    i = 0
    while i < len(pq) and p[1] >= pq[i][1]:
        i+=1
    pq.insert(i,p)
    return pq

'''
  Backtracks through the pathDict to build a path of points
  @arg  pathDict:    Map of points to previous state
  @arg  p:           starting point
  @arg  start:       position to end at
  @return list of points that make path from start to start
'''
def back_track_scoreO(pathDict, p, start):
    path = [start, p[0]]
    while pathDict[p[2]][0] != start:
        path.append(pathDict[p[2]][0])
        p = pathDict[p[2]]
    path.append(start)
    return path

def scoreO_path_finder(map, points, timeout, prevTimes={}):
    init = points.pop(0)
    goal = init
    time = 0
    travelPath = []

    # tuple of unvisited points -> parent (point, unvisited)
    parents = {}
    pq = [(init, 0, points)]
    while len(pq):
        state = pq.pop(0)
        if len(state[2]) == 0:
            print("    ...")
            pathOfPoints = back_track_scoreO(parents, state, init)
            i = 0
            while i < len(pathOfPoints) - 1:
                if (pathOfPoints[i], pathOfPoints[i+1]) not in prevTimes:
                    path, t = classic_path_finder(m, [pathOfPoints[i], pathOfPoints[i+1]])
                    prevTimes[(pathOfPoints[i], pathOfPoints[i+1])] = (path, t)
                else:
                    path, t = prevTimes[(pathOfPoints[i], pathOfPoints[i+1])]
                time += t
                for p in path:
                    travelPath.append(p)
                i+=1
                if time > timeout:
                    pathOfPoints.pop(1)
                    pathOfPoints.pop(-1)
                    travelPath, time, pathOfPoints = scoreO_path_finder(m, pathOfPoints, timeout, prevTimes)
            return travelPath, time, pathOfPoints
        succ = get_time_heuristic(m, state, goal, state[2])
        for p in succ:
            if p[2] not in parents:
                pq = add_in_order_scoreO(pq, p)
                parents[p[2]] = state
                pass
            elif parents[p[2]][1] > p[1]:
                pq = add_in_order_scoreO(pq, p)
                parents[p[2]] = state
            pass
        pass

'''
  Draws path over map image
    @arg  inF:    image to draw path on
    @arg  outF:   file to save to
    @arg  path:   list of pixel froming the path
    @arg  points: list of control points
'''
def plot_path(inF, outF, path, points = []):
    image = Image.open(inF)
    for point in path:
        image.putpixel(point, (255, 0, 0))
    for point in points:
        image.putpixel(point, (0,255,255))
    image.putpixel(points[0], (148,0,211))
    image.save(outF)

if __name__ == "__main__":
    if len(sys.argv) != 5:
        print(len(sys.argv))
        print("USAGE:\tpython OrienteeringSearch.py < map_image > < map_heights > < output > < course >")
    else:
        m = build_map(str(sys.argv[1]), str(sys.argv[2]))
        run_course(m, str(sys.argv[4]), str(sys.argv[3]), str(sys.argv[1]))
    exit()
