import math
import heapq
import time
from PIL import Image

heightmap = None

# how many km is the map across
mapsize = 28

# total km range in heights on the map image
heightscale = 0.27432

# extra multiplier to penalize elevation changes even more
# don't take this too high or it can't make a path, but if it's too low it will take a straight shot
elevation_weight = 80

def go():
    global heightmap

    # open the image
    with Image.open("sc.tif") as img:

        # create the final rendering of the terrain
        result = to3DImage(img)

        # load in the heightmap
        heightmap = Heightmap(img)

        # perform the computation
        start = time.time()
        path = astar(heightmap, Node(heightmap.width, 78*4, 179*4), Node(heightmap.width, 181*4, 87*4))
        end = time.time()

        # open up result pixels for writing
        pixels = result.load()

        # draw the path
        for c in path:
            coords = idToCoords(heightmap.width, c)

            # draw with a 3x3 kernel
            for i in range(coords[0] - 1, coords[0] + 2):
                for j in range(coords[1] - 1, coords[1] + 2):
                    pixels[i, j] = (255, 0, 0)

        # show the final image
        result.show()

        # print duration
        print("Took "+str(end-start)+" sec")

# cost function from any start coordinate to any end coordinate
def g(hm, start, end):
    return hm.get3DDistance(start.x, start.y, end.x, end.y)

# trace the path from end to start
def buildPath(origins, current):
    path = [current]
    while current in origins:
        current = origins[current]
        path.insert(0, current)
    return path

def idToCoords(hmw, id):
    return (id % hmw, id // hmw)

# convert the heightmap into a more realistic representation of the terrain
def to3DImage(img):
    result = Image.new('RGB', (img.width, img.height), "black")
    pixels = result.load()
    for y in range(img.height):
        for x in range(0,img.width-1):
            offset = 20*(img.getpixel((x+1,y)) - img.getpixel((x,y)))
            offset = min(offset, 40)
            color = (167 + offset, 197 + offset, 168 + offset)

            pixels[x,y] = color
    return result


def astar(hm, start, goal):

    # create the heap
    heap = []
    heapq.heappush(heap, (g(hm, start, goal), start))

    # stores where each reached cell was reached from
    origins = {}

    # stores g-scores (weight of path from start to current)
    gs = {start.id: 0}

    # stores f-scores (g + estimate weight to end)
    fs = {start.id: g(hm, start, goal)}

    while len(heap) > 0:
        # gets the node with lowest f-score from the heap
        current = heapq.heappop(heap)[1]

        # if reached the goal
        if current.id == goal.id:
            return buildPath(origins, current.id)

        # iterate through the (usually) 8 neighboring cells
        neighbors = current.getNeighbors()
        for neighbor in neighbors:
            # compute new g score
            tg = gs[current.id] + g(hm, current, neighbor)

            # if it's better than the current best g-score or is the first occurrence of that node, record it
            if tg < gs.get(neighbor.id, math.inf):

                # set new origin for the neighbor
                origins[neighbor.id] = current.id

                # write down g-score
                gs[neighbor.id] = tg

                # compute f-score
                fs[neighbor.id] = gs[neighbor.id] + g(hm, neighbor, goal)

                # add it to the heap for processing
                if (fs[neighbor.id], neighbor) not in heap:
                    heapq.heappush(heap, (fs[neighbor.id], neighbor))

class Heightmap:
    def __init__(self, img):
        self.img = img
        self.width = img.width
        self.height = img.height
        self.km_per_pix = mapsize / img.width

    # gets the elevation of a coordinate in pixel units
    def getElevation(self, x, y):
        return (heightscale*self.img.getpixel((x, y))/255) / self.km_per_pix

    # gets the pythagorean distance between 2 coordinates, factoring in elevation as well.
    def get3DDistance(self, x1, y1, x2, y2):
        return (x1-x2) ** 2 + (y1-y2) ** 2 + (elevation_weight*(self.getElevation(x1, y1) - self.getElevation(x2, y2))) ** 2

class Node:
    def __init__(self, hmw, x, y):
        self.hmw = hmw
        self.x = x
        self.y = y
        self.id = y * hmw + x

    def getNeighbors(self):
        neighbors = []

        if self.x > 0:
            neighbors.append(Node(self.hmw, self.x - 1, self.y))

            if self.y > 0:
                neighbors.append(Node(self.hmw, self.x - 1, self.y - 1))

            if self.y < self.hmw - 1:
                neighbors.append(Node(self.hmw, self.x - 1, self.y + 1))

        if self.x < self.hmw - 1:
            neighbors.append(Node(self.hmw, self.x + 1, self.y))

            if self.y > 0:
                neighbors.append(Node(self.hmw, self.x + 1, self.y - 1))

            if self.y < self.hmw - 1:
                neighbors.append(Node(self.hmw, self.x + 1, self.y + 1))

        if self.y > 0:
            neighbors.append(Node(self.hmw, self.x, self.y - 1))

        if self.y < self.hmw - 1:
            neighbors.append(Node(self.hmw, self.x, self.y + 1))

        return neighbors

    def __lt__(self, other):
        return self.id < other.id

if __name__ == "__main__":
    go()
