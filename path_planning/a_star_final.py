import math
#for python3 replace raw_input() with input()
max_velx = 3
min_velx = -3
max_vely = 3
min_vely = -3

class coordinates:

    def __init__(self, x, y, vx, vy):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        

class Node:

    def __init__(self, x, y, vx, vy, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind
        self.vx = vx
        self.vy = vy

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


def revise_obstacle_coordinates(ox, oy):
    print("Enter Obstacle coordinates in the form (x,y)\n" + "Enter -1 -1 to terminate")
    s = raw_input().split()
    a = int(s[0])
    b = int(s[1])
    while a!=-1 and b!=-1:
        ox.append(a)
        oy.append(b)
        s = raw_input().split()
        a = int(s[0])
        b = int(s[1])
    return ox,oy


def path_planning(x_in, y_in, vx, vy, gx, gy, vgx, vgy, ox, oy):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    vx:starting x velocity
    vy:starting y velocity
    vgx:goal x velocity
    vgy:goal y vleocity
    """
    reso = 1
    rr = 1
    print("Starting the process")
    ares_main=0
    rx=x_in
    ry=y_in

    print("My velocity in x direction is: " + str(vx) + " and in y direction is: " + str(vy))
    while 1:
        ares_main=ares_main+1
        ggkk = coordinates(67, 7, 1, 1)
        ggkk = a_star_planning(rx, ry, vx, vy, gx, gy, vgx, vgy, ox, oy, reso, rr)

        rx = ggkk.x
        ry = ggkk.y
        print("now i am here"+ str(rx)+str(ry))
        if rx==gx and ry==gy and ggkk.vx == vgx and ggkk.vy == vgy:
            break
        '''
        Suppose we also get feedback corresponding to the location of the rover as raw_input
        then we can do incorporate that as wellself.
        if bool variable feed_loc is true then the revised location of rover will be used
        for path path_planning
        '''
        ox,oy = revise_obstacle_coordinates(ox,oy)
        print("Enter 1 for location feedback")
        feed_loc = int(raw_input())
        if feed_loc is 1:
            print("Enter new coordinates")
            d = raw_input().split()
            rx = int(d[0])
            ry = int(d[1])


    print("Yeah! We reached the target")


def a_star_planning(sx, sy, vx, vy, gx, gy, vgx, vgy, ox, oy, reso, rr):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    rx: rover x-co-ordinates
    ry : rover y-co-ordinate
    vx:starting x velocity
    vy:starting y velocity
    vgx:goal x velocity
    vgy:goal y vleocity
    """

    nstart = Node(round(sx / reso), round(sy / reso), vx, vy, 0.0, -1)
    ngoal = Node(round(gx / reso), round(gy / reso), vgx, vgy, 0.0, -1)
    #ox = [iox / reso for iox in ox]
    #oy = [ioy / reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart

    testing = 0
    finalx = 0
    finaly = 0
    finalvx = 0
    finalvy = 0

    while testing < 2:

        #print(len(openset))
        c_id = min(
        openset, key=lambda o: openset[o].cost + calc_heuristic(ngoal, openset[o]))
        current = openset[c_id]
        print(str(current.x) + "," + str(current.y) )

        finalx = current.x
        finaly = current.y
        finalvx = current.vx
        finalvy = current.vy

        print(finalvx)
        print(finalvy)

        if current.x == ngoal.x and current.y == ngoal.y and current.vx == ngoal.vx and current.vy == ngoal.vy:
            #print("Goal found")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for vi in range(min_velx, max_velx+1):
            for vj in range(min_vely, max_vely+1):
                cost = pow((pow((vi-current.vx),2)+pow((vj-current.vy),2)),0.5)
                node = Node(current.x + vi,
                            current.y + vj,
                            vi , vj, cost, c_id)
                n_id = calc_index(node, xw, minx, miny)

                if n_id in closedset:
                    continue
                if not verify_node(node, obmap, minx, miny, maxx, maxy, current):
                    continue
                if n_id not in openset:
                    openset[n_id] = node  # Discover a new node
                else:
                    if openset[n_id].cost >= node.cost:
                        # This path is the best until now. record it!
                        openset[n_id] = node

        testing += 1

    return coordinates(finalx, finaly, finalvx, finalvy)

def calc_heuristic(n1, n2):
    w = 1.0  # weight of heuristic
    d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
    return d


def verify_node(node, obmap, minx, miny, maxx, maxy, parent):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx or node.y >= maxy:
        return False

    if obmap[int(node.x-minx)][int(node.y-miny)]:
        return False
    if node.vx > max_velx:
        return False
    if node.vx < min_velx:
        return False
    if node.vy > max_vely:
        return False
    if node.vy < min_vely:
        return False

    parentx = parent.x
    parenty = parent.y
    currentx = node.x
    currenty = node.y
    #first check if it is a straight line
    if currentx == parentx:
        smally = min(currenty, parenty)
        bigy = max(currenty, parenty)
        for i in range(int(smally), int(bigy)+1):
            if obmap[int(currentx-minx)][i-int(miny)]:
                return False
    #go to check from small x co-ordinate to bigger x co-ordinate check if the robot if the calculated y point is an integer and then if it is check it there is a obstacle there.
    if currenty == parenty:
        smallx = min(currentx, parentx)
        bigx = max(currentx, parentx)
        for i in range(int(smallx), int(bigx)+1):
            if obmap[int(currentx-minx)][i-int(miny)]:
                return False

    if currenty != parenty and currentx != parentx:
        if currentx > parentx:
            for i in range(int(parentx), int(currentx)+1):
                y = ((int(currentx-parentx)*((i - int(parentx))))/int(currenty-parenty))+parenty
                temp_y = int(y)
                temp_y1 = temp_y + 1
                if (temp_y - y) == 0:
                    if obmap[i-int(minx)][temp_y-int(miny)] or obmap[i-int(minx)][temp_y1-int(miny)]:
                        return False
        elif currentx < parentx:
            for i in range(int(currentx), int(parentx)+1):
                y = ((int(parentx-currentx)*((i - int(currentx))))/int(currenty-parenty))+parenty
                temp_y = int(y)
                temp_y1 = temp_y + 1
                if (temp_y - y) == 0:
                    if obmap[i-int(minx)][temp_y-int(miny)] or obmap[i-int(minx)][temp_y1-int(miny)]:
                        return False

    return True


def calc_obstacle_map(ox, oy, reso, vr):

    minx = 1
    miny = 1
    maxx = 101
    maxy = 101

    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)

    # obstacle map generation
    obmap = [[False for i in range(101)] for i in range(101)]
    for i in range(len(ox)):
        temp1 = int(ox[i] - minx)
        temp2 = int(oy[i] - miny)
        obmap[temp1][temp2] = True

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)



#main

print("Lets start working")
print("Enter starting position coordinates")
d = raw_input().split()
print(d)
x_in = int(d[0])
print(x_in)
y_in = int(d[1])

print(y_in)
print("Enter goal coordinates")
d = raw_input().split()
gx = int(d[0])
gy = int(d[1])
print("Enter starting x velocity")
vx = int(raw_input())
print("Enter starting y velocity")
vy = int(raw_input())
print("Enter goal x velocity")
vgx = int(raw_input())
print("Enter goal y velocity")
vgy = int(raw_input())
print("Depth Map updated at starting position?")
depth_map_status = raw_input()
while depth_map_status == 0:
    depth_map_status = raw_input()
ox = []
oy = []
ox, oy = revise_obstacle_coordinates(ox, oy)

path_planning(x_in, y_in, vx, vy, gx, gy, vgx, vgy, ox, oy)