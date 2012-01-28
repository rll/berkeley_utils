import math
import random
from numpy import *
EPSILON = 0.001
def floatEquals(x,y):
    return math.fabs(x-y) <= EPSILON

class Shape:
    def __init__(self):
        #do nothing
        return None
    def contains(self,point):
        abstract
        
    def translate(self,dx,dy):
        abstract

    def dupl(self):
        abstract
    
    def isPolygon(self):
        return False


## --- Points --- ##
class Point:
    def __init__(self,x,y):
        self.xval = float(x)
        self.yval = float(y)
        
    def __eq__(self,pt):
        return floatEquals(self.x(),pt.x()) and floatEquals(self.y(), pt.y())
    
    def __str__(self):
        return "(%f,%f)"%(self.x(),self.y())
        
    def toTuple(self):
        return (int(self.xval),int(self.yval))
        
    def x(self):
        return self.xval
    
    def y(self):
        return self.yval
        
    def translate(self,dx,dy):
        self.xval += dx
        self.yval += dy

    def dupl(self):
        return Point(self.xval,self.yval)

## --- Lines --- ##
class Line(Shape):
        
    def __init__(self,pt1,pt2):
        self.pt1 = pt1
        self.pt2 = pt2
        
    def __eq__(self,line):
        [a1,b1,c1] = self.standardForm()
        [a2,b2,c2] = line.standardForm()
        if c1 != 0:
            a1 /= c1
            b1 /= c1
            if c2 == 0:
                return False
            a2 /= c2
            b2 /= c2
            c1 = 1
            c2 = 1
        elif b1 != 0:
            a1 /= b1
            b1 = 1
            if b2 == 0:
                return False
            a2 /= b2
            b2 = 1
        else:
            return True
        if not floatEquals(a1,a2):
            return False
        if not floatEquals(b1,b2):
            return False
        return True
        
    def directed(self):
        return False
    
    def a(self):
        return -1*(self.pt2.y() - self.pt1.y())
    
    def b(self):
        return self.pt2.x() - self.pt1.x()
        
    def c(self):
        return self.a()*self.pt1.x() + self.b()*self.pt1.y()
        
    def standardForm(self):
        return [self.a(), self.b(), self.c()]
        
    def ptSlopeForm(self):
        m = -1 * self.a()/self.b()
        b = self.c() / self.b()
        return [m, b]
    
    def contains(self,pt):
        if floatEquals(self.a()*pt.x() + self.b()*pt.y(),self.c()):
            return True
        else:
            return False
            
    def translate(self,dx,dy):
        self.pt1.translate(dx,dy)
        self.pt2.translate(dx,dy)
        
    def isSegment(self):
        return False
        
            
    def __str__(self):
        return "%fx + %fy = %f"%(self.a(),self.b(),self.c())

    def dupl(self):
        return Line(self.pt1.dupl(),self.pt2.dupl())
        
    def overlaps(self,ln):
        return Line.__eq__(self,ln)
            
class LineSegment(Line):
    def contains(self,pt):
        if(not Line.contains(self,pt)):
            return False
        xbounds = [p.x() for p in self.pts()]
        ybounds = [p.y() for p in self.pts()]
        return min(xbounds)-EPSILON <= pt.x() <= max(xbounds)+EPSILON and min(ybounds)-EPSILON <= pt.y() <= max(ybounds)+EPSILON
            
    def pts(self):
        return [self.pt1,self.pt2]
        
    def isSegment(self):
        return True
    
    def __eq__(self,seg):
        if not Line.__eq__(self,seg):
            return False
        elif self.start() == seg.start() and self.end() == seg.end():
            return True
        elif self.start() == seg.end() and self.end() == seg.start():
            return True
        else:
            return False
    
    def containsExclusive(self,pt):
        if not self.contains(pt):
            return False
        return pt != self.start() and pt != self.end()
        
    def overlaps(self,seg):
        if not Line.overlaps(self,seg):
            return False
        return self.contains(seg.start()) or self.contains(seg.end())
        
    def start(self):
        return self.pt1

    def end(self):
        return self.pt2

    def center(self):
        return Point((self.pt1.x() + self.pt2.x())/2,(self.pt1.y() + self.pt2.y())/2)
        
    def dx(self):
        return self.pt2.x() - self.pt1.x()
    
    def dy(self):
        return self.pt2.y() - self.pt1.y()
        
    def __str__(self):
        return "[%s -- %s]"%(self.start(),self.end())
        
    def length(self):
        if self.pt1 == self.pt2:
            return 0
        return math.sqrt(self.dx()**2 + self.dy()**2)
    
    def flip(self):
        p1 = self.pt1
        p2 = self.pt2
        self.pt1 = p2
        self.pt2 = p1

    def dupl(self):
        return LineSegment(self.pt1.dupl(),self.pt2.dupl())
    
    def extrapolate(self,percent):
        newX = self.end().x() + (percent - 1)*self.dx()
        newY = self.end().y() + (percent - 1)*self.dy()
        return Point(newX,newY)
        
    def expand(self,percent):
        newStart = self.extrapolate(-1*percent)
        newEnd = self.extrapolate(1+percent)
        self.setStart(newStart)
        self.setEnd(newEnd)
        
    def setEnd(self,pt):
        self.pt2 = pt
    
    def setStart(self,pt):
        self.pt1 = pt
        
        
class DirectedLineSegment(LineSegment):
    def directed(self):
        return True
    def isLeftOf(self,pt):
        return self.a()*pt.x() + self.b()*pt.y() > self.c()
        
    def isRightOf(self,pt):
        if ptLineDisplacement(pt,self).length() < 1.0:
            return False
        return self.a()*pt.x() + self.b()*pt.y() < self.c() and not floatEquals(self.a()*pt.x()+ self.b() * pt.y(), self.c())
    
    def __eq__(self,dseg):
        if not LineSegment.__eq__(self,dseg):
            return False
        else:
            return self.start() == dseg.start()
    def __str__(self):
        return "[%s -> %s]"%(self.start(),self.end())

    def dupl(self):
        return DirectedLineSegment(self.pt1.dupl(),self.pt2.dupl())

## --- Polygons --- ##
class Polygon(Shape):
    
    def __init__(self,*pts):
        self.myVertices = []
        for pt in pts:
            self.myVertices.append(pt)
            
            
    def __str__(self):
        output = "{\n"
        for side in self.sides():
            output += "%s\n"%side
        output += "}"
        return output
        
    def isPolygon(self):
        return True 
            
    def vertices(self):
        return self.myVertices
            
    def num_vertices(self):
        return len(self.myVertices)

    def neighboring_vertices(self,vertex):
        """
        takes a vertex and returns its neighbors in order
        """
        neighbors = []
        index = self.myVertices.index(vertex)
        # append preceding vertex
        if(index == 0):
            neighbors.append(self.myVertices[-1])
        else:
            neighbors.append(self.myVertices[index-1])
        
        # append succeeding vertex
        if(index + 1 == self.num_vertices()):
            neighbors.append(self.myVertices[0])
        else:
            neighbors.append(self.myVertices[index+1])

        return neighbors

    def sides(self):
        segments = []
        vertices = self.vertices()
        for i in range(len(vertices)):
            segments.append(LineSegment(vertices[i],vertices[(i+1)%len(vertices)]))
        return segments
        
    def contains(self,pt):
        #Uses raytracing method
        startPt = pt
        endPt = Point(1000000,1000000)
        ray = LineSegment(startPt,endPt)
        sides = self.sides()
        for seg in sides:
            if seg.contains(pt):
                return True
        intersects = []
        for seg in sides:
            inter = intersect(ray,seg)
            if(inter and not inter in intersects):
                intersects.append(inter)
        return len(intersects)%2 == 1

    def containsExclusive(self,pt):
        if not self.contains(pt):
            return False
        for line in self.sides():
            if line.contains(pt):
                return False
        return True
    
    def translate(self,dx,dy):
        for vertex in self.vertices():
            vertex.translate(dx,dy)

    def center(self):
        newx = sum([v.x() for v in self.vertices()]) / len(self.vertices())
        newy = sum([v.y() for v in self.vertices()]) / len(self.vertices())
        return Point(newx,newy)
        
    def randPt(self):
        (xRange,yRange) = self.boundingBox()
        pt = self.center()
        while(not self.containsExclusive(pt)):
            x = random.randint(xRange[0],xRange[-1])
            y = random.randint(yRange[0],yRange[-1])
            pt = Point(x,y)
        return pt
    def boundingBox(self):
        return getBoundingBox(self.vertices())

    def dupl(self):
        return Polygon(*[pt.dupl for pt in self.vertices()])


    def touchesSeg(self,seg):
        inters = [x for x in [intersect(side,seg) for side in self.sides()] if x]
        return len(inters) > 0 or (self.contains(seg.start()) and self.contains(seg.end()))
    
    def convexVertices(self):
        return [pt for pt in self.vertices() if self.isConvexPt(pt)]
        
    def concaveVertices(self):
        return [pt for pt in self.vertices() if not self.isConvexPt(pt)]
        
    def isConvexPt(self,pt):
        i = self.vertices().index(pt)
        prior = self.vertices()[(i-1)%len(self.vertices())]
        post = self.vertices()[(i+1)%len(self.vertices())]
        seg = DirectedLineSegment(prior,post)
        #output = min([ptLineDisplacement(seg.center(),side).length() for side in self.sides()]) > 0.2
        intersects = [intersect(side,seg) for side in self.sides() if intersect(side,seg)]
        if len(intersects) > 0:
            if ptLineDisplacement(pt,seg) > 2:
                return True
        else:
            return False
        toSides = [side for side in self.sides() if side.end() == pt]
        for side in toSides:
            if self.contains(side.extrapolate(1 + 0.1/side.length())):
                return False
        return True


    def adjacentTo(self,poly):
        return len([side for side in self.sides() if len([side2 for side2 in poly.sides() if side.overlaps(side2)])>0]) > 0
            
class Circle(Shape):
    def __init__(self,center,radius):
        self.centerval = center
        self.radiusval = radius
    
    def center(self):
        return self.centerval
    
    def radius(self):
        return self.radiusval
        
    def setRadius(self,val):
        self.radiusval = val
    
    def contains(self,pt):
        return distance(pt,self.center()) <= self.radius()

    def dupl(self):
        return Circle(self.center().dupl,self.radius())
        
## --- Methods --- ##
def ptSum(pt1,pt2):
    xsum = pt1.x()+pt2.x()
    ysum = pt1.y()+pt2.y()
    return Point(xsum,ysum)
    
def ptDiff(pt1,pt2):
    xdiff = pt1.x()-pt2.x()
    ydiff = pt1.y()-pt2.y()
    return Point(xdiff,ydiff)
    
def ptDotProd(pt1,pt2):
    return pt1.x()*pt2.x() + pt1.y()*pt2.y()
    
def ptScale(pt,s):
    xscale = pt.x()*s
    yscale = pt.y()*s
    return Point(xscale,yscale)

def intersect(line1,line2):
    if line1.overlaps(line2):
        if not line1.isSegment() or not line2.isSegment():
            return False
        else:
            shared_pts = [pt for pt in line1.pts() if pt in line2.pts()]
            if len(shared_pts) == 0:
                return False
            else:
                return shared_pts[0]
    [a1, b1, c1] = line1.standardForm()
    [a2, b2, c2] = line2.standardForm()
    pt = intersectStandardForm([a1,b1,c1],[a2,b2,c2])
    if not pt:
        return False
    if(line1.contains(pt) and line2.contains(pt)):
        return pt
    else:
        return False
        
def intersectStandardForm(form1,form2):
    [a1,b1,c1] = form1
    [a2,b2,c2] = form2
    if(a1*b2 - a2*b1 != 0):
            y = (a1*c2 - a2*c1) / (a1*b2 - a2*b1)
    else:
        return False
    if(a1 != 0):
        x = (c1 - b1*y) / a1
    elif(a2 != 0):
        x = (c2 - b2*y) / a2
    else:
        return False
    return Point(x,y)
    
def angleBetweenLines(line_from,line_to):
    a1 = line_from.a()
    a2 = line_to.a()
    b1 = line_from.b()
    b2 = line_from.b()
    angle = arccos((b1*b2+a1*a2)/(sqrt(b1**2+a1**2)*sqrt(b2**2+a2**2)))
    
def safeArctan(num,denom):
    if denom == 0:
        return pi/2 * num/abs(num)
    else:
        return arctan(num/denom)
   
    
    

def mirrorPt(pt,line):
    displ = ptLineDisplacement(pt,line)
    return ptDiff(ptScale(displ.end(),2), displ.start())
    
def closestPtOnLine(pt,line):
    return ptLineDisplacement(pt,line).end()
    
def ptLineDisplacement(pt,line):
    [a1,b1,c1] = line.standardForm()
    a2 = -b1
    b2 = a1
    c2 = a2*pt.x() + b2*pt.y()
    pt2 = intersectStandardForm([a1,b1,c1],[a2,b2,c2])
    return LineSegment(pt,pt2)
    
def ptSegmentDisplacement(pt,seg):
    if seg.contains(pt):
        return LineSegment(pt,pt)
    [a1,b1,c1] = seg.standardForm()
    a2 = -b1
    b2 = a1
    c2 = a2*pt.x() + b2*pt.y()
    pt2 = intersectStandardForm([a1,b1,c1],[a2,b2,c2])
    if not pt2 or not seg.contains(pt2):
        pt2 = min([seg.start(),seg.end()], key=lambda p: distance(pt,p))
    return LineSegment(pt,pt2)
    
def bisectLine(seg,bisector):
    start = seg.start()
    end = seg.end()
    inter = intersect(seg,bisector)
    if not inter or not (seg.contains(inter) and bisector.contains(inter)) or inter == end:
        return False
    line1 = LineSegment(start,inter)
    line2 = LineSegment(inter,end)
    return [line1,line2]
    

def bisectLineByPts(seg,pts):
    output = [seg]    
    for pt in pts:
        toSplit = [line for line in output if line.contains(pt)]
        for line in toSplit:
            output.remove(line)
            line1 = LineSegment(line.start(),pt)
            line2 = LineSegment(pt,line.end())
            output.append(line1)
            output.append(line2)
    return output

def bisectLineByPts2(seg,pts):
    output = []
    [a,b,c] = seg.standardForm()
    start = seg.start()
    for pt in sorted(pts,key=lambda p: a*p.x() + b*p.y()):
        if seg.containsExclusive(pt):
            output.append(LineSegment(start,pt))
            start = pt
    output.append(LineSegment(start,seg.end()))
    return output

    
def bisectPoly(poly,bisector):
    poly1 = True
    lines1 = []
    lines2 = []
    inters = []
    for seg in poly.sides():
        lines = bisectLine(seg,bisector)
        if lines:
            [line1,line2] = lines
            if(poly1):
                if line1.length() > 0:
                    lines1.append(line1)
                if line2.length() > 0:
                    lines2.append(line2)
            else:
                if line1.length() > 0:
                    lines2.append(line1)
                if line2.length() > 0:
                    lines1.append(line2)
            inters.append(intersect(seg,bisector))
            poly1 = not poly1
        else:
            if(poly1):
                lines1.append(seg)
            else:
                lines2.append(seg)
    if(len(inters) < 2):
        return [poly,False]
    if(len(inters) >= 2):
        line = LineSegment(inters[0],inters[1])
        lines1.append(line)
        lines2.append(line)
    output = []
    for lines in [lines1,lines2]:
        if len(lines) < 3:
            output.append(False)
        else:
            output.append(polyFromSides(lines))
    return output
    pts1 = []
    for line in lines1:
        if not line.start() in pts1:
            pts1.append(line.start())
        if not line.end() in pts1:
            pts1.append(line.end())
    for line in lines2:
        if not line.start() in pts2:
            pts2.append(line.start())
        if not line.end() in pts2:
            pts2.append(line.end())
    if len(pts1) >= 3:
        poly1 = Polygon(*pts1)
    else:
        poly1 = False
    if len(pts2) >= 3:
        poly2 = Polygon(*pts2)
    else:
        poly2 = False
    return [poly1,poly2]
    
def mirrorPoly(poly,line):
    pts = [mirrorPt(pt,line) for pt in poly.vertices()]
    return Polygon(*pts)
    

    
def findNearestPt(pt,pts):
    return min(pts, key= lambda pt1: distance(pt,pt1))
    
def distance(pt1,pt2):
    return math.sqrt( (pt2.x()-pt1.x())**2 + (pt2.y()-pt1.y())**2)

def shiftLeft(seg,amt):
    old_dx = seg.dx()
    old_dy = seg.dy()
    new_dx = -amt * old_dy / (min(old_dx,old_dy)+0.5)
    new_dy = -amt * old_dx / (min(old_dx,old_dy)+0.5)
    output = seg.dupl()
    output.translate(new_dx,new_dy)
    return output
    
def getBoundingBox(pts):
    xMin = int(min([pt.x() for pt in pts]))
    xMax = int(max([pt.x() for pt in pts]))
    yMin = int(min([pt.y() for pt in pts]))
    yMax = int(max([pt.y() for pt in pts]))
    xRange = range(xMin,xMax+1) #make inclusive
    yRange = range(yMin,yMax+1) #make inclusive
    return (xRange,yRange)

def polyFromSides(sides):
    if len(sides) < 3:
        return False
    verts = []
    curside = sides[0]
    while len(sides)>0:
        verts.append(curside.start())
        sides.remove(curside)
        if len(sides)>0:
            contiguous = [side for side in sides if side.start() == curside.end() or side.end() == curside.end()]
            if len(contiguous) == 0:
                return False
            nextSide = max(contiguous,key=lambda x: x.length())
            for ln in [side for side in contiguous if side != nextSide]:
                sides.remove(ln)
            if nextSide.end() == curside.end():
                nextSide.flip()
            curside = nextSide
    return Polygon(*verts)

def mergeLines(segs):
    pts = []
    for seg in segs:
        for pt in seg.pts():
            if not pt in pts:
                pts.append(pt)
    newlines = [LineSegment(pt1,pt2) for pt1 in pts for pt2 in pts if pt1 != pt2]
    return max(newlines,key=lambda seg: seg.length())

def getConnectedComponents(lines):
    segs = list(lines)
    components = []
    while len(segs) > 0:
        curcomp = []
        seg = segs[0]
        connected = [seg]
        while len (connected) > 0:
            seg = connected[0]
            for seg2 in connected:
                segs.remove(seg2)
                curcomp.append(seg2)
            connected = [seg2 for seg2 in segs if seg.start() in seg2.pts() or seg.end() in seg2.pts()]
        components.append(curcomp)
        
    return components

def getBoundingArea(segs):
    pts = []
    for seg in segs:
        for pt in seg.pts():
            if not pt in pts:
                 pts.append(pt)
    (xRange,yRange) = getBoundingBox(pts)
    return len(xRange) * len(yRange)
