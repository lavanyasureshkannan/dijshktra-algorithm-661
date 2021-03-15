#import all the necessary libraries
import cv2
import numpy as np
import math
import time
#fourcc = cv2.VideoWriter_fourcc('X','V','I','D')
#videoWriter = cv2.VideoWriter('output.avi', fourcc, 30.0,(400,400))
#get input from the user
x1=int(input("enter start point i coordinate:"))
y1=int(input("enter start point j coordinate:"))
x2=int(input("enter goal point i coordinate:"))
y2=int(input("enter goal point j coordinate:"))
#declaring the grid
xaxis = 300
yaxis = 200
#declaring the image size
img = np.ones((yaxis,xaxis,3),np.uint8)*150
#creating a visited array
marked=np.array(np.zeros((xaxis,yaxis)))
#declaring a start time
s_time=time.time()
#declaring the parent node
start_point=[x1,y1]
goal_point=[x2,y2]

parent_node=[]

for j in range (300):
    a=[]
    for i in range (200):
        a.append(0)
    parent_node.append(a)
#defin the obs map
def map(x,y):
    #using semi algebric equation defining the obstacle
    #the shape is divided into three polygon
    poly1 = (x+(y-100) < 0 and (y+15*x-340 > 0) and y+(-7/5)*x+20 > 0)
    poly2 = ((7/5)*x+y-120<0 and (y-15) > 0 and y+(-7/5)*x+20 < 0)
    poly3 = ((6/5)*x+y-169 <0 and (-6/5)*x+y+9 < 0 and (6/5)*x+y-169<0 and (-7/5)*x+y+89>0)
    circle = (((x-220)**2)+ ((y-40)**2) <=(23)**2)
    ellipse = ((((x-150)**2)/(40)**2)+(((y-100)**2)/(20)**2) -1 <=0)
    rectpol=(x >= 110 and x <= 120 and y <= 190 and y >=140 ) or (x>= 120 and x <= 140 and y >=180 and y <= 190) or (y >= 140 and y <= 150 and x >= 120 and x <= 140)
    rectangle = (200-y) - (1.7)*x + 135 > 0 and (200-y) + (0.5)*x - 96  <= 0 and (200-y) - (1.7)*x - 15 <= 0 and (200-y) + (0.5)*x - 84 >= 0

    if poly1 or poly2 or poly3 or circle or ellipse or rectpol or rectangle:
        val = 0
    else:
        val = 1
    return val


def poly1(x,y):
    if(x+(y-100) < 0 and (y+15*x-340 > 0) and y+(-7/5)*x+20 > 0):
        return True
def poly2(x,y):
    if((7/5)*x+y-120<0 and (y-15) > 0 and y+(-7/5)*x+20 < 0):
        return True
def poly3(x,y):
    if ((6/5)*x+y-169 <0 and (-6/5)*x+y+9 < 0 and (6/5)*x+y-169<0 and (-7/5)*x+y+89>0):
        return True
def circle(x,y):
    if (((x-225)**2)+ ((y-50)**2) <=(25)**2):
        return True
def ellipse(x,y):
    if ((((x-150)**2)/(40)**2)+(((y-100)**2)/(20)**2) -1 <=0):
        return True
def rectangle(x,y):
    if (200-y) - (1.7)*x + 135 > 0 and (200-y) + (0.5)*x - 96  <= 0 and (200-y) - (1.7)*x - 15 <= 0 and (200-y) + (0.5)*x - 84 >= 0:
        return True
def rectpol(x,y):
    if (x >= 110 and x <= 120 and y <= 190 and y >=140 ) or (x>= 120 and x <= 140 and y >=180 and y <= 190) or (y >= 140 and y <= 150 and x >= 120 and x <= 140):
        return True
#checking the boundary
def border(i,j):
    if (i<0 or j<0 or i>299 or j>199 ):
        return 0
    else:
        return 1
#making the cost of other nodes as infinity
cost=np.array(np.ones((xaxis,yaxis)) * 1000000000000000000)

A=[]
A.append([x1,y1])
#declaring that the starting cost as zero
cost[x1][y1]=0
#importing queue
def pop(A):
    min_value=0
    X,Y = A[0][0],A[0][1]
    for i in range(len(A)):
        nx = A[i][0]
        ny = A[i][1]
        if cost[nx,ny] < cost[X,Y]:
            min_value = i
            X = nx
            Y= ny

    present_node = A[min_value]
    A.remove(A[min_value])
    return present_node
#defining the action set

#action1- upwards direction
def action1(i,j):
    possible_set=[i,j+1]
    return possible_set
#action2 - downwards direction
def action2(i,j):
    possible_set=[i,j-1]
    return possible_set
#action3- right
def action3(i,j):
    possible_set=[i+1,j]
    return possible_set
#action4-left
def action4(i,j):
    possible_set=[i-1,j]
    return possible_set
#action5-upright
def action5(i,j):
    possible_set=[i+1,j+1]
    return possible_set
#action6-downright
def action6(i,j):
    possible_set=[i+1,j-1]
    return possible_set
#action7-upleft
def action7(i,j):
    possible_set=[i-1,j+1]
    return possible_set
#action8-downleft
def action8(i,j):
    possible_set=[i-1,j-1]
    return possible_set

#creating a list that are marked alerady
marked_nodes=[]
present_node=[x1,y1]
#implementing dijkstra algorithm
while present_node != goal_point:
    present_node=pop(A)

    #upwards
    n_action1=action1(present_node[0],present_node[1])
    m=border(n_action1[0],n_action1[1])
    n=map(n_action1[0],n_action1[1])

    if (m and n == 1):
        if marked[n_action1[0],n_action1[1]]==0:
            marked[n_action1[0],n_action1[1]]=1
            marked_nodes.append(n_action1)
            A.append(n_action1)
            parent_node[n_action1[0]][n_action1[1]]=present_node
            cost[n_action1[0],n_action1[1]]=(cost[present_node[0],present_node[1]]+1)
        else:
            if cost[n_action1[0],n_action1[1]]>(cost[n_action1[0],n_action1[1]]+1):
                cost[n_action1[0],n_action1[1]]=(cost[n_action1[0],n_action1[1]]+1)
                parent_node[n_action1[0]][n_action1[1]]=present_node

    #downwards
    n_action2=action2(present_node[0],present_node[1])
    m=border(n_action2[0],n_action2[1])
    n=map(n_action2[0],n_action2[1])
    if (m and n == 1):
        if marked[n_action2[0],n_action2[1]]==0:
            marked[n_action2[0],n_action2[1]]=1
            marked_nodes.append(n_action2)
            A.append(n_action2)
            parent_node[n_action2[0]][n_action2[1]]=present_node
            cost[n_action2[0],n_action2[1]]=(cost[present_node[0],present_node[1]]+1)
        else:
            if cost[n_action2[0],n_action2[1]]>(cost[n_action2[0],n_action2[1]]+1):
                cost[n_action2[0],n_action2[1]]=(cost[n_action2[0],n_action2[1]]+1)
                parent_node[n_action2[0]][n_action2[1]]=present_node

    #right
    n_action3=action3(present_node[0],present_node[1])
    m=border(n_action3[0],n_action3[1])
    n=map(n_action3[0],n_action3[1])
    if (m and n == 1):
        if marked[n_action3[0],n_action3[1]]==0:
            marked[n_action3[0],n_action3[1]]=1
            marked_nodes.append(n_action3)
            A.append(n_action3)
            parent_node[n_action3[0]][n_action3[1]]=present_node
            cost[n_action3[0],n_action3[1]]=(cost[present_node[0],present_node[1]]+1)
        else:
            if cost[n_action3[0],n_action3[1]]>(cost[n_action3[0],n_action3[1]]+1):
                cost[n_action3[0],n_action3[1]]=(cost[n_action3[0],n_action3[1]]+1)
                parent_node[n_action3[0]][n_action3[1]]=present_node
    #left
    n_action4=action4(present_node[0],present_node[1])
    m=border(n_action4[0],n_action4[1])
    n=map(n_action4[0],n_action4[1])
    if (m and n == 1):
        if marked[n_action4[0],n_action4[1]]==0:
            marked[n_action4[0],n_action4[1]]=1
            marked_nodes.append(n_action4)
            A.append(n_action4)
            parent_node[n_action4[0]][n_action4[1]]=present_node
            cost[n_action4[0],n_action4[1]]=(cost[present_node[0],present_node[1]]+1)
        else:
            if cost[n_action4[0],n_action4[1]]>(cost[n_action4[0],n_action4[1]]+1):
                cost[n_action4[0],n_action4[1]]=(cost[n_action4[0],n_action4[1]]+1)
                parent_node[n_action4[0]][n_action4[1]]=present_node
    #upright
    n_action5=action5(present_node[0],present_node[1])
    m=border(n_action5[0],n_action5[1])
    n=map(n_action5[0],n_action5[1])
    if (m and n == 1):
        if marked[n_action5[0],n_action5[1]]==0:
            marked[n_action5[0],n_action5[1]]=1
            marked_nodes.append(n_action5)
            A.append(n_action5)
            parent_node[n_action5[0]][n_action5[1]]=present_node
            cost[n_action5[0],n_action5[1]]=(cost[present_node[0],present_node[1]]+1.414)
        else:
            if cost[n_action5[0],n_action5[1]]>(cost[n_action5[0],n_action5[1]]+1.414):
                cost[n_action5[0],n_action5[1]]=(cost[n_action5[0],n_action5[1]]+1.414)
                parent_node[n_action5[0]][n_action5[1]]=present_node
    #upleft

    n_action6=action6(present_node[0],present_node[1])
    m=border(n_action6[0],n_action6[1])
    n=map(n_action6[0],n_action6[1])
    if (m and n == 1):
        if marked[n_action6[0],n_action6[1]]==0:
            marked[n_action6[0],n_action6[1]]=1
            marked_nodes.append(n_action6)
            A.append(n_action6)
            parent_node[n_action6[0]][n_action6[1]]=present_node
            cost[n_action6[0],n_action6[1]]=(cost[present_node[0],present_node[1]]+1.414)
        else:
            if cost[n_action6[0],n_action6[1]]>(cost[n_action6[0],n_action6[1]]+1.414):
                cost[n_action6[0],n_action6[1]]=(cost[n_action6[0],n_action6[1]]+1.414)
                parent_node[n_action6[0]][n_action6[1]]=present_node
    #downright
    n_action7=action7(present_node[0],present_node[1])
    m=border(n_action7[0],n_action7[1])
    n=map(n_action7[0],n_action7[1])
    if (m and n == 1):
        if marked[n_action7[0],n_action7[1]]==0:
            marked[n_action7[0],n_action7[1]]=1
            marked_nodes.append(n_action7)
            A.append(n_action7)
            parent_node[n_action7[0]][n_action7[1]]=present_node
            cost[n_action7[0],n_action7[1]]=(cost[present_node[0],present_node[1]]+1.414)
        else:
            if cost[n_action7[0],n_action7[1]]>(cost[n_action7[0],n_action7[1]]+1.414):
                cost[n_action7[0],n_action7[1]]=(cost[n_action7[0],n_action7[1]]+1.414)
                parent_node[n_action7[0]][n_action7[1]]=present_node
    #downleft

    n_action8=action8(present_node[0],present_node[1])
    m=border(n_action8[0],n_action8[1])
    n=map(n_action8[0],n_action8[1])
    if (m and n == 1):
        if marked[n_action8[0],n_action8[1]]==0:
            marked[n_action8[0],n_action8[1]]=1
            marked_nodes.append(n_action8)
            A.append(n_action8)
            parent_node[n_action8[0]][n_action8[1]]=present_node
            cost[n_action8[0],n_action8[1]]=(cost[present_node[0],present_node[1]]+1.414)
        else:
            if cost[n_action8[0],n_action8[1]]>(cost[n_action8[0],n_action8[1]]+1.414):
                cost[n_action8[0],n_action8[1]]=(cost[n_action8[0],n_action8[1]]+1.414)
                parent_node[n_action8[0]][n_action8[1]]=present_node
#finding the path
p=[]
def find(goal_point,start_point):
    end =goal_point
    p.append(goal_point)
    while (end != start_point):
        b=parent_node[end[0]][end[1]]
        p.append(b)
        end=b
find(goal_point,start_point)
#visualisation
final=cv2.resize(img,None,fx=3,fy=3)
cv2.circle(img,(int(goal_point[0]),int(goal_point[1])), (1), (0,150,255), -1);
cv2.circle(img,(int(start_point[0]),int(start_point[1])), (1), (0,150,255), -1);

for i in range(yaxis):
    for j in range(xaxis):
         if poly1(j,i) or poly2(j,i) or poly3(j,i) or circle(j,i) or ellipse(j,i) or rectangle(j,i) or rectpol(j,i) :
            img[i][j] = 0


for n in marked_nodes:
    cv2.circle(img,(int(n[0]),int(n[1])), (1), (0,255,80));
    final=cv2.resize(img,None,fx=3,fy=3)
    cv2.imshow('final',final)
    cv2.waitKey(10)

for n in p:
    cv2.circle(img,(int(n[0]),int(n[1])), (1), (150,50,204));
    final=cv2.resize(img,None,fx=3,fy=3)
    cv2.imshow('final',final)
    cv2.waitKey(10)

print("reached")
print('The shortest path cost is',cost[x2,y2])
cv2.waitKey(0)
cv2.destroyAllWindows()
