#!/usr/bin/env python
# coding: utf-8

# In[ ]:


from copy import deepcopy
from itertools import chain
import time
import heapq as heap
import sys
import matplotlib.pyplot as plt

goal8 = [[1,2,3],[4,5,6],[7,8,0]]    #for 8 puzzle
goal15 = [[1,2,3,4],[5,6,7,8],[9,10,11,12],[13,14,15,0]]     #for 15 puzzle
goal24 = [[1,2,3,4,5],[6,7,8,9,10],[11,12,13,14,15],[16,17,18,19,20],[21,22,23,24,0]]    #for 24 puzzle
mapping = {}                                #to trace the path of parent states
mq_size=0      #maximum size of queue

def main():
    
    depth_samples = ( [[1,2,3],[4,5,6],[7,8,0]] , [[1,2,3],[4,5,6],[0,7,8]] , [[1,2,3],[5,0,6],[4,7,8]],
                     
                      [[1,3,6],[5,0,2],[4,7,8]], [[1,3,6],[5,0,7],[4,8,2]], [[1,6,7],[5,0,3],[4,8,2]], 
                     
                      [[7,1,2],[4,8,5],[6,3,0]], [[0,7,2],[4,6,1],[3,5,8]] )
    
    print("8 puzzle solver!\n\n")
    
    print("The puzzle can be a default and also the one of your choice. Choose 1 for default or choose 2 for your own puzzle\n")
    
    choice = int(input())
    
    if(choice==1):
        
        puzzle = [[7,1,2],[4,8,5],[6,3,0]]
        print("The default puzzle looks something like this\n")
        print(puzzle)
        
    elif(choice==2):
        
        print("Enter number of rows\n")
        rows = int(input())
        print("Enter number of columns\n")
        cols = int(input())
        
        print("Enter the elements one by one. Use zero to represent a blank element in the puzzle\n")
        
        puzzlein = []
        for i in range(rows):
            inside = []
            for j in range(cols):
                temp = int(input())
                inside.append(temp)
            puzzlein.append(inside)

        print("\n")
        print("The puzzle entered by you looks something like this \n")
    
        for z1 in range(rows):
            for z2 in range(cols):
                print(puzzlein[z1][z2],end=" ")
            print()
            
        print("\n")
    
    print("There are three ways to solve it! Please choose one of the following methods:\n\n")
    
    print("1. Uniform Cost Search\n2. A* with the Misplaced Tile heuristic\n3. A* with Manhattan Distance heuristic\n")
    
    method = int(input())
    
    if(choice==1):
        expanded_nodes, depth, maximum_size_of_queue, final_time = general_search(puzzle,method)
    else:
        expanded_nodes, depth, maximum_size_of_queue, final_time = general_search(puzzlein,method)
    
    print("Expanded nodes: " + str(expanded_nodes))
    print("Depth: " + str(depth))
    print("Maximum size of queue:" + str(maximum_size_of_queue))
    
    print('%.2f' % (final_time))
    
    print("Choose 1 to plot the graph, 0 to terminate")
    
    graph = int(input())
    
    if graph==1:
        plotter(depth_samples,method)
    else:
        sys.exit()
    sys.exit()
    
def general_search(problem,choice):
    
    many_nodes = []
    initial_node = (0,0,problem)                #initial node is a tuple where I store the cost, depth and the puzzle state
    heap.heappush(many_nodes,initial_node)
    expanded = 0
    visited_set = set()                         #to keep track of already visited states
    
    initial_time = time.time()
    
    global mq_size
    mq_size = 0
    
    while True:
        
        if(len(many_nodes)==0):
            print("Failure")
            break
        
        node = heap.heappop(many_nodes)
        print("The best state to expand with a g(n) = ",node[0],"and h(n) = ",node[1],"is...\n")
        node_content = node[2]
        for insider in range(len(node_content)):
            print(node_content[insider])
        print("\n")
        expanded += 1

        visited_set.add(tuple(map(tuple,node[2])))
        
        if node[2]==goal8:                      #change goal8 to goal15 for 15 puzzle and goal24 for 24 puzzle
            ending_time = time.time()
            time_diff = ending_time - initial_time
            return expanded,node[1],mq_size,time_diff
        
        many_nodes = queueing_function(many_nodes,expansion(node,choice,visited_set))
    
def expansion(parent_node,algorithm,seen):
    
    child_states = []
    
    cost = parent_node[0]
    depth = parent_node[1]
    matmat = parent_node[2]
    
    for hail in range(len(matmat)):
        for ucr in range(len(matmat)):
            if(matmat[hail][ucr]==0):

                ele_row = hail
                ele_col = ucr

                if(ele_col>0):
                    left_child = (cost,depth,left(matmat,ele_row,ele_col))
                    temp_tup_left = (tuple(map(tuple,left_child[2])))
                    if (temp_tup_left not in seen):
                        child_states.append(left_child)
                    
                if(ele_col<len(matmat)-1):
                    right_child = (cost,depth,right(matmat,ele_row,ele_col))
                    temp_tup_right = (tuple(map(tuple,right_child[2])))
                    if (temp_tup_right not in seen):
                        child_states.append(right_child)

                if(ele_row>0):
                    up_child = (cost,depth,up(matmat,ele_row,ele_col))
                    temp_tup_up = (tuple(map(tuple,up_child[2])))
                    if (temp_tup_up not in seen):
                        child_states.append(up_child)

                if(ele_row<len(matmat)-1):
                    down_child = (cost,depth,down(matmat,ele_row,ele_col))
                    temp_tup_down = (tuple(map(tuple,down_child[2])))
                    if (temp_tup_down not in seen):
                        child_states.append(down_child)
    
    final = []
    for each_child in child_states:
        
        each_child = list(each_child)
        each_child[1] += 1
        each_child = tuple(each_child)
        
        if algorithm == 1:
            each_child = list(each_child)
            each_child[0] += 1
            each_child = tuple(each_child)
        if algorithm == 2:
            h_n = misplaced_tiles(each_child[2])
            each_child = list(each_child)
            each_child[0] = each_child[0] + h_n + 1
            each_child = tuple(each_child)
        if algorithm ==3:
            h_n = manhattan(each_child[2])
            each_child = list(each_child)
            each_child[0] = each_child[0] + h_n + 1
            each_child = tuple(each_child)
        
        final.append(each_child)
        
    return final

def queueing_function(node_list,child_list):
    global mq_size
    for e in child_list:
        heap.heappush(node_list,e)
        if (len(node_list) > mq_size):
            mq_size = len(node_list)
    return node_list
    
def left(matrix1,p,q):
    
    xten1 = deepcopy(matrix1)
    tempo1 = xten1[p][q]
    xten1[p][q] = xten1[p][q-1]
    xten1[p][q-1] = tempo1
    
    fedup1 = tuple((map(tuple,xten1)))
    
    mapping[fedup1] = tuple((map(tuple,matrix1)))
    return xten1
    
def right(matrix2,r,s):
    
    xten2 = deepcopy(matrix2)
    tempo2 = xten2[r][s]
    xten2[r][s] = xten2[r][s+1]
    xten2[r][s+1] = tempo2
    
    fedup2 = tuple((map(tuple,xten2)))

    mapping[fedup2] = tuple((map(tuple,matrix2)))
    return xten2
    
def up(matrix3,t,u):
    
    xten3 = deepcopy(matrix3)
    tempo3 = xten3[t][u]
    xten3[t][u] = xten3[t-1][u]
    xten3[t-1][u] = tempo3
    
    fedup3 = tuple((map(tuple,xten3)))

    mapping[fedup3] = tuple((map(tuple,matrix3)))
    return xten3
    
def down(matrix4,v,w):
    
    xten4 = deepcopy(matrix4)
    tempo4 = xten4[v][w]
    xten4[v][w] = xten4[v+1][w]
    xten4[v+1][w] = tempo4
    
    fedup4 = tuple((map(tuple,xten4)))

    mapping[fedup4] = tuple((map(tuple,matrix4)))
    return xten4

def manhattan(two_d):

    dict = {1:[0,0], 2:[0,1], 3:[0,2], 4:[1,0], 5:[1,1], 6:[1,2], 7:[2,0], 8:[2,1]}  #to create this mapping for puzzle of any size, need to traverse the entire puzzle once
    cost_of_one_state = 0

    for ice in range(len(two_d)):
        for juice in range(len(two_d)):
            if(two_d[ice][juice]!=0):
                r = ice
                c = juice
                linear = dict.get(two_d[ice][juice])
                cost_of_one_state += abs(linear[0]-r) + abs(linear[1]-c)

    return cost_of_one_state
        
def misplaced_tiles(tile_cost):

    count = 0    
    calculation = list(chain.from_iterable(tile_cost))

    for element in calculation:
        if(element!=0):
            position = calculation.index(element)
            if((element-position)!=1):
                count+=1
    
    return count

def plotter(testers,option):
    
    exp_node_testers_uniform = []
    depth_testers_uniform = []
    max_queue_testers_uniform = []
    time_testers_uniform = []
    
    exp_node_testers_misplaced = []
    depth_testers_misplaced = []
    max_queue_testers_misplaced = []
    time_testers_misplaced = []
    
    exp_node_testers_manhattan = []
    depth_testers_manhattan = []
    max_queue_testers_manhattan= []
    time_testers_manhattan = []
    
    for algos in range(1,4):
        
        for looping in testers:
            
            exp_node,d,mqsz,t = general_search(looping,algos)

            if algos==1:
                depth_testers_uniform.append(d)
                max_queue_testers_uniform.append(mqsz)
                time_testers_uniform.append(t)
                exp_node_testers_uniform.append(exp_node)

            if algos==2:
                depth_testers_misplaced.append(d)
                max_queue_testers_misplaced.append(mqsz)
                time_testers_misplaced.append(t)
                exp_node_testers_misplaced.append(exp_node)

            if algos==3:
                depth_testers_manhattan.append(d)
                max_queue_testers_manhattan.append(mqsz)
                time_testers_manhattan.append(t)
                exp_node_testers_manhattan.append(exp_node)

    #Depth and nodes expanded
    plt.figure("Figure 1")
    plt.plot(depth_testers_uniform,exp_node_testers_uniform)
    plt.plot(depth_testers_misplaced,exp_node_testers_misplaced)
    plt.plot(depth_testers_manhattan,exp_node_testers_manhattan)
    plt.title("Depth vs Number of expanded nodes")
    plt.xlabel("Depth")
    plt.ylabel("Expanded nodes")
    plt.show()
    
    #Depth and time
    plt.plot(depth_testers_uniform,time_testers_uniform)
    plt.plot(depth_testers_misplaced,time_testers_misplaced)
    plt.plot(depth_testers_manhattan,time_testers_manhattan)
    plt.title("Depth vs Time")
    plt.xlabel("Depth")
    plt.ylabel("Time")
    plt.show()
    
    #Depth and maximum queue size
    plt.plot(depth_testers_uniform,max_queue_testers_uniform)
    plt.plot(depth_testers_misplaced,max_queue_testers_misplaced)
    plt.plot(depth_testers_manhattan,max_queue_testers_manhattan)
    plt.title("Depth vs Queue Size")
    plt.xlabel("Depth")
    plt.ylabel("Max Queue")
    plt.show()

   
if __name__ == "__main__":
    main()    

