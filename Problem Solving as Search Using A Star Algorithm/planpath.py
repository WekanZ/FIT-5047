import argparse as ap

import re

import platform



######## RUNNING THE CODE ####################################################

#   You can run this code from terminal by executing the following command

#   python planpath.py <INPUT/input#.txt> <OUTPUT/output#.txt> <flag>

#   for example: python planpath.py INPUT/input2.txt OUTPUT/output2.txt 0

#   NOTE: THIS IS JUST ONE EXAMPLE INPUT DATA

###############################################################################





################## YOUR CODE GOES HERE ########################################

def graphsearch(map, flag):

    solution = ''

    open_list = []
    close_list = []
    
    nodes = read_map(map)

    start_point = find_start(nodes)
    goal_point = find_goal(nodes)

    id_list = generate_id_list(nodes)
    start_point.set_id(id_list)

    open_list.append(start_point)


    while len(open_list) != 0:

        exp_node = find_exp_node(open_list)
        
        open_list.remove(exp_node)
        close_list.append(exp_node)

        exp_node.set_order(len(close_list))

        if exp_node == goal_point:
           
            if flag > 0:
                flag = diagnose(open_list, close_list, exp_node, flag)
            
            break
        
        neighbors = find_neighbors(nodes, exp_node)
        open_neighbors(nodes, neighbors, open_list, close_list, exp_node, goal_point, id_list)
        
        if flag > 0:

            flag = diagnose(open_list, close_list, exp_node, flag)


    if (len(open_list) == 0) & (goal_point not in close_list):

        solution = 'NO-PATH'
        print('No Feasible Solution')

    else:
        
        route = generate_route(goal_point)
        
        directions = generate_directions(route)
        
        solution = generate_result(nodes, route, directions)
        
        print('Successful, please open the output file to find the detailed solution')



    return solution



def read_from_file(file_name):

    # You can change the file reading function to suit the way

    # you want to parse the file

    file_handle = open(file_name)

    map = file_handle.readlines()

    return map



def read_map(map):
    
    size = int(map[0][0:-1])
    nodes = []
    
    for x in range(size):
        
        for y in range(size):
            
            nodes.append(Node(x, y, map[y + 1][x]))
    
    return nodes



def find_node(x, y, nodes):
    
    for node in nodes:
        
        if (node.x == x) & (node.y == y):
            
            return node 
    
    return None   



def find_start(nodes):
    
    for node in nodes:
        
        if node.name == 'S':
            
            return node
    
    return None



def find_goal(nodes):
    
    for node in nodes:
        
        if node.name == 'G':
            
            return node
    
    return None



def generate_id_list(nodes):
    
    id_list = []
    
    for i in range(len(nodes)):
        
        id_list.append('N' + str(i))
    
    return id_list



def find_neighbors(nodes, parent):
    
    neighbors = []
    
    for node in nodes:
        
        if (abs(parent.x - node.x) <= 1) & (abs(parent.y - node.y) <= 1):
            
            neighbors.append(node)
    
    neighbors.remove(parent) 
    
    return neighbors



def open_neighbors(nodes, neighbors, open_list, close_list, parent, goal_point, id_list):
    
    for neighbor in neighbors:
        
        if (neighbor not in open_list) and (neighbor not in close_list):
            
            neighbor.add_parent(parent)
            
            if (not neighbor.isMT()) and (not neighbor.isBlock(nodes)):
                
                neighbor.calculate_g()
                neighbor.calculate_h(goal_point)
                neighbor.calculate_f()
                
                neighbor.set_id(id_list)
                neighbor.extend_route(judge_direction(parent, neighbor))
                
                open_list.append(neighbor)
                
                parent.add_child(neighbor)
            
            else:
                neighbor.remove_parent()



def find_exp_node(open_list):
    
    exp_node = open_list[0]
    
    for node in open_list:
        
        if exp_node.f > node.f:
            
            exp_node = node
    
    return exp_node



def generate_route(goal_point):
    
    parent = goal_point.parent
    arc_route = [goal_point]
    
    while not parent is None:
        
        arc_route.append(parent)
        parent = parent.parent
    
    route = [arc_route[len(arc_route) - i - 1] for i in range(len(arc_route))]
    
    return route



def judge_direction(op, ed):
    
    direction = ''
    
    if ed.x - op.x == 1:
        direction += 'R'
    
    if ed.x - op.x == -1:
        direction += 'L'
    
    if ed.y - op.y == 1:
        direction += 'D'
    
    if ed.y - op.y == -1:
        direction += 'U'
   
    return direction



def generate_directions(route):
    
    directions = []
    
    for i in range(len(route) - 1):
        
        directions.append(judge_direction(route[i], route[i + 1]))
    
    return directions



def generate_map_view(nodes, current_node):
    
    map = ''
    
    for y in range(int(len(nodes)**0.5)):
        
        for x in range(int(len(nodes)**0.5)):
            
            if (current_node.x == x) & (current_node.y == y) & (not current_node.name in ['S', 'G']):
                map += '*'
            else:
                map += find_node(x, y, nodes).name
        
        map += '\n'
    
    return map



def generate_current_route(route, directions, current_node):
    
    route_line = 'S'
    
    for i in range(route.index(current_node)):
        
        route_line += '-' + directions[i]
  
    if current_node.name == 'G':
        
        route_line += '-G'
    
    route_line += ' ' + str(current_node.g) + '\n'
    
    return route_line



def generate_result(nodes, route, directions):
    
    result = '------------------------------\n'
    
    for node in route:
        
        map = generate_map_view(nodes, node)
        
        result += map + '\n'
        result += generate_current_route(route, directions, node)
        result += '------------------------------\n'
    
    return result



def get_node_id(node):
    return str(node.id[1:])



def get_node_f(node):
    return node.f



def diagnose(open_list, close_list, exp_node, flag):
    
    index = len(close_list)
    print('########Step' + str(index) + '########')
    
    print(exp_node.id, ': ', exp_node.route, ' ', end = '',sep = '')
    print(exp_node.order, exp_node.g, exp_node.h, exp_node.f, sep = ' ')
    
    if len(exp_node.children) == 0:
        
        print('Children: [None]')
    
    else:
        
        print('Children:', [i.id + ': ' + i.route for i in exp_node.children])
    
    if len(open_list) == 0:

        print('Open: [None]')

    else:

        open_list.sort(key = get_node_f, reverse = False)

        print('Open:', [i.id + ': ' + i.route + ' ' + str(i.g) + ' ' + str(i.h) + ' ' + 
            str(i.f) for i in open_list])

    close_list.sort(key = get_node_id, reverse = False)
    
    print('Closed:', [i.id + ': ' + i.route + ' ' + str(i.order) + ' ' + str(i.g) + ' ' + 
        str(i.h) + ' ' + str(i.f) for i in close_list])

    print()
    
    return flag - 1



class Node:
    'Each cell in this map'
    
    def __init__(self, x, y, name):
        
        self.x = x
        self.y = y
        self.parent = None
        self.children = []
        self.route = 'S'
        self.name = name 
        self.id = 0
        self.order = 0
        self.g = 0
        self.h = 0
        self.f = 0



    def add_child(self, node):
        self.children.append(node)
        self.children.sort(key = get_node_id, reverse = False)



    def add_parent(self, node):
        self.parent = node



    def remove_parent(self):
        self.parent = None



    def extend_route(self, direction):
        self.route = self.parent.route + '-' + direction



    def isMT(self):
        return self.name == 'X'
    


    def isBlock(self, nodes):
        
        if abs(self.x - self.parent.x) + abs(self.y - self.parent.y) == 2:
            
            if (find_node(self.x, self.parent.y, nodes).isMT()) | (find_node(self.parent.x, self.y, nodes).isMT()):
                
                return True
        
        return False



    def calculate_g(self):
        
        if abs(self.x - self.parent.x) + abs(self.y - self.parent.y) == 1:
            
            self.g = self.parent.g + 2
        
        else:
            
            self.g = self.parent.g + 1



    def calculate_h(self, goal_point):
        
        dx = abs(self.x - goal_point.x)
        dy = abs(self.y - goal_point.y)
        
        self.h = min(dx,dy) + round(0.1 + abs(dx - dy) / 2) * 2



    def calculate_f(self):
        self.f = self.g + self.h



    def set_id(self, id_list):
        self.id = id_list[0]
        id_list.remove(self.id)



    def set_order(self, num):
        self.order = num




###############################################################################

########### DO NOT CHANGE ANYTHING BELOW ######################################

###############################################################################



def write_to_file(file_name, solution):

    file_handle = open(file_name, 'w')

    file_handle.write(solution)



def main():

    # create a parser object

    parser = ap.ArgumentParser()



    # specify what arguments will be coming from the terminal/commandline

    parser.add_argument("input_file_name", help="specifies the name of the input file", type=str)

    parser.add_argument("output_file_name", help="specifies the name of the output file", type=str)

    parser.add_argument("flag", help="specifies the number of steps that should be printed", type=int)

    # parser.add_argument("procedure_name", help="specifies the type of algorithm to be applied, can be D, A", type=str)





    # get all the arguments

    arguments = parser.parse_args()



##############################################################################

# these print statements are here to check if the arguments are correct.

#    print("The input_file_name is " + arguments.input_file_name)

#    print("The output_file_name is " + arguments.output_file_name)

#    print("The flag is " + str(arguments.flag))

#    print("The procedure_name is " + arguments.procedure_name)

##############################################################################



    # Extract the required arguments



    operating_system = platform.system()



    if operating_system == "Windows":

        input_file_name = arguments.input_file_name

        input_tokens = input_file_name.split("\\")

        if not re.match(r"(INPUT\\input)(\d)(.txt)", input_file_name):

            print("Error: input path should be of the format INPUT\input#.txt")

            return -1



        output_file_name = arguments.output_file_name

        output_tokens = output_file_name.split("\\")

        if not re.match(r"(OUTPUT\\output)(\d)(.txt)", output_file_name):

            print("Error: output path should be of the format OUTPUT\output#.txt")

            return -1

    else:

        input_file_name = arguments.input_file_name

        input_tokens = input_file_name.split("/")

        if not re.match(r"(INPUT/input)(\d)(.txt)", input_file_name):

            print("Error: input path should be of the format INPUT/input#.txt")

            return -1



        output_file_name = arguments.output_file_name

        output_tokens = output_file_name.split("/")

        if not re.match(r"(OUTPUT/output)(\d)(.txt)", output_file_name):

            print("Error: output path should be of the format OUTPUT/output#.txt")

            return -1



    flag = arguments.flag

    # procedure_name = arguments.procedure_name





    try:

        map = read_from_file(input_file_name) # get the map

    except FileNotFoundError:

        print("input file is not present")

        return -1

    # print(map)

    

    solution_string = "" # contains solution



    solution_string = graphsearch(map, flag)

    write_flag = 1

    

    # call function write to file only in case we have a solution

    if write_flag == 1:

        write_to_file(output_file_name, solution_string)



if __name__ == "__main__":

    main()

