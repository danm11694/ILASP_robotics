#!/usr/bin/env python3
import rospkg
from os import listdir
from os.path import isfile

    
def remove_ex(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
        rm_index = []
        for i in range(len(lines)):
            if "#pos" in lines[i] or "#neg" in lines[i]: #an example was found
                rm_index.append(i)
        
        lines = [line for line in lines if lines.index(line) not in rm_index]
    
    with open(filename, "w") as f:
        f.writelines(lines)

            
            
                    







def main():
    path = rospkg.RosPack().get_path("task_reasoning")
    ilasp_files = [f for f in listdir(path+"/ilasp/") if isfile(path+"/ilasp/"+f)]
    for f in ilasp_files:
        remove_ex(path + "/ilasp/" + f)








if __name__ == '__main__':
    main()



