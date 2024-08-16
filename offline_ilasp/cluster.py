from itertools import count
from operator import index, mod
import re
import sys
import numpy as np
import time
import math
from sklearn.neighbors import NearestNeighbors
from sklearn.cluster import KMeans
from scipy.spatial.distance import cdist
from matplotlib import pyplot as plt

KMeans()

class Counter(object):
    def __init__(self):
        self.times = []












def generate_pylasp(action):
    with open("ilasp/params.las", "r") as f:
        pylasp = f.readlines()

    return [l.replace("grasp", action) for l in pylasp]








def mixed_dist(x, y):
    # e_max = 1.
    e_max = 39.26705099020227
    h_max = 0.16666666666666666
    full_max = 1.4110291415247098
    #for fluents
    # hamm = cdist([x], [y], metric='hamming')
    hamm = cdist([x[:30]], [y[:30]], metric='hamming') / h_max
    #for poly coeffs
    eucl = cdist([x[30:]], [y[30:]], metric='euclidean') / e_max

    # return hamm
    # return eucl
    return math.sqrt(hamm**2 + eucl**2)# / full_max


def h_dist(x, y):
    # h_max = 1.
    h_max = 0.13333333333333333
    #for fluents
    # hamm = cdist([x], [y], metric='hamming')
    hamm = cdist([x[:30]], [y[:30]], metric='hamming') / h_max

    return hamm
    # return eucl
    # return math.sqrt(hamm**2 + eucl**2)


def e_dist_short(x, y):
    # h_max = 1.
    e_max = 122.2718997988127
    #for fluents
    # hamm = cdist([x], [y], metric='hamming')
    eucl = cdist([x[30:]], [y[30:]], metric='euclidean')# / e_max

    return eucl
    # return eucl
    # return math.sqrt(hamm**2 + eucl**2)


def e_dist_long(x, y):
    # h_max = 1.
    e_max = 39.26705099020227
    #for fluents
    # hamm = cdist([x], [y], metric='hamming')
    eucl = cdist([x[30:]], [y[30:]], metric='euclidean')# / e_max

    return eucl
    # return eucl
    # return math.sqrt(hamm**2 + eucl**2)







def main():

    MODE_BIAS = False
    PYLASP = False
    CONSIDER_NOISE = True
    NOISE_INF_THRESHOLD = 1 #-1: all examples have noise; 1: only examples with prob < 1 have noise; IRRELEVANT FOR NO NOISE!
    HARD_THRESHOLD = 2 #0: all neg examples considered; 1: only neg with 100% probability considered; 2: only pos examples considered
    ILASP_FOLDER = "./ilasp/new_features/above_mean_pos_noise_inf/"

    coeff_list = []
    poly_list = []
    init_time = time.time()
    tests = ["full", "blue_red", "fail_test"]

    ## BUILD FEATURES coeff_list, using Meli & Fiorini, RA-L 2021

    # kinematic features
    for test in tests:
        with open(test + "/coeffs.npy", 'rb') as f:
            coeffs = np.load(f)
            for i in range(np.shape(coeffs)[0]):
                poly_list.append(coeffs[i])

    #semantic features
    contexts = []
    for test in tests:
        f = open(test + "/fluents.txt", 'r')
        coeffs = np.zeros(14) 
        placed = [] 
        context = ""
        action = ""
        lines = f.readlines()
        actions = [lines[i] for i in range(len(lines)) if (i+1 < len(lines) and "TIMESTEP" in lines[i+1]) or i == len(lines)-1]
        j = 0
        for x in lines:
            if "TIMESTEP" in x:
                #NOT BEGINNING OF TIME -> APPEND COEFFS AND CONTEXTS
                if not " 1\n" in x and not " 1 " in x:
                    #placed
                    for p in placed:
                        l = re.split(r"[(,)]", p)
                        if l[2] in relevant_rings_psm1:
                            coeffs[6] = 1
                        if l[2] in relevant_rings_psm2:
                            coeffs[13] = 1
                        if l[4] in relevant_pegs_psm1:
                            coeffs[6] = 1
                        if l[4] in relevant_pegs_psm2:
                            coeffs[13] = 1

                    #swap to avoid asymmetries in coefficients (both psms are equal)
                    for i in range(7):
                        if coeffs[i] == 0:
                            coeffs[i] = coeffs[i+7]
                            coeffs[i+7] = 0 

                    coeff_list.append(coeffs)
                    context = context.split("\n")
                    # action = context[-1]
                    context = context[:-1]
                    contexts.append(context)
                #for placed
                relevant_rings_psm1 = []
                relevant_rings_psm2 = []
                relevant_pegs_psm1 = []
                relevant_pegs_psm2 = []
                placed = []
                #for placed
                coeffs = np.zeros(14)
                context = "" 
                action = actions[j] 
                j += 1
            else:
                x = x.replace("\n", ".\n")
                context += x
                if "at(psm1,ring" in x:
                    if ("center" in action or "psm1" in action):
                        coeffs[0] = 1
                    #is placed related to this arm?
                    where_object = x.find("at(psm1,ring")
                    where_bracket = x[where_object:].find(")")
                    color = x[where_object+13: where_object + where_bracket]
                    if color not in relevant_rings_psm1:
                        relevant_rings_psm1.append(color)
                if "at(psm2,ring" in x:
                    if ("center" in action or "psm2" in action):
                        coeffs[7] = 1
                    #is placed related to this arm?
                    where_object = x.find("at(psm2,ring")
                    where_bracket = x[where_object:].find(")")
                    color = x[where_object+13: where_object + where_bracket]
                    if color not in relevant_rings_psm2:
                        relevant_rings_psm2.append(color)
                if "at(psm1,peg" in x:
                    if ("center" in action or "psm1" in action):
                        coeffs[1] = 1
                    #is placed related to this arm?
                    where_object = x.find("at(psm1,peg")
                    where_bracket = x[where_object:].find(")")
                    color = x[where_object+12: where_object + where_bracket]
                    if color not in relevant_pegs_psm1:
                        relevant_pegs_psm1.append(color)
                if "at(psm2,peg" in x:
                    if ("center" in action or "psm2" in action):
                        coeffs[8] = 1
                    #is placed related to this arm?
                    where_object = x.find("at(psm2,peg")
                    where_bracket = x[where_object:].find(")")
                    color = x[where_object+12: where_object + where_bracket]
                    if color not in relevant_pegs_psm2:
                        relevant_pegs_psm2.append(color)
                if "at(psm1,center" in x:
                    if ("center" in action or "psm1" in action):
                        coeffs[2] = 1
                    #is placed related to this arm?
                    where_object = x.find("at(psm1,center")
                    where_bracket = x[where_object:].find(")")
                    color = x[where_object+15: where_object + where_bracket]
                    if color not in relevant_rings_psm1:
                        relevant_rings_psm1.append(color)
                if "at(psm2,center" in x:
                    if ("center" in action or "psm2" in action):
                        coeffs[9] = 1
                    #is placed related to this arm?
                    where_object = x.find("at(psm2,center")
                    where_bracket = x[where_object:].find(")")
                    color = x[where_object+15: where_object + where_bracket]
                    if color not in relevant_rings_psm2:
                        relevant_rings_psm2.append(color)
                if "closed_gripper(psm1" in x:
                    if ("center" in action or "psm1" in action):
                        coeffs[5] = 1
                if "closed_gripper(psm2" in x:
                    if ("center" in action or "psm2" in action):
                        coeffs[12] = 1
                if "placed" in x:
                    placed.append(x)
            
        #placed
        for p in placed:
            l = re.split(r"[(,)]", p)
            if l[2] in relevant_rings_psm1:
                coeffs[6] = 1
            if l[2] in relevant_rings_psm2:
                coeffs[13] = 1
            if l[4] in relevant_pegs_psm1:
                coeffs[6] = 1
            if l[4] in relevant_pegs_psm2:
                coeffs[13] = 1
        #swap to avoid asymmetries in coefficients (both psms are equal)
        for i in range(7):
            if coeffs[i] == 0:
                coeffs[i] = coeffs[i+7]
                coeffs[i+7] = 0 
        coeff_list.append(coeffs)
        context = context.split("\n")
        context = context[:-1]
        contexts.append(context)

        f.close()
    
    coeff_list = np.array(coeff_list)
    # print(coeff_list)

    flags = []
    for poly in poly_list:
        flag = np.array([])
        for i in range(int(np.shape(poly)[0]/6)):
            if np.max(poly[6*i:6*i+6]) > 1.:
                flag = np.append(flag, 0)
            else:
                flag = np.append(flag, 0)
        flags.append(flag)
    
    flags = np.array(flags)
    coeff_list = np.append(coeff_list, flags, 1)
    coeff_list = np.append(coeff_list, poly_list, 1)

    ## GROUND TRUTH
    #full
    mr_indices = [2 + 8*i for i in range(4)] #move_ring
    mc_indices = [5 + 8*i for i in range(4)] #move_center
    mp_indices = [8 + 8*i for i in range(4)] #move_peg
    r_indices = [0, 1] #release_start
    r_indices += [9 + 8*i for i in range(4)] #release_final
    r_indices += [7 + 8*i for i in range(4)] #release_transfer
    g_indices = [3 + 8*i for i in range(4)] #grasp
    g_indices += [6 + 8*i for i in range(4)] #grasp_transfer
    e_indices = [4 + 8*i for i in range(4)] #extract
    #blue_red
    mr_indices += [36  + 5*i for i in range(3)] #move_ring
    mp_indices += [39  + 5*i for i in range(3)] #move_peg
    r_indices += [34 , 35 ] #release_start
    r_indices += [40  + 5*i for i in range(3)] #release_final
    g_indices += [37  + 5*i for i in range(3)] #grasp
    e_indices += [38  + 5*i for i in range(3)] #extract
    #fail_test
    mr_indices += [53, 58, 65] #move_ring
    mc_indices += [60] #move_center
    mp_indices += [56, 63, 67] #move_peg
    r_indices += [51, 52, 57, 62, 64, 68] #release
    g_indices += [54, 59, 61, 66] #grasp
    e_indices += [55] #extract

    np.set_printoptions(threshold=sys.maxsize)

    ## CLUSTER
    #short actions
    short_indices = g_indices + r_indices + e_indices
    MAX_NUMBER = len(short_indices)-1
    short_indices.sort()
    nbrs = NearestNeighbors(n_neighbors=len(short_indices)-1, algorithm='auto', metric=h_dist).fit(coeff_list[short_indices])
    distances, indices = nbrs.kneighbors()
    # print(nbrs.kneighbors_graph(coeff_list[short_indices]).toarray())
    short_couples = []
    for j in range(len(distances)):
        where_mins = np.argpartition(distances[j], MAX_NUMBER-1)[:MAX_NUMBER]
        for where_min in where_mins:
            # distances d_{ij}
            short_couples.append((short_indices[j], short_indices[indices[j][where_min]], 1-distances[j][where_min]))

    #long actions
    long_indices = mr_indices + mp_indices + mc_indices
    # MAX_NUMBER = 15
    MAX_NUMBER = len(long_indices)-1
    long_indices.sort()
    nbrs = NearestNeighbors(n_neighbors=len(long_indices)-1, algorithm='auto', metric=mixed_dist).fit(coeff_list[long_indices])
    distances, indices = nbrs.kneighbors()
    long_couples = []
    for j in range(len(distances)):
        where_mins = np.argpartition(distances[j], MAX_NUMBER-1)[:MAX_NUMBER]
        for where_min in where_mins:
            # distances d_{ij}
            long_couples.append((long_indices[j], long_indices[indices[j][where_min]], 1-distances[j][where_min]))





    ## MULTIPLE ACTIONS: CONSIDER DISTANCE FROM FIRST INSTANCES OF ACTIONS
    #short actions [release, grasp, extract]: first occurrency per each dataset
    short_couples_final = [[0,0,1], [3,3,1], [4,4,1]]
    for i in range(len(contexts)):
        if i in [0,3,4]:
            continue
        try:
            relevant_list = [[s[1], s[0], s[-1]] for s in short_couples if s[0]==i and s[1] in [0,3,4]]
            relevant_list.sort(key=lambda couple : -couple[-1])
            short_couples_final.append(relevant_list[0])
        except:
            pass

    #long actions [move_ring, move_center, move_peg]
    long_couples_final = [[2,2,1], [5,5,1], [8,8,1]]
    for i in range(len(contexts)):
        if i in [2,5,8]:
            continue
        try:
            relevant_list = [[s[1], s[0], s[-1]] for s in long_couples if s[0]==i and s[1] in [2,5,8]]
            relevant_list.sort(key=lambda couple : -couple[-1])
            long_couples_final.append(relevant_list[0])
        except:
            pass
            
        
    
    ## GENERATE ILASP EXAMPLES FOR ACTIONS
    id_release = []
    id_grasp = []
    id_extract = []
    id_move_ring = []
    id_move_peg = []
    id_move_center = []

    ex_lists = [[], [], [], [], [], []] #ordered: grasp release extract movering movepeg movecenter
    ex_count = 1
    values = []
    mean_value = [np.mean([s[2] for s in short_couples_final if s[0] in g_indices]), np.mean([s[2] for s in short_couples_final if s[0] in r_indices]), np.mean([s[2] for s in short_couples_final if s[0] in e_indices]), np.mean([s[2] for s in long_couples_final if s[0] in mr_indices]), np.mean([s[2] for s in long_couples_final if s[0] in mp_indices]), np.mean([s[2] for s in long_couples_final if s[0] in mc_indices])]
    std = [np.std([s[2] for s in short_couples_final if s[0] in g_indices]), np.std([s[2] for s in short_couples_final if s[0] in r_indices]), np.std([s[2] for s in short_couples_final if s[0] in e_indices]), np.std([s[2] for s in long_couples_final if s[0] in mr_indices]), np.std([s[2] for s in long_couples_final if s[0] in mp_indices]), np.std([s[2] for s in long_couples_final if s[0] in mc_indices])]
    for i in range(len(short_couples_final)):
        if short_couples_final[i][0] in [0, 3, 4]:
            value = short_couples_final[i][2]
            num = value
            if short_couples_final[i][0] in g_indices:
                ex_index = 0
                first_action = "GRASP"
                if value < mean_value[ex_index]:
                    continue
                if id_grasp == []:
                    id_grasp.append((short_couples_final[i][0], 1))
                id_grasp.append((short_couples_final[i][1], short_couples_final[i][2]))
            elif short_couples_final[i][0] in r_indices:
                ex_index = 1
                first_action = "RELEASE"
                if value < mean_value[ex_index]:
                    continue
                if id_release == []:
                    id_release.append((short_couples_final[i][0], 1))
                id_release.append((short_couples_final[i][1], short_couples_final[i][2]))
            elif short_couples_final[i][0] in e_indices:
                ex_index = 2
                first_action = "EXTRACT"
                if value < mean_value[ex_index]:
                    continue
                if id_extract == []:
                    id_extract.append((short_couples_final[i][0], 1))
                id_extract.append((short_couples_final[i][1], short_couples_final[i][2]))
            if short_couples_final[i][1] in g_indices:
                second_action = "GRASP"
            elif short_couples_final[i][1] in r_indices:
                second_action = "RELEASE"
            elif short_couples_final[i][1] in e_indices:
                second_action = "EXTRACT"

            values.append(value)
            context = contexts[int(short_couples_final[i][1])]
            arm_id = int(context[-1][context[-1].index("psm")+3])
            # for _ in range(int(num*100)):
            ex_line = "%" + str(num) + "\n"
            if num != NOISE_INF_THRESHOLD and CONSIDER_NOISE:
                ex_line += "#pos(ex" + str(ex_count) + "@" + str(int(num*100)) + ", {action" + str(ex_index) + "(psm" + str(arm_id) + ")}, {action" + str(ex_index) + "(psm" + str(3-arm_id) + ")}, {" + " ".join(context) + "}).\n"
            else:
                ex_line += "#pos(ex" + str(ex_count) + ", {action" + str(ex_index) + "(psm" + str(arm_id) + ")}, {action" + str(ex_index) + "(psm" + str(3-arm_id) + ")}, {" + " ".join(context) + "}).\n"
            ex_count += 1
            ex_lists[ex_index].append(ex_line)
            list_indices = [0,1,2]
            list_indices.remove(ex_index)
            for idx in list_indices:
                # if idx != ex_index:
                ex_line = "%" + str(num) + "\n"
                if num != NOISE_INF_THRESHOLD and CONSIDER_NOISE:
                    ex_line += "#pos(ex" + str(ex_count) + "@" + str(int(num*100)) + ", {}, {action" + str(idx) + "(_)}, {" + " ".join(context) + "}).\n"
                else:
                    ex_line += "#pos(ex" + str(ex_count) + ", {}, {action" + str(idx) + "(_)}, {" + " ".join(context) + "}).\n"
                ex_count += 1
                if num >= HARD_THRESHOLD:
                    ex_lists[idx].append(ex_line)

    MAX_NUMBER = len(long_indices)-1
    values = []
    # long_couples_final = np.array(long_couples_final)
    for i in range(len(long_couples_final)):
        if long_couples_final[i][0] in [2,5,8]:
            value = long_couples_final[i][2]
            num = value
            if long_couples_final[i][0] in mr_indices:
                ex_index = 3
                first_action = "MOVE RING"
                if value < mean_value[ex_index]:
                    continue
                if id_move_ring == []:
                    id_move_ring.append((long_couples_final[i][0], 1))
                id_move_ring.append((long_couples_final[i][1], long_couples_final[i][2]))
            elif long_couples_final[i][0] in mp_indices:
                ex_index = 4
                first_action = "MOVE PEG"
                if value < mean_value[ex_index]:
                    continue
                if id_move_peg == []:
                    id_move_peg.append((long_couples_final[i][0], 1))
                id_move_peg.append((long_couples_final[i][1], long_couples_final[i][2]))
            elif long_couples_final[i][0] in mc_indices:
                if value < mean_value[ex_index]:
                    continue
                ex_index = 5
                first_action = "MOVE CENTER"
                if id_move_center == []:
                    id_move_center.append((long_couples_final[i][0], 1))
                id_move_center.append((long_couples_final[i][1], long_couples_final[i][2]))
            if long_couples_final[i][1] in mr_indices:
                second_action = "MOVE RING"
            elif long_couples_final[i][1] in mp_indices:
                second_action = "MOVE PEG"
            elif long_couples_final[i][1] in mc_indices:
                second_action = "MOVE CENTER"

            values.append(value)
            context = contexts[int(long_couples_final[i][1])]
            arm_id = int(context[-1][context[-1].index("psm")+3])
            ex_line = "%" + str(num) + "\n"
            if ex_index != 5:
                # for _ in range(int(num*100)):
                if num != NOISE_INF_THRESHOLD and CONSIDER_NOISE:
                    ex_line += "#pos(ex" + str(ex_count) + "@" + str(int(num*100)) + ", {action" + str(ex_index) + "(psm" + str(arm_id) + ")}, {action" + str(ex_index) + "(psm" + str(3-arm_id) + ")}, {" + " ".join(context[:-1]) + "}).\n"
                else:
                    ex_line += "#pos(ex" + str(ex_count) + ", {action" + str(ex_index) + "(psm" + str(arm_id) + ")}, {action" + str(ex_index) + "(psm" + str(3-arm_id) + ")}, {" + " ".join(context[:-1]) + "}).\n"
                ex_count += 1
                ex_lists[ex_index].append(ex_line)
                list_indices = [3,4,5]
                list_indices.remove(ex_index)
                for idx in list_indices:
                #     if idx != ex_index:
                    ex_line = "%" + str(num) + "\n"
                    if num != NOISE_INF_THRESHOLD and CONSIDER_NOISE:
                        if idx != 5:
                            ex_line += "#pos(ex" + str(ex_count) + "@" + str(int(num*100)) + ", {}, {action" + str(idx) + "(_)}, {" + " ".join(context) + "}).\n"
                        else:
                            ex_line += "#pos(ex" + str(ex_count) + "@" + str(int(num*100)) + ", {}, {action" + str(idx) + "(_,_)}, {" + " ".join(context) + "}).\n"
                    else:
                        if idx != 5:
                            ex_line += "#pos(ex" + str(ex_count) + ", {}, {action" + str(idx) + "(_)}, {" + " ".join(context) + "}).\n"
                        else:
                            ex_line += "#pos(ex" + str(ex_count) + ", {}, {action" + str(idx) + "(_,_)}, {" + " ".join(context) + "}).\n"
                    ex_count += 1 
                    if num >= HARD_THRESHOLD:
                        ex_lists[idx].append(ex_line)
            else:
                ex_line += "#pos(ex" + str(ex_count) + "@" + str(int(num*100)) + ", {action" + str(ex_index) + "(psm" + str(arm_id) + ",psm" + str(3-arm_id) + ")}, {action" + str(ex_index) + "(psm" + str(3-arm_id) + ",psm" + str(arm_id) + ")}, {" + " ".join(context) + "}).\n"
                ex_count += 1
                ex_lists[ex_index].append(ex_line)
                list_indices = [3,4,5]
                list_indices.remove(ex_index)
                for idx in list_indices:
                    # if idx != ex_index:
                    ex_line = "%" + str(num) + "\n"
                    if num != NOISE_INF_THRESHOLD and CONSIDER_NOISE:
                        ex_line += "#pos(ex" + str(ex_count) + "@" + str(int(num*100)) + ", {}, {action" + str(idx) + "(_)}, {" + " ".join(context) + "}).\n"
                    else:
                        ex_line += "#pos(ex" + str(ex_count) + ", {}, {action" + str(idx) + "(_)}, {" + " ".join(context) + "}).\n"
                    ex_count += 1
                    if num >= HARD_THRESHOLD:
                        ex_lists[idx].append(ex_line)

    for idx in range(len(ex_lists)):
        print("ACTION " + str(idx))
        print(ex_lists[idx])
        print("========================")



    ## GENERATE MORE PROBABLE EFFECTS OF ACTIONS, DEPENDING ON ACTION IDENTIFICATION

    ex_count = 1
    at_ring = []
    at_peg = []
    at_center = []
    closed = []
    placed = []
    action_list = [id_extract, id_grasp, id_release, id_move_center, id_move_peg, id_move_ring]
    fluents = dict()
    for k in ["at_ring", "at_peg", "at_center", "placed", "closed_gripper"]:
        fluents[k] = np.zeros((6, 2)) #row = action; column = added / removed


    fluent_prev = []
    split_fluent = []
    diff_contexts = []
    for j in range(len(contexts)-1):
        if j == 33 or j == 50: #end of task instance -> shall not consider delayed effects!
            continue
        k_a = -2
        #FIND ACTION
        for i in range(len(action_list)):
            where_action = [el for el in action_list[i] if el[0] == j]
            if len(where_action) > 0:
                k_a = i
                break
        if k_a == -2: #action not classified -> skip to next timestep
            continue
        #FIND FLUENT
        for el in contexts[j+1][:-1]:
            if el not in contexts[j]: #initiated
                where_bracket = el.index("(")
                name_f = el[:where_bracket]
                which_f = [k for k in fluents.keys() if name_f in k]
                if len(which_f) > 1:
                    if "ring" in el:
                        which_f = ["at_ring"]
                    elif "peg" in el:
                        which_f = ["at_peg"]
                    else:
                        which_f = ["at_center"]
                if len(which_f) == 1:
                    fluents[which_f[0]][k_a, 0] += 1
        for el in contexts[j][:-1]:
            if el not in contexts[j+1]: #terminated
                where_bracket = el.index("(")
                name_f = el[:where_bracket]
                which_f = [k for k in fluents.keys() if name_f in k]
                if len(which_f) > 1:
                    if "ring" in el:
                        which_f = ["at_ring"]
                    elif "peg" in el:
                        which_f = ["at_peg"]
                    else:
                        which_f = ["at_center"]
                if len(which_f) == 1:
                    fluents[which_f[0]][k_a, 1] += 1
    
    action_list = ["extract", "grasp", "release", "move_center", "move_peg", "move_ring"]
    for fluent in fluents.keys():
        print(fluent)
        print("EFFECT PROABILITY")
        for i in range(len(fluents[fluent])):
            print(action_list[i] + str(fluents[fluent][i]))
        print("==========================")


if __name__ == '__main__':
    main()

    


