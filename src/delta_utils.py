import numpy as np
import math
def linspace_vec(history_data1, history_data2, interval):
    vec1 = np.array(history_data1[1:4])
    vec2 = np.array(history_data2[1:4])
    start_time = history_data1[0]
    end_time = history_data2[0]

    diff_time = end_time - start_time
    section_num = math.ceil(diff_time / interval)
    
    add_history_list = []

    if section_num >= 2:
        vec = (vec2 - vec1) / section_num
        new_interval = diff_time / section_num
        
        for i in range(1, section_num):
            add_vec  = vec1       + (vec * i)
            add_time = start_time + (new_interval * i)
            add_history_data = [round(add_time,0), round(add_vec[0],2), round(add_vec[1],2), round(add_vec[2],2)]
            add_history_list.append(add_history_data)
    return add_history_list

def linear_interpolation(history):
    record_list = []
    record_num = len(history)
    current_data = None
    next_data = None

    
    for i in history:
        i[0] = i[0] * 2.0
        # i[1] = i[1] * 2.5
        # i[2] = i[2] * 2.5
    

    for i in range(record_num - 1):
        current_data = history[i]
        next_data = history[i+1]
        add_data_list = linspace_vec(current_data, next_data, 1)
        record_list.append(current_data)
        record_list.extend(add_data_list)
    record_list.append(next_data)
    return record_list
