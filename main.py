#!/usr/bin/env python
# coding: utf-8
from importlib.resources import path
from program_uav_pushing import Program
from Path import Path
import random


program = Program()

'''
result_1= program.greedy_truck_node_list([3, 8, 15])
print("truck_node_list:", result_1)

result_2 = program.greedy_tsp(result_1)
print("greedy_path:", result_2)

result_3 = program.get_truck_cum_dist(result_2)
print("cum_dist_greedy_path:",result_3)
'''
'''
result_4, result_4b = program.get_dpt_node(0, 8, result_2)
print(result_4, result_4b)

result_5 = program.search_available_arv_nodes(8, result_4, result_4b, result_2)
print(result_5)

result_6 = program.get_arv_node(8, result_4, result_4b, result_2, result_5)
print(result_6)

#result_7 = program.choose_dpt_arv( 0, 8, result_2)
#print(result_7)
'''

#result_8 = program.calculate_DR_1_path([3, 15, 8], result_2)
#print("bike_a_path(bike_num, dpt_node, target_node, arv_node, bike_dist):", result_8)
# [1, 20, 18, 9, 19]
#input_list = [5, 9, 1, 4, 11, 12, 18, 6, 8, 2, 19, 7, 3, 17, 20, 10, 13, 14, 15, 16]
input_list = random.sample(range(1,21),20)
print('input_list:', input_list)

print('#######normal try########')
program.decision_best_path(input_list, False)
print('DR: ', len(program.DR_path_list))
for path in program.DR_path_list:
    print(path)
print('UAV: ', len(program.UAV_path_list))
for path in program.UAV_path_list:
    print(path)


# is last try로 마지막 쥐어짜서 넣기
program2 = Program()
print('#######is last try########')
program2.decision_best_path(input_list, True)
print('DR: ', len(program2.DR_path_list))
for path in program2.DR_path_list:
    print(path)
print('UAV: ', len(program2.UAV_path_list))
for path in program2.UAV_path_list:
    print(path)

print('Total time : ', program.min_total_time)
print('Total time with last try: ', program2.min_total_time)
