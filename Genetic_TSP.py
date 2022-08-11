import numpy as np, random, operator, pandas as pd, matplotlib.pyplot as plt
from math import dist
from CSVReader import CSVReader
from DR import DR
from SK import SK
from UAV import UAV
from GraphInfo import GraphInfo
from Node import Node
from Path import Path

INFINITE = 999999



class Fitness:    #각 개체에 대한 적합도를 관리

    def __init__(self, route):
        self.route = route
        self.distance = 0
        self.fitness= 0.0

    def get_routeDistance(self):    #거리 매기기
        if self.distance ==0:
            pathDistance = 0
            depot_node = tau_graph.get_node(0)
            last_depot_node = tau_graph.get_node(21)
            pathDistance += depot_node.get_distance(self.route[0])
            for i in range(0, len(self.route)-1):
                node = tau_graph.get_node(self.route[i])    #현 노드 기준으로
                pathDistance += node.get_distance(self.route[i+1])     #다음 노드까지 거리 적산
            pathDistance += last_depot_node.get_distance(self.route[-1])
            self.distance = pathDistance
        
        return self.distance
            
    def routeFitness(self):    #거리 값의 역순(작아야 좋으므로) 으로 점수 매기기
        if self.fitness == 0:
            self.fitness = 1 / float(self.get_routeDistance())
        return self.fitness


def createRoute(nodelist):     #random하게 sample route생성
    route = random.sample(nodelist, len(nodelist))     
    return route


def createRoute_2(individual, mutationRate):     #greedy input 기준으로 sample routes 생성, pre-mutate
    individual_2 = individual[:]
    for swapped in range(len(individual_2)):     #노드끼리 직접 swap 한다
        if(random.random() < mutationRate):
            swapWith = int(random.random() * len(individual_2))
            
            node1 = individual_2[swapped]
            node2 = individual_2[swapWith]
            
            individual_2[swapped] = node2
            individual_2[swapWith] = node1
    return individual_2


def initialPopulation(popSize, nodelist):     #초기 population 선정, pop size 100개, nodeList 입력
    population = []
    for i in range(0, popSize):
        population.append(createRoute(nodelist))     #nodelist(20개)를 100개로 재생성 함(증폭)
    return population


def initialPopulation_2(popSize, nodelist):     #초기 population 배양(greedy 기준으로), pop size 100개, nodeList 입력
    population = []
    population.append(nodelist)
    a = nodelist[:]
    cnt = int(popSize)-1

    while cnt > 0:
        b = createRoute_2(a, mutationRate=0.01)
        if b == a:
            continue
        cnt -= 1
        population.append(b)
    
    return population


def initialPopulation_greedy(truck_nodes_list):     #초기 popultation을 greedy 돈 것으로 줌, pop size 없음
    depot_node = tau_graph.get_node(0).node_num     # depot 0번 노드
    greedy_path = [depot_node]     #경로에 대한 번호 리스트
    visit_node = {0:True}
    node_cnt = len(truck_nodes_list)
    current_node = depot_node
    
    for j in range(node_cnt-1):
        min_dist = INFINITE
        next_node = None
        for i in range(node_cnt):
            if truck_nodes_list[i] in visit_node.keys():
                continue
            cur_distance = tau_graph.get_distance(current_node, truck_nodes_list[i])
            if cur_distance < min_dist:
                min_dist = cur_distance
                next_node = truck_nodes_list[i]            
        visit_node[next_node] = True
        greedy_path.append(next_node)
        current_node = next_node    #current_node를 출발점 기준으로 고정
    greedy_path.remove(0)
    
    return greedy_path     



def rankRoutes(population):     #population 100개에 대해서 순위를 매김
    fitnessResults = {}     #dict.
    for i in range(0,len(population)):
        fitnessResults[i] = Fitness(population[i]).routeFitness()     #dict.를 routeFitness로 채움, {i:routeFitness점수}
    #print(fitnessResults)
    a = sorted(fitnessResults.items(), key = operator.itemgetter(1), reverse = True)
    #print(a)
    return a     # 점수가 높은 순서대로 정렬


def selection(popRanked, eliteSize):     #가중 비례, 토너먼트, 랭크 처리된 pop을 입력
    selectionResults = []
    df = pd.DataFrame(np.array(popRanked), columns=["Index","Fitness"])     #행렬 만들어서 룰렛 돌리기
    df['cum_sum'] = df.Fitness.cumsum()     #누적 합
    df['cum_perc'] = 100*df.cum_sum/df.Fitness.sum()     #각각에 대해 가중치 반영(%)
    for i in range(0, eliteSize):     #선정된 엘리트 수만큼 반복
        selectionResults.append(popRanked[i][0])     #노드 path를 순서대로 넣는다
    
    for i in range(0, len(popRanked) - eliteSize):     #가장 좋은 값 반환(선택)
        pick = 100*random.random()
        for i in range(0, len(popRanked)):
            if pick <= df.iat[i,3]:     #패스트 인덱싱
                selectionResults.append(popRanked[i][0])
                break
    
    return selectionResults


def matingPool(population, selectionResults):     #셀렉션 된 개체(리스트)들 정리하여 mating pool에 넣기
    matingpool = []
    for i in range(0, len(selectionResults)):
        index = selectionResults[i]
        matingpool.append(population[index])
    return matingpool


def breed(parent1, parent2):     #번식, 노드 순서중에 랜덤하게 발췌하여 뒤집음
    child = []
    childP1 = []
    childP2 = []
    
    geneA = int(random.random() * len(parent1))
    geneB = int(random.random() * len(parent1))
    
    startGene = min(geneA, geneB)
    endGene = max(geneA, geneB)

    for i in range(startGene, endGene):     #특정 노드 구간만 랜덤으로 찢어서 랜덤 돌려서 교체
        childP1.append(parent1[i])
        
    childP2 = [item for item in parent2 if item not in childP1]     #crossover!!!

    child = childP1 + childP2
    return child

def breedPopulation(matingpool, eliteSize):
    children = []
    length = len(matingpool) - eliteSize
    pool = random.sample(matingpool, len(matingpool))

    for i in range(0,eliteSize):
        children.append(matingpool[i])
    
    for i in range(0, length):
        child = breed(pool[i], pool[len(matingpool)-i-1])     #선정된 pop size 만큼 breed 시행
        children.append(child)
    return children


def mutate(individual, mutationRate):     #변이, 로컬 벗어나기 위한 과정
    individual_2 = individual[:]
    for swapped in range(len(individual_2)):     #노드끼리 직접 swap 한다
        if(random.random() < mutationRate):
            swapWith = int(random.random() * len(individual_2))
            
            node1 = individual_2[swapped]
            node2 = individual_2[swapWith]
            
            individual_2[swapped] = node2
            individual_2[swapWith] = node1
    return individual_2


def mutatePopulation(population, mutationRate):
    mutatedPop = []
    mutatedPop.append(population[0])
    
    for ind in range(0, len(population)-1):
        mutatedInd = mutate(population[ind], mutationRate)
        mutatedPop.append(mutatedInd)
    return mutatedPop


#세대 승계
def nextGeneration(currentGen, eliteSize, mutationRate):     #입력 받아서 다음 세대 반복(함수 이어주기)
    popRanked = rankRoutes(currentGen)
    selectionResults = selection(popRanked, eliteSize)
    matingpool = matingPool(currentGen, selectionResults)
    children = breedPopulation(matingpool, eliteSize)
    nextGeneration = mutatePopulation(children, mutationRate)
    return nextGeneration


def geneticAlgorithm(population, popSize, eliteSize, mutationRate, generations):
    pop = initialPopulation(popSize, population)     #population 리스트 100개 생성
    print("Initial distance: " + str(1 / rankRoutes(pop)[0][1]))     #가장 첫번째 poputation 개체의 두번째 원소, 즉 dist.의 역순을 취함
    
    for i in range(0, generations):    #세대수 만큼 돌아가면서,
        pop = nextGeneration(pop, eliteSize, mutationRate)     #다음 세대 연산(pop 리스트와, 엘리트 사ㅣ즈와 와 변이율 필요), pop 리스트 안에서 모든 변이가 이루어짐
    
    print("Final distance: " + str(1 / rankRoutes(pop)[0][1]))     #가장 앞에 위치하게 되는(가장 fitness 점수가 좋은) 개체의 두번째 원소, 즉 dist.의 역순을 취함
    bestRouteIndex = rankRoutes(pop)[0][0]     #가장 첫번째 poputation 개체의 첫번째 원소, 즉 노드 순서의 인덱스
    bestRoute = pop[bestRouteIndex]     #해당 인덱스의 값
    return bestRoute


def geneticAlgorithm_greedy(population, popSize, eliteSize, mutationRate, generations):
    initial_greedy_route = initialPopulation_greedy(population)     # input greedy 리스트 1개 생성
    #print("initial_input: ", initial_greedy_route)
    pop = initialPopulation_2(popSize, initial_greedy_route)         # popSize 20개 만큼 증식
    print("Initial_best_distance: ", str(1 / rankRoutes(pop)[0][1]))     #가장 첫번째 poputation 개체의 두번째 원소, 즉 dist.의 역순을 취함
    
    for i in range(0, generations):    #세대수 만큼 돌아가면서,
        pop = nextGeneration(pop, eliteSize, mutationRate)     #다음 세대 연산(pop 리스트와, 엘리트 사이즈와 와 변이율 필요), pop 리스트 안에서 모든 변이가 이루어짐
    
    print("Final distance: " + str(1 / rankRoutes(pop)[0][1]))     #가장 앞에 위치하게 되는(가장 fitness 점수가 좋은) 개체의 두번째 원소, 즉 dist.의 역순을 취함
    bestRouteIndex = rankRoutes(pop)[0][0]     #가장 첫번째 poputation 개체의 첫번째 원소, 즉 노드 순서의 인덱스
    bestRoute = pop[bestRouteIndex]     #해당 인덱스의 값
    return bestRoute



reader = CSVReader() #CSV 파일 읽어옴
tau_graph = reader.load_csv('tau_v2.csv')
taup_graph = reader.load_csv('taup_v2.csv')
tau_graph.init_nearest_nodes()  #nearest 노드로 정렬 초기화
taup_graph.init_nearest_nodes()

truck_nodes_list = [node.node_num for node in tau_graph.nodes]     #노드 번호 리스트
truck_nodes_list.remove(0)
truck_nodes_list.remove(21)
'''
print("truck_nodes_list", truck_nodes_list)

temp_0 = initialPopulation_greedy(truck_nodes_list)
print("greedy_truck_nodes_list", temp_0)

temp = initialPopulation_2(10, temp_0)
print(temp)

temp_2 = rankRoutes(temp)
print(temp_2)

temp_3 = selection(temp_2, 5)
print(temp_3)

temp_4 = matingPool(temp, temp_3)
#print(temp_4)

temp_5 = breedPopulation(temp_4, 5)
#print(temp_5)

temp_6 = mutatePopulation(temp_5, 0.01)
print(temp_6)
'''

#geneticAlgorithm(population=truck_nodes_list, popSize=100, eliteSize=20, mutationRate=0.01, generations=1000)
geneticAlgorithm_greedy(population=truck_nodes_list, popSize=100, eliteSize=20, mutationRate=0.02, generations=1000)

