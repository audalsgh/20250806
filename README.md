# 33일차 - 경로계획 알고리즘

## 1. 완전한 지식 스펙트럼을 갖는 세계
자율주행 경로 문제
- 기계가 세상을 완벽히 알고있을때(=경로를 알때), 어떻게 최적의 경로를 구하는가?
- 자율주행 내비게이션의 기본인, 경로 문제를 풀때는 지도를 그래프로 나타낸다.
- 자기자신의 칸으로는 이동불가, 대각선으로 이동불가 가정.

  <img width="873" height="479" alt="image" src="https://github.com/user-attachments/assets/3d651bd6-4e14-4e7e-a97b-20268d67c111" />

지도, 경로를 아는 세계에서의 알고리즘들
- 깊이 우선 탐색(DFS) = "LIFO로, 노드 u에서 목표를 찾는 함수는, u의 이웃 v에서 목표를 찾는 함수를 호출하는 재귀적, 스택 알고리즘"
- 너비 우선 탐색(BFS) = "FIFO로, 먼저 들어온 노드부터 처리하는 반복문, 큐 알고리즘, 모든 경로의 비용이 같다면 최단 거리를 얻을수있다."
- 다익스트라(Dijkstra) 알고리즘 = "경로마다 비용이 다를때, 시작점부터 출발하여 비용이 가장 적은 노드부터 찾아냄."
- 에이스타(A*) 알고리즘 = "지금까지 온 경로의 비용 + 앞으로 갈 경로의 비용의 합이 제일 적은 경로를 찾아냄." 다익스트라에 방향성이 더해진 것.<br>
-> 데이터 구조에따라 알고리즘의 논리가 결정됨.<br>
<img width="635" height="677" alt="image" src="https://github.com/user-attachments/assets/b3bd60af-a4fb-496b-8ada-cc3bd7f31f70" />

`dfs_recursive` 함수를 완성하세요. 이 함수는 자기 자신을 호출하여 이웃 노드를 탐색해야 합니다.
- **Base Case (종료 조건):** 현재 노드가 목표이거나, 벽이거나, 이미 방문한 곳이면 탐색을 중단해야 합니다.
- **Recursive Step (재귀 단계):** 유효한 이웃 노드에 대해 `dfs_recursive` 함수를 다시 호출합니다.

```python
def dfs_recursive_helper(grid, current, goal, visited, path_order):
    rows, cols = len(grid), len(grid[0])
    row, col = current

    # 종료 조건 1: current가 그리드 밖이거나, 벽인 경우
    if not (0 <= row < rows and 0 <= col < cols and grid[row][col] == 0):
        return False

    # 종료 조건 2: 이미 방문한 경우
    if current in visited:
        return False
        
    # 현재 노드 방문 처리
    # 현재 노드를 visited 집합에 추가하고, path_order 기록,
    visited.add(current)
    path_order.append(current)

    # 종료 조건 3: 목표 도달시 종료.
    if current == goal:
        return True

    # 재귀 단계(Recursive Step) : 4방향 이웃에 대해 재귀 호출
    # 4방향(상·우·하·좌)으로 이웃 노드를 순회하며 재귀 호출
    # 이웃 호출 중 하나라도 True(목표 발견)를 반환하면 상위 호출도 True를 반환하며 탐색 종료
    for dr, dc in [(0, 1), (1, 0), (0, -1), (-1, 0)]: # 우, 하, 좌, 상
        if dfs_recursive_helper(grid, (row + dr, col + dc), goal, visited, path_order):
            return True # 목표를 찾았다면 재귀 중단

    return False  # 이 경로에서는 목표를 찾지 못함
```
## 2. 지역적 지식(Local)과 이론적 깊이
이번엔 지도 일부만 아는 상황
- 최단 경로 문제의 재귀적 정의 : u에서 목표 G까지 최단 경로 비용은, u의 모든 이웃 v에 대해 (u->v 비용 + v->G까지 최단 경로 비용) 중 최솟값.
- 동적 계획법 (Dynamic Programming) : 부분 문제와 최적 부분 구조 문제에서, 반복적 완화 과정을 통해 재귀의 비효율성을 해결하는 기법.
- Top-Down (메모이제이션) = 재귀 구조는 유지하되, 한번 계산한 부분 문제의 답은 어딘가에 저장(메모이제이션, Memoization)해두고, 다시 필요하게 되면 계산없이 변수처럼 가져와 쓰는것.
- Bottom-Up (타뷸레이션) =  가장 작은 부분 문제부터 차례로 답을 계산하여 테이블을 채워간 후, 더 큰 문제 해결에 사용하는 기법. "벨만-포드"도 여기에 속함.

벨만 포드 알고리즘
- 모든 노드로의 최단 경로는 최대 N-1개의 엣지(경로)로 이뤄져있다.
- 이론적 기반은 벨만 방정식.
- 벨만 방정식의 연속 시간 버전이 바로 "최적 제어 이론의 HJB방정식"임.

<img width="398" height="372" alt="image" src="https://github.com/user-attachments/assets/b4c853a5-6933-4919-80e9-652dd72d2fcf" />

-> 첫 반복에서 이미 모든 최단 거리가 수렴했고, 그 이후에는 더 짧은 경로가 없으므로 업데이트가 발생하지 않습니다.

```python
# 벨만-포드 함수 구현

def bellman_ford(edges, num_v, source):
    # 1. 거리 초기화 source는 0, 나머지는 ∞ (float('inf') 사용)
    distances = [float('inf')] * num_v
    distances[source] = 0

    # 2. V-1 번 반복하며 엣지 완화
    for i in range(num_v - 1):
        print(f"--- 반복 {i+1} ---")
        # [문제] 모든 엣지에 대해 반복
        for u, v, w in edges:
            # 문제] 만약 노드 u까지의 현재 거리가 무한대가 아니고, "노드 u까지의 거리에, u에서 v로 가는 엣지의 가중치 w를 더한 값"이 노드 v까지의 현재 거리보다 작다면
            if distances[u] != (float('inf')) and (distances[u] + w < distances[v]):
                print(f"  (엣지 {u}->{v}) 노드 {v}의 거리 갱신: {distances[v]:.2f} -> {distances[u] + w:.2f}")
                # 문제] 노드 v까지의 최단 거리를 이 새로운, 더 짧은 거리로 업데이트하라
                distances[v] = distances[u] + w
        print(f"  현재 거리: {[f'{d:.2f}' for d in distances]}")

    return distances
```
## 3. 지식이 전혀 없는 세계
오직 시행착오로만 경로를 배워야하는 "강화 학습"의 세계
- 에이전트(Agent) : 학습하고 결정하는 주체, 자율주행차
- 환경 (Environment) : 에이전트가 상호작용하는 세상, 그리드 지도
- 상태 (State) : 에이전트의 현재 상황, 좌표 (row, col)
- 행동 (Action) : 에이전트가 할수있는 선택 (상,하,좌,우 이동)
- 보상 (Reward) : 환경이 주는 피드백으로, 좋은 행동에는 양수+ / 나쁜 행동에는 음수-
-> 에이전트는 미래에 받을 보상을 최대화하는 "정책(Policy)"를 학습하는게 목표!

- 활용 (Exploitation) : 지금까지의 경험상 가장 좋았던 행동만 하는것. ( 가봤던 식당만 가는것 )
- 탐색 (Exploration) : 더 좋은 선택지가 있을수도 있으니, 새로운 시도를 해보는것. ( 한번도 안가본 식당을 가는것 )
-> 좋은 강화학습이란, 초기에는 탐색을 많이 하다가 점차 확신이 생기면 활용의 비중을 높힌다.

가치 기반 학습 (Q-Learning)
- 에이전트가 환경을 탐색하며 얻은 보상으로부터, 상태–행동 가치 함수 Q(s,a) 를 학습하여 최적 정책을 찾아가는 방법
- 가치 함수(value function): 각 상태 𝑠 에서 기대되는 누적 보상(가치)을 나타냄
- Q함수(Q-value): 상태𝑠 에서 행동𝑎 를 선택했을 때 기대되는 누적 보상
-> 에이전트는 환경과 상호작용하는 경험으로부터 Q함수를 업데이트하는데, 이 역시 벨만 방정식으로 학습하는 것이다.

**실습 : 미리 학습된 정책 사용, 초기 출발점 환경은 정해져있음**
```python
# 학습된 정책과 환경 설정

# 강화학습을 통해 이미 학습이 완료된 최적 정책이라고 가정합니다.
policy_grid = [
    ['>', '>', '>', 'v', '#'],
    ['^', '#', '>', 'v', '#'],
    ['^', '#', '>', 'v', 'G'],  # ['^', '#', '>', '>', 'G'] 로 이어지게 수정한다면 도착가능.
    ['^', '<', '<', '<', '<'],
]

start_pos = (3, 0)  # 출발은 4번째 행의 1번 인덱스 '^'
```

<img width="684" height="508" alt="image" src="https://github.com/user-attachments/assets/aac417cc-b7be-4f0d-8750-584c6dcd6e74" /><br>
-> 학습된 정책이 'G'까지 연결되었다면 20번안에 도착할수있으나, 

<img width="1447" height="488" alt="image" src="https://github.com/user-attachments/assets/764afda4-11b6-4de8-9ed0-21331d413ea0" /><br>
-> 'G'까지 연결되지 않은 정책 사용시, 루프에 빠지게된다.

```python
# 정책 실행 시뮬레이터 구현 및 실행

def execute_policy(policy, start):
    row, col = start
    path = [(row, col)]

    # 최대 20번만 이동하도록 제한 (무한 루프 방지)
    for _ in range(20):
        # [문제] 현재 에이전트가 있는 위치의 정책(가야 할 방향)을 확인합니다.
        # ex) policy[3][0] = 4번째 행의 1번 인덱스 '^'가 출발점.
        action = policy[row][col]
        
        # [문제] 만약 현재 위치가 'G' (목표 지점)이면
        if action == 'G':
            # [문제] "목표 도달!" 이라고 출력하고
            # [문제] 지금까지 지나온 길(경로)을 반환하고 함수를 끝냅니다.
            print("목표 도달!")
            return path   

        # [문제] 만약 정책이 '>' (오른쪽)이면, 오른쪽으로 한 칸 이동합니다.
        # [문제] 아니면 만약 정책이 '<' (왼쪽)이면, 왼쪽으로 한 칸 이동합니다.
        # [문제] 아니면 만약 정책이 'v' (아래쪽)이면, 아래쪽으로 한 칸 이동합니다.
        # [문제] 아니면 만약 정책이 '^' (위쪽)이면, 위쪽으로 한 칸 이동합니다.
        if action == '>':
            col += 1
        elif action == '<':
            col -= 1
        elif action == 'v':
            row += 1
        elif action == '^':
            row -= 1
        else:
            # 장애물 혹은 정의되지 않은 기호인 경우
            print(f"잘못된 정책 기호 '{action}'를 만났습니다. 중단합니다.")
            break
            
        path.append((row, col))  # 이동한 위치 기록

    print("경로를 찾지 못했거나 너무 오래 걸렸습니다.")
    return path
```
