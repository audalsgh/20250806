# 33일차 - 경로계획 알고리즘

## 1. 완전한 지식 스펙트럼을 갖는 세계
자율주행 경로 문제
- 기계가 세상을 완벽히 알고있을때(=경로를 알때), 어떻게 최적의 경로를 구하는가?
- 자율주행 내비게이션의 기본인, 경로 문제를 풀때는 지도를 그래프로 나타낸다.<br>
  <img width="873" height="479" alt="image" src="https://github.com/user-attachments/assets/3d651bd6-4e14-4e7e-a97b-20268d67c111" />
- 자기자신의 칸으로는 이동불가, 대각선으로 이동불가 가정.

지도, 경로를 아는 세계에서의 알고리즘들
- 깊이 우선 탐색(DFS) = "LIFO로, 노드 u에서 목표를 찾는 함수는, u의 이웃 v에서 목표를 찾는 함수를 호출하는 재귀적, 스택 알고리즘"
- 너비 우선 탐색(BFS) = "FIFO로, 먼저 들어온 노드부터 처리하는 반복문, 큐 알고리즘, 모든 경로의 비용이 같다면 최단 거리를 얻을수있다."
- 다익스트라(Dijkstra) 알고리즘 = "경로마다 비용이 다를때, 시작점부터 출발하여 비용이 가장 적은 노드부터 찾아냄."
- 에이스타(A*) 알고리즘 = "지금까지 온 경로의 비용 + 앞으로 갈 경로의 비용의 합이 제일 적은 경로를 찾아냄. 다익스트라에 방향성이 더해진 것."
-> 데이터 구조에따라 알고리즘의 논리가 결정됨.<br>
<img width="635" height="677" alt="image" src="https://github.com/user-attachments/assets/b3bd60af-a4fb-496b-8ada-cc3bd7f31f70" />

```python
`dfs_recursive` 함수를 완성하세요. 이 함수는 자기 자신을 호출하여 이웃 노드를 탐색해야 합니다.
- **Base Case (종료 조건):** 현재 노드가 목표이거나, 벽이거나, 이미 방문한 곳이면 탐색을 중단해야 합니다.
- **Recursive Step (재귀 단계):** 유효한 이웃 노드에 대해 `dfs_recursive` 함수를 다시 호출합니다.

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

## 3. 지식이 전혀 없는 세계
