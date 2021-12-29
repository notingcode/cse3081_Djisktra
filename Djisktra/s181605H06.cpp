#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <queue>

#define NONE -1

typedef struct elm_edge {
	int  vf, vr;  // e = (vf, vr) where vf, vr are vertex array indices
	int  cost;    // edge cost
	bool flag;    // true if the edge is in the SP spanning tree, false otherwise 
	int  fp, rp;  // adj list ptr of forward and reverse (-1 if none)
} edge;

typedef struct elm_vertex {
	int  f_hd, r_hd;	// adj list header(-1 if none)
	int  distance;		// distance from src to this vertex
	int  heap_idx;		// minHeap index
	bool inS;			// true if SP length found(true when move from V-S to S) 
} vertex;

void heap_insert(int vIndex, vertex* V, int* minHeap, int* heapSize);
void heapMoveUp(int heapIndex, vertex* V, int* minHeap);
int heapPopMin(vertex* V, int* minHeap, int* heapSize);
void initMinHeap(int src, int Vnum, vertex* V, int* minHeap, int* heapSize);
void initialize(int src, vertex* V, edge* E);

void heap_insert(
	int vIndex,		//	vertex index to be inserted
	vertex* V,		//	vertex array
	int* minHeap,	//	minHeap array
	int* heapSize	//	pointer to int storing size of heap
)
	// Output:
	// 1. minHeap with vIndex added: minimum heap structure is maintained after insertion
	// 2. heapSize = heapSize + 1
{
	//	새로운 vertex index를 heap에 추가하기 위해
	//	heap의 크기가 1 증가하고 V[idx] 가 heap array의 마지막 index에 추가된다.
	minHeap[++(*heapSize)] = vIndex;
	V[vIndex].heap_idx = *heapSize;

	//	새로운 vertex index가 heap에 insert되면 heap은 minHeap이
	//	아닐 수도 있다. 따라서 새로 입력된 vertex의 index를 이동하여
	//	minHeap으로 만든다.
	heapMoveUp(*heapSize, V, minHeap);
}

void heapMoveUp(
	int heapIndex,	//	index of heap to start move-up operation
	vertex* V,		//	vertex array
	int* minHeap	//	minHeap array
)
	// Input:
	// minimum heap with minHeap[heapIndex] out of place 
	//
	// Output:
	// 1. sorted minimum heap
{
	int temp;

	//	minHeap[heapIndex] 를 시작으로 parent vertex의 distance와 비교하여 heap에서 vertex의 위치를 교환한다.
	//	만약 heapIndex가 1에 도달하거나 parent vertex의 distance가 더 작은 경우 while loop이 terminate된다.
	while (heapIndex > 1 && (V[minHeap[heapIndex / 2]].distance > V[minHeap[heapIndex]].distance)) {
		//	parent의 distance가 child의 distance보다 크기 때문에 heap에서 두 vertex의 위치를 교환한다.
		temp = minHeap[heapIndex / 2];
		minHeap[heapIndex / 2] = minHeap[heapIndex];
		minHeap[heapIndex] = temp;

		//	V[heapIndex/2].heap_idx와 V[heapIndex].heap_idx 또한 교환한다.
		temp = V[minHeap[heapIndex / 2]].heap_idx;
		V[minHeap[heapIndex / 2]].heap_idx = V[minHeap[heapIndex]].heap_idx;
		V[minHeap[heapIndex]].heap_idx = temp;

		heapIndex /= 2; // parent가 새로운 child가 되어 다시 heap에서의 parent와 비교한다. 
	}
}

void initMinHeap(
	int src,		//	source vertex index
	int Vnum,		//	number vertices
	vertex* V,		//	vertex array
	int* minHeap,	//	minHeap array
	int* heapSize	//	pointer to int storing size of heap
)
	// Output:
	// initialized minimum heap
{
	// source vertex를 제외한 나머지 vertex를 heap에 하나씩 추가한다.
	for (int i = 0; i < Vnum; i++) {
		if (i != src) {
			heap_insert(i, V, minHeap, heapSize);
		}
	}
}

void initialize(
	int src,		//	source vertex index
	vertex* V,		//	vertex array
	edge* E			//	edge array
)
	// Output:
	// V[0...Vnum-1].distance is initialized: V[i].distance of all vertex i adjacent to source vertex is
	//										  set equal to edge cost of (source, i)
{
	int fp, rp, k;

	//	source에서 source로의 거리는 0이고 이미 shortest path이다.
	V[src].inS = true;
	V[src].distance = 0;

	fp = V[src].f_hd;
	rp = V[src].r_hd;

	//	source와 adjacent한 vertex와의 거리를 구하여 각 vertex의 distance에 assign한다.
	while (fp != NONE) { //	source vertex를 front vertex로 가진 edge들을 찾는다.
		k = E[fp].vr;
		V[k].distance = E[fp].cost;
		fp = E[fp].fp;
	}
	while (rp != NONE) { //	source vertex를 rear vertex로 가진 edge들을 찾는다.
		k = E[rp].vf;
		V[k].distance = E[rp].cost;
		rp = E[rp].rp;
	}

}

int heapPopMin(
	vertex* V,		//	vertex array
	int* minHeap,	//	minHeap array
	int* heapSize	//	pointer to int storing size of heap
)
	// Output:
	// 1. u = minHeap[1]: index of vertex that has the shortest distance to source vertex 
	// 2. heapSize = heapSize - 1
	// 3. new minHeap with u deleted
{
	int u, k, temp;
	k = 1;
	u = minHeap[k];						//	distance가 minimum인 vertex index를 u에 assign한다.
	minHeap[k] = minHeap[(*heapSize)--];//	minHeap의 마지막 element를 heap의 top에 assign하고 heap의 크기를 1 줄인다.
	V[minHeap[k]].heap_idx = k;			//	마지막 element였던 vertex의 heap index도 top에 대응하는 1로 assign한다.

	//	새로 top에 assign된 vertex index를 이동하여 minimum heap 형태를 유지한다.
	while ((k * 2) <= (*heapSize)) { // child를 가진 마지막 heap index에 도달하면 while loop을 terminate한다.
		
		if ((k * 2) == (*heapSize)) { // 왼쪽 child가 heap의 마지막 element인 경우
			if (V[minHeap[k]].distance > V[minHeap[k * 2]].distance) {
				// heap element 교환
				temp = minHeap[k * 2];
				minHeap[k * 2] = minHeap[k];
				minHeap[k] = temp;
				// vertex에 저장된 heap index도 교환
				temp = V[minHeap[k * 2]].heap_idx;
				V[minHeap[k * 2]].heap_idx = V[minHeap[k]].heap_idx;
				V[minHeap[k]].heap_idx = temp;

				k *= 2; // 왼쪽 child로 이동하여 heap 점검
			}
			else break; // minHeap이기 때문에 while loop을 terminate
		}

		else { // 왼쪽 오른쪽 child가 모두 있는 경우
			if (V[minHeap[k * 2]].distance < V[minHeap[(k * 2) + 1]].distance) { // 왼쪽 child가 오른쪽 child보다 작은 경우
				if (V[minHeap[k]].distance > V[minHeap[k * 2]].distance) { // 왼쪽 child의 distance가 현재 vertex의 distance보다 작은 경우
					// heap element 교환
					temp = minHeap[k * 2];
					minHeap[k * 2] = minHeap[k];
					minHeap[k] = temp;
					// vertex에 저장된 heap index도 교환
					temp = V[minHeap[k * 2]].heap_idx;
					V[minHeap[k * 2]].heap_idx = V[minHeap[k]].heap_idx;
					V[minHeap[k]].heap_idx = temp;

					k *= 2; // 왼쪽 child로 이동하여 heap 점검
				}
				else break; // minHeap이기 때문에 while loop을 terminate
			}
			else { // 오른쪽 child가 왼쪽 child보다 작거나 같은 경우
				if (V[minHeap[k]].distance > V[minHeap[(k * 2) + 1]].distance) { // 오른쪽 child의 distance가 현재 vertex의 distance보다 작은 경우
					// heap element 교환
					temp = minHeap[(k * 2) + 1];
					minHeap[(k * 2) + 1] = minHeap[k];
					minHeap[k] = temp;
					// vertex에 저장된 heap index도 교환
					temp = V[minHeap[(k * 2) + 1]].heap_idx;
					V[minHeap[(k * 2) + 1]].heap_idx = V[minHeap[k]].heap_idx;
					V[minHeap[k]].heap_idx = temp;

					k = (k * 2) + 1; // 오른쪽 child로 이동하여 heap 점검
				}
				else break; // minHeap이기 때문에 while loop을 terminate
			}
		}
	}

	return u;
}

int SPT_Dijkstra(
	int src,	// source vertex index
	// graph structure array
	// 1. the adjacency list structure is the same as PHW02
	// 2. additional fields are added for Dijkstra's algorithm(see .h file)
	int Vnum, vertex *V,	// Vertex array size and the array
	int Enum, edge *E,		// Edge array size and the array

	int *minHeap	// array for min heap (array size = Vnum+1)
		// heap index range is 1 ~ (Vnum - 1) note: src must not in the initial heap
		// just arry is passed. must construct min-heap in this function

	// OUTPUT
	// 1. V[].distance : shortest path length from src to this vertex
	// 2. shortest path spanning tree : set E[].flag = true if the edge is in the tree
	// 3. return the sum of edge costs in the shortest path spanning tree.
	//    should be small as possible (think a way to make it small)
) {
	int treeCost = 0;
	int heapSize, u, w, edgeIndex, edgeLim, spEdge = NONE;
	// *** 이 함수를 작성하자 ***
	// 반드시 min-heap을 사용하여 O((n+m)logn) 알고리즘을 구현해야 한다(아니면 trivial한 프로그램임)
	// heap 연산 등 필요한 함수는 자유롭게 작성하여 추가한다.
	// 그러나 global 변수, dynamic array 등은 추가로 사용하지 않는다(실제로 필요 없다)
	heapSize = 0;

	initialize(src, V, E); // source와 adjacent한 vertex까지의 distance 초기화
	initMinHeap(src, Vnum, V, minHeap, &heapSize); // minHeap 초기화

	for (int i = 0; i < Vnum - 1; i++) {
		u = heapPopMin(V, minHeap, &heapSize);	// source vertex로부터 가장 가까운 vertex의 index를 구한다
		V[u].heap_idx = NONE;
		if (V[u].distance != INT32_MAX) {
			V[u].inS = true;					// source에서 u로의 shortest path 발견
			edgeLim = INT32_MAX;
			edgeIndex = V[u].f_hd;				// u를 front vertex로 갖는 모든 incident edge 확인
			while (edgeIndex != NONE) {
				w = E[edgeIndex].vr;
				// u와 adjacent한 vertex 중에서 아직 shortest path가 발견되지 않았고 u를 통하는
				// 더 가까운 path가 있다면 w까지의 distance를 update한다.
				if (V[w].inS == false && V[u].distance + E[edgeIndex].cost < V[w].distance) {
					V[w].distance = V[u].distance + E[edgeIndex].cost;
					heapMoveUp(V[w].heap_idx, V, minHeap); // distance가 변경되었기 때문에 heap을 조정해야 한다.
				}
				// u에 도달하는 가장 가까운 path의 edge들은 spanning tree edge에 포함된다.
				// u와 adjacent한 vertex w 중에서 이미 shortest path가 발견되었고 이 중
				// (V[w].distance + E[edgeIndex].cost)를 가장 작게 만드는 edge (u, w)가
				// spanning tree에 포함된다.
				if (V[w].inS && (V[w].distance + E[edgeIndex].cost) < edgeLim) {
					edgeLim = V[w].distance + E[edgeIndex].cost;
					spEdge = edgeIndex;
				}
				edgeIndex = E[edgeIndex].fp;	// 다음 incident edge 확인
			}

			edgeIndex = V[u].r_hd;				// u를 rear vertex로 갖는 모든 incident edge 확인
			while (edgeIndex != NONE) {
				w = E[edgeIndex].vf;
				// u와 adjacent한 vertex 중에서 아직 shortest path가 발견되지 않았고 u를 통하는
				// 더 가까운 path가 있다면 w까지의 distance를 update한다.
				if (V[w].inS == false && V[u].distance + E[edgeIndex].cost < V[w].distance) {
					V[w].distance = V[u].distance + E[edgeIndex].cost;
					heapMoveUp(V[w].heap_idx, V, minHeap); // distance가 변경되었기 때문에 heap을 조정해야 한다.
				}
				// 위와 같은 방법으로 spanning tree의 edge를 찾는다.
				if (V[w].inS && (V[w].distance + E[edgeIndex].cost) < edgeLim) {
					edgeLim = V[w].distance + E[edgeIndex].cost;
					spEdge = edgeIndex;
				}
				edgeIndex = E[edgeIndex].rp;	// 다음 incident edge 확인
			}
			E[spEdge].flag = true;
		}
	}

	// spanning tree에 포함되는 edge들의 total cost를 구한다.
	for (int i = 0; i < Enum; i++) {
		if (E[i].flag)
			treeCost += E[i].cost;
	}

	return treeCost;
}

void Read_Graph(int Vnum, vertex *V, int Enum, edge *E) {
	// Graph 자료구조를 만드는 함수
	// *** 이 함수를 추가하자 ***
	// PHW02의 Read_Graph_adj_array()를 이 과제의 자료구조에 맞춰 살짝 수정하여 사용한다
	// 즉, Read_Graph_adj_array()에서 불필요한 내용을 제거한 후 사용.

	int k, vf, vr, ec;

	//	Initialize each vertex
	for (k = 0; k < Vnum; ++k) {
		V[k].f_hd = V[k].r_hd = NONE;
		V[k].distance = INT32_MAX;
		V[k].heap_idx = NONE;
		V[k].inS = false;
	}
	//	Attach incident edge index to each vertex and initialize each edge
	for (k = 0; k < Enum; k++) {
		scanf_s("%d %d %d", &vf, &vr, &ec);
		E[k].fp = E[k].rp = NONE;

		if (V[vf].f_hd == NONE) {
			V[vf].f_hd = k;
		}
		else {
			E[k].fp = V[vf].f_hd;
			V[vf].f_hd = k;
		}
		
		if (V[vr].r_hd == NONE) {
			V[vr].r_hd = k;
		}
		else {
			E[k].rp = V[vr].r_hd;
			V[vr].r_hd = k;
		}

		E[k].vf = vf;
		E[k].vr = vr;
		E[k].cost = ec;
		E[k].flag = false;
	}

}

// the following functions are for testing if the submitted program is correct.
int  Tree_Check(int Vnum, vertex *V, int Enum, edge *E, int *visited);
bool SPT_test(int src, int Vnum, vertex *V, int Enum, edge *E, int *minHeap);
void Error_Exit(const char *s);

int main ( void ) {
	int		src;
	vertex *V;		int Vnum;
	edge   *E;		int Enum;
	int    *minHeap;
	int    Tree_cost;
	int    Tnum; 		// # of test cases
	clock_t start, finish;
	double cmpt;

	scanf_s("%d", &Tnum);		// read # of tests

	for (int t = 0; t < Tnum; t++ ) {
		scanf_s("%d %d %d", &Vnum, &Enum, &src);
		V = new vertex [Vnum];
		E = new edge [Enum];
		minHeap = new int[Vnum + 1];	// heap array allocation
		if ( V == NULL || E == NULL || minHeap == NULL ) {
			Error_Exit("Memory Allocation Error");
		}
		Read_Graph(Vnum, V, Enum, E);

		/**/start = clock();	// start timer

		Tree_cost = SPT_Dijkstra(src, Vnum, V, Enum, E, minHeap);	// code by students

		/**/finish = clock();	// stop timer
		cmpt = ((double)(finish - start)) / (double)CLK_TCK;

		// 아래 Tree_Check와 SPT_test 함수는 첨부한 SPT_test.obj 파일에 있다.
		// 이 테스트에서 오류로 인하여 프로그램이 정지하면 뭔가 잘못된 것이다(제출해도 0점)
        if (Tree_Check(Vnum, V, Enum, E, minHeap) == 0) {
			Error_Exit("   ERROR The result is not a spanning tree");
		}
		if (SPT_test(src, Vnum, V, Enum, E, minHeap) == false) {
			Error_Exit("** Something wrong in applying Dijkstra's");
		}
		if ( t != 0 ) 
			printf("\n");

		printf("**T%d (Dijkstra) (V = %d, E = %d, time = %.3f sec) Tree Cost = %d\n", 
			t+1, Vnum, Enum, cmpt, Tree_cost);

		delete [] minHeap; delete [] V; delete [] E;
	}
	return 0;
}

void Error_Exit (const char *s ) {
  printf("%s\n", s);
  exit(-1);
}
