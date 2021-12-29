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
	//	���ο� vertex index�� heap�� �߰��ϱ� ����
	//	heap�� ũ�Ⱑ 1 �����ϰ� V[idx] �� heap array�� ������ index�� �߰��ȴ�.
	minHeap[++(*heapSize)] = vIndex;
	V[vIndex].heap_idx = *heapSize;

	//	���ο� vertex index�� heap�� insert�Ǹ� heap�� minHeap��
	//	�ƴ� ���� �ִ�. ���� ���� �Էµ� vertex�� index�� �̵��Ͽ�
	//	minHeap���� �����.
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

	//	minHeap[heapIndex] �� �������� parent vertex�� distance�� ���Ͽ� heap���� vertex�� ��ġ�� ��ȯ�Ѵ�.
	//	���� heapIndex�� 1�� �����ϰų� parent vertex�� distance�� �� ���� ��� while loop�� terminate�ȴ�.
	while (heapIndex > 1 && (V[minHeap[heapIndex / 2]].distance > V[minHeap[heapIndex]].distance)) {
		//	parent�� distance�� child�� distance���� ũ�� ������ heap���� �� vertex�� ��ġ�� ��ȯ�Ѵ�.
		temp = minHeap[heapIndex / 2];
		minHeap[heapIndex / 2] = minHeap[heapIndex];
		minHeap[heapIndex] = temp;

		//	V[heapIndex/2].heap_idx�� V[heapIndex].heap_idx ���� ��ȯ�Ѵ�.
		temp = V[minHeap[heapIndex / 2]].heap_idx;
		V[minHeap[heapIndex / 2]].heap_idx = V[minHeap[heapIndex]].heap_idx;
		V[minHeap[heapIndex]].heap_idx = temp;

		heapIndex /= 2; // parent�� ���ο� child�� �Ǿ� �ٽ� heap������ parent�� ���Ѵ�. 
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
	// source vertex�� ������ ������ vertex�� heap�� �ϳ��� �߰��Ѵ�.
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

	//	source���� source���� �Ÿ��� 0�̰� �̹� shortest path�̴�.
	V[src].inS = true;
	V[src].distance = 0;

	fp = V[src].f_hd;
	rp = V[src].r_hd;

	//	source�� adjacent�� vertex���� �Ÿ��� ���Ͽ� �� vertex�� distance�� assign�Ѵ�.
	while (fp != NONE) { //	source vertex�� front vertex�� ���� edge���� ã�´�.
		k = E[fp].vr;
		V[k].distance = E[fp].cost;
		fp = E[fp].fp;
	}
	while (rp != NONE) { //	source vertex�� rear vertex�� ���� edge���� ã�´�.
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
	u = minHeap[k];						//	distance�� minimum�� vertex index�� u�� assign�Ѵ�.
	minHeap[k] = minHeap[(*heapSize)--];//	minHeap�� ������ element�� heap�� top�� assign�ϰ� heap�� ũ�⸦ 1 ���δ�.
	V[minHeap[k]].heap_idx = k;			//	������ element���� vertex�� heap index�� top�� �����ϴ� 1�� assign�Ѵ�.

	//	���� top�� assign�� vertex index�� �̵��Ͽ� minimum heap ���¸� �����Ѵ�.
	while ((k * 2) <= (*heapSize)) { // child�� ���� ������ heap index�� �����ϸ� while loop�� terminate�Ѵ�.
		
		if ((k * 2) == (*heapSize)) { // ���� child�� heap�� ������ element�� ���
			if (V[minHeap[k]].distance > V[minHeap[k * 2]].distance) {
				// heap element ��ȯ
				temp = minHeap[k * 2];
				minHeap[k * 2] = minHeap[k];
				minHeap[k] = temp;
				// vertex�� ����� heap index�� ��ȯ
				temp = V[minHeap[k * 2]].heap_idx;
				V[minHeap[k * 2]].heap_idx = V[minHeap[k]].heap_idx;
				V[minHeap[k]].heap_idx = temp;

				k *= 2; // ���� child�� �̵��Ͽ� heap ����
			}
			else break; // minHeap�̱� ������ while loop�� terminate
		}

		else { // ���� ������ child�� ��� �ִ� ���
			if (V[minHeap[k * 2]].distance < V[minHeap[(k * 2) + 1]].distance) { // ���� child�� ������ child���� ���� ���
				if (V[minHeap[k]].distance > V[minHeap[k * 2]].distance) { // ���� child�� distance�� ���� vertex�� distance���� ���� ���
					// heap element ��ȯ
					temp = minHeap[k * 2];
					minHeap[k * 2] = minHeap[k];
					minHeap[k] = temp;
					// vertex�� ����� heap index�� ��ȯ
					temp = V[minHeap[k * 2]].heap_idx;
					V[minHeap[k * 2]].heap_idx = V[minHeap[k]].heap_idx;
					V[minHeap[k]].heap_idx = temp;

					k *= 2; // ���� child�� �̵��Ͽ� heap ����
				}
				else break; // minHeap�̱� ������ while loop�� terminate
			}
			else { // ������ child�� ���� child���� �۰ų� ���� ���
				if (V[minHeap[k]].distance > V[minHeap[(k * 2) + 1]].distance) { // ������ child�� distance�� ���� vertex�� distance���� ���� ���
					// heap element ��ȯ
					temp = minHeap[(k * 2) + 1];
					minHeap[(k * 2) + 1] = minHeap[k];
					minHeap[k] = temp;
					// vertex�� ����� heap index�� ��ȯ
					temp = V[minHeap[(k * 2) + 1]].heap_idx;
					V[minHeap[(k * 2) + 1]].heap_idx = V[minHeap[k]].heap_idx;
					V[minHeap[k]].heap_idx = temp;

					k = (k * 2) + 1; // ������ child�� �̵��Ͽ� heap ����
				}
				else break; // minHeap�̱� ������ while loop�� terminate
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
	// *** �� �Լ��� �ۼ����� ***
	// �ݵ�� min-heap�� ����Ͽ� O((n+m)logn) �˰����� �����ؾ� �Ѵ�(�ƴϸ� trivial�� ���α׷���)
	// heap ���� �� �ʿ��� �Լ��� �����Ӱ� �ۼ��Ͽ� �߰��Ѵ�.
	// �׷��� global ����, dynamic array ���� �߰��� ������� �ʴ´�(������ �ʿ� ����)
	heapSize = 0;

	initialize(src, V, E); // source�� adjacent�� vertex������ distance �ʱ�ȭ
	initMinHeap(src, Vnum, V, minHeap, &heapSize); // minHeap �ʱ�ȭ

	for (int i = 0; i < Vnum - 1; i++) {
		u = heapPopMin(V, minHeap, &heapSize);	// source vertex�κ��� ���� ����� vertex�� index�� ���Ѵ�
		V[u].heap_idx = NONE;
		if (V[u].distance != INT32_MAX) {
			V[u].inS = true;					// source���� u���� shortest path �߰�
			edgeLim = INT32_MAX;
			edgeIndex = V[u].f_hd;				// u�� front vertex�� ���� ��� incident edge Ȯ��
			while (edgeIndex != NONE) {
				w = E[edgeIndex].vr;
				// u�� adjacent�� vertex �߿��� ���� shortest path�� �߰ߵ��� �ʾҰ� u�� ���ϴ�
				// �� ����� path�� �ִٸ� w������ distance�� update�Ѵ�.
				if (V[w].inS == false && V[u].distance + E[edgeIndex].cost < V[w].distance) {
					V[w].distance = V[u].distance + E[edgeIndex].cost;
					heapMoveUp(V[w].heap_idx, V, minHeap); // distance�� ����Ǿ��� ������ heap�� �����ؾ� �Ѵ�.
				}
				// u�� �����ϴ� ���� ����� path�� edge���� spanning tree edge�� ���Եȴ�.
				// u�� adjacent�� vertex w �߿��� �̹� shortest path�� �߰ߵǾ��� �� ��
				// (V[w].distance + E[edgeIndex].cost)�� ���� �۰� ����� edge (u, w)��
				// spanning tree�� ���Եȴ�.
				if (V[w].inS && (V[w].distance + E[edgeIndex].cost) < edgeLim) {
					edgeLim = V[w].distance + E[edgeIndex].cost;
					spEdge = edgeIndex;
				}
				edgeIndex = E[edgeIndex].fp;	// ���� incident edge Ȯ��
			}

			edgeIndex = V[u].r_hd;				// u�� rear vertex�� ���� ��� incident edge Ȯ��
			while (edgeIndex != NONE) {
				w = E[edgeIndex].vf;
				// u�� adjacent�� vertex �߿��� ���� shortest path�� �߰ߵ��� �ʾҰ� u�� ���ϴ�
				// �� ����� path�� �ִٸ� w������ distance�� update�Ѵ�.
				if (V[w].inS == false && V[u].distance + E[edgeIndex].cost < V[w].distance) {
					V[w].distance = V[u].distance + E[edgeIndex].cost;
					heapMoveUp(V[w].heap_idx, V, minHeap); // distance�� ����Ǿ��� ������ heap�� �����ؾ� �Ѵ�.
				}
				// ���� ���� ������� spanning tree�� edge�� ã�´�.
				if (V[w].inS && (V[w].distance + E[edgeIndex].cost) < edgeLim) {
					edgeLim = V[w].distance + E[edgeIndex].cost;
					spEdge = edgeIndex;
				}
				edgeIndex = E[edgeIndex].rp;	// ���� incident edge Ȯ��
			}
			E[spEdge].flag = true;
		}
	}

	// spanning tree�� ���ԵǴ� edge���� total cost�� ���Ѵ�.
	for (int i = 0; i < Enum; i++) {
		if (E[i].flag)
			treeCost += E[i].cost;
	}

	return treeCost;
}

void Read_Graph(int Vnum, vertex *V, int Enum, edge *E) {
	// Graph �ڷᱸ���� ����� �Լ�
	// *** �� �Լ��� �߰����� ***
	// PHW02�� Read_Graph_adj_array()�� �� ������ �ڷᱸ���� ���� ��¦ �����Ͽ� ����Ѵ�
	// ��, Read_Graph_adj_array()���� ���ʿ��� ������ ������ �� ���.

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

		// �Ʒ� Tree_Check�� SPT_test �Լ��� ÷���� SPT_test.obj ���Ͽ� �ִ�.
		// �� �׽�Ʈ���� ������ ���Ͽ� ���α׷��� �����ϸ� ���� �߸��� ���̴�(�����ص� 0��)
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
