#include "point.h"

typedef struct NodeList_tag {
	struct Node_tag *node;
	struct NodeList_tag *next;
} NodeList;
	
typedef struct Node_tag {
	char visited;
	struct NodeList_tag *nexts;
} Node;
	
typedef struct Graph_tag {
	struct Node_tag *root;
	Point coord;
} Graph;
	
Node *find(Point *p);
	
Node *dijkstra(Point *p);