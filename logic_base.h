#ifndef LOGIC_BASE
#define LOGIC_BASE




struct coords {
  double x_max;
  double y_max;
  double x_min;
  double y_min;

  coords * next;
  coords * prev;
};

struct node {
  double x;
  double y;
  node * next;

};



void addNode(node * head, double x, double y);
void popNode(node ** head);
void addObstacle(coords * head, double x_min, double x_max, double y_min, double y_max);
void removeObstacle(coords ** head);
coords intersection(coords * head, node * start);
node * determinePath(double *x_start, double *y_start, double *phi, coords * head, double x_end, double y_end);





#endif
