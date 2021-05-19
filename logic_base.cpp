#include <Arduino.h>
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

coords * head;

void addNode(node * head, double x, double y) {
  node * current = head;
  while(current->next != NULL) {
    current = current->next;
  }
  current->next = (node *) malloc(sizeof(node));
  current->next->x = x;
  current->next->y = y;
  current->next->next = NULL;
}

void popNode(node ** head) {
  node * next_node = NULL;
  if (*head == NULL) return;

  next_node = (*head)->next;
  free(*head);
  *head = next_node;


}

void addObstacle(coords * head, double x_min, double x_max, double y_min, double y_max) {
  coords * current  = head;
  while(current->next != NULL) {
    current = current->next;
  }
  current->next = (coords *) malloc(sizeof(coords));
  current->next->x_min = x_min;
  current->next->x_max = x_max;
  current->next->y_min = y_min;
  current->next->y_max = y_max;

  current->next->next = NULL;
  current->next->prev = current;
}

void removeObstacle(coords ** head) {
  coords * next_coords = NULL;
  if (*head == NULL) {
    return;
  }
  next_coords = (*head)->next;
  free(*head);
  *head = next_coords;

  return;
}

int intersection(coords * head, node * start) { //Determine if the line and object intersect 
  //Step 1: Determine equation for m
  double x1 = start->x;
  double y1 = start->y;
  double x2  = start->next->x;
  double y2 = start->next->y;

  double m = (y1-y2)/(x1-x2);
  double b = y1-m*x1;
  
  
  //Step 2: loop through coords and determine intersection
  coords * current  = head;
  while(current != NULL) {
    //Step 2.1 setup new doubles/ints
    double y_min = min(current->y_min, current->y_max);
    double y_max = max(current->y_min, current->y_max);
    double x_min = min(current->x_min, current->x_max);
    double x_max = max(current->x_min, current->x_max);

    //Step 2.2 check intersection on y axis
    //double y = 
  
    //Step 2.3 check intersection on x axis

    current = current->next;
  }
  
}

int determinePath(double *x_start, double *y_start, coords * head) {
  node * top;
  top  = (node *) malloc(sizeof(node));
  top->x = *x_start;
  top->y = *y_start;


  return 0;
}
