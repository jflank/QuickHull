#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>
#include <algorithm>
#include <assert.h>
#include <sstream>
#include <limits>
#include "Point2D.h"
          
using namespace std;

/*
 * Taken from https://en.wikipedia.org/wiki/Quickhull
 */ 


#define debug true

void PrintArray(vector<Point2D> &A)
{
  std::vector<Point2D>::iterator it;
  for (it=A.begin(); it != A.end(); ++it ) {
    cout << *it << " ";
  }
  cout << endl;
}

bool isLeft(Point2D a, Point2D b, Point2D c){
     return ((b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x)) > 0;
}
// formula for line ax+by+c = 0
void getLine(Point2D p1, Point2D p2, float &a, float &b, float &c)
{
    // (x- p1X) / (p2X - p1X) = (y - p1Y) / (p2Y - p1Y) 
    a = p2.y - p1.y;
    b = p2.x - p1.x;
    c = p1.x * p2.y - p2.x * p1.y;
}
/*
float distLinePoint(Point2D &p0, Point2D &p1, Point2D &p2)
{
    float a, b, c;
    getLine(p1, p2, a, b, c);
    float A
    return abs(a * p0.x + b * p0.y + c) / sqrt(a * a + b * b);
}
*/
/*
Note that each 3 points [a,b,p] for each p in the array form a trianle, 
whose area is denoted by: (ab) * h /2 [where h is the distance from p to ab].
Largest Triangle == greatest distance.
*/
float AreaOfTriangle(Point2D &p0, Point2D &p1, Point2D &p2)
{
  float area;
  //area=(1.0/2)* fabs((p1.x-p0.x)*(p2.y-p1.y) - (p1.x-p2.x)*(p0.y-p1.y));
  area = abs((p0.x*(p1.y-p2.y)+p1.x*(p2.y-p0.y)+p2.x*(p0.y-p1.y))/2.0);
  return area;
}

Point2D FurthestPoint(vector<Point2D> &S, Point2D &p1, Point2D &p2)
{
  Point2D ret = S[0];
  int remove = 0;
  //  float curdist = distLinePoint(ret, p1, p2);
  float curdist = AreaOfTriangle(ret, p1, p2);
  for (int i = 1; i < S.size(); i ++) {
    //    float tmp = distLinePoint(S[i], p1, p2);
    float tmp = AreaOfTriangle(S[i], p1, p2);
    if (tmp > curdist) {
      ret = S[i];
      curdist = tmp;
      remove = i;
    }
  }
  S.erase(S.begin()+ remove);

  return ret;
}  

Point2D GetMinX(vector<Point2D> &A)
{
  Point2D ret = A[0];
  int remove = 0;
  for (int i = 1; i < A.size(); i ++) {
    if (ret.x > A[i].x) {
      ret = A[i];
      remove = i;
    }
  }
  A.erase(A.begin()+ remove);
  return ret;
}

Point2D GetMaxX(vector<Point2D> &A)
{
  Point2D ret = Point2D(A[0]);
  int remove = 0;
  for (int i = 1; i < A.size(); i ++) {
    if (ret.x < A[i].x) {
      ret = A[i];
      remove = i;
    }
  }
  A.erase(A.begin()+remove);

  return ret;
}

void FindHull (vector<Point2D> Sk, vector<Point2D> &H, Point2D &P, Point2D &Q);

//2,2, 4,6, 6,2, 4,0 
/*
 *    Input = a set S of n points 
 * Assume that there are at least 2 points in the input set S of points
*/
void QuickHull(vector<Point2D> &S,vector<Point2D> &H)
{

  //Find left and right most points, say A & B, and add A & B to convex hull 

  Point2D A = GetMinX(S);
  Point2D B = GetMaxX(S);

  H.push_back(A);
  H.push_back(B);

  /*
   *  Segment AB divides the remaining (n-2) points into 2 groups S1 and S2 
   *  where S1 are points in S that are on the right side of the oriented line from A to B, 
   *  and S2 are points in S that are on the right side of the oriented line from B to A 
  */
  vector<Point2D> S1;
  vector<Point2D> S2;

  for (int i =0; i < S.size(); i++) {
    if (A.y < B.y) {
      if (isLeft(A, B, S[i])) {
	S2.push_back(S[i]);
      } else {
	S1.push_back(S[i]);
      }
    } else {
      if (isLeft(A, B, S[i])) {
	S1.push_back(S[i]);
      } else {
	S2.push_back(S[i]);
      }
    }      
    
  }
  FindHull (S1, H, A, B);
  FindHull (S2, H, B, A);
}

/* 
 * Find points on convex hull from the set Sk of points 
 * that are on the right side of the oriented line from P to Q
 */
void FindHull (vector<Point2D> Sk, vector<Point2D> &H, Point2D &P, Point2D &Q)
{
  /*
  cout << "Starting FindHull\n";
  cout << "H:  ";
  PrintArray(H);
  cout << "Sk: ";
  PrintArray(Sk);
  cout << P << Q << endl;
  */
  std::vector<Point2D>::iterator it;
  if (Sk.size() == 0) {
    return;
  }
  // From the given set of points in Sk, find farthest point, say C, from segment PQ 
  Point2D C = FurthestPoint(Sk, P, Q);
  //  Add point C to convex hull at the location between P and Q
  for (it = H.begin(); it != H.end(); ++it) {
    if (*it == Q) {
      H.insert(it, C);
      break;
    }
  }
  /*
   * Three points P, Q, and C partition the remaining points of Sk into 3 subsets: S0, S1, and S2 
   * where S0 are points inside triangle PCQ, S1 are points on the right side of the oriented 
   * line from  P to C, and S2 are points on the right side of the oriented line from C to Q. 
  */

  std::vector<Point2D> S0; //inside
  std::vector<Point2D> S1; //right
  std::vector<Point2D> S2; //left

  for (int i =0; i < Sk.size(); i++) {
    if (P.x < C.x) {
      if (isLeft(C, P, Sk[i])) {
	S2.push_back(Sk[i]);
      } else if (isLeft(C, Q, Sk[i])) {
	S0.push_back(Sk[i]);
      } else {
	S1.push_back(Sk[i]);
      }
    } else {
      if (isLeft(P, C, Sk[i])) {
	S1.push_back(Sk[i]);
      } else if (isLeft(Q, C, Sk[i])) {
	S0.push_back(Sk[i]);
      } else {
	S2.push_back(Sk[i]);
      }
    }
  }
  FindHull(S1, H, P, C);
  FindHull(S2, H, C, Q);
}

//general
int test1()
{
  vector<Point2D> H;
  vector<Point2D> A
  {{3, 2}, {6, 3}, {2, 5}, {5, 2}, {1, 1}, {4, 4}}; 
  cout << "A: ";  PrintArray(A);
  QuickHull(A, H);
  cout << "H: ";  PrintArray(H);
}
//xmin = ymin
int test2()
{
  vector<Point2D> H;
  
  vector<Point2D> A
  {{3, 2}, {6, 15}, {8, 1}, {9, 2}, {1, 1}, {4, 4}, {12, 14}, {7, 8}, {5, 3}, {2, 10}}; 
  cout << "A: ";
  PrintArray(A);
  QuickHull(A, H);
  cout << "H: ";
  PrintArray(H);
}
//xmin == ymax
int test3()
{
  vector<Point2D> H;
  vector<Point2D> A
  {{3, 2}, {6, 15}, {8, 1}, {9, 2}, {1, 15}, {4, 4}, {12, 14}, {7, 8}, {5, 3}, {2, 10}}; 
  cout << "A: ";
  PrintArray(A);
  QuickHull(A, H);
  cout << "H: ";
  PrintArray(H);
}
//xmin == ymax, xmax == ymin
int test4()
{
  vector<Point2D> H;
  vector<Point2D> A
  {{13,1}, {3, 2}, {6, 15}, {8, 1}, {9, 2}, {1, 15}, {4, 4}, {12, 14}, {7, 8}, {5, 3}, {2, 10}}; 
  cout << "A: ";
  PrintArray(A);
  QuickHull(A, H);
  cout << "H: ";
  PrintArray(H);
}

int main(int argc, char * argv[])
{
  test1();
  test2();
  test3();
  test4();

  
  return 0;
}
