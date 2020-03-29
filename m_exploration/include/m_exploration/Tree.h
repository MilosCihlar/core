#ifndef __TREE_H__
#define __TREE_H__

#include <iostream>

#include "m_exploration/Point.h"

class Tree
{
	/*Prvn� bod je reprezentovan 0, dal�� celymi �isly a v dvourozmernem poli bond je na pozici [x][0] vzdy poradi bodu*/
	int** bond;
	int seed;
	int numberPoint;


public:
	/*C'tors and D'tors*/
	Tree();
	Tree(const Tree& tree);
	~Tree();

	/*Methods*/
	void addPoint(const int node, const int neighbor);
	
	Tree removeSingleBranch(int end) const;
	bool haveNodeNeighbor(const int node) const;
	bool freeNeighbor() const;

	void print() const;
	void getTrajectory(int* traj, int end) const;

	/*Getter and Setter*/
	int getNumberPoint() const;
	int getNumberOfNode(int end) const;

	int getFirstNeighbor(const int node) const;
	int getNeighbor(const int node, const int pos) const;
	void removeNeighbor(const int node, const int neighbor);
	int getNumberNeighbor(const int node) const;

	void setSeed(const int s);
	int getSeed() const;
};
#endif // !__TREE_H__