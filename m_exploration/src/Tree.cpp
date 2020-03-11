#include "m_exploration/Tree.h"

Tree::Tree():bond(nullptr), numberPoint(0), seed(0) // zmena
{
	bond = new int*[1];
	bond[0] = new int[1];
	bond[0][0] = 0;
}


Tree::Tree(const Tree& tree) // zmena
{
	numberPoint = tree.numberPoint;
	seed = tree.seed;
	bond = new int* [tree.numberPoint+1];
	for (int y = 0; y <= tree.numberPoint; ++y)
	{
		int max = tree.getNumberNeighbor(y);
		bond[y] = new int[max + 1];
		for (int x = 0; x <= max; ++x)
		{
			bond[y][x] = tree.bond[y][x];
		}
	}
}

Tree::~Tree()
{
	if (bond != nullptr)
	{
		for (int i = 0; i <= numberPoint; ++i)
		{
			delete[] bond[i];
		}

		delete[] bond;
	}
}

void Tree::addPoint(const int node, const int neighbor)
{
	int** copy = new int* [numberPoint + 1];
	int l_x = 0;
	for (int y = 0; y <= numberPoint; ++y)
	{
		copy[y] = new int[bond[y][0]+1];
		for (int x = 0; x <= bond[y][0]; ++x)
		{
			copy[y][x] = bond[y][x];
		}
		delete[] bond[y];
	}
	delete[] bond;

	numberPoint += 1;
	bond = new int* [numberPoint + 1];
	for (int y = 0; y < numberPoint; ++y)
	{
		if (y == neighbor)
		{
			l_x = copy[y][0] +2;// sizeof(copy[y]) / sizeof(int) + 1;
			bond[y] = new int[l_x];
			bond[y][l_x - 1] = node;
			bond[y][0] = l_x - 1;
			for (int x = 1; x < (l_x-1); ++x)
			{
				bond[y][x] = copy[y][x];
			}
		}
		else
		{
			l_x = copy[y][0] + 1;
			bond[y] = new int[l_x];
			for (int x = 0; x < l_x; ++x)
			{
				bond[y][x] = copy[y][x];
			}
		}
		delete[] copy[y];
	}
	delete[] copy;
	bond[numberPoint] = new int[2];
	bond[numberPoint][0] = 1;
	bond[numberPoint][1] = neighbor;
}

Tree Tree::removeSingleBranch() const   // zmena
{
	//remove all single brach, replace nodes in branches -1
	Tree copy = *this;
	bool step = true;
	while (step)
	{
		step = false;
		for (int y = 0; y < numberPoint; ++y)
		{
			if (y != seed)
			{
				if (copy.bond[y][0] == 1)
				{
					step = true;
					copy.bond[y][0] = -1;
					for (int yy = 0; yy < numberPoint; yy++)
					{
						int max = copy.bond[yy][0];
						for (int xx = 1; xx <= max; xx++)
						{
							if (copy.bond[yy][xx] == -1)
								max += 1;

							if (copy.bond[yy][xx] == y)
							{
								copy.bond[yy][xx] = -1;
								copy.bond[yy][0] = copy.bond[yy][0] - 1;
							}
						}
					}
				}
			}
		}

	}

	return copy;
}

void Tree::getTrajectory(int* traj) const
{
	int l = getNumberOfNode();
	for (int i = 0; i < l; i++)
	{
		traj[i] = -1;
	}

	Tree tree = removeSingleBranch();

	int node = seed;
	int nextNode = 0;
	int index = 0;
	while (node != numberPoint)
	{
		traj[index] = node;
		int max = tree.bond[node][0];
		for (int i = 1; i <= max; i++)
		{
			nextNode = tree.bond[node][i];
			if (nextNode == -1)
			{
				max += 1;
				continue;
			}

			bool nextLoop = false;
			for (int i = 0; i < l; i++)
			{
				if (traj[i] == nextNode)
					nextLoop = true;
			}

			if (!nextLoop)
			{
				node = nextNode;
				break;
			}
		}
		index += 1;
	}

	traj[index] = node;

}

bool Tree::haveNodeNeighbor(const int node) const
{
	if (numberPoint >= node)
	{
		int max = bond[node][0];
		for (int i = 1; i <= max; i++)
		{
			if (bond[node][i] != -1)
			{
				return true;
			}
			else
				max += 1;

		}
	}
	return false;
}

bool Tree::freeNeighbor() const
{
	for (int node = 0; node <= numberPoint; node++)
	{
		int max = bond[node][0];
		for (int n = 1; n <= max; n++)
		{
			if (bond[node][n] != -1)
			{
				return true;
			}
			else
				max += 1;
		}
	}
	return false;
}


int Tree::getFirstNeighbor(const int node) const
{
	int max = bond[node][0];
	for (int neigh = 1; neigh <= max; neigh++)
	{
		if (bond[node][neigh] != -1)
		{
			return bond[node][neigh];
		}
		else
			max += 1;
	}
	return -1;
}

int Tree::getNeighbor(const int node, const int pos) const
{
	if (numberPoint >= node)
	{
		int max = bond[node][0];
		int count = 0;
		for (int i = 1; i < max; i++)
		{
			if (bond[node][i] == -1)
				max += 1;
			else
				count += 1;
		}

		if (count >= pos)
		{
			return bond[node][pos];
		}
	}
	return -1;
}

void Tree::removeNeighbor(const int node, const int neighbor)
{
	if (numberPoint >= node)
	{
		int max = bond[node][0];
		for (int i = 1; i <= max; i++)
		{
			if (bond[node][i] == neighbor)
			{
				bond[node][i] = -1;
				bond[node][0] = bond[node][0] - 1;
			}
			else if (bond[node][i] == -1)
				max += 1;
		}
	}
}

int Tree::getNumberNeighbor(const int node) const // zmena
{
	if (numberPoint >= node)
	{
		int max = bond[node][0];
		int count = 0;
		for (int i = 1; i <= max; i++)
		{
			if (bond[node][i] == -1)
				max += 1;
			else
				count += 1;
		}

		return count;
	}

	return 0;
}

void Tree::print() const 
{
	for (int y = 0; y <= numberPoint; ++y)
	{
		std::cout << y << ": ";
		int max = bond[y][0];
		for (int x = 0; x <= max; ++x)
		{
			if (bond[y][x] == -1)
			{
				max += 1;
			}
			std::cout << bond[y][x] << " ";
		}
		std::cout << std::endl;
		
	}
}

int Tree::getNumberPoint() const
{
	return numberPoint;
}

int Tree::getNumberOfNode() const
{
	Tree copy = *this;
	bool step = true;
	while (step)
	{
		step = false;
		for (int y = 0; y < numberPoint; ++y)
		{
			if (y != seed)
			{
				if (copy.bond[y][0] == 1)
				{
					step = true;
					copy.bond[y][0] = -1;
					for (int yy = 0; yy < numberPoint; yy++)
					{
						int max = copy.bond[yy][0];
						for (int xx = 1; xx <= max; xx++)
						{
							if (copy.bond[yy][xx] == -1)
								max += 1;

							if (copy.bond[yy][xx] == y)
							{
								copy.bond[yy][xx] = -1;
								copy.bond[yy][0] = copy.bond[yy][0] - 1;
							}
						}
					}
				}
			}
		}
	}

	int index = 0;
	for (int y = 0; y <= numberPoint; y++)
	{
		if (copy.bond[y][0] != -1)
		{
			index += 1;
		}
	}

	return index;
}

void Tree::setSeed(const int s)
{
	seed = s;
}

int Tree::getSeed() const
{
	return seed;
}





