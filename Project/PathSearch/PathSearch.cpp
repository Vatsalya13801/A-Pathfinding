#include "PathSearch.h"


namespace AStarPathfinding { namespace algorithms {

	PathSearch::PathSearch()
	{

	}

	PathSearch::~PathSearch()
	{
	}

	void PathSearch::initialize(TileMap* _tileMap)
	{
		tileMap = _tileMap;
		
		for (int y = 0; y < tileMap->getColumnCount(); y++)
		{
			for (int x = 0; x < tileMap->getRowCount(); x++)
			{
					SearchNode* temp = new SearchNode();
					temp->tile = tileMap->getTile(x, y);
					nodes.insert(std::pair<Tile*, SearchNode*>(tileMap->getTile(x, y), temp));
				
			}
		}
		
		for (int x = 0; x < tileMap->getRowCount(); x++)
		{
			for (int y = 0; y < tileMap->getColumnCount(); y++)
			{
				if (tileMap->getTile(x, y)->getWeight() > 0)
				{
					if (x % 2 == 0)
					{
						if (tileMap->getTile(x - 1, y - 1))
							if (tileMap->getTile(x - 1, y - 1)->getWeight() != 0)
								nodes[tileMap->getTile(x, y)]->neighbors.push_back(nodes[tileMap->getTile(x - 1, y - 1)]);
						if (tileMap->getTile(x - 1, y))
							if (tileMap->getTile(x - 1, y)->getWeight() != 0)
								nodes[tileMap->getTile(x, y)]->neighbors.push_back(nodes[tileMap->getTile(x - 1, y)]);
						if (tileMap->getTile(x, y - 1))
							if (tileMap->getTile(x, y - 1)->getWeight() != 0)
								nodes[tileMap->getTile(x, y)]->neighbors.push_back(nodes[tileMap->getTile(x, y - 1)]);
						if (tileMap->getTile(x - 1, y + 1))
							if (tileMap->getTile(x - 1, y + 1)->getWeight() != 0)
								nodes[tileMap->getTile(x, y)]->neighbors.push_back(nodes[tileMap->getTile(x - 1, y + 1)]);
						if (tileMap->getTile(x + 1, y ))
							if (tileMap->getTile(x + 1, y )->getWeight() != 0)
								nodes[tileMap->getTile(x, y)]->neighbors.push_back(nodes[tileMap->getTile(x + 1, y )]);
						if (tileMap->getTile(x, y + 1))
							if (tileMap->getTile(x, y + 1)->getWeight() != 0)
								nodes[tileMap->getTile(x, y)]->neighbors.push_back(nodes[tileMap->getTile(x , y + 1)]);
					}
					else
					{

						if (tileMap->getTile(x - 1, y ))
							if (tileMap->getTile(x - 1, y)->getWeight() != 0)
								nodes[tileMap->getTile(x, y)]->neighbors.push_back(nodes[tileMap->getTile(x - 1, y )]);
						if (tileMap->getTile(x + 1, y))
							if (tileMap->getTile(x + 1, y)->getWeight() != 0)
								nodes[tileMap->getTile(x, y)]->neighbors.push_back(nodes[tileMap->getTile(x + 1, y)]);
						if (tileMap->getTile(x, y - 1))
							if (tileMap->getTile(x, y - 1)->getWeight() != 0)
								nodes[tileMap->getTile(x, y)]->neighbors.push_back(nodes[tileMap->getTile(x, y - 1)]);
						if (tileMap->getTile(x , y + 1))
							if (tileMap->getTile(x , y + 1)->getWeight() != 0)
								nodes[tileMap->getTile(x, y)]->neighbors.push_back(nodes[tileMap->getTile(x , y + 1)]);
						if (tileMap->getTile(x - 1, y + 1))
							if (tileMap->getTile(x - 1, y + 1)->getWeight() != 0)
								nodes[tileMap->getTile(x, y)]->neighbors.push_back(nodes[tileMap->getTile(x - 1, y +1 )]);
						if (tileMap->getTile(x + 1, y + 1))
							if (tileMap->getTile(x + 1, y + 1)->getWeight() != 0)
								nodes[tileMap->getTile(x, y)]->neighbors.push_back(nodes[tileMap->getTile(x + 1, y + 1)]);
					}
				}
			}
		}
	}

	void PathSearch::enter(int startRow, int startColumn, int goalRow, int goalColumn)
	{
		end = tileMap->getTile(goalRow, goalColumn);
		PlannerNode* start = new PlannerNode(nodes.at(tileMap->getTile(startRow, startColumn)));
		open.push(start);
		visited.insert(std::pair<SearchNode*, PlannerNode*>(start->searchNode, start));
		open.front()->gCost = 0;
		open.front()->hCost = Distance(open.front()->searchNode->tile, end);
		float hWeight = open.front()->searchNode->tile->getWeight();
		open.front()->fCost = open.front()->gCost + (open.front()->hCost * hWeight);	
		TD = Distance(open.front()->searchNode->tile, open.front()->searchNode->neighbors[0]->tile);			
	}

	void PathSearch::update(long timeslice)
	{
		while (!open.empty()) {
			
			PlannerNode* curr = open.front();
			open.pop();

			if (curr->searchNode->tile == end)
			{
				goal = curr;
				done = true;
				return;
			}
			for (auto successor :curr->searchNode->neighbors)
			{
				float tempGivenCost = curr->gCost + TD * successor->tile->getWeight();
				if (visited[successor] != nullptr)
				{
					PlannerNode* node = visited[successor];
					if (tempGivenCost < node->gCost)
					{
						open.remove(node);
						node->gCost = tempGivenCost;
						node->fCost = node->gCost + (node->hCost * hW);
						node->parent = curr;
						open.push(node);
					}
				}
				else
				{
					PlannerNode* node = new PlannerNode(successor);
					node->gCost = tempGivenCost;
					node->hCost = Distance(node->searchNode->tile, end);
					node->fCost = node->gCost + (node->hCost * hW);
					node->parent = curr;
					visited[successor] = node;
					open.push(node);
					node->searchNode->tile->setFill(0xFF0000FF);
					
				}
			}
			
		}
		if (timeslice == 0)
		{
			return;
		}
	}

	void PathSearch::exit()
	{
		for (auto it = visited.begin(); it != visited.end(); it++)
		{
			delete it->second;
		}
		visited.clear();
		open.clear();
		//delete goal;
		end = nullptr;
	}

	void PathSearch::shutdown()
	{
		for (auto it = visited.begin(); it != visited.end(); it++)
		{
			delete it->second;
		}
		visited.clear();
		open.clear();
		//delete goal;
		end = nullptr;
		nodes.clear();
	}

	

	bool PathSearch::isDone() const
	{
		if (visited.size() == 0)
		{
			return true;
		}
		return done;
	}

	std::vector<Tile const*> const PathSearch::getSolution() const
	{
		std::vector<Tile const*> temp;
		for (PlannerNode* i = goal; i != nullptr; i = i->parent)
		{
			temp.push_back(i->searchNode->tile);
			if (i->parent != nullptr)
			{
				i->searchNode->tile->addLineTo(i->parent->searchNode->tile, 0xFFFF0000);
			}
		}
		return temp;
	}

	float PathSearch::Distance(Tile* a, Tile* b)
	{		
		float ans = ((a->getXCoordinate() - b->getXCoordinate()) * (a->getXCoordinate() - b->getXCoordinate())) + ((a->getYCoordinate() - b->getYCoordinate()) * (a->getYCoordinate() - b->getYCoordinate()));
			ans = sqrt(ans);
			return ans;
	}
}}  // namespace fullsail_ai::algorithms

