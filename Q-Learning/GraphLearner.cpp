#include "GraphLearner.h"

// Test comment 

GraphLearner::GraphLearner()
{
	srand(time(NULL));
	init_environment();
}


void GraphLearner::QLearning()
{
	for (int i = 0; i < 5000; i++)
	{
		
		graph.Init();			// Will place current node at starting node.
		cstate.current_node = graph.GetCurrentNode();	// Load current node into cstate.
		cstate.visited_nodes = (char)1;
		if (!(i % 1000))
			std::cout << i << std::endl;
		float sum_r = 0; // TEST
		int steps = 0;
		while ((int)cstate.visited_nodes != 255 && steps < 10)	// Should be after x time units.. / moves / all rooms visited
		{
			action a = GetNextAction(cstate);
			float reward = GetReward(cstate, a);
			state nstate = GetNextState(cstate, a);

			sum_r += reward;	// Used for plot

			Q[cstate.current_node->GetValue()][cstate.visited_nodes][a] = Q[cstate.current_node->GetValue()][cstate.visited_nodes][a] + alpha*(reward + discount_factor*GetHighestActionValue(nstate) - Q[cstate.current_node->GetValue()][cstate.visited_nodes][a]);

			cstate = nstate;

			steps++;	// Used for plot
		}
		xplot.push_back(i);
		yplot.push_back(sum_r);
		zplot.push_back(steps);
	}
}

state GraphLearner::GetNextState(state s, action a)
{
	s.visited_nodes |= 1UL << a;
	std::vector<Node*> neighs = s.current_node->GetNeighbors();
	bool valid = false;
	for (int i = 0; i < neighs.size(); i++)
	{
		if (neighs[i]->GetValue() == a)
		{
			s.current_node = neighs[i];
			valid = true;
		}
	}
	if (!valid)
		std::cout << "Invalid action" << std::endl;
	return s;
}

float GraphLearner::GetReward(state s, action a)		// This is used to simulate a reward ( R_{t+1} ).
{
	state ns = GetNextState(s, a);
	if (!((s.visited_nodes >> ns.current_node->GetValue()) & 1U))	// Check if next node have already been visited.
		return 10;
	else
		return -5;
	return 0;
}

action GraphLearner::GetNextAction(state s)
{
	if (greedy_or_nongreedy(epsilon) == GREEDY)							// This will decide if a random action or greedy action should be taken.
		return GetNextGreedyAction(s);			// Return the greedy action.
	std::vector<Node *> possible_nodes = s.current_node->GetNeighbors();	// Get vec of possible nodes.
	return actions[possible_nodes[rand() % possible_nodes.size()]->GetValue()];	// Choose a random node from the vec of possible nodes, get the value and choose appropriate action.
}

action GraphLearner::GetNextGreedyAction(state s)
{
	float current_max = -std::numeric_limits<float>::max();	// get the most negative number.

	std::vector<Node *> possible_nodes =s.current_node->GetNeighbors();	// Get vec of possible nodes.
	action best = actions[possible_nodes[0]->GetValue()];	// default action
	//std::cout << "best: " << best << std::endl;
	for (int i = 0; i < possible_nodes.size(); i++)
	{
		if ((Q[s.current_node->GetValue()][s.visited_nodes][actions[possible_nodes[i]->GetValue()]] > current_max || ((Q[s.current_node->GetValue()][s.visited_nodes][actions[i]] == current_max) && random_choice())))	// This will be true for:
		{														// next state is within environment AND ( action-value estimate is above current max OR ( action-value estimate is equal to current max AND random choice ) )
			best = actions[possible_nodes[i]->GetValue()];
			current_max = Q[s.current_node->GetValue()][s.visited_nodes][actions[possible_nodes[i]->GetValue()]];
		}
	}
	//std::cout << "best final: " << best << std::endl;
	return best;
}

float GraphLearner::GetHighestActionValue(state s)
{
	std::vector<Node *> possible_nodes = s.current_node->GetNeighbors();
	float current_max = -std::numeric_limits<float>::max();
	for (int i = 0; i < possible_nodes.size(); i++)
	{
		if (Q[s.current_node->GetValue()][s.visited_nodes][possible_nodes[i]->GetValue()] > current_max)
			current_max = Q[s.current_node->GetValue()][s.visited_nodes][possible_nodes[i]->GetValue()];
	}
	return current_max;
}

bool GraphLearner::greedy_or_nongreedy(float e)
{
	// Probability of choosing a random action: epsilon*(amount(actions)-1) / amount(actions)
	float random_num = (rand() % 1001) / 10.0;		// Generate random number between 0.0 and 100.0
	int amount_actions = cstate.current_node->GetNeighbors().size();

	float value = e*(amount_actions - 1) / (float)amount_actions * 100.0;	// The possibility of taking a non greedy action

																			//	std::cout << "Random num: " << random_num << std::endl;
																			//	std::cout << "Treshold: " << value << std::endl;

	if (random_num < value)
		return NONGREEDY;

	return GREEDY;
}

void GraphLearner::print_route()
{
	graph.Init();			// Will place current node at starting node.
	cstate.current_node = graph.GetCurrentNode();	// Load current node into cstate.
	cstate.visited_nodes = (char)1;
	std::cout << "NEW RUN" << std::endl;
	std::cout << "node: " << cstate.current_node->GetValue() << std::endl;

	while ((int)cstate.visited_nodes != 255)	// Should be after x time units.. / moves / all rooms visited
	{
		//std::cout << "Visited: " << (int)cstate.visited_nodes << std::endl;
		action a = GetNextGreedyAction(cstate);
		std::vector<Node*> possible = cstate.current_node->GetNeighbors();
		for (int i = 0; i < possible.size(); i++)
		{
			std::cout << action_text[possible[i]->GetValue()] <<": " << Q[cstate.current_node->GetValue()][cstate.visited_nodes][possible[i]->GetValue()] << std::endl;
		}

		state nstate = GetNextState(cstate, a);

		// Print estimates before for debugging						value_from_action should be replaced by a function that gives the highest action-value value for the next state.
		// INFO DONE

		cstate = nstate;
		std::cout << "node: " << cstate.current_node->GetValue() << std::endl;
		// print info for debugging

	}
}

float GraphLearner::GetRandomReward(float mean, float std_deviation) // will return a random variable generated from a normal distribution.
{
	std::random_device random_seed;
	std::default_random_engine random_engine(random_seed());
	std::normal_distribution<float> distribution(mean, std_deviation);
	return distribution(random_engine);
}


GraphLearner::~GraphLearner()
{
}

bool GraphLearner::random_choice()
{
	if (rand() % 101 > 50)
		return true;
	return false;
}

void GraphLearner::init_environment()
{
	/* Creation of the graph */
	Node* node0 = new Node(0);
	Node* node1 = new Node(1);
	Node* node2 = new Node(2);
	Node* node3 = new Node(3);
	Node* node4 = new Node(4);
	Node* node5 = new Node(5);
	Node* node6 = new Node(6);
	Node* node7 = new Node(7);

	graph.AddNode(node0);
	graph.AddNode(node1);
	graph.AddNode(node2);
	graph.AddNode(node3);
	graph.AddNode(node4);
	graph.AddNode(node5);
	graph.AddNode(node6);
	graph.AddNode(node7);

	node0->AddNeighbor(node1);
	node1->AddNeighbor(node0);
	node1->AddNeighbor(node2);
	node2->AddNeighbor(node1);
	node2->AddNeighbor(node3);
	node3->AddNeighbor(node2);
	node3->AddNeighbor(node6);
	node3->AddNeighbor(node7);
	node3->AddNeighbor(node4);
	node4->AddNeighbor(node3);
	node4->AddNeighbor(node5);
	node5->AddNeighbor(node4);
	node6->AddNeighbor(node3);
	node7->AddNeighbor(node3);

	graph.Init();
	amount_nodes = graph.GetAmountNodes();
}

std::vector<int> GraphLearner::get_xplot()
{
	return xplot;
}

std::vector<float> GraphLearner::get_yplot()
{
	return yplot;
}

std::vector<int> GraphLearner::get_zplot()
{
	return zplot;
}

