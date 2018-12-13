#include "GraphLearner.h"

// Test comment 

GraphLearner::GraphLearner(float e, float b, float a, float g)
{
	srand(time(NULL));
	init_environment();
	

	this->discount_factor = g;
	this->learning_rate = a;
	this->epsilon = e;
	this->epsilon_decay = b;

	epsilon_org = epsilon;
}


void GraphLearner::QLearning()
{
	for (int j = 0; j < 100; j++)
	{
		Q = new LinkedList(*new LinkNode(0, 0, 0, 0));
		epsilon = epsilon_org;
		for (int i = 0; i < 10000; i++)
		{

			graph.Init();			// Will place current node at starting node.
			//graph.PrintDistribution();

			cstate.current_node = graph.GetCurrentNode();	// Load current node into cstate.
			cstate.visited_nodes = (char)1;

			float sum_r = 0; // TEST
			int steps = 0;
			float change = 0;
			
			while ((int)cstate.visited_nodes != VISITED_ALL && steps <= AMOUNT_STEPS)	// Should be after x time units.. / moves / all rooms visited
			{
				action a = GetNextAction(cstate);
				float reward = GetReward(cstate, a);
				state nstate = GetNextState(cstate, a);

				sum_r += reward;	// Used for plot

				int node = cstate.current_node->GetValue();
				float q_val = Q->GetValue(node, cstate.visited_nodes, a);
				float updated_Q = q_val + learning_rate*(reward + discount_factor*GetHighestActionValue(nstate) - q_val);
				Q->SetValue(node, cstate.visited_nodes, a, updated_Q);

				cstate = nstate;
				
				steps++;	// Used for plot
				change = abs(q_val - updated_Q);
			}
			epsilon = epsilon*epsilon_decay;
			xplot.push_back(epsilon);
			yplot.push_back(sum_r);
			//GetRewardSum();
			zplot.push_back(change / steps);

		}
		std::cout << j << std::endl;
		
}
	//Q2->print_data();
	//std::cout << "Size, Q: " << Q->GetSize() << std::endl;

}

state GraphLearner::GetNextState(state s, action a)
{
	s.visited_nodes |= 1UL << a;			// Update state.visited
	std::vector<Node*> neighs = s.current_node->GetNeighbors();
	bool valid = false;
	for (int i = 0; i < neighs.size(); i++)	// Check if action is valid.
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
		return GetRandomReward(ns.current_node->GetMean(), ns.current_node->GetStdDev());		// Return the reward of the next state, if it have not been visited
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
	for (int i = 0; i < possible_nodes.size(); i++)
	{
		float q_val = Q->GetValue(s.current_node->GetValue(), s.visited_nodes, actions[possible_nodes[i]->GetValue()]);
		if ((q_val > current_max || ((q_val == current_max) && random_choice())))	// This will be true for:
		{														// next state is within environment AND ( action-value estimate is above current max OR ( action-value estimate is equal to current max AND random choice ) )
			best = actions[possible_nodes[i]->GetValue()];
			current_max = q_val;
		}
	}
	return best;
}

float GraphLearner::GetHighestActionValue(state s)
{
	std::vector<Node *> possible_nodes = s.current_node->GetNeighbors();
	float current_max = -std::numeric_limits<float>::max();
	for (int i = 0; i < possible_nodes.size(); i++)
	{
		float q_val = Q->GetValue(s.current_node->GetValue(),s.visited_nodes, possible_nodes[i]->GetValue());
		if (q_val > current_max)
			current_max = q_val;
	}
	return current_max;
}

bool GraphLearner::greedy_or_nongreedy(float e)
{
	// Probability of choosing a random action: epsilon*(amount(actions)-1) / amount(actions)
	float random_num = (rand() % 10001) / 100.0;		// Generate random number between 0.00 and 100.00
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
	int steps = 0;
	while ((int)cstate.visited_nodes != VISITED_ALL && steps <= AMOUNT_STEPS)	// Should be after x time units.. / moves / all rooms visited
	{
		//std::cout << "Visited: " << (int)cstate.visited_nodes << std::endl;
		action a = GetNextGreedyAction(cstate);
		std::vector<Node*> possible = cstate.current_node->GetNeighbors();
		for (int i = 0; i < possible.size(); i++)
		{
			std::cout << action_text[possible[i]->GetValue()] << ": " << Q->GetValue(cstate.current_node->GetValue(), cstate.visited_nodes, possible[i]->GetValue()) << std::endl;
		}

		state nstate = GetNextState(cstate, a);

		// Print estimates before for debugging						value_from_action should be replaced by a function that gives the highest action-value value for the next state.
		// INFO DONE

		cstate = nstate;
		std::cout << "node: " << cstate.current_node->GetValue() << std::endl;
		// print info for debugging
		steps++;
	}
}

float GraphLearner::GetRandomReward(float mean, float std_deviation) // will return a random variable generated from a normal distribution.
{
	std::random_device random_seed;
	std::default_random_engine random_engine(random_seed());
	std::normal_distribution<float> distribution(mean, std_deviation);
	float rew = round(distribution(random_engine));
	if (rew < 0)
		rew = 0;
	return rew * 10;
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
	Node* node0  = new Node(0,  1.0, 0.1);
	Node* node1  = new Node(1,  1.0, 1.0);
	Node* node2  = new Node(2,  2.0, 0.5);
	Node* node3  = new Node(3,  2.7, 0.4);
	Node* node4  = new Node(4,  1.0, 0.5);
	Node* node5  = new Node(5,  2.0, 0.4);
	Node* node6  = new Node(6,  2.1, 0.5);
	Node* node7  = new Node(7,  2.0, 0.4);
	Node* node8  = new Node(8,  3.0, 0.5);
	Node* node9  = new Node(9,  2.0, 0.5);
	Node* node10 = new Node(10, 4.0, 1.0);
	Node* node11 = new Node(11, 2.7, 0.5);
	Node* node12 = new Node(12, 1.0, 0.5);
	Node* node13 = new Node(13, 1.0, 0.5);
	Node* node14 = new Node(14, 2.0, 0.5);
	Node* node15 = new Node(15, 1.0, 0.5);

	graph.AddNode(node0);
	graph.AddNode(node1);
	graph.AddNode(node2);
	graph.AddNode(node3);
	graph.AddNode(node4);
	graph.AddNode(node5);
	graph.AddNode(node6);
	graph.AddNode(node7);
	graph.AddNode(node8);
	graph.AddNode(node9);
	graph.AddNode(node10);
	graph.AddNode(node11);
	graph.AddNode(node12);
	graph.AddNode(node13);
	graph.AddNode(node14);
	graph.AddNode(node15);



	node0->AddNeighbor({ node1, node11, node3 });
	node1->AddNeighbor({ node0, node2,  node14 });
	node2->AddNeighbor({ node1 });
	node3->AddNeighbor({ node0, node4, node6});
	node4->AddNeighbor({ node3, node5 });
	node5->AddNeighbor({ node4, node6 });
	node6->AddNeighbor({ node5, node7 });
	node7->AddNeighbor({ node8, node9 });
	node8->AddNeighbor({ node7 });
	node9->AddNeighbor({ node7, node10 });
	node10->AddNeighbor({ node9 });
	node11->AddNeighbor({ node0, node12, node13 });
	node12->AddNeighbor({ node11 });
	node13->AddNeighbor({ node11 });
	node14->AddNeighbor({ node1, node15 });
	node15->AddNeighbor({ node14 });

	graph.Init();
	amount_nodes = graph.GetAmountNodes();
}

float GraphLearner::GetEpsilon()
{
	return epsilon_org;
}

float GraphLearner::GetLearningRate()
{
	return learning_rate;
}

float GraphLearner::GetDiscountFactor()
{
	return discount_factor;
}

float GraphLearner::GetEpsilonDecay()
{
	return epsilon_decay;
}

std::vector<int> GraphLearner::get_xplot()
{
	return xplot;
}

std::vector<float> GraphLearner::get_yplot()
{
	return yplot;
}

std::vector<float> GraphLearner::get_zplot()
{
	return zplot;
}

