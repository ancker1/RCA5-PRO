#include "C:\Users\An-ck\OneDrive\Dokumenter\GitHub\RCA5-PRO\Q-Learning\GraphLearner.h"
#include "C:\Users\An-ck\OneDrive\Dokumenter\GitHub\RCA5-PRO\Q-Learning\Graph.h"
#include "C:\Users\An-ck\OneDrive\Dokumenter\GitHub\RCA5-PRO\Q-Learning\Node.h"
#include "C:\Users\An-ck\OneDrive\Dokumenter\GitHub\RCA5-PRO\Q-Learning\LinkedList.h"
#include "C:\Users\An-ck\OneDrive\Dokumenter\GitHub\RCA5-PRO\Q-Learning\LinkNode.h"
#include <string>
#include <fstream>
#include <iostream>
#include <thread>

/*
	Name:			Source.cpp
	Description:	This source file is used to test the implemented Q-Learning.
	
	Comment:		To perform the test - edit the file paths stated above in the includes.
*/


void run_test(float e, float b, float a, float g);
void run_mem(float e, float b, float a, float g);
int main()
{

	/* Test of memory usage */
	float e = 0.20;
	float decay = 0.999;
	float a = 0.3;
	float g = 0.2;
	run_mem(e, decay, a, g);
	std::cout << "Memory test: DONE." << std::endl;
	return 0;


	float epsilons[3] =		{ 0.010, 0.10, 0.20 };
	//float epsilon = 0.20;									// Test was performed on 3 computers simultanously; each computer having different values of epsilon.
	float decays[2] =		{ 0.999, 1.00 };
	float alphas[4] =		{ 0.100, 0.20, 0.30, 0.50 };
	float discounts[3] =	{ 0.100, 0.20, 0.30 };

	/*	 This is used to generate plots of chosen combinations -> will result in 76 data sets containing 5M floats in each.	*/
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			for (int k = 0; k < 4; k++)
			{
				for (int z = 0; z < 3; z++)
				{
					std::cout << "e" << epsilons[i] << "d" << decays[j] << "a" << alphas[k] << "g" << discounts[z] << std::endl;
					run_test(epsilons[i], decays[j], alphas[k], discounts[z]);

				}
			}
		}
	}

	std::cout << "Done" << std::endl;

	return 0;
}

void run_test(float e, float b, float a, float g)
{
	GraphLearner* learners[5];

	for (int i = 0; i < 5; i++)
		learners[i] = new GraphLearner(e, b, a, g);

	string file_name = "e" + std::to_string(learners[0]->GetEpsilon()) + "d" + std::to_string(learners[0]->GetEpsilonDecay()) + "a" + std::to_string(learners[0]->GetLearningRate()) + "g" + std::to_string(learners[0]->GetDiscountFactor()) + ".txt";

	std::cout << "Initiating test: " << file_name << std::endl;

	std::thread first(&GraphLearner::QLearning, learners[0]);
	std::thread second(&GraphLearner::QLearning, learners[1]);
	std::thread third(&GraphLearner::QLearning, learners[2]);
	std::thread fourth(&GraphLearner::QLearning, learners[3]);
	std::thread fifth(&GraphLearner::QLearning, learners[4]);

	first.join();
	second.join();
	third.join();
	fourth.join();
	fifth.join();

	std::vector<float> data = learners[0]->get_yplot();
	std::vector<float> temp = learners[1]->get_yplot();
	data.insert(data.end(), temp.begin(), temp.end());
	temp = learners[2]->get_yplot();
	data.insert(data.end(), temp.begin(), temp.end());
	temp = learners[3]->get_yplot();
	data.insert(data.end(), temp.begin(), temp.end());
	temp = learners[4]->get_yplot();
	data.insert(data.end(), temp.begin(), temp.end());

	std::cout << "y.size(): " << data.size() << std::endl;


	std::cout << "Saving to file: " << file_name << std::endl;
	std::ofstream dataTXT(file_name);
	dataTXT.close();
	dataTXT.open(file_name, std::ios_base::app);
	for (int i = 0; i < data.size(); i++)
	{
		if (dataTXT.is_open())
		{
			dataTXT << data[i] << "\n";
		}
		else
			std::cout << "FAIL" << std::endl;
	}
	dataTXT.close();

	for (int i = 0; i < 5; i++)
		delete learners[i];
}



void run_mem(float e, float b, float a, float g)
{
	GraphLearner* learners[5];

	for (int i = 0; i < 5; i++)
		learners[i] = new GraphLearner(e, b, a, g);

	string file_name = "memory_test.txt";

	std::cout << "Initiating test: " << file_name << std::endl;

	std::thread first(&GraphLearner::QLearning, learners[0]);
	std::thread second(&GraphLearner::QLearning, learners[1]);
	std::thread third(&GraphLearner::QLearning, learners[2]);
	std::thread fourth(&GraphLearner::QLearning, learners[3]);
	std::thread fifth(&GraphLearner::QLearning, learners[4]);

	first.join();
	second.join();
	third.join();
	fourth.join();
	fifth.join();

	std::vector<int> data = learners[0]->get_xplot();
	std::vector<int> temp = learners[1]->get_xplot();
	data.insert(data.end(), temp.begin(), temp.end());
	temp = learners[2]->get_xplot();
	data.insert(data.end(), temp.begin(), temp.end());
	temp = learners[3]->get_xplot();
	data.insert(data.end(), temp.begin(), temp.end());
	temp = learners[4]->get_xplot();
	data.insert(data.end(), temp.begin(), temp.end());

	std::cout << "x.size(): " << data.size() << std::endl;


	std::cout << "Saving to file: " << file_name << std::endl;
	std::ofstream dataTXT(file_name);
	dataTXT.close();
	dataTXT.open(file_name, std::ios_base::app);
	for (int i = 0; i < data.size(); i++)
	{
		if (dataTXT.is_open())
		{
			dataTXT << data[i] << "\n";
		}
		else
			std::cout << "FAIL" << std::endl;
	}
	dataTXT.close();

	for (int i = 0; i < 5; i++)
		delete learners[i];
}