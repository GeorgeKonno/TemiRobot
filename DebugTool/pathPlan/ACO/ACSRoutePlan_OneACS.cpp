/*
AOC
*/

#include <iostream>
#include <cmath>
#include <time.h>
#include <vector>
#include <fstream>

using namespace std;

typedef struct _POINT {
	int x;
	int y;
}POINT;

POINT points;

//#define N 30
//
//double C[N][2]={
//    { 2,99},{ 4,50},{ 7,64},{13,40},{18,54},{18,40},{22,60},{24,42},{25,62},{25,38},
//    {37,84},{41,94},{41,26},{44,35},{45,21},{54,67},{54,62},{58,35},{58,69},{62,32},
//    {64,60},{68,58},{71,44},{71,71},{74,78},{82, 7},{83,46},{83,69},{87,76},{91,38}};


#define N 32

int C[N][2] = {
	{ 2,99 },{ 4,50 },{ 7,64 },{ 13,40 },{ 18,54 },{ 18,40 },{ 20, 6 },{ 21,78 },{ 22,60 },{ 24,42 },
{ 25,62 },{ 25,38 },{ 37,84 },{ 41,94 },{ 41,26 },{ 44,35 },{ 45,21 },{ 54,67 },{ 54,62 },{ 58,35 },
{ 58,69 },{ 62,32 },{ 64,60 },{ 68,58 },{ 71,44 },{ 71,71 },{ 74,78 },{ 82, 7 },{ 83,46 },{ 83,69 },
{ 87,76 },{ 91,38 } };


#define M 30

int NcMax = 500;

double alpha = 2, beta = 3, rou = 0.1, alpha1 = 0.1,  qzero = 0.01;


double Lnn;
double allDistance[N][N];
double calculateDistance(int i, int j)
{
	return sqrt(pow((C[i][0] - C[j][0]), 2.0) + pow((C[i][1] - C[j][1]), 2.0));
}
void calculateAllDistance()
{
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			if (i != j) {
				allDistance[i][j] = calculateDistance(i, j);
				allDistance[j][i] = allDistance[i][j];
			}
		}
	}
}

double calculateSumOfDistance(int* tour)
{
	double sum = 0;
	for (int i = 0; i < N; i++) {
		int row = *(tour + 2 * i);
		int col = *(tour + 2 * i + 1);
		sum += allDistance[row][col];
	}

	return sum;
}

class ACSAnt;
class AntColonySystem
{
private:

	double info[N][N], visible[N][N];//�ڵ�֮�����Ϣ��ǿ��,�ڵ�֮����ܼ���
public:
	AntColonySystem() {

	}
	double Transition(int i, int j);
	void UpdateLocalPathRule(int i, int j);
	void InitParameter(double value);
	void UpdateGlobalPathRule(int* bestTour, int globalBestLength);

};
double AntColonySystem::Transition(int i, int j)
{
	if (i != j) {
		return (pow(info[i][j], alpha) * pow(visible[i][j], beta));
	}
	else {
		return 0.0;
	}
}
void AntColonySystem::UpdateLocalPathRule(int i, int j)
{
	info[i][j] = (1.0 - alpha1) * info[i][j] + alpha1 * (1.0 / (N * Lnn));
	info[j][i] = info[i][j];
}

void AntColonySystem::InitParameter(double value)
{
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			info[i][j] = value;
			info[j][i] = value;

			if (i != j) {
				visible[i][j] = 1.0 / allDistance[i][j];
				visible[j][i] = visible[i][j];
			}
		}
	}
}

void AntColonySystem::UpdateGlobalPathRule(int* bestTour, int globalBestLength)
{
	for (int i = 0; i < N; i++) {
		int row = *(bestTour + 2 * i);
		int col = *(bestTour + 2 * i + 1);

		info[row][col] = (1.0 - rou) * info[row][col] + rou * (1.0 / globalBestLength);
		info[col][row] = info[row][col];
	}
}

class ACSAnt
{
private:
	AntColonySystem * antColony;

protected:
	int startCity, cururentCity;
	int allowed[N];

	int Tour[N][2];
	int currentTourIndex;

public:
	ACSAnt(AntColonySystem* acs, int start) {
		antColony = acs;
		startCity = start;
	}

	int* Search();

	int Choose();

	void MoveToNextCity(int nextCity);
};

int* ACSAnt::Search()
{
	cururentCity = startCity;

	int toCity;
	currentTourIndex = 0;
	for (int i = 0; i < N; i++) {
		allowed[i] = 1;
	}
	allowed[cururentCity] = 0;

	int endCity;
	int count = 0;

	do {
		count++;
		endCity = cururentCity;

		toCity = Choose();
		if (toCity >= 0) {
			MoveToNextCity(toCity);
			antColony->UpdateLocalPathRule(endCity, toCity);
			cururentCity = toCity;
		}

	} while (toCity >= 0);

	MoveToNextCity(startCity);
	antColony->UpdateLocalPathRule(endCity, startCity);

	return *Tour;
}

int ACSAnt::Choose()
{
	int nextCity = -1;
	double q = rand() / (double)RAND_MAX;

	if (q <= qzero) {
		double probability = -1.0;
		for (int i = 0; i < N; i++)
		{
			if (1 == allowed[i])
			{
				double prob = antColony->Transition(cururentCity, i);
				if (prob > probability)
				{
					nextCity = i;
					probability = prob;
				}
			}
		}
	}
	else {		
		double p = rand() / (double)RAND_MAX;
		double sum = 0.0;
		double probability = 0.0;
		for (int i = 0; i < N; i++) {
			if (1 == allowed[i]) {
				sum += antColony->Transition(cururentCity, i);
			}
		}

		for (int j = 0; j < N; j++) {
			if (1 == allowed[j] && sum > 0) {
				probability += antColony->Transition(cururentCity, j) / sum;
				if (probability >= p || (p > 0.9999 && probability > 0.9999)) {
					nextCity = j;
					break;
				}
			}
		}
	}

	return nextCity;
}

void ACSAnt::MoveToNextCity(int nextCity)
{
	allowed[nextCity] = 0;
	Tour[currentTourIndex][0] = cururentCity;
	Tour[currentTourIndex][1] = nextCity;
	currentTourIndex++;
	cururentCity = nextCity;
}

int ChooseNextNode(int currentNode, int visitedNode[])
{
	int nextNode = -1;
	double shortDistance = 0.0;
	for (int i = 0; i < N; i++) {
		if (1 == visitedNode[i]) {
			if (shortDistance == 0.0) {
				shortDistance = allDistance[currentNode][i];
				nextNode = i;
			}

			if (shortDistance > allDistance[currentNode][i]) {
				shortDistance = allDistance[currentNode][i];
				nextNode = i;
			}
		}
	}

	return nextNode;
}

double CalAdjacentDistance(int node) {
	double sum = 0.0;
	int visitedNode[N];
	for (int j = 0; j < N; j++) {
		visitedNode[j] = 1;
	}
	visitedNode[node] = 0;
	int currentNode = node;
	int nextNode;

	do {
		nextNode = ChooseNextNode(currentNode, visitedNode);
		if (nextNode >= 0) {
			sum += allDistance[currentNode][nextNode];
			currentNode = nextNode;
			visitedNode[currentNode] = 0;
		}
	} while (nextNode >= 0);

	sum += allDistance[currentNode][node];

	return sum;
}

int main() {
	time_t timer, timerl;
	time(&timer);
	unsigned long seed = timer;
	seed %= 56000;
	srand((unsigned int)seed);

	calculateAllDistance();

	AntColonySystem* acs = new AntColonySystem();
	ACSAnt* ants[M];
	for (int k = 0; k < M; k++) {
		ants[k] = new ACSAnt(acs, (int)(k%N));
	}
	calculateAllDistance();

	int node = rand() % N;
	Lnn = CalAdjacentDistance(node);

	double initInfo = 1 / (N * Lnn);
	acs->InitParameter(initInfo);

	int globalTour[N][2];

	double globalBestLength = 0.0;
	for (int i = 0; i < NcMax; i++) {
		int localTour[N][2];

		double localBestLength = 0.0;

		double tourLength;
		for (int j = 0; j < M; j++) {
			int* tourPath = ants[j]->Search();
			tourLength = calculateSumOfDistance(tourPath);
			if (tourLength < localBestLength || abs(localBestLength) < 0.000001) {
				for (int m = 0; m < N; m++) {
					int row = *(tourPath + 2 * m);
					int col = *(tourPath + 2 * m + 1);
					localTour[m][0] = row;
					localTour[m][1] = col;
				}
				localBestLength = tourLength;
			}
		}
		if (localBestLength < globalBestLength || abs(globalBestLength) < 0.000001) {
			for (int m = 0; m < N; m++) {
				globalTour[m][0] = localTour[m][0];
				globalTour[m][1] = localTour[m][1];
			}
			globalBestLength = localBestLength;
		}
		acs->UpdateGlobalPathRule(*globalTour, globalBestLength);


		for (int m = 0; m < N; m++) {
			cout << localTour[m][0] << "-";
		}
		cout << endl;
	}

	vector<POINT> vect;
	POINT points;
	for (int m = 0; m < N; m++)
	{
		cout << globalTour[m][0] << "-";
		int s = globalTour[m][0];
		points.x = C[s][0];
		points.y = C[s][1];
		vect.push_back(points);
	}
	cout << endl;

	ofstream files("robot.txt");
	for (vector<POINT>::iterator it = vect.begin(); it != vect.end(); it++)
	{
		files << it->x << "," << it->y << endl;
	}
	files.close();

	time(&timerl);

	int t = timerl - timer;

	return 0;
}