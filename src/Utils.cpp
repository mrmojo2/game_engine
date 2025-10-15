#include "Utils.h"
#include <random>

float Utils::get_random_float(float start, float end){
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(start,end);


	float randomValue = dis(gen);
	return randomValue;
}
