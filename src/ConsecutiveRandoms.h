#pragma once
#include <random>
#include <ctime>

//template that transforms uniform_distribution in std::uniform_int_distribution<T> in case of integers or in uniform_real_distribution<T> in case of floating point 
template<class T>
using uniform_distribution =
typename std::conditional<
	std::is_floating_point<T>::value,
	std::uniform_real_distribution<T>,
	typename std::conditional<
	std::is_integral<T>::value,
	std::uniform_int_distribution<T>,
	void
	>::type
>::type;


/**
* class that generates random numbers in a given interval.
* numbers can be either integers or floating point
* 
*/
template<class T>
class ConsecutiveRandoms
{
public:

	/**
	* contructor
	* 
	* input:
	* min: minimum value of the interval
	* max: minimum value of the interval
	* 
	* in case of integers the inveral is closed [min; max]
	* otherwise is right open [min; max)
	* 
	*/
	ConsecutiveRandoms(T min, T max);

	/**
	* function that generates a random number
	* 
	* output:
	* random number in the given interval
	* 
	*/
	T generate();

	/**
	* static function used when consecutive randoms are not needed
	* 
	* input:
	* min: minimum value of the interval
	* max: minimum value of the interval
	* 
	* output:
	* random number in the given interval
	* 
	*/
	static T generate(T min, T max);

private:

	uniform_distribution<T>* nodes_id_generator;

};

//needed because otherwise the template is not usable in the header file
#include "ConsecutiveRandoms.cpp"