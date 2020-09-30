#include"ConsecutiveRandoms.h"

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


template<class T>
ConsecutiveRandoms<T>::ConsecutiveRandoms(T min, T max)
{
	this->nodes_id_generator = new uniform_distribution<T>(min, max);
}

template<class T>
T ConsecutiveRandoms<T>::generate()
{
	static std::default_random_engine default_engine(std::time(0));
	return this->nodes_id_generator->operator()(default_engine);
}

template<class T>
T ConsecutiveRandoms<T>::generate(T min, T max)
{
	static std::default_random_engine default_engine(std::time(0));
	return uniform_distribution<T>(min, max)(default_engine);
}

