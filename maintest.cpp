#include<set>

#include<iostream>

template <typename T>
void prova(T &leggi)
{
	leggi.insert(std::pair(9,3));
	leggi.insert(std::pair(7, 13));
}

int main()
{
	auto compare = [](std::pair<int, double> a, std::pair<int, double> b) {return a.first != b.first && a.second <= b.second;};
	std::set < std::pair<int, double>, decltype(compare) > prova1(compare);
	prova1.insert(std::pair(7, 12));
	prova1.insert(std::pair(6, 12));
	prova1.insert(std::pair(1, 15));
	auto it = prova1.insert(std::pair(1, 14));
	prova1.erase(it.first);
	prova1.insert(std::pair(1, 14));
	prova(prova1);
	for (auto i = prova1.begin(); i != prova1.end(); i++)
	{
		std::cout << i->first << " " << i->second<<std::endl;
	}
}

