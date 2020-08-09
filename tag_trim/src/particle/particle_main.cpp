#include "tag_particle.h"
#include "iostream"


int main(){
	TagParticle a ;
	std::vector<Particle> pars = a.getParticle();
	std::cout << pars[0].x <<std::endl ;
	int i =0;
	for(Particle p : pars){
		i++;
		std::cout << i<<std::endl ;


	}
	std::cout << "aaa"<<std::endl ;


	return 0;
}
