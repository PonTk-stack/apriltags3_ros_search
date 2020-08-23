#include <random>
#include <algorithm>
#include "tag_particle.h"

TagParticle::TagParticle() : particle_num(100), random_particle_num(30), surviving_particle_num(5), particle(particle_num){
	// set initial particles at the center of the image.
	
//	Particle p(center_width, center_height);
	Particle p(0,0,0);
	for(int i = 0; i < surviving_particle_num; i++) {
		particle[i] = p;
	}
}
TagParticle::~TagParticle(){
}
std::vector<Particle> TagParticle::getParticle(void){
	return particle;
}

/*
std::vector<Particle>  TagParticle::getTagParticle(void){
	std::vector<Particle> ret; 
	return ret;
}
*/
