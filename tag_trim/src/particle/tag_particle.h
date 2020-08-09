#ifndef TAG_PARTICLE_H
#define TAG_PARTICLE_H

#include <vector>
//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include "particle.h"
//#include "../object_types.h"

class TagParticle
{
public:
	TagParticle();
	~TagParticle();
//	void apply(cv::Mat &, std::vector<object_pos> &);
	std::vector<Particle> getParticle(void);

private:
	int particle_num;
	int random_particle_num;
	const int surviving_particle_num;
	std::vector<Particle> particle;
};

#endif // BALL_PARTICLE_H

