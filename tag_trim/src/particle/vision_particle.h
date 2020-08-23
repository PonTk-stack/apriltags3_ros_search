#ifndef TAG_PARTICLE_Y_H
#define TAG_PARTICLE_Y_H

class TagParticleT{
	public:
		TagParticleT() : x(0),y(0),radius(0),score(0){}
		TagParticleT(const int xx,const int yy,const int radiust,const float scoree): x(xx),y(yy),radius(radiust),score(scoree){}

		constexpr int w = 1280; // TODO: get these values from vision module.
		constexpr int h = 720;
		constexpr int center_width = w / 2;
		constexpr int center_height = h / 2;

		int x;
		int y;
		int radius;
		float score;
}
#endif
