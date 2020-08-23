#ifndef PARTICLE_H
#define PARTICLE_H

class Particle{
	public:
		Particle() : x(0), y(0),z(0),w(0) {}
		Particle(const int xx, const int yy, const int zz) : x(xx), y(yy),z(zz),w(0) {}
		Particle(const int xx, const int yy, const int zz, const double weight) : x(xx), y(yy),z(zz),w(weight) {}
		~Particle() {}
		bool operator<(const Particle &p) const {
			return this->score > p.score;
		}
		int x;
		int y;
		int z;
		double w;



		int rad;
		int radius;
		double score;
};
#endif // PARTICLE_H
