#ifndef IMAGE_GRABBER_H
#define IMAGE_GRABBER_H

#include"camera.h"
#include<vector>
class ImageGrabber:Camera{

	public:
		ImageGrabber(){};
		void getA(){std::cout << Camera::A << std::endl;};

	private:
		int i ;

};
#endif //IMAGE_GRABBER_H
