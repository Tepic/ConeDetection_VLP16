#ifndef _VLP16_H
#define _VLP16_H

#include "CONSTANTS.h"

class VLP16
{
public:
	VLP16();
	~VLP16();
	void addPoint(double x, double y, double z);
	void addScan(long dataSize);
	void run()
	{
		if (totalScans == maxScans)
		{
			busy = true;
			process2();
			end = 0;
		}
	}

	friend ostream& operator<<(ostream& os, const VLP16& vlp);

	// ALGORITHM FUNCTION
	long findInRadius2(double* point, float R, long* indices, double* distances, double meanDistance);
	void process2();
	bool running() { return busy; }
	long conesLength(){return totalCones; }
	double* getCones(){return &cones[0];}
	bool ready(){return readyToPub;}
	
	void doPub() {readyToPub = true;}
	void donePub(){readyToPub = false;}

private:
	long start, end;
	int first, last;

	long totalScans;
	
	// max number of scans
	long maxScans   = 1;			// size of scan_indeces and scan_lengths has to match initialization of maxScans in CONSTRUCTOR
	long scan_indeces[1]={0};
	long scan_lengths[1]={0};

	long maxLength = 6666; // maximum points that data has 3*maxLength due to x,y,z coordinates
	double data[3*6666];
	//long indices[6666];
	long lastIndex = 0;
	// data[0] <=> x0
	// data[1] <=> y0
	// data[2] <=> z0
	// data[3] <=> x1
	//		...
	// data[3n] <=> xn
	// data[3n+1] <=> yn
	// data[3n+2] <=> zn

	long totalCones = 0;
	long maxCones = 33;
	double cones[3*33];

	bool data_initialized = false;

	bool empty = true;
	bool busy = false;
	bool readyToPub = false;
};
#endif
