#include "VLP16.h"
// LINE 236 change maxSize Array

VLP16::VLP16()
{
	// INITIALIZATION

	/*for (long index = 0; index < 360; index++)
	{
		scan_indeces[index] = 0;
		scan_lengths[index] = 0;;
	}
	for (long index = 0; index < maxCones; index++)
	{
		cones[index] = 0;
	}*/
	start = 0;
	end = 0;

	// indeces for scanVector which stores the index of the first point of each scan inside pointVector (==data)
	first = 0;
	last = 0;
	
	totalScans = 0;
	//maxScans = 3;
	//scanLength = 4;
	//maxLength = 3333;
}

VLP16::~VLP16()
{}

void VLP16::addScan(long dataSize)
{
	if (!empty)
	{
		int temp = last;

		start = 0;
		first = (++first) % maxScans;
		last = (first + maxScans - 1) % maxScans;

		scan_lengths[last] = dataSize;
		scan_indeces[last] = scan_lengths[last];

		end = scan_indeces[last]% maxLength;
	}
	else
	{
		if (totalScans == 0)
		{
			scan_lengths[0] = dataSize;
			++totalScans;
		}
		else
		{
			scan_indeces[totalScans] = scan_indeces[totalScans - 1] + scan_lengths[totalScans - 1];
			scan_lengths[totalScans] = dataSize;

			//start = scan_indeces[first] % maxLength;
			end = scan_indeces[totalScans] % maxLength;

			++totalScans;
		}

		if (totalScans >= maxScans)
		{
			empty = false;
			totalScans = maxScans;
			last = maxScans - 1;
		}
	}
}

void VLP16::addPoint(double x, double y, double z)
{
	if(end < maxLength)
	{
		data[3 * end] = x;
		data[3 * end + 1] = y;
		data[3 * end + 2] = z;
		end = ++end;
	}
}

ostream& operator<<(ostream& out, const VLP16& vlp)
{
	out << "\n\n_____________________________________\nSESSION log file\n\n";
	
	for (long index = vlp.start; index != vlp.end;)
	{
		out << "[X,Y,Z] = " << vlp.data[3 * index] << ", " << vlp.data[3 * index + 1] << ", " << vlp.data[3 * index + 2] << "\n";
		index = (++index) % vlp.maxLength;
	}

	return out;
}

long VLP16::findInRadius2(double* point, float R, long* indices, double* distances, double meanDistance)
{
	double d;
	meanDistance = 0;
	long found = 0;
	//double indices[500];
	int element = 0;
	long indicesLength = 0;
	long tempLastIndex = lastIndex;

	meanDistance = 0;
	for (long index = 0; index < end;index++)
	{
		// INSERT CODE HERE
		d = sqrt(pow(data[3 * index] - point[0], 2) + pow(data[3 * index + 1] - point[1], 2));

		if (d < R && d != 0)
		{
			++found;
			if (found>MAX_NEIGHBOURS)
			{
				found = 0;
				break;
			}
			if (tempLastIndex < 3333)
			{
				meanDistance += d;
				indices[tempLastIndex] = index;
				++tempLastIndex;
				distances[element] = d;
				++element;
			}
			else
			{
				found = 0;
				break;
			}
		}

		//
	}
	meanDistance = meanDistance/element;
	//cout << "rms: " << meanDistance << "\n";
	if (found >= MIN_NEIGHBOURS  && meanDistance<SIGMA_DISTANCE) // unambiguous
		return element;
	else
		return 0;
}

void VLP16::process2()
{
	int nElementsOfIndices = 0;
	long indices[6666];
	double distances[70];
	double meanDistance = 0;
	bool existAsConePoint = false;

	totalCones = 0;
	lastIndex = 0;
	for (long index = 0; index < end;index++)
	{
		// INSERT CODE HERE

		//isMember
		for (int n = 0; n < lastIndex; n++)
		{ 
			if (index == indices[n]) // indeces is an array which stores indeces of points which are part of some cone
			{
				existAsConePoint = true;
				break;
			}
		}
		if (!existAsConePoint)
		{
			nElementsOfIndices = findInRadius2(&data[3 * index], RADIUS, &indices[0], &distances[0], meanDistance);

			if (nElementsOfIndices > 0 && (nElementsOfIndices + lastIndex)<maxLength)
			{
				double x = data[3 * index];
				double y = data[3 * index+1];
				double z = data[3 * index+2];
				for (int n = lastIndex; n < lastIndex+nElementsOfIndices; n++)
				{
					x += data[3 * indices[n]];
					y += data[3 * indices[n]+1];
					z += data[3 * indices[n]+2];
				}
				if(z / (nElementsOfIndices + 1)<MAX_HEIGHT)
				{
					cones[totalCones * 3] = x / (nElementsOfIndices + 1);
					cones[totalCones * 3 + 1] = y / (nElementsOfIndices + 1);
					cones[totalCones * 3 + 2] = z / (nElementsOfIndices + 1);
					lastIndex += nElementsOfIndices;
					++totalCones;
				}
				if(totalCones>=maxCones)
				{
					break;
				}
			}				
		}
		else
			existAsConePoint = false;
		//
		if(totalCones>=maxCones)
		{
			totalCones=0;
			break;
		}
	}
	
	/*for (int i = 0; i < totalCones; i++)
		cout << "[X,Y,Z]_" << i+1 << " = " << cones[3 * i] << ", " << cones[3 * i + 1] << ", " << cones[3 * i + 2] << "\n";*/
	cout << "Total cones: " << totalCones << "\n";
	
	busy = false;
}
