import sys
import math, re

from collections import OrderedDict
from collections import namedtuple
from collections import defaultdict 


def original():

	thresholdV = 15
	thetaDivisions = 180;
	thetaIncrement =  math.pi/ thetaDivisions;
	kinectData = parser("sample.txt")

	for kdata in kinectData:

		if kdata["maxY"] == '0' or kdata["maxX"] == '0':
			continue

		imgHeight = int(math.ceil(kdata["maxY"]) * 100)
		accumulator = [[0]*thetaDivisions]*(4 * imgHeight)
		dpKinectL = defaultdict(list)

		for data in kdata['dataPoints']:

			if data == 0:
				continue 

			for j in range(0,thetaDivisions):

				r = int(100 * ((data.x * math.cos(j * thetaIncrement))
					+ (data.y * math.sin(j * thetaIncrement))))
				k = j
				r += 2 * imgHeight

				if r >= ( 4 * imgHeight) or r < 0:
					continue

				accumulator[r][k] += 1 
				key = "" + str(r) + "" + str(k) 
				dpKinectL[key].append(data)


def parser(filepath):

	kinect_file = open(filepath,"rb")
	kinectData = []
	subgroup = OrderedDict()

	for line in kinect_file:

		m = re.match("(.*):(.*)", line)
		#Check if regex matches
		if m:
			key = m.kinectData()[0].strip()
			value =  m.kinectData()[1].strip()

			if key in subgroup:
				kinectData.append(subgroup)
				subgroup = dict()
			
			subgroup[key] = value

	for kdata in kinectData:
		kdata['ranges'] = kdata['ranges'].replace("[","").replace("]","").split(",")
	
	kinectData = process_kinect_data(kinectData)

	return kinectData


def process_kinect_data(kinectData):

	for data in kinectData:

		i = -1
		maxX, maxY, minY = -99999 , -99999 , 999999
		dataPoints = []
		Point = namedtuple('Point', 'x y')
		for krange in data['ranges']:

			i += 1
			krange = float(krange)
			theta = float(data['angle_min']) + ((float(data['angle_increment']) * i)) 
			
			if math.isnan(krange):
				continue

			else: 
				x = krange * math.cos(theta)
				y = krange * math.sin(theta)
				
				if(x > maxX):
					maxX = x

				if(y > maxY):
					maxY = y

				if(y < minY):
					minY = y
	
			dataPoints.append(Point(x,y))

		data["maxX"] = maxX
		data["maxY"] = maxY
		data["minY"] = minY
		data["dataPoints"] = dataPoints

	return kinectData

def main():

	original()

if __name__ == '__main__':
	main()