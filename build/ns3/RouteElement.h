/*
 * RouteElement.h
 *
 *  Created on: Dec 31, 2014
 *      Author: chen
 */

#ifndef ROUTEELEMENT_H_
#define ROUTEELEMENT_H_

#include "ns3/tinyxml.h"
#include "ns3/vector.h"

#include <string>
#include <map>
#include <iostream>
#include <vector>

namespace ns3
{
namespace vanetmobility
{
namespace sumomobility
{

#define ATTR_ID        1
#define ATTR_FROM      2
#define ATTR_TO        3
#define ATTR_PRIORITY  4
#define ATTR_INDEX     5
#define ATTR_SPEED     6
#define ATTR_LENGTH    7
#define ATTR_SHAPE     8
#define ATTR_DEPART    9
#define ATTR_EDGES     10
#define ATTR_TIME      11
#define ATTR_X         12
#define ATTR_Y         13
#define ATTR_ANGLE     14
#define ATTR_TYPE      15
#define ATTR_POS       16
#define ATTR_LANE      17
#define ATTR_SLOPE     18

const std::string originLanCharactor("/");
const std::string changeLaneCharactor("-");

void StringReplace(std::string&,const std::string&,const std::string&);

int getAttribuutID(const char* attribute);

struct Lane
{
	std::string id;
	int index;
	double speed;
	double length;
	std::string shape;
};

struct Edge
{
	std::string id;
	std::string from;
	std::string to;
	double priority;
	Lane   lane;
};

class RoadMap
{
public:
	RoadMap();
	RoadMap(const RoadMap& r);
	virtual ~RoadMap();
	void Clear(){edges.clear();};
	void LoadNetXMLFile(const char* pFilename);
	void printedges();
	const std::map<std::string,Edge>& getEdges()const;  //warning: the key is lane's id, not edges


private:
	std::map<std::string,Edge> edges;
	Edge m_temp_edge;
	bool edge_with_attribs(TiXmlElement* pElement,const char* str);//check whether "pElement" has "str" attribute
	int Read_edges(TiXmlElement* pElement);
	int Read_lane(TiXmlElement* pElement);
	void InitializeEdges( TiXmlNode* pParent);
    void ChangeLaneCharactor(Edge& edge);
    void ChangeLaneCharactor(Lane& lane);

};


class Route
{
public:
	Route();
	Route(const std::vector<std::string>& e);
	Route(const Route& r);
	void LoadRouteString(const char* pRouteStr);
	void printroute();
	//std::vector<std::string>& getedgesID();
	virtual ~Route();
	std::vector<std::string> edgesID;
};

class Trace
{
public:
	double time; //in second
	double x;    //x coordinate
	double y;    //y coordinate
	double angle;
	double speed;
	double pos;   //away from the start of the lane
	double slope; //slope
	std::string lane;  //at lane
	std::string type;  //vehicle type,default:"DEFAULT_VEHTYPE"

	bool operator == (const ns3::Vector& vector)const;
};

inline bool Trace::operator == (const ns3::Vector& vector)const
{
	//if(this->x == vector.x && this->y == vector.y && vector.z==0)
	if(this->x == vector.x && this->y == vector.y)
		return true;
	return false;
}

struct Vehicle
{
	int id;
	double depart;
	Route route;
	std::vector<Trace> trace;
};

class VehicleLoader
{
public:
	VehicleLoader();
	virtual ~VehicleLoader();
	VehicleLoader(const VehicleLoader& v);
	void LoadRouteXML(const char *  pXMLFilename);
	void LoadFCDOutputXML(const char *  pXMLFilename);
	void print_vehicle();
	const std::vector<Vehicle>& getVehicles() const;
	void Clear();

private:
	std::vector<Vehicle> vehicles;
	std::map<int,Vehicle> mapvehicles;
	Vehicle *m_temp_vehicle;
	Trace   m_temp_trace;
	void initialize_vehicles( TiXmlNode* pParent);
	int read_vehicle(TiXmlElement* pElement);
	void initialize_trace( TiXmlNode* pParent);
	int read_trace(TiXmlElement* pElement);//Return vehicle ID value
	void ReadMapIntoVector();
};

} /* namespace sumomobility */
} /* namespace vanetmobility */
} /* namespace ns3 */



#endif /* ROUTEELEMENT_H_ */
