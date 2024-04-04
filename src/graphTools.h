#ifndef GENERAL_H
#define GENERAL_H
#include <set>
//#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp" //LMEDS
#include <vector>
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <map>
#include <boost/property_map/property_map.hpp> //property map
//#include <boost/variant/get.hpp> //get function
#include <boost/graph/copy.hpp>
#include <utility>
#include "disturbance.h"

// enum M_CODES {THREE_M=3, FOUR_M=4};

// enum GRAPH_CONSTRUCTION {BACKTRACKING, A_STAR, A_STAR_DEMAND, E};

typedef std::vector <float> DistanceVector;

struct Edge{
	Direction direction=DEFAULT;
	float probability=1.0;
	int step=0;
	//State*ID=this;

	Edge()=default;
};


struct State{
	Disturbance disturbance; //disturbance encounters
	b2Transform endPose = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0)); 
	simResult::resultType outcome;
	std::vector <Direction> options;
	int nodesInSameSpot =0;
	bool filled =0;
	int nObs=0;
	State* ID=this;
	float phi=10.0; //arbitrarily large phi

	
	State()=default;

	bool visited(){
		return phi<=1.0;
	}

	void resetVisited(){
		phi=10.0;
	}
};


typedef b2Transform Transform;
bool operator!=(Transform const &, Transform const &);
bool operator==(Transform const &, Transform const &);
void operator-=(Transform &, Transform const&);

typedef boost::adjacency_list<boost::setS, boost::vecS, boost::bidirectionalS, State, Edge> TransitionSystem;
typedef boost::graph_traits<TransitionSystem>::vertex_iterator vertexIterator; 
typedef boost::graph_traits<TransitionSystem>::vertex_descriptor vertexDescriptor;
typedef boost::graph_traits<TransitionSystem>::edge_descriptor edgeDescriptor;
typedef boost::graph_traits<TransitionSystem>::edge_iterator edgeIterator;

struct Connected{
	Connected(){}
	Connected(TransitionSystem * ts): g(ts){}
	
	bool operator()(const vertexDescriptor& v)const{
	 	bool in= boost::in_degree(v, *g)>0;
		bool out =boost::out_degree(v, *g)>0;
	 	return in || out;
	}
private:
TransitionSystem * g;
};

struct Visited{
	Visited(){}
	Visited(TransitionSystem*ts):g(ts){}

	bool operator()(const vertexDescriptor&v )const{
		return (*g)[v].phi<2.0;
	}

	private:
	TransitionSystem*g;
};

struct Memorable{
	Memorable(){}
	Memorable(TransitionSystem* ts):g(ts){}

	bool operator()(const vertexDescriptor& v){//const
		bool result=true;
		// if ((*g)[e].probability<FORGET_THRESHOLD){
		// 	result=false;
		// 	forget.insert(e);
		// }
		// auto es=boost::in_edges(e.m_source, *g);
		// for (auto ei=es.first; ei!=es.second; ei++){
		// 	for (edgeDescriptor ed:forget){
		// 		if (*ei==ed){
		// 			result=false;
		// 			break;
		// 		}
		// 	}
		// }
		return result;
	}

	private: 
	TransitionSystem *g;
	std::set <edgeDescriptor> forget;
};


namespace gt{

	std::pair<State, Edge> fill(simResult);

	int simToMotorStep(int);

	void update(edgeDescriptor,  std::pair <State, Edge>, TransitionSystem&, bool, std::unordered_map<State*, float>&); //returns disturbance rror based on expected vs observed D

	void set(edgeDescriptor,  std::pair <State, Edge>, TransitionSystem&, bool, std::unordered_map<State*, float>&);
}


typedef boost::filtered_graph<TransitionSystem, boost::keep_all, Connected> FilteredTS;

struct StateMatcher{
        std::vector <float> weights; //disturbance, position vector, angle
		//assume mean difference 0
		//std::vector <float> SDvector={0.03, 0.03, 0, 0.08, 0.08, M_PI/6};//hard-coded standard deviations for matching
		
		struct Error{
			const float endPosition=0.03;
			const float angle= M_PI/6;
			const float dPosition= 0.1;
			const float affordance =0;
		}error;

		float mu=0.001;
	    StateMatcher(){}

		void initOnes(){
			for (auto i=weights.begin(); i!= weights.end(); i++){
				*i=1.0;
			}
		}

		DistanceVector getDistance(State, State);  //DO NOT TRY TO LEARN DISTRIBUTION

		float sumVector(DistanceVector);

		bool isPerfectMatch(DistanceVector); // is this the same state?

		bool isPerfectMatch(State, State); // is this the same state?

		std::pair<bool, vertexDescriptor> isPerfectMatch(TransitionSystem, vertexDescriptor, Direction, State); //find match amoung vertex out edges
		
		void ICOadjustWeight(DistanceVector, DistanceVector); //simple ICO learning rule

};

#endif