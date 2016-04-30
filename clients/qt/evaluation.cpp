/*
 * Copyright 2016 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "evaluation.h"
#include "kdtree2.h"
#include  <math.h>
#include <cfloat>
#include<iostream>
#include<time.h>

#define PI 3.14159

using namespace std;
using namespace Eigen;

const double RobRadius = 178 / 2; //mm the radius of the soccer robots
const double BallRadius = 43; //mm radius of the ball
const double FieldLength = 9000; //mm
const double FieldWidth = 6000;
const double alpha = 0.05; // a constant proportional to the importance of distance in the query points' score

struct DefenseBot
{
    double _angle;
    BotState _botPosition;

    DefenseBot(double angle, BotState& botPosition) : _angle(angle), _botPosition(botPosition) {}

    bool operator < (const DefenseBot& bot) const
    {
        return (_angle < bot._angle);
    }
};

bool Evaluation::TeamHavingBall(BotState *robot) {
  
  if(SoccerFieldInfo::Instance()->ball[2] > 0)
    return false;
  
  double distToNearestYellowBot = DBL_MAX;
  BotState nearestYellowBot(true);
  std::vector<BotState> *team = SoccerFieldInfo::Instance()->yellowTeamBots;
  std::vector<BotState>::iterator iter = team->begin();
  for(;iter!=team->end();++iter) {
    double distToBall = (*iter).distanceToLocation(SoccerFieldInfo::Instance()->ball);
    if(distToBall < distToNearestYellowBot) {
      nearestYellowBot = *iter;
      distToNearestYellowBot = distToBall;
    }
  }
  
  double distToNearestBlueBot = DBL_MAX;
  BotState nearestBlueBot(false);
  team = SoccerFieldInfo::Instance()->blueTeamBots;
  iter = team->begin();
  for(;iter!=team->end();++iter) {
    double distToBall = (*iter).distanceToLocation(SoccerFieldInfo::Instance()->ball);
    if(distToBall < distToNearestBlueBot) {
      nearestBlueBot = *iter;
      distToNearestBlueBot = distToBall;
    }
  }
  
  if(distToNearestBlueBot > 0.0215000000 && distToNearestYellowBot > 0.0215000000) {
    return false;
  }
  if(distToNearestBlueBot < distToNearestYellowBot) {
    *robot = nearestBlueBot;
  } else {
    *robot = nearestYellowBot;
  }
  return true;
}

std::vector< KickAngles > Evaluation::EvaluateKickDirection(bool isYellowTeamKicking, Eigen::Vector2d kickFrom, Eigen::Vector2d kickToStart, Eigen::Vector2d kickToEnd)
{
  std::vector<BotState> *kickingTeam, *defendingTeam;
  if(isYellowTeamKicking) {
    kickingTeam = SoccerFieldInfo::Instance()->yellowTeamBots;
    defendingTeam = SoccerFieldInfo::Instance()->blueTeamBots;
  } else {
    kickingTeam = SoccerFieldInfo::Instance()->blueTeamBots;
    defendingTeam = SoccerFieldInfo::Instance()->yellowTeamBots;
  }
  
  /*Find chance of striker kicking between the points*/
  //Find defenders which are in front of the striker
  std::vector<DefenseBot> defenseBots;
  double vx = (kickToStart[0] + kickToEnd[0] / 2) - kickFrom[0];
  double vy = (kickToStart[1] + kickToEnd[1] / 2) - kickFrom[1];
  Eigen::Vector2d kickVector(vx / sqrt(vx*vx + vy*vy), vy / sqrt(vx*vx + vy*vy));
  
  std::vector<BotState>::iterator iter = defendingTeam->begin();
  for(; iter != defendingTeam->end(); iter++) {
    vx = (*iter)._position[0] - kickFrom[0];
    vy = (*iter)._position[1] - kickFrom[1];
    Eigen::Vector2d obstacleVector(vx / sqrt(vx*vx + vy*vy), vy / sqrt(vx*vx + vy*vy));
    double dotProduct = obstacleVector[0]*kickVector[0] + obstacleVector[1]*kickVector[1];
    if(dotProduct >= 0) {
      double angle = atan2((*iter)._position[1] - kickFrom[1], (*iter)._position[0] - kickFrom[0]);
      defenseBots.push_back(DefenseBot(angle, *iter));
    }
  }
  if(defenseBots.size() == 0)
    return std::vector<KickAngles>();
  
  //Sort the bots according to the angles with x axis
  std::sort(defenseBots.begin(), defenseBots.end());
  
  //Find angle of striker to both sides of the kick region
  Eigen::Vector3d side1 = Eigen::Vector3d(kickToStart[0], kickToStart[1], 0), side2 = Eigen::Vector3d(kickToEnd[0], kickToEnd[1], 0);
  double angle1 = atan2(kickToStart[1] - kickFrom[1], kickToStart[0] - kickFrom[0]);
  double angle2 = atan2(kickToEnd[1] - kickFrom[1], kickToEnd[0] - kickFrom[0]);
  if(angle2 < angle1) {
    double temp = angle2;
    angle2 = angle1;
    angle1 = temp;
    
    Eigen::Vector3d tempV;
    tempV = side1;
    side1 = side2;
    side2 = side1;
  }
  
  //Find bots at the two extremes
  int startIndex = -1;
  std::vector<DefenseBot>::iterator dIter = defenseBots.begin();
  for(; dIter != defenseBots.end(); dIter++, startIndex++) {
    if((*dIter)._angle > angle1)
      break;
  }
  int endIndex = defenseBots.size();
  std::vector<DefenseBot>::reverse_iterator rIter = defenseBots.rbegin();
  for(; rIter != defenseBots.rend(); rIter++, endIndex--) {
    if((*rIter)._angle < angle2)
      break;
  }
  
  std::vector<KickAngles> kickOpenings;
  //If a robot is present just before the line between bot and kick point of min angle
  if(startIndex > -1 && startIndex < defenseBots.size()) {
    kickOpenings.push_back(KickAngles(defenseBots.at(startIndex)._angle - angle1, BotState(isYellowTeamKicking, side1), defenseBots.at(startIndex)._botPosition));
    ++startIndex;
  }
  //If a robot is present just after the line between bot and kick point of max angle
  if(endIndex < defenseBots.size() && endIndex > -1) {
    kickOpenings.push_back(KickAngles(angle2 - defenseBots.at(endIndex)._angle, defenseBots.at(endIndex)._botPosition, BotState(isYellowTeamKicking, side2)));
    --endIndex;
  }
  
  if(startIndex > -1 && startIndex < defenseBots.size() && endIndex < defenseBots.size() && endIndex > -1) {
    for(int index = startIndex; index < endIndex; index++) {
      kickOpenings.push_back(KickAngles(defenseBots.at(index+1)._angle - defenseBots.at(index)._angle, defenseBots.at(index)._botPosition, defenseBots.at(index+1)._botPosition));
    }
  }
  
  return kickOpenings;
}

double Evaluation::ClosestRobotToBall(bool isTeamYellow, BotState* robot) {
  
  double distToNearestBot = DBL_MAX;
  
  std::vector<BotState> *team = SoccerFieldInfo::Instance()->yellowTeamBots;
  if(isTeamYellow == false) {
    team = SoccerFieldInfo::Instance()->blueTeamBots;
  }
  
  std::vector<BotState>::iterator iter = team->begin();
  for(;iter!=team->end();++iter) {
    double distToBall = (*iter).distanceToLocation(SoccerFieldInfo::Instance()->ball);
    if(distToBall < distToNearestBot) {
      *robot = *iter;
      distToNearestBot = distToBall;
    }
  }
  
  return distToNearestBot;
}


MatrixXd Evaluation::openAngleFinder(vector<double> shooterPosition, int shooterInd, vector<double> targetSt, vector<double> targetEn, MatrixXd robPosition_OwnTeam, MatrixXd robPosition_Opponent)
{
	Vector2d targetStart(targetSt[0], targetSt[1]);
	Vector2d targetEnd(targetEn[0], targetEn[1]);

	Vector2d shooterPos(shooterPosition[0], shooterPosition[1]);

	// Calculate the goal viewing angle from the shooter position
	//************** TO DO: Consider the ball radius when calculating the goal viewing angle
	double ang1 = atan2(targetStart(1) - shooterPos(1), targetStart(0) - shooterPos(0));
	double ang2 = atan2(targetEnd(1) - shooterPos(1), targetEnd(0) - shooterPos(0));

	// if the target is divided because of[-pi, pi] interval, convert the angles to[0, 2 * pi] interval
	int flag_2pi = 0;

	if (abs(ang1 - ang2) > PI)
	{
		flag_2pi = 1;
		if (ang1 < 0)
			ang1 = ang1 + 2 * PI;
		else
			ang2 = ang2 + 2 * PI;
	}

	Vector2d goalAng;
	goalAng << min(ang1, ang2), max(ang1, ang2);

	// Calculating the viewing angle from the query point to all the
	// opponent robots(Only opponent robots are assumed as obstacles since own team robots can be moved to free the viewing angle to the goal)
	// ******************** DO: CHANGE IT TO ALL ROBOTS RATHER THAN ONLY OPPONENT ONES*************

	// Defining a matrix containing all the opponent robots and own team robots except the one from own team possesing the ball
	MatrixXd robPosition_All(robPosition_Opponent.rows() + robPosition_OwnTeam.rows() - 1, robPosition_Opponent.cols());
	robPosition_All.block(0, 0, robPosition_Opponent.rows(), robPosition_All.cols()) = robPosition_Opponent;
	int count = 0;
	for (int i = 0; i < robPosition_OwnTeam.rows(); i++)
	{
		if (i == shooterInd)
			continue;
		else
		{
			robPosition_All.row(robPosition_Opponent.rows() + count) = robPosition_OwnTeam.row(i);
			count++;
		}

	}


	MatrixXd viewAng(robPosition_All.rows(), 2);
	for (int k = 0; k < robPosition_All.rows(); k++)
	{
		double del_x = robPosition_All(k, 0) - shooterPos(0);
		double del_y = robPosition_All(k, 1) - shooterPos(1);
		double theta_k = atan2(del_y, del_x);
		double rho_k = sqrt(del_x * del_x + del_y * del_y);
		if (flag_2pi)
		{
			if (theta_k < 0)
				theta_k = theta_k + 2 * PI;
		}

		double margin_ang = asin((RobRadius + BallRadius) / rho_k);
		viewAng.row(k) << theta_k - margin_ang, theta_k + margin_ang;
	}

	// Omitting the robots view angles which do not obstruct the goal viewing angle
	vector<int> indexTmp;
	for (int i = 0; i < viewAng.rows(); i++)
	{
		if (!((viewAng(i, 1) < goalAng(0)) || (viewAng(i, 0) > goalAng(1))))
			indexTmp.push_back(i);
	}

	MatrixXd viewAng_pruned(indexTmp.size(), 2);
	for (int i = 0; i < indexTmp.size(); i++)
	{
		viewAng_pruned.row(i) = viewAng.row(indexTmp[i]);
	}

	//cout << "The view angles to all opponents are: \n" << viewAng << endl;
	//cout << "The view angles to all obstructing opponents are: \n" << viewAng_pruned << endl;

	// Calculate a list of unobstructed angle intervals toward the goal
	MatrixXd openAng(0, 3); // [start,  end,  length]
	int m = 0; // the counter keeping the number of unobstructed intervals
	double curr_point = goalAng(0); // the current point in the interval overlapping problem

	while (curr_point <= goalAng(1))
	{
		// choosing the intevals which overlap with[curr_point, goalAng(1)]
		vector<int> indexTmp;
		for (int i = 0; i < viewAng_pruned.rows(); i++)
		{
			if (viewAng_pruned(i, 1) > curr_point)
				indexTmp.push_back(i);
		}

		MatrixXd intervalsOfInterest(indexTmp.size(), 2);
		for (int i = 0; i < indexTmp.size(); i++)
		{
			intervalsOfInterest.row(i) = viewAng_pruned.row(indexTmp[i]);
		}

		// if no other obstacle until the other post (if intervalsOfInterest is empty)
		if (indexTmp.size() == 0)
		{
			openAng.conservativeResize(openAng.rows() + 1, openAng.cols());
			openAng.row(m) << curr_point, goalAng(1), goalAng(1) - curr_point;
			m++;
			curr_point = goalAng(1) + 0.0001; //increasing curr_point value by epsilon for conditional purposes
			continue;
		}

		int indMin_st; // index of the minimum starting value between the candidate intervals
		double min_st = intervalsOfInterest.col(0).minCoeff(&indMin_st); // the minimum starting value between the candidate intervals

		if (min_st <= curr_point)
		{
			curr_point = intervalsOfInterest(indMin_st, 1) + 0.0001;
			continue;
		}
		else
		{
			openAng.conservativeResize(openAng.rows() + 1, openAng.cols());
			openAng.row(m) << curr_point, min_st, min_st - curr_point;
			m++;
			curr_point = intervalsOfInterest(indMin_st, 1) + 0.0001;
		}
	}
	vector<double> intervalLen(openAng.rows());
	vector<double> intervalLenSorted(openAng.rows());
	// sorting the open angles in descending order
	for (int i = 0; i < openAng.rows(); i++)
	{
		intervalLen[i] = openAng(i, 2);
		intervalLenSorted[i] = openAng(i, 2);
	}
	sort(intervalLenSorted.begin(), intervalLenSorted.end());

	MatrixXd openAngSorted(openAng.rows(), openAng.cols() - 1);
	vector<double>::iterator it;

	for (int i = 0; i < openAng.rows(); i++)
	{
		it = find(intervalLen.begin(), intervalLen.end(), intervalLenSorted[intervalLenSorted.size() - 1 - i]);
		openAngSorted.row(i) << openAng(it - intervalLen.begin(), 0), openAng(it - intervalLen.begin(), 1);
	}

	return(openAngSorted);
}


//Recieves a region of query and evaluates the score for points in that
//region as an index showing the chance of scoring a goal if a shot is tried
//from those positions. Returns the optimum pose in the queryRegion.
VectorXd Evaluation::shotEvaluator(double queryRegion, int Num_queryPoints, int shooterInd, vector<double> targetSt, vector<double> targetEn, MatrixXd robPosition_OwnTeam, MatrixXd robPosition_Opponent)
{
	srand(time(NULL));
	double r; // radius of the sampled point with respect to the shooter 
	double theta; // angle of the sampled point with respect to the shooter
	MatrixXd queryPoints(Num_queryPoints, 5); //{x, y, theta_start, del_theta, score}
	vector<double> shooterPos = { robPosition_OwnTeam(shooterInd, 0), robPosition_OwnTeam(shooterInd, 1) };

	for (int i = 0; i < Num_queryPoints; i++)
	{
		r = (double)rand() * queryRegion / RAND_MAX;
		theta = (double)rand() * 2 * PI / RAND_MAX;
		double delX = r * cos(theta);
		double delY = r * sin(theta);
		double x = robPosition_OwnTeam(shooterInd, 0) + delX;
		double y = robPosition_OwnTeam(shooterInd, 1) + delY;
		// do not accept sample points outside the field
		while (!((x > -FieldLength / 2) && (x < FieldLength / 2) && (y < FieldWidth / 2) && (y > -FieldWidth / 2)))
		{
			r = (double)rand() * queryRegion / RAND_MAX;
			theta = (double)rand() * 2 * PI / RAND_MAX;
			double delX = r * cos(theta);
			double delY = r * sin(theta);
			double x = robPosition_OwnTeam(shooterInd, 0) + delX;
			double y = robPosition_OwnTeam(shooterInd, 1) + delY;
		}
		// calculate the sorted list of open angles for the sampled point
		MatrixXd openAng = openAngleFinder({x, y}, shooterInd, targetSt, targetEn,  robPosition_OwnTeam,  robPosition_Opponent);
		if (openAng.size() == 0)
			queryPoints.row(i) << 0, 0, 0, 0, 0;
		else
			queryPoints.row(i) << x, y, openAng(0, 0), openAng(0, 1) - openAng(0, 0), 0; //{x, y, theta_start, del_theta, score}
	}
	//Calculate scores as the normalized value of the largest open angles minus a penalty term for distance from the current position of the shooter robot
	Vector4d optimumPoint; // {x, y, theta_start, theta_end}
	ArrayXXd dist(queryPoints.rows(),1);
	MatrixXd penaltyTerm(queryPoints.rows(),2);
	// if there is no point with an open angle to the goal
	if (queryPoints.col(3).maxCoeff() == 0)
		optimumPoint << 0, 0, 0, 0;
	else
	{
		dist = ((queryPoints.col(0).array() - shooterPos[0]).square() + (queryPoints.col(1).array() - shooterPos[1]).square());
		dist = dist.sqrt();
		penaltyTerm << queryPoints.col(3), dist.matrix() * alpha / queryRegion;
		queryPoints.col(4) = queryPoints.col(3) - penaltyTerm.rowwise().minCoeff();
		if (queryPoints.col(4).maxCoeff() == 0)
			optimumPoint << 0, 0, 0, 0;
		else
		{
			int maxIndex;
			double maxScore = queryPoints.col(4).maxCoeff(&maxIndex);
			queryPoints.col(4) = queryPoints.col(4) / maxScore;
			optimumPoint << queryPoints(maxIndex, 0), queryPoints(maxIndex, 1), queryPoints(maxIndex, 2), queryPoints(maxIndex, 2) + queryPoints(maxIndex, 3);
		}
	}

	return optimumPoint;
}

















