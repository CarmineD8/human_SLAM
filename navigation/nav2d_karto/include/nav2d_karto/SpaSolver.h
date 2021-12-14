/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef KARTO_SPASOLVER_H
#define KARTO_SPASOLVER_H

#include "OpenKarto/OpenMapper.h"

#ifndef EIGEN_USE_NEW_STDVECTOR
#define EIGEN_USE_NEW_STDVECTOR
#endif // EIGEN_USE_NEW_STDVECTOR

#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(10)

#include <Eigen/Eigen>
#include <ros/time.h>

#include <nav2d_karto/spa2d.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>

typedef std::vector<karto::Matrix3> CovarianceVector;

class SpaSolver : public karto::ScanSolver
{
public:
	SpaSolver();
	virtual ~SpaSolver();

public:
	virtual void Clear();
	virtual void Compute();
	virtual const karto::ScanSolver::IdPoseVector& GetCorrections() const;

	virtual void AddNode(karto::Vertex<karto::LocalizedObjectPtr>* pVertex);
	virtual void AddConstraint(karto::Edge<karto::LocalizedObjectPtr>* pEdge);

	// Get the underlying graph from SBA
	// return the graph of constraints
	/// x,y -> x',y'   4 floats per connection
	void getGraph(std::vector<float> &g) { m_Spa.getGraph(g); }
	void reCompute();

private:
	karto::ScanSolver::IdPoseVector corrections;

	SysSPA2d m_Spa;
	
	ros::Time mLastSPA;

	int16_t num_closed_loops;

	ofstream file;

	// std::string filename;
};

#endif // KARTO_SPASOLVER_H

