#include <visualization_msgs/Marker.h>
#include <nav2d_msgs/RobotPose.h>
#include <nav2d_karto/MultiMapper.h>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

using namespace std;
MultiMapper::MultiMapper()
{
	// Get parameters from the ROS parameter server
	ros::NodeHandle robotNode;
	robotNode.param("robot_id", mRobotID, 1);
	robotNode.param("scan_input_topic", mScanInputTopic, std::string("karto_in"));
	robotNode.param("scan_output_topic", mScanOutputTopic, std::string("karto_out"));
	robotNode.param("object_input_topic", mObjectInputTopic, std::string("object_in"));
	robotNode.param("object_output_topic", mObjectOutputTopic, std::string("object_out"));
	robotNode.param("laser_frame", mLaserFrame, std::string("laser"));
	robotNode.param("robot_frame", mRobotFrame, std::string("robot"));
	robotNode.param("odometry_frame", mOdometryFrame, std::string("odometry_base"));
	robotNode.param("offset_frame", mOffsetFrame, std::string("odometry_offset"));
	robotNode.param("map_frame", mMapFrame, std::string("map"));
	robotNode.param("map_service", mMapService, std::string("get_map"));
	robotNode.param("laser_topic", mLaserTopic, std::string("scan"));
	robotNode.param("map_topic", mMapTopic, std::string("map"));
	//robotNode.param("customer_order", mCustomInputTopic, std::string("Input/keyup"));
	robotNode.param("customer_order", mCustomInputTopic, std::string("index"));

	ros::NodeHandle mapperNode("~/");
	mapperNode.param("grid_resolution", mMapResolution, 0.05);
	mapperNode.param("range_threshold", mRangeThreshold, 30.0);
	mapperNode.param("map_update_rate", mMapUpdateRate, 1.0);
	mapperNode.param("publish_pose_graph", mPublishPoseGraph, true);
	mapperNode.param("max_covariance", mMaxCovariance, 0.05);
	mapperNode.param("min_map_size", mMinMapSize, 50);

	// Apply tf_prefix to all used frame-id's
	mLaserFrame = mTransformListener.resolve(mLaserFrame);
	mRobotFrame = mTransformListener.resolve(mRobotFrame);
	mOdometryFrame = mTransformListener.resolve(mOdometryFrame);
	mOffsetFrame = mTransformListener.resolve(mOffsetFrame);
	mMapFrame = mTransformListener.resolve(mMapFrame);

	// Initialize Publisher/Subscribers
	mScanSubscriber = robotNode.subscribe(mScanInputTopic, 100, &MultiMapper::receiveLocalizedScan, this);
	mScanPublisher = robotNode.advertise<nav2d_msgs::LocalizedScan>(mScanOutputTopic, 100, true);
	mObjectSubscriber = robotNode.subscribe(mObjectInputTopic, 100, &MultiMapper::receiveLocalizedObject, this);
	mObjectPublisher = robotNode.advertise<nav2d_msgs::LocalizedObject>(mObjectOutputTopic, 100, true);
	mMapServer = robotNode.advertiseService(mMapService, &MultiMapper::getMap, this);
	mMapPublisher = robotNode.advertise<nav_msgs::OccupancyGrid>(mMapTopic, 1, true);
	mLaserSubscriber = robotNode.subscribe(mLaserTopic, 100, &MultiMapper::receiveLaserScan, this);
	mInitialPoseSubscriber = robotNode.subscribe("initialpose", 1, &MultiMapper::receiveInitialPose, this);
	mOtherRobotsPublisher = robotNode.advertise<nav2d_msgs::RobotPose>("others", 10, true);

	mVerticesPublisher = mapperNode.advertise<visualization_msgs::Marker>("vertices", 1, true);
	mEdgesPublisher = mapperNode.advertise<visualization_msgs::Marker>("edges", 1, true);
	mMatchedPublisher = mapperNode.advertise<visualization_msgs::Marker>("matched_markers", 1, true);
	mPosePublisher = robotNode.advertise<geometry_msgs::PoseStamped>("localization_result", 1, true);
	mMarkersPublisher = mapperNode.advertise<visualization_msgs::Marker>("markers", 1, true);
	mMessagePublisher=mapperNode.advertise<visualization_msgs::Marker>("text_marker",1,true);
	mCustomerSubscriber = robotNode.subscribe(mCustomInputTopic, 1, &MultiMapper::receiveCustomerOrder, this);

	// Initialize KARTO-Mapper
	mMapper = new karto::OpenMapper(true);
	mMapper->start_service();
	
	double param_d;
	bool param_b;
	int param_i;

	if (mapperNode.getParam("UseScanMatching", param_b))
		mMapper->SetParameters("UseScanMatching", param_b);

	if (mapperNode.getParam("UseScanBarycenter", param_b))
		mMapper->SetParameters("UseScanBarycenter", param_b);

	if (mapperNode.getParam("MinimumTravelDistance", param_d))
		mMapper->SetParameters("MinimumTravelDistance", param_d);

	if (mapperNode.getParam("MinimumTravelHeading", param_d))
		mMapper->SetParameters("MinimumTravelHeading", param_d);

	if (mapperNode.getParam("ScanBufferSize", param_i))
		mMapper->SetParameters("ScanBufferSize", param_i);

	if (mapperNode.getParam("ScanBufferMaximumScanDistance", param_d))
		mMapper->SetParameters("ScanBufferMaximumScanDistance", param_d);

	if (mapperNode.getParam("UseResponseExpansion", param_b))
		mMapper->SetParameters("UseResponseExpansion", param_b);

	if (mapperNode.getParam("DistanceVariancePenalty", param_d))
		mMapper->SetParameters("DistanceVariancePenalty", param_d);

	if (mapperNode.getParam("MinimumDistancePenalty", param_d))
		mMapper->SetParameters("MinimumDistancePenalty", param_d);

	if (mapperNode.getParam("AngleVariancePenalty", param_d))
		mMapper->SetParameters("AngleVariancePenalty", param_d);

	if (mapperNode.getParam("MinimumAnglePenalty", param_d))
		mMapper->SetParameters("MinimumAnglePenalty", param_d);

	if (mapperNode.getParam("LinkMatchMinimumResponseFine", param_d))
		mMapper->SetParameters("LinkMatchMinimumResponseFine", param_d);

	if (mapperNode.getParam("LinkScanMaximumDistance", param_d))
		mMapper->SetParameters("LinkScanMaximumDistance", param_d);

	if (mapperNode.getParam("CorrelationSearchSpaceDimension", param_d))
		mMapper->SetParameters("CorrelationSearchSpaceDimension", param_d);

	if (mapperNode.getParam("CorrelationSearchSpaceResolution", param_d))
		mMapper->SetParameters("CorrelationSearchSpaceResolution", param_d);

	if (mapperNode.getParam("CorrelationSearchSpaceSmearDeviation", param_d))
		mMapper->SetParameters("CorrelationSearchSpaceSmearDeviation", param_d);

	if (mapperNode.getParam("CoarseSearchAngleOffset", param_d))
		mMapper->SetParameters("CoarseSearchAngleOffset", param_d);

	if (mapperNode.getParam("FineSearchAngleOffset", param_d))
		mMapper->SetParameters("FineSearchAngleOffset", param_d);

	if (mapperNode.getParam("CoarseAngleResolution", param_d))
		mMapper->SetParameters("CoarseAngleResolution", param_d);

	if (mapperNode.getParam("LoopSearchSpaceDimension", param_d))
		mMapper->SetParameters("LoopSearchSpaceDimension", param_d);

	if (mapperNode.getParam("LoopSearchSpaceResolution", param_d))
		mMapper->SetParameters("LoopSearchSpaceResolution", param_d);

	if (mapperNode.getParam("LoopSearchSpaceSmearDeviation", param_d))
		mMapper->SetParameters("LoopSearchSpaceSmearDeviation", param_d);

	if (mapperNode.getParam("LoopSearchMaximumDistance", param_d))
		mMapper->SetParameters("LoopSearchMaximumDistance", param_d);

	if (mapperNode.getParam("LoopMatchMinimumChainSize", param_i))
		mMapper->SetParameters("LoopMatchMinimumChainSize", param_i);

	if (mapperNode.getParam("LoopMatchMaximumVarianceCoarse", param_d))
		mMapper->SetParameters("LoopMatchMaximumVarianceCoarse", param_d);

	if (mapperNode.getParam("LoopMatchMinimumResponseCoarse", param_d))
		mMapper->SetParameters("LoopMatchMinimumResponseCoarse", param_d);

	if (mapperNode.getParam("LoopMatchMinimumResponseFine", param_d))
		mMapper->SetParameters("LoopMatchMinimumResponseFine", param_d);

	mMapper->Message += karto::delegate(this, &MultiMapper::onMessage);

	mLaser = NULL;
	mCustomerOrder = false;
	mFirstOrder = false;
	//    mCustomerOrderArray.push_back(0);
	//    mOrderCounter = 0;
	// Initialize Variables
	mMapToOdometry.setIdentity();
	mOdometryOffset.setIdentity();
	mNodesAdded = 0;
	mMapChanged = true;
	mLastMapUpdate = ros::WallTime(0);

	if (mRobotID == 1)
	{
		// I am the number one, so start mapping right away.
		mState = ST_MAPPING;
		ROS_INFO("Inititialized robot 1, starting to map now.");
		mSelfLocalizer = NULL;

		geometry_msgs::PoseStamped locResult;
		locResult.header.stamp = ros::Time::now();
		locResult.header.frame_id = mMapFrame.c_str();
		locResult.pose.position.x = 0;
		locResult.pose.position.y = 0;
		locResult.pose.position.z = 0;
		locResult.pose.orientation = tf::createQuaternionMsgFromYaw(0);
		mPosePublisher.publish(locResult);
	}
	else
	{
		// I am not number one, so wait to receive a map from number one.
		mState = ST_WAITING_FOR_MAP;
		ROS_INFO("Initialized robot %d, waiting for map from robot 1 now.", mRobotID);
		mSelfLocalizer = new SelfLocalizer();
	}
}

MultiMapper::~MultiMapper()
{
}

void MultiMapper::setScanSolver(karto::ScanSolver *scanSolver)
{
	mMapper->SetScanSolver(scanSolver);
}

void MultiMapper::setRobotPose(double x, double y, double yaw)
{
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x, y, 0));
	transform.setRotation(tf::createQuaternionFromYaw(yaw));
	transform = transform.inverse();

	tf::Stamped<tf::Pose> pose_in, pose_out;
	pose_in.setData(transform);
	pose_in.frame_id_ = mRobotFrame;
	pose_in.stamp_ = ros::Time(0);
	mTransformListener.transformPose(mOdometryFrame, pose_in, pose_out);

	transform = pose_out;
	mOdometryOffset = transform.inverse();

	if (mSelfLocalizer)
	{
		delete mSelfLocalizer;
		mSelfLocalizer = NULL;
	}

	// Publish the new pose (to inform other nodes, that we are localized now)
	geometry_msgs::PoseStamped locResult;
	locResult.header.stamp = ros::Time::now();
	locResult.header.frame_id = mMapFrame.c_str();
	locResult.pose.position.x = x;
	locResult.pose.position.y = y;
	locResult.pose.position.z = 0;
	locResult.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
	mPosePublisher.publish(locResult);

	// Publish via tf
	mState = ST_MAPPING;
	publishTransform();
}

karto::LocalizedRangeScan *MultiMapper::createFromRosMessage(const sensor_msgs::LaserScan &scan, const karto::Identifier &robot)
{
	// Implementing REP 117: Informational Distance Measurements
	// http://www.ros.org/reps/rep-0117.html
	karto::RangeReadingsList readings;
	std::vector<float>::const_iterator it;
	for (it = scan.ranges.begin(); it != scan.ranges.end(); it++)
	{
		if (*it >= scan.range_min && *it <= scan.range_max)
		{
			// This is a valid measurement.
			readings.Add(*it);
		}
		else if (!std::isfinite(*it) && *it < 0)
		{
			// Object too close to measure.
			readings.Add(scan.range_max);
		}
		else if (!std::isfinite(*it) && *it > 0)
		{
			// No objects detected in range.
			readings.Add(scan.range_max);
		}
		else if (std::isnan(*it))
		{
			// This is an erroneous, invalid, or missing measurement.
			ROS_WARN_THROTTLE(1, "Laser scan contains nan-values!");
			readings.Add(scan.range_max);
		}
		else
		{
			// The sensor reported these measurements as valid, but they are
			// discarded per the limits defined by minimum_range and maximum_range.
			ROS_WARN_THROTTLE(1, "Laser reading not between range_min and range_max!");
			readings.Add(scan.range_max);
		}
	}
	return new karto::LocalizedRangeScan(robot, readings);
}

karto::LocalizedObject *MultiMapper::createObject(const karto::Identifier &robot)
{
	return new karto::LocalizedObject(robot);
}

void MultiMapper::receiveCustomerOrder(const std_msgs::Int16 index) /// works
{
	

	// ///Here the subscribe function already decided that if it's the first order or not
	mCustomerOrder = true;
	mFirstOrder = true;
	OrderNum = index.data;

	// for (auto i : mCustomerProbArray)
	// {
	// 	if (i == OrderNum)
	// 	{
	// 		mFirstOrder = false;
	// 	}
	// }
	

	// std::vector<kt_float>::iterator it;
	// it = mCustomerProbArray.begin();
	// it = mCustomerProbArray.insert(it, OrderNum);

	mCustomerProbArray.push_back(OrderNum);
	if (mCustomerProbArray.size()>1)
	{
		mFirstOrder = false;
	}

}

void MultiMapper::receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	// Ignore own readings until map has been received
	if (mState == ST_WAITING_FOR_MAP)
	{
		return;
	}

	if (!mLaser)
	{
		// Create a laser range finder device and copy in data from the first scan
		char name[10];
		sprintf(name, "robot_%d", mRobotID);

		// Add the laser to the mapper
		try
		{
			mLaser = karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, name);
			mLaser->SetMinimumRange(scan->range_min);
			mLaser->SetMaximumRange(scan->range_max);
			mLaser->SetMinimumAngle(scan->angle_min);
			mLaser->SetMaximumAngle(scan->angle_max);
			mLaser->SetAngularResolution(scan->angle_increment);
			mLaser->SetRangeThreshold(mRangeThreshold);
			mMapper->Process(mLaser);
		}
		catch (karto::Exception e)
		{
			ROS_ERROR("Could not add new Laser to Mapper: %s", e.GetErrorMessage().ToCString());
			return;
		}
	}

	if (mState == ST_LOCALIZING)
	{
		mSelfLocalizer->process(scan);
		if (mSelfLocalizer->getCovariance() < mMaxCovariance)
		{
			// Localization finished, kill the localizer and start mapping
			ROS_INFO("Localization finished on robot %d, now starting to map.", mRobotID);
			tf::Transform p = mSelfLocalizer->getBestPose();
			setRobotPose(p.getOrigin().getX(), p.getOrigin().getY(), tf::getYaw(p.getRotation()));
		}
	}
	else

		if (mState == ST_MAPPING)
	{
		// get the odometry pose from tf
		tf::StampedTransform tfPose;
		try
		{
			mTransformListener.lookupTransform(mOffsetFrame, mLaserFrame, scan->header.stamp, tfPose);
		}
		catch (tf::TransformException e)
		{
			try
			{
				mTransformListener.lookupTransform(mOffsetFrame, mLaserFrame, ros::Time(0), tfPose);
			}
			catch (tf::TransformException e)
			{
				ROS_WARN("Failed to compute odometry pose, skipping scan (%s)", e.what());
				return;
			}
		}
		karto::Pose2 kartoPose = karto::Pose2(tfPose.getOrigin().x(), tfPose.getOrigin().y(), tf::getYaw(tfPose.getRotation()));

		// create localized laser scan/object
		if (!mCustomerOrder)
		{
			karto::LocalizedLaserScanPtr laserScan = createFromRosMessage(*scan, mLaser->GetIdentifier());
			laserScan->SetOdometricPose(kartoPose);
			laserScan->SetCorrectedPose(kartoPose);

			bool success;
			try
			{
				success = mMapper->Process(laserScan);
			}
			catch (karto::Exception e)
			{
				ROS_ERROR("%s", e.GetErrorMessage().ToCString());
				success = false;
			}

			if (success)
			{
				// Compute the map->odom transform
				karto::Pose2 corrected_pose = laserScan->GetCorrectedPose();
				tf::Pose map_in_robot(tf::createQuaternionFromYaw(corrected_pose.GetHeading()),
									  tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0));
				map_in_robot = map_in_robot.inverse();
				tf::Stamped<tf::Pose> map_in_odom;
				bool ok = true;
				try
				{
					mTransformListener.transformPose(mOffsetFrame, tf::Stamped<tf::Pose>(map_in_robot, ros::Time(0) /*scan->header.stamp*/, mLaserFrame), map_in_odom);
				}
				catch (tf::TransformException e)
				{
					ROS_WARN("Transform from %s to %s failed! (%s)", mLaserFrame.c_str(), mOffsetFrame.c_str(),
							 e.what());
					ok = false;
				}
				if (ok)
				{
					mMapToOdometry = tf::Transform(tf::Quaternion(map_in_odom.getRotation()),
												   tf::Point(map_in_odom.getOrigin()))
										 .inverse();
					tf::Vector3 v = mMapToOdometry.getOrigin();
					v.setZ(0);
					mMapToOdometry.setOrigin(v);
				}
				mNodesAdded++;
				mMapChanged = true;

				std::cout << "number of nodes: " << mNodesAdded << std::endl;
				std::cout << "Pose after correction:" << std::endl;
				std::cout << "x: " << laserScan->GetCorrectedPose().GetX() << std::endl;
				std::cout << "y: " << laserScan->GetCorrectedPose().GetY() << std::endl;
				std::cout << "heading: " << laserScan->GetCorrectedPose().GetHeading() << std::endl;

				cout << "Order vector [";

				for (const auto &e : mCustomerProbArray)
				{
					std::cout << e << ", ";
				}
				cout << "]" << endl;

				ros::WallDuration d = ros::WallTime::now() - mLastMapUpdate;
				if (mMapUpdateRate > 0 && d.toSec() > mMapUpdateRate)
				{
					
					sendMap();
				}

				// Send the scan to the other robots via com-layer (DDS)
				ROS_DEBUG("Robot %d: Sending scan (uniqueID: %d, Sensor: %s, stateID: %d)", mRobotID,
						  laserScan->GetUniqueId(), laserScan->GetSensorIdentifier().ToString().ToCString(),
						  laserScan->GetStateId());
				sendLocalizedScan(scan, laserScan->GetOdometricPose());

				// Publish via extra topic
				nav2d_msgs::RobotPose other;
				other.header.stamp = ros::Time::now();
				other.header.frame_id = mMapFrame;
				other.robot_id = mRobotID;
				other.pose.x = laserScan->GetCorrectedPose().GetX();
				other.pose.y = laserScan->GetCorrectedPose().GetY();
				other.pose.theta = laserScan->GetCorrectedPose().GetHeading();
				mOtherRobotsPublisher.publish(other);
			}
		}
		else if (mCustomerOrder && mFirstOrder)
		{
			mCustomerOrder = false;

			karto::LocalizedLaserScanPtr laserScan = createFromRosMessage(*scan, mLaser->GetIdentifier());
			karto::CustomItemPtr ProbListItem = new karto::CustomItem(laserScan->GetIdentifier());
			karto::List<kt_float> ProbList;

			for (size_t ArraySize = 0; ArraySize < mCustomerProbArray.size(); ArraySize++)
			{
				ProbList.Add(mCustomerProbArray[ArraySize]);
			}
			ProbListItem->Write(ProbList);

			std::cout << "This object does not have match, put a normal scan here." << std::endl;
			std::cout << "The number of received orders: " << mCustomerProbArray.size() << std::endl;
			cout << "PROB LIST is " << ProbListItem << endl;
			laserScan->SetOdometricPose(kartoPose);
			laserScan->SetCorrectedPose(kartoPose);
			laserScan->AddCustomItem(ProbListItem);
			laserScan->SetCustomMessage("fun stuff");


			bool success;
			try
			{
				success = mMapper->Process(laserScan);
			}
			catch (karto::Exception e)
			{
				ROS_ERROR("%s", e.GetErrorMessage().ToCString());
				success = false;
			}

			if (success)
			{
				// Compute the map->odom transform
				karto::Pose2 corrected_pose = laserScan->GetCorrectedPose();
				tf::Pose map_in_robot(tf::createQuaternionFromYaw(corrected_pose.GetHeading()),
									  tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0));
				map_in_robot = map_in_robot.inverse();
				tf::Stamped<tf::Pose> map_in_odom;
				bool ok = true;
				try
				{
					mTransformListener.transformPose(mOffsetFrame, tf::Stamped<tf::Pose>(map_in_robot, ros::Time(0) /*scan->header.stamp*/, mLaserFrame), map_in_odom);
				}
				catch (tf::TransformException e)
				{
					ROS_WARN("Transform from %s to %s failed! (%s)", mLaserFrame.c_str(), mOffsetFrame.c_str(),
							 e.what());
					ok = false;
				}
				if (ok)
				{
					mMapToOdometry = tf::Transform(tf::Quaternion(map_in_odom.getRotation()),
												   tf::Point(map_in_odom.getOrigin()))
										 .inverse();
					tf::Vector3 v = mMapToOdometry.getOrigin();
					v.setZ(0);
					mMapToOdometry.setOrigin(v);
				}
				mNodesAdded++;
				mMapChanged = true;
				cout << "In case it dies here" << endl;
				karto::MapperGraph::VertexList MarkedNodes = mMapper->GetGraph()->GetVertices();


				karto::LocalizedObjectPtr markedObject = MarkedNodes[MarkedNodes.Size() - 1]->GetVertexObject();
				markedList.push_back(markedObject);


				std::cout << "number of nodes: " << mNodesAdded << std::endl;
				std::cout << "Pose after correction:" << std::endl;
				std::cout << "x: " << laserScan->GetCorrectedPose().GetX() << std::endl;
				std::cout << "y: " << laserScan->GetCorrectedPose().GetY() << std::endl;
				std::cout << "heading: " << laserScan->GetCorrectedPose().GetHeading() << std::endl;

				cout << "Order vector [";

				for (const auto &e : mCustomerProbArray)
				{
					std::cout << e << ", ";
				}
				cout << "]" << endl;

				ros::WallDuration d = ros::WallTime::now() - mLastMapUpdate;
				if (mMapUpdateRate > 0 && d.toSec() > mMapUpdateRate)
				{
					sendMap();
				}

				// Send the scan to the other robots via com-layer (DDS)
				ROS_DEBUG("Robot %d: Sending scan (uniqueID: %d, Sensor: %s, stateID: %d)", mRobotID,
						  laserScan->GetUniqueId(), laserScan->GetSensorIdentifier().ToString().ToCString(),
						  laserScan->GetStateId());
				sendLocalizedScan(scan, laserScan->GetOdometricPose(), mCustomerProbArray);

				// Publish via extra topic
				nav2d_msgs::RobotPose other;
				other.header.stamp = ros::Time::now();
				other.header.frame_id = mMapFrame;
				other.robot_id = mRobotID;
				other.pose.x = laserScan->GetCorrectedPose().GetX();
				other.pose.y = laserScan->GetCorrectedPose().GetY();
				other.pose.theta = laserScan->GetCorrectedPose().GetHeading();
				mOtherRobotsPublisher.publish(other);
			}
		}
		else
		{
			std::cout << "This obejct may have a match, the number of received orders: " << mCustomerProbArray.size() << std::endl;
			mCustomerOrder = false;
			karto::LocalizedObjectPtr imageObject = createObject(mLaser->GetIdentifier());
			imageObject->SetSensorIdentifier(mLaser->GetIdentifier());
			imageObject->SetOdometricPose(kartoPose);
			imageObject->SetCorrectedPose(kartoPose);
			imageObject->SetCustomMessage("fun stuff");

			karto::CustomItemPtr ProbListItem = new karto::CustomItem(imageObject->GetIdentifier());
			karto::List<kt_float> ProbList;
			for (size_t ArraySize = 0; ArraySize < mCustomerProbArray.size(); ArraySize++)
			{
				ProbList.Add(mCustomerProbArray[ArraySize]);
			}

			ProbListItem->Write(ProbList);
			imageObject->AddCustomItem(ProbListItem);

			if (!imageObject->HasCustomItem())
				return;

			bool success;
			try
			{
				success = mMapper->Process(imageObject);
			}
			catch (karto::Exception e)
			{
				ROS_ERROR("%s", e.GetErrorMessage().ToCString());
				success = false;
			}
			if (success)
			{
				karto::Pose2 corrected_pose = imageObject->GetCorrectedPose();
				tf::Pose map_in_robot(tf::createQuaternionFromYaw(corrected_pose.GetHeading()),
									  tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0));
				map_in_robot = map_in_robot.inverse();
				tf::Stamped<tf::Pose> map_in_odom;
				bool ok = true;
				try
				{
					mTransformListener.transformPose(mOffsetFrame, tf::Stamped<tf::Pose>(map_in_robot, ros::Time(0) /*scan->header.stamp*/, mLaserFrame), map_in_odom);
				}
				catch (tf::TransformException e)
				{
					ROS_WARN("Transform from %s to %s failed! (%s)", mLaserFrame.c_str(), mOffsetFrame.c_str(),
							 e.what());
					ok = false;
				}
				if (ok)
				{
					mMapToOdometry = tf::Transform(tf::Quaternion(map_in_odom.getRotation()),
												   tf::Point(map_in_odom.getOrigin()))
										 .inverse();
					tf::Vector3 v = mMapToOdometry.getOrigin();
					v.setZ(0);
					mMapToOdometry.setOrigin(v);
				}
				mNodesAdded++;
				mMapChanged = true;
				std::cout << "number of node: " << mNodesAdded << ", It's a special node" << std::endl;
				std::cout << "Pose after correction:" << std::endl;
				std::cout << "x: " << imageObject->GetCorrectedPose().GetX() << std::endl;
				std::cout << "y: " << imageObject->GetCorrectedPose().GetY() << std::endl;
				std::cout << "heading: " << imageObject->GetCorrectedPose().GetHeading() << std::endl;

				cout << "Order vector [";

				for (const auto &e : mCustomerProbArray)
				{
					std::cout << e << ", ";
				}
				cout << "]" << endl;

				karto::MapperGraph::VertexList matchedNodes = mMapper->GetGraph()->GetVertices();

				karto::LocalizedObjectPtr matchedObject = matchedNodes[matchedNodes.Size() - 1]->GetVertexObject();
				matchedList.push_back(matchedObject);

				ros::WallDuration d = ros::WallTime::now() - mLastMapUpdate;
				if (mMapUpdateRate > 0 && d.toSec() > mMapUpdateRate)
				{
					sendMap();
				}
				sendLocalizedObject(imageObject);
				// Publish via extra topic
				nav2d_msgs::RobotPose other;
				other.header.stamp = ros::Time::now();
				other.header.frame_id = mMapFrame;
				other.robot_id = mRobotID;
				other.pose.x = imageObject->GetCorrectedPose().GetX();
				other.pose.y = imageObject->GetCorrectedPose().GetY();
				other.pose.theta = imageObject->GetCorrectedPose().GetHeading();
				mOtherRobotsPublisher.publish(other);
			}
		}
	}
}

bool MultiMapper::getMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
	if (mState == ST_WAITING_FOR_MAP && mNodesAdded < mMinMapSize)
	{
		ROS_INFO("Still waiting for map from robot 1.");
		return false;
	}

	if (sendMap())
	{
		res.map = mGridMap;
		return true;
	}
	else
	{
		ROS_WARN("Serving map request failed!");
		return false;
	}
}

bool MultiMapper::sendMap()
{
	if (!updateMap())
		return false;

	// Publish the map
	mMapPublisher.publish(mGridMap);
	mLastMapUpdate = ros::WallTime::now();

	// Publish the pose-graph
	if (mPublishPoseGraph)
	{
		// Publish the vertices
		karto::MapperGraph::VertexList vertices = mMapper->GetGraph()->GetVertices();
		visualization_msgs::Marker marker;
		marker.header.frame_id = mMapFrame;
		marker.header.stamp = ros::Time();
		marker.id = 0;
		marker.type = visualization_msgs::Marker::SPHERE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		

		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;

		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		marker.points.resize(vertices.Size());

		for (int i = 0; i < vertices.Size(); i++)
		{
			marker.points[i].x = vertices[i]->GetVertexObject()->GetCorrectedPose().GetX();
			marker.points[i].y = vertices[i]->GetVertexObject()->GetCorrectedPose().GetY();
			marker.points[i].z = 0;
		}
		
		mVerticesPublisher.publish(marker);

		// Publish the edges
		karto::MapperGraph::EdgeList edges = mMapper->GetGraph()->GetEdges();
		marker.header.frame_id = mMapFrame;
		marker.header.stamp = ros::Time();
		marker.id = 0;
		marker.type = visualization_msgs::Marker::LINE_LIST;
		marker.scale.x = 0.01;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		marker.points.resize(edges.Size() * 2);

		for (int i = 0; i < edges.Size(); i++)
		{
			marker.points[2 * i].x = edges[i]->GetSource()->GetVertexObject()->GetCorrectedPose().GetX();
			marker.points[2 * i].y = edges[i]->GetSource()->GetVertexObject()->GetCorrectedPose().GetY();
			marker.points[2 * i].z = 0;

			marker.points[2 * i + 1].x = edges[i]->GetTarget()->GetVertexObject()->GetCorrectedPose().GetX();
			marker.points[2 * i + 1].y = edges[i]->GetTarget()->GetVertexObject()->GetCorrectedPose().GetY();
			marker.points[2 * i + 1].z = 0;
		}
		mEdgesPublisher.publish(marker);

		// Publish special nodes

		visualization_msgs::Marker sp_nodes;
		sp_nodes.header.frame_id = mMapFrame;
		sp_nodes.header.stamp = ros::Time();
		sp_nodes.id = 0;
		sp_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
		sp_nodes.action = visualization_msgs::Marker::ADD;

		sp_nodes.pose.position.x = 0;
		sp_nodes.pose.position.y = 0;
		sp_nodes.pose.position.z = 0;
		sp_nodes.pose.orientation.x = 0.0;
		sp_nodes.pose.orientation.y = 0.0;
		sp_nodes.pose.orientation.z = 0.0;
		sp_nodes.pose.orientation.w = 1.0;

		sp_nodes.scale.x = 1;
		sp_nodes.scale.y = 1;
		sp_nodes.scale.z = 0.1;
		sp_nodes.color.a = 1.0;
		sp_nodes.color.r = 1.0;
		sp_nodes.color.g = 0.0;
		sp_nodes.color.b = 0.0;
		sp_nodes.points.resize(markedList.size());
		
		for (int i = 0; i < markedList.size(); i++)
		{
			sp_nodes.points[i].x = markedList[i]->GetCorrectedPose().GetX();
			sp_nodes.points[i].y = markedList[i]->GetCorrectedPose().GetY();
			sp_nodes.points[i].z = 0;
		}

		mMarkersPublisher.publish(sp_nodes);

		// Publish matched nodes

		visualization_msgs::Marker m_nodes;
		m_nodes.header.frame_id = mMapFrame;
		m_nodes.header.stamp = ros::Time();
		m_nodes.id = 0;
		m_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
		m_nodes.action = visualization_msgs::Marker::ADD;

		//m_nodes.pose.position.x = 0;
		//m_nodes.pose.position.y = 0;
		//m_nodes.pose.position.z = 0;
		m_nodes.pose.orientation.x = 0.0;
		m_nodes.pose.orientation.y = 0.0;
		m_nodes.pose.orientation.z = 0.0;
		m_nodes.pose.orientation.w = 1.0;

		m_nodes.scale.x = 1;
		m_nodes.scale.y = 1;
		m_nodes.scale.z = 0.1;
		m_nodes.color.a = 1.0;
		m_nodes.color.r = 0.0;
		m_nodes.color.g = 1.0;
		m_nodes.color.b = 0.0;
		m_nodes.points.resize(matchedList.size());

		for (int i = 0; i < matchedList.size(); i++)
		{
			m_nodes.points[i].x = matchedList[i]->GetCorrectedPose().GetX();
			m_nodes.points[i].y = matchedList[i]->GetCorrectedPose().GetY();
			m_nodes.points[i].z = 0;
		}

		mMatchedPublisher.publish(m_nodes);

		// Publish text tags
		visualization_msgs::Marker txt_nodes;
		txt_nodes.header.frame_id = mMapFrame;
		txt_nodes.header.stamp = ros::Time();
		txt_nodes.id = 0;
		txt_nodes.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		txt_nodes.action = visualization_msgs::Marker::ADD;

		txt_nodes.pose.position.x = 0;
		txt_nodes.pose.position.y = 0;
		txt_nodes.pose.position.z = 0;

		txt_nodes.scale.x = 1;
		txt_nodes.scale.y = 1;
		txt_nodes.scale.z = 0.1;
		txt_nodes.color.a = 1.0;
		txt_nodes.color.r = 1.0;
		txt_nodes.color.g = 1.0;
		txt_nodes.color.b = 1.0;
		txt_nodes.points.resize(markedList.size());
		txt_nodes.text="yayayayyayaayayaya";
		for (int i = 0; i < markedList.size(); i++)
		{
			txt_nodes.points[i].x = markedList[i]->GetCorrectedPose().GetX();
			txt_nodes.points[i].y = markedList[i]->GetCorrectedPose().GetY();
			txt_nodes.points[i].z = 0;
	
			
		}

		mMessagePublisher.publish(txt_nodes);

	}
	return true;
}

bool MultiMapper::updateMap()
{
	if (!mMapChanged)
		return true;

	const karto::LocalizedLaserScanList allScans = mMapper->GetAllProcessedScans();
	karto::OccupancyGridPtr kartoGrid = karto::OccupancyGrid::CreateFromScans(allScans, mMapResolution);

	if (!kartoGrid)
	{
		ROS_WARN("Failed to get occupancy map from Karto-Mapper.");
		return false;
	}

	// Translate to ROS format
	unsigned int width = kartoGrid->GetWidth();
	unsigned int height = kartoGrid->GetHeight();
	karto::Vector2<kt_double> offset = kartoGrid->GetCoordinateConverter()->GetOffset();

	if (mGridMap.info.width != width ||
		mGridMap.info.height != height ||
		mGridMap.info.origin.position.x != offset.GetX() ||
		mGridMap.info.origin.position.y != offset.GetY())
	{
		mGridMap.info.resolution = mMapResolution;
		mGridMap.info.origin.position.x = offset.GetX();
		mGridMap.info.origin.position.y = offset.GetY();
		mGridMap.info.width = width;
		mGridMap.info.height = height;
		mGridMap.data.resize(mGridMap.info.width * mGridMap.info.height);
	}

	for (unsigned int y = 0; y < height; y++)
	{
		for (unsigned int x = 0; x < width; x++)
		{
			// Getting the value at position x,y
			kt_int8u value = kartoGrid->GetValue(karto::Vector2<kt_int32s>(x, y));

			switch (value)
			{
			case karto::GridStates_Unknown:
				mGridMap.data[MAP_IDX(mGridMap.info.width, x, y)] = -1;
				break;
			case karto::GridStates_Occupied:
				mGridMap.data[MAP_IDX(mGridMap.info.width, x, y)] = 100;
				break;
			case karto::GridStates_Free:
				mGridMap.data[MAP_IDX(mGridMap.info.width, x, y)] = 0;
				break;
			default:
				ROS_WARN("Encountered unknown cell value at %d, %d", x, y);
				break;
			}
		}
	}

	// Set the header information on the map
	mGridMap.header.stamp = ros::Time::now();
	mGridMap.header.frame_id = mMapFrame.c_str();
	mMapChanged = false;
	return true;
}

void MultiMapper::receiveLocalizedScan(const nav2d_msgs::LocalizedScan::ConstPtr &scan)
{
	// Ignore my own scans
	if (scan->robot_id == mRobotID)
		return;

	// Get the robot id
	char robot[10];
	sprintf(robot, "robot_%d", scan->robot_id);
	// Get the scan pose
	karto::Pose2 scanPose(scan->x, scan->y, scan->yaw);

	// create localized laser scan
	karto::LocalizedLaserScanPtr localizedScan = createFromRosMessage(scan->scan, robot);
	localizedScan->SetOdometricPose(scanPose);
	localizedScan->SetCorrectedPose(scanPose);

	if (scan->prob_vec.size() > 0)
	{
		karto::CustomItemPtr ProbListItem = new karto::CustomItem(localizedScan->GetIdentifier());
		karto::List<kt_float> ProbList;
		for (size_t i = 0; i < scan->prob_vec.size(); i++)
			ProbList.Add(scan->prob_vec[i]);
		ProbListItem->Write(ProbList);
		localizedScan->AddCustomItem(ProbListItem);
		std::cout << "Receives a special scan from " << robot << std::endl;
	}
	// feed the localized scan to the Karto-Mapper
	bool added = false;
	try
	{
		added = mMapper->Process(localizedScan);
	}
	catch (karto::Exception e1)
	{
		if (mOtherLasers.find(scan->robot_id) == mOtherLasers.end())
		{
			try
			{
				karto::LaserRangeFinderPtr laser = karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, robot);
				laser->SetMinimumRange(scan->scan.range_min);
				laser->SetMaximumRange(scan->scan.range_max);
				laser->SetMinimumAngle(scan->scan.angle_min);
				laser->SetMaximumAngle(scan->scan.angle_max);
				laser->SetAngularResolution(scan->scan.angle_increment);
				laser->SetRangeThreshold(mRangeThreshold);
				mMapper->Process(laser);
				mOtherLasers.insert(std::pair<int, karto::LaserRangeFinderPtr>(scan->robot_id, laser));

				added = mMapper->Process(localizedScan);
			}
			catch (karto::Exception e2)
			{
				ROS_ERROR("%s", e2.GetErrorMessage().ToCString());
			}
		}
		else
		{
			ROS_ERROR("%s", e1.GetErrorMessage().ToCString());
		}
	}
	if (added)
	{
		mNodesAdded++;
		std::cout << "number of nodes: " << mNodesAdded << std::endl;
		std::cout << "Pose after correction:" << std::endl;
		std::cout << "x: " << localizedScan->GetCorrectedPose().GetX() << std::endl;
		std::cout << "y: " << localizedScan->GetCorrectedPose().GetY() << std::endl;
		std::cout << "heading: " << localizedScan->GetCorrectedPose().GetHeading() << std::endl;
		mMapChanged = true;
		ROS_DEBUG("Robot %d: Received scan (uniqueID: %d, Sensor: %s, stateID: %d)", mRobotID, localizedScan->GetUniqueId(), localizedScan->GetSensorIdentifier().ToString().ToCString(), localizedScan->GetStateId());

		// Publish via extra topic
		nav2d_msgs::RobotPose other;
		other.header.stamp = ros::Time::now();
		other.header.frame_id = mMapFrame;
		other.robot_id = scan->robot_id;
		other.pose.x = localizedScan->GetCorrectedPose().GetX();
		other.pose.y = localizedScan->GetCorrectedPose().GetY();
		other.pose.theta = localizedScan->GetCorrectedPose().GetHeading();
		mOtherRobotsPublisher.publish(other);

		// Send the map via topic
		ros::WallDuration d = ros::WallTime::now() - mLastMapUpdate;
		if (mMapUpdateRate > 0 && d.toSec() > mMapUpdateRate)
		{
			sendMap();
			if (mState == ST_LOCALIZING)
			{
				mSelfLocalizer->convertMap(mGridMap);
			}
		}
	}
	else
	{
		ROS_DEBUG("Discarded Scan from Robot %d!", scan->robot_id);
	}

	if (mState == ST_WAITING_FOR_MAP && mNodesAdded >= mMinMapSize)
	{
		sendMap();
		mSelfLocalizer->convertMap(mGridMap);
		mSelfLocalizer->initialize();
		mState = ST_LOCALIZING;
		ROS_INFO("Received a map, now starting to localize.");
		mSelfLocalizer->publishParticleCloud();
	}
}

void MultiMapper::sendLocalizedScan(const sensor_msgs::LaserScan::ConstPtr &scan, const karto::Pose2 &pose)
{
	nav2d_msgs::LocalizedScan rosScan;
	rosScan.robot_id = mRobotID;
	rosScan.laser_type = 0;
	rosScan.x = pose.GetX();
	rosScan.y = pose.GetY();
	rosScan.yaw = pose.GetHeading();
	rosScan.scan.angle_min = scan->angle_min;
	rosScan.scan.angle_max = scan->angle_max;
	rosScan.scan.range_min = scan->range_min;
	rosScan.scan.range_max = scan->range_max;
	rosScan.scan.angle_increment = scan->angle_increment;
	rosScan.scan.time_increment = scan->time_increment;
	rosScan.scan.scan_time = scan->scan_time;

	unsigned int nReadings = scan->ranges.size();
	rosScan.scan.ranges.resize(nReadings);
	for (unsigned int i = 0; i < nReadings; i++)
	{
		rosScan.scan.ranges[i] = scan->ranges[i];
	}

	//	rosScan.scan = *scan;
	mScanPublisher.publish(rosScan);
}

void MultiMapper::sendLocalizedScan(const sensor_msgs::LaserScan::ConstPtr &scan, const karto::Pose2 &pose, const std::vector<kt_float> ProbVec)
{
	nav2d_msgs::LocalizedScan rosScan;
	rosScan.robot_id = mRobotID;
	rosScan.laser_type = 0;
	rosScan.x = pose.GetX();
	rosScan.y = pose.GetY();
	rosScan.yaw = pose.GetHeading();
	rosScan.prob_vec = ProbVec;
	rosScan.scan.angle_min = scan->angle_min;
	rosScan.scan.angle_max = scan->angle_max;
	rosScan.scan.range_min = scan->range_min;
	rosScan.scan.range_max = scan->range_max;
	rosScan.scan.angle_increment = scan->angle_increment;
	rosScan.scan.time_increment = scan->time_increment;
	rosScan.scan.scan_time = scan->scan_time;

	unsigned int nReadings = scan->ranges.size();
	rosScan.scan.ranges.resize(nReadings);
	for (unsigned int i = 0; i < nReadings; i++)
	{
		rosScan.scan.ranges[i] = scan->ranges[i];
	}

	//	rosScan.scan = *scan;
	mScanPublisher.publish(rosScan);
	std::cout << "Sent this special scan." << std::endl;
}

void MultiMapper::sendLocalizedObject(const karto::LocalizedObjectPtr object)
{
	nav2d_msgs::LocalizedObject pObject;
	karto::CustomItemList imageItemList;
	imageItemList = object->GetCustomItems();
	karto::List<kt_float> ProbList = imageItemList[0]->Read();
	;
	std::vector<float> ProbVec;
	karto_forEach(karto::List<kt_float>, &ProbList)
		ProbVec.push_back(*iter);

	if (imageItemList.IsEmpty())
		return;
	kt_size_t ListSize = imageItemList.Size();
	if (ListSize > 1)
		return;

	karto::String name = object->GetSensorIdentifier().GetName();
	kt_size_t IDdex = name.Size();
	pObject.robot_id = name[IDdex - 1] - 48;
	pObject.prob_vec = ProbVec;
	pObject.x = object->GetOdometricPose().GetX();
	pObject.y = object->GetOdometricPose().GetY();
	pObject.yaw = object->GetOdometricPose().GetHeading();

	mObjectPublisher.publish(pObject);
}

void MultiMapper::receiveLocalizedObject(const nav2d_msgs::LocalizedObject::ConstPtr &object)
{
	if (object->robot_id == mRobotID)
		return;

	char robot[10];
	sprintf(robot, "robot_%d", object->robot_id);
	karto::Pose2 ObjectPose(object->x, object->y, object->yaw);
	karto::CustomItemPtr ProbListItem = new karto::CustomItem(robot);
	std::vector<float> ProbVec = object->prob_vec;
	karto::List<kt_float> ProbList;

	for (size_t i = 0; i < ProbVec.size(); i++)
		ProbList.Add(ProbVec[i]);

	ProbListItem->Write(ProbList);

	std::cout << "Receive an object from " << robot << std::endl;
	karto::LocalizedObjectPtr LocalizedObject = createObject(robot);
	LocalizedObject->SetOdometricPose(ObjectPose);
	LocalizedObject->SetCorrectedPose(ObjectPose);
	LocalizedObject->AddCustomItem(ProbListItem);

	bool added = false;
	try
	{
		added = mMapper->Process(LocalizedObject);
	}
	catch (karto::Exception e)
	{
		ROS_ERROR("%s", e.GetErrorMessage().ToCString());
	}
	if (added)
	{
		mNodesAdded++;
		mMapChanged = true;
		std::cout << "Add a object node from " << robot << std::endl;
		std::cout << "number of nodes: " << mNodesAdded++ << std::endl;
		std::cout << "Pose after correction:" << std::endl;
		std::cout << "x: " << LocalizedObject->GetCorrectedPose().GetX() << std::endl;
		std::cout << "y: " << LocalizedObject->GetCorrectedPose().GetY() << std::endl;
		std::cout << "heading: " << LocalizedObject->GetCorrectedPose().GetHeading() << std::endl;
		// Publish via extra topic
		nav2d_msgs::RobotPose other;
		other.header.stamp = ros::Time::now();
		other.header.frame_id = mMapFrame;
		other.robot_id = object->robot_id;
		other.pose.x = LocalizedObject->GetCorrectedPose().GetX();
		other.pose.y = LocalizedObject->GetCorrectedPose().GetY();
		other.pose.theta = LocalizedObject->GetCorrectedPose().GetHeading();
		mOtherRobotsPublisher.publish(other);

		// Send the map via topic
		ros::WallDuration d = ros::WallTime::now() - mLastMapUpdate;
		if (mMapUpdateRate > 0 && d.toSec() > mMapUpdateRate)
		{
			sendMap();
			if (mState == ST_LOCALIZING)
			{
				mSelfLocalizer->convertMap(mGridMap);
			}
		}
	}
	else
	{
		ROS_DEBUG("Discarded Scan from Robot %d!", object->robot_id);
	}

	if (mState == ST_WAITING_FOR_MAP && mNodesAdded >= mMinMapSize)
	{
		sendMap();
		mSelfLocalizer->convertMap(mGridMap);
		mSelfLocalizer->initialize();
		mState = ST_LOCALIZING;
		ROS_INFO("Received a map, now starting to localize.");
		mSelfLocalizer->publishParticleCloud();
	}
}

void MultiMapper::receiveInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose)
{
	double x = pose->pose.pose.position.x;
	double y = pose->pose.pose.position.y;
	double yaw = tf::getYaw(pose->pose.pose.orientation);
	ROS_INFO("Received initial pose (%.2f, %.2f, %.2f) on robot %d, now starting to map.", x, y, yaw, mRobotID);
	try
	{
		setRobotPose(x, y, yaw);
	}
	catch (tf::TransformException e)
	{
		ROS_ERROR("Failed to set pose on robot %d! (%s)", mRobotID, e.what());
		return;
	}
}

void MultiMapper::onMessage(const void *sender, karto::MapperEventArguments &args)
{
	ROS_DEBUG("OpenMapper: %s\n", args.GetEventMessage().ToCString());
}

void MultiMapper::publishTransform()
{
	if (mState == ST_MAPPING)
	{
		mTransformBroadcaster.sendTransform(tf::StampedTransform(mOdometryOffset, ros::Time::now(), mOffsetFrame, mOdometryFrame));
		mTransformBroadcaster.sendTransform(tf::StampedTransform(mMapToOdometry, ros::Time::now(), mMapFrame, mOffsetFrame));
	}
}
