#include <jsk_uav_forest_perception/tree_database.h>
 
TreeHandle::TreeHandle(ros::NodeHandle nh, ros::NodeHandle nhp, tf::Vector3 pos):nh_(nh), nhp_(nhp), pos_(pos), vote_(1), radius_(-1)
{
  nhp_.param("filter_rate", filter_rate_, 0.8);
}

void TreeHandle::updatePos(const tf::Vector3& pos, bool lpf)
{
  if(lpf) pos_ = filter_rate_ * pos_  + (1 - filter_rate_) * pos;
  else pos_ = pos;
  vote_++;
}

void TreeHandle::setRadius(double radius, bool lpf)
{
  if (radius_ < 0) {
    radius_ = radius; //init
  } else {
    if(lpf) radius_ = filter_rate_ * radius_  + (1 - filter_rate_) * radius;
    else radius_ = radius;
  }
}

bool operator>(const TreeHandlePtr& left, const TreeHandlePtr& right)
{
  return left->getVote() > right->getVote() ;
}

TreeDataBase::TreeDataBase(ros::NodeHandle nh, ros::NodeHandle nhp):nh_(nh), nhp_(nhp)
{
  trees_.resize(0);
  nhp_.param("min_distance", min_distance_, 1.0); // 1.0[m]
  nhp_.param("max_radius", max_radius_, 0.5); // 1.0[m]
  nhp_.param("valid_num", valid_num_, 7);
  nhp_.param("verbose", verbose_, false);
  nhp_.param("visualization_marker_topic_name", visualization_marker_topic_name_, string("/visualization_marker"));
  pub_visualization_marker_ = nh_.advertise<visualization_msgs::MarkerArray>(visualization_marker_topic_name_, 1);
}

void TreeDataBase::add(const TreeHandlePtr new_tree)
{
  trees_.push_back(new_tree);
  ROS_INFO("add new tree No.%d: [%f, %f]", (int)trees_.size(), new_tree->getPos().x(), new_tree->getPos().y());
}

bool TreeDataBase::updateSingleTree(const tf::Vector3& tree_pos, const double& tree_radius, const bool only_target)
{
  bool new_tree = true;
  float min_dist = 1e6;
  TreeHandlePtr target_tree;
  int tree_index = 0;
  for(vector<TreeHandlePtr>::iterator it = trees_.begin(); it != trees_.end(); ++it)
    {
      float dist = (tree_pos - (*it)->getPos()).length();
      size_t index = distance(trees_.begin(), it);

      /* we assume that the distance of any two trees is more than min_ditance_ */
      if(dist < min_distance_)
	{
	  if(!new_tree) ROS_WARN("there are two trees which are to close to each other");
	  new_tree = false;
	}

      /* update */
      if(min_dist > dist)
	{
	  min_dist = dist;
	  target_tree = *it;
	  tree_index = index;
	  if(verbose_) cout << "Database tree No." << tree_index << ": udpate min_dist: " << min_dist << endl;
	}
    }

  if(min_dist < max_radius_)
    {
      /* update the global pos of the tree */
      target_tree->updatePos(tree_pos,false);
      target_tree->setRadius(tree_radius);
      if(verbose_) cout << "Database tree No." << tree_index << ": update, small diff:" << min_dist << endl;
      return true;
    }
  else
    {
      if(verbose_  && !only_target)
	ROS_WARN("Database tree No.%d, lost the target tree, drift: %f, the nearest target location: [%f, %f, %f], prev target location: [%f, %f, %f]", tree_index, min_dist, target_tree->getPos().x(), target_tree->getPos().y(), target_tree->getPos().z(), tree_pos.x(), tree_pos.y(), tree_pos.z());
    }

  /* add new tree if necessary */
  if(new_tree && !only_target)
    {
      TreeHandlePtr new_tree = TreeHandlePtr(new TreeHandle(nh_, nhp_, tree_pos));
      add(new_tree);
      return true;
    }

  return false;
}

void TreeDataBase::update()
{
  /* sort */
  std::sort(trees_.begin(),trees_.end(),std::greater<TreeHandlePtr>());

  if(verbose_)
    {
      cout << "update(sort): ";
      for (vector<TreeHandlePtr>::iterator it = trees_.begin(); it != trees_.end(); it++)
	cout << (*it)->getVote() << ", ";
      cout << endl;
    }
}

void TreeDataBase::visualization(std_msgs::Header header)
{
  visualization_msgs::MarkerArray msg;

  for (vector<TreeHandlePtr>::iterator it = trees_.begin(); it != trees_.end(); it++) {
    size_t index = distance(trees_.begin(), it);

    /* only show top level trees */
    if(index == valid_num_) break;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "tree_diameter";
    marker.id = index;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (*it)->getPos().x();
    marker.pose.position.y = (*it)->getPos().y();
    marker.pose.position.z = 3.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.5;
    marker.color.g = 1.0;
    marker.color.a = 1.0;
    ostringstream sout;
    sout << fixed << setprecision(3) << (((*it)->getRadius()) * 2);
    marker.text = sout.str();
    marker.lifetime = ros::Duration();
    msg.markers.push_back(marker);

    marker.ns = "tree";
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.z = 2.0; //tree height
    marker.scale.x = marker.scale.y = (*it)->getRadius() * 2;
    marker.pose.position.z = marker.scale.z / 2;
    marker.color.r = 0.95;
    marker.color.g = 0.59;
    marker.color.b = 0;
    marker.color.a = 0.5;
    msg.markers.push_back(marker);
  }
  pub_visualization_marker_.publish(msg);
}
