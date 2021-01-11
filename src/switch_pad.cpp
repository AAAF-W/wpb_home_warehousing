/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>

#include <std_msgs/String.h>

#include "../include/switch_pad.h"

namespace wpr_warehousing_monitor
{

SwitchPanel::SwitchPanel( QWidget* parent )
  : rviz::Panel( parent )
  , nMode( 0 )
{
  QVBoxLayout* topic_layout = new QVBoxLayout;
  topic_layout->addWidget( new QLabel( "Output Topic:" ));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_ );
  QVBoxLayout* btn_layout = new QVBoxLayout;
  pose_waypoint_btn_= new QPushButton;
  pose_waypoint_btn_->setText("定位到航点");
  btn_layout->addWidget(pose_waypoint_btn_);
  start_btn_= new QPushButton;
  start_btn_->setText("开始");
  btn_layout->addWidget(start_btn_);
  stop_btn_= new QPushButton;
  stop_btn_->setText("停止"); 
  btn_layout->addWidget(stop_btn_);
  pause_btn_= new QPushButton;
  pause_btn_->setText("暂停");
  btn_layout->addWidget(pause_btn_);
  continue_btn_= new QPushButton;
  continue_btn_->setText("继续");
  btn_layout->addWidget(continue_btn_);
  charge_btn_= new QPushButton;
  charge_btn_->setText("去充电"); 
  btn_layout->addWidget(charge_btn_);
  pose_init_btn_= new QPushButton;
  pose_init_btn_->setText("移动后定位"); 
  btn_layout->addWidget(pose_init_btn_);
  pose_set_btn_= new QPushButton;
  pose_set_btn_->setText("原地定位"); 
  btn_layout->addWidget(pose_set_btn_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );
  layout->addLayout( btn_layout );
  setLayout( layout );

  connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
  connect( pose_waypoint_btn_, SIGNAL( pressed() ), this, SLOT( pose_waypoint_slot() ));
  connect( start_btn_, SIGNAL( pressed() ), this, SLOT( start_slot() ));
  connect( stop_btn_, SIGNAL( pressed() ), this, SLOT( stop_slot() ));
  connect( continue_btn_, SIGNAL( pressed() ), this, SLOT( continue_slot() ));
  connect( pause_btn_, SIGNAL( pressed() ), this, SLOT( pause_slot() ));
  connect( charge_btn_, SIGNAL( pressed() ), this, SLOT( charge_slot() ));
  connect( pose_init_btn_, SIGNAL( pressed() ), this, SLOT( pose_init_slot() ));
  connect( pose_set_btn_, SIGNAL( pressed() ), this, SLOT( pose_set_slot() ));

}

void SwitchPanel::setMode( int mode )
{
  nMode = mode;
}

void SwitchPanel::start_slot()
{
    nMode = 1;
    sendMode();
}
void SwitchPanel::stop_slot()
{
    nMode = 0;
    sendMode();
}
void SwitchPanel::pause_slot()
{
    nMode = 2;
    sendMode();
}
void SwitchPanel::continue_slot()
{
    nMode = 3;
    sendMode();
}
void SwitchPanel::charge_slot()
{
    nMode = 4;
    sendMode();
}
void SwitchPanel::pose_init_slot()
{
    nMode = 5;
    sendMode();
}
void SwitchPanel::pose_set_slot()
{
    nMode = 6;
    sendMode();
}
  
void SwitchPanel::pose_waypoint_slot()
{
    nMode = 7;
    sendMode();
}

void SwitchPanel::updateTopic()
{
  setTopic( output_topic_editor_->text() );
}

void SwitchPanel::setTopic( const QString& new_topic )
{
  // Only take action if the name has changed.
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
    // If the topic is the empty string, don't publish anything.
    if( output_topic_ == "" )
    {
      command_publisher_.shutdown();
    }
    else
    {
      command_publisher_ = nh_.advertise<std_msgs::String>( output_topic_.toStdString(), 1 );
    }
    // rviz::Panel defines the configChanged() signal.  Emitting it
    // tells RViz that something in this panel has changed that will
    // affect a saved config file.  Ultimately this signal can cause
    // QWidget::setWindowModified(true) to be called on the top-level
    // rviz::VisualizationFrame, which causes a little asterisk ("*")
    // to show in the window's title bar indicating unsaved changes.
    Q_EMIT configChanged();
  }

}

// Publish the message if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void SwitchPanel::sendMode()
{
  if( ros::ok() && command_publisher_ )
  {
    std_msgs::String msg;
    switch (nMode)
    {
    case 0:
        msg.data = "stop";
        break;
    case 1:
        msg.data = "start";
        break;
    case 2:
        msg.data = "pause";
        break;
    case 3:
        msg.data = "continue";
        break;
    case 4:
        msg.data = "charge";
        break;
    case 5:
        msg.data = "pose_init";
        break;
    case 6:
        msg.data = "pose_set";
        break;
    case 7:
        msg.data = "pose_waypoint";
        break;
    default:
        break;
    }
    command_publisher_.publish( msg );
  }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void SwitchPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Cmd_Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void SwitchPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Cmd_Topic", &topic ))
  {
    output_topic_editor_->setText( topic );
    updateTopic();
  }
}

} // end namespace wpr_warehousing_monitor

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(wpr_warehousing_monitor::SwitchPanel,rviz::Panel )