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
#ifndef SWITCH_PANEL_H
#define SWITCH_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>

class QPushButton;
class QLineEdit;

namespace wpr_warehousing_monitor
{

  class SwitchPanel: public rviz::Panel
  {
  Q_OBJECT
  public:
    
    SwitchPanel( QWidget* parent = 0 );

    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

  public Q_SLOTS:
    void setMode( int mode );
    void start_slot();
    void stop_slot();
    void charge_slot();
    void pause_slot();
    void continue_slot();
    void pose_init_slot();
    void pose_set_slot();
    void pose_waypoint_slot();

    void setTopic( const QString& topic );

  protected Q_SLOTS:
    void sendMode();
    void updateTopic();

  protected:
    QLineEdit* output_topic_editor_;
    QPushButton* pose_waypoint_btn_;
    QPushButton* start_btn_;
    QPushButton* stop_btn_;
    QPushButton* pause_btn_;
    QPushButton* continue_btn_;
    QPushButton* charge_btn_;
    QPushButton* pose_init_btn_;
    QPushButton* pose_set_btn_;

    QString output_topic_;

    ros::Publisher command_publisher_;

    ros::NodeHandle nh_;

    int nMode;
  };

} // end namespace wpr_warehousing_monitor

#endif 