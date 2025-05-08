/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date January 2025
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/tsp/qnode.hpp"

QNode::QNode() {
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("tsp");

  // 참외 감지 데이터 구독
  crop_subscription = node->create_subscription<vision_msgs::msg::DetectedCropArray>("detected_crops", 10,  // 토픽 이름과 큐 크기
                                                                                     std::bind(&QNode::cropCallback, this, std::placeholders::_1));

  this->start();
}

QNode::~QNode() {
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void QNode::run() {
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

void QNode::cropCallback(const vision_msgs::msg::DetectedCropArray::SharedPtr msg) {
  // 수신된 참외 데이터 저장
  crop_data = *msg;

  // 메인 윈도우에 신호 전송
  Q_EMIT newCropDataReceived();
}