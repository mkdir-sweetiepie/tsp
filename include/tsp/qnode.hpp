/**
 * @file /include/tsp/qnode.hpp
 *
 * @brief ROS2 communications central!
 *
 * @date January 2025
 **/
#ifndef TSP_QNODE_HPP
#define TSP_QNODE_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QObject>
#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vision_msgs/msg/detected_crop_array.hpp>  // 추가: 참외 감지 메시지 타입

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
  Q_OBJECT
 public:
  QNode();
  virtual ~QNode();

  // Thread-related functions
  void run() override;

  // ROS2 node access
  std::shared_ptr<rclcpp::Node> get_node() { return node; }

  // 감지된 참외 데이터 접근
  const vision_msgs::msg::DetectedCropArray& getCropData() const { return crop_data; }

 Q_SIGNALS:
  void rosShutDown();
  void newCropDataReceived();  // 새 참외 데이터가 수신되면 발생하는 시그널

 private:
  std::shared_ptr<rclcpp::Node> node;

  // 참외 감지 데이터 구독
  rclcpp::Subscription<vision_msgs::msg::DetectedCropArray>::SharedPtr crop_subscription;
  vision_msgs::msg::DetectedCropArray crop_data;

  // 참외 데이터 콜백 함수
  void cropCallback(const vision_msgs::msg::DetectedCropArray::SharedPtr msg);
};

#endif  // TSP_QNODE_HPP