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

#include "../include/tsp/main_window.hpp"  // Point3D 구조체 사용을 위해 추가

QNode::QNode() {
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("tsp");

  // 참외 감지 데이터 구독
  crop_subscription = node->create_subscription<vision_msgs::msg::DetectedCropArray>("/detected_crops", 10,  // 토픽 이름과 큐 크기
                                                                                     std::bind(&QNode::cropCallback, this, std::placeholders::_1));

  // 수확 순서 발행자 초기화
  harvest_publisher = node->create_publisher<vision_msgs::msg::HarvestOrdering>("/harvest_ordering", 10);  // 토픽 이름과 큐 크기

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

// 수확 순서 발행 메소드 구현
void QNode::publishHarvestOrder(const QVector<int>& path, const QVector<Point3D>& points) {
  // 경로가 비어있으면 발행하지 않음
  if (path.isEmpty() || points.isEmpty()) {
    return;
  }

  // 메시지 생성
  auto message = std::make_shared<vision_msgs::msg::HarvestOrdering>();

  // 헤더 설정
  message->header.stamp = node->now();
  message->header.frame_id = "map";  // 원본 메시지와 동일한 frame_id 사용

  // 원본 메시지와 동일한 형태로 참외 객체 정보 복사
  for (const auto& point : points) {
    vision_msgs::msg::DetectedCrop crop;

    // 이름에서 ID 추출 ("참외1" -> 1)
    QString name = point.name;
    bool ok;
    int id = name.mid(2).toInt(&ok);

    if (!ok) {
      // 변환 실패 시 인덱스를 찾아서 ID 설정
      for (int i = 0; i < points.size(); ++i) {
        if (points[i].name == name) {
          id = i + 1;  // 1부터 시작하는 ID 사용
          break;
        }
      }
    }

    crop.id = id;
    crop.x = point.x;
    crop.y = point.y;
    crop.z = point.z;

    message->objects.push_back(crop);
  }

  // 총 객체 수 설정
  message->total_objects = points.size();

  // 수확 순서 추가 (ID 배열)
  for (int i = 0; i < path.size(); ++i) {
    int pointIndex = path[i];

    // 인덱스 유효성 검사
    if (pointIndex < 0 || pointIndex >= points.size()) {
      continue;
    }

    // 참외 이름에서 ID 추출 ("참외1" -> 1)
    QString name = points[pointIndex].name;
    bool ok;
    int id = name.mid(2).toInt(&ok);

    if (!ok) {
      // 변환 실패 시 인덱스 사용
      id = pointIndex + 1;  // 1부터 시작하는 ID 사용
    }

    message->crop_ids.push_back(id);
  }

  // 메시지 발행
  harvest_publisher->publish(*message);
}