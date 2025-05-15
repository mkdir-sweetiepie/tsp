/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui with ROS2 integration.
 *
 * @date January 2025
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/tsp/main_window.hpp"

#include <QGroupBox>                              // 그룹박스를 위해 추가
#include <QHBoxLayout>                            // 수평 레이아웃을 위해 추가
#include <QHeaderView>                            // 헤더 뷰를 위해 추가
#include <QMessageBox>                            // 메시지 박스를 위해 추가
#include <QVBoxLayout>                            // 수직 레이아웃을 위해 추가
#include <QtDataVisualization/Q3DTheme>           // 3D 테마를 위해 추가
#include <QtDataVisualization/QScatterDataProxy>  // 산포 데이터 프록시를 위해 추가
#include <QtDataVisualization/QValue3DAxis>       // 3D 축을 위해 추가
#include <algorithm>                              // 알고리즘 함수를 위해 추가
#include <bitset>                                 // 비트셋을 위해 추가
#include <cmath>                                  // 수학 함수를 위해 추가
#include <limits>                                 // 수치형 타입의 최대/최소값을 위해 추가
#include <random>                                 // 랜덤 기능을 위해 추가

#include "ui_mainwindow.h"

using namespace QtDataVisualization;

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow), orderSeries(nullptr) {
  ui->setupUi(this);
  setWindowTitle("참외 수확 경로 최적화 (Held-Karp)");

  // 매니퓰레이터 위치 초기화 (0, 0, 0)
  manipulatorPosition = Point3D(0, 0, 0, "로봇");

  // ROS2 노드 초기화
  qnode = new QNode();
  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
  QObject::connect(qnode, SIGNAL(newCropDataReceived()), this, SLOT(processCropData()));

  // UI 요소 설정
  setupUI();

  // 시그널/슬롯 연결
  setupConnections();

  // 초기 데이터 추가
  addDefaultPoints();

  // 초기 시각화 업데이트
  updateVisualization();
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::closeEvent(QCloseEvent* event) { QMainWindow::closeEvent(event); }

void MainWindow::setupUI() {
  // 메인 레이아웃 설정
  QWidget* centralWidget = new QWidget(this);
  setCentralWidget(centralWidget);
  QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);

  // 왼쪽 패널 (컨트롤)
  QVBoxLayout* leftLayout = new QVBoxLayout();
  leftLayout->setContentsMargins(10, 10, 10, 10);
  leftLayout->setSpacing(10);

  // 좌표 테이블 그룹
  QGroupBox* coordGroupBox = new QGroupBox("참외 3D 좌표");
  QVBoxLayout* coordLayout = new QVBoxLayout(coordGroupBox);

  coordTable = new QTableWidget(0, 4);
  coordTable->setHorizontalHeaderLabels(QStringList() << "이름" << "X" << "Y" << "Z");
  coordTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  coordTable->setSelectionBehavior(QAbstractItemView::SelectRows);
  coordLayout->addWidget(coordTable);

  // 버튼 영역
  QHBoxLayout* buttonLayout = new QHBoxLayout();
  addPointButton = new QPushButton("참외 추가");
  removePointButton = new QPushButton("참외 제거");
  randomizeButton = new QPushButton("랜덤 좌표");
  buttonLayout->addWidget(addPointButton);
  buttonLayout->addWidget(removePointButton);
  buttonLayout->addWidget(randomizeButton);
  coordLayout->addLayout(buttonLayout);

  leftLayout->addWidget(coordGroupBox);

  // 계산 제어 영역
  QGroupBox* controlGroupBox = new QGroupBox("최적화 제어");
  QVBoxLayout* controlLayout = new QVBoxLayout(controlGroupBox);

  calculateButton = new QPushButton("경로 계산 (Held-Karp)");
  resetButton = new QPushButton("초기화");
  cropDataButton = new QPushButton("감지된 참외 데이터 사용");
  controlLayout->addWidget(cropDataButton);
  controlLayout->addWidget(calculateButton);
  controlLayout->addWidget(resetButton);

  leftLayout->addWidget(controlGroupBox);

  // 매니퓰레이터 위치 설정 영역 추가
  QGroupBox* manipulatorGroupBox = new QGroupBox("매니퓰레이터 시작 위치");
  QVBoxLayout* manipulatorLayout = new QVBoxLayout(manipulatorGroupBox);

  QLabel* manipulatorLabel = new QLabel("현재 위치: (0, 0, 0)");
  manipulatorLayout->addWidget(manipulatorLabel);

  leftLayout->addWidget(manipulatorGroupBox);

  // 결과 표시 영역
  QGroupBox* resultGroupBox = new QGroupBox("계산 결과");
  QVBoxLayout* resultLayout = new QVBoxLayout(resultGroupBox);
  resultLabel = new QLabel("아직 계산되지 않음");
  resultLabel->setWordWrap(true);
  resultLayout->addWidget(resultLabel);

  leftLayout->addWidget(resultGroupBox);

  // 상태 표시 영역
  statusLabel = new QLabel("ROS2 통신 대기 중...");
  leftLayout->addWidget(statusLabel);

  leftLayout->addStretch();

  // 오른쪽 패널 (3D 시각화)
  QGroupBox* visualGroupBox = new QGroupBox("3D 시각화");
  QVBoxLayout* visualLayout = new QVBoxLayout(visualGroupBox);

  scatter3D = new Q3DScatter();
  QWidget* container = QWidget::createWindowContainer(scatter3D);
  container->setMinimumSize(300, 300);
  visualLayout->addWidget(container);

  // 3D 시각화 설정
  scatter3D->activeTheme()->setType(Q3DTheme::ThemePrimaryColors);
  scatter3D->setShadowQuality(QAbstract3DGraph::ShadowQualitySoftLow);
  scatter3D->scene()->activeCamera()->setCameraPreset(Q3DCamera::CameraPresetIsometricRight);

  // 축 설정 (X,Y축: -50~50, Z축: 0~100)
  scatter3D->axisX()->setTitle("X");
  scatter3D->axisY()->setTitle("Y");  // Y축 레이블 변경
  scatter3D->axisZ()->setTitle("Z");
  scatter3D->axisX()->setTitleVisible(true);
  scatter3D->axisY()->setTitleVisible(true);
  scatter3D->axisZ()->setTitleVisible(true);

  // Qt 버전에 따라 올바른 플래그 설정 방법이 다름
  scatter3D->setSelectionMode(QAbstract3DGraph::SelectionNone);        // 점 선택 비활성화
  scatter3D->scene()->activeCamera()->setCameraPosition(45, 45, 200);  // 초기 카메라 위치 설정
  scatter3D->scene()->activeCamera()->setZoomLevel(85);                // 줌 레벨 설정
  scatter3D->setAspectRatio(1.5f);                                     // 화면비 설정

  // 고정 범위 설정
  scatter3D->axisX()->setRange(-50, 50);
  scatter3D->axisY()->setRange(-50, 50);
  scatter3D->axisZ()->setRange(0, 100);

  // Y축 방향 반전 (아래로 +값이 되도록)
  QValue3DAxis* yAxis = scatter3D->axisY();
  yAxis->setReversed(true);  // Y축 방향 반전

  // 참외 위치 시리즈 설정
  pointSeries = new QScatter3DSeries;
  pointSeries->setItemSize(0.15f);
  pointSeries->setMesh(QAbstract3DSeries::MeshSphere);
  pointSeries->setBaseColor(QColor(255, 165, 0));  // 참외 색상 (주황색)
  scatter3D->addSeries(pointSeries);

  // 경로 시리즈 설정
  pathSeries = new QScatter3DSeries;
  pathSeries->setItemSize(0.05f);
  pathSeries->setMesh(QAbstract3DSeries::MeshPoint);
  pathSeries->setBaseColor(QColor(Qt::red));  // 경로는 빨간색으로 표시
  scatter3D->addSeries(pathSeries);

  // 경로 순서 표시 시리즈 설정
  orderSeries = new QScatter3DSeries;
  orderSeries->setItemSize(0.2f);
  orderSeries->setMesh(QAbstract3DSeries::MeshCube);  // 큐브 모양으로 표시
  scatter3D->addSeries(orderSeries);

  // 로봇 위치 시리즈 설정 (별도 추가)
  manipulatorSeries = new QScatter3DSeries;
  manipulatorSeries->setItemSize(0.3f);
  manipulatorSeries->setMesh(QAbstract3DSeries::MeshMinimal);  // 다른 형태로 표시
  manipulatorSeries->setBaseColor(QColor(0, 0, 255));          // 파란색
  scatter3D->addSeries(manipulatorSeries);

  // 레이아웃 추가
  mainLayout->addLayout(leftLayout, 1);
  mainLayout->addWidget(visualGroupBox, 2);

  publishButton = new QPushButton("경로 ROS2 발행");
  controlLayout->addWidget(publishButton);
}

void MainWindow::resetAll() {
  // 모든 데이터 초기화
  coordTable->setRowCount(0);
  points.clear();
  optimalPath.clear();
  minCost = 0;
  resultLabel->setText("아직 계산되지 않음");
  statusLabel->setText("ROS2 통신 대기 중...");

  // 기본 점 추가
  addDefaultPoints();

  // 시각화 업데이트 (테이블에서 읽지 않고 직접 points 배열 사용)
  updateVisualization();
}

void MainWindow::setupConnections() {
  connect(addPointButton, &QPushButton::clicked, this, &MainWindow::addPoint);
  connect(removePointButton, &QPushButton::clicked, this, &MainWindow::removePoint);
  connect(randomizeButton, &QPushButton::clicked, this, &MainWindow::randomizePoints);
  connect(calculateButton, &QPushButton::clicked, this, &MainWindow::calculateOptimalPath);
  connect(resetButton, &QPushButton::clicked, this, &MainWindow::resetAll);
  connect(cropDataButton, &QPushButton::clicked, this, &MainWindow::requestCropData);

  // 테이블 데이터 변경 감지
  connect(coordTable, &QTableWidget::cellChanged, this, &MainWindow::onTableDataChanged);
  connect(publishButton, &QPushButton::clicked, this, &MainWindow::publishOptimalPath);
}

// 참외 데이터 요청 핸들러 (버튼 클릭 시)
void MainWindow::requestCropData() {
  // 이미 수신된 가장 최근 참외 데이터 처리
  processCropData();
}

// 새로운 함수: ROS2에서 감지된 참외 데이터 처리
void MainWindow::processCropData() {
  // QNode에서 참외 데이터 가져오기
  const vision_msgs::msg::DetectedCropArray& crop_data = qnode->getCropData();

  if (crop_data.total_objects == 0) {
    QMessageBox::information(this, "정보", "감지된 참외가 없습니다.");
    return;
  }

  isInitializing = true;  // 테이블 업데이트 중 이벤트 발생 방지

  // 기존 데이터 지우기
  coordTable->setRowCount(0);
  points.clear();

  // 감지된 참외 데이터로 테이블과 점 목록 업데이트
  for (size_t i = 0; i < crop_data.objects.size(); ++i) {
    const auto& crop = crop_data.objects[i];

    int row = coordTable->rowCount();
    coordTable->insertRow(row);

    // 이름은 "참외" + ID로 설정
    QString name = QString("참외%1").arg(crop.id);

    // Z좌표가 음수일 경우 0으로 조정
    float z = crop.z;
    if (z < 0) z = 0.0f;

    // 테이블 항목 추가
    coordTable->setItem(row, 0, new QTableWidgetItem(name));
    coordTable->setItem(row, 1, new QTableWidgetItem(QString::number(crop.x, 'f', 2)));
    coordTable->setItem(row, 2, new QTableWidgetItem(QString::number(crop.y, 'f', 2)));
    coordTable->setItem(row, 3, new QTableWidgetItem(QString::number(z, 'f', 2)));

    // points 배열에 추가 (Z좌표 조정됨)
    points.append(Point3D(crop.x, crop.y, z, name));
  }

  isInitializing = false;

  // 경로 초기화
  optimalPath.clear();
  minCost = 0;
  resultLabel->setText("감지된 참외 데이터 로드됨. 경로 계산 필요.");

  // 상태 업데이트
  statusLabel->setText(QString("감지된 참외 수: %1").arg(crop_data.total_objects));

  // 시각화 업데이트
  updateVisualization();
}

// 테이블 데이터 변경 감지 함수 추가
void MainWindow::onTableDataChanged(int row, int column) {
  // 초기화 중에는 무시
  if (isInitializing) return;

  // 이름 변경이 아닌 좌표 변경일 경우에만 처리
  if (column >= 1 && column <= 3 && row < points.size()) {
    // 변경된 값 읽기
    bool ok;
    float value = coordTable->item(row, column)->text().toFloat(&ok);

    // 변환 실패 시 0으로 설정
    if (!ok) {
      value = 0.0f;
      coordTable->item(row, column)->setText("0.0");
    }

    // Z좌표(column=3)가 음수일 경우 0으로 조정
    if (column == 3 && value < 0) {
      value = 0.0f;
      coordTable->item(row, column)->setText("0.0");
    }

    // points 배열 업데이트
    if (column == 1)
      points[row].x = value;
    else if (column == 2)
      points[row].y = value;
    else if (column == 3)
      points[row].z = value;

    // 시각화 업데이트
    updateVisualization();
  } else if (column == 0 && row < points.size()) {
    // 이름 변경
    points[row].name = coordTable->item(row, 0)->text();
  }
}

void MainWindow::addPoint() {
  int row = coordTable->rowCount();  // 현재 데이블의 행 개수 가져옴
  coordTable->insertRow(row);        // 테이블 맨 아래에 새 행 삽입

  QString name = QString("참외%1").arg(row + 1);
  coordTable->setItem(row, 0, new QTableWidgetItem(name));
  coordTable->setItem(row, 1, new QTableWidgetItem("0.0"));  // X = 0
  coordTable->setItem(row, 2, new QTableWidgetItem("0.0"));  // Y = 0
  coordTable->setItem(row, 3, new QTableWidgetItem("0.0"));  // Z = 0 (양수 범위에서 시작)

  Point3D newPoint(0, 0, 0, name);  // 기본 좌표(0,0,0)와 이름으로 새 점 생성
  points.append(newPoint);          // points 배열에 새 점 추가

  updateVisualization();
}

void MainWindow::removePoint() {
  QList<QTableWidgetItem*> selectedItems = coordTable->selectedItems();  // 선택된 항목 가져옴
  if (selectedItems.isEmpty()) {                                         // 선택된 항목이 없으면 경고 메시지 출력
    QMessageBox::warning(this, "경고", "제거할 행을 선택하세요.");
    return;
  }

  int row = coordTable->row(selectedItems.first());  // 첫 번째 선택된 항목의 행 인덱스 가져옴
  coordTable->removeRow(row);                        // 해당 행 제거

  // points 배열에서도 제거
  if (row < points.size()) {
    points.remove(row);
  }

  // 경로가 있으면 초기화
  optimalPath.clear();

  updateVisualization();
}

// 랜덤 좌표 생성 함수 추가
void MainWindow::randomizePoints() {
  isInitializing = true;  // 초기화 플래그 설정

  std::random_device rd;   // 랜덤 디바이스 생성
  std::mt19937 gen(rd());  // 메르센 트위스터 엔진 생성

  // X, Y축 범위 (-50~50)
  std::uniform_real_distribution<float> distXY(-50.0f, 50.0f);

  // Z축 범위 (0~100)
  std::uniform_real_distribution<float> distZ(0.0f, 100.0f);

  // 기존의 모든 행에 대해 랜덤 좌표 설정
  for (int row = 0; row < coordTable->rowCount(); ++row) {
    float x = distXY(gen);
    float y = distXY(gen);
    float z = distZ(gen);  // Z축은 0~100 범위로 설정

    // 테이블 업데이트
    coordTable->item(row, 1)->setText(QString::number(x, 'f', 2));
    coordTable->item(row, 2)->setText(QString::number(y, 'f', 2));
    coordTable->item(row, 3)->setText(QString::number(z, 'f', 2));

    // points 배열 업데이트
    if (row < points.size()) {
      points[row].x = x;
      points[row].y = y;
      points[row].z = z;
    }
  }

  isInitializing = false;  // 초기화 플래그 해제

  // 경로가 있으면 초기화
  optimalPath.clear();

  // 시각화 업데이트
  updateVisualization();
}

void MainWindow::updateVisualization() {
  // 항상 고정된 축 범위 사용
  scatter3D->axisX()->setRange(-50, 50);
  scatter3D->axisY()->setRange(-50, 50);
  scatter3D->axisZ()->setRange(0, 100);

  // 점 데이터 업데이트
  QScatterDataArray* dataArray = new QScatterDataArray;
  dataArray->resize(points.size());

  // 각 점의 좌표를 데이터 배열에 설정
  for (int i = 0; i < points.size(); ++i) {
    // Y 좌표는 그대로 사용 (Qt에서 setReversed(true)를 통해 이미 반전됨)
    (*dataArray)[i].setPosition(QVector3D(points[i].x, points[i].y, points[i].z));
  }
  // 점 시리즈 업데이트
  pointSeries->dataProxy()->resetArray(dataArray);

  // 매니퓰레이터 위치 시각화
  QtDataVisualization::QScatterDataArray* manipulatorArray = new QtDataVisualization::QScatterDataArray;
  manipulatorArray->resize(1);
  (*manipulatorArray)[0].setPosition(QVector3D(manipulatorPosition.x, manipulatorPosition.y, manipulatorPosition.z));
  manipulatorSeries->dataProxy()->resetArray(manipulatorArray);

  // 경로가 있을 경우 경로 데이터 업데이트
  if (!optimalPath.isEmpty()) {
    updatePathVisualization();
  } else {
    // 경로가 없으면 경로 시리즈 초기화
    pathSeries->dataProxy()->resetArray(new QScatterDataArray);
    orderSeries->dataProxy()->resetArray(new QScatterDataArray);
  }
}

void MainWindow::updatePathVisualization() {
  // 경로가 없으면 초기화
  if (optimalPath.isEmpty()) {
    pathSeries->dataProxy()->resetArray(new QScatterDataArray);
    orderSeries->dataProxy()->resetArray(new QScatterDataArray);
    return;
  }

  // 경로 선 그리기
  QScatterDataArray* pathArray = new QScatterDataArray;
  int pathLength = optimalPath.size();
  int pointsPerSegment = 100;
  int totalPoints = (pathLength - 1) * pointsPerSegment;
  pathArray->resize(totalPoints);

  int arrayIndex = 0;

  // 각 세그먼트를 여러 점으로 보간하여 선으로 표현
  for (int i = 0; i < pathLength - 1; ++i) {
    QVector3D startPos, endPos;

    // 시작점 결정
    if (optimalPath[i] == -1) {
      // 매니퓰레이터 위치
      startPos = QVector3D(manipulatorPosition.x, manipulatorPosition.y, manipulatorPosition.z);
    } else {
      // 참외 위치
      startPos = QVector3D(points[optimalPath[i]].x, points[optimalPath[i]].y, points[optimalPath[i]].z);
    }

    // 끝점 결정
    if (optimalPath[i + 1] == -1) {
      // 매니퓰레이터 위치
      endPos = QVector3D(manipulatorPosition.x, manipulatorPosition.y, manipulatorPosition.z);
    } else {
      // 참외 위치
      endPos = QVector3D(points[optimalPath[i + 1]].x, points[optimalPath[i + 1]].y, points[optimalPath[i + 1]].z);
    }

    // 선분을 여러 점으로 보간
    for (int j = 0; j < pointsPerSegment; ++j) {
      float t = static_cast<float>(j) / pointsPerSegment;
      float x = startPos.x() * (1 - t) + endPos.x() * t;
      float y = startPos.y() * (1 - t) + endPos.y() * t;
      float z = startPos.z() * (1 - t) + endPos.z() * t;

      (*pathArray)[arrayIndex++].setPosition(QVector3D(x, y, z));
    }
  }

  // 경로 시리즈 업데이트
  pathSeries->dataProxy()->resetArray(pathArray);
  pathSeries->setItemSize(0.05f);
  pathSeries->setBaseColor(QColor(Qt::red));  // 경로는 빨간색
}

// 여기서 부터 최적 경로 계산 함수
void MainWindow::calculateOptimalPath() {
  int n = points.size();
  if (n == 0) {
    QMessageBox::warning(this, "경고", "최소 1개 이상의 참외가 필요합니다.");
    return;
  }

  // 결과 초기화
  optimalPath.clear();
  minCost = 0;

  // 매니퓰레이터와 모든 참외 간의 거리 행렬 계산
  std::vector<std::vector<double>> dist(n + 1, std::vector<double>(n + 1, 0));

  // 매니퓰레이터 위치는 인덱스 0으로 처리
  for (int i = 0; i <= n; ++i) {
    for (int j = 0; j <= n; ++j) {
      if (i == j) {
        dist[i][j] = 0;  // 자기 자신까지의 거리는 0
      } else if (i == 0) {
        // 매니퓰레이터에서 참외까지의 거리
        double dx = manipulatorPosition.x - points[j - 1].x;
        double dy = manipulatorPosition.y - points[j - 1].y;
        double dz = manipulatorPosition.z - points[j - 1].z;
        dist[i][j] = std::sqrt(dx * dx + dy * dy + dz * dz);
      } else if (j == 0) {
        // 참외에서 매니퓰레이터까지의 거리
        double dx = points[i - 1].x - manipulatorPosition.x;
        double dy = points[i - 1].y - manipulatorPosition.y;
        double dz = points[i - 1].z - manipulatorPosition.z;
        dist[i][j] = std::sqrt(dx * dx + dy * dy + dz * dz);
      } else {
        // 참외 간의 거리
        double dx = points[i - 1].x - points[j - 1].x;
        double dy = points[i - 1].y - points[j - 1].y;
        double dz = points[i - 1].z - points[j - 1].z;
        dist[i][j] = std::sqrt(dx * dx + dy * dy + dz * dz);
      }
    }
  }

  // 많은 참외가 있을 경우 경고
  if (n > 11) {
    QMessageBox::warning(this, "경고", "계산 효율성을 위해 최대 11개 참외까지 지원합니다.");
    return;
  }

  // 참외가 적을 경우(11개 이하) 모든 순열을 시도하는 방식 사용
  std::vector<int> indices;
  for (int i = 1; i <= n; ++i) {  // 1부터 시작 (0은 매니퓰레이터 위치)
    indices.push_back(i);
  }

  std::vector<int> bestPath;
  double bestCost = std::numeric_limits<double>::max();

  // 모든 순열 시도
  do {
    double currentCost = dist[0][indices[0]];  // 매니퓰레이터에서 첫 번째 참외까지

    for (int i = 0; i < indices.size() - 1; ++i) {
      currentCost += dist[indices[i]][indices[i + 1]];
    }

    currentCost += dist[indices.back()][0];  // 마지막 참외에서 매니퓰레이터로

    if (currentCost < bestCost) {
      bestCost = currentCost;
      bestPath = indices;
    }

  } while (std::next_permutation(indices.begin(), indices.end()));

  // 최적 경로 구성 (매니퓰레이터 위치에서 시작하여 다시 돌아옴)
  optimalPath.clear();
  optimalPath.push_back(-1);  // 매니퓰레이터 위치 (-1로 표시)

  for (int idx : bestPath) {
    optimalPath.push_back(idx - 1);  // 실제 points 배열 인덱스로 변환
  }

  optimalPath.push_back(-1);  // 매니퓰레이터 위치로 돌아오기
  minCost = bestCost;

  // 결과 표시
  QString resultText = QString("최적 경로 비용: %1\n경로: %2").arg(minCost).arg(manipulatorPosition.name);

  for (int i = 1; i < optimalPath.size() - 1; ++i) {
    int idx = optimalPath[i];
    resultText += " → " + points[idx].name;
  }

  resultText += " → " + manipulatorPosition.name;
  resultLabel->setText(resultText);

  // 시각화 업데이트
  updatePathVisualization();
  publishOptimalPath();
}

void MainWindow::addDefaultPoints() {
  isInitializing = true;  // 초기화 플래그 설정

  // 테이블 초기화
  coordTable->setRowCount(0);
  points.clear();

  // 랜덤 좌표 생성을 위한 설정
  std::random_device rd;
  std::mt19937 gen(rd());

  // X, Y축 범위 (-50~50)
  std::uniform_real_distribution<float> distXY(-50.0f, 50.0f);

  // Z축 범위 (0~100)
  std::uniform_real_distribution<float> distZ(0.0f, 100.0f);

  // 8개의 참외 생성 (랜덤 좌표)
  const int numPoints = 8;

  for (int i = 0; i < numPoints; ++i) {
    int row = coordTable->rowCount();
    coordTable->insertRow(row);

    // 랜덤 좌표 생성 (X,Y: -50~50, Z: 0~100)
    float x = distXY(gen);
    float y = distXY(gen);
    float z = distZ(gen);

    QString name = QString("참외%1").arg(i + 1);
    coordTable->setItem(row, 0, new QTableWidgetItem(name));
    coordTable->setItem(row, 1, new QTableWidgetItem(QString::number(x, 'f', 2)));
    coordTable->setItem(row, 2, new QTableWidgetItem(QString::number(y, 'f', 2)));
    coordTable->setItem(row, 3, new QTableWidgetItem(QString::number(z, 'f', 2)));

    points.append(Point3D(x, y, z, name));
  }

  isInitializing = false;  // 초기화 플래그 해제
}

// 최적 경로 ROS2 발행 함수
void MainWindow::publishOptimalPath() {
  // 경로가 없으면 발행하지 않음
  if (optimalPath.isEmpty()) {
    statusLabel->setText("발행할 경로가 없습니다. 먼저 경로를 계산하세요.");
    return;
  }

  // optimalPath에서 매니퓰레이터 위치 제외하고 실제 참외만 포함된 경로 생성
  QVector<int> harvestPath;

  // 매니퓰레이터 시작/종료 위치 제외하고 중간 참외들만 수집
  for (int i = 1; i < optimalPath.size() - 1; ++i) {
    if (optimalPath[i] != -1) {
      harvestPath.append(optimalPath[i]);
    }
  }

  // QNode를 통해 수확 순서 발행 (매니퓰레이터 시작 위치 정보 포함)
  qnode->publishHarvestOrder(harvestPath, points, manipulatorPosition);

  // 상태 업데이트
  statusLabel->setText("수확 순서가 ROS2 토픽 '/harvest_ordering'에 발행되었습니다.");
}