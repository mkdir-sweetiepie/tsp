#include "./ui_mainwindow.h"
#include "mainwindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QMessageBox>
#include <QtDataVisualization/Q3DTheme>
#include <QtDataVisualization/QScatterDataProxy>
#include <QtDataVisualization/QValue3DAxis>
#include <cmath>
#include <algorithm>
#include <limits>
#include <bitset>
#include <random> // 랜덤 기능을 위해 추가

using namespace QtDataVisualization;

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow), orderSeries(nullptr)
{
  ui->setupUi(this);
  setWindowTitle("참외 수확 경로 최적화 (Held-Karp)");
  
  // UI 요소 설정
  setupUI();
  
  // 시그널/슬롯 연결
  setupConnections();
  
  // 초기 데이터 추가
  addDefaultPoints();
  updateVisualization();
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::setupUI()
{
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
  randomizeButton = new QPushButton("랜덤 좌표"); // 랜덤 버튼 추가
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
  controlLayout->addWidget(calculateButton);
  controlLayout->addWidget(resetButton);
  
  leftLayout->addWidget(controlGroupBox);
  
  // 결과 표시 영역
  QGroupBox* resultGroupBox = new QGroupBox("계산 결과");
  QVBoxLayout* resultLayout = new QVBoxLayout(resultGroupBox);
  resultLabel = new QLabel("아직 계산되지 않음");
  resultLabel->setWordWrap(true);
  resultLayout->addWidget(resultLabel);
  
  leftLayout->addWidget(resultGroupBox);
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
  
  // 축 설정
  scatter3D->axisX()->setTitle("X");
  scatter3D->axisY()->setTitle("Y");
  scatter3D->axisZ()->setTitle("Z");
  scatter3D->axisX()->setTitleVisible(true);
  scatter3D->axisY()->setTitleVisible(true);
  scatter3D->axisZ()->setTitleVisible(true);
  scatter3D->axisX()->setRange(0, 10);
  scatter3D->axisY()->setRange(0, 10);
  scatter3D->axisZ()->setRange(0, 10);
  
  // 참외 위치 시리즈 설정
  pointSeries = new QScatter3DSeries;
  pointSeries->setItemSize(0.15f);
  pointSeries->setMesh(QAbstract3DSeries::MeshSphere);
  pointSeries->setBaseColor(QColor(255, 165, 0)); // 참외 색상 (주황색)
  scatter3D->addSeries(pointSeries);
  
  // 경로 시리즈 설정
  pathSeries = new QScatter3DSeries;
  pathSeries->setItemSize(0.05f);
  pathSeries->setMesh(QAbstract3DSeries::MeshPoint);
  pathSeries->setBaseColor(QColor(Qt::red)); // 경로는 빨간색으로 표시
  scatter3D->addSeries(pathSeries);
  
  // 경로 순서 표시 시리즈 설정
  orderSeries = new QScatter3DSeries;
  orderSeries->setItemSize(0.2f);
  orderSeries->setMesh(QAbstract3DSeries::MeshCube); // 큐브 모양으로 표시
  scatter3D->addSeries(orderSeries);
  
  // 레이아웃 추가
  mainLayout->addLayout(leftLayout, 1);
  mainLayout->addWidget(visualGroupBox, 2);
}

void MainWindow::resetAll()
{
  // 모든 데이터 초기화
  coordTable->setRowCount(0);
  points.clear();
  optimalPath.clear();
  minCost = 0;
  resultLabel->setText("아직 계산되지 않음");
  
  // 기본 점 추가
  addDefaultPoints();
  
  // 시각화 업데이트 (테이블에서 읽지 않고 직접 points 배열 사용)
  updateVisualization();
}

void MainWindow::setupConnections()
{
  connect(addPointButton, &QPushButton::clicked, this, &MainWindow::addPoint);
  connect(removePointButton, &QPushButton::clicked, this, &MainWindow::removePoint);
  connect(randomizeButton, &QPushButton::clicked, this, &MainWindow::randomizePoints); // 랜덤 버튼 연결
  connect(calculateButton, &QPushButton::clicked, this, &MainWindow::calculateOptimalPath);
  connect(resetButton, &QPushButton::clicked, this, &MainWindow::resetAll);
  
  // 테이블 데이터 변경 감지
  connect(coordTable, &QTableWidget::cellChanged, this, &MainWindow::onTableDataChanged);
}

// 테이블 데이터 변경 감지 함수 추가
void MainWindow::onTableDataChanged(int row, int column)
{
  // 초기화 중에는 무시
  if (isInitializing)
    return;
    
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
    
    // points 배열 업데이트
    if (column == 1)
      points[row].x = value;
    else if (column == 2)
      points[row].y = value;
    else if (column == 3)
      points[row].z = value;
      
    // 시각화 업데이트
    updateVisualization();
  }
  else if (column == 0 && row < points.size()) {
    // 이름 변경
    points[row].name = coordTable->item(row, 0)->text();
  }
}

void MainWindow::addPoint()
{
  int row = coordTable->rowCount();
  coordTable->insertRow(row);
  
  QString name = QString("참외%1").arg(row + 1);
  coordTable->setItem(row, 0, new QTableWidgetItem(name));
  coordTable->setItem(row, 1, new QTableWidgetItem("0.0"));
  coordTable->setItem(row, 2, new QTableWidgetItem("0.0"));
  coordTable->setItem(row, 3, new QTableWidgetItem("0.0"));
  
  Point3D newPoint(0, 0, 0, name);
  points.append(newPoint);
  
  updateVisualization();
}

void MainWindow::removePoint()
{
  QList<QTableWidgetItem*> selectedItems = coordTable->selectedItems();
  if (selectedItems.isEmpty()) {
    QMessageBox::warning(this, "경고", "제거할 행을 선택하세요.");
    return;
  }
  
  int row = coordTable->row(selectedItems.first());
  coordTable->removeRow(row);
  
  // points 배열에서도 제거
  if (row < points.size()) {
    points.remove(row);
  }
  
  // 경로가 있으면 초기화
  optimalPath.clear();
  
  updateVisualization();
}

// 랜덤 좌표 생성 함수 추가
void MainWindow::randomizePoints()
{
  isInitializing = true; // 초기화 플래그 설정
  
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dist(0.0f, 10.0f);
  
  // 기존의 모든 행에 대해 랜덤 좌표 설정
  for (int row = 0; row < coordTable->rowCount(); ++row) {
    float x = dist(gen);
    float y = dist(gen);
    float z = dist(gen);
    
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
  
  isInitializing = false; // 초기화 플래그 해제
  
  // 경로가 있으면 초기화
  optimalPath.clear();
  
  // 시각화 업데이트
  updateVisualization();
}

void MainWindow::readPointsFromTable()
{
  isInitializing = true; // 초기화 플래그 설정
  
  // 원래 크기 기억
  int origSize = points.size();
  
  // 테이블 크기에 맞게 포인트 배열 크기 조정
  points.resize(coordTable->rowCount());
  
  for (int row = 0; row < coordTable->rowCount(); ++row) {
    QString name = coordTable->item(row, 0)->text();
    float x = coordTable->item(row, 1)->text().toFloat();
    float y = coordTable->item(row, 2)->text().toFloat();
    float z = coordTable->item(row, 3)->text().toFloat();
    
    // 테이블 값으로 업데이트
    points[row] = Point3D(x, y, z, name);
  }
  
  isInitializing = false; // 초기화 플래그 해제
}

void MainWindow::updateVisualization()
{
  // 점 데이터 업데이트
  QScatterDataArray* dataArray = new QScatterDataArray;
  dataArray->resize(points.size());
  
  for (int i = 0; i < points.size(); ++i) {
    (*dataArray)[i].setPosition(QVector3D(points[i].x, points[i].y, points[i].z));
  }
  
  pointSeries->dataProxy()->resetArray(dataArray);
  
  // 경로가 있을 경우 경로 데이터 업데이트
  if (!optimalPath.isEmpty()) {
    updatePathVisualization();
  }
  else {
    // 경로가 없으면 경로 시리즈 초기화
    pathSeries->dataProxy()->resetArray(new QScatterDataArray);
    orderSeries->dataProxy()->resetArray(new QScatterDataArray);
  }
}

void MainWindow::updatePathVisualization()
{
  if (optimalPath.isEmpty() || points.isEmpty()) {
    pathSeries->dataProxy()->resetArray(new QScatterDataArray);
    orderSeries->dataProxy()->resetArray(new QScatterDataArray);
    return;
  }
  
  // 먼저 모든 경로 인덱스가 유효한지 확인
  bool validPath = true;
  for (int idx : optimalPath) {
    if (idx < 0 || idx >= points.size()) {
      validPath = false;
      break;
    }
  }
  
  if (!validPath) {
    // 경로가 유효하지 않으면 경로 초기화
    optimalPath.clear();
    pathSeries->dataProxy()->resetArray(new QScatterDataArray);
    orderSeries->dataProxy()->resetArray(new QScatterDataArray);
    return;
  }
  
  // 1. 경로 선 그리기
  QScatterDataArray* pathArray = new QScatterDataArray;
  int pathLength = optimalPath.size();
  int pointsPerSegment = 50; // 더 많은 점으로 선을 부드럽게
  int totalPoints = (pathLength - 1) * pointsPerSegment;
  pathArray->resize(totalPoints);
  
  int arrayIndex = 0;
  
  // 각 세그먼트를 여러 점으로 보간하여 선으로 표현
  for (int i = 0; i < pathLength - 1; ++i) {
    int pointIndex1 = optimalPath[i];
    int pointIndex2 = optimalPath[i + 1];
    
    for (int j = 0; j < pointsPerSegment; ++j) {
      float t = static_cast<float>(j) / pointsPerSegment;
      
      float x = points[pointIndex1].x * (1 - t) + points[pointIndex2].x * t;
      float y = points[pointIndex1].y * (1 - t) + points[pointIndex2].y * t;
      float z = points[pointIndex1].z * (1 - t) + points[pointIndex2].z * t;
      
      (*pathArray)[arrayIndex++].setPosition(QVector3D(x, y, z));
    }
  }
  
  // 경로 시리즈 업데이트
  pathSeries->dataProxy()->resetArray(pathArray);
  pathSeries->setItemSize(0.05f); // 작은 점으로 선처럼 보이게
  pathSeries->setBaseColor(QColor(Qt::red)); // 경로는 빨간색
  
  // 2. 경로 시작점 표시
  QScatterDataArray* startArray = new QScatterDataArray;
  startArray->resize(1);
  
  int startPointIndex = optimalPath[0];
  (*startArray)[0].setPosition(QVector3D(
    points[startPointIndex].x,
    points[startPointIndex].y + 0.3f,
    points[startPointIndex].z
  ));
  
  orderSeries->dataProxy()->resetArray(startArray);
  orderSeries->setItemSize(0.25f);
  orderSeries->setBaseColor(QColor(0, 0, 255)); // 시작점은 파란색
  orderSeries->setMesh(QAbstract3DSeries::MeshCube);
  
}

void MainWindow::calculateOptimalPath()
{
  // 현재 points 배열 사용 (테이블에서 다시 읽지 않음)
  
  int n = points.size();
  if (n <= 1) {
    QMessageBox::warning(this, "경고", "최소 2개 이상의 점이 필요합니다.");
    return;
  }
  
  // 결과 초기화
  optimalPath.clear();
  minCost = 0;
  
  // 8개 이하의 점에 대해서는 무차별 대입(Brute Force) 방식 사용
  if (n <= 8) {
    std::vector<int> indices;
    for (int i = 1; i < n; ++i) {
      indices.push_back(i);
    }
    
    std::vector<int> bestPath;
    double bestCost = std::numeric_limits<double>::max();
    
    // 거리 행렬 계산
    std::vector<std::vector<double>> dist(n, std::vector<double>(n));
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        if (i == j) {
          dist[i][j] = 0;
        } else {
          double dx = points[i].x - points[j].x;
          double dy = points[i].y - points[j].y;
          double dz = points[i].z - points[j].z;
          dist[i][j] = std::sqrt(dx*dx + dy*dy + dz*dz);
        }
      }
    }
    
    // 모든 순열 시도
    do {
      double currentCost = dist[0][indices[0]]; // 시작점에서 첫 번째 도시까지
      
      for (int i = 0; i < indices.size() - 1; ++i) {
        currentCost += dist[indices[i]][indices[i+1]];
      }
      
      currentCost += dist[indices.back()][0]; // 마지막 도시에서 시작점으로
      
      if (currentCost < bestCost) {
        bestCost = currentCost;
        bestPath = indices;
      }
      
    } while (std::next_permutation(indices.begin(), indices.end()));
    
    // 최적 경로 구성
    optimalPath.clear();
    optimalPath.push_back(0); // 시작점
    
    for (int idx : bestPath) {
      optimalPath.push_back(idx);
    }
    
    optimalPath.push_back(0); // 시작점으로 돌아오기
    minCost = bestCost;
    
    // 결과 표시
    QString resultText = QString("최적 경로 비용: %1\n경로: ").arg(minCost);
    for (int i = 0; i < optimalPath.size(); ++i) {
      int idx = optimalPath[i];
      resultText += points[idx].name;
      if (i < optimalPath.size() - 1) {
        resultText += " → ";
      }
    }
    
    resultLabel->setText(resultText);
    updatePathVisualization();
    return;
  }
  
  // 최대 12개 점까지 계산 허용 (2^12 상태를 고려)
  if (n > 12) {
    QMessageBox::warning(this, "경고", "계산 효율성을 위해 최대 12개 점까지 지원합니다.");
    return;
  }
  
  // 거리 행렬 계산
  std::vector<std::vector<double>> dist(n, std::vector<double>(n));
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      if (i == j) {
        dist[i][j] = 0;
      } else {
        // 3D 유클리드 거리 계산
        double dx = points[i].x - points[j].x;
        double dy = points[i].y - points[j].y;
        double dz = points[i].z - points[j].z;
        dist[i][j] = std::sqrt(dx*dx + dy*dy + dz*dz);
      }
    }
  }
  
  // Held-Karp 알고리즘 (비트마스크를 사용한 동적 계획법)
  // dp[mask][j] = mask에 있는 모든 도시를 방문하고 j에 있는 경로의 최소 비용
  std::unordered_map<std::pair<int, int>, double, StateHash> dp;
  std::unordered_map<std::pair<int, int>, int, StateHash> parent;
  
  // 초기 상태: 0번 점만 방문
  for (int j = 0; j < n; ++j) {
    if (j != 0) { // 시작점 0 에서 다른 점으로
      dp[{1 | (1 << j), j}] = dist[0][j];
      parent[{1 | (1 << j), j}] = 0;
    }
  }
  
  // 모든 부분집합에 대해 계산
  int allVisited = (1 << n) - 1;
  for (int mask = 3; mask <= allVisited; ++mask) {
    // 0번은 항상 시작점이므로 포함되어야 함
    if (!(mask & 1)) continue;
    
    for (int j = 1; j < n; ++j) {
      // j가 mask에 포함되어 있는지 확인
      if (!(mask & (1 << j))) continue;
      
      // j 이전의 마지막 도시를 결정
      int prevMask = mask & ~(1 << j);
      double minCost = std::numeric_limits<double>::max();
      int minPrev = -1;
      
      for (int k = 0; k < n; ++k) {
        if (!(prevMask & (1 << k))) continue;
        if (k == j) continue;
        
        double cost = dp[{prevMask, k}] + dist[k][j];
        if (cost < minCost) {
          minCost = cost;
          minPrev = k;
        }
      }
      
      dp[{mask, j}] = minCost;
      parent[{mask, j}] = minPrev;
    }
  }
  
  // 최종 경로 비용 계산 (모두 방문 후 0으로 돌아오기)
  minCost = std::numeric_limits<double>::max();
  int lastCity = -1;
  
  for (int j = 1; j < n; ++j) {
    double cost = dp[{allVisited, j}] + dist[j][0];
    if (cost < minCost) {
      minCost = cost;
      lastCity = j;
    }
  }
  
  // 경로 재구성
  optimalPath.clear();
  optimalPath.push_back(0); // 시작점
  
  int mask = allVisited;
  int currentCity = lastCity;
  
  optimalPath.push_back(currentCity); // 마지막 도시 추가
  
  // 역추적으로 경로 복원
  while (currentCity != 0) {
    int nextMask = mask & ~(1 << currentCity);
    int nextCity = parent[{mask, currentCity}];
    
    if (nextCity != 0) // 시작점은 이미 추가됨
      optimalPath.push_back(nextCity);
    
    mask = nextMask;
    currentCity = nextCity;
  }
  
  std::reverse(optimalPath.begin(), optimalPath.end()); // 경로를 시작점부터 순서대로 정렬
  optimalPath.push_back(0); // 다시 시작점으로 돌아오기
  
  // 결과 표시
  QString resultText = QString("최적 경로 비용: %1\n경로: ").arg(minCost);
  for (int i = 0; i < optimalPath.size(); ++i) {
    int idx = optimalPath[i];
    resultText += points[idx].name;
    if (i < optimalPath.size() - 1) {
      resultText += " → ";
    }
  }
  
  resultLabel->setText(resultText);
  
  // 시각화 업데이트
  updatePathVisualization();
}

void MainWindow::addDefaultPoints()
{
  isInitializing = true; // 초기화 플래그 설정
  
  // 테이블 초기화
  coordTable->setRowCount(0);
  points.clear();
  
  // 랜덤 좌표 생성을 위한 설정
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dist(0.0f, 10.0f);
  
  // 8개의 참외 생성 (랜덤 좌표)
  const int numPoints = 8;
  
  for (int i = 0; i < numPoints; ++i) {
    int row = coordTable->rowCount();
    coordTable->insertRow(row);
    
    // 랜덤 좌표 생성
    float x = dist(gen);
    float y = dist(gen);
    float z = dist(gen);
    
    QString name = QString("참외%1").arg(i + 1);
    coordTable->setItem(row, 0, new QTableWidgetItem(name));
    coordTable->setItem(row, 1, new QTableWidgetItem(QString::number(x, 'f', 2)));
    coordTable->setItem(row, 2, new QTableWidgetItem(QString::number(y, 'f', 2)));
    coordTable->setItem(row, 3, new QTableWidgetItem(QString::number(z, 'f', 2)));
    
    points.append(Point3D(x, y, z, name));
  }
  
  isInitializing = false; // 초기화 플래그 해제
}