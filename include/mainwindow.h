#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVector>
#include <QTableWidget>
#include <QPushButton>
#include <QLabel>
#include <QtDataVisualization/Q3DScatter>
#include <QtDataVisualization/QScatter3DSeries>
#include <unordered_map>

namespace Ui {
class MainWindow;
}

// 3차원 점 클래스
struct Point3D {
    float x, y, z;
    QString name;
    
    Point3D(float _x = 0, float _y = 0, float _z = 0, const QString& _name = "")
        : x(_x), y(_y), z(_z), name(_name) {}
};

// 상태 해시 함수 - Held-Karp 알고리즘에서 사용
struct StateHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ std::hash<int>()(p.second);
    }
};

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    
private slots:
    void addPoint();
    void removePoint();
    void randomizePoints(); // 랜덤 좌표 생성 함수 추가
    void calculateOptimalPath();
    void resetAll();
    void onTableDataChanged(int row, int column); // 테이블 데이터 변경 이벤트 처리
    
private:
    Ui::MainWindow *ui;
    QTableWidget* coordTable;
    QPushButton* addPointButton;
    QPushButton* removePointButton;
    QPushButton* randomizeButton; // 랜덤 버튼 추가
    QPushButton* calculateButton;
    QPushButton* resetButton;
    QLabel* resultLabel;
    
    QtDataVisualization::Q3DScatter* scatter3D;
    QtDataVisualization::QScatter3DSeries* pointSeries;
    QtDataVisualization::QScatter3DSeries* pathSeries;
    QtDataVisualization::QScatter3DSeries* orderSeries;
    
    QVector<Point3D> points;
    QVector<int> optimalPath;
    double minCost;
    
    bool isInitializing = false; // 초기화 플래그 추가
    
    void setupUI();
    void setupConnections();
    void readPointsFromTable();
    void updateVisualization();
    void updatePathVisualization();
    void addDefaultPoints();
};

#endif // MAINWINDOW_H