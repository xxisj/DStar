#ifndef DIALOG_H
#define DIALOG_H

#include "xmap.h"
#include "planner.h"

#include <QDialog>

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();

    //当前机器人的位置
    Map::Cell* current;
    //目标位置
    Map::Cell* goal;
    //真实地图数据
    unsigned char* realWorldData;
    //机器人感知到的地图数据
    unsigned char* robotPerceiveData;
     //机器人的半径（单位：像素）
    int robot_radius;
    //机器人的探测半径（单位：像素）
    int scan_radius;
    //走过的路径
    list<Map::Cell*> path_traversed;
    //规划生成的路径
    list<Map::Cell*> path_planned;

    //机器人执行一步动作
    int execute();
    //检查地图是否有更新
    bool update_map();

private slots:
    void on_pushButton_clicked();
    void on_timer();

private:
    Ui::Dialog *ui;

    bool _init;
    QPoint startPoint;
    QPoint goalPoint;
    Map* _map;
    Planner* _planner;

    QTimer* timer;
    QImage real_bitmap;
    QImage robot_bitmap;
    QRect leftRect;
    QRect rightRect;

    void paintEvent(QPaintEvent *);
    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    QImage Pk8bitGrayToQIm(const unsigned char *pBuffer, const int &bufWidth, const int &bufHight);
};

#endif // DIALOG_H
