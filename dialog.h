#ifndef DIALOG_H
#define DIALOG_H

#include "xmap.h"
#include "planner.h"

#include <QDialog>

namespace Ui {
class Dialog;
}

class BaseModel
{
public:
    //当前机器人的位置
    Map::Cell* current;
    //地图数据
    unsigned char* data;
    //目标位置
    Map::Cell* goal;
    //机器人的半径（单位：像素）
    int robot_radius;

    BaseModel(){};
    ~BaseModel()
    {
        delete data;
    };
};

class RealModel: public BaseModel
{
public:
    //走过的路径
    list<Map::Cell*> path_traversed;

    RealModel(){};
};

class RobotModel : public BaseModel
{
public:
    //机器人的探测半径（单位：像素）
    int scan_radius;
    //规划生成的路径
    list<Map::Cell*> path_planned;

    RobotModel(){};
};


class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();

    /**
     * Draws the window.
     *
     * @return  int
     */
    int redraw();

    /**
     * Main execution method.
     *
     * @return  int  successfull
     */
    int execute();

    /*
     * Scans map for updated cells.
     *
     * @return  bool  updates found
     */
    bool update_map();
private slots:
    void on_pushButton_clicked();
    void on_timer();

private:
    Ui::Dialog *ui;

    QPoint start;
    QPoint goal;
    bool _init;

    Map* _map;

    Planner* _planner;

    QTimer* timer;
    QImage real_bitmap;
    QImage robot_bitmap;
    RealModel* _real_widget;
    RobotModel* _robot_widget;
    QRect leftRect;
    QRect rightRect;

    void paintEvent(QPaintEvent *);
    void mousePressEvent(QMouseEvent *event);

    QImage Pk8bitGrayToQIm(const unsigned char *pBuffer, const int &bufWidth, const int &bufHight);
};

#endif // DIALOG_H
