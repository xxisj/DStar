﻿#include "dialog.h"
#include "ui_dialog.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QTimer>
#include <QPaintEngine>
#include <QMouseEvent>
#include <QBitmap>

/**
 * @var  double  cost difference between bitmap and tile;
 */
const double COST_DIFFERENCE = 255.0;

/**
 * @var  double  unwalkable value of bitmap
 */
const double UNWALKABLE_CELL = 0.0;

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);

    leftRect=QRect(10,10,200,300);
    rightRect=QRect(220,10,200,300);
    resize(leftRect.width()+rightRect.width()+30,leftRect.height()+60);
    ui->pushButton->move(leftRect.width()-30,leftRect.height()+20);

    _init = false;
}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::on_pushButton_clicked()
{
    if (_init) return;

    // Widths and heights are the same, set a constant
    int img_width = real_bitmap.width();
    int img_height = real_bitmap.height();
    int img_depth = real_bitmap.depth()/8;

    // Make sure the real image and the robot's image are the same dimensions
    if (img_width != robot_bitmap.width() || img_height != robot_bitmap.height() || img_depth != robot_bitmap.depth()/8)
    {
        QMessageBox::warning(NULL,"Error!","Size not match!",QMessageBox::Ok);
        return;
    }

    startPoint=QPoint(0,0);
    goalPoint=QPoint(249,249);

    realWorldData = new unsigned char[img_width * img_height];
    robotPerceiveData = new unsigned char[img_width * img_height];

    for (int i = 0; i < img_height; i++)
    {
        for (int j = 0; j < img_width; j++)
        {
            // Index key
            int k1 = (i * img_width) + j;
            // Depth key
            int k2 = (i * img_width * img_depth) + (j * img_depth);

            if (img_depth == 1)
            {
                realWorldData[k1] = real_bitmap.bits()[k2];
                robotPerceiveData[k1] = robot_bitmap.bits()[k2];
            }
            else if ((img_depth == 3)||(img_depth == 4))
            {
                // Convert to grayscale
                realWorldData[k1] = (unsigned char) (0.3 * real_bitmap.bits()[k2] + 0.59 * real_bitmap.bits()[k2 + 1] + 0.11 * real_bitmap.bits()[k2 + 2] + 0.5);
                robotPerceiveData[k1] = (unsigned char) (0.3 * robot_bitmap.bits()[k2] + 0.59 * robot_bitmap.bits()[k2 + 1] + 0.11 * robot_bitmap.bits()[k2 + 2] + 0.5);
            }
            else
            // Unsupported depth
            {
                return;
            }
        }
    }

    // Set the robot radius
    robot_radius = 2;

    // Sert the scan radius
    scan_radius = 30;

    // Make the map
    _map = new Map(img_height, img_width);

    // Set current and goal position
    current = (*_map)(startPoint.x(), startPoint.y());
    goal = (*_map)(goalPoint.x(), goalPoint.y());

    // Build map
    for (int i = 0; i < img_height; i++)
    {
        for (int j = 0; j < img_width; j++)
        {
            int k = (i * img_width) + j;
            double v = (double) robotPerceiveData[k];

            // Cell is unwalkable
            if (v == UNWALKABLE_CELL)
            {
                v = Map::Cell::COST_UNWALKABLE;
            }
            else
            {
                v = COST_DIFFERENCE - v + 1.0;
            }

            (*_map)(i, j)->cost = v;
        }
    }

    // Make planner
    _planner = new Planner(_map, current, goal);

    // Push start position
    path_traversed.push_back(_planner->start());

    if ( ! _planner->replan())
    {
        QMessageBox::warning(NULL,"Error!","No Solution Found!");
        return;
    }
    path_planned = _planner->path();

    timer=new QTimer();
    connect(timer,SIGNAL(timeout()),this,SLOT(on_timer()));
    timer->start(50);

    _init=true;
}

void Dialog::on_timer()
{
    if (execute() == 0)
          update();
    else
    {
        timer->stop();
    }
}

/*
 * Scans map for updated tiles.
 *
 * @return  bool  updates found
 */
bool Dialog::update_map()
{
    //机器人感知的地图与真实世界的差别标志
    bool error = false;

    unsigned int x= current->x();
    unsigned int y= current->y();

    // Radius^2
    unsigned int radius = scan_radius;
    unsigned int radius2 = radius * radius;

    unsigned int rows, cols;
    rows = _map->rows();
    cols = _map->cols();

    // Make an imaginary box around the scan circle
    unsigned int max_x, max_y, min_x, min_y;
    max_x = (x + radius < cols) ? x + radius : cols;
    max_y = (y + radius < rows) ? y + radius : rows;
    min_x = (x > radius) ? x - radius : 0;
    min_y = (y > radius) ? y - radius : 0;

    for (unsigned int i = min_y; i < max_y; i++)
    {
        int dy = y - i;
        unsigned int dy2 = dy * dy;

        for (unsigned int j = min_x; j < max_x; j++)
        {
            int dx = x - j;

            if ((dx * dx) + dy2 < radius2)
            {
                unsigned int k = (i * cols) + j;

                // Check if an update is required
                if (robotPerceiveData[k] != realWorldData[k])
                {
                    error = true;

                    robotPerceiveData[k] = realWorldData[k];
                    double v = (double) robotPerceiveData[k];

                    if (v == UNWALKABLE_CELL)
                    {
                        v = Map::Cell::COST_UNWALKABLE;
                    }
                    else
                    {
                        v = COST_DIFFERENCE - v + 1.0;
                    }

                    _planner->update((*_map)(i, j), v);
                }
            }
        }
    }

    return error;
}


int Dialog::execute()
{
    if (_planner->start() == _planner->goal())
    {
        QMessageBox::information(NULL,"OK","Goal Reached!");
        current = _planner->goal();
        return 1;
    }

    // Check if map was updated
    if (update_map())
    {
        // Replan the path
        if ( ! _planner->replan())
        {
            QMessageBox::warning(NULL,"Failed!","No Solution Found!");
            return 0;
        }

        path_planned = _planner->path();

        if ( ! path_planned.empty())
        {
           path_planned.pop_front();
        }
    }

    // Step
    path_traversed.push_back(path_planned.front());
    _planner->start(path_planned.front());
    current =  _planner->start();
    path_planned.pop_front();

    return 0;
}

void Dialog::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    QPen pen;
    pen.setColor(QColor(0,0,0));
    painter.setPen(pen);
    painter.setBrush(QBrush(QColor(0,0,0,0)));
    painter.drawRect(leftRect);
    painter.drawRect(rightRect);

    if (real_bitmap.bits()==NULL)
        painter.drawText(leftRect.center().x()-70,leftRect.center().y(),"Click to open real map.");
    else
        if (_init)
            painter.drawImage(leftRect,Pk8bitGrayToQIm(realWorldData,real_bitmap.width(),real_bitmap.height()));
        else
            painter.drawImage(leftRect,real_bitmap);

    if (robot_bitmap.bits()==NULL)
        painter.drawText(rightRect.center().x()-70,rightRect.center().y(),"Click to open robot map.");
    else
        if (_init)
            painter.drawImage(rightRect,Pk8bitGrayToQIm(robotPerceiveData,robot_bitmap.width(),robot_bitmap.height()));
        else
            painter.drawImage(rightRect,robot_bitmap);

    if (!_init)
        return;

    //Real
        // Draw traversed path
        pen.setColor(QColor(0,255,0));
        painter.setPen(pen);
        for (list<Map::Cell*>::iterator i = path_traversed.begin(); i != path_traversed.end(); i++)
            painter.drawPoint(leftRect.left() + (*i)->x(), leftRect.top() + (*i)->y());
        // Draw current position
        pen.setColor(QColor(255,128,128));
        painter.setPen(pen);
        painter.drawEllipse(QPoint(leftRect.left() + current->x(), leftRect.top() + current->y()),
                           robot_radius, robot_radius);

    //Robot
        // Draw planned path
        pen.setColor(QColor(0,0,255));
        painter.setPen(pen);
        for (list<Map::Cell*>::iterator i = path_planned.begin(); i != path_planned.end(); i++)
            painter.drawPoint(rightRect.left() + (*i)->x(), rightRect.top() + (*i)->y());
        // Draw scanner radius
        pen.setColor(QColor(255,0,0));
        painter.setPen(pen);
        painter.drawEllipse(QPoint(rightRect.left() + current->x(), rightRect.top() + current->y()),
                            scan_radius,scan_radius);
        // Draw current position
        pen.setColor(QColor(255,128,128));
        painter.setPen(pen);
        painter.drawEllipse(QPoint(rightRect.left() + current->x(), rightRect.top() + current->y()),
                            robot_radius, robot_radius);
}

void Dialog::mousePressEvent(QMouseEvent* event)
{
    if (leftRect.contains(event->localPos().x(),event->localPos().y()))
        if (real_bitmap.bits()==NULL)
        {
            QString fileName = QFileDialog::getOpenFileName(this, "Open Map File", " ",  "Map File(*.bmp);;All File(*.*)");
            if (fileName.isEmpty()) return;

            if (real_bitmap.load(fileName))
            {
                leftRect.setWidth(real_bitmap.width());
                leftRect.setHeight(real_bitmap.height());
                rightRect.setRight(leftRect.width()+20+rightRect.width());
                rightRect.setLeft(leftRect.width()+20);

                resize(leftRect.width()+rightRect.width()+30,leftRect.height()+60);
                ui->pushButton->move(leftRect.width()-30,leftRect.height()+20);

                update();
            }
        }

    if (rightRect.contains(event->localPos().x(),event->localPos().y()))
        if (robot_bitmap.bits()==NULL)
        {
            QString fileName = QFileDialog::getOpenFileName(this, "Open Map File", " ",  "Map File(*.bmp);;All File(*.*)");
            if (fileName.isEmpty()) return;

            if (robot_bitmap.load(fileName))
            {
                rightRect.setWidth(robot_bitmap.width());
                rightRect.setHeight(robot_bitmap.height());

                resize(leftRect.width()+rightRect.width()+30,leftRect.height()+60);
                ui->pushButton->move(leftRect.width()-30,leftRect.height()+20);

                update();
            }
        }
}

void Dialog::mouseMoveEvent(QMouseEvent* event)
{
    if (event->buttons() & Qt::LeftButton)
    {
        if (leftRect.contains(event->localPos().x(),event->localPos().y()))
        {
            if (real_bitmap.bits()==NULL)
                return;

            real_bitmap.setPixel(event->localPos().x(),event->localPos().y(),qRgb(0,0,0));

            if (_init)
            {
                int k = event->localPos().x()+event->localPos().y()*real_bitmap.width();

                realWorldData[k]=0;
                (*_map)(event->localPos().x(),event->localPos().y())->cost = Map::Cell::COST_UNWALKABLE;
            }

            update();
        }
    }

    QDialog::mouseMoveEvent(event);
}

QImage Dialog::Pk8bitGrayToQIm(const unsigned char *pBuffer, const int &bufWidth, const int &bufHight)
{
    //对参数的有效性进行检查
    if ((pBuffer == NULL) || (bufWidth<=0) || (bufHight<=0)) return QImage();

    QVector<QRgb> vcolorTable;
    for (int i = 0; i < 256; i++)
    {
        vcolorTable.append(qRgb(i, i, i));
    }

    int biBitCount = 8; //灰度图像像素bit数
    int lineByte = (bufWidth * biBitCount/8 + 3) / 4 * 4; //bmp行byte数（格式宽度，为4的倍数）

    if (bufWidth == lineByte) //判断图像宽度与格式宽度
    {
        QImage qIm = QImage(pBuffer, bufWidth, bufHight, QImage::Format_Indexed8);  //封装QImage
        qIm.setColorTable(vcolorTable); //设置颜色表

        return qIm;
    }
    else
    {
        unsigned char *qImageBuffer = new unsigned char[lineByte * bufHight]; //分配内存空间
        uchar *QImagePtr = qImageBuffer;

        for (int i = 0; i < bufHight; i++) //Copy line by line
        {
            memcpy(QImagePtr, pBuffer, bufWidth);
            QImagePtr += lineByte;
            pBuffer += bufWidth;
        }

        QImage qImage = QImage(qImageBuffer, bufWidth, bufHight, QImage::Format_Indexed8);  //封装QImage
        qImage.setColorTable(vcolorTable); //设置颜色表

        return qImage;
    }
}
